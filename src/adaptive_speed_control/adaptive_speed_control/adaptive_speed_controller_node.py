import math
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

from adaptive_speed_interfaces.msg import ObstacleInfo
from adaptive_speed_interfaces.srv import SetThreshold
from adaptive_speed_control.clear_barrier import move_barrier
from adaptive_speed_control.utils.speed_policy import (
    calculate_adaptive_speed,
    process_laser_scan_sectors,
    rate_limit_velocity,
)


class AdaptiveSpeedControllerNode(Node):
    """
    ROS 2 Adaptive Speed Controller node for mobile robots in cluttered environments.
    - Subscribes to /scan, /cmd_vel_in, and /odom.
    - Continuously parses multi-sector LiDAR geometry and surrounding obstacle density.
    - Regulates commanded translational velocity using distance and clutter attenuation factors.
    - Sole publisher of final motion commands to /cmd_vel.
    - Enforces rate-limited acceleration and safe timeouts.
    """

    def __init__(self):
        super().__init__('adaptive_speed_controller_node')

        # Parameters
        self.declare_parameter('threshold', 0.45)
        self.declare_parameter('slowdown_distance', 2.00)
        self.declare_parameter('density_radius', 1.30)
        self.declare_parameter('density_influence', 0.60)
        self.declare_parameter('c_min', 0.25)
        self.declare_parameter('max_linear_speed', 0.40)
        self.declare_parameter('max_angular_speed', 1.00)
        self.declare_parameter('max_acceleration', 0.60)
        self.declare_parameter('front_angle_deg', 60.0)
        self.declare_parameter('sensor_timeout', 0.50)
        self.declare_parameter('cmd_timeout', 0.50)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('adaptive_enabled', True)

        self._load_parameters()
        self.add_on_set_parameters_callback(self._on_parameters_set)

        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.cmd_in_sub = self.create_subscription(Twist, '/cmd_vel_in', self.on_cmd_vel_in, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.info_pub = self.create_publisher(ObstacleInfo, '/obstacle_info', 10)
        self.density_pub = self.create_publisher(Float32, '/obstacle_density', 10)

        # Services
        self.th_srv = self.create_service(SetThreshold, '/set_threshold', self.handle_set_threshold)
        self.clear_barrier_srv = self.create_service(Trigger, '/clear_barrier', self.handle_clear_barrier)

        # State tracking
        self.requested_cmd = Twist()
        self.last_cmd_time = 0.0
        self.last_scan_time = 0.0

        self.robot_x = -9.0
        self.robot_y = 0.0

        self.closest_distance = float('inf')
        self.closest_direction = 'unknown'
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.obstacle_density = 0.0
        self.safety_state = 'STARTING'

        self.last_cmd_linear = 0.0
        self.last_log_time = 0.0
        self.last_logged_state = None

        timer_period = 1.0 / max(1.0, self.control_rate)
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(
            f"Adaptive Speed Controller initialized:\n"
            f"  Safety Distance (d_min)   : {self.threshold:.2f} m\n"
            f"  Slowdown Dist (d_slow)    : {self.slowdown_distance:.2f} m\n"
            f"  Density Radius (R_dens)   : {self.density_radius:.2f} m\n"
            f"  Density Weight (k)        : {self.density_influence:.2f}\n"
            f"  Max Velocity Limit        : {self.max_linear_speed:.2f} m/s\n"
            f"  Adaptive Mode             : {'ENABLED' if self.adaptive_enabled else 'FIXED BASELINE'}\n"
            f"  Subscribed to             : /scan, /cmd_vel_in, /odom\n"
            f"  Sole Controller Publisher : /cmd_vel"
        )

    def _load_parameters(self):
        self.threshold = float(self.get_parameter('threshold').value)
        self.slowdown_distance = float(self.get_parameter('slowdown_distance').value)
        self.density_radius = float(self.get_parameter('density_radius').value)
        self.density_influence = float(self.get_parameter('density_influence').value)
        self.c_min = float(self.get_parameter('c_min').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.max_acceleration = float(self.get_parameter('max_acceleration').value)
        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)
        self.sensor_timeout = float(self.get_parameter('sensor_timeout').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.adaptive_enabled = bool(self.get_parameter('adaptive_enabled').value)

    def _on_parameters_set(self, params):
        for param in params:
            if param.name == 'threshold':
                if param.value <= 0.0 or not math.isfinite(param.value) or param.value >= self.slowdown_distance:
                    return SetParametersResult(successful=False, reason="threshold must be > 0 and < slowdown_distance")
                self.threshold = float(param.value)
            elif param.name == 'slowdown_distance':
                if param.value <= self.threshold or not math.isfinite(param.value):
                    return SetParametersResult(successful=False, reason="slowdown_distance must be > threshold")
                self.slowdown_distance = float(param.value)
            elif param.name == 'density_influence':
                if param.value < 0.0 or param.value > 1.0:
                    return SetParametersResult(successful=False, reason="density_influence must be between 0.0 and 1.0")
                self.density_influence = float(param.value)
            elif param.name == 'adaptive_enabled':
                self.adaptive_enabled = bool(param.value)

        return SetParametersResult(successful=True)

    def handle_set_threshold(self, request, response):
        new_th = request.threshold
        if new_th <= 0.0 or not math.isfinite(new_th) or new_th >= self.slowdown_distance:
            response.success = False
            response.message = f"Invalid threshold ({new_th:.2f}m). Must be >0 and < slowdown_distance."
            return response

        self.threshold = float(new_th)
        self.set_parameters([Parameter('threshold', Parameter.Type.DOUBLE, self.threshold)])
        response.success = True
        response.message = f"Safety threshold updated successfully to {self.threshold:.3f} m."
        self.get_logger().info(response.message)
        return response

    def handle_clear_barrier(self, request, response):
        success = move_barrier(10.0)
        response.success = success
        if success:
            response.message = "Zone 4 barrier cleared from path. Robot will resume automatically."
            self.get_logger().info(response.message)
            if self.safety_state in ("CRITICAL_PROXIMITY", "CONTROLLED_STOP"):
                self.safety_state = "PATH_CLEAR"
        else:
            response.message = "Failed to contact Gazebo service to move barrier."
            self.get_logger().warn(response.message)
        return response

    def on_odom(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def on_cmd_vel_in(self, msg: Twist):
        self.requested_cmd = msg
        self.last_cmd_time = time.time()

    def on_scan(self, scan: LaserScan):
        if not scan.ranges:
            return

        self.last_scan_time = time.time()

        min_range = scan.range_min if (math.isfinite(scan.range_min) and scan.range_min > 0.0) else 0.05
        max_range = scan.range_max if (math.isfinite(scan.range_max) and scan.range_max > 0.0) else 30.0

        sectors = process_laser_scan_sectors(
            ranges=scan.ranges,
            angle_min=scan.angle_min,
            angle_increment=scan.angle_increment,
            range_min=min_range,
            range_max=max_range,
            front_angle_deg=self.front_angle_deg,
            density_radius=self.density_radius,
        )

        self.closest_distance = sectors["closest_distance"]
        self.closest_direction = sectors["direction"]
        self.front_distance = sectors["front_distance"]
        self.left_distance = sectors["left_distance"]
        self.right_distance = sectors["right_distance"]
        self.obstacle_density = sectors["density_rho"]

        # Telemetry publisher
        info = ObstacleInfo()
        info.closest_distance = float(self.closest_distance)
        info.direction = str(self.closest_direction)
        info.threshold = float(self.threshold)
        info.front_distance = float(self.front_distance)
        info.left_distance = float(self.left_distance)
        info.right_distance = float(self.right_distance)
        info.density_rho = float(self.obstacle_density)
        info.zone = str(self.safety_state)
        self.info_pub.publish(info)

        dens_msg = Float32()
        dens_msg.data = float(self.obstacle_density)
        self.density_pub.publish(dens_msg)

    def control_loop(self):
        now = time.time()

        # Check destination reached
        if self.robot_x >= 8.8:
            self.safety_state = "DESTINATION_REACHED"
            self.cmd_pub.publish(Twist())
            self._log_status(0.0, 0.0, 0.0)
            return

        # Fail-safe 1: Sensor timeout
        if (now - self.last_scan_time) > self.sensor_timeout:
            self.safety_state = "SENSOR_TIMEOUT"
            self.cmd_pub.publish(Twist())
            self._log_status(0.0, 0.0, 0.0)
            return

        # Fail-safe 2: Upstream command timeout
        if (now - self.last_cmd_time) > self.cmd_timeout:
            self.safety_state = "IDLE"
            self.cmd_pub.publish(Twist())
            self._log_status(0.0, 0.0, 0.0)
            return

        v_req = self.requested_cmd.linear.x
        omega_req = self.requested_cmd.angular.z

        v_cmd_target, omega_cmd, scale, next_state = calculate_adaptive_speed(
            v_req=v_req,
            omega_req=omega_req,
            front_dist=self.front_distance,
            density=self.obstacle_density,
            threshold=self.threshold,
            slowdown_dist=self.slowdown_distance,
            density_influence=self.density_influence,
            c_min=self.c_min,
            max_lin_speed=self.max_linear_speed,
            max_ang_speed=self.max_angular_speed,
            current_state=self.safety_state,
            adaptive_enabled=self.adaptive_enabled,
        )

        dt = 1.0 / max(1.0, self.control_rate)
        v_cmd = rate_limit_velocity(
            v_target=v_cmd_target,
            v_current=self.last_cmd_linear,
            max_accel=self.max_acceleration,
            dt=dt
        )

        self.last_cmd_linear = v_cmd
        self.safety_state = next_state

        cmd = Twist()
        cmd.linear.x = float(v_cmd)
        cmd.angular.z = float(omega_cmd)
        self.cmd_pub.publish(cmd)

        self._log_status(v_req, v_cmd, scale)

    def _log_status(self, v_req, v_cmd, scale):
        now = time.time()
        state_changed = (self.safety_state != self.last_logged_state)
        time_elapsed = (now - self.last_log_time) >= 1.0

        if not (state_changed or time_elapsed):
            return

        self.last_log_time = now
        self.last_logged_state = self.safety_state

        if self.safety_state == "DESTINATION_REACHED":
            self.get_logger().info(
                f"[SUCCESS] DESTINATION_REACHED | Pos: ({self.robot_x:.2f}, {self.robot_y:.2f}) | Robot Arrived Safely!"
            )
        elif self.safety_state in ("CRITICAL_PROXIMITY", "CONTROLLED_STOP"):
            self.get_logger().warn(
                f"[WARNING] CRITICAL_PROXIMITY | Distance: {self.front_distance:.2f}m <= {self.threshold:.2f}m | Robot stopped"
            )
        elif self.safety_state == "PATH_CLEAR":
            self.get_logger().info(
                f"[STATUS] PATH_CLEAR | Resuming navigation | Front: {self.front_distance:.2f}m"
            )
        elif self.safety_state == "SENSOR_TIMEOUT":
            self.get_logger().warn(
                "[STATUS] SENSOR_TIMEOUT | No recent LiDAR data on /scan! Robot halted for safety."
            )
        elif self.safety_state == "IDLE":
            self.get_logger().info("[STATUS] IDLE | Waiting for commands on /cmd_vel_in...")
        else:
            self.get_logger().info(
                f"[STATUS] {self.safety_state:<18} | Distance: {self.front_distance:.2f}m | "
                f"Density: {self.obstacle_density:.2f} | Speed: {v_cmd:.2f} m/s"
            )


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveSpeedControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
