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

from rt1_assignment2_interfaces.msg import ObstacleInfo
from rt1_assignment2_interfaces.srv import SetThreshold
from rt1_assignment2.clear_barrier import move_barrier


def calculate_adaptive_speed(
    v_req: float,
    omega_req: float,
    front_dist: float,
    density: float,
    threshold: float = 0.45,
    slowdown_dist: float = 2.00,
    density_influence: float = 0.60,
    c_min: float = 0.25,
    max_lin_speed: float = 0.40,
    max_ang_speed: float = 1.00,
    current_state: str = "STARTING",
):
    """
    Computes commanded linear and angular velocities based on LiDAR obstacle distance and density,
    with continuous hysteresis-based state machine classification and state speed limits.
    Returns: (v_cmd, omega_cmd, scale_factor, safety_state)
    """
    # Clamp angular velocity
    if max_ang_speed > 0.0:
        omega_clamped = max(-max_ang_speed, min(max_ang_speed, omega_req))
    else:
        omega_clamped = omega_req

    # Reverse motion handling
    if v_req < 0.0:
        v_clamped = max(v_req, -max_lin_speed)
        return v_clamped, omega_clamped, 1.0, "REVERSE"

    # Zero linear motion handling
    if abs(v_req) < 1e-6:
        state = "ROTATING" if abs(omega_clamped) > 1e-6 else "STOPPED"
        return 0.0, omega_clamped, 1.0, state

    # Forward motion limits
    v_clamped = min(v_req, max_lin_speed)

    # 1. Critical safety threshold evaluation
    if front_dist <= threshold:
        return 0.0, 0.0, 0.0, "CRITICAL_PROXIMITY"

    # Check recovery from controlled stop / critical proximity
    if current_state in ("CRITICAL_PROXIMITY", "CONTROLLED_STOP"):
        if front_dist > (threshold + 0.20):  # Hysteresis buffer (e.g. > 0.65m)
            v_actual = min(v_clamped, 0.25)
            scale = (v_actual / v_clamped) if v_clamped > 0 else 1.0
            return v_actual, omega_clamped, scale, "PATH_CLEAR"
        else:
            return 0.0, 0.0, 0.0, "CONTROLLED_STOP"
    elif current_state == "PATH_CLEAR":
        new_state = "RESUME_NAVIGATION"
    else:
        new_state = current_state

    # 2. Mathematical model: Distance Factor D
    if front_dist < slowdown_dist:
        d_factor = (front_dist - threshold) / (slowdown_dist - threshold)
        d_factor = max(0.0, min(1.0, d_factor))
    else:
        d_factor = 1.0

    # 3. Mathematical model: Density Factor C
    dens_clamped = max(0.0, min(1.0, density))
    c_factor = max(c_min, min(1.0, 1.0 - density_influence * dens_clamped))

    # 4. Raw adaptive speed
    v_raw = v_clamped * d_factor * c_factor

    # 5. Continuous environmental state classification with hysteresis
    if new_state == "CLUTTERED_CORRIDOR":
        if dens_clamped < 0.25 and front_dist > 1.60:
            new_state = "MODERATE_TRAFFIC"
    elif new_state == "MODERATE_TRAFFIC":
        if dens_clamped >= 0.35 or (front_dist < 1.35 and dens_clamped >= 0.18):
            new_state = "CLUTTERED_CORRIDOR"
        elif dens_clamped < 0.04 and front_dist >= 2.40:
            new_state = "EMPTY_CORRIDOR"
    elif new_state in ("EMPTY_CORRIDOR", "STARTING", "RESUME_NAVIGATION", "PATH_CLEAR"):
        if dens_clamped >= 0.35 or (front_dist < 1.35 and dens_clamped >= 0.20):
            new_state = "CLUTTERED_CORRIDOR"
        elif dens_clamped >= 0.08 or front_dist < 2.20:
            new_state = "MODERATE_TRAFFIC"
        else:
            new_state = "EMPTY_CORRIDOR"
    else:
        # Default state determination
        if dens_clamped >= 0.35 or (front_dist < 1.35 and dens_clamped >= 0.20):
            new_state = "CLUTTERED_CORRIDOR"
        elif dens_clamped >= 0.08 or front_dist < 2.20:
            new_state = "MODERATE_TRAFFIC"
        else:
            new_state = "EMPTY_CORRIDOR"

    # 6. Apply state-based speed bounding
    if new_state == "CLUTTERED_CORRIDOR":
        v_actual = max(0.10, min(v_raw, 0.15))
    elif new_state == "MODERATE_TRAFFIC":
        v_actual = max(0.20, min(v_raw, 0.28))
    elif new_state in ("EMPTY_CORRIDOR", "PATH_CLEAR", "RESUME_NAVIGATION"):
        v_actual = min(v_raw, 0.40)
    else:
        v_actual = v_raw

    scale = (v_actual / v_clamped) if v_clamped > 0 else 1.0
    return v_actual, omega_clamped, scale, new_state


class DistanceNode(Node):
    """
    ROS 2 Adaptive Speed Controller node for Start-to-End Navigation.
    - Subscribes to /scan, /cmd_vel_in, and /odom.
    - Continuous LiDAR-driven environmental analysis (distance & density).
    - Hysteresis-governed state machine: EMPTY_CORRIDOR, MODERATE_TRAFFIC,
      CLUTTERED_CORRIDOR, CRITICAL_PROXIMITY, CONTROLLED_STOP, PATH_CLEAR,
      RESUME_NAVIGATION, DESTINATION_REACHED.
    - Sole publisher of final motion commands to /cmd_vel.
    - Smooth rate-limited acceleration and safe timeouts.
    """

    def __init__(self):
        super().__init__('distance_node')

        # Parameters
        self.declare_parameter('threshold', 0.45)
        self.declare_parameter('slowdown_distance', 2.00)
        self.declare_parameter('density_radius', 1.30)
        self.declare_parameter('density_influence', 0.60)
        self.declare_parameter('c_min', 0.25)
        self.declare_parameter('max_linear_speed', 0.40)
        self.declare_parameter('max_angular_speed', 1.00)
        self.declare_parameter('front_angle_deg', 60.0)
        self.declare_parameter('sensor_timeout', 0.50)
        self.declare_parameter('cmd_timeout', 0.50)
        self.declare_parameter('control_rate', 20.0)

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
        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)
        self.sensor_timeout = float(self.get_parameter('sensor_timeout').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.control_rate = float(self.get_parameter('control_rate').value)

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

        front_rad = math.radians(self.front_angle_deg)

        global_min_dist = float('inf')
        global_min_angle = 0.0
        front_min_dist = float('inf')

        n_valid_front = 0
        n_close_front = 0

        for i, r in enumerate(scan.ranges):
            if r is None or not math.isfinite(r) or r < min_range or r > max_range:
                continue

            angle = scan.angle_min + i * scan.angle_increment

            if r < global_min_dist:
                global_min_dist = r
                global_min_angle = angle

            if -front_rad <= angle <= front_rad:
                n_valid_front += 1
                if r < front_min_dist:
                    front_min_dist = r
                if r <= self.density_radius:
                    n_close_front += 1

        direction = "unknown"
        if math.isfinite(global_min_dist):
            deg = math.degrees(global_min_angle)
            if -30.0 <= deg <= 30.0:
                direction = "front"
            elif 30.0 < deg <= 150.0:
                direction = "left"
            elif -150.0 <= deg < -30.0:
                direction = "right"
            else:
                direction = "behind"

        density = (float(n_close_front) / float(n_valid_front)) if n_valid_front > 0 else 0.0
        density = max(0.0, min(1.0, density))

        self.closest_distance = global_min_dist
        self.closest_direction = direction
        self.front_distance = front_min_dist
        self.obstacle_density = density

        # Telemetry
        info = ObstacleInfo()
        info.closest_distance = float(global_min_dist)
        info.direction = direction
        info.threshold = float(self.threshold)
        self.info_pub.publish(info)

        dens_msg = Float32()
        dens_msg.data = float(density)
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
        )

        # Smooth acceleration / rate limiter (dt = 0.05s, max accel = 0.5 m/s^2)
        dt = 1.0 / self.control_rate
        max_delta_v = 0.60 * dt
        if v_cmd_target > self.last_cmd_linear:
            v_cmd = min(v_cmd_target, self.last_cmd_linear + max_delta_v)
        else:
            v_cmd = v_cmd_target

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
    node = DistanceNode()
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
