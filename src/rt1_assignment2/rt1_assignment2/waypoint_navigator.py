import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from rt1_assignment2_interfaces.msg import ObstacleInfo


def quaternion_to_yaw(q) -> float:
    """Extracts yaw (rotation around Z axis) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointNavigator(Node):
    """
    Lightweight autonomous corridor waypoint-following controller.
    - Subscribes to /odom for odometry feedback.
    - Tracks the predefined corridor route from START (-9.0, 0.0) to GOAL (9.0, 0.0).
    - Uses lateral feedback to keep the robot centered along y = 0.0.
    - Streams requested velocities to /cmd_vel_in at 10 Hz.
    - Safely interacts with the adaptive controller (distance_node), which acts as the velocity arbiter.
    """

    def __init__(self):
        super().__init__('waypoint_navigator')

        # Parameters
        self.declare_parameter('start_x', -9.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', 9.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('nominal_speed', 0.40)
        self.declare_parameter('goal_tolerance', 0.35)
        self.declare_parameter('auto_start', True)

        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.nominal_speed = float(self.get_parameter('nominal_speed').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.is_navigating = bool(self.get_parameter('auto_start').value)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.obstacle_sub = self.create_subscription(ObstacleInfo, '/obstacle_info', self.on_obstacle_info, 10)

        # Publisher to /cmd_vel_in (Upstream input to Adaptive Controller)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_in', 10)

        # Services
        self.start_srv = self.create_service(Trigger, '/start_navigation', self.handle_start_nav)
        self.stop_srv = self.create_service(Trigger, '/stop_navigation', self.handle_stop_nav)

        # Robot state
        self.current_x = self.start_x
        self.current_y = self.start_y
        self.current_yaw = 0.0
        self.has_odom = False

        self.closest_dist = float('inf')
        self.destination_reached = False
        self.last_log_time = 0.0

        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.navigation_loop)

        self.get_logger().info(
            f"\n"
            f"===============================================================\n"
            f"   Start-to-End Corridor Waypoint Navigator Initialized\n"
            f"===============================================================\n"
            f"  [NAVIGATION] Start: ({self.start_x:.1f}, {self.start_y:.1f}) | Goal: ({self.goal_x:.1f}, {self.goal_y:.1f})\n"
            f"  Nominal Speed       : {self.nominal_speed:.2f} m/s\n"
            f"  Publishing Target   : /cmd_vel_in (filtered by distance_node)\n"
            f"  Feedback Source     : /odom\n"
            f"  Status              : {'AUTONOMOUS RUNNING' if self.is_navigating else 'STANDBY'}\n"
            f"==============================================================="
        )

    def handle_start_nav(self, request, response):
        self.is_navigating = True
        self.destination_reached = False
        response.success = True
        response.message = f"Navigation started towards goal ({self.goal_x:.1f}, {self.goal_y:.1f})."
        self.get_logger().info(response.message)
        return response

    def handle_stop_nav(self, request, response):
        self.is_navigating = False
        self.cmd_pub.publish(Twist())
        response.success = True
        response.message = "Navigation halted by user."
        self.get_logger().info(response.message)
        return response

    def on_odom(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.has_odom = True

    def on_obstacle_info(self, msg: ObstacleInfo):
        self.closest_dist = msg.closest_distance

    def navigation_loop(self):
        if not self.has_odom or not self.is_navigating:
            return

        # Check goal arrival
        dist_to_goal = math.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)
        if (self.current_x >= (self.goal_x - self.goal_tolerance)) or dist_to_goal <= self.goal_tolerance:
            if not self.destination_reached:
                self.destination_reached = True
                self.is_navigating = False
                self.cmd_pub.publish(Twist())
                self.get_logger().info(
                    f"\n"
                    f"=============================================================\n"
                    f" [SUCCESS] DESTINATION_REACHED!\n"
                    f" Robot arrived at destination ({self.current_x:.2f}, {self.current_y:.2f}).\n"
                    f" Total corridor traverse completed successfully.\n"
                    f"============================================================="
                )
            return

        # Corridor Centerline Guidance:
        # Desired line is y = 0.0 towards +X direction.
        # Lateral error: robot is at current_y; cross-track error = -current_y
        cross_track_err = -self.current_y
        target_heading = math.atan2(cross_track_err, 1.5)  # Aim 1.5m ahead on centerline
        heading_err = target_heading - self.current_yaw

        # Normalize heading error to [-pi, pi]
        heading_err = (heading_err + math.pi) % (2 * math.pi) - math.pi

        # Proportional heading & lateral steering control
        ang_cmd = 1.8 * heading_err
        ang_cmd = max(-0.8, min(0.8, ang_cmd))

        # Nominal forward speed
        lin_cmd = self.nominal_speed

        cmd = Twist()
        cmd.linear.x = float(lin_cmd)
        cmd.angular.z = float(ang_cmd)
        self.cmd_pub.publish(cmd)

        # Periodic navigation logging
        now = time.time()
        if (now - self.last_log_time) >= 1.5:
            self.last_log_time = now
            total_dist = self.goal_x - self.start_x
            progress = max(0.0, min(100.0, ((self.current_x - self.start_x) / total_dist) * 100.0))
            self.get_logger().info(
                f"[NAVIGATION] Progress: {progress:5.1f}% | Pos: ({self.current_x:5.2f}, {self.current_y:5.2f}) | "
                f"Goal: ({self.goal_x:.1f}, {self.goal_y:.1f}) | Requesting: {lin_cmd:.2f} m/s"
            )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
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

