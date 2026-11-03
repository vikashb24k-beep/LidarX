import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from adaptive_speed_interfaces.msg import ObstacleInfo
from adaptive_speed_control.utils.geometry_utils import (
    quaternion_to_yaw,
    normalize_angle,
    euclidean_distance,
    cross_track_error,
)


class WaypointNavigatorNode(Node):
    """
    Autonomous Corridor Waypoint Navigator.
    - Full State Machine: IDLE, ROTATE_TO_GOAL, DRIVE, AVOID_OBSTACLE, RECOVERY, GOAL_REACHED, FAILED.
    - Tracks sequential corridor waypoints from START to GOAL.
    - Subscribes to /odom for pose tracking and /obstacle_info for reactive obstacle avoidance.
    - Publishes desired velocities to /cmd_vel_in.
    - Stuck detection and automatic recovery behaviors.
    - Services: /start_navigation, /stop_navigation, /reset_navigation.
    """

    def __init__(self):
        super().__init__('waypoint_navigator_node')

        # Parameters
        self.declare_parameter('start_x', -9.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', 9.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('nominal_speed', 0.40)
        self.declare_parameter('goal_tolerance', 0.35)
        self.declare_parameter('waypoint_tolerance', 0.60)
        self.declare_parameter('heading_gain', 1.80)
        self.declare_parameter('lookahead_dist', 1.50)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('stuck_timeout', 6.0)
        self.declare_parameter('navigation_timeout', 120.0)

        self._load_parameters()

        # Sequential corridor waypoints
        self.waypoints = [
            (-5.0, 0.0),  # End Zone 1
            (-1.0, 0.0),  # End Zone 2
            (4.0, 0.0),   # End Zone 3
            (self.goal_x, self.goal_y)  # Zone 4 Destination
        ]
        self.current_wp_idx = 0

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.obstacle_sub = self.create_subscription(ObstacleInfo, '/obstacle_info', self.on_obstacle_info, 10)

        # Publisher to /cmd_vel_in
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_in', 10)

        # Services
        self.start_srv = self.create_service(Trigger, '/start_navigation', self.handle_start_nav)
        self.stop_srv = self.create_service(Trigger, '/stop_navigation', self.handle_stop_nav)
        self.reset_srv = self.create_service(Trigger, '/reset_navigation', self.handle_reset_nav)

        # Navigation State Machine
        self.nav_state = 'DRIVE' if self.auto_start else 'IDLE'
        self.current_x = self.start_x
        self.current_y = self.start_y
        self.current_yaw = 0.0
        self.has_odom = False

        # Obstacle Telemetry
        self.closest_dist = float('inf')
        self.front_dist = float('inf')
        self.left_dist = float('inf')
        self.right_dist = float('inf')

        # Stuck & Recovery Tracking
        self.last_progress_x = self.start_x
        self.last_progress_time = time.time()
        self.recovery_start_time = 0.0
        self.recovery_phase = 0

        # Mission Timing & Metrics
        self.mission_start_time = time.time()
        self.last_log_time = 0.0
        self.destination_reached = False

        # 10 Hz Control Loop
        self.timer = self.create_timer(0.1, self.navigation_loop)

        self.get_logger().info(
            f"\n"
            f"===============================================================\n"
            f"   Start-to-End Corridor Waypoint Navigator Initialized\n"
            f"===============================================================\n"
            f"  Route               : ({self.start_x:.1f}, {self.start_y:.1f}) -> ({self.goal_x:.1f}, {self.goal_y:.1f})\n"
            f"  Waypoints Count     : {len(self.waypoints)}\n"
            f"  Nominal Speed       : {self.nominal_speed:.2f} m/s\n"
            f"  Publishing Target   : /cmd_vel_in\n"
            f"  Feedback Source     : /odom, /obstacle_info\n"
            f"  Initial State       : {self.nav_state}\n"
            f"==============================================================="
        )

    def _load_parameters(self):
        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.nominal_speed = float(self.get_parameter('nominal_speed').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.heading_gain = float(self.get_parameter('heading_gain').value)
        self.lookahead_dist = float(self.get_parameter('lookahead_dist').value)
        self.auto_start = bool(self.get_parameter('auto_start').value)
        self.stuck_timeout = float(self.get_parameter('stuck_timeout').value)
        self.navigation_timeout = float(self.get_parameter('navigation_timeout').value)

    @property
    def is_navigating(self) -> bool:
        return self.nav_state in ('ROTATE_TO_GOAL', 'DRIVE', 'AVOID_OBSTACLE', 'RECOVERY')

    def handle_start_nav(self, request, response):
        self.nav_state = 'DRIVE'
        self.destination_reached = False
        self.last_progress_time = time.time()
        self.mission_start_time = time.time()
        response.success = True
        response.message = f"Navigation started toward ({self.goal_x:.1f}, {self.goal_y:.1f})."
        self.get_logger().info(response.message)
        return response

    def handle_stop_nav(self, request, response):
        self.nav_state = 'IDLE'
        self.cmd_pub.publish(Twist())
        response.success = True
        response.message = "Navigation stopped."
        self.get_logger().info(response.message)
        return response

    def handle_reset_nav(self, request, response):
        self.nav_state = 'DRIVE' if self.auto_start else 'IDLE'
        self.destination_reached = False
        self.current_wp_idx = 0
        self.last_progress_x = self.current_x
        self.last_progress_time = time.time()
        self.mission_start_time = time.time()
        response.success = True
        response.message = "Navigation reset."
        self.get_logger().info(response.message)
        return response

    def on_odom(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.has_odom = True

    def on_obstacle_info(self, msg: ObstacleInfo):
        self.closest_dist = msg.closest_distance
        self.front_dist = msg.front_distance
        self.left_dist = msg.left_distance
        self.right_dist = msg.right_distance

    def navigation_loop(self):
        if not self.has_odom or self.nav_state == 'IDLE':
            return

        now = time.time()

        # Check final destination reached
        dist_to_goal = euclidean_distance(self.current_x, self.current_y, self.goal_x, self.goal_y)
        if (self.current_x >= (self.goal_x - self.goal_tolerance)) or dist_to_goal <= self.goal_tolerance:
            if not self.destination_reached:
                self.destination_reached = True
                self.nav_state = 'GOAL_REACHED'
                self.cmd_pub.publish(Twist())
                elapsed = now - self.mission_start_time
                self.get_logger().info(
                    f"\n"
                    f"=============================================================\n"
                    f" [SUCCESS] DESTINATION REACHED!\n"
                    f" Pos: ({self.current_x:.2f}, {self.current_y:.2f}) | Dist to Goal: {dist_to_goal:.2f}m\n"
                    f" Traverse time: {elapsed:.1f} s\n"
                    f"============================================================="
                )
            return

        # Advance intermediate waypoint if reached
        if self.current_wp_idx < len(self.waypoints) - 1:
            wp_x, wp_y = self.waypoints[self.current_wp_idx]
            if euclidean_distance(self.current_x, self.current_y, wp_x, wp_y) <= self.waypoint_tolerance or self.current_x >= wp_x:
                self.current_wp_idx += 1
                self.get_logger().info(f"[NAV] Advanced to waypoint {self.current_wp_idx}: {self.waypoints[self.current_wp_idx]}")

        # Check global navigation timeout
        if (now - self.mission_start_time) > self.navigation_timeout:
            self.nav_state = 'FAILED'
            self.cmd_pub.publish(Twist())
            self.get_logger().error(f"[TIMEOUT] Mission exceeded timeout ({self.navigation_timeout}s). Navigation halted.")
            return

        # Check progress for stuck detection
        if abs(self.current_x - self.last_progress_x) > 0.15:
            self.last_progress_x = self.current_x
            self.last_progress_time = now
        elif (now - self.last_progress_time) > self.stuck_timeout and self.nav_state != 'RECOVERY':
            self.get_logger().warn(
                f"[STUCK DETECTED] No progress in {self.stuck_timeout:.1f}s at x={self.current_x:.2f}. Entering RECOVERY mode."
            )
            self.nav_state = 'RECOVERY'
            self.recovery_start_time = now
            self.recovery_phase = 0

        # State Machine Execution
        cmd = Twist()

        if self.nav_state == 'RECOVERY':
            elapsed_rec = now - self.recovery_start_time
            if elapsed_rec < 1.5:
                # Phase 0: Gently reverse
                cmd.linear.x = -0.15
                cmd.angular.z = 0.0
            elif elapsed_rec < 3.0:
                # Phase 1: Turn towards side with greater clearance
                cmd.linear.x = 0.0
                cmd.angular.z = 0.4 if (self.left_dist > self.right_dist) else -0.4
            else:
                self.get_logger().info("[RECOVERY] Recovery complete. Resuming DRIVE.")
                self.nav_state = 'DRIVE'
                self.last_progress_time = now
                self.last_progress_x = self.current_x

        elif self.nav_state in ('DRIVE', 'AVOID_OBSTACLE'):
            # Corridor Centerline Guidance: Desired line is y = 0.0 towards +X direction.
            cte = cross_track_error(self.current_x, self.current_y, line_y=0.0)
            target_heading = math.atan2(-cte, self.lookahead_dist)
            heading_err = normalize_angle(target_heading - self.current_yaw)

            # Lateral steering
            ang_cmd = self.heading_gain * heading_err

            # Reactive obstacle nudge if obstacle detected on one side closer than 0.70m
            if self.left_dist < 0.70 and self.left_dist < self.right_dist:
                ang_cmd -= 0.25  # Steer right away from left obstacle
                self.nav_state = 'AVOID_OBSTACLE'
            elif self.right_dist < 0.70 and self.right_dist < self.left_dist:
                ang_cmd += 0.25  # Steer left away from right obstacle
                self.nav_state = 'AVOID_OBSTACLE'
            else:
                self.nav_state = 'DRIVE'

            ang_cmd = max(-0.8, min(0.8, ang_cmd))
            lin_cmd = self.nominal_speed

            cmd.linear.x = float(lin_cmd)
            cmd.angular.z = float(ang_cmd)

        self.cmd_pub.publish(cmd)

        # Periodic navigation logging
        if (now - self.last_log_time) >= 1.5:
            self.last_log_time = now
            total_dist = max(1.0, self.goal_x - self.start_x)
            progress = max(0.0, min(100.0, ((self.current_x - self.start_x) / total_dist) * 100.0))
            self.get_logger().info(
                f"[NAV] State: {self.nav_state:<14} | Progress: {progress:5.1f}% | "
                f"Pos: ({self.current_x:5.2f}, {self.current_y:5.2f}) | Front: {self.front_dist:.2f}m"
            )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigatorNode()
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
