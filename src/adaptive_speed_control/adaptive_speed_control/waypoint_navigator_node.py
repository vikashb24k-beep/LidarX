import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

from adaptive_speed_interfaces.msg import ObstacleInfo
from adaptive_speed_control.utils.geometry_utils import (
    quaternion_to_yaw,
    normalize_angle,
    euclidean_distance,
)


class WaypointNavigatorNode(Node):
    """
    Autonomous 2D Waypoint Navigator with Reactive Dynamic Obstacle Avoidance.
    - Full State Machine: IDLE, DRIVE, WAITING, AVOID_OBSTACLE, RECOVERY, GOAL_REACHED, FAILED.
    - Follows arbitrary sequential 2D waypoints loaded via YAML parameters.
    - Subscribes to /odom for pose tracking and /obstacle_info for reactive obstacle avoidance.
    - Publishes desired velocities to /cmd_vel_in.
    - Dynamic obstacle soft-stop (WAITING) and automatic recovery behaviors.
    - Services: /start_navigation, /stop_navigation, /reset_navigation.
    """

    def __init__(self):
        super().__init__('waypoint_navigator_node')

        # Parameters
        self.declare_parameter('start_x', -9.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('goal_x', 9.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('nominal_speed', 0.40)
        self.declare_parameter('goal_tolerance', 0.40)
        self.declare_parameter('waypoint_tolerance', 0.65)
        self.declare_parameter('heading_gain', 1.80)
        self.declare_parameter('lookahead_dist', 1.50)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('stuck_timeout', 6.0)
        self.declare_parameter('wait_timeout', 4.0)
        self.declare_parameter('navigation_timeout', 180.0)
        self.declare_parameter('waypoints', [float('nan')])
        self.declare_parameter('initial_mode', 'AUTONOMOUS')

        self._load_parameters()

        # Load sequential waypoints
        raw_waypoints = self.get_parameter('waypoints').value
        valid_coords = [float(v) for v in raw_waypoints if not math.isnan(v)]
        if len(valid_coords) >= 4 and len(valid_coords) % 2 == 0:
            self.waypoints = [
                (valid_coords[i], valid_coords[i + 1])
                for i in range(0, len(valid_coords), 2)
            ]
        else:
            # Default corridor waypoints for scenarios 1-4
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
        self.mode_sub = self.create_subscription(String, '/control_mode', self.on_control_mode, 10)

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

        # Dynamic Obstacle Wait & Recovery Tracking
        self.blocked_start_time = 0.0
        self.last_progress_x = self.start_x
        self.last_progress_y = self.start_y
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
            f"   Start-to-End Autonomous Waypoint Navigator Initialized\n"
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
        self.wait_timeout = float(self.get_parameter('wait_timeout').value)
        self.navigation_timeout = float(self.get_parameter('navigation_timeout').value)
        self.control_mode = str(self.get_parameter('initial_mode').value).upper()

    def on_control_mode(self, msg: String):
        prev = self.control_mode
        self.control_mode = msg.data.upper()
        if prev != self.control_mode:
            self.get_logger().info(f"[NAV] Control mode changed: {prev} -> {self.control_mode}")
            if self.control_mode == 'MANUAL':
                self.cmd_pub.publish(Twist())

    @property
    def is_navigating(self) -> bool:
        return self.nav_state in ('ROTATE_TO_GOAL', 'DRIVE', 'WAITING', 'AVOID_OBSTACLE', 'RECOVERY')

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
        self.last_progress_y = self.current_y
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
        if not self.has_odom or self.nav_state == 'IDLE' or self.control_mode == 'MANUAL':
            return

        now = time.time()

        # Check final destination reached
        dist_to_goal = euclidean_distance(self.current_x, self.current_y, self.goal_x, self.goal_y)
        is_at_final_wp = (self.current_wp_idx >= len(self.waypoints) - 1)
        straight_corridor_reached = (self.current_x >= (self.goal_x - self.goal_tolerance)) and abs(self.current_y - self.goal_y) < 1.0

        if dist_to_goal <= self.goal_tolerance or (is_at_final_wp and dist_to_goal <= self.goal_tolerance * 1.5) or straight_corridor_reached:
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
            dist_to_wp = euclidean_distance(self.current_x, self.current_y, wp_x, wp_y)
            # Advance if close enough or if progressing past in straight corridor
            if dist_to_wp <= self.waypoint_tolerance or (self.current_x >= wp_x and abs(wp_y - self.current_y) < 0.4 and (wp_y == 0.0)):
                self.current_wp_idx += 1
                self.get_logger().info(f"[NAV] Advanced to waypoint {self.current_wp_idx}: {self.waypoints[self.current_wp_idx]}")

        # Check global navigation timeout
        if (now - self.mission_start_time) > self.navigation_timeout:
            self.nav_state = 'FAILED'
            self.cmd_pub.publish(Twist())
            self.get_logger().error(f"[TIMEOUT] Mission exceeded timeout ({self.navigation_timeout}s). Navigation halted.")
            return

        # Check progress for stuck detection
        dist_moved = math.hypot(self.current_x - self.last_progress_x, self.current_y - self.last_progress_y)
        if dist_moved > 0.15:
            self.last_progress_x = self.current_x
            self.last_progress_y = self.current_y
            self.last_progress_time = now
        elif (now - self.last_progress_time) > self.stuck_timeout and self.nav_state not in ('RECOVERY', 'WAITING'):
            self.get_logger().warn(
                f"[STUCK DETECTED] No progress in {self.stuck_timeout:.1f}s at ({self.current_x:.2f}, {self.current_y:.2f}). Entering RECOVERY mode."
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
                cmd.angular.z = 0.45 if (self.left_dist > self.right_dist) else -0.45
            else:
                self.get_logger().info("[RECOVERY] Recovery complete. Resuming DRIVE.")
                self.nav_state = 'DRIVE'
                self.last_progress_time = now
                self.last_progress_x = self.current_x
                self.last_progress_y = self.current_y

        elif self.nav_state in ('DRIVE', 'WAITING', 'AVOID_OBSTACLE'):
            target_wp = self.waypoints[self.current_wp_idx]
            target_heading = math.atan2(target_wp[1] - self.current_y, target_wp[0] - self.current_x)
            heading_err = normalize_angle(target_heading - self.current_yaw)

            # Lateral steering
            ang_cmd = self.heading_gain * heading_err

            # Dynamic obstacle blocking logic:
            # If obstacle directly in front closer than 0.65m and no clear corridor
            if self.front_dist < 0.65 and self.front_dist < min(self.left_dist, self.right_dist) and (self.left_dist < 0.75 or self.right_dist < 0.75):
                if self.blocked_start_time == 0.0:
                    self.blocked_start_time = now
                time_blocked = now - self.blocked_start_time

                if time_blocked < self.wait_timeout:
                    # Soft stop: wait for dynamic obstacle to move past
                    self.nav_state = 'WAITING'
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.cmd_pub.publish(cmd)
                    return
                else:
                    # Wait timed out: enter recovery to go around
                    self.get_logger().warn(f"[WAIT TIMEOUT] Blocked for {time_blocked:.1f}s. Initiating RECOVERY.")
                    self.nav_state = 'RECOVERY'
                    self.recovery_start_time = now
                    self.blocked_start_time = 0.0
                    return
            else:
                self.blocked_start_time = 0.0

            # Reactive obstacle nudge if obstacle detected on one side closer than 0.60m
            if self.left_dist < 0.60 and self.left_dist < self.right_dist:
                ang_cmd -= 0.20  # Steer right away from left obstacle
                self.nav_state = 'AVOID_OBSTACLE'
            elif self.right_dist < 0.60 and self.right_dist < self.left_dist:
                ang_cmd += 0.20  # Steer left away from right obstacle
            else:
                self.nav_state = 'DRIVE'

            ang_cmd = max(-0.85, min(0.85, ang_cmd))

            # Scale forward speed based on turn sharpness
            if abs(heading_err) > 0.45:
                # Pivot in place / crawl to align heading cleanly before moving forward
                lin_cmd = 0.04
            else:
                speed_factor = max(0.25, math.cos(heading_err))
                lin_cmd = self.nominal_speed * speed_factor

            cmd.linear.x = float(lin_cmd)
            cmd.angular.z = float(ang_cmd)

        self.cmd_pub.publish(cmd)

        # Periodic navigation logging
        if (now - self.last_log_time) >= 1.5:
            self.last_log_time = now
            total_dist = max(1.0, euclidean_distance(self.start_x, self.start_y, self.goal_x, self.goal_y))
            rem_dist = euclidean_distance(self.current_x, self.current_y, self.goal_x, self.goal_y)
            progress = max(0.0, min(100.0, ((total_dist - rem_dist) / total_dist) * 100.0))
            self.get_logger().info(
                f"[NAV] State: {self.nav_state:<14} | WP {self.current_wp_idx+1}/{len(self.waypoints)} | "
                f"Progress: {progress:5.1f}% | Pos: ({self.current_x:5.2f}, {self.current_y:5.2f}) | Front: {self.front_dist:.2f}m"
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
