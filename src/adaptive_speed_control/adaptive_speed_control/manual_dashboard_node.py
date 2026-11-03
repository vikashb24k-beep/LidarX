#!/usr/bin/env python3
import math
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from adaptive_speed_interfaces.msg import ObstacleInfo, RobotStatus
from adaptive_speed_control.utils.geometry_utils import quaternion_to_yaw


class ManualDashboardNode(Node):
    """
    Live ANSI terminal dashboard for interactive manual driving and real-time environmental diagnostics.
    Refreshes in-place at 5 Hz with zero scrolling spam.
    """

    def __init__(self):
        super().__init__('manual_dashboard_node')

        self.declare_parameter('refresh_rate', 5.0)
        self.refresh_rate = float(self.get_parameter('refresh_rate').value)

        # Subscribers
        self.status_sub = self.create_subscription(RobotStatus, '/robot_status', self.on_robot_status, 10)
        self.mode_sub = self.create_subscription(String, '/control_mode', self.on_control_mode, 10)
        self.adaptive_sub = self.create_subscription(Bool, '/toggle_adaptive', self.on_adaptive_toggle, 10)
        self.cmd_in_sub = self.create_subscription(Twist, '/cmd_vel_in', self.on_cmd_vel_in, 10)
        self.obs_sub = self.create_subscription(ObstacleInfo, '/obstacle_info', self.on_obstacle_info, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)

        # Telemetry State
        self.control_mode = "MANUAL"
        self.adaptive_enabled = True
        self.v_requested = 0.0
        self.v_safe = 0.0
        self.v_actual = 0.0
        self.speed_limit = 0.40
        self.zone = "FREE"
        self.limit_reason = "CLEAR"
        self.closest_dist = float('inf')
        self.closest_dir = "front"
        self.front_dist = float('inf')
        self.left_dist = float('inf')
        self.right_dist = float('inf')
        self.density_rho = 0.0
        self.ttc = -1.0
        self.heading_deg = 0.0
        self.distance_travelled = 0.0

        timer_period = 1.0 / max(1.0, self.refresh_rate)
        self.timer = self.create_timer(timer_period, self.render_hud)

    def on_robot_status(self, msg: RobotStatus):
        self.v_actual = msg.speed
        self.v_safe = msg.commanded_speed
        self.speed_limit = msg.speed_limit if msg.speed_limit > 0 else 0.40
        self.zone = msg.zone
        self.distance_travelled = msg.distance_travelled
        if hasattr(msg, 'limit_reason') and msg.limit_reason:
            self.limit_reason = msg.limit_reason
        if hasattr(msg, 'time_to_collision') and msg.time_to_collision is not None:
            self.ttc = msg.time_to_collision

    def on_control_mode(self, msg: String):
        self.control_mode = msg.data.upper()

    def on_adaptive_toggle(self, msg: Bool):
        self.adaptive_enabled = bool(msg.data)

    def on_cmd_vel_in(self, msg: Twist):
        self.v_requested = msg.linear.x

    def on_obstacle_info(self, msg: ObstacleInfo):
        self.closest_dist = msg.closest_distance
        self.closest_dir = msg.direction
        self.front_dist = msg.front_distance
        self.left_dist = msg.left_distance
        self.right_dist = msg.right_distance
        self.density_rho = msg.density_rho
        if hasattr(msg, 'limit_reason') and msg.limit_reason:
            self.limit_reason = msg.limit_reason
        if hasattr(msg, 'time_to_collision') and msg.time_to_collision is not None:
            self.ttc = msg.time_to_collision

    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.heading_deg = math.degrees(yaw)

    def _speed_color(self, speed: float) -> str:
        if speed >= 0.28:
            return "\033[92m"  # Green
        elif speed >= 0.18:
            return "\033[93m"  # Yellow
        elif speed >= 0.05:
            return "\033[38;5;208m"  # Orange
        else:
            return "\033[91m"  # Red

    def _speed_color_name(self, speed: float) -> str:
        if speed >= 0.28:
            return "GREEN"
        elif speed >= 0.18:
            return "YELLOW"
        elif speed >= 0.05:
            return "ORANGE"
        else:
            return "RED"

    def _zone_color(self, zone: str) -> str:
        z = zone.upper()
        if 'FREE' in z:
            return "\033[92m"
        elif 'MODERATE' in z:
            return "\033[93m"
        elif 'DENSE' in z or 'CLUTTER' in z:
            return "\033[38;5;208m"
        else:
            return "\033[91m"

    def render_hud(self):
        reset = "\033[0m"
        bold = "\033[1m"
        clear_screen = "\033[H\033[2J"

        # Percentage calculation for speed bar
        max_limit = max(0.01, self.speed_limit)
        pct = max(0, min(100, int((self.v_actual / max_limit) * 100)))
        bar_len = 16
        filled = int((pct / 100.0) * bar_len)
        bar_str = "[" + "#" * filled + "-" * (bar_len - filled) + "]"

        # Colors
        v_col = self._speed_color(self.v_actual)
        z_col = self._zone_color(self.zone)
        col_name = self._speed_color_name(self.v_actual)
        ttc_str = f"{self.ttc:4.1f} s" if (self.ttc >= 0 and self.ttc < 90.0) else "N/A"

        adapt_str = "ON" if self.adaptive_enabled else "\033[91mOFF (emergency stop still on)\033[0m"

        front_str = f"{self.front_dist:.2f} m" if math.isfinite(self.front_dist) else "> 10 m"
        left_str = f"{self.left_dist:.2f} m" if math.isfinite(self.left_dist) else "> 10 m"
        right_str = f"{self.right_dist:.2f} m" if math.isfinite(self.right_dist) else "> 10 m"

        hud = (
            f"{clear_screen}"
            f"================================================================================\n"
            f" {bold}ROBOT SAFETY & SPEED INTERACTION DASHBOARD (5 Hz){reset}\n"
            f"================================================================================\n"
            f" MODE: {bold}{self.control_mode:<12}{reset} ADAPTIVE: {adapt_str}\n"
            f" requested {self.v_requested:.2f} m/s -> safe {self.v_safe:.2f} m/s -> {v_col}actual {self.v_actual:.2f} m/s{reset}   {bar_str} {pct}% of max\n"
            f" ZONE: {z_col}{self.zone:<10}{reset} reason: {bold}{self.limit_reason}{reset}\n"
            f" nearest: front {front_str} | left {left_str} | right {right_str} | time-to-collision {ttc_str}\n"
            f" heading: {self.heading_deg:4.0f} deg   total distance: {self.distance_travelled:4.1f} m   speed colour: {v_col}{col_name}{reset}\n"
            f"================================================================================\n"
            f" [Controls: Use keyboard teleop terminal to drive | Press Ctrl-C in teleop to stop]\n"
        )

        sys.stdout.write(hud)
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = ManualDashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("\n[INFO] Manual dashboard closed.")


if __name__ == '__main__':
    main()
