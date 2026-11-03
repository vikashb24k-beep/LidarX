#!/usr/bin/env python3
import os
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from adaptive_speed_interfaces.msg import ObstacleInfo, RobotStatus


class SpeedDashboardNode(Node):
    """
    Live Visual Speed & Safety Dashboard using Matplotlib.
    - Gauge / Bar for actual speed vs. max speed.
    - Rolling time-series plot of requested vs. limit vs. actual speed with zone shading.
    - Live text telemetry for d_min, obstacle density rho, and safety zone.
    - Headless detection: skips display gracefully if DISPLAY environment is missing.
    """

    def __init__(self):
        super().__init__('speed_dashboard_node')

        self.declare_parameter('max_speed', 0.50)
        self.declare_parameter('window_seconds', 20.0)
        self.declare_parameter('update_rate', 5.0)

        self.max_speed = float(self.get_parameter('max_speed').value)
        self.window_sec = float(self.get_parameter('window_seconds').value)
        self.update_rate = float(self.get_parameter('update_rate').value)

        # Check graphical display availability
        self.has_display = bool(os.environ.get('DISPLAY'))

        # Data buffers for rolling plots
        self.timestamps = deque()
        self.actual_speeds = deque()
        self.commanded_speeds = deque()
        self.speed_limits = deque()
        self.zones = deque()

        self.current_speed = 0.0
        self.current_cmd = 0.0
        self.current_limit = 0.40
        self.current_zone = 'FREE'
        self.current_dmin = 0.45
        self.current_front_dist = 9.99
        self.current_rho = 0.0

        self.start_time = time.time()

        # Subscribers
        self.status_sub = self.create_subscription(RobotStatus, '/robot_status', self.on_robot_status, 10)
        self.obstacle_sub = self.create_subscription(ObstacleInfo, '/obstacle_info', self.on_obstacle_info, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)

        if not self.has_display:
            self.get_logger().info(
                "No graphical DISPLAY detected in environment. SpeedDashboardNode will run in headless monitoring mode."
            )
            return

        # Setup Matplotlib GUI
        try:
            import matplotlib
            import matplotlib.pyplot as plt
            import matplotlib.patches as patches

            self.plt = plt
            self.patches = patches
            self.plt.ion()
            self.fig, (self.ax_gauge, self.ax_plot, self.ax_info) = self.plt.subplots(
                3, 1, figsize=(8, 7), gridspec_kw={'height_ratios': [1, 2.5, 1]}
            )
            self.fig.suptitle('Adaptive Speed Control - Live Telemetry Dashboard', fontsize=12, fontweight='bold')
            self.fig.canvas.manager.set_window_title('Adaptive Speed Dashboard')
            self._init_plots()

            timer_period = 1.0 / max(1.0, self.update_rate)
            self.timer = self.create_timer(timer_period, self.update_gui)
            self.get_logger().info("SpeedDashboardNode initialized GUI successfully.")
        except Exception as e:
            self.get_logger().warn(f"Failed to initialize graphical dashboard window: {e}. Running in headless fallback.")
            self.has_display = False

    def on_robot_status(self, msg: RobotStatus):
        self.current_speed = msg.speed
        self.current_cmd = msg.commanded_speed
        self.current_limit = msg.speed_limit
        self.current_zone = msg.zone

    def on_obstacle_info(self, msg: ObstacleInfo):
        self.current_dmin = msg.threshold
        self.current_front_dist = msg.front_distance
        self.current_rho = msg.density_rho
        self.current_zone = msg.zone

    def on_odom(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = (vx**2 + vy**2)**0.5

    def on_cmd_vel(self, msg: Twist):
        self.current_cmd = msg.linear.x

    def _init_plots(self):
        # 1. Gauge Bar
        self.ax_gauge.set_xlim(0.0, self.max_speed)
        self.ax_gauge.set_ylim(-0.5, 0.5)
        self.ax_gauge.set_yticks([])
        self.ax_gauge.set_xlabel('Speed (m/s)', fontsize=9)
        self.bar = self.ax_gauge.barh(0, 0, height=0.6, color='green', alpha=0.85)
        self.gauge_text = self.ax_gauge.text(0.01, 0.0, '', va='center', ha='left', fontweight='bold', fontsize=10)

        # 2. Rolling Plot
        self.ax_plot.set_ylabel('Speed (m/s)', fontsize=9)
        self.ax_plot.set_xlabel('Time (s)', fontsize=9)
        self.ax_plot.set_ylim(-0.05, self.max_speed + 0.05)
        self.ax_plot.grid(True, linestyle='--', alpha=0.5)

        self.line_actual, = self.ax_plot.plot([], [], label='Actual Speed', color='#2ca02c', lw=2.2)
        self.line_cmd, = self.ax_plot.plot([], [], label='Commanded Speed', color='#1f77b4', lw=1.6, ls='--')
        self.line_limit, = self.ax_plot.plot([], [], label='Safe Speed Limit', color='#d62728', lw=1.8, ls=':')
        self.ax_plot.legend(loc='upper right', fontsize=8)

        # 3. Status Info Box
        self.ax_info.axis('off')
        self.info_text = self.ax_info.text(
            0.05, 0.5, '', va='center', ha='left', fontfamily='monospace', fontsize=9,
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#f0f0f0', edgecolor='#cccccc')
        )

        self.fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    def update_gui(self):
        if not self.has_display:
            return

        now = time.time()
        t = now - self.start_time

        # Append rolling data
        self.timestamps.append(t)
        self.actual_speeds.append(self.current_speed)
        self.commanded_speeds.append(self.current_cmd)
        self.speed_limits.append(self.current_limit)
        self.zones.append(self.current_zone)

        # Prune old samples
        while self.timestamps and (t - self.timestamps[0]) > self.window_sec:
            self.timestamps.popleft()
            self.actual_speeds.popleft()
            self.commanded_speeds.popleft()
            self.speed_limits.popleft()
            self.zones.popleft()

        # Update Gauge
        v = self.current_speed
        bar_color = '#2ca02c' if v >= 0.28 else ('#ffbb78' if v >= 0.18 else ('#ff7f0e' if v >= 0.06 else '#d62728'))
        self.bar[0].set_width(min(self.max_speed, v))
        self.bar[0].set_color(bar_color)
        self.gauge_text.set_text(f" Actual: {v:.2f} m/s  |  Limit: {self.current_limit:.2f} m/s")

        # Update Rolling Plot
        ts = list(self.timestamps)
        self.line_actual.set_data(ts, list(self.actual_speeds))
        self.line_cmd.set_data(ts, list(self.commanded_speeds))
        self.line_limit.set_data(ts, list(self.speed_limits))

        min_t = ts[0] if ts else 0
        max_t = max(ts[-1] if ts else 10.0, min_t + 5.0)
        self.ax_plot.set_xlim(min_t, max_t)

        # Update Info Box
        info_str = (
            f" ZONE: {self.current_zone:<12} | d_min (safety dist): {self.current_dmin:4.2f} m\n"
            f" Front Dist: {self.current_front_dist:4.2f} m | Density (rho)      : {self.current_rho:4.2f}\n"
            f" V_actual  : {self.current_speed:4.2f} m/s | V_commanded        : {self.current_cmd:4.2f} m/s"
        )
        self.info_text.set_text(info_str)

        try:
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SpeedDashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

