import math
import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from adaptive_speed_interfaces.msg import ObstacleInfo, RobotStatus
from adaptive_speed_control.utils.geometry_utils import euclidean_distance


class OdometryMonitorNode(Node):
    """
    Monitors robot motion, odometry, and safety state.
    - Tracks actual speed, cumulative distance travelled, distance to goal, and ETA.
    - Publishes comprehensive live telemetry to /robot_status.
    - Publishes high-visibility RViz markers:
      * Speedometer text floating above robot (/visualization_speedometer)
      * Color-coded speed trajectory trail with legend (/visualization_trajectory)
      * Zone safety ring flat on ground (/visualization_zone_ring)
      * Combined marker array (/robot_visual_markers)
    - Prints an ANSI color-coded 1-2 Hz terminal HUD.
    """

    def __init__(self):
        super().__init__('odometry_monitor_node')

        self.declare_parameter('goal_x', 9.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('speed_limit', 0.40)
        self.declare_parameter('log_rate', 1.5)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.speed_limit = float(self.get_parameter('speed_limit').value)
        self.log_rate = float(self.get_parameter('log_rate').value)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.obs_sub = self.create_subscription(ObstacleInfo, '/obstacle_info', self.on_obstacle_info, 10)

        # Telemetry & Marker Publishers
        self.status_pub = self.create_publisher(RobotStatus, '/robot_status', 10)
        self.speedometer_pub = self.create_publisher(Marker, '/visualization_speedometer', 10)
        self.trajectory_pub = self.create_publisher(Marker, '/visualization_trajectory', 10)
        self.zone_ring_pub = self.create_publisher(Marker, '/visualization_zone_ring', 10)
        self.legend_pub = self.create_publisher(Marker, '/visualization_legend', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/robot_visual_markers', 10)

        # Odometry tracking
        self.has_odom = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.actual_speed = 0.0
        self.distance_travelled = 0.0
        self.commanded_speed = 0.0

        # State tracking
        self.current_zone = "FREE"
        self.current_state = "STARTING"
        self.front_dist = float('inf')
        self.density = 0.0

        # Rolling speed window for smooth ETA calculation
        self.speed_history = []
        self.max_history = 10

        # Path history for color-coded trajectory marker
        self.path_points: List[Point] = []
        self.path_colors: List[ColorRGBA] = []
        self.last_path_x = 0.0
        self.last_path_y = 0.0

        timer_period = 1.0 / max(0.5, self.log_rate)
        self.timer = self.create_timer(timer_period, self.monitor_loop)

        self.get_logger().info("Odometry Monitor Node initialized with high-visibility speed displays.")

    def on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.actual_speed = math.hypot(vx, vy)

        if not self.has_odom:
            self.last_x = x
            self.last_y = y
            self.last_path_x = x
            self.last_path_y = y
            self.has_odom = True

        dx = x - self.last_x
        dy = y - self.last_y
        step_dist = math.hypot(dx, dy)
        if step_dist < 0.5:  # Filter discontinuous jumps
            self.distance_travelled += step_dist

        self.last_x = x
        self.last_y = y
        self.current_x = x
        self.current_y = y

        self.speed_history.append(self.actual_speed)
        if len(self.speed_history) > self.max_history:
            self.speed_history.pop(0)

        # Accumulate path point every 0.08m for trajectory marker
        if math.hypot(x - self.last_path_x, y - self.last_path_y) >= 0.08:
            pt = Point(x=float(x), y=float(y), z=0.04)
            col = self._speed_to_rgba(self.actual_speed)
            self.path_points.append(pt)
            self.path_colors.append(col)
            self.last_path_x = x
            self.last_path_y = y

    def on_cmd_vel(self, msg: Twist):
        self.commanded_speed = msg.linear.x

    def on_obstacle_info(self, msg: ObstacleInfo):
        self.current_zone = msg.zone
        self.front_dist = msg.front_distance
        self.density = msg.density_rho

    def _speed_to_rgba(self, speed: float) -> ColorRGBA:
        """Map velocity to RGBA: Green (fast), Yellow (moderate), Orange (slow), Red (stopped)."""
        c = ColorRGBA()
        c.a = 0.90
        if speed >= 0.28:
            c.r, c.g, c.b = 0.10, 0.90, 0.10  # Fast: Green
        elif speed >= 0.18:
            c.r, c.g, c.b = 0.95, 0.88, 0.05  # Moderate: Yellow
        elif speed >= 0.05:
            c.r, c.g, c.b = 1.00, 0.50, 0.00  # Slow: Orange
        else:
            c.r, c.g, c.b = 0.95, 0.10, 0.10  # Stopped: Red
        return c

    def _speed_to_ansi(self, speed: float) -> str:
        """ANSI escape color string for terminal."""
        if speed >= 0.28:
            return "\033[92m"  # Green
        elif speed >= 0.18:
            return "\033[93m"  # Yellow
        elif speed >= 0.05:
            return "\033[38;5;208m"  # Orange
        else:
            return "\033[91m"  # Red

    def _zone_to_rgba(self, zone: str) -> ColorRGBA:
        c = ColorRGBA()
        c.a = 0.45
        z = zone.upper()
        if 'FREE' in z:
            c.r, c.g, c.b = 0.10, 0.85, 0.20
        elif 'MODERATE' in z:
            c.r, c.g, c.b = 0.95, 0.85, 0.05
        elif 'DENSE' in z:
            c.r, c.g, c.b = 1.00, 0.45, 0.00
        else:
            c.r, c.g, c.b = 0.95, 0.05, 0.05
        return c

    def _zone_to_ansi(self, zone: str) -> str:
        z = zone.upper()
        if 'FREE' in z:
            return "\033[92m"
        elif 'MODERATE' in z:
            return "\033[93m"
        elif 'DENSE' in z:
            return "\033[38;5;208m"
        else:
            return "\033[91m"

    def publish_rviz_markers(self):
        stamp = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        # 1. RViz Speedometer Text Marker above robot
        speed_col = self._speed_to_rgba(self.actual_speed)
        marker_speed = Marker()
        marker_speed.header.frame_id = 'odom'
        marker_speed.header.stamp = stamp
        marker_speed.ns = 'speedometer'
        marker_speed.id = 0
        marker_speed.type = Marker.TEXT_VIEW_FACING
        marker_speed.action = Marker.ADD
        marker_speed.pose.position.x = float(self.current_x)
        marker_speed.pose.position.y = float(self.current_y)
        marker_speed.pose.position.z = 0.85
        marker_speed.scale.z = 0.28
        marker_speed.color = speed_col
        marker_speed.color.a = 1.0
        marker_speed.text = f"speed {self.actual_speed:.2f} m/s | limit {self.speed_limit:.2f} | zone {self.current_zone}"
        self.speedometer_pub.publish(marker_speed)
        marker_array.markers.append(marker_speed)

        # 2. Zone Safety Ring flat on ground under robot
        zone_col = self._zone_to_rgba(self.current_zone)
        marker_ring = Marker()
        marker_ring.header.frame_id = 'odom'
        marker_ring.header.stamp = stamp
        marker_ring.ns = 'zone_ring'
        marker_ring.id = 1
        marker_ring.type = Marker.CYLINDER
        marker_ring.action = Marker.ADD
        marker_ring.pose.position.x = float(self.current_x)
        marker_ring.pose.position.y = float(self.current_y)
        marker_ring.pose.position.z = 0.015
        marker_ring.scale.x = 1.10
        marker_ring.scale.y = 1.10
        marker_ring.scale.z = 0.02
        marker_ring.color = zone_col
        self.zone_ring_pub.publish(marker_ring)
        marker_array.markers.append(marker_ring)

        # 3. Colour-Coded Trajectory Trail
        if len(self.path_points) >= 2:
            marker_traj = Marker()
            marker_traj.header.frame_id = 'odom'
            marker_traj.header.stamp = stamp
            marker_traj.ns = 'trajectory'
            marker_traj.id = 2
            marker_traj.type = Marker.LINE_STRIP
            marker_traj.action = Marker.ADD
            marker_traj.scale.x = 0.08  # line thickness
            marker_traj.points = list(self.path_points)
            marker_traj.colors = list(self.path_colors)
            self.trajectory_pub.publish(marker_traj)
            marker_array.markers.append(marker_traj)

        # 4. Trajectory Legend Marker
        marker_legend = Marker()
        marker_legend.header.frame_id = 'odom'
        marker_legend.header.stamp = stamp
        marker_legend.ns = 'legend'
        marker_legend.id = 3
        marker_legend.type = Marker.TEXT_VIEW_FACING
        marker_legend.action = Marker.ADD
        marker_legend.pose.position.x = float(self.current_x)
        marker_legend.pose.position.y = float(self.current_y)
        marker_legend.pose.position.z = 1.15
        marker_legend.scale.z = 0.16
        marker_legend.color = ColorRGBA(r=0.9, g=0.9, b=0.9, a=0.85)
        marker_legend.text = "[TRAIL: Green>=0.28 | Yellow>=0.18 | Orange>=0.05 | Red<0.05 m/s]"
        self.legend_pub.publish(marker_legend)
        marker_array.markers.append(marker_legend)

        self.markers_pub.publish(marker_array)

    def monitor_loop(self):
        if not self.has_odom:
            return

        dist_to_goal = euclidean_distance(self.current_x, self.current_y, self.goal_x, self.goal_y)

        # ETA estimation
        avg_speed = (sum(self.speed_history) / len(self.speed_history)) if self.speed_history else 0.0
        if avg_speed > 0.05:
            eta = dist_to_goal / avg_speed
        else:
            eta = 999.0

        # Publish RobotStatus
        status = RobotStatus()
        status.speed = float(self.actual_speed)
        status.commanded_speed = float(self.commanded_speed)
        status.speed_limit = float(self.speed_limit)
        status.zone = str(self.current_zone)
        status.state = "ACTIVE" if self.actual_speed > 0.02 else "HALTED"
        status.distance_to_goal = float(dist_to_goal)
        status.distance_travelled = float(self.distance_travelled)
        status.eta = float(min(999.0, eta))
        self.status_pub.publish(status)

        # Publish RViz visual markers
        self.publish_rviz_markers()

        # Terminal HUD with ANSI Colors
        v_col = self._speed_to_ansi(self.actual_speed)
        z_col = self._zone_to_ansi(self.current_zone)
        reset = "\033[0m"
        bold = "\033[1m"
        eta_str = f"{eta:4.1f}s" if eta < 900.0 else "INF"

        self.get_logger().info(
            f"{bold}[HUD]{reset} Pos: ({self.current_x:5.2f}, {self.current_y:5.2f}) | "
            f"{v_col}V_act: {self.actual_speed:4.2f} m/s{reset} | V_cmd: {self.commanded_speed:4.2f} m/s | "
            f"{z_col}Zone: {self.current_zone:<12}{reset} | Dist: {dist_to_goal:4.2f}m | Trav: {self.distance_travelled:4.2f}m | ETA: {eta_str}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdometryMonitorNode()
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
