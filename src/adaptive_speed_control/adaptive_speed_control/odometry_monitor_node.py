import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from adaptive_speed_interfaces.msg import ObstacleInfo, RobotStatus
from adaptive_speed_control.utils.geometry_utils import euclidean_distance


class OdometryMonitorNode(Node):
    """
    Monitors robot motion, odometry, and safety state.
    - Tracks actual speed, cumulative distance travelled, distance to goal, and ETA.
    - Publishes comprehensive live telemetry to /robot_status.
    - Prints a clean 1 Hz terminal HUD.
    """

    def __init__(self):
        super().__init__('odometry_monitor_node')

        self.declare_parameter('goal_x', 9.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('speed_limit', 0.40)
        self.declare_parameter('log_rate', 1.0)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.speed_limit = float(self.get_parameter('speed_limit').value)
        self.log_rate = float(self.get_parameter('log_rate').value)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.obs_sub = self.create_subscription(ObstacleInfo, '/obstacle_info', self.on_obstacle_info, 10)

        # Publisher
        self.status_pub = self.create_publisher(RobotStatus, '/robot_status', 10)

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
        self.current_zone = "UNKNOWN"
        self.current_state = "STARTING"
        self.front_dist = float('inf')
        self.density = 0.0

        # Rolling speed window for smooth ETA calculation
        self.speed_history = []
        self.max_history = 10

        timer_period = 1.0 / max(0.5, self.log_rate)
        self.timer = self.create_timer(timer_period, self.monitor_loop)

        self.get_logger().info("Odometry Monitor Node initialized. Publishing /robot_status.")

    def on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.actual_speed = math.hypot(vx, vy)

        if not self.has_odom:
            self.last_x = x
            self.last_y = y
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

    def on_cmd_vel(self, msg: Twist):
        self.commanded_speed = msg.linear.x

    def on_obstacle_info(self, msg: ObstacleInfo):
        self.current_zone = msg.zone
        self.front_dist = msg.front_distance
        self.density = msg.density_rho

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

        # One-line HUD
        eta_str = f"{eta:4.1f}s" if eta < 900.0 else "INF"
        self.get_logger().info(
            f"[HUD] Pos: ({self.current_x:5.2f}, {self.current_y:5.2f}) | "
            f"V_act: {self.actual_speed:4.2f} m/s | V_cmd: {self.commanded_speed:4.2f} m/s | "
            f"Zone: {self.current_zone:<16} | Dist: {dist_to_goal:4.2f}m | Trav: {self.distance_travelled:4.2f}m | ETA: {eta_str}"
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

