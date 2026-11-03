import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from adaptive_speed_interfaces.msg import ObstacleInfo, RobotStatus
from adaptive_speed_control.utils.geometry_utils import quaternion_to_yaw


class MetricsLoggerNode(Node):
    """
    Subscribes to odometry, commanded velocity, obstacle telemetry, and status,
    logging time-series trajectory and safety metrics to CSV for benchmarking.
    """

    def __init__(self):
        super().__init__('metrics_logger_node')

        self.declare_parameter('csv_filename', 'results/metrics_run.csv')
        self.declare_parameter('log_frequency', 10.0)

        csv_param = self.get_parameter('csv_filename').value
        # Ensure results directory exists
        os.makedirs(os.path.dirname(os.path.abspath(csv_param)), exist_ok=True)
        self.csv_path = os.path.abspath(csv_param)

        # State storage
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.actual_speed = 0.0
        self.distance_travelled = 0.0
        self.cmd_linear = 0.0
        self.req_linear = 0.0
        self.closest_dist = float('inf')
        self.front_dist = float('inf')
        self.density = 0.0
        self.zone = 'STARTING'

        self.start_time = time.time()
        self.file_handle = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.file_handle)
        self.csv_writer.writerow([
            'elapsed_time_s', 'x', 'y', 'yaw_rad',
            'v_actual', 'v_cmd', 'v_req',
            'front_dist', 'closest_dist', 'density_rho',
            'zone', 'distance_travelled'
        ])
        self.file_handle.flush()

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.cmd_in_sub = self.create_subscription(Twist, '/cmd_vel_in', self.on_cmd_vel_in, 10)
        self.obs_sub = self.create_subscription(ObstacleInfo, '/obstacle_info', self.on_obstacle_info, 10)
        self.status_sub = self.create_subscription(RobotStatus, '/robot_status', self.on_status, 10)

        # Timer
        period = 1.0 / max(1.0, float(self.get_parameter('log_frequency').value))
        self.timer = self.create_timer(period, self.log_sample)

        self.get_logger().info(f"Metrics Logger initialized. Logging metrics to: {self.csv_path}")

    def on_odom(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.actual_speed = math.hypot(vx, vy)

    def on_cmd_vel(self, msg: Twist):
        self.cmd_linear = msg.linear.x

    def on_cmd_vel_in(self, msg: Twist):
        self.req_linear = msg.linear.x

    def on_obstacle_info(self, msg: ObstacleInfo):
        self.closest_dist = msg.closest_distance
        self.front_dist = msg.front_distance
        self.density = msg.density_rho
        self.zone = msg.zone

    def on_status(self, msg: RobotStatus):
        self.distance_travelled = msg.distance_travelled

    def log_sample(self):
        elapsed = time.time() - self.start_time
        self.csv_writer.writerow([
            f"{elapsed:.3f}",
            f"{self.robot_x:.4f}",
            f"{self.robot_y:.4f}",
            f"{self.robot_yaw:.4f}",
            f"{self.actual_speed:.4f}",
            f"{self.cmd_linear:.4f}",
            f"{self.req_linear:.4f}",
            f"{self.front_dist:.4f}",
            f"{self.closest_dist:.4f}",
            f"{self.density:.4f}",
            self.zone,
            f"{self.distance_travelled:.4f}"
        ])
        self.file_handle.flush()

    def destroy_node(self):
        if self.file_handle and not self.file_handle.closed:
            self.file_handle.flush()
            self.file_handle.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLoggerNode()
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
