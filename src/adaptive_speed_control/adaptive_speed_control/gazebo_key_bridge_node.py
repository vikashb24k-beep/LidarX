#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Bool
from geometry_msgs.msg import Twist


class GazeboKeyBridgeNode(Node):
    """
    Bridges Gazebo Sim KeyPublisher GUI events (/keyboard/keypress)
    to twist_mux's teleoperation input (/cmd_vel_teleop).
    Allows driving when the Gazebo 3D simulation window has focus.
    """

    def __init__(self):
        super().__init__('gazebo_key_bridge_node')

        self.declare_parameter('key_timeout', 0.35)
        self.declare_parameter('speed_step', 0.05)
        self.declare_parameter('default_speed_limit', 0.40)

        self.key_timeout = float(self.get_parameter('key_timeout').value)
        self.speed_step = float(self.get_parameter('speed_step').value)
        self.speed_limit = float(self.get_parameter('default_speed_limit').value)

        # Subscribers
        self.key_sub = self.create_subscription(Int32, '/keyboard/keypress', self.on_keypress, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.mode_pub = self.create_publisher(String, '/control_mode', 10)
        self.adaptive_pub = self.create_publisher(Bool, '/toggle_adaptive', 10)

        # State
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_key_time = 0.0
        self.control_mode = "MANUAL"
        self.adaptive_enabled = True

        self.timer = self.create_timer(0.05, self.publish_loop)  # 20 Hz
        self.get_logger().info("Gazebo Key Bridge initialized. Listening on /keyboard/keypress...")

    def on_keypress(self, msg: Int32):
        key_code = msg.data
        now = time.time()

        # Qt Key Codes
        # W=87, S=83, A=65, D=68, Q=81, E=69, Z=90, C=67, Space=32, X=88, T=84, M=77, R=82
        if key_code in (87, 119):  # 'w' or 'W'
            self.target_linear = min(self.speed_limit, max(0.10, self.target_linear + self.speed_step))
            self.last_key_time = now
        elif key_code in (83, 115):  # 's' or 'S'
            self.target_linear = max(-self.speed_limit, min(-0.10, self.target_linear - self.speed_step))
            self.last_key_time = now
        elif key_code in (65, 97):  # 'a' or 'A'
            self.target_angular = min(1.0, max(0.30, self.target_angular + 0.25))
            self.last_key_time = now
        elif key_code in (68, 100):  # 'd' or 'D'
            self.target_angular = max(-1.0, min(-0.30, self.target_angular - 0.25))
            self.last_key_time = now
        elif key_code in (81, 113):  # 'q' or 'Q'
            self.target_linear = min(self.speed_limit, max(0.10, self.target_linear + self.speed_step))
            self.target_angular = min(1.0, max(0.30, self.target_angular + 0.25))
            self.last_key_time = now
        elif key_code in (69, 101):  # 'e' or 'E'
            self.target_linear = min(self.speed_limit, max(0.10, self.target_linear + self.speed_step))
            self.target_angular = max(-1.0, min(-0.30, self.target_angular - 0.25))
            self.last_key_time = now
        elif key_code in (90, 122):  # 'z' or 'Z'
            self.target_linear = max(-self.speed_limit, min(-0.10, self.target_linear - self.speed_step))
            self.target_angular = max(-1.0, min(-0.30, self.target_angular - 0.25))
            self.last_key_time = now
        elif key_code in (67, 99):  # 'c' or 'C'
            self.target_linear = max(-self.speed_limit, min(-0.10, self.target_linear - self.speed_step))
            self.target_angular = min(1.0, max(0.30, self.target_angular + 0.25))
            self.last_key_time = now
        elif key_code in (32, 88, 120):  # Space or 'x'
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.last_key_time = now
        elif key_code in (84, 116):  # 't'
            self.control_mode = "AUTONOMOUS" if self.control_mode == "MANUAL" else "MANUAL"
            self.mode_pub.publish(String(data=self.control_mode))
            self.get_logger().info(f"Gazebo Key: Control Mode toggled to {self.control_mode}")
        elif key_code in (77, 109):  # 'm'
            self.adaptive_enabled = not self.adaptive_enabled
            self.adaptive_pub.publish(Bool(data=self.adaptive_enabled))
            self.get_logger().info(f"Gazebo Key: Adaptive toggled to {self.adaptive_enabled}")
        elif key_code in (82, 114):  # 'r'
            self.speed_limit = float(self.get_parameter('default_speed_limit').value)

    def publish_loop(self):
        if self.control_mode != "MANUAL":
            return

        now = time.time()
        if (now - self.last_key_time) > self.key_timeout:
            self.target_linear = 0.0
            self.target_angular = 0.0

        msg = Twist()
        msg.linear.x = float(self.target_linear)
        msg.angular.z = float(self.target_angular)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboKeyBridgeNode()
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
