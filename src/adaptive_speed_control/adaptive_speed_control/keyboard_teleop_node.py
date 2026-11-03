#!/usr/bin/env python3
import sys
import os
import time
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

# Try importing termios and tty for raw terminal key reading
try:
    import termios
    import tty
    TERMIOS_AVAILABLE = True
except ImportError:
    TERMIOS_AVAILABLE = False


HELP_TEXT = """
=============================================================================
                    KEYBOARD TELEOP CONTROLLER (20 Hz)
=============================================================================
 Motion Controls (Hold-to-move, expires after 0.3s):
   w / s        : Increase / Decrease forward speed request
   a / d        : Turn left / Turn right
   q / e        : Forward-Left / Forward-Right
   z / c        : Reverse-Left / Reverse-Right
   , / .        : Reverse step / Forward step nudge
   x / Space    : Zero velocity request (Soft stop)

 Speed & Configuration:
   + / -        : Raise / Lower max requested speed limit (+/- 0.05 m/s)
   r            : Reset speed limit to default (0.40 m/s)
   t            : Toggle MANUAL / AUTONOMOUS navigation mode
   m            : Toggle Adaptive Speed Scaling ON / OFF
                  *SAFETY INVARIANT: EMERGENCY STOP & TIMEOUTS STAY ALWAYS ACTIVE*
   h            : Display this help table
   Ctrl-C / Esc : Quit teleop safely with zero velocity
=============================================================================
"""


class KeyboardTeleopNode(Node):
    """
    Real-time interactive keyboard teleoperation node for mobile robots.
    - Publishes desired velocities strictly to /cmd_vel_teleop at 20 Hz.
    - Hold-to-move semantics: commands expire after key_timeout seconds.
    - Mode switching (MANUAL / AUTONOMOUS) published to /control_mode.
    - Adaptive scaling toggle published to /toggle_adaptive.
    """

    def __init__(self):
        super().__init__('keyboard_teleop_node')

        self.declare_parameter('key_timeout', 0.30)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('linear_step', 0.05)
        self.declare_parameter('angular_step', 0.25)
        self.declare_parameter('default_speed_limit', 0.40)
        self.declare_parameter('max_allowed_speed', 1.00)
        self.declare_parameter('min_allowed_speed', 0.05)

        self.key_timeout = float(self.get_parameter('key_timeout').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.linear_step = float(self.get_parameter('linear_step').value)
        self.angular_step = float(self.get_parameter('angular_step').value)
        self.default_speed_limit = float(self.get_parameter('default_speed_limit').value)
        self.max_allowed_speed = float(self.get_parameter('max_allowed_speed').value)
        self.min_allowed_speed = float(self.get_parameter('min_allowed_speed').value)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.mode_pub = self.create_publisher(String, '/control_mode', 10)
        self.adaptive_pub = self.create_publisher(Bool, '/toggle_adaptive', 10)

        # Teleop State
        self.speed_limit = self.default_speed_limit
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_key_time = 0.0
        self.control_mode = "MANUAL"
        self.adaptive_enabled = True

        # Timer for 20 Hz publish loop
        timer_period = 1.0 / max(1.0, self.publish_rate)
        self.timer = self.create_timer(timer_period, self.publish_loop)

        # Initial mode announcement
        self._publish_mode()

    def _publish_mode(self):
        msg = String()
        msg.data = self.control_mode
        self.mode_pub.publish(msg)

    def _publish_adaptive(self):
        msg = Bool()
        msg.data = self.adaptive_enabled
        self.adaptive_pub.publish(msg)

    def process_key(self, key: str) -> bool:
        """
        Processes a key press.
        Returns: True to continue running, False to terminate.
        """
        now = time.time()

        if key in ('\x03', '\x1b'):  # Ctrl-C or Esc
            return False

        if key == 'h':
            self._print_raw(HELP_TEXT)
            return True

        if key == 't':
            # Toggle between MANUAL and AUTONOMOUS
            if self.control_mode == "MANUAL":
                self.control_mode = "AUTONOMOUS"
            else:
                self.control_mode = "MANUAL"
            # Stop teleop motion on switch
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.cmd_pub.publish(Twist())
            self._publish_mode()
            self._print_raw(f"\r\n>>> MODE TOGGLED: {self.control_mode}\r\n")
            return True

        if key == 'm':
            # Toggle adaptive speed scaling ON/OFF
            self.adaptive_enabled = not self.adaptive_enabled
            self._publish_adaptive()
            status_str = "ON" if self.adaptive_enabled else "OFF (emergency stop still on)"
            self._print_raw(f"\r\n>>> ADAPTIVE SCALING: {status_str}\r\n")
            return True

        if key in ('+', '='):
            self.speed_limit = min(
                self.max_allowed_speed,
                round(self.speed_limit + self.linear_step, 2)
            )
            self._print_raw(f"\r\n>>> Speed Limit: {self.speed_limit:.2f} m/s\r\n")
            return True

        if key in ('-', '_'):
            self.speed_limit = max(
                self.min_allowed_speed,
                round(self.speed_limit - self.linear_step, 2)
            )
            self._print_raw(f"\r\n>>> Speed Limit: {self.speed_limit:.2f} m/s\r\n")
            return True

        if key == 'r':
            self.speed_limit = self.default_speed_limit
            self._print_raw(f"\r\n>>> Speed Limit Reset: {self.speed_limit:.2f} m/s\r\n")
            return True

        if key in ('x', ' '):
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.last_key_time = now
            return True

        # Movement keys (only take effect if in MANUAL mode)
        if self.control_mode != "MANUAL":
            return True

        if key == 'w':
            self.target_linear = min(self.speed_limit, max(0.10, self.target_linear + self.linear_step))
            self.last_key_time = now
        elif key == 's':
            self.target_linear = max(-self.speed_limit, min(-0.10, self.target_linear - self.linear_step))
            self.last_key_time = now
        elif key == 'a':
            self.target_angular = min(1.0, max(0.30, self.target_angular + self.angular_step))
            self.last_key_time = now
        elif key == 'd':
            self.target_angular = max(-1.0, min(-0.30, self.target_angular - self.angular_step))
            self.last_key_time = now
        elif key == 'q':
            self.target_linear = min(self.speed_limit, max(0.10, self.target_linear + self.linear_step))
            self.target_angular = min(1.0, max(0.30, self.target_angular + self.angular_step))
            self.last_key_time = now
        elif key == 'e':
            self.target_linear = min(self.speed_limit, max(0.10, self.target_linear + self.linear_step))
            self.target_angular = max(-1.0, min(-0.30, self.target_angular - self.angular_step))
            self.last_key_time = now
        elif key == 'z':
            self.target_linear = max(-self.speed_limit, min(-0.10, self.target_linear - self.linear_step))
            self.target_angular = max(-1.0, min(-0.30, self.target_angular - self.angular_step))
            self.last_key_time = now
        elif key == 'c':
            self.target_linear = max(-self.speed_limit, min(-0.10, self.target_linear - self.linear_step))
            self.target_angular = min(1.0, max(0.30, self.target_angular + self.angular_step))
            self.last_key_time = now
        elif key == '.':
            self.target_linear = min(self.speed_limit, 0.15)
            self.target_angular = 0.0
            self.last_key_time = now
        elif key == ',':
            self.target_linear = max(-self.speed_limit, -0.15)
            self.target_angular = 0.0
            self.last_key_time = now

        return True

    def publish_loop(self):
        """20 Hz publishing loop with hold-to-move timeout check."""
        if self.control_mode != "MANUAL":
            return

        now = time.time()
        # Hold-to-move timeout expiration
        if (now - self.last_key_time) > self.key_timeout:
            self.target_linear = 0.0
            self.target_angular = 0.0

        msg = Twist()
        msg.linear.x = float(self.target_linear)
        msg.angular.z = float(self.target_angular)
        self.cmd_pub.publish(msg)

    def _print_raw(self, msg: str):
        formatted = msg.replace('\n', '\r\n')
        sys.stdout.write(formatted)
        sys.stdout.flush()


def read_single_key(timeout: float = 0.05) -> str:
    """Non-blocking single character read from stdin."""
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            # Check for multi-byte escape sequences like arrow keys
            rlist2, _, _ = select.select([sys.stdin], [], [], 0.02)
            if rlist2:
                ch2 = sys.stdin.read(1)
                if ch2 == '[':
                    ch3 = sys.stdin.read(1)
                    if ch3 == 'A': return 'w'
                    if ch3 == 'B': return 's'
                    if ch3 == 'C': return 'd'
                    if ch3 == 'D': return 'a'
            return '\x1b'
        return ch
    return ''


def main(args=None):
    if not sys.stdin.isatty():
        sys.stderr.write("ERROR: keyboard_teleop_node requires an interactive TTY terminal.\n")
        sys.stderr.write("Run it in an interactive shell: ros2 run adaptive_speed_control keyboard_teleop_node\n")
        sys.exit(1)

    if not TERMIOS_AVAILABLE:
        sys.stderr.write("ERROR: termios is required for raw keyboard reading.\n")
        sys.exit(1)

    rclpy.init(args=args)
    node = KeyboardTeleopNode()

    # Save original terminal settings
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())

    node._print_raw(HELP_TEXT)
    node._print_raw(
        f"\r\nReady to drive! Mode: {node.control_mode} | Limit: {node.speed_limit:.2f} m/s\r\n"
        f"Hold keys [w, a, s, d] to drive. Press 'h' for help, 'Ctrl-C' or 'Esc' to exit.\r\n"
    )

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            key = read_single_key(timeout=0.03)
            if key:
                running = node.process_key(key)
                if not running:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        # Strictly ensure terminal settings are restored and zero speed sent
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        except Exception:
            pass

        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        print("\r\n[INFO] Keyboard teleop cleanly terminated. Terminal restored.")


if __name__ == '__main__':
    main()
