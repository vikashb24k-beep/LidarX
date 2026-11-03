import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from adaptive_speed_interfaces.srv import GetAverageVel


class TeleopUINode(Node):
    """
    User Interface Node for Timed Teleoperation.
    - Prompts user for: <linear_velocity> <angular_velocity> <duration_sec>
    - Continuously publishes requested velocity commands to /cmd_vel_in for the specified duration.
    - Publishes zero velocity to /cmd_vel_in once the duration expires.
    - Maintains the 5 most recent velocity commands in a rolling deque.
    - Provides the /get_average_vel service.
    """

    def __init__(self, start_input_thread: bool = True):
        super().__init__('teleop_ui_node')

        # Publisher to /cmd_vel_in (Adaptive controller subscribes here)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_in', 10)

        # Rolling history of the last 5 velocity commands (lin, ang)
        self._last_cmds = deque(maxlen=5)
        self._lock = threading.Lock()

        # Service to return average of recent velocity commands
        self._avg_srv = self.create_service(GetAverageVel, '/get_average_vel', self.handle_get_average)

        # Active command state for periodic publishing
        self._active_cmd = Twist()
        self._cmd_active = False
        self._cmd_end_time = 0.0

        # Periodic publishing timer (10 Hz)
        self._pub_timer = self.create_timer(0.1, self._publish_timer_callback)

        # Start input thread
        self._running = True
        self._thread = None
        if start_input_thread:
            self._thread = threading.Thread(target=self._input_loop, daemon=True)
            self._thread.start()

        self.get_logger().info(
            "\n"
            "===============================================================\n"
            "   Adaptive Speed Control - User Teleop Interface Ready\n"
            "===============================================================\n"
            "Commands format: <linear_x> <angular_z> <duration_seconds>\n"
            "Examples:\n"
            "   0.30 0.0 5.0   -> Forward at 0.30 m/s for 5 seconds\n"
            "   0.00 0.5 3.0   -> Rotate in-place at 0.5 rad/s for 3 seconds\n"
            "   0.20 0.2 4.0   -> Curved forward motion for 4 seconds\n"
            "\n"
            "Note: Final speed is dynamically regulated by the Adaptive Speed\n"
            "Controller based on obstacle distance, obstacle density, and safety limits.\n"
            "Type 'q' or 'exit' to quit.\n"
            "==============================================================="
        )

    def _publish_timer_callback(self):
        with self._lock:
            if not self._cmd_active:
                return

            now = time.time()
            if now < self._cmd_end_time:
                self.cmd_pub.publish(self._active_cmd)
            else:
                stop_msg = Twist()
                self.cmd_pub.publish(stop_msg)
                self._cmd_active = False
                self.get_logger().info(">> Command duration completed. Sent stop command to /cmd_vel_in.\n")

    def _input_loop(self):
        while rclpy.ok() and self._running:
            try:
                raw = input("\nEnter [lin_x ang_z duration] (or 'q' to quit): ").strip()
                if not raw:
                    continue

                if raw.lower() in ('q', 'quit', 'exit'):
                    self.get_logger().info("Exiting UI Node.")
                    self._running = False
                    self.cmd_pub.publish(Twist())
                    rclpy.shutdown()
                    return

                parts = raw.split()
                if len(parts) != 3:
                    print(" [Error] Please provide exactly 3 parameters: linear_x angular_z duration")
                    print("         Example: 0.30 0.0 5.0")
                    continue

                lin = float(parts[0])
                ang = float(parts[1])
                duration = float(parts[2])

                if duration <= 0.0:
                    print(" [Error] Duration must be positive (> 0.0 seconds).")
                    continue

                if lin > 0.40:
                    print(f" [Notice] Requested linear velocity ({lin:.2f} m/s) exceeds recommended maximum (0.40 m/s).")
                    print("          The adaptive controller will clamp and dynamically scale it based on obstacles.")

                cmd = Twist()
                cmd.linear.x = lin
                cmd.angular.z = ang

                with self._lock:
                    self._active_cmd = cmd
                    self._cmd_end_time = time.time() + duration
                    self._cmd_active = True
                    self._last_cmds.append((lin, ang))

                print(
                    f" [Dispatched] Target: Lin={lin:.2f} m/s, Ang={ang:.2f} rad/s | "
                    f"Duration: {duration:.1f} s | Target Topic: /cmd_vel_in"
                )

            except ValueError:
                print(" [Error] Invalid input. All values must be valid numbers.")
                print("         Example: 0.30 0.0 5.0")
            except (EOFError, KeyboardInterrupt, OSError):
                self._running = False
                if rclpy.ok():
                    self.cmd_pub.publish(Twist())
                    rclpy.shutdown()
                return
            except Exception as e:
                print(f" [Unexpected Error]: {e}")

    def handle_get_average(self, request, response):
        with self._lock:
            n = len(self._last_cmds)
            if n == 0:
                response.avg_linear = 0.0
                response.avg_angular = 0.0
                response.samples = 0
                return response

            response.avg_linear = float(sum(v[0] for v in self._last_cmds) / n)
            response.avg_angular = float(sum(v[1] for v in self._last_cmds) / n)
            response.samples = int(n)
            return response


def main(args=None):
    rclpy.init(args=args)
    node = TeleopUINode()
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
