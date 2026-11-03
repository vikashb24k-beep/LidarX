import time
import pytest
import rclpy
from geometry_msgs.msg import Twist
from adaptive_speed_control.keyboard_teleop_node import KeyboardTeleopNode


@pytest.fixture
def rclpy_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_teleop_init(rclpy_init):
    node = KeyboardTeleopNode()
    assert node.control_mode == "MANUAL"
    assert node.speed_limit == 0.40
    assert node.adaptive_enabled is True
    node.destroy_node()


def test_teleop_key_mappings(rclpy_init):
    node = KeyboardTeleopNode()

    # w increases forward speed
    node.process_key('w')
    assert node.target_linear > 0.0

    # x zeroes request
    node.process_key('x')
    assert node.target_linear == 0.0
    assert node.target_angular == 0.0

    # s decreases forward speed (negative)
    node.process_key('s')
    assert node.target_linear < 0.0

    # space zeroes request
    node.process_key(' ')
    assert node.target_linear == 0.0

    # a turns left (positive angular z)
    node.process_key('a')
    assert node.target_angular > 0.0

    # d turns right (negative angular z)
    node.process_key('d')
    assert node.target_angular < 0.0

    # q: forward left
    node.process_key('x')
    node.process_key('q')
    assert node.target_linear > 0.0
    assert node.target_angular > 0.0

    # e: forward right
    node.process_key('x')
    node.process_key('e')
    assert node.target_linear > 0.0
    assert node.target_angular < 0.0

    # z: reverse left
    node.process_key('x')
    node.process_key('z')
    assert node.target_linear < 0.0
    assert node.target_angular < 0.0

    # c: reverse right
    node.process_key('x')
    node.process_key('c')
    assert node.target_linear < 0.0
    assert node.target_angular > 0.0

    # nudge steps . and ,
    node.process_key('.')
    assert node.target_linear > 0.0
    node.process_key(',')
    assert node.target_linear < 0.0

    node.destroy_node()


def test_teleop_speed_limit_adjustments(rclpy_init):
    node = KeyboardTeleopNode()
    orig_limit = node.speed_limit

    # Increase speed limit with +
    node.process_key('+')
    assert round(node.speed_limit, 2) == round(orig_limit + 0.05, 2)

    # Decrease speed limit with -
    node.process_key('-')
    assert round(node.speed_limit, 2) == round(orig_limit, 2)

    # Reset speed limit with r
    node.speed_limit = 0.80
    node.process_key('r')
    assert round(node.speed_limit, 2) == round(orig_limit, 2)

    # Clamp bounds: cannot exceed max_allowed_speed
    for _ in range(30):
        node.process_key('+')
    assert node.speed_limit <= node.max_allowed_speed

    # Clamp bounds: cannot fall below min_allowed_speed
    for _ in range(30):
        node.process_key('-')
    assert node.speed_limit >= node.min_allowed_speed

    node.destroy_node()


def test_teleop_mode_toggle(rclpy_init):
    node = KeyboardTeleopNode()
    assert node.control_mode == "MANUAL"

    # Set some velocity first
    node.process_key('w')
    assert node.target_linear > 0.0

    # Toggle to AUTONOMOUS
    node.process_key('t')
    assert node.control_mode == "AUTONOMOUS"
    # Transition must zero out velocity commands
    assert node.target_linear == 0.0
    assert node.target_angular == 0.0

    # While in AUTONOMOUS, manual movement keys must be ignored
    node.process_key('w')
    assert node.target_linear == 0.0

    # Toggle back to MANUAL
    node.process_key('t')
    assert node.control_mode == "MANUAL"
    node.process_key('w')
    assert node.target_linear > 0.0

    node.destroy_node()


def test_teleop_adaptive_toggle(rclpy_init):
    node = KeyboardTeleopNode()
    assert node.adaptive_enabled is True

    # Press 'm' to toggle
    node.process_key('m')
    assert node.adaptive_enabled is False

    # Press 'm' to toggle back
    node.process_key('m')
    assert node.adaptive_enabled is True

    node.destroy_node()


def test_teleop_hold_to_move_timeout(rclpy_init):
    node = KeyboardTeleopNode()
    node.key_timeout = 0.05  # Short timeout for fast testing

    node.process_key('w')
    assert node.target_linear > 0.0

    # Wait past timeout
    time.sleep(0.08)
    node.publish_loop()

    # Must automatically expire and drop to zero
    assert node.target_linear == 0.0
    assert node.target_angular == 0.0

    node.destroy_node()


def test_teleop_quit_keys(rclpy_init):
    node = KeyboardTeleopNode()
    # Ctrl-C ('\x03') returns False (stop running)
    assert node.process_key('\x03') is False
    # Esc ('\x1b') returns False
    assert node.process_key('\x1b') is False
    # Regular key returns True
    assert node.process_key('w') is True
    node.destroy_node()
