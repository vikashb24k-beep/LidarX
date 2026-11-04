import time
import pytest
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from adaptive_speed_interfaces.msg import ObstacleInfo
from adaptive_speed_control.adaptive_speed_controller_node import AdaptiveSpeedControllerNode


@pytest.fixture
def rclpy_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def create_fake_scan(front_dist=5.0, angle_min=-1.57, angle_max=1.57, num_readings=180):
    scan = LaserScan()
    scan.header.stamp = rclpy.clock.Clock().now().to_msg()
    scan.header.frame_id = 'base_link'
    scan.angle_min = angle_min
    scan.angle_max = angle_max
    scan.angle_increment = (angle_max - angle_min) / float(num_readings)
    scan.range_min = 0.05
    scan.range_max = 30.0

    ranges = []
    for i in range(num_readings):
        angle = angle_min + i * scan.angle_increment
        # Front sector: -30 deg to +30 deg
        if -0.52 <= angle <= 0.52:
            ranges.append(front_dist)
        else:
            ranges.append(8.0)
    scan.ranges = ranges
    return scan


def test_controller_sole_publisher_forwarding(rclpy_init):
    node = AdaptiveSpeedControllerNode()

    # Provide clear scan
    scan = create_fake_scan(front_dist=5.0)
    node.on_scan(scan)

    # Send requested velocity
    cmd_in = Twist()
    cmd_in.linear.x = 0.30
    node.on_cmd_vel_in(cmd_in)

    # First cycle: accelerating from 0.0 -> ramping
    node.control_loop()
    assert node.last_cmd_linear > 0.0
    assert node.limit_reason == "ACCEL_LIMIT (ramping)"

    # Fast-forward acceleration ramp
    node.last_cmd_linear = 0.30
    node.control_loop()
    assert node.last_cmd_linear == 0.30
    assert node.safe_speed_limit == 0.30
    assert node.limit_reason == "CLEAR"
    node.destroy_node()


def test_controller_slowdown_and_limit_reason(rclpy_init):
    node = AdaptiveSpeedControllerNode()

    # Provide obstacle ahead at 0.80 m
    scan = create_fake_scan(front_dist=0.80)
    node.on_scan(scan)

    # Request full nominal speed 0.40 m/s
    cmd_in = Twist()
    cmd_in.linear.x = 0.40
    node.on_cmd_vel_in(cmd_in)

    node.control_loop()

    # Speed must be reduced significantly below requested 0.40
    assert node.last_cmd_linear < 0.30
    assert 0.0 < node.safe_speed_limit < 0.30
    assert "OBSTACLE_AHEAD" in node.limit_reason
    assert "0.80 m" in node.limit_reason
    node.destroy_node()


def test_controller_safety_invariant_emergency_stop(rclpy_init):
    node = AdaptiveSpeedControllerNode()
    # Explicitly toggle adaptive scaling OFF
    node.adaptive_enabled = False

    # Obstacle placed dangerously close (0.35m <= threshold of 0.45m)
    scan = create_fake_scan(front_dist=0.35)
    node.on_scan(scan)

    cmd_in = Twist()
    cmd_in.linear.x = 0.40
    node.on_cmd_vel_in(cmd_in)

    node.control_loop()

    # SAFETY INVARIANT: Must immediately halt (v=0) even when adaptive mode is turned off!
    assert node.last_cmd_linear == 0.0
    assert node.safe_speed_limit == 0.0
    assert "EMERGENCY_STOP" in node.limit_reason
    assert "0.35 m" in node.limit_reason
    node.destroy_node()


def test_controller_sensor_timeout_halts(rclpy_init):
    node = AdaptiveSpeedControllerNode()
    node.sensor_timeout = 0.01  # Very short timeout

    # Provide scan then wait
    scan = create_fake_scan(front_dist=5.0)
    node.on_scan(scan)
    time.sleep(0.02)

    cmd_in = Twist()
    cmd_in.linear.x = 0.40
    node.on_cmd_vel_in(cmd_in)

    node.control_loop()

    # Failsafe must halt robot and report SENSOR_TIMEOUT
    assert node.safety_state == "SENSOR_TIMEOUT"
    assert node.limit_reason == "SENSOR_TIMEOUT"
    node.destroy_node()
