import time
import math
import pytest
import rclpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from rt1_assignment2.distance_node import DistanceNode
from rt1_assignment2.ui_node import UInode
from rt1_assignment2.waypoint_navigator import WaypointNavigator
from rt1_assignment2_interfaces.srv import SetThreshold, GetAverageVel
from std_srvs.srv import Trigger


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_set_threshold_service(ros_context):
    """Test the /set_threshold service accepts valid thresholds and rejects invalid ones."""
    node = DistanceNode()
    client = node.create_client(SetThreshold, '/set_threshold')

    assert client.wait_for_service(timeout_sec=2.0)

    # 1. Valid threshold
    req = SetThreshold.Request()
    req.threshold = 0.55
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    res = future.result()
    assert res.success is True
    assert math.isclose(node.threshold, 0.55, abs_tol=1e-3)

    # 2. Invalid negative threshold
    req.threshold = -0.2
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    res = future.result()
    assert res.success is False

    # 3. Invalid threshold >= slowdown_distance (2.00)
    req.threshold = 2.50
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    res = future.result()
    assert res.success is False

    node.destroy_node()


def test_ui_get_average_service(ros_context):
    """Test the /get_average_vel service returns proper averages and sample count."""
    ui = UInode(start_input_thread=False)
    client = ui.create_client(GetAverageVel, '/get_average_vel')
    assert client.wait_for_service(timeout_sec=2.0)

    # Initially empty
    req = GetAverageVel.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(ui, future, timeout_sec=2.0)
    res = future.result()
    assert res.samples == 0

    # Add 3 commands to history
    with ui._lock:
        ui._last_cmds.append((0.2, 0.1))
        ui._last_cmds.append((0.4, -0.1))
        ui._last_cmds.append((0.3, 0.0))

    future = client.call_async(req)
    rclpy.spin_until_future_complete(ui, future, timeout_sec=2.0)
    res = future.result()
    assert res.samples == 3
    assert math.isclose(res.avg_linear, 0.30, abs_tol=1e-3)
    assert math.isclose(res.avg_angular, 0.00, abs_tol=1e-3)

    ui.destroy_node()


def test_distance_node_pipeline_and_safety_stop(ros_context):
    """Test full pipeline: scan processing, telemetry, adaptive scaling, and emergency stop."""
    node = DistanceNode()

    published_cmds = []
    original_publish = node.cmd_pub.publish

    def mock_publish(msg):
        published_cmds.append(msg)
        return original_publish(msg)

    node.cmd_pub.publish = mock_publish

    # Synthetic LaserScan: clear at 5.0m
    scan = LaserScan()
    scan.header.stamp = node.get_clock().now().to_msg()
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = (2 * math.pi) / 360
    scan.range_min = 0.05
    scan.range_max = 10.0
    scan.ranges = [5.0] * 360

    node.on_scan(scan)
    assert node.front_distance == 5.0
    assert node.obstacle_density == 0.0

    cmd_in = Twist()
    cmd_in.linear.x = 0.40
    cmd_in.angular.z = 0.0
    node.on_cmd_vel_in(cmd_in)

    # 1. Open corridor test
    node.control_loop()
    assert len(published_cmds) > 0
    assert node.safety_state == "EMPTY_CORRIDOR"
    assert published_cmds[-1].linear.x > 0.0

    # 2. Obstacle inside safety threshold (0.35m <= 0.45m) -> CRITICAL_PROXIMITY
    scan.ranges[180] = 0.35  # Forward beam
    node.on_scan(scan)
    node.on_cmd_vel_in(cmd_in)
    node.control_loop()
    assert node.safety_state == "CRITICAL_PROXIMITY"
    assert published_cmds[-1].linear.x == 0.0
    assert published_cmds[-1].angular.z == 0.0

    # 3. Path clears (obstacle removed, range jumps to 4.0m) -> PATH_CLEAR
    scan.ranges[180] = 4.0
    node.on_scan(scan)
    node.on_cmd_vel_in(cmd_in)
    node.control_loop()
    assert node.safety_state in ("PATH_CLEAR", "EMPTY_CORRIDOR")
    assert published_cmds[-1].linear.x > 0.0

    # 4. Sensor timeout test
    node.last_scan_time = time.time() - 2.0
    node.on_cmd_vel_in(cmd_in)
    node.control_loop()
    assert node.safety_state == "SENSOR_TIMEOUT"
    assert published_cmds[-1].linear.x == 0.0

    # 5. Command timeout test
    node.last_scan_time = time.time()
    node.last_cmd_time = time.time() - 2.0
    node.control_loop()
    assert node.safety_state == "IDLE"
    assert published_cmds[-1].linear.x == 0.0

    node.destroy_node()


def test_waypoint_navigator_guidance(ros_context):
    """Test waypoint navigator tracks odometry, stays on centerline, and targets goal."""
    nav = WaypointNavigator()

    nav_cmds = []
    original_publish = nav.cmd_pub.publish

    def mock_publish(msg):
        nav_cmds.append(msg)
        return original_publish(msg)

    nav.cmd_pub.publish = mock_publish

    # Simulate robot at (-8.0, 0.2), slightly offset from centerline
    odom = Odometry()
    odom.pose.pose.position.x = -8.0
    odom.pose.pose.position.y = 0.2
    odom.pose.pose.orientation.w = 1.0  # yaw = 0

    nav.on_odom(odom)
    nav.navigation_loop()

    assert len(nav_cmds) > 0
    # Should request nominal forward speed
    assert math.isclose(nav_cmds[-1].linear.x, 0.40, abs_tol=1e-3)
    # Since robot is at y = +0.2, cross-track error is negative, steering angular command should be negative (steer right towards y=0)
    assert nav_cmds[-1].angular.z < 0.0

    # Simulate robot reaching destination at x = 9.0
    odom.pose.pose.position.x = 9.05
    odom.pose.pose.position.y = 0.0
    nav.on_odom(odom)
    nav.navigation_loop()

    assert nav.destination_reached is True
    assert nav.is_navigating is False
    assert nav_cmds[-1].linear.x == 0.0

    nav.destroy_node()
