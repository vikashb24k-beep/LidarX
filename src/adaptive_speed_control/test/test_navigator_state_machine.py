import time
import math
import pytest
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from adaptive_speed_control.waypoint_navigator_node import WaypointNavigatorNode
from adaptive_speed_interfaces.msg import ObstacleInfo


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_navigator_idle_and_start_stop_services(ros_context):
    """Tests that navigator respects IDLE state, starts via service, and stops cleanly."""
    nav = WaypointNavigatorNode()
    nav.nav_state = 'IDLE'

    # When in IDLE, loop produces no velocity
    published = []
    nav.cmd_pub.publish = lambda msg: published.append(msg)

    odom = Odometry()
    odom.pose.pose.position.x = -9.0
    odom.pose.pose.position.y = 0.0
    odom.pose.pose.orientation.w = 1.0
    nav.on_odom(odom)
    nav.navigation_loop()

    assert len(published) == 0

    # Test /start_navigation service handler
    req = Trigger.Request()
    res = Trigger.Response()
    nav.handle_start_nav(req, res)
    assert res.success is True
    assert nav.nav_state == 'DRIVE'

    # Now loop should publish
    nav.navigation_loop()
    assert len(published) > 0
    assert published[-1].linear.x > 0.0

    # Test /stop_navigation service handler
    nav.handle_stop_nav(req, res)
    assert res.success is True
    assert nav.nav_state == 'IDLE'

    nav.destroy_node()


def test_navigator_obstacle_avoidance_nudge(ros_context):
    """Tests reactive nudge away from obstacles close on the left or right flank."""
    nav = WaypointNavigatorNode()
    published = []
    nav.cmd_pub.publish = lambda msg: published.append(msg)

    odom = Odometry()
    odom.pose.pose.position.x = 0.0
    odom.pose.pose.position.y = 0.0
    odom.pose.pose.orientation.w = 1.0
    nav.on_odom(odom)
    nav.nav_state = 'DRIVE'

    # Simulate obstacle close on the left (0.50m) vs right (1.50m)
    obs = ObstacleInfo()
    obs.closest_distance = 0.50
    obs.left_distance = 0.50
    obs.right_distance = 1.50
    obs.front_distance = 2.00
    nav.on_obstacle_info(obs)

    nav.navigation_loop()

    assert len(published) > 0
    assert nav.nav_state == 'AVOID_OBSTACLE'
    # Should turn right (negative angular z) away from the left obstacle
    assert published[-1].angular.z < 0.0

    nav.destroy_node()


def test_navigator_goal_arrival(ros_context):
    """Tests that reaching goal coordinate transitions state to GOAL_REACHED and stops."""
    nav = WaypointNavigatorNode()
    published = []
    nav.cmd_pub.publish = lambda msg: published.append(msg)

    odom = Odometry()
    odom.pose.pose.position.x = 9.10
    odom.pose.pose.position.y = 0.0
    odom.pose.pose.orientation.w = 1.0
    nav.on_odom(odom)
    nav.nav_state = 'DRIVE'

    nav.navigation_loop()

    assert nav.nav_state == 'GOAL_REACHED'
    assert nav.destination_reached is True
    assert len(published) > 0
    assert published[-1].linear.x == 0.0

    nav.destroy_node()

