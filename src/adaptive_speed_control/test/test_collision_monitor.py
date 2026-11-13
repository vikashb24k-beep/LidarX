import time
import pytest
import rclpy
from std_msgs.msg import Int32
from ros_gz_interfaces.msg import Contacts, Contact, Entity
from adaptive_speed_interfaces.msg import ObstacleInfo
from adaptive_speed_control.collision_monitor_node import CollisionMonitorNode


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_collision_monitor_init(ros_context):
    node = CollisionMonitorNode()
    assert node.collision_count == 0
    assert node.near_miss_count == 0
    metrics = node.compute_metrics()
    assert metrics['collisions'] == 0
    assert metrics['near_misses'] == 0
    node.destroy_node()


def test_positive_control_contact_collision(ros_context):
    """
    Positive control test: verifies that contact sensor events
    are reliably detected, recorded, and debounced into collision counts.
    """
    node = CollisionMonitorNode()

    # Create dummy contact
    msg = Contacts()
    contact = Contact()
    contact.collision1 = Entity(name="robot::chassis::collision")
    contact.collision2 = Entity(name="obstacle::box::collision")
    msg.contacts.append(contact)

    # First contact event
    node.on_contacts(msg)
    assert node.collision_count == 1
    assert node.in_contact is True

    # Immediate second contact message (within debounce period) -> should NOT double-count
    node.on_contacts(msg)
    assert node.collision_count == 1

    # Release contact
    empty_msg = Contacts()
    node.on_contacts(empty_msg)

    # Simulate debounce passage
    node.last_contact_time = time.time() - 1.0
    node.on_contacts(empty_msg)
    assert node.in_contact is False

    # Second distinct contact event after debounce
    node.on_contacts(msg)
    assert node.collision_count == 2

    metrics = node.compute_metrics()
    assert metrics['collisions'] == 2

    node.destroy_node()


def test_positive_control_footprint_clearance_fallback(ros_context):
    """
    Verifies that when clearance drops to zero (footprint penetration),
    a collision is detected even without Gazebo physics contact sensor.
    """
    node = CollisionMonitorNode()

    obs_msg = ObstacleInfo()
    obs_msg.clearance = 0.005  # inside obstacle boundary
    obs_msg.front_distance = 0.20
    obs_msg.threshold = 0.45

    node.on_obstacle_info(obs_msg)
    assert node.collision_count == 1
    assert node.in_contact is True

    node.destroy_node()


def test_near_miss_detection(ros_context):
    """
    Verifies that clearance between 0.01 and near_miss_clearance triggers near-miss.
    """
    node = CollisionMonitorNode()

    obs_msg = ObstacleInfo()
    obs_msg.clearance = 0.10  # between 0.01 and 0.15
    obs_msg.front_distance = 0.55
    obs_msg.threshold = 0.45

    node.on_obstacle_info(obs_msg)
    assert node.near_miss_count == 1
    assert node.in_near_miss is True

    # Clearance increases back to safe zone
    safe_msg = ObstacleInfo()
    safe_msg.clearance = 0.50
    node.last_near_miss_time = time.time() - 1.0
    node.on_obstacle_info(safe_msg)
    assert node.in_near_miss is False

    node.destroy_node()
