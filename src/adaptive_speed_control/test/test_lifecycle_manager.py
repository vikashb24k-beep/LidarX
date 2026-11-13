import pytest
import rclpy
from lifecycle_msgs.msg import Transition
from adaptive_speed_control.lifecycle_manager_node import LifecycleManagerNode


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_lifecycle_manager_initialization(ros_context):
    node = LifecycleManagerNode()
    assert 'adaptive_speed_controller_node' in node.managed_nodes
    assert node.auto_activate is True
    node.destroy_node()


def test_lifecycle_manager_change_state_absent_service(ros_context):
    node = LifecycleManagerNode()
    # Should safely return False when service is not up, without crashing
    success = node.change_state('non_existent_node', Transition.TRANSITION_CONFIGURE)
    assert success is False
    node.destroy_node()
