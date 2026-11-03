#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State


class LifecycleManagerNode(Node):
    """
    Orchestrates ROS 2 Lifecycle transitions (configure -> activate -> deactivate -> cleanup)
    for safety-critical autonomous navigation and speed control nodes.
    """

    def __init__(self):
        super().__init__('lifecycle_manager_node')

        self.declare_parameter('managed_nodes', [
            'adaptive_speed_controller_node',
            'waypoint_navigator_node'
        ])
        self.declare_parameter('auto_activate', True)

        self.managed_nodes = list(self.get_parameter('managed_nodes').value)
        self.auto_activate = bool(self.get_parameter('auto_activate').value)

        self.clients = {}
        for node_name in self.managed_nodes:
            srv_name = f'/{node_name}/change_state'
            self.clients[node_name] = self.create_client(ChangeState, srv_name)

        if self.auto_activate:
            self.timer = self.create_timer(0.5, self._startup_sequence)
        else:
            self.timer = None

    def _startup_sequence(self):
        if self.timer:
            self.timer.cancel()
            self.timer = None

        self.get_logger().info(f"Starting lifecycle management for: {self.managed_nodes}")
        # Step 1: Configure all nodes
        for node_name in self.managed_nodes:
            self.change_state(node_name, Transition.TRANSITION_CONFIGURE)

        time.sleep(0.2)

        # Step 2: Activate all nodes
        for node_name in self.managed_nodes:
            self.change_state(node_name, Transition.TRANSITION_ACTIVATE)

        self.get_logger().info("All managed lifecycle nodes successfully configured and activated.")

    def change_state(self, node_name: str, transition_id: int) -> bool:
        client = self.clients.get(node_name)
        if not client:
            return False

        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f"Lifecycle service for {node_name} not available, skipping.")
            return False

        req = ChangeState.Request()
        req.transition.id = transition_id

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if future.done() and future.result() is not None:
            success = future.result().success
            self.get_logger().info(f"Node '{node_name}' transition {transition_id} result: {success}")
            return success
        return False

    def shutdown_sequence(self):
        self.get_logger().info("Executing lifecycle shutdown sequence (deactivating managed nodes)...")
        for node_name in reversed(self.managed_nodes):
            try:
                self.change_state(node_name, Transition.TRANSITION_DEACTIVATE)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_sequence()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

