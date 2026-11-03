import subprocess
import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def move_barrier(y_pos: float) -> bool:
    """
    Invokes Gazebo Harmonic's /world/empty/set_pose service
    to update the pose of 'zone4_barrier'.
    """
    req_str = f'name: "zone4_barrier", position: {{x: 6.6, y: {y_pos}, z: 0.5}}'
    cmd = [
        "gz", "service",
        "-s", "/world/empty/set_pose",
        "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "2500",
        "--req", req_str
    ]
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, check=True)
        return "data: true" in res.stdout
    except Exception as e:
        print(f"[ERROR] Failed to move barrier via gz service: {e}")
        return False


def main(args=None):
    """Command-line utility and ROS 2 client to remove Zone 4 barrier."""
    print(">> Removing Zone 4 barrier from corridor...")
    success = move_barrier(10.0)
    if success:
        print("[SUCCESS] Zone 4 barrier moved to (x=6.6, y=10.0). Path is now CLEAR!")
    else:
        print("[WARNING] Could not confirm barrier removal via Gazebo service. Ensure Gazebo simulation is active.")


def reset_main(args=None):
    """Command-line utility to restore Zone 4 barrier into the corridor."""
    print(">> Restoring Zone 4 barrier into corridor...")
    success = move_barrier(0.0)
    if success:
        print("[SUCCESS] Zone 4 barrier restored to (x=6.6, y=0.0). Path is now BLOCKED.")
    else:
        print("[WARNING] Could not confirm barrier restoration via Gazebo service. Ensure Gazebo simulation is active.")


if __name__ == '__main__':
    main()

