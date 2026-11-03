#!/usr/bin/env python3
import math
import time
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity


class MovingObstacleNode(Node):
    """
    Simulates dynamic moving obstacles (workers / pedestrians) in Gazebo Sim.
    - Smoothly translates models back-and-forth along defined 3D linear trajectories.
    - Calls /world/<world_name>/set_pose service via ros_gz_interfaces/srv/SetEntityPose.
    - Fully configurable via ROS parameters / YAML.
    """

    def __init__(self):
        super().__init__('moving_obstacle_node')

        self.declare_parameter('world_name', 'warehouse')
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('obstacles', [
            'dynamic_worker_1',
            'dynamic_worker_2',
            'dynamic_worker_3'
        ])

        self.world_name = str(self.get_parameter('world_name').value)
        self.update_rate = float(self.get_parameter('update_rate').value)
        obstacle_names = self.get_parameter('obstacles').value

        self.obstacles: Dict[str, Dict[str, Any]] = {}
        for name in obstacle_names:
            prefix = f'obstacle_config.{name}'
            self.declare_parameter(f'{prefix}.start_x', 0.0)
            self.declare_parameter(f'{prefix}.start_y', 0.0)
            self.declare_parameter(f'{prefix}.start_z', 0.85)
            self.declare_parameter(f'{prefix}.end_x', 0.0)
            self.declare_parameter(f'{prefix}.end_y', 0.0)
            self.declare_parameter(f'{prefix}.end_z', 0.85)
            self.declare_parameter(f'{prefix}.speed', 0.30)
            self.declare_parameter(f'{prefix}.pause_duration', 0.5)

            sx = float(self.get_parameter(f'{prefix}.start_x').value)
            sy = float(self.get_parameter(f'{prefix}.start_y').value)
            sz = float(self.get_parameter(f'{prefix}.start_z').value)
            ex = float(self.get_parameter(f'{prefix}.end_x').value)
            ey = float(self.get_parameter(f'{prefix}.end_y').value)
            ez = float(self.get_parameter(f'{prefix}.end_z').value)
            spd = max(0.05, float(self.get_parameter(f'{prefix}.speed').value))
            pause = float(self.get_parameter(f'{prefix}.pause_duration').value)

            dist = math.sqrt((ex - sx)**2 + (ey - sy)**2 + (ez - sz)**2)
            travel_time = dist / spd if dist > 0.01 else 1.0

            self.obstacles[name] = {
                'start': (sx, sy, sz),
                'end': (ex, ey, ez),
                'dist': dist,
                'speed': spd,
                'pause': pause,
                'travel_time': travel_time,
                'total_half_period': travel_time + pause,
                'total_period': 2.0 * (travel_time + pause),
            }

        service_name = f'/world/{self.world_name}/set_pose'
        self.set_pose_cli = self.create_client(SetEntityPose, service_name)
        self.get_logger().info(
            f"MovingObstacleNode started for world '{self.world_name}' with {len(self.obstacles)} dynamic obstacles.\n"
            f"Target service: {service_name}"
        )

        self.start_time = time.time()
        timer_period = 1.0 / max(1.0, self.update_rate)
        self.timer = self.create_timer(timer_period, self.update_positions)

    def update_positions(self):
        if not self.set_pose_cli.service_is_ready():
            return

        now = time.time()
        elapsed = now - self.start_time

        for name, config in self.obstacles.items():
            if config['dist'] < 0.01:
                continue

            period = config['total_period']
            half = config['total_half_period']
            travel_time = config['travel_time']
            pause = config['pause']

            t_cycle = elapsed % period

            if t_cycle < travel_time:
                # Forward motion (start -> end)
                alpha = t_cycle / travel_time
            elif t_cycle < half:
                # Pause at end
                alpha = 1.0
            elif t_cycle < (half + travel_time):
                # Reverse motion (end -> start)
                alpha = 1.0 - ((t_cycle - half) / travel_time)
            else:
                # Pause at start
                alpha = 0.0

            sx, sy, sz = config['start']
            ex, ey, ez = config['end']

            curr_x = sx + alpha * (ex - sx)
            curr_y = sy + alpha * (ey - sy)
            curr_z = sz + alpha * (ez - sz)

            # Yaw orientation pointing along direction of travel
            dx = (ex - sx) if t_cycle < half else (sx - ex)
            dy = (ey - sy) if t_cycle < half else (sy - ex)
            yaw = math.atan2(dy, dx)
            cy = math.cos(yaw * 0.5)
            sy_sin = math.sin(yaw * 0.5)

            req = SetEntityPose.Request()
            req.entity = Entity(name=name, type=Entity.MODEL)
            req.pose = Pose(
                position=Point(x=float(curr_x), y=float(curr_y), z=float(curr_z)),
                orientation=Quaternion(x=0.0, y=0.0, z=float(sy_sin), w=float(cy))
            )
            self.set_pose_cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = MovingObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

