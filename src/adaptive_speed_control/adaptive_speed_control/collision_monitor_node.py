#!/usr/bin/env python3
import math
import time
from collections import deque
from typing import List

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
from adaptive_speed_interfaces.msg import ObstacleInfo
from std_msgs.msg import Int32, Float32
from std_srvs.srv import Trigger


class CollisionMonitorNode(Node):
    """
    Monitors physical collisions, near-miss events, and motion smoothness.
    - Subscribes to /contacts (bridged Gazebo contact sensor) with footprint clearance fallback.
    - Debounces contiguous contact periods into discrete collision events.
    - Counts near-miss incidents (clearance < near_miss_clearance).
    - Computes real-time and summary smoothness metrics: RMS jerk, accel variance, hard brakes, speed stddev.
    """

    def __init__(self):
        super().__init__('collision_monitor_node')

        self.declare_parameter('near_miss_clearance', 0.15)
        self.declare_parameter('hard_brake_threshold', -0.75)
        self.declare_parameter('debounce_sec', 0.50)

        self.near_miss_clearance = float(self.get_parameter('near_miss_clearance').value)
        self.hard_brake_thresh = float(self.get_parameter('hard_brake_threshold').value)
        self.debounce_sec = float(self.get_parameter('debounce_sec').value)

        # Subscribers
        self.contact_sub = self.create_subscription(
            Contacts, '/contacts', self.on_contacts, 10
        )
        self.obs_sub = self.create_subscription(
            ObstacleInfo, '/obstacle_info', self.on_obstacle_info, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.on_odom, 10
        )

        # Publishers
        self.collision_pub = self.create_publisher(Int32, '/collision_count', 10)
        self.near_miss_pub = self.create_publisher(Int32, '/near_miss_count', 10)

        # Service
        self.summary_srv = self.create_service(
            Trigger, '/get_collision_summary', self.handle_get_summary
        )

        # State
        self.collision_count = 0
        self.near_miss_count = 0
        self.min_clearance_observed = float('inf')

        self.in_contact = False
        self.last_contact_time = 0.0

        self.in_near_miss = False
        self.last_near_miss_time = 0.0

        # Motion tracking for smoothness metrics
        self.timestamps: List[float] = []
        self.speeds: List[float] = []
        self.accelerations: List[float] = []
        self.jerks: List[float] = []
        self.hard_brake_count = 0

        self.timer = self.create_timer(1.0, self.status_loop)
        self.get_logger().info("Collision and Smoothness Monitor Node initialized.")

    def on_contacts(self, msg: Contacts):
        now = time.time()
        has_contact = len(msg.contacts) > 0

        if has_contact:
            if not self.in_contact and (now - self.last_contact_time) > self.debounce_sec:
                self.collision_count += 1
                self.in_contact = True
                self.last_contact_time = now
                self.collision_pub.publish(Int32(data=self.collision_count))
                self.get_logger().error(
                    f"[COLLISION DETECTED] Physics contact event #{self.collision_count}!"
                )
        else:
            if self.in_contact and (now - self.last_contact_time) > self.debounce_sec:
                self.in_contact = False

    def on_obstacle_info(self, msg: ObstacleInfo):
        now = time.time()
        clearance = msg.clearance if hasattr(msg, 'clearance') and msg.clearance >= 0.0 else (msg.front_distance - msg.threshold)

        if math.isfinite(clearance):
            if clearance < self.min_clearance_observed:
                self.min_clearance_observed = clearance

            # Fallback contact detection if physical sensor bridge is unavailable
            if clearance <= 0.01:
                if not self.in_contact and (now - self.last_contact_time) > self.debounce_sec:
                    self.collision_count += 1
                    self.in_contact = True
                    self.last_contact_time = now
                    self.collision_pub.publish(Int32(data=self.collision_count))
                    self.get_logger().error(
                        f"[COLLISION DETECTED] Footprint geometric collision #{self.collision_count}!"
                    )
            elif clearance > 0.08:
                if self.in_contact and (now - self.last_contact_time) > self.debounce_sec:
                    self.in_contact = False

            # Near-miss debounced tracking
            if 0.01 < clearance < self.near_miss_clearance:
                if not self.in_near_miss and (now - self.last_near_miss_time) > self.debounce_sec:
                    self.near_miss_count += 1
                    self.in_near_miss = True
                    self.last_near_miss_time = now
                    self.near_miss_pub.publish(Int32(data=self.near_miss_count))
                    self.get_logger().warn(
                        f"[NEAR MISS] Near-miss #{self.near_miss_count} (clearance={clearance:.2f}m)!"
                    )
            elif clearance >= self.near_miss_clearance:
                if self.in_near_miss and (now - self.last_near_miss_time) > self.debounce_sec:
                    self.in_near_miss = False

    def on_odom(self, msg: Odometry):
        now = time.time()
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.hypot(vx, vy)

        if self.timestamps:
            dt = now - self.timestamps[-1]
            if dt >= 0.02:
                dv = speed - self.speeds[-1]
                accel = dv / dt
                self.accelerations.append(accel)

                if accel < self.hard_brake_thresh:
                    self.hard_brake_count += 1

                if len(self.accelerations) >= 2:
                    da = self.accelerations[-1] - self.accelerations[-2]
                    jerk = da / dt
                    self.jerks.append(jerk)

                self.timestamps.append(now)
                self.speeds.append(speed)
        else:
            self.timestamps.append(now)
            self.speeds.append(speed)

    def compute_metrics(self):
        if not self.speeds:
            return {
                "collisions": self.collision_count,
                "near_misses": self.near_miss_count,
                "min_clearance": self.min_clearance_observed,
                "rms_jerk": 0.0,
                "accel_variance": 0.0,
                "hard_decelerations": 0,
                "speed_stddev": 0.0,
            }

        mean_speed = sum(self.speeds) / len(self.speeds)
        speed_var = sum((s - mean_speed) ** 2 for s in self.speeds) / len(self.speeds)
        speed_std = math.sqrt(speed_var)

        if self.accelerations:
            mean_acc = sum(self.accelerations) / len(self.accelerations)
            acc_var = sum((a - mean_acc) ** 2 for a in self.accelerations) / len(self.accelerations)
        else:
            acc_var = 0.0

        if self.jerks:
            rms_jerk = math.sqrt(sum(j ** 2 for j in self.jerks) / len(self.jerks))
        else:
            rms_jerk = 0.0

        return {
            "collisions": self.collision_count,
            "near_misses": self.near_miss_count,
            "min_clearance": self.min_clearance_observed,
            "rms_jerk": rms_jerk,
            "accel_variance": acc_var,
            "hard_decelerations": self.hard_brake_count,
            "speed_stddev": speed_std,
        }

    def status_loop(self):
        self.collision_pub.publish(Int32(data=self.collision_count))
        self.near_miss_pub.publish(Int32(data=self.near_miss_count))

    def handle_get_summary(self, request, response):
        m = self.compute_metrics()
        response.success = True
        response.message = (
            f"Collisions: {m['collisions']} | Near-misses: {m['near_misses']} | "
            f"Min Clearance: {m['min_clearance']:.2f}m | RMS Jerk: {m['rms_jerk']:.2f}m/s^3 | "
            f"Accel Var: {m['accel_variance']:.4f} | Hard Brakes: {m['hard_decelerations']}"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CollisionMonitorNode()
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

