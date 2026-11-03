#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from adaptive_speed_interfaces.msg import ObstacleInfo, RobotStatus


class ChallengeScoringNode(Node):
    """
    Evaluates manual driving performance and safety compliance.
    Scores: Completion Time, Minimum Obstacle Clearance, Emergency Stops, Average Speed.
    """

    def __init__(self):
        super().__init__('challenge_scoring_node')

        self.declare_parameter('goal_x', 7.5)
        self.declare_parameter('goal_y', 4.5)
        self.declare_parameter('goal_tolerance', 0.8)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)

        # Subscribers
        self.status_sub = self.create_subscription(RobotStatus, '/robot_status', self.on_robot_status, 10)
        self.obs_sub = self.create_subscription(ObstacleInfo, '/obstacle_info', self.on_obstacle_info, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)

        # Metrics
        self.mission_started = False
        self.mission_completed = False
        self.start_time = 0.0
        self.end_time = 0.0
        self.min_clearance = float('inf')
        self.emergency_stop_count = 0
        self.in_emergency = False
        self.speed_samples = []
        self.current_x = 0.0
        self.current_y = 0.0
        self.total_distance = 0.0

        self.timer = self.create_timer(1.0, self.score_loop)
        self.get_logger().info("Challenge Scoring Node active! Waiting for robot to start moving...")

    def on_odom(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def on_robot_status(self, msg: RobotStatus):
        self.total_distance = msg.distance_travelled
        if msg.speed > 0.02 and not self.mission_started and not self.mission_completed:
            self.mission_started = True
            self.start_time = time.time()
            self.get_logger().info("[CHALLENGE] Motion detected! Timer started.")

        if self.mission_started and not self.mission_completed:
            self.speed_samples.append(msg.speed)

    def on_obstacle_info(self, msg: ObstacleInfo):
        if not self.mission_started or self.mission_completed:
            return

        dist = msg.closest_distance
        if math.isfinite(dist) and dist < self.min_clearance:
            self.min_clearance = dist

        is_crit = (msg.front_distance <= msg.threshold) or ("CRITICAL" in msg.zone.upper())
        if is_crit and not self.in_emergency:
            self.emergency_stop_count += 1
            self.in_emergency = True
            self.get_logger().warn(
                f"[CHALLENGE PENALTY] Emergency stop #{self.emergency_stop_count} triggered! (d={msg.front_distance:.2f}m)"
            )
        elif not is_crit:
            self.in_emergency = False

    def score_loop(self):
        if not self.mission_started or self.mission_completed:
            return

        # Check goal distance
        d_goal = math.hypot(self.current_x - self.goal_x, self.current_y - self.goal_y)
        if d_goal <= self.goal_tolerance:
            self.mission_completed = True
            self.end_time = time.time()
            self.print_final_score()

    def print_final_score(self):
        duration = max(0.1, self.end_time - self.start_time)
        avg_speed = (sum(self.speed_samples) / len(self.speed_samples)) if self.speed_samples else 0.0

        # Score computation:
        # 100 base points
        # -15 pts per emergency stop
        # -10 pts if min clearance < 0.35m
        # Bonus for fast completion
        score = 100.0 - (self.emergency_stop_count * 15.0)
        if self.min_clearance < 0.35:
            score -= 10.0
        if avg_speed >= 0.25:
            score += 10.0
        score = max(0.0, min(100.0, score))

        if score >= 90:
            rank = "S (Expert Driver)"
        elif score >= 75:
            rank = "A (Safe & Smooth)"
        elif score >= 60:
            rank = "B (Acceptable)"
        else:
            rank = "C (Needs Practice)"

        summary = (
            f"\n"
            f"========================================================================\n"
            f"                   MANUAL DRIVE CHALLENGE RESULTS                       \n"
            f"========================================================================\n"
            f"  Completion Time          : {duration:.1f} s\n"
            f"  Total Distance           : {self.total_distance:.2f} m\n"
            f"  Average Speed            : {avg_speed:.2f} m/s\n"
            f"  Minimum Obstacle Margin  : {self.min_clearance:.2f} m\n"
            f"  Emergency Stops Triggered: {self.emergency_stop_count}\n"
            f"------------------------------------------------------------------------\n"
            f"  FINAL SCORE              : {score:.1f} / 100.0\n"
            f"  SAFETY RANK              : {rank}\n"
            f"========================================================================\n"
        )
        self.get_logger().info(summary)


def main(args=None):
    rclpy.init(args=args)
    node = ChallengeScoringNode()
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
