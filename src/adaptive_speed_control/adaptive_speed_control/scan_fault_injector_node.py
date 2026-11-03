#!/usr/bin/env python3
import math
import os
import random
import time
from collections import deque

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan
from ament_index_python.packages import get_package_share_directory
import diagnostic_updater
import diagnostic_msgs.msg


class ScanFaultInjectorNode(Node):
    """
    Simulates realistic LiDAR sensor degradations and faults.
    Subscribes to /scan_raw and publishes degraded /scan.
    When no faults are enabled, passes messages through untouched.
    """

    def __init__(self):
        super().__init__('scan_fault_injector_node')

        self.declare_parameter('fault_profile', 'none')
        self.declare_parameter('seed', 42)
        self.declare_parameter('noise_sigma', 0.0)
        self.declare_parameter('noise_range_proportional', False)
        self.declare_parameter('dropout_prob', 0.0)
        self.declare_parameter('dropout_value', 'inf')
        self.declare_parameter('delay_ms', 0.0)
        self.declare_parameter('drop_scan_prob', 0.0)
        self.declare_parameter('frozen_scan_sec', 0.0)
        self.declare_parameter('blind_sector_start_deg', 0.0)
        self.declare_parameter('blind_sector_end_deg', 0.0)

        # Load profile if specified
        self._load_profiles_file()
        profile_name = str(self.get_parameter('fault_profile').value).strip().lower()
        if profile_name in self.profiles:
            self._apply_profile(profile_name)

        self._load_parameters()
        self.add_on_set_parameters_callback(self._on_parameters_set)

        # Seed PRNG
        if self.seed >= 0:
            self.rng = random.Random(self.seed)
        else:
            self.rng = random.Random()

        # Buffers & state
        self.delayed_queue = deque()
        self.frozen_scan_msg = None
        self.frozen_until_time = 0.0

        # Stats for diagnostics
        self.input_scans = 0
        self.published_scans = 0
        self.dropped_scans = 0
        self.last_input_time = 0.0
        self.last_pub_time = 0.0

        # Gazebo's parameter bridge currently offers /scan_raw as RELIABLE.
        # Request reliable delivery at this ingress point; the downstream
        # controller-facing /scan deliberately remains sensor-data BEST_EFFORT.
        raw_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.on_scan_raw,
            raw_qos
        )

        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan',
            qos_profile_sensor_data
        )

        # Timer to flush delayed scans
        self.flush_timer = self.create_timer(0.01, self._flush_delayed_queue)

        # Diagnostic Updater
        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID("lidar_fault_injector")
        self.updater.add("Fault Injection Status", self._produce_diagnostics)

        self.get_logger().info(
            f"Scan Fault Injector initialized | Profile: {self.fault_profile} | "
            f"Noise σ: {self.noise_sigma:.3f}m | Dropout: {self.dropout_prob * 100:.1f}% | "
            f"Delay: {self.delay_ms:.1f}ms | DropScan: {self.drop_scan_prob * 100:.1f}%"
        )

    def _load_profiles_file(self):
        self.profiles = {}
        try:
            pkg_share = get_package_share_directory('adaptive_speed_control')
            config_file = os.path.join(pkg_share, 'config', 'fault_profiles.yaml')
            if os.path.exists(config_file):
                with open(config_file, 'r') as f:
                    doc = yaml.safe_load(f)
                    if doc and 'fault_profiles' in doc:
                        self.profiles = doc['fault_profiles']
        except Exception as e:
            self.get_logger().warn(f"Could not load fault_profiles.yaml: {e}")

    def _apply_profile(self, name: str):
        prof = self.profiles.get(name, {})
        self.fault_profile = name
        self.noise_sigma = float(prof.get('noise_sigma', 0.0))
        self.noise_range_proportional = bool(prof.get('noise_range_proportional', False))
        self.dropout_prob = float(prof.get('dropout_prob', 0.0))
        self.dropout_value = str(prof.get('dropout_value', 'inf'))
        self.delay_ms = float(prof.get('delay_ms', 0.0))
        self.drop_scan_prob = float(prof.get('drop_scan_prob', 0.0))
        self.frozen_scan_sec = float(prof.get('frozen_scan_sec', 0.0))
        self.blind_sector_start_deg = float(prof.get('blind_sector_start_deg', 0.0))
        self.blind_sector_end_deg = float(prof.get('blind_sector_end_deg', 0.0))

    def _load_parameters(self):
        self.fault_profile = str(self.get_parameter('fault_profile').value).strip().lower()
        self.seed = int(self.get_parameter('seed').value)
        self.noise_sigma = float(self.get_parameter('noise_sigma').value)
        self.noise_range_proportional = bool(self.get_parameter('noise_range_proportional').value)
        self.dropout_prob = float(self.get_parameter('dropout_prob').value)
        self.dropout_value = str(self.get_parameter('dropout_value').value)
        self.delay_ms = float(self.get_parameter('delay_ms').value)
        self.drop_scan_prob = float(self.get_parameter('drop_scan_prob').value)
        self.frozen_scan_sec = float(self.get_parameter('frozen_scan_sec').value)
        self.blind_sector_start_deg = float(self.get_parameter('blind_sector_start_deg').value)
        self.blind_sector_end_deg = float(self.get_parameter('blind_sector_end_deg').value)

    def _on_parameters_set(self, params):
        for param in params:
            if param.name == 'fault_profile':
                p_name = str(param.value).strip().lower()
                if p_name in self.profiles:
                    self._apply_profile(p_name)
            elif param.name == 'noise_sigma':
                self.noise_sigma = float(param.value)
            elif param.name == 'dropout_prob':
                self.dropout_prob = float(param.value)
            elif param.name == 'delay_ms':
                self.delay_ms = float(param.value)
            elif param.name == 'drop_scan_prob':
                self.drop_scan_prob = float(param.value)
            elif param.name == 'seed':
                self.seed = int(param.value)
                self.rng = random.Random(self.seed)
        return SetParametersResult(successful=True)

    def _produce_diagnostics(self, stat: diagnostic_updater.DiagnosticStatusWrapper):
        has_faults = (
            self.noise_sigma > 0.0 or
            self.dropout_prob > 0.0 or
            self.delay_ms > 0.0 or
            self.drop_scan_prob > 0.0 or
            self.fault_profile != 'none'
        )
        if has_faults:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, f"Faults Active: {self.fault_profile}")
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal Passthrough")

        stat.add("Profile", str(self.fault_profile))
        stat.add("Noise Sigma (m)", f"{self.noise_sigma:.3f}")
        stat.add("Dropout Probability", f"{self.dropout_prob:.3f}")
        stat.add("Delay (ms)", f"{self.delay_ms:.1f}")
        stat.add("Scan Drop Probability", f"{self.drop_scan_prob:.3f}")
        stat.add("Input Scans", str(self.input_scans))
        stat.add("Published Scans", str(self.published_scans))
        stat.add("Dropped Scans", str(self.dropped_scans))
        return stat

    def on_scan_raw(self, msg: LaserScan):
        now = time.time()
        self.input_scans += 1
        self.last_input_time = now

        # 1. Whole scan drop fault
        if self.drop_scan_prob > 0.0 and self.rng.random() < self.drop_scan_prob:
            self.dropped_scans += 1
            return

        # 2. Frozen scan fault
        if self.frozen_scan_sec > 0.0:
            if now < self.frozen_until_time and self.frozen_scan_msg is not None:
                # Reuse frozen scan but preserve original stamp
                out_msg = self.frozen_scan_msg
                self._dispatch_scan(out_msg, now)
                return
            elif self.rng.random() < 0.10:
                self.frozen_until_time = now + self.frozen_scan_sec
                self.frozen_scan_msg = msg

        # If no faults are active, pass message through untouched immediately
        no_faults_active = (
            self.noise_sigma <= 1e-6 and
            self.dropout_prob <= 1e-6 and
            self.delay_ms <= 1e-6 and
            self.blind_sector_start_deg == 0.0 and
            self.blind_sector_end_deg == 0.0
        )
        if no_faults_active:
            self._dispatch_scan(msg, now)
            return

        # Create output scan copy
        out_msg = LaserScan()
        out_msg.header = msg.header  # PRESERVE ORIGINAL STAMP AND FRAME
        out_msg.angle_min = msg.angle_min
        out_msg.angle_max = msg.angle_max
        out_msg.angle_increment = msg.angle_increment
        out_msg.time_increment = msg.time_increment
        out_msg.scan_time = msg.scan_time
        out_msg.range_min = msg.range_min
        out_msg.range_max = msg.range_max

        val_dropout = 0.0 if '0' in str(self.dropout_value) else float('inf')
        blind_start_rad = math.radians(self.blind_sector_start_deg)
        blind_end_rad = math.radians(self.blind_sector_end_deg)
        has_blind_sector = (self.blind_sector_start_deg != self.blind_sector_end_deg)

        new_ranges = []
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            # Blind sector check
            if has_blind_sector and (blind_start_rad <= angle <= blind_end_rad):
                new_ranges.append(float('inf'))
                continue

            # Random beam dropout
            if self.dropout_prob > 0.0 and self.rng.random() < self.dropout_prob:
                new_ranges.append(val_dropout)
                continue

            # Valid range processing
            if math.isfinite(r) and r >= msg.range_min and r <= msg.range_max:
                if self.noise_sigma > 0.0:
                    sigma = self.noise_sigma * (1.0 + 0.1 * r) if self.noise_range_proportional else self.noise_sigma
                    noise = self.rng.gauss(0.0, sigma)
                    r_noisy = max(msg.range_min, min(msg.range_max, r + noise))
                    new_ranges.append(r_noisy)
                else:
                    new_ranges.append(r)
            else:
                new_ranges.append(r)

        out_msg.ranges = new_ranges
        out_msg.intensities = list(msg.intensities)

        self._dispatch_scan(out_msg, now)

    def _dispatch_scan(self, msg: LaserScan, now: float):
        if self.delay_ms > 0.0:
            target_time = now + (self.delay_ms / 1000.0)
            self.delayed_queue.append((target_time, msg))
        else:
            self.scan_pub.publish(msg)
            self.published_scans += 1
            self.last_pub_time = now

    def _flush_delayed_queue(self):
        now = time.time()
        while self.delayed_queue and self.delayed_queue[0][0] <= now:
            _, msg = self.delayed_queue.popleft()
            self.scan_pub.publish(msg)
            self.published_scans += 1
            self.last_pub_time = now


def main(args=None):
    rclpy.init(args=args)
    node = ScanFaultInjectorNode()
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
