#!/usr/bin/env python3
import math
import os
import time
from collections import deque
from typing import Dict, Any

import yaml
import rclpy
from rclpy.lifecycle import Node as LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String, Bool
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

import diagnostic_updater
import diagnostic_msgs.msg

from adaptive_speed_interfaces.msg import ObstacleInfo
from adaptive_speed_interfaces.srv import SetThreshold
from adaptive_speed_control.clear_barrier import move_barrier
from adaptive_speed_control.utils.speed_policy import (
    calculate_adaptive_speed,
    process_laser_scan_sectors,
    process_footprint_corridor,
    compute_dynamic_safety_margin,
    evaluate_speed_policy,
    rate_limit_velocity,
    compute_distance_factor,
    compute_density_factor,
    compute_limit_reason,
    compute_time_to_collision,
)


class AdaptiveSpeedControllerNode(LifecycleNode):
    """
    ROS 2 Lifecycle Adaptive Speed Controller node for mobile robots in cluttered environments.
    - Subscribes to /scan (best effort QoS), /cmd_vel_in, and /odom.
    - Evaluates footprint-aware swept corridor obstacle clearance and clutter density.
    - Implements physics-based speed-dependent safety margins and multiple selectable speed policies.
    - Lifecycle-managed (configure, activate, deactivate, cleanup).
    - Sole publisher of final motion commands to /cmd_vel.
    - Enforces rate-limited acceleration and safe timeouts.
    - Publishes diagnostics on /diagnostics.
    """

    def __init__(self, auto_activate: bool = True):
        super().__init__('adaptive_speed_controller_node')

        # Parameters
        self.declare_parameter('speed_policy', 'distance_density')
        self.declare_parameter('threshold', 0.45)
        self.declare_parameter('slowdown_distance', 2.00)
        self.declare_parameter('density_radius', 1.30)
        self.declare_parameter('density_influence', 0.60)
        self.declare_parameter('c_min', 0.25)
        self.declare_parameter('max_linear_speed', 0.40)
        self.declare_parameter('max_angular_speed', 1.00)
        self.declare_parameter('max_acceleration', 0.60)
        self.declare_parameter('t_reaction', 0.10)
        self.declare_parameter('front_angle_deg', 60.0)
        self.declare_parameter('sensor_timeout', 0.50)
        self.declare_parameter('max_scan_age', 0.35)
        self.declare_parameter('cmd_timeout', 0.50)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('adaptive_enabled', True)
        self.declare_parameter('enable_corridor_stop', False)
        self.declare_parameter('use_footprint_corridor', True)
        self.declare_parameter('governor_v_creep', 0.05)
        self.declare_parameter('governor_comfort_distance', 2.00)
        self.declare_parameter('governor_a_brake', 0.60)
        self.declare_parameter('governor_a_dec', 0.80)
        self.declare_parameter('governor_a_lat', 0.50)
        self.declare_parameter('governor_ttc_min', 1.00)
        self.declare_parameter('governor_goal_distance', 1.00)
        self.declare_parameter('governor_rho_gain', 0.60)

        self._load_parameters()
        self._load_footprint_config()
        self.add_on_set_parameters_callback(self._on_parameters_set)

        # State tracking
        self.requested_cmd = Twist()
        self.last_cmd_time = 0.0
        self.last_scan_recv_time = 0.0
        # Compatibility alias retained for tools/tests that inspect arrival age.
        self.last_scan_time = 0.0
        self.last_scan_header_time = 0.0
        self.scan_latency = 0.0

        self.robot_x = -9.0
        self.robot_y = 0.0

        self.closest_distance = float('inf')
        self.closest_direction = 'unknown'
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.corridor_clearance = float('inf')
        self.effective_clearance = float('inf')
        self.obstacle_density = 0.0
        self.safety_state = 'STARTING'

        self.limit_reason = "CLEAR"
        self.scale_distance = 1.0
        self.scale_density = 1.0
        self.speed_scale = 1.0
        self.time_to_collision = -1.0
        self.effective_margin = self.threshold
        self.control_mode = "MANUAL"
        # Policy output before the acceleration ramp. Dashboards use this to
        # show the live safety limit rather than the configured maximum.
        self.safe_speed_limit = 0.0

        # Conservative 2-of-3 scan temporal filter
        self.clearance_history = deque(maxlen=3)

        self.last_cmd_linear = 0.0
        self.last_log_time = 0.0
        self.last_logged_state = None

        # Setup ROS communications
        self._setup_communications()

        # Diagnostic Updater
        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID("adaptive_speed_controller")
        self.updater.add("Adaptive Speed Controller", self._produce_diagnostics)

        # Standalone auto-activation
        if auto_activate:
            self.trigger_configure()
            self.trigger_activate()

    def _load_footprint_config(self):
        self.footprint_cfg = {
            'length': 0.40,
            'width': 0.35,
            'half_width': 0.175,
            'front_offset': 0.20,
            'rear_offset': -0.20,
            'sensor_offset_x': 0.15,
            'sensor_offset_y': 0.0,
            'circumscribed_radius': 0.266,
            'lateral_margin': 0.08,
            'a_brake_max': 0.60,
            't_reaction': 0.10,
        }
        try:
            pkg_share = get_package_share_directory('adaptive_speed_control')
            fp_path = os.path.join(pkg_share, 'config', 'robot_footprint.yaml')
            if os.path.exists(fp_path):
                with open(fp_path, 'r') as f:
                    doc = yaml.safe_load(f)
                    if doc and 'robot_footprint' in doc:
                        self.footprint_cfg.update(doc['robot_footprint'])
        except Exception as e:
            self.get_logger().warn(f"Could not load robot_footprint.yaml: {e}")

    def _setup_communications(self):
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data
        )
        self.cmd_in_sub = self.create_subscription(
            Twist, '/cmd_vel_in', self.on_cmd_vel_in, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.on_odom, 10
        )
        self.mode_sub = self.create_subscription(
            String, '/control_mode', self.on_control_mode, 10
        )
        self.adaptive_toggle_sub = self.create_subscription(
            Bool, '/toggle_adaptive', self.on_toggle_adaptive, 10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.info_pub = self.create_publisher(ObstacleInfo, '/obstacle_info', 10)
        self.density_pub = self.create_publisher(Float32, '/obstacle_density', 10)
        self.safe_limit_pub = self.create_publisher(Float32, '/safe_speed_limit', 10)

        # Services
        self.th_srv = self.create_service(
            SetThreshold, '/set_threshold', self.handle_set_threshold
        )
        self.clear_barrier_srv = self.create_service(
            Trigger, '/clear_barrier', self.handle_clear_barrier
        )

        timer_period = 1.0 / max(1.0, self.control_rate)
        self.timer = self.create_timer(timer_period, self.control_loop)

    # Lifecycle Callbacks
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("[LIFECYCLE] Configuring Adaptive Speed Controller Node...")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("[LIFECYCLE] Activating Adaptive Speed Controller Node...")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().warn("[LIFECYCLE] Deactivating controller -> Publishing immediate ZERO /cmd_vel!")
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("[LIFECYCLE] Cleaning up Adaptive Speed Controller Node...")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().warn("[LIFECYCLE] Shutting down controller -> Publishing immediate ZERO /cmd_vel!")
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass
        return TransitionCallbackReturn.SUCCESS

    def _load_parameters(self):
        self.speed_policy = str(self.get_parameter('speed_policy').value).strip().lower()
        self.threshold = float(self.get_parameter('threshold').value)
        self.slowdown_distance = float(self.get_parameter('slowdown_distance').value)
        self.density_radius = float(self.get_parameter('density_radius').value)
        self.density_influence = float(self.get_parameter('density_influence').value)
        self.c_min = float(self.get_parameter('c_min').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.max_acceleration = float(self.get_parameter('max_acceleration').value)
        self.t_reaction = float(self.get_parameter('t_reaction').value)
        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)
        self.sensor_timeout = float(self.get_parameter('sensor_timeout').value)
        self.max_scan_age = float(self.get_parameter('max_scan_age').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.adaptive_enabled = bool(self.get_parameter('adaptive_enabled').value)
        self.enable_corridor_stop = bool(self.get_parameter('enable_corridor_stop').value)
        self.use_footprint_corridor = bool(self.get_parameter('use_footprint_corridor').value)
        self.governor_v_creep = float(self.get_parameter('governor_v_creep').value)
        self.governor_comfort_distance = float(self.get_parameter('governor_comfort_distance').value)
        self.governor_a_brake = float(self.get_parameter('governor_a_brake').value)
        self.governor_a_dec = float(self.get_parameter('governor_a_dec').value)
        self.governor_a_lat = float(self.get_parameter('governor_a_lat').value)
        self.governor_ttc_min = float(self.get_parameter('governor_ttc_min').value)
        self.governor_goal_distance = float(self.get_parameter('governor_goal_distance').value)
        self.governor_rho_gain = float(self.get_parameter('governor_rho_gain').value)

    def _on_parameters_set(self, params):
        for param in params:
            if param.name == 'threshold':
                if param.value <= 0.0 or not math.isfinite(param.value) or param.value >= self.slowdown_distance:
                    return SetParametersResult(successful=False, reason="threshold must be > 0 and < slowdown_distance")
                self.threshold = float(param.value)
            elif param.name == 'slowdown_distance':
                if param.value <= self.threshold or not math.isfinite(param.value):
                    return SetParametersResult(successful=False, reason="slowdown_distance must be > threshold")
                self.slowdown_distance = float(param.value)
            elif param.name == 'speed_policy':
                self.speed_policy = str(param.value).strip().lower()
                self.get_logger().info(f"Speed policy set to: {self.speed_policy}")
            elif param.name == 'density_influence':
                if param.value < 0.0 or param.value > 1.0:
                    return SetParametersResult(successful=False, reason="density_influence must be between 0.0 and 1.0")
                self.density_influence = float(param.value)
            elif param.name == 'adaptive_enabled':
                self.adaptive_enabled = bool(param.value)
            elif param.name == 'enable_corridor_stop':
                self.enable_corridor_stop = bool(param.value)
        return SetParametersResult(successful=True)

    def _produce_diagnostics(self, stat: diagnostic_updater.DiagnosticStatusWrapper):
        if self.safety_state in ("CRITICAL_PROXIMITY", "CONTROLLED_STOP"):
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, f"Emergency Stop: {self.limit_reason}")
        elif self.safety_state == "SENSOR_TIMEOUT":
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "LiDAR Sensor Timeout")
        elif self.scan_latency > 0.15:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, f"High LiDAR Latency: {self.scan_latency*1000:.1f}ms")
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, f"Nominal ({self.safety_state})")

        stat.add("Active Policy", str(self.speed_policy))
        stat.add("Safety State", str(self.safety_state))
        stat.add("Limit Reason", str(self.limit_reason))
        stat.add("Scan Latency (ms)", f"{self.scan_latency * 1000:.1f}")
        stat.add("Effective Margin (m)", f"{self.effective_margin:.3f}")
        stat.add("Corridor Clearance (m)", f"{self.effective_clearance:.3f}")
        stat.add("Clutter Density (rho)", f"{self.obstacle_density:.3f}")
        stat.add("Last Commanded Speed (m/s)", f"{self.last_cmd_linear:.3f}")
        return stat

    def handle_set_threshold(self, request, response):
        new_th = request.threshold
        if new_th <= 0.0 or not math.isfinite(new_th) or new_th >= self.slowdown_distance:
            response.success = False
            response.message = f"Invalid threshold ({new_th:.2f}m). Must be >0 and < slowdown_distance."
            return response

        self.threshold = float(new_th)
        self.set_parameters([Parameter('threshold', Parameter.Type.DOUBLE, self.threshold)])
        response.success = True
        response.message = f"Base safety threshold (d_base) updated to {self.threshold:.3f} m."
        self.get_logger().info(response.message)
        return response

    def handle_clear_barrier(self, request, response):
        success = move_barrier(10.0)
        response.success = success
        if success:
            response.message = "Zone 4 barrier cleared from path. Robot will resume automatically."
            self.get_logger().info(response.message)
            if self.safety_state in ("CRITICAL_PROXIMITY", "CONTROLLED_STOP"):
                self.safety_state = "PATH_CLEAR"
        else:
            response.message = "Failed to contact Gazebo service to move barrier."
            self.get_logger().warn(response.message)
        return response

    def on_odom(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def on_control_mode(self, msg: String):
        self.control_mode = msg.data.upper()

    def on_toggle_adaptive(self, msg: Bool):
        self.adaptive_enabled = bool(msg.data)
        self.get_logger().info(f"Adaptive scaling toggled to: {'ON' if self.adaptive_enabled else 'OFF (emergency stop active)'}")

    def on_cmd_vel_in(self, msg: Twist):
        self.requested_cmd = msg
        self.last_cmd_time = time.time()

    def on_scan(self, scan: LaserScan):
        if not scan.ranges:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        header_sec = scan.header.stamp.sec + (scan.header.stamp.nanosec * 1e-9)

        self.last_scan_recv_time = time.time()
        self.last_scan_time = self.last_scan_recv_time
        self.last_scan_header_time = header_sec
        self.scan_latency = max(0.0, now_sec - header_sec)

        min_range = scan.range_min if (math.isfinite(scan.range_min) and scan.range_min > 0.0) else 0.05
        max_range = scan.range_max if (math.isfinite(scan.range_max) and scan.range_max > 0.0) else 30.0

        # Multi-sector processing
        sectors = process_laser_scan_sectors(
            ranges=scan.ranges,
            angle_min=scan.angle_min,
            angle_increment=scan.angle_increment,
            range_min=min_range,
            range_max=max_range,
            front_angle_deg=self.front_angle_deg,
            density_radius=self.density_radius,
        )

        # Footprint-aware swept corridor processing
        corridor = process_footprint_corridor(
            ranges=scan.ranges,
            angle_min=scan.angle_min,
            angle_increment=scan.angle_increment,
            range_min=min_range,
            range_max=max_range,
            footprint_cfg=self.footprint_cfg,
            v_cmd=self.last_cmd_linear,
            omega_cmd=self.requested_cmd.angular.z,
            lookahead=self.slowdown_distance + 1.0,
        )

        self.closest_distance = corridor["closest_distance"]
        self.closest_direction = sectors["direction"]
        self.front_distance = sectors["front_distance"]
        self.left_distance = sectors["left_distance"]
        self.right_distance = sectors["right_distance"]
        self.obstacle_density = corridor["density_rho"]

        # Swept corridor clearance with conservative 2-of-3 scan temporal filter
        raw_clearance = corridor["forward_clearance"] if self.use_footprint_corridor else self.front_distance
        self.corridor_clearance = raw_clearance
        self.clearance_history.append(raw_clearance)

        # Conservative rule: an obstacle seen within safety buffer in 2 of last 3 scans cannot be erased by a single dropout
        close_seen_count = sum(1 for c in self.clearance_history if c <= (self.threshold + 0.20))
        if close_seen_count >= 2 and raw_clearance > (self.threshold + 0.40):
            self.effective_clearance = min(self.clearance_history)
        else:
            self.effective_clearance = raw_clearance

        self._publish_obstacle_info()

        dens_msg = Float32()
        dens_msg.data = float(self.obstacle_density)
        self.density_pub.publish(dens_msg)

    def _publish_obstacle_info(self):
        info = ObstacleInfo()
        info.closest_distance = float(self.closest_distance)
        info.direction = str(self.closest_direction)
        info.threshold = float(self.threshold)
        info.front_distance = float(self.front_distance)
        info.left_distance = float(self.left_distance)
        info.right_distance = float(self.right_distance)
        info.density_rho = float(self.obstacle_density)
        info.zone = str(self.safety_state)
        info.limit_reason = str(self.limit_reason)
        info.scale_distance = float(self.scale_distance)
        info.scale_density = float(self.scale_density)
        info.speed_scale = float(self.speed_scale)
        info.time_to_collision = float(self.time_to_collision)
        info.clearance = float(self.effective_clearance)
        self.info_pub.publish(info)

    def _publish_safe_speed_limit(self, speed: float):
        """Publish the current forward policy limit for dashboards and logs."""
        self.safe_speed_limit = max(0.0, float(speed))
        msg = Float32()
        msg.data = self.safe_speed_limit
        self.safe_limit_pub.publish(msg)

    def control_loop(self):
        # Enforce lifecycle state: only publish while active!
        if hasattr(self, '_state_machine') and self._state_machine.current_state[1] != 'active':
            return

        now = time.time()
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # Check destination reached (optional corridor end stop)
        if self.enable_corridor_stop and self.robot_x >= 8.8:
            self.safety_state = "DESTINATION_REACHED"
            self.limit_reason = "CLEAR"
            self.cmd_pub.publish(Twist())
            self._publish_safe_speed_limit(0.0)
            self._publish_obstacle_info()
            self._log_status(0.0, 0.0, 0.0)
            return

        # Fail-safe 1: Sensor staleness from message HEADER stamp (sim time aware)
        scan_age = (now_sec - self.last_scan_header_time) if self.last_scan_header_time > 0 else (now - self.last_scan_recv_time)
        if scan_age > self.max_scan_age or (now - self.last_scan_time) > self.sensor_timeout:
            self.safety_state = "SENSOR_TIMEOUT"
            self.limit_reason = "SENSOR_TIMEOUT"
            self.scale_distance = 0.0
            self.scale_density = 0.0
            self.speed_scale = 0.0
            self.time_to_collision = -1.0
            self.cmd_pub.publish(Twist())
            self._publish_safe_speed_limit(0.0)
            self._publish_obstacle_info()
            self._log_status(0.0, 0.0, 0.0)
            return

        # Fail-safe 2: Upstream command timeout
        if (now - self.last_cmd_time) > self.cmd_timeout:
            self.safety_state = "IDLE"
            if self.effective_clearance <= self.threshold:
                self.limit_reason = f"EMERGENCY_STOP ({self.effective_clearance:.2f} m)"
            else:
                self.limit_reason = "CLEAR"
            self.scale_distance = compute_distance_factor(self.effective_clearance, self.threshold, self.slowdown_distance)
            self.scale_density = compute_density_factor(self.obstacle_density, self.density_influence, self.c_min)
            self.speed_scale = 0.0
            self.time_to_collision = -1.0
            self.cmd_pub.publish(Twist())
            self._publish_safe_speed_limit(0.0)
            self._publish_obstacle_info()
            self._log_status(0.0, 0.0, 0.0)
            return

        v_req = self.requested_cmd.linear.x
        omega_req = self.requested_cmd.angular.z

        # Clamp angular velocity
        if self.max_angular_speed > 0.0:
            omega_cmd = max(-self.max_angular_speed, min(self.max_angular_speed, omega_req))
        else:
            omega_cmd = omega_req

        # Dynamic physics-based safety margin d_margin(v)
        d_margin = compute_dynamic_safety_margin(
            v=abs(self.last_cmd_linear),
            d_base=self.threshold,
            t_reaction=self.t_reaction,
            t_scan_latency=self.scan_latency,
            a_brake=self.max_acceleration
        )
        self.effective_margin = d_margin

        # Critical proximity evaluation (EMERGENCY STOP)
        if self.effective_clearance <= self.threshold:
            self.safety_state = "CRITICAL_PROXIMITY"
            self.limit_reason = f"EMERGENCY_STOP ({self.front_distance:.2f} m)"
            self.last_cmd_linear = 0.0
            self.cmd_pub.publish(Twist())
            self._publish_safe_speed_limit(0.0)
            self.scale_distance = 0.0
            self.speed_scale = 0.0
            self._publish_obstacle_info()
            self._log_status(v_req, 0.0, 0.0)
            return

        # Reverse motion handling
        if v_req < 0.0:
            v_cmd = max(v_req, -self.max_linear_speed)
            self.safety_state = "REVERSE"
            self.limit_reason = "CLEAR"
            cmd = Twist()
            cmd.linear.x = float(v_cmd)
            cmd.angular.z = float(omega_cmd)
            self.cmd_pub.publish(cmd)
            self.last_cmd_linear = v_cmd
            self._publish_safe_speed_limit(abs(v_cmd))
            self._publish_obstacle_info()
            return

        # Pure rotation handling
        if abs(v_req) < 1e-5:
            self.safety_state = "ROTATING" if abs(omega_cmd) > 1e-4 else "STOPPED"
            self.limit_reason = "CLEAR"
            cmd = Twist()
            cmd.angular.z = float(omega_cmd)
            self.cmd_pub.publish(cmd)
            self.last_cmd_linear = 0.0
            self._publish_safe_speed_limit(0.0)
            self._publish_obstacle_info()
            return

        # Forward motion: evaluate chosen speed policy
        v_clamped = min(v_req, self.max_linear_speed)

        if not self.adaptive_enabled:
            # Baseline fixed speed mode (emergency stop and failsafes still strictly active)
            v_target = v_clamped
            next_state = "FIXED_BASELINE"
        else:
            v_target = evaluate_speed_policy(
                policy_name=self.speed_policy,
                v_req=v_clamped,
                d_clear=self.effective_clearance,
                density=self.obstacle_density,
                d_margin=d_margin,
                d_slow=self.slowdown_distance,
                d_base=self.threshold,
                a_brake=self.max_acceleration,
                density_influence=self.density_influence,
                c_min=self.c_min,
                scan_latency=self.scan_latency,
                closing_speed=max(0.0, self.last_cmd_linear),
                comfort_distance=self.governor_comfort_distance,
                v_creep=self.governor_v_creep,
                a_dec=self.governor_a_dec,
                a_lat=self.governor_a_lat,
                ttc_min=self.governor_ttc_min,
                rho_gain=self.governor_rho_gain,
            )
            # Environmental zone classification
            if self.effective_clearance < 1.35 or self.obstacle_density >= 0.30:
                next_state = "CLUTTERED_CORRIDOR"
            elif self.effective_clearance < 2.20 or self.obstacle_density >= 0.08:
                next_state = "MODERATE_TRAFFIC"
            else:
                next_state = "EMPTY_CORRIDOR"

        # Rate limit acceleration (instant deceleration permitted for safety)
        dt = 1.0 / max(1.0, self.control_rate)
        v_cmd = rate_limit_velocity(
            v_target=v_target,
            v_current=self.last_cmd_linear,
            max_accel=self.max_acceleration,
            dt=dt
        )

        self.last_cmd_linear = v_cmd
        self.safety_state = next_state
        self._publish_safe_speed_limit(v_target)

        # Diagnostics telemetry fields
        self.scale_distance = compute_distance_factor(self.effective_clearance, self.threshold, self.slowdown_distance)
        self.scale_density = compute_density_factor(self.obstacle_density, self.density_influence, self.c_min)
        self.speed_scale = (v_cmd / v_clamped) if v_clamped > 0 else 1.0
        self.time_to_collision = compute_time_to_collision(self.effective_clearance, abs(v_cmd))
        self.limit_reason = compute_limit_reason(
            v_req=v_req,
            v_cmd_target=v_target,
            v_cmd_actual=v_cmd,
            # Report the measured front ray in the human-facing reason while
            # policy decisions continue to use footprint clearance above.
            front_dist=self.front_distance,
            density=self.obstacle_density,
            threshold=self.threshold,
            slowdown_dist=self.slowdown_distance,
            density_influence=self.density_influence,
            c_min=self.c_min,
            max_lin_speed=self.max_linear_speed,
            is_sensor_timeout=False,
            adaptive_enabled=self.adaptive_enabled,
        )

        cmd = Twist()
        cmd.linear.x = float(v_cmd)
        cmd.angular.z = float(omega_cmd)
        self.cmd_pub.publish(cmd)

        self._publish_obstacle_info()
        self._log_status(v_req, v_cmd, self.speed_scale)

    def _log_status(self, v_req, v_cmd, scale):
        now = time.time()
        state_changed = (self.safety_state != self.last_logged_state)
        time_elapsed = (now - self.last_log_time) >= 1.0

        if not (state_changed or time_elapsed):
            return

        self.last_log_time = now
        self.last_logged_state = self.safety_state

        if self.safety_state == "DESTINATION_REACHED":
            self.get_logger().info(
                f"[SUCCESS] DESTINATION_REACHED | Pos: ({self.robot_x:.2f}, {self.robot_y:.2f}) | Robot Arrived Safely!"
            )
        elif self.safety_state in ("CRITICAL_PROXIMITY", "CONTROLLED_STOP"):
            self.get_logger().warn(
                f"[WARNING] CRITICAL_PROXIMITY | Clearance: {self.effective_clearance:.2f}m <= {self.threshold:.2f}m | Robot stopped"
            )
        elif self.safety_state == "SENSOR_TIMEOUT":
            self.get_logger().warn(
                "[STATUS] SENSOR_TIMEOUT | No recent LiDAR data or scan too stale! Robot halted for safety."
            )
        elif self.safety_state == "IDLE":
            self.get_logger().info("[STATUS] IDLE | Waiting for commands on /cmd_vel_in...")
        else:
            self.get_logger().info(
                f"[STATUS] {self.safety_state:<18} | Policy: {self.speed_policy} | Clearance: {self.effective_clearance:.2f}m | "
                f"Margin: {self.effective_margin:.2f}m | Density: {self.obstacle_density:.2f} | Speed: {v_cmd:.2f} m/s"
            )


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveSpeedControllerNode(auto_activate=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
