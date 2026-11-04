import math
from typing import Tuple, Dict, Any


def process_laser_scan_sectors(
    ranges: list,
    angle_min: float,
    angle_increment: float,
    range_min: float = 0.05,
    range_max: float = 30.0,
    front_angle_deg: float = 60.0,
    density_radius: float = 1.30,
) -> Dict[str, Any]:
    """
    Processes raw LaserScan ranges into sector minimum distances and clutter density.
    Returns dictionary with:
      - closest_distance: global minimum valid range
      - closest_angle: angle of closest obstacle
      - direction: 'front', 'left', 'right', 'behind', or 'unknown'
      - front_distance: minimum range in front sector [-front_angle_deg, +front_angle_deg]
      - left_distance: minimum range in left sector [20 deg, 90 deg]
      - right_distance: minimum range in right sector [-90 deg, -20 deg]
      - density_rho: fraction of valid beams within density_radius in front sector
    """
    front_rad = math.radians(front_angle_deg)
    left_min_rad = math.radians(20.0)
    left_max_rad = math.radians(90.0)
    right_min_rad = math.radians(-90.0)
    right_max_rad = math.radians(-20.0)

    global_min_dist = float('inf')
    global_min_angle = 0.0
    front_min_dist = float('inf')
    left_min_dist = float('inf')
    right_min_dist = float('inf')

    n_valid_front = 0
    n_close_front = 0

    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r < range_min or r > range_max:
            continue

        angle = angle_min + i * angle_increment

        if r < global_min_dist:
            global_min_dist = r
            global_min_angle = angle

        # Front sector
        if -front_rad <= angle <= front_rad:
            n_valid_front += 1
            if r < front_min_dist:
                front_min_dist = r
            if r <= density_radius:
                n_close_front += 1

        # Left sector
        if left_min_rad <= angle <= left_max_rad:
            if r < left_min_dist:
                left_min_dist = r

        # Right sector
        if right_min_rad <= angle <= right_max_rad:
            if r < right_min_dist:
                right_min_dist = r

    direction = "unknown"
    if math.isfinite(global_min_dist):
        deg = math.degrees(global_min_angle)
        if -30.0 <= deg <= 30.0:
            direction = "front"
        elif 30.0 < deg <= 150.0:
            direction = "left"
        elif -150.0 <= deg < -30.0:
            direction = "right"
        else:
            direction = "behind"

    density = (float(n_close_front) / float(n_valid_front)) if n_valid_front > 0 else 0.0
    density = max(0.0, min(1.0, density))

    return {
        "closest_distance": global_min_dist,
        "closest_angle": global_min_angle,
        "direction": direction,
        "front_distance": front_min_dist,
        "left_distance": left_min_dist,
        "right_distance": right_min_dist,
        "density_rho": density,
    }


def calculate_adaptive_speed(
    v_req: float,
    omega_req: float,
    front_dist: float,
    density: float,
    threshold: float = 0.45,
    slowdown_dist: float = 2.00,
    density_influence: float = 0.60,
    c_min: float = 0.25,
    max_lin_speed: float = 0.40,
    max_ang_speed: float = 1.00,
    current_state: str = "STARTING",
    adaptive_enabled: bool = True,
) -> Tuple[float, float, float, str]:
    """
    Computes commanded linear and angular velocities based on LiDAR obstacle distance and density,
    with continuous hysteresis-based state machine classification and state speed limits.

    Returns:
        v_cmd: float, commanded translational velocity
        omega_cmd: float, commanded rotational velocity
        scale_factor: float, ratio of v_cmd / v_req (clamped)
        safety_state: str, active behavioral / environmental state
    """
    # 1. Clamp angular velocity
    if max_ang_speed > 0.0:
        omega_clamped = max(-max_ang_speed, min(max_ang_speed, omega_req))
    else:
        omega_clamped = omega_req

    # 2. Reverse motion handling
    if v_req < 0.0:
        v_clamped = max(v_req, -max_lin_speed)
        return v_clamped, omega_clamped, 1.0, "REVERSE"

    # 3. Pure rotation / stopped handling
    if abs(v_req) < 1e-6:
        state = "ROTATING" if abs(omega_clamped) > 1e-6 else "STOPPED"
        return 0.0, omega_clamped, 1.0, state

    # 4. Forward motion clamping
    v_clamped = min(v_req, max_lin_speed)

    # 5. Critical proximity threshold evaluation (emergency stop)
    if front_dist <= threshold:
        return 0.0, 0.0, 0.0, "CRITICAL_PROXIMITY"

    # 6. Check recovery from controlled stop / critical proximity with hysteresis buffer (+0.20m)
    if current_state in ("CRITICAL_PROXIMITY", "CONTROLLED_STOP"):
        if front_dist > (threshold + 0.20):  # e.g. > 0.65m
            v_actual = min(v_clamped, 0.25)
            scale = (v_actual / v_clamped) if v_clamped > 0 else 1.0
            return v_actual, omega_clamped, scale, "PATH_CLEAR"
        else:
            return 0.0, 0.0, 0.0, "CONTROLLED_STOP"
    elif current_state == "PATH_CLEAR":
        new_state = "RESUME_NAVIGATION"
    else:
        new_state = current_state

    # If adaptive scaling is disabled, robot drives at nominal speed unless in emergency
    if not adaptive_enabled:
        return v_clamped, omega_clamped, 1.0, "FIXED_BASELINE"

    # 7. Mathematical model: Distance Scaling Factor D(d)
    # D(d) = 0 for d <= d_safe; (d - d_safe)/(d_slow - d_safe) for d < d_slow; 1.0 for d >= d_slow
    if front_dist < slowdown_dist:
        d_factor = (front_dist - threshold) / (slowdown_dist - threshold)
        d_factor = max(0.0, min(1.0, d_factor))
    else:
        d_factor = 1.0

    # 8. Mathematical model: Obstacle Clutter Factor C(rho)
    # C(rho) = max(c_min, 1 - k * rho)
    dens_clamped = max(0.0, min(1.0, density))
    c_factor = max(c_min, min(1.0, 1.0 - density_influence * dens_clamped))

    # 9. Raw combined adaptive speed
    v_raw = v_clamped * d_factor * c_factor

    # 10. Environmental state classification with hysteresis
    if new_state == "CLUTTERED_CORRIDOR":
        if dens_clamped < 0.25 and front_dist > 1.60:
            new_state = "MODERATE_TRAFFIC"
    elif new_state == "MODERATE_TRAFFIC":
        if dens_clamped >= 0.35 or (front_dist < 1.35 and dens_clamped >= 0.18):
            new_state = "CLUTTERED_CORRIDOR"
        elif dens_clamped < 0.04 and front_dist >= 2.40:
            new_state = "EMPTY_CORRIDOR"
    elif new_state in ("EMPTY_CORRIDOR", "STARTING", "RESUME_NAVIGATION", "PATH_CLEAR"):
        if dens_clamped >= 0.35 or (front_dist < 1.35 and dens_clamped >= 0.20):
            new_state = "CLUTTERED_CORRIDOR"
        elif dens_clamped >= 0.08 or front_dist < 2.20:
            new_state = "MODERATE_TRAFFIC"
        else:
            new_state = "EMPTY_CORRIDOR"
    else:
        if dens_clamped >= 0.35 or (front_dist < 1.35 and dens_clamped >= 0.20):
            new_state = "CLUTTERED_CORRIDOR"
        elif dens_clamped >= 0.08 or front_dist < 2.20:
            new_state = "MODERATE_TRAFFIC"
        else:
            new_state = "EMPTY_CORRIDOR"

    # 11. State-based velocity envelope bounding
    if new_state == "CLUTTERED_CORRIDOR":
        v_actual = max(0.10, min(v_raw, 0.15))
    elif new_state == "MODERATE_TRAFFIC":
        v_actual = max(0.20, min(v_raw, 0.28))
    elif new_state in ("EMPTY_CORRIDOR", "PATH_CLEAR", "RESUME_NAVIGATION"):
        v_actual = min(v_raw, 0.40)
    else:
        v_actual = v_raw

    scale = (v_actual / v_clamped) if v_clamped > 0 else 1.0
    return v_actual, omega_clamped, scale, new_state


def rate_limit_velocity(v_target: float, v_current: float, max_accel: float, dt: float) -> float:
    """
    Limits translational acceleration while permitting instant braking for safety.
    """
    if v_target > v_current:
        max_delta = max_accel * dt
        return min(v_target, v_current + max_delta)
    else:
        # Instant deceleration allowed for rapid safety halting
        return v_target


def compute_distance_factor(front_dist: float, threshold: float = 0.45, slowdown_dist: float = 2.00) -> float:
    """
    Computes distance attenuation factor D(d) in [0.0, 1.0].
    """
    if front_dist <= threshold:
        return 0.0
    if front_dist >= slowdown_dist:
        return 1.0
    factor = (front_dist - threshold) / (slowdown_dist - threshold)
    return max(0.0, min(1.0, factor))


def compute_density_factor(density: float, density_influence: float = 0.60, c_min: float = 0.25) -> float:
    """
    Computes clutter density attenuation factor C(rho) in [c_min, 1.0].
    """
    dens_clamped = max(0.0, min(1.0, density))
    return max(c_min, min(1.0, 1.0 - density_influence * dens_clamped))


def compute_time_to_collision(front_dist: float, current_speed: float) -> float:
    """
    Computes time-to-collision in seconds, or -1.0 if not closing.
    """
    if current_speed > 0.01 and math.isfinite(front_dist) and front_dist > 0.0:
        return float(front_dist / current_speed)
    return -1.0


def compute_limit_reason(
    v_req: float,
    v_cmd_target: float,
    v_cmd_actual: float,
    front_dist: float,
    density: float,
    threshold: float = 0.45,
    slowdown_dist: float = 2.00,
    density_influence: float = 0.60,
    c_min: float = 0.25,
    max_lin_speed: float = 0.40,
    is_sensor_timeout: bool = False,
    adaptive_enabled: bool = True,
) -> str:
    """
    Determines human-readable cause for speed limitation:
      - SENSOR_TIMEOUT
      - EMERGENCY_STOP (d.dd m)
      - OBSTACLE_AHEAD (d.dd m) + CROWDED (rho d.dd)
      - OBSTACLE_AHEAD (d.dd m)
      - CROWDED (rho d.dd)
      - SPEED_CAP (user)
      - ACCEL_LIMIT (ramping)
      - CLEAR
    """
    if is_sensor_timeout:
        return "SENSOR_TIMEOUT"

    if front_dist <= threshold:
        return f"EMERGENCY_STOP ({front_dist:.2f} m)"

    if not adaptive_enabled:
        if v_req > max_lin_speed + 1e-4:
            return "SPEED_CAP (user)"
        return "CLEAR"

    d_factor = compute_distance_factor(front_dist, threshold, slowdown_dist)
    c_factor = compute_density_factor(density, density_influence, c_min)

    obs_limiting = (d_factor < 0.999)
    crowd_limiting = (c_factor < 0.999)

    if obs_limiting and crowd_limiting:
        return f"OBSTACLE_AHEAD ({front_dist:.2f} m) + CROWDED (rho {density:.2f})"
    if obs_limiting:
        return f"OBSTACLE_AHEAD ({front_dist:.2f} m)"
    if crowd_limiting:
        return f"CROWDED (rho {density:.2f})"

    if v_req > max_lin_speed + 1e-4:
        if v_cmd_actual < (max_lin_speed - 0.03):
            return "ACCEL_LIMIT (ramping)"
        return "SPEED_CAP (user)"

    if v_req > 0.01 and v_cmd_actual < (v_cmd_target - 0.03):
        return "ACCEL_LIMIT (ramping)"

    return "CLEAR"


def compute_dynamic_safety_margin(
    v: float,
    d_base: float = 0.45,
    t_reaction: float = 0.10,
    t_scan_latency: float = 0.0,
    a_brake: float = 0.60,
) -> float:
    """
    Speed-dependent safety margin based on vehicle physics:
      d_margin(v) = d_base + v * (t_reaction + t_scan_latency) + v^2 / (2 * a_brake)
    """
    v_fwd = max(0.0, v)
    a_safe = max(0.1, a_brake)
    braking_distance = (v_fwd ** 2) / (2.0 * a_safe)
    reaction_distance = v_fwd * (t_reaction + max(0.0, t_scan_latency))
    return float(d_base + reaction_distance + braking_distance)


def process_footprint_corridor(
    ranges: list,
    angle_min: float,
    angle_increment: float,
    range_min: float = 0.05,
    range_max: float = 30.0,
    footprint_cfg: Dict[str, Any] = None,
    v_cmd: float = 0.0,
    omega_cmd: float = 0.0,
    lookahead: float = 3.50,
) -> Dict[str, Any]:
    """
    Converts valid LiDAR scan points into the robot chassis frame (using sensor offsets)
    and checks obstacle clearance within the dynamic swept corridor.
    """
    if footprint_cfg is None:
        footprint_cfg = {
            'half_width': 0.175,
            'front_offset': 0.20,
            'rear_offset': -0.20,
            'sensor_offset_x': 0.15,
            'sensor_offset_y': 0.0,
            'circumscribed_radius': 0.266,
            'lateral_margin': 0.08,
        }

    half_w = footprint_cfg.get('half_width', 0.175)
    lat_margin = footprint_cfg.get('lateral_margin', 0.08)
    front_offset = footprint_cfg.get('front_offset', 0.20)
    rear_offset = footprint_cfg.get('rear_offset', -0.20)
    sx = footprint_cfg.get('sensor_offset_x', 0.15)
    sy = footprint_cfg.get('sensor_offset_y', 0.0)
    r_circ = footprint_cfg.get('circumscribed_radius', 0.266)

    corridor_half_w = half_w + lat_margin

    min_fwd_clearance = float('inf')
    min_rev_clearance = float('inf')
    min_rot_clearance = float('inf')
    closest_dist = float('inf')
    closest_x = 0.0
    closest_y = 0.0

    n_valid_fwd = 0
    n_clutter_fwd = 0
    density_radius = 1.30

    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r < range_min or r > range_max:
            continue

        angle = angle_min + i * angle_increment
        # Transform from sensor frame to robot base_link frame
        x_r = r * math.cos(angle) + sx
        y_r = r * math.sin(angle) + sy
        dist_origin = math.hypot(x_r, y_r)

        if dist_origin < closest_dist:
            closest_dist = dist_origin
            closest_x = x_r
            closest_y = y_r

        # Rotational clearance relative to circumscribed robot boundary
        rot_clear = dist_origin - (r_circ + lat_margin)
        if rot_clear < min_rot_clearance:
            min_rot_clearance = rot_clear

        # Curvature offset of swept corridor when steering
        y_curv = 0.25 * omega_cmd * max(0.0, x_r - front_offset)
        y_err = abs(y_r - y_curv)

        # Forward corridor
        if front_offset < x_r <= (front_offset + lookahead):
            if y_err <= corridor_half_w:
                clearance = x_r - front_offset
                if clearance < min_fwd_clearance:
                    min_fwd_clearance = clearance

            # Forward clutter evaluation
            if -math.radians(60.0) <= angle <= math.radians(60.0):
                n_valid_fwd += 1
                if dist_origin <= density_radius:
                    n_clutter_fwd += 1

        # Reverse corridor
        if (rear_offset - lookahead) <= x_r < rear_offset:
            if abs(y_r) <= corridor_half_w:
                clearance = rear_offset - x_r
                if clearance < min_rev_clearance:
                    min_rev_clearance = clearance

    density = (float(n_clutter_fwd) / float(n_valid_fwd)) if n_valid_fwd > 0 else 0.0
    density = max(0.0, min(1.0, density))

    return {
        "forward_clearance": min_fwd_clearance,
        "reverse_clearance": min_rev_clearance,
        "rotational_clearance": min_rot_clearance,
        "closest_distance": closest_dist,
        "closest_x": closest_x,
        "closest_y": closest_y,
        "density_rho": density,
    }


# Pure function speed policies
def policy_fixed(v_req: float, d_clear: float, d_margin: float) -> float:
    """Fixed baseline speed policy: drives at nominal requested speed until emergency boundary."""
    if d_clear <= d_margin:
        return 0.0
    return v_req


def policy_distance_linear(v_req: float, d_clear: float, d_margin: float, d_slow: float = 2.00) -> float:
    """Distance-only linear deceleration policy between d_margin and d_slow."""
    if d_clear <= d_margin:
        return 0.0
    if d_clear >= d_slow:
        return v_req
    scale = (d_clear - d_margin) / max(0.05, (d_slow - d_margin))
    return float(v_req * max(0.0, min(1.0, scale)))


def policy_distance_braking(v_req: float, d_clear: float, d_base: float = 0.45, a_brake: float = 0.60) -> float:
    """Physics-based stoppable velocity limit: v_max = sqrt(2 * a_brake * max(0, d_clear - d_base))."""
    if d_clear <= d_base:
        return 0.0
    stopping_dist = max(0.0, d_clear - d_base)
    v_stoppable = math.sqrt(2.0 * max(0.1, a_brake) * stopping_dist)
    return float(min(v_req, v_stoppable))


def policy_density_only(v_req: float, density: float, d_clear: float, d_margin: float, density_influence: float = 0.60, c_min: float = 0.25) -> float:
    """Density-only speed attenuation: S_dens = 1 - k * rho."""
    if d_clear <= d_margin:
        return 0.0
    dens_clamped = max(0.0, min(1.0, density))
    c_factor = max(c_min, min(1.0, 1.0 - density_influence * dens_clamped))
    return float(v_req * c_factor)


def policy_distance_density(v_req: float, d_clear: float, density: float, d_margin: float, d_slow: float = 2.00, density_influence: float = 0.60, c_min: float = 0.25) -> float:
    """Combined distance linear ramp x clutter density attenuation."""
    if d_clear <= d_margin:
        return 0.0
    s_dist = 1.0 if d_clear >= d_slow else max(0.0, min(1.0, (d_clear - d_margin) / max(0.05, (d_slow - d_margin))))
    dens_clamped = max(0.0, min(1.0, density))
    c_factor = max(c_min, min(1.0, 1.0 - density_influence * dens_clamped))
    return float(v_req * s_dist * c_factor)


def policy_distance_density_braking(v_req: float, d_clear: float, density: float, d_base: float = 0.45, a_brake: float = 0.60, density_influence: float = 0.60, c_min: float = 0.25) -> float:
    """Combined physics-based braking limit x clutter density attenuation."""
    if d_clear <= d_base:
        return 0.0
    stopping_dist = max(0.0, d_clear - d_base)
    v_stoppable = math.sqrt(2.0 * max(0.1, a_brake) * stopping_dist)
    dens_clamped = max(0.0, min(1.0, density))
    c_factor = max(c_min, min(1.0, 1.0 - density_influence * dens_clamped))
    return float(min(v_req, v_stoppable) * c_factor)


def _finite(value: float, default: float) -> float:
    """Return a finite scalar; policy inputs must never propagate NaN."""
    return float(value) if isinstance(value, (int, float)) and math.isfinite(value) else default


def smoothstep01(value: float) -> float:
    value = max(0.0, min(1.0, _finite(value, 0.0)))
    return value * value * (3.0 - 2.0 * value)


def layered_braking_cap(clearance: float, d0: float, a_brake: float, latency: float) -> float:
    """Maximum speed that can stop before ``d0`` after measured latency.

    Solves d_c - d0 = v*t_lat + v^2/(2*a) for the non-negative root.
    """
    clearance = _finite(clearance, 0.0)
    available = clearance - max(0.0, _finite(d0, 0.45))
    if available <= 0.0:
        return 0.0
    a = max(1e-3, _finite(a_brake, 0.6))
    t = max(0.0, _finite(latency, 0.0))
    return max(0.0, -a * t + math.sqrt((a * t) ** 2 + 2.0 * a * available))


def policy_layered_governor(
    v_req: float, d_clear: float, rho_path: float, lateral_free_width: float,
    curvature: float, closing_speed: float, distance_to_goal: float,
    scan_latency: float = 0.0, confidence: float = 1.0,
    profile_caps: Tuple[Tuple[float, float], ...] = (),
    d0: float = 0.45, comfort_distance: float = 2.0, a_brake: float = 0.60,
    a_lat: float = 0.50, ttc_min: float = 1.0, goal_distance: float = 1.0,
    v_creep: float = 0.05, rho_gain: float = 0.60,
    robot_width: float = 0.35, lateral_margin: float = 0.08,
    a_dec: float = 0.80,
) -> Dict[str, float]:
    """Pure layered speed governor.

    Hard limits (braking/TTC) are intentionally independent of confidence.
    ``profile_caps`` is an iterable of ``(distance_along_path, cap)`` pairs.
    """
    request = max(0.0, _finite(v_req, 0.0))
    clearance = _finite(d_clear, 0.0)
    safety = layered_braking_cap(clearance, d0, a_brake, scan_latency)
    if not math.isfinite(safety):
        safety = 0.0
    # Comfort is soft: it eases from creep to request, and confidence affects
    # only this term, never the physical stopping envelope.
    span = max(1e-3, _finite(comfort_distance, 2.0) - d0)
    comfort_scale = smoothstep01((clearance - d0) / span)
    confidence = max(0.0, min(1.0, _finite(confidence, 0.0)))
    comfort = request * (confidence * comfort_scale)
    width_needed = max(1e-3, _finite(robot_width, .35) + 2.0 * max(0.0, _finite(lateral_margin, .08)))
    width_scale = smoothstep01((_finite(lateral_free_width, 0.0) - width_needed) / width_needed)
    lateral_cap = request * width_scale
    clutter_cap = request * max(0.0, 1.0 - max(0.0, min(1.0, _finite(rho_path, 1.0))) * max(0.0, _finite(rho_gain, .6)))
    kappa = abs(_finite(curvature, 0.0))
    turn_cap = request if kappa < 1e-6 else math.sqrt(max(1e-3, _finite(a_lat, .5)) / kappa)
    closing = max(0.0, _finite(closing_speed, 0.0))
    ttc_cap = request if closing <= 1e-6 else max(0.0, clearance / max(1e-3, _finite(ttc_min, 1.0)))
    goal = max(0.0, _finite(distance_to_goal, 0.0))
    goal_cap = request * smoothstep01(goal / max(1e-3, _finite(goal_distance, 1.0)))
    profile_cap = request
    for distance, cap in profile_caps:
        distance, cap = _finite(distance, float('inf')), max(0.0, _finite(cap, 0.0))
        if distance >= 0.0:
            profile_cap = min(profile_cap, math.sqrt(cap * cap + 2.0 * max(1e-3, _finite(a_dec, .8)) * distance))
    lookahead = min(profile_cap, lateral_cap, clutter_cap, turn_cap, ttc_cap, goal_cap)
    soft = max(max(0.0, _finite(v_creep, .05)), min(comfort, lookahead))
    target = min(request, safety, soft)
    # Never create a creep command through a hard stop.
    if safety <= 0.0 or ttc_cap <= 0.0:
        target = 0.0
    return {'speed': max(0.0, target), 'safety_cap': safety, 'comfort_cap': comfort,
            'lookahead_cap': lookahead, 'ttc_cap': ttc_cap, 'turn_cap': turn_cap}


def evaluate_speed_policy(
    policy_name: str,
    v_req: float,
    d_clear: float,
    density: float,
    d_margin: float,
    d_slow: float = 2.00,
    d_base: float = 0.45,
    a_brake: float = 0.60,
    density_influence: float = 0.60,
    c_min: float = 0.25, **kwargs,
) -> float:
    """Selects and computes commanded speed for the requested policy name."""
    name = policy_name.strip().lower()
    if name == 'fixed':
        return policy_fixed(v_req, d_clear, d_margin)
    elif name == 'distance_linear':
        return policy_distance_linear(v_req, d_clear, d_margin, d_slow)
    elif name == 'distance_braking':
        return policy_distance_braking(v_req, d_clear, d_base, a_brake)
    elif name == 'density_only':
        return policy_density_only(v_req, density, d_clear, d_margin, density_influence, c_min)
    elif name == 'distance_density':
        return policy_distance_density(v_req, d_clear, density, d_margin, d_slow, density_influence, c_min)
    elif name in ('distance_density_braking', 'braking_density'):
        return policy_distance_density_braking(v_req, d_clear, density, d_base, a_brake, density_influence, c_min)
    elif name == 'layered_governor':
        return policy_layered_governor(
            v_req=v_req, d_clear=d_clear, rho_path=kwargs.get('rho_path', density),
            lateral_free_width=kwargs.get('lateral_free_width', float('inf')),
            curvature=kwargs.get('curvature', 0.0), closing_speed=kwargs.get('closing_speed', 0.0),
            distance_to_goal=kwargs.get('distance_to_goal', float('inf')),
            scan_latency=kwargs.get('scan_latency', 0.0), confidence=kwargs.get('confidence', 1.0),
            profile_caps=kwargs.get('profile_caps', ()), d0=d_base, a_brake=a_brake,
            comfort_distance=kwargs.get('comfort_distance', d_slow),
            v_creep=kwargs.get('v_creep', .05), a_dec=kwargs.get('a_dec', a_brake),
        )['speed']
    else:
        # Default to distance_density
        return policy_distance_density(v_req, d_clear, density, d_margin, d_slow, density_influence, c_min)
