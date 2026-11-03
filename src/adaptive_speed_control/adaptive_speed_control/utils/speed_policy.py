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


