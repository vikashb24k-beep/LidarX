import math
import pytest
from adaptive_speed_control.utils.speed_policy import (
    calculate_adaptive_speed,
    process_laser_scan_sectors,
    rate_limit_velocity,
)
from adaptive_speed_control.utils.geometry_utils import (
    normalize_angle,
    cross_track_error,
    euclidean_distance,
)


def test_boundary_conditions_zero_and_infinite_distance():
    """Validates velocity behavior at zero distance, threshold, and infinity."""
    # At zero distance -> emergency stop
    v_zero, _, _, s_zero = calculate_adaptive_speed(
        v_req=0.40, omega_req=0.0, front_dist=0.0, density=0.0, threshold=0.45, slowdown_dist=2.00
    )
    assert v_zero == 0.0
    assert s_zero == "CRITICAL_PROXIMITY"

    # Exactly at threshold -> emergency stop
    v_th, _, _, s_th = calculate_adaptive_speed(
        v_req=0.40, omega_req=0.0, front_dist=0.45, density=0.0, threshold=0.45, slowdown_dist=2.00
    )
    assert v_th == 0.0
    assert s_th == "CRITICAL_PROXIMITY"

    # At infinite distance -> maximum allowed speed
    v_inf, _, _, s_inf = calculate_adaptive_speed(
        v_req=0.40, omega_req=0.0, front_dist=float('inf'), density=0.0, threshold=0.45, slowdown_dist=2.00
    )
    assert math.isclose(v_inf, 0.40, abs_tol=1e-3)
    assert s_inf == "EMPTY_CORRIDOR"


def test_monotonicity_distance_scaling():
    """Ensures velocity monotonically decreases as obstacle proximity decreases."""
    test_distances = [4.0, 3.0, 2.0, 1.8, 1.5, 1.2, 0.9, 0.6, 0.46]
    speeds = []
    for d in test_distances:
        v, _, _, _ = calculate_adaptive_speed(
            v_req=0.40, omega_req=0.0, front_dist=d, density=0.1, threshold=0.45, slowdown_dist=2.00
        )
        speeds.append(v)

    # Monotonicity check: each subsequent speed must be <= previous speed
    for i in range(len(speeds) - 1):
        assert speeds[i] >= speeds[i + 1] - 1e-4, f"Speed increased from {speeds[i]} to {speeds[i+1]} as distance decreased!"


def test_monotonicity_density_scaling():
    """Ensures velocity monotonically decreases as obstacle density increases."""
    test_densities = [0.0, 0.1, 0.25, 0.4, 0.6, 0.8, 1.0]
    speeds = []
    for rho in test_densities:
        v, _, _, _ = calculate_adaptive_speed(
            v_req=0.40, omega_req=0.0, front_dist=1.8, density=rho, threshold=0.45, slowdown_dist=2.00
        )
        speeds.append(v)

    for i in range(len(speeds) - 1):
        assert speeds[i] >= speeds[i + 1] - 1e-4, f"Speed increased from {speeds[i]} to {speeds[i+1]} as density increased!"


def test_emergency_stop_at_threshold():
    """Emergency stop triggers immediately whenever front_dist <= threshold."""
    v, omega, scale, state = calculate_adaptive_speed(
        v_req=0.40, omega_req=0.5, front_dist=0.449, density=0.0, threshold=0.45
    )
    assert v == 0.0
    assert omega == 0.0
    assert scale == 0.0
    assert state == "CRITICAL_PROXIMITY"


def test_hysteresis_state_transitions():
    """Tests that transitions between states avoid jitter through hysteresis."""
    # Robot stopped in CONTROLLED_STOP
    # At front_dist = 0.55m (above threshold 0.45m but within buffer 0.65m), must hold stop
    v1, _, _, s1 = calculate_adaptive_speed(
        v_req=0.40, omega_req=0.0, front_dist=0.55, density=0.5, threshold=0.45, current_state="CONTROLLED_STOP"
    )
    assert v1 == 0.0
    assert s1 == "CONTROLLED_STOP"

    # Once front_dist exceeds 0.65m (threshold + 0.20m), triggers PATH_CLEAR
    v2, _, _, s2 = calculate_adaptive_speed(
        v_req=0.40, omega_req=0.0, front_dist=0.70, density=0.2, threshold=0.45, current_state="CONTROLLED_STOP"
    )
    assert v2 > 0.0
    assert s2 == "PATH_CLEAR"


def test_rate_limiter_acceleration_and_braking():
    """Tests acceleration limiter ramps up smoothly, but braking is instantaneous."""
    dt = 0.05
    max_accel = 0.50

    # Accelerating from 0 to 0.40 m/s: limited to 0.5 * 0.05 = 0.025 m/s step
    v_next = rate_limit_velocity(v_target=0.40, v_current=0.0, max_accel=max_accel, dt=dt)
    assert math.isclose(v_next, 0.025, abs_tol=1e-4)

    # Braking from 0.40 to 0.0 m/s: instant zero command for safety
    v_brake = rate_limit_velocity(v_target=0.0, v_current=0.40, max_accel=max_accel, dt=dt)
    assert v_brake == 0.0


def test_nan_inf_invalid_range_filtering():
    """Synthetic LaserScan with NaNs, infs, and negative ranges filtered out correctly."""
    ranges = [float('nan'), float('inf'), -1.0, 0.02, 3.5, 4.0, float('nan')]
    angle_min = -math.pi / 2
    angle_increment = math.pi / 6

    sectors = process_laser_scan_sectors(
        ranges=ranges,
        angle_min=angle_min,
        angle_increment=angle_increment,
        range_min=0.05,
        range_max=10.0,
    )
    # Valid readings should only be 3.5 and 4.0
    assert math.isclose(sectors["closest_distance"], 3.5, abs_tol=1e-3)
    assert math.isfinite(sectors["closest_distance"])


def test_sector_distance_and_density_extraction():
    """Verifies front, left, right sector isolation and clutter density computation."""
    # 360 rays from -pi to pi (resolution: 1 deg = 2*pi/360)
    ranges = [5.0] * 360

    # Put an obstacle at left (75 deg: index ~255)
    ranges[255] = 0.80

    # Put obstacles in front sector (0 deg: index 180) within density radius
    ranges[175] = 0.90
    ranges[180] = 0.95
    ranges[185] = 0.92

    sectors = process_laser_scan_sectors(
        ranges=ranges,
        angle_min=-math.pi,
        angle_increment=(2 * math.pi) / 360,
        front_angle_deg=60.0,
        density_radius=1.30,
    )

    assert math.isclose(sectors["front_distance"], 0.90, abs_tol=1e-2)
    assert sectors["density_rho"] > 0.0
    assert sectors["left_distance"] <= 1.0


def test_geometry_cross_track_and_heading():
    """Tests geometry utility math."""
    # Cross track error
    cte = cross_track_error(px=2.0, py=0.35, line_y=0.0)
    assert math.isclose(cte, 0.35, abs_tol=1e-4)

    # Angle normalization
    assert math.isclose(normalize_angle(3 * math.pi), math.pi, abs_tol=1e-4) or math.isclose(normalize_angle(3 * math.pi), -math.pi, abs_tol=1e-4)
    assert math.isclose(normalize_angle(0.0), 0.0, abs_tol=1e-4)

    # Distance
    dist = euclidean_distance(0.0, 0.0, 3.0, 4.0)
    assert math.isclose(dist, 5.0, abs_tol=1e-4)
