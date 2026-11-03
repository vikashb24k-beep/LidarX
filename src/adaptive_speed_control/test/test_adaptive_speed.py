import pytest
import math
from adaptive_speed_control.utils.speed_policy import calculate_adaptive_speed


def test_empty_corridor_high_speed():
    """Zone 1: Empty Corridor — High speed around 0.40 m/s."""
    v_cmd, omega_cmd, scale, state = calculate_adaptive_speed(
        v_req=0.40,
        omega_req=0.0,
        front_dist=4.5,
        density=0.0,
        threshold=0.45,
        slowdown_dist=2.00,
        density_influence=0.60,
        c_min=0.25,
        max_lin_speed=0.40,
        max_ang_speed=1.00,
        current_state="STARTING",
    )
    assert math.isclose(v_cmd, 0.40, abs_tol=1e-3)
    assert math.isclose(scale, 1.0, abs_tol=1e-3)
    assert state == "EMPTY_CORRIDOR"


def test_moderate_traffic_reduced_speed():
    """Zone 2: Moderate Traffic — Reduced speed in [0.20, 0.28] m/s."""
    v_cmd, omega_cmd, scale, state = calculate_adaptive_speed(
        v_req=0.40,
        omega_req=0.0,
        front_dist=1.8,
        density=0.20,
        threshold=0.45,
        slowdown_dist=2.00,
        density_influence=0.60,
        c_min=0.25,
        max_lin_speed=0.40,
        max_ang_speed=1.00,
        current_state="EMPTY_CORRIDOR",
    )
    assert 0.20 <= v_cmd <= 0.28
    assert state == "MODERATE_TRAFFIC"


def test_cluttered_corridor_low_speed():
    """Zone 3: Cluttered Corridor — Low speed in [0.10, 0.15] m/s."""
    v_cmd, omega_cmd, scale, state = calculate_adaptive_speed(
        v_req=0.40,
        omega_req=0.0,
        front_dist=0.95,
        density=0.55,
        threshold=0.45,
        slowdown_dist=2.00,
        density_influence=0.60,
        c_min=0.25,
        max_lin_speed=0.40,
        max_ang_speed=1.00,
        current_state="MODERATE_TRAFFIC",
    )
    assert 0.10 <= v_cmd <= 0.15
    assert state == "CLUTTERED_CORRIDOR"


def test_critical_proximity_controlled_stop():
    """Zone 4: Critical Proximity — Immediate zero speed stop."""
    v_cmd, omega_cmd, scale, state = calculate_adaptive_speed(
        v_req=0.40,
        omega_req=0.2,
        front_dist=0.42,
        density=0.75,
        threshold=0.45,
        slowdown_dist=2.00,
        density_influence=0.60,
        c_min=0.25,
        max_lin_speed=0.40,
        max_ang_speed=1.00,
        current_state="CLUTTERED_CORRIDOR",
    )
    assert v_cmd == 0.0
    assert omega_cmd == 0.0
    assert scale == 0.0
    assert state == "CRITICAL_PROXIMITY"


def test_controlled_stop_holding_within_hysteresis():
    """Remains in CONTROLLED_STOP while obstacle is still too close (<= 0.65m)."""
    v_cmd, omega_cmd, scale, state = calculate_adaptive_speed(
        v_req=0.40,
        omega_req=0.0,
        front_dist=0.55,
        density=0.60,
        threshold=0.45,
        slowdown_dist=2.00,
        density_influence=0.60,
        c_min=0.25,
        max_lin_speed=0.40,
        max_ang_speed=1.00,
        current_state="CONTROLLED_STOP",
    )
    assert v_cmd == 0.0
    assert state == "CONTROLLED_STOP"


def test_path_clear_automatic_resumption():
    """Transitions to PATH_CLEAR and automatically resumes velocity once obstacle is cleared."""
    v_cmd, omega_cmd, scale, state = calculate_adaptive_speed(
        v_req=0.40,
        omega_req=0.0,
        front_dist=3.0,
        density=0.0,
        threshold=0.45,
        slowdown_dist=2.00,
        density_influence=0.60,
        c_min=0.25,
        max_lin_speed=0.40,
        max_ang_speed=1.00,
        current_state="CONTROLLED_STOP",
    )
    assert v_cmd > 0.0
    assert state == "PATH_CLEAR"


def test_max_linear_speed_clamp():
    """User request exceeding max speed limit is clamped to max_linear_speed."""
    v_cmd, omega_cmd, scale, state = calculate_adaptive_speed(
        v_req=0.85,
        omega_req=0.0,
        front_dist=5.0,
        density=0.0,
        threshold=0.45,
        slowdown_dist=2.00,
        density_influence=0.60,
        c_min=0.25,
        max_lin_speed=0.40,
        max_ang_speed=1.00,
        current_state="EMPTY_CORRIDOR",
    )
    assert math.isclose(v_cmd, 0.40, abs_tol=1e-3)


def test_angular_speed_clamping():
    """Angular velocities exceeding max_angular_speed are clamped safely."""
    v_cmd, omega_cmd, scale, state = calculate_adaptive_speed(
        v_req=0.0,
        omega_req=2.50,
        front_dist=2.0,
        density=0.0,
        threshold=0.45,
        slowdown_dist=2.00,
        density_influence=0.60,
        c_min=0.25,
        max_lin_speed=0.40,
        max_ang_speed=1.00,
    )
    assert math.isclose(omega_cmd, 1.00, abs_tol=1e-4)


def test_reverse_motion():
    """Reverse motion preserves direction and is clamped by max limits."""
    v_cmd, omega_cmd, scale, state = calculate_adaptive_speed(
        v_req=-0.30,
        omega_req=0.0,
        front_dist=0.20,
        density=0.90,
        threshold=0.45,
        slowdown_dist=2.00,
        density_influence=0.60,
        c_min=0.25,
        max_lin_speed=0.40,
        max_ang_speed=1.00,
    )
    assert math.isclose(v_cmd, -0.30, abs_tol=1e-4)
    assert state == "REVERSE"


def test_in_place_rotation():
    """Pure angular commands result in in-place rotation with state ROTATING."""
    v_cmd, omega_cmd, scale, state = calculate_adaptive_speed(
        v_req=0.0,
        omega_req=0.50,
        front_dist=0.35,
        density=0.80,
        threshold=0.45,
        slowdown_dist=2.00,
        density_influence=0.60,
        c_min=0.25,
        max_lin_speed=0.40,
        max_ang_speed=1.00,
    )
    assert v_cmd == 0.0
    assert math.isclose(omega_cmd, 0.50, abs_tol=1e-4)
    assert state == "ROTATING"
