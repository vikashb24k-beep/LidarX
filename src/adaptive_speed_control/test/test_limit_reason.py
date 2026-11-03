import pytest
from adaptive_speed_control.utils.speed_policy import (
    compute_distance_factor,
    compute_density_factor,
    compute_time_to_collision,
    compute_limit_reason,
)


def test_distance_factor():
    # Below threshold -> 0.0
    assert compute_distance_factor(0.30, threshold=0.45, slowdown_dist=2.0) == 0.0
    assert compute_distance_factor(0.45, threshold=0.45, slowdown_dist=2.0) == 0.0
    # Above slowdown -> 1.0
    assert compute_distance_factor(2.00, threshold=0.45, slowdown_dist=2.0) == 1.0
    assert compute_distance_factor(3.50, threshold=0.45, slowdown_dist=2.0) == 1.0
    # Intermediate linear scaling
    mid = compute_distance_factor(1.225, threshold=0.45, slowdown_dist=2.0)
    assert 0.49 < mid < 0.51


def test_density_factor():
    # Empty environment rho=0 -> 1.0
    assert compute_density_factor(0.0, density_influence=0.60, c_min=0.25) == 1.0
    # Heavy clutter rho=1.0 -> 1.0 - 0.6 = 0.40
    assert abs(compute_density_factor(1.0, density_influence=0.60, c_min=0.25) - 0.40) < 1e-4
    # Max clutter clamping to c_min
    assert compute_density_factor(1.0, density_influence=0.90, c_min=0.25) == 0.25


def test_time_to_collision():
    # Stopped robot -> -1.0 (not closing)
    assert compute_time_to_collision(1.0, 0.0) == -1.0
    # Closing at 0.2 m/s towards obstacle 1.0m away -> 5.0s
    assert abs(compute_time_to_collision(1.0, 0.20) - 5.0) < 1e-3
    # Negative / infinite distance -> -1.0
    assert compute_time_to_collision(float('inf'), 0.20) == -1.0


def test_limit_reason_clear():
    reason = compute_limit_reason(
        v_req=0.30,
        v_cmd_target=0.30,
        v_cmd_actual=0.30,
        front_dist=3.0,
        density=0.0,
        threshold=0.45,
        slowdown_dist=2.0,
        max_lin_speed=0.40,
        is_sensor_timeout=False,
        adaptive_enabled=True,
    )
    assert reason == "CLEAR"


def test_limit_reason_sensor_timeout():
    reason = compute_limit_reason(
        v_req=0.30,
        v_cmd_target=0.0,
        v_cmd_actual=0.0,
        front_dist=3.0,
        density=0.0,
        threshold=0.45,
        slowdown_dist=2.0,
        is_sensor_timeout=True,
    )
    assert reason == "SENSOR_TIMEOUT"


def test_limit_reason_emergency_stop():
    reason = compute_limit_reason(
        v_req=0.40,
        v_cmd_target=0.0,
        v_cmd_actual=0.0,
        front_dist=0.41,
        density=0.10,
        threshold=0.45,
        slowdown_dist=2.0,
    )
    assert "EMERGENCY_STOP" in reason
    assert "0.41 m" in reason


def test_limit_reason_obstacle_ahead():
    reason = compute_limit_reason(
        v_req=0.40,
        v_cmd_target=0.20,
        v_cmd_actual=0.20,
        front_dist=0.85,
        density=0.0,
        threshold=0.45,
        slowdown_dist=2.0,
    )
    assert "OBSTACLE_AHEAD" in reason
    assert "0.85 m" in reason
    assert "CROWDED" not in reason


def test_limit_reason_crowded():
    reason = compute_limit_reason(
        v_req=0.40,
        v_cmd_target=0.25,
        v_cmd_actual=0.25,
        front_dist=2.50,
        density=0.55,
        threshold=0.45,
        slowdown_dist=2.0,
    )
    assert "CROWDED" in reason
    assert "0.55" in reason
    assert "OBSTACLE_AHEAD" not in reason


def test_limit_reason_obstacle_and_crowded():
    reason = compute_limit_reason(
        v_req=0.40,
        v_cmd_target=0.12,
        v_cmd_actual=0.12,
        front_dist=0.62,
        density=0.48,
        threshold=0.45,
        slowdown_dist=2.0,
    )
    assert "OBSTACLE_AHEAD" in reason
    assert "0.62 m" in reason
    assert "CROWDED" in reason
    assert "0.48" in reason


def test_limit_reason_speed_cap():
    reason = compute_limit_reason(
        v_req=0.70,
        v_cmd_target=0.40,
        v_cmd_actual=0.40,
        front_dist=4.0,
        density=0.0,
        threshold=0.45,
        slowdown_dist=2.0,
        max_lin_speed=0.40,
    )
    assert reason == "SPEED_CAP (user)"


def test_limit_reason_accel_limit():
    reason = compute_limit_reason(
        v_req=0.40,
        v_cmd_target=0.40,
        v_cmd_actual=0.10,  # Still ramping up
        front_dist=4.0,
        density=0.0,
        threshold=0.45,
        slowdown_dist=2.0,
        max_lin_speed=0.40,
    )
    assert reason == "ACCEL_LIMIT (ramping)"


def test_safety_invariant_emergency_stop_when_adaptive_disabled():
    # Critical invariant: emergency stop must NEVER be bypassed, even with adaptive_enabled=False
    reason = compute_limit_reason(
        v_req=0.40,
        v_cmd_target=0.0,
        v_cmd_actual=0.0,
        front_dist=0.35,
        density=0.50,
        threshold=0.45,
        adaptive_enabled=False,
    )
    assert "EMERGENCY_STOP" in reason
