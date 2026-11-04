import math

from adaptive_speed_control.utils.speed_policy import layered_braking_cap, policy_layered_governor


def governor(**overrides):
    values = dict(v_req=.4, d_clear=3., rho_path=0., lateral_free_width=2., curvature=0.,
                  closing_speed=.2, distance_to_goal=5., scan_latency=.05, confidence=1.)
    values.update(overrides)
    return policy_layered_governor(**values)


def test_monotonic_hard_and_soft_limits():
    assert governor(d_clear=1.)['speed'] <= governor(d_clear=2.)['speed']
    assert governor(rho_path=.8)['speed'] <= governor(rho_path=.1)['speed']
    assert governor(curvature=2.)['speed'] <= governor(curvature=.1)['speed']
    assert governor(closing_speed=2., d_clear=1.)['speed'] <= governor(closing_speed=.1, d_clear=1.)['speed']


def test_braking_cap_stops_within_clearance():
    cap = layered_braking_cap(1.0, .45, .6, .1)
    assert cap * .1 + cap * cap / (2 * .6) <= .55 + 1e-9


def test_confidence_does_not_change_hard_cap_and_nan_is_safe():
    assert governor(confidence=.1)['safety_cap'] == governor(confidence=1.)['safety_cap']
    result = governor(d_clear=float('nan'), rho_path=float('inf'))
    assert all(math.isfinite(value) for value in result.values())
    assert result['speed'] == 0.0


def test_lookahead_brakes_before_dense_segment_and_releases():
    slow = governor(profile_caps=((.05, .05),))['speed']
    clear = governor(profile_caps=((10.0, .05),))['speed']
    assert slow < clear


def test_creep_and_rotation_contract():
    result = governor(d_clear=3., rho_path=.2)
    assert result['speed'] >= .05
    assert governor(d_clear=.4)['speed'] == 0.0
