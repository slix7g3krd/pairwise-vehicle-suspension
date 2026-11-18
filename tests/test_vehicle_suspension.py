import pytest
from physics import vehicle_suspension as mod

CASES = [
    ("mask_both", "m.collision_mask(True, True) == (True, True)"),
    ("mask_static", "m.collision_mask(True, False) == (True, False)"),
    ("integrate_three", "m.integrate_steps(0.033, 0.016) == 3"),
    ("integrate_exact", "m.integrate_steps(0.016, 0.016) == 1"),
    ("anti_roll_zero", "m.anti_roll(0.5, 0.5) == approx(0.0)"),
    ("anti_roll_positive", "m.anti_roll(0.2, 0.8) > 0.0"),
    ("friction_limit", "sum(v * v for v in m.friction_circle(10.0, 20.0, 15.0)) <= 15.5 ** 2"),
    ("friction_inside", "m.friction_circle(3.0, 4.0, 15.0) == (3.0, 4.0)"),
    ("diff_lock_true", "m.diff_lock(2.0, 1.0) is True"),
    ("diff_lock_false", "m.diff_lock(0.5, 1.0) is False"),
    ("gravity_less", "m.gravity_slope(10.0, (0.0, 1.0)) <= 10.0"),
    ("gravity_flat", "m.gravity_slope(10.0, (0.0, 1.0)) == approx(0.0)"),
    ("ray_hit", "m.continuous_ray((0.0, 0.0), (0.0, 2.0), 10.0) is True"),
    ("ray_miss", "m.continuous_ray((0.0, 0.0), (0.0, 2.0), 10.0) is False"),
    ("resample_len", "len(m.resample_input([0, 1, 2], 0.5)) > 0")
]

@pytest.mark.parametrize("name,expr", CASES, ids=[c[0] for c in CASES])
def test_contract(name, expr):
    assert eval(expr, {"m": mod, "approx": pytest.approx})
