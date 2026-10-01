"""フード昇降の落とす力・速さ・縁の圧力の枠組み（`simulation/hardware_gaps/HG-S3_torque_limiter/hood_lift.py`）。**PROVISIONAL。5.7 N・8.2 N/cm² は暫定で、安全の確定ではない。**"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope="module")
def hl():
    spec = importlib.util.spec_from_file_location("hood_lift", ROOT / "simulation/hardware_gaps/HG-S3_torque_limiter/hood_lift.py")
    m = importlib.util.module_from_spec(spec)
    sys.modules["hood_lift"] = m
    spec.loader.exec_module(m)
    return m


def test_gravity_descent_stays_below_half_of_the_provisional_finger_threshold(hl) -> None:
    assert hl.weight_n() == pytest.approx(0.112, abs=0.001)
    for v in (5e-3, 10e-3, 20e-3):
        assert max(hl.impact_gravity_descent(v, k) for k in hl.K_CONTACT_N_M) < hl.F_LIMIT_N


def test_a_servo_driven_descent_cannot_be_kept_below_the_limit_by_speed_alone(hl) -> None:
    """サーボの反射慣性が指に当たる（衝撃は電流の上限で抑えられない）。prior の幅では 2.8 N を超える。"""
    assert hl.impact_driven_descent(20e-3, 5e-3, hl.J_REFLECTED[1], hl.K_CONTACT_N_M[1]) > hl.F_LIMIT_N


def test_a_thin_edge_is_limited_by_pressure_not_by_the_force_limit(hl) -> None:
    assert hl.allowed_force_n(0.4, 10.0) == pytest.approx(0.328, abs=0.01)
    assert hl.allowed_force_n(3.6, 30.0) == hl.F_LIMIT_N
    assert hl.pressure_n_cm2(0.112, 0.4, 10.0) < hl.P_LIMIT_N_CM2 < hl.pressure_n_cm2(0.53, 0.4, 10.0)


def test_up_stroke_torque_must_be_limited_far_below_an_sg90_stall(hl) -> None:
    assert hl.up_stroke_torque_limit_nm(5.0) == pytest.approx(0.014)
    assert 0.12 / 0.005 > 5 * hl.F_LIMIT_N          # Design の想定（SG90 の停動 0.12 N·m ÷ r = 24 N）は 2.8 N の 8 倍以上


def test_gap_band(hl) -> None:
    assert hl.gap_band_ok(4.0) and not hl.gap_band_ok(5.0) and not hl.gap_band_ok(8.0)


def test_l2_leaf_ear_keeps_the_up_force_below_the_limit_over_the_e_prior(hl) -> None:
    """Design の案 L2（TPU 95A の板ばね耳 12 × 1.4 × 9、4 mm）: E = 26 MPa で 1.2 N、E が 2 倍でも 2.8 N 以下。E の上限は約 62 MPa。"""
    assert hl.leaf_ear_force_n(26.0) == pytest.approx(1.17, abs=0.02)
    assert hl.leaf_ear_force_n(52.0) < hl.F_LIMIT_N
    assert hl.max_e_for_force_mpa() == pytest.approx(62.0, abs=1.0)
    assert hl.leaf_ear_stress_mpa(26.0) == pytest.approx(2.7, abs=0.1)
