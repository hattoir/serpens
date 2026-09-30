"""脇の V の許容トルクの算術（`simulation/hardware_gaps/HG-S3_torque_limiter/flank_v.py`）。

**PROVISIONAL / SAFETY_UNVERIFIED。** 5.7 N は暫定値で、ここで確かめるのは算術と、暫定値が黙って変わっていないことだけ。安全の確定ではない。
"""
from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location("flank_v", ROOT / "simulation" / "hardware_gaps" / "HG-S3_torque_limiter" / "flank_v.py")
flank_v = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(flank_v)


def test_provisional_thresholds_are_unchanged() -> None:
    """暫定しきい値は変えていない（緩める変更は User の承認が要る）。"""
    th = flank_v.thresholds()
    assert th["hand_finger"] == 5.7 and th["neck"] == 5.0 and th["chest"] == 5.7 and th["face"] == 2.7


def test_design_arithmetic_0_26_is_the_r_46_mm_case() -> None:
    assert flank_v.allowed_torque_nm(5.7, 46.0) == pytest.approx(0.2622, abs=1e-4)
    assert flank_v.allowed_torque_nm(5.0, 46.0) == pytest.approx(0.23, abs=1e-6)


def test_allowed_torque_falls_with_a_smaller_contact_radius() -> None:
    """User の指摘: r が 46 mm より小さければ、許容トルクはさらに下がる。F = τ / r と τ = F × r は逆算で一致する。"""
    a = [flank_v.allowed_torque_nm(5.7, r) for r in (10.0, 20.0, 30.0, 46.0)]
    assert a == sorted(a) and a[0] < a[-1]
    assert flank_v.force_n(a[1], 20.0) == pytest.approx(5.7)


def test_limiter_window_exceeds_the_hand_threshold_at_every_radius() -> None:
    """リミッターの窓 0.7〜1.0 N·m と、ソフトの上限 0.45 N·m は、r = 46 mm でも手・指の 5.7 N を超える（OPEN の食い違い）。"""
    for tau in (0.45, 0.7, 1.0):
        assert flank_v.force_n(tau, 46.0) > 5.7
        assert flank_v.force_n(tau, 20.0) > flank_v.force_n(tau, 46.0)


def test_wedge_geometry_follows_designs_formula() -> None:
    """Design の式: 幅 = 4 mm + s · tan φ。5〜12.5 mm を通る s の範囲と、動径 r(s) ≥ 46 mm。"""
    for phi in (10.0, 25.0, 43.0):
        s0, s1 = flank_v.wedge_s_range_mm(phi)
        t = math.tan(math.radians(phi))
        assert 4.0 + s0 * t == pytest.approx(5.0) and 4.0 + s1 * t == pytest.approx(12.5)
        assert flank_v.radial_mm(s0) >= 46.0
    assert flank_v.wedge_s_range_mm(25.0)[1] - flank_v.wedge_s_range_mm(25.0)[0] == pytest.approx(7.5 / math.tan(math.radians(25.0)))
