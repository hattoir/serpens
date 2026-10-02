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


def _j1_head():
    spec = importlib.util.spec_from_file_location("j1_head", ROOT / "simulation" / "hardware_gaps" / "HG-S3_torque_limiter" / "j1_head.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_j3_j5_allowed_torque_is_about_0_25_and_below_the_soft_cap() -> None:
    """Design の r = 44.4〜49.8 mm（J3〜J5）で、5.7 N の暫定値の許容トルクは約 0.25 N·m。ソフトの上限 0.45 N·m と窓 0.7〜1.0 N·m は上回る（OPEN の不一致）。暫定値であり安全の確定ではない。"""
    m = _j1_head()
    lo, hi = m.allowed_nm(5.7, 44.4), m.allowed_nm(5.7, 49.8)
    assert lo == pytest.approx(0.253, abs=1e-3) and hi == pytest.approx(0.284, abs=1e-3)
    assert m.SOFT_CAP_NM > hi and m.WINDOW_NM[0] > hi
    assert m.force_n(m.SOFT_CAP_NM, 49.8) > 5.7


def test_j1_lowering_side_cannot_be_protected_by_a_force_limit_alone() -> None:
    """J1 の下げる側 r = 5〜44 mm: 許容トルクは 0.029〜0.25 N·m で、ソフトの上限（0.45）を大きく下回る。"""
    m = _j1_head()
    assert m.allowed_nm(5.7, 5.0) == pytest.approx(0.0285, abs=1e-4)
    assert m.allowed_nm(5.7, 44.0) < m.SOFT_CAP_NM


def test_hollow_added_mass_keeps_the_head_com_close_to_j1() -> None:
    """中空 10 g 台なら重心と J1 の静的トルクの増え方は小さい（現行 21.9 mm / 0.020 N·m）。中実 55〜65 g は大きい。"""
    m = _j1_head()
    assert m.com_lever_mm() == pytest.approx(21.9, abs=0.1)
    lever_h, tau_h, _ = m.with_added_mass(19.0, 80.0)
    lever_s, tau_s, _ = m.with_added_mass(65.0, 80.0)
    assert lever_h < 35.0 and tau_h < 0.04 and lever_s > 40.0 and tau_s > tau_h
    assert tau_s < m.SOFT_CAP_NM


def test_j7_static_torque_uses_the_robot_yaml_lifted_mass_not_the_design_head_only_value() -> None:
    """訂正: J7 が持ち上げる質量 200 g・重心 104 mm → 静的 0.204 N·m（ソフトの上限の 45%）。Design の頭だけの 0.020 N·m ではない。
    中空 10〜19 g を足しても上限内（〜53%）、中実 55〜65 g は 60% を超える。暫定値であり安全の確定ではない。"""
    m = _j1_head()
    t0, lever = m.lifted_torque_nm()
    assert t0 == pytest.approx(0.204, abs=0.002) and lever == pytest.approx(104.0)
    assert t0 / m.SOFT_CAP_NM == pytest.approx(0.45, abs=0.01)
    t_h, _ = m.lifted_torque_nm(19.0, 180.0)
    t_s, _ = m.lifted_torque_nm(65.0, 180.0)
    assert t_h / m.SOFT_CAP_NM < 0.55 and t_s / m.SOFT_CAP_NM > 0.60 and t_s < m.SOFT_CAP_NM


def test_mass_additions_stay_within_the_total_mass_limit_and_outside_the_budget() -> None:
    """質量の追加（PROPOSED）は収支 mass_budget_g に含めない。全部足しても mass_total_g_max（1700 g）以内。"""
    import yaml
    cfg = yaml.safe_load((ROOT / "config" / "robot.yaml").read_text(encoding="utf-8"))
    b, add = cfg["mass_budget_g"], cfg["mass_additions_g"]
    assert add["status"] == "PROPOSED" and "knuckle_drum_solid_max" not in b
    base = b["servo_each"] * b["servo_count"] + b["segment_frame_each"] * b["segment_frame_count"] + b["passive_wheels_total"] + b["head_total"] + b["skin_and_wiring"]
    worst = base + add["knuckle_drum_solid_max"]["total"] + add["head_lower_rear_fill_solid"] + add["intake_head_solid"][1]
    assert add["knuckle_drum_solid_max"]["total"] == pytest.approx(61.1) and add["knuckle_drum_hollow_shell_1p2mm_max"]["total"] == pytest.approx(55.8)
    assert worst <= cfg["safety_limits"]["mass_total_g_max"] and 61.1 / cfg["safety_limits"]["mass_total_g_max"] == pytest.approx(0.036, abs=0.001)


def test_curtain_edge_pressure_limits_the_force_before_the_force_threshold_does() -> None:
    """受け身の垂れ布: 力 1 N は手・指 5.7 N の 1/5.7 だが、縁 0.4 mm では圧力が暫定の限界（8.2 N/cm²）の手前（0.98 N）。3 mN 以下なら限界の 1/40 以下。"""
    assert flank_v.pressure_limit_n_cm2() == pytest.approx(8.2)
    assert flank_v.allowed_curtain_force_n(0.4) == pytest.approx(0.984, abs=0.001) and flank_v.allowed_curtain_force_n(0.05) == pytest.approx(0.123, abs=0.001)
    assert flank_v.edge_pressure_n_cm2(1.0, 0.4) > flank_v.pressure_limit_n_cm2()
    assert flank_v.edge_pressure_n_cm2(0.003, 0.05) < flank_v.pressure_limit_n_cm2() / 40


def test_s_sweep_boundary_torque_limit_cannot_protect_the_wedge_for_any_s_in_the_design_band() -> None:
    """LB-E-011 / B2: s（法線力の腕）が来るまでの暫定の範囲計算。0.45 N·m を許容するのに要る腕は約 79 mm で、
    幅 5〜12.5 mm を通る s の最大（φ = 9° で約 54 mm）を超える = s がどの値でもトルクの制限だけでは 5.7 N を守れない。"""
    th = flank_v.thresholds()["hand_finger"]
    assert flank_v.s_needed_mm(0.45, th) == pytest.approx(78.95, abs=0.01)
    s_band_max = max(flank_v.wedge_s_range_mm(phi)[1] for phi in flank_v.PHIS_DEG)
    assert s_band_max == pytest.approx(53.7, abs=0.1)
    for tau in (0.45, 0.70, 1.00):
        assert flank_v.s_needed_mm(tau, th) > s_band_max
    assert flank_v.s_needed_mm(0.26, th) < s_band_max                  # Design の 0.26 N·m は、s が十分大きい位置でだけ許容
    assert flank_v.allowed_torque_nm(th, flank_v.s_needed_mm(0.45, th)) == pytest.approx(0.45)


def test_design_contact_s_makes_the_allowed_torque_far_below_the_gait_need_for_j2_j5_and_below_the_j7_hold_for_j1() -> None:
    """ENTRY-D-0020（Design の s。p5 を代表値に）: J2〜J5 の 5.7 N × s(p5) は 0.03 N·m 以下 = 歩容の必要（0.13）の 21% 以下。J1 は表のどの θ_E でも J7 の静的保持 0.204 N·m を下回る。"""
    th = flank_v.thresholds()["hand_finger"]
    a25 = [flank_v.allowed_torque_nm(th, r[4]) for r in flank_v.DESIGN_S_J25]
    assert len(a25) == 15 and max(a25) < 0.03 and max(a25) / flank_v.GAIT_NEED_NM[0] < 0.22
    a1 = [flank_v.allowed_torque_nm(th, r[3]) for r in flank_v.DESIGN_S_J1]
    assert len(a1) == 11 and all(x < flank_v.J7_STATIC_NM for x in a1)
    assert all(r[4] >= r[3] for r in flank_v.DESIGN_S_J25)                       # p5 は最小以上（データの整合）
