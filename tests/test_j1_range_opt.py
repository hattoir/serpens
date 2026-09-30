"""J1（J7）の動作範囲・ストッパーの荷重の最適化（`simulation/hardware_gaps/HG-H1_actuator/j1_range_opt.py`）。
**ACTUATOR_MODEL_SIM + MUJOCO_SIM + CAD_CONCEPT（Design の見積もり）。実機で未確認。**"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope="module")
def o():
    spec = importlib.util.spec_from_file_location("j1_range_opt", ROOT / "simulation/hardware_gaps/HG-H1_actuator/j1_range_opt.py")
    m = importlib.util.module_from_spec(spec)
    sys.modules["j1_range_opt"] = m
    spec.loader.exec_module(m)
    return m


def test_computed_hazard_points_are_used_as_is_and_the_gaps_are_interpolated(o) -> None:
    assert o.hazard(3.0, True) == (44.0, False)
    assert o.hazard(-3.0, True) == (6.0, False)                     # 上の覆いあり（Design の STL）
    assert o.hazard(-3.0, False) == (105.0, False)
    assert o.hazard(4.0, True) == (151.0, False) and o.hazard(-4.0, False) == (319.0, False) and o.hazard(-4.0, True) == (23.0, False)   # Design が R-003 で計算
    v, interp = o.hazard(6.0, True)
    assert interp and 151.0 < v < 1562.0                             # +6° は未計算 = 補間


def test_poses_lost_by_the_range(o) -> None:
    lost3 = o.lost_poses(-4.0, 3.0)
    assert {"home", "rest_arc", "coil"} <= set(lost3) and len([x for x in lost3 if not x.startswith("breath")]) == 5
    lost8 = o.lost_poses(-4.0, 8.0)
    assert "home" not in lost8 and "look(人を見る)" in lost8 and len([x for x in lost8 if not x.startswith("breath")]) == 2


def test_gap_mapping_follows_the_design_geometry(o) -> None:
    assert o.gap_from_theta(0.0) == pytest.approx(0.1)
    assert o.gap_from_theta(1.0) == pytest.approx(1.05)
    assert o.gap_from_theta(-1.0) == pytest.approx(1.15, abs=0.02)   # Design: −1° で奥の壁の下 1.15 mm


def test_intake_band_is_about_two_degrees_wide_and_inside_the_working_window(o) -> None:
    it = o.load_intake()
    lo, hi = o.intake_band(it, 0.90)
    assert -1.3 < lo < -0.8 and 1.0 < hi < 1.4
    assert o.WINDOW[0] < lo and hi < o.WINDOW[1] + 0.5


def test_hazard_steps_beyond_plus_3_grow_3_to_4_fold_per_step(o) -> None:
    """Design が +4° を計算したので、+3 → +5 の急な段（13.7 倍）は 3.4 倍 + 4.0 倍の 2 段になった。膝は 1 点に決まらない（範囲は制約で決める）。"""
    st = {x["theta"]: x for x in o.hazard_steps("hi", True)}
    assert st[4.0]["ratio"] == pytest.approx(151 / 44) and st[5.0]["ratio"] == pytest.approx(601 / 151)
    assert o.knee("hi", True)["chord_knee"] in (3.0, 4.0)


def test_plus_3_is_feasible_for_the_whole_prior_but_plus_2_is_not(o) -> None:
    fe = o.feasibility_mc(n=4000)
    assert fe["hi"][3.0]["p_feasible"] == pytest.approx(1.0) and fe["hi"][2.0]["p_feasible"] < 0.95
    assert fe["lo"][-4.0]["p_feasible"] == pytest.approx(1.0) and fe["lo"][-2.0]["p_feasible"] == 0.0


def test_recommendation_is_minus_4_to_plus_3_and_a_steel_pin_is_needed(o) -> None:
    rec = o.recommend(o.main())
    assert (rec["cover"]["lo"], rec["cover"]["hi"]) == (-4.0, 3.0) and rec["cover"]["worst"] == 44.0
    assert rec["nocover"]["worst"] == 319.0                          # 覆いなしは −4° の 319 mm³（Design の計算）
    steel = o.stopper_mc(n=3000, omega=120.0, tpu=True, material="steel", a_decel=1000.0, e_mpa=3.0, t_mm=3.0)
    pla = o.stopper_mc(n=3000, omega=120.0, tpu=True, material="PLA", a_decel=1000.0, e_mpa=3.0, t_mm=3.0)
    assert steel["sf_p05"] > 1.5 and pla["sf_p05"] < 1.0
    thin = o.stopper_mc(n=3000, omega=120.0, tpu=True, material="steel", e_mpa=3.0, t_mm=1.0)
    thick = o.stopper_mc(n=3000, omega=120.0, tpu=True, material="steel", e_mpa=3.0, t_mm=6.0)
    assert thick["impact_p95"] < 0.6 * thin["impact_p95"]            # 薄い（1 mm）TPU パッドは硬く、厚さ 6 mm で衝撃が減る


def test_pad_bottoming_check_and_the_dead_backlash_thresholds(o) -> None:
    """Design のパッド（7.2 mm²）: E 3 MPa は底付き（ひずみ > 0.4）、E 10 MPa は 4 mm 以上で底付きしない。鋼なら底付きしても SF > 1.5。"""
    ps = o.pad_sweep(n=1500)
    row = {(x["E_mpa"], x["t_mm"]): x for x in ps["t_rows"] if x["omega_dps"] == 120.0}
    assert row[(3.0, 3.0)]["strain"] > 0.4 and row[(3.0, 8.0)]["strain"] > 0.4
    assert row[(10.0, 4.0)]["strain"] <= 0.4 and row[(10.0, 6.0)]["strain"] <= 0.4
    assert min(x["sf_p05"] for x in ps["t_rows"]) > 1.5
    assert 5.5 < o.range_narrow_threshold_steps(2.0, 1000.0) < 7.0    # +2° は D + B が約 6 ステップ以下（a = 1000）
    b = o.band_miss_threshold_steps()
    assert 10.0 < b["steps_lower"] < 13.0 and 12.0 < b["steps_upper"] < 14.0


def test_floor_watch_range_config_matches_the_optimization_and_keeps_the_existing_j7_limits(o) -> None:
    import yaml
    cfg = yaml.safe_load(open(ROOT / "config" / "robot.yaml", encoding="utf-8"))
    n = cfg["neck"]
    lo, hi = n["floor_watch_stop_deg"]
    assert (lo, hi) == (-4.0, 3.0)
    assert lo <= n["floor_watch_soft_deg"][0] < n["floor_watch_soft_deg"][1] <= hi          # 作業窓は範囲の内側
    assert n["floor_watch_near_limit_speed_dps"] <= o.speed_cap_dps(hi, "hi", o.control_margin_deg()["max"]) + 3.0
    j7 = next(j for j in cfg["joints"] if j["name"] == "J7")
    assert (j7["min_deg"], j7["max_deg"]) == (-8.0, 90.0)                                     # 既存の J7 の範囲は変えていない
