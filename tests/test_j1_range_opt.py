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
    v, interp = o.hazard(4.0, True)
    assert interp and 44.0 < v < 601.0                               # +4° は Design が未計算 = 補間


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


def test_knee_is_plus_3_and_moves_only_if_v5_is_much_smaller(o) -> None:
    assert o.knee("hi", True)["knee"] == 3.0
    s = o.sensitivity_knee()
    assert s[1.0]["knee"] == 3.0 and s[4.0]["knee"] == 5.0


def test_plus_3_is_feasible_for_the_whole_prior_but_plus_2_is_not(o) -> None:
    fe = o.feasibility_mc(n=4000)
    assert fe["hi"][3.0]["p_feasible"] == pytest.approx(1.0) and fe["hi"][2.0]["p_feasible"] < 0.95
    assert fe["lo"][-4.0]["p_feasible"] == pytest.approx(1.0) and fe["lo"][-2.0]["p_feasible"] == 0.0


def test_recommendation_is_minus_4_to_plus_3_and_a_steel_pin_is_needed(o) -> None:
    rec = o.recommend(o.main())
    assert (rec["cover"]["lo"], rec["cover"]["hi"]) == (-4.0, 3.0)
    steel = o.stopper_mc(n=3000, omega=120.0, tpu=True, material="steel", a_decel=1000.0, e_mpa=3.0, t_mm=3.0)
    pla = o.stopper_mc(n=3000, omega=120.0, tpu=True, material="PLA", a_decel=1000.0, e_mpa=3.0, t_mm=3.0)
    assert steel["sf_p05"] > 1.5 and pla["sf_p05"] < 1.0
    thin = o.stopper_mc(n=3000, omega=120.0, tpu=True, material="steel", e_mpa=3.0, t_mm=1.0)
    thick = o.stopper_mc(n=3000, omega=120.0, tpu=True, material="steel", e_mpa=3.0, t_mm=6.0)
    assert thick["impact_p95"] < 0.6 * thin["impact_p95"]            # 薄い（1 mm）TPU パッドは硬く、厚さ 6 mm で衝撃が減る
