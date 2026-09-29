"""HG-S1 接触安全と HG-C1 ケーブル経路の解析（式の正しさと、既存の記録との一致）。"""
from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from serpens.config import load_config

ROOT = Path(__file__).resolve().parent.parent


def _load(path: str, name: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def s1():
    return _load("simulation/hardware_gaps/HG-S1_contact_safety/run.py", "hg_s1_run")


@pytest.fixture(scope="module")
def c1():
    return _load("simulation/hardware_gaps/HG-C1_cable_routing/run.py", "hg_c1_run")


def test_hole_matches_the_documented_bend_radius(s1) -> None:
    """docs/safety_limits.md: リンク 95mm・±50° で中心線の曲げ半径 101.9mm。幅 0 なら内径はその 2 倍。"""
    assert s1.hole_diameter_mm(50.0, 95.0, 0.0) == pytest.approx(2 * 101.9, abs=0.2)
    assert s1.hole_diameter_mm(55.0, 95.0, 92.0) < s1.hole_diameter_mm(50.0, 95.0, 92.0)


def test_max_contiguous_sum(s1) -> None:
    assert s1.max_contiguous_sum(np.array([10.0, 20.0, -5.0, 30.0])) == pytest.approx(55.0)
    assert s1.max_contiguous_sum(np.array([-40.0, -40.0, 30.0])) == pytest.approx(80.0)


def test_gaits_need_less_than_a_half_turn(s1) -> None:
    """歩容（旋回を含む）に要る連続した角度の和は 180° 未満 → 角度合計の上限で囲い込みを防げる。"""
    for name in ("FW5", "FW6_YAW5", "FW6_HEADYAW"):
        cfg = load_config(overlay=ROOT / s1.A["configs"][name]["overlay"])
        for amp in (20.0, 30.0, 40.0):
            need = s1.gait_need(cfg, amp, 0.75, 50.0 - amp, extra_head_deg=25.0)
            assert need < 180.0, f"{name} A{amp}: {need:.0f}°"


def test_cable_chord_reproduces_the_fw04_memo(c1) -> None:
    """Serpens_配線設計メモ_20260927.md の仮アンカーの直線距離（±55°）。"""
    want = {-55: 23.509, -50: 26.526, 0: 55.000, 50: 74.207, 55: 75.424}
    for th, d in want.items():
        assert c1.chord_mm(20.0, 35.0, -28.5, -28.5, th) == pytest.approx(d, abs=0.002)


def test_on_axis_routing_changes_length_far_less(c1) -> None:
    side = c1.evaluate(25.0, 25.0, 28.5, 0.0, 2.0, 55.0)
    axis = c1.evaluate(25.0, 25.0, 0.0, 0.0, 2.0, 55.0)
    assert axis["length_change"] < 0.2 * side["length_change"]
    assert axis["chord_max"] == pytest.approx(50.0)                       # θ = 0 で 2a
    assert axis["chord_min"] == pytest.approx(50.0 * math.cos(math.radians(27.5)), abs=0.01)
