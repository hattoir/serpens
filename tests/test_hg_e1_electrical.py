"""HG-E1 電気層の判断境界（`simulation/hardware_gaps/HG-E1_electrical_gate/run.py`）。ELECTRICAL_BUDGET_MC。すべて prior。ここで確かめるのは**モデルの整合**で、電気の合否ではない。"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope="module")
def e1():
    spec = importlib.util.spec_from_file_location("hg_e1_run", ROOT / "simulation/hardware_gaps/HG-E1_electrical_gate/run.py")
    m = importlib.util.module_from_spec(spec)
    sys.modules["hg_e1_run"] = m
    spec.loader.exec_module(m)
    return m


def test_per_servo_currents_are_coherent(e1) -> None:
    """1 軸が自分のストール電流を超える組み合わせは作らない: 保持 ≤ 0.6 × ストール、歩容の実効 ≤ 0.8 × ストール。"""
    c = e1.currents(np.random.default_rng(1), 5)
    assert (c["hold"] <= 0.6 * c["all_stall"] + 1e-9).all() and (c["gait_rms"] <= 0.8 * c["all_stall"] + 1e-9).all()
    assert (c["gait_peak"] <= c["all_stall"] + 1e-9).all() and (c["peak_start"] <= c["all_stall"] + 1e-9).all()


def test_sag_boundary_resistance_is_the_6v_point_at_p95(e1) -> None:
    rng = np.random.default_rng(2)
    c = e1.currents(rng, 5)
    s = e1.sag(rng, c["peak_start"], 7.4)
    assert s["r_total_max_for_p95_ohm"] == pytest.approx((7.4 - 6.0) / np.percentile(c["peak_start"], 95))
    assert 0.0 <= s["p_below_6v"] <= 1.0 and s["v_min"]["p5"] >= 0.0                     # 0 V が下限


def test_full_charge_gives_more_margin_than_nominal(e1) -> None:
    c = e1.currents(np.random.default_rng(3), 9)
    a, b = e1.sag(np.random.default_rng(4), c["peak_start"], 7.4), e1.sag(np.random.default_rng(4), c["peak_start"], 8.4)
    assert b["r_total_max_for_p95_ohm"] > a["r_total_max_for_p95_ohm"] and b["p_below_6v"] < a["p_below_6v"]


def test_protection_trade_off_is_monotone_and_balanced_point_lies_between(e1) -> None:
    c = e1.currents(np.random.default_rng(5), 5)
    p = e1.protection(np.random.default_rng(6), c)
    rows = p["table"]
    assert all(r2["false_trip"] <= r1["false_trip"] + 1e-12 and r2["no_trip_at_all_stall"] >= r1["no_trip_at_all_stall"] - 1e-12 for r1, r2 in zip(rows, rows[1:]))
    assert 0.0 < p["p_window_open_per_unit"] < 1.0                                       # 個体によって窓が開いたり閉じたりする（prior の幅）
    assert np.percentile(c["gait_peak"], 5) <= p["balanced"]["trip_a"] <= np.percentile(c["all_stall"], 95)


def test_wiring_boundary_current_follows_the_gauge(e1) -> None:
    w = e1.wiring(np.random.default_rng(7), e1.currents(np.random.default_rng(8), 5))
    i = {g: v["i_rms_for_20k_a"] for g, v in w["wire"].items()}
    assert i["AWG24"] < i["AWG22"] < i["AWG20"]


def test_servo_temperature_order_of_limits_and_load_dependence(e1) -> None:
    A = e1.A["thermal"]["limits_c"]
    assert A["link_fault"] < A["servo_register"] < A["safety_stop"]                       # サーボが自分で止まる前に機体が止める
    t = e1.servo_temp(np.random.default_rng(9))
    p = [t["by_load"][k]["p_reaches_60c"] for k in ("保持 0.3 A", "保持 0.6 A", "保持 1.0 A", "歩容 1.5 A", "ストール 2.0 A")]
    assert p == sorted(p) and p[0] < 0.05 and p[-1] > 0.95


def test_comm_loss_cannot_be_faster_than_the_configured_heartbeat_timeout(e1) -> None:
    from serpens.config import load_config
    cfg = load_config()
    s = e1.stop_times(np.random.default_rng(10), cfg)
    assert s["comm_loss_to_hold_ms"]["p5"] >= cfg["link"]["heartbeat_timeout_ms"]
    assert s["supply_cut_ms"]["p50"] < s["estop_to_motion_stop_ms"]["p50"] < s["comm_loss_to_hold_ms"]["p50"]


def test_run_is_reproducible(e1) -> None:
    a = e1.currents(np.random.default_rng(e1.SEED), 5)["peak_start"]
    b = e1.currents(np.random.default_rng(e1.SEED), 5)["peak_start"]
    assert (a == b).all()
