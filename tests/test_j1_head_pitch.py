"""J1（頭ピッチ）の床接触の較正（手順の参照実装）・prior での確認・ToF 崖の時間予算。**ACTUATOR_MODEL_SIM。実機の値ではない**（不感帯・バックラッシは UNKNOWN の仮定）。"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent


def _load(name: str, rel: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / rel)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def fc():
    return _load("j1_floor_contact", "simulation/hardware_gaps/HG-H1_actuator/j1_floor_contact.py")


@pytest.fixture(scope="module")
def chk():
    return _load("j1_head_pitch_check", "simulation/hardware_gaps/HG-H1_actuator/j1_head_pitch_check.py")


@pytest.fixture(scope="module")
def budget():
    return _load("tof_cliff_budget", "simulation/hardware_gaps/HG-H2_sensor_head/tof_cliff_budget.py")


def test_one_step_is_0p083_mm_at_the_mouth_front_edge(fc) -> None:
    assert fc.STEP_DEG == pytest.approx(0.0879, abs=1e-4)
    assert fc.MM_PER_STEP == pytest.approx(0.0832, abs=2e-4)


def test_calibration_of_an_ideal_servo_lands_within_one_step_of_the_target(fc) -> None:
    for c_mm, steps in ((0.1, 1), (0.3, 4), (1.0, 12)):
        s = fc.ServoModel(q_floor=-7.0)
        r = fc.calibrate(s, c_mm=c_mm)
        assert r["ok"] and r["n_c"] == steps
        assert abs(r["err_mm"]) <= fc.MM_PER_STEP


def test_a_contact_threshold_below_the_deadband_gives_a_false_contact(fc) -> None:
    """不感帯 D ≥ しきい値だと、床に触れていなくても「目標に追従しない」= 接触と誤判定する（しきい値は D + 2 以上）。"""
    s = fc.ServoModel(deadband_steps=4.0, q_floor=-30.0)
    bad = fc.calibrate(s, lag_thresh=3)
    assert bad["ok"] and bad["q_contact_err_steps"] > 10
    s = fc.ServoModel(deadband_steps=4.0, q_floor=-30.0)
    good = fc.calibrate(s)                       # 既定 = D + 2
    assert good["ok"] and abs(good["q_contact_err_steps"]) <= 1


def test_a_motor_side_encoder_needs_the_load_signal_not_position_lag(fc) -> None:
    s = fc.ServoModel(encoder="ENC_MOTOR", q_floor=-9.0)
    assert fc.calibrate(s, signal="position_lag")["ok"] is False     # モーター側の読み値は床で止まっても目標に追従する → 接触が見えない
    s = fc.ServoModel(encoder="ENC_MOTOR", q_floor=-9.0)
    assert fc.calibrate(s)["ok"] is True                              # 既定は負荷で判定


def test_design_estimates_are_confirmed_by_the_hg_h1_prior(chk) -> None:
    o = chk.main()
    assert o["p_backdrive_lt_gravity"]["頭だけ 0.035"] == 0.0                       # 頭だけなら、受動の柔らかさは成り立たない（Design と一致）
    assert o["p_backdrive_lt_gravity"]["首を上げた姿勢（中空）0.238"] > 0.3          # 首を上げた姿勢では成り立ちうる（Design は触れていない）
    assert o["cap_range_nm"][1] < 0.05                                              # 作業中の上限 8/1000 は、バックドライブの最小 0.05 より小さい
    assert o["p_cap_gt_backdrive"] == 0.0
    assert o["force_range_n"][0] == pytest.approx(0.15, abs=0.01) and o["force_range_n"][1] == pytest.approx(0.76, abs=0.01)
    assert o["edge_touch_deg_c0.1"] == pytest.approx(0.573, abs=0.001)               # 前縁が横スキッド前端の 10 mm 前


def test_tof_cliff_overhang_is_dominated_by_the_10_5_mm_lead(budget) -> None:
    assert budget.overhang_mm(0.0, 0.033, 1) == pytest.approx(10.5)
    assert budget.overhang_mm(80.0, 0.033, 1) == pytest.approx(10.5 + 80 * (0.033 + 0.005 + 0.010))
    assert budget.overhang_mm(80.0, 0.033, 3) > budget.overhang_mm(80.0, 0.033, 1) > budget.overhang_mm(30.0, 0.033, 1)
