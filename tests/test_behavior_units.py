"""STEP 6: 内部状態・効用・状態機械・移動制御・表現（単体）のテスト。"""
from __future__ import annotations

import math
import random

import numpy as np
import pytest

from serpens.behavior.controller import Controller
from serpens.behavior.expression import Expression
from serpens.behavior.fsm import StateMachine
from serpens.behavior.internal_state import InternalState, Stimuli, energy_from_temperature
from serpens.behavior.utility import STATES, Context, UtilityModel, thought_line
from serpens.config import load_config
from serpens.hw.mock_bus import MockServoBus
from serpens.motion.animator import Animator
from serpens.motion.poses import Poses
from serpens.perception.snake_pose import SnakePose
from tests.helpers import FakeClock


@pytest.fixture()
def cfg() -> dict:
    return load_config()


def pose(x: float, y: float, th_deg: float) -> SnakePose:
    th = math.radians(th_deg)
    return SnakePose(x, y, th, th, th, 0.0, True, True)


# ---- 内部状態 ---------------------------------------------------------------------
def test_first_order_decay_and_gain(cfg: dict) -> None:
    st = InternalState(cfg)
    c = cfg["behavior"]["internal"]["stress"]
    st.values["stress"] = 1.0
    st.update(c["tau_s"], Stimuli(), None)             # 1ステップで τ 進めると (x−x0) は 0 に（オイラー）
    assert st.stress == pytest.approx(c["x0"], abs=1e-9)
    st2 = InternalState(cfg)
    for _ in range(100):
        st2.update(0.05, Stimuli(approach=1.0, presence=1.0), None)
    assert st2.stress > 0.3 and st2.stress <= 1.0


def test_stress_does_not_saturate_when_a_visitor_walks_up(cfg: dict) -> None:
    """Bug-1 の回帰: 人が全速で近づき続けても Stress は飽和しない（飽和すると RETREAT だけが残る）。"""
    st = InternalState(cfg)
    for _ in range(int(10.0 / 0.02)):
        st.update(0.02, Stimuli(approach=1.0, presence=1.0), None)
    assert st.stress < 0.95
    c = cfg["behavior"]["internal"]["stress"]
    eq = c["x0"] + c["gains"]["approach"] * c["tau_s"]          # approach = 1 の平衡値
    assert 0.6 <= eq <= 0.9, eq
    # 歩いて 1.2m 近づいて立ち止まった人（approach 0.7 が 3.5 秒）とは、かかわれる
    walked = InternalState(cfg)
    for _ in range(int(3.5 / 0.02)):
        walked.update(0.02, Stimuli(approach=0.7, presence=1.0), None)
    walked.values.update(affection=0.6, attention=0.8)
    ev = UtilityModel(cfg, random.Random(0)).evaluate(walked, Context(True, 300.0, 0.0, 0.0), 0.0)
    assert ev.raw["ENGAGE"] > ev.raw["RETREAT"], ev.raw


def test_energy_from_servo_temperature(cfg: dict) -> None:
    e = cfg["behavior"]["energy"]
    assert energy_from_temperature(e["temp_fresh_c"] - 5, e["temp_fresh_c"], e["temp_tired_c"]) == 1.0
    assert energy_from_temperature(e["temp_tired_c"] + 5, e["temp_fresh_c"], e["temp_tired_c"]) == 0.0
    mid = (e["temp_fresh_c"] + e["temp_tired_c"]) / 2
    assert energy_from_temperature(mid, e["temp_fresh_c"], e["temp_tired_c"]) == pytest.approx(0.5)
    st = InternalState(cfg)
    for _ in range(200):
        st.update(0.1, Stimuli(), e["temp_tired_c"])    # 熱い = 疲れている
    assert st.energy < 0.01 and st.heat_c == e["temp_tired_c"]


# ---- 効用 ------------------------------------------------------------------------
def test_utility_noise_is_bounded_and_resampled(cfg: dict) -> None:
    u = UtilityModel(cfg, random.Random(0))
    st = InternalState(cfg)
    ctx = Context(True, 800.0, 0.0, 0.3)
    noise = cfg["behavior"]["utility"]["noise_ratio"]
    period = cfg["behavior"]["utility"]["noise_period_s"]
    ev0 = u.evaluate(st, ctx, 0.0)
    assert all(abs(ev0.noisy[k] - ev0.raw[k]) <= noise * ev0.raw[k] + 1e-12 for k in STATES)
    assert u.evaluate(st, ctx, period * 0.5).noisy == ev0.noisy          # 周期内は同じ乱数
    assert u.evaluate(st, ctx, period * 1.01).noisy != ev0.noisy


def test_utility_picks_sensible_states(cfg: dict) -> None:
    u = UtilityModel(cfg, random.Random(0))
    st = InternalState(cfg)
    best = lambda ctx: max((ev := u.evaluate(st, ctx, 0.0)).noisy, key=lambda k: ev.noisy[k])  # noqa: E731
    assert best(Context(True, 800.0, 1.0, 0.0)) == "PETTED"
    st.values["curiosity"], st.energy = 0.9, 1.0
    assert best(Context(False, None, 0.0, 0.0)) == "PATROL"
    assert best(Context(True, 900.0, 0.0, 0.0)) == "APPROACH"
    assert best(Context(True, 420.0, 0.0, 0.0)) == "ENGAGE"
    st.energy = 0.0
    assert best(Context(False, None, 0.0, 0.0)) == "COIL_REST_MOOD"
    ev = u.evaluate(st, Context(False, None, 0.0, 0.0), 0.0)
    assert "→ とぐろで休む" in thought_line(ev, "COIL_REST_MOOD")


# ---- 状態機械 --------------------------------------------------------------------
def test_fsm_hysteresis_and_min_dwell(cfg: dict) -> None:
    h = cfg["behavior"]["utility"]["hysteresis"]
    dwell = cfg["behavior"]["utility"]["min_dwell_s"]
    fsm = StateMachine(cfg, "PATROL", 0.0)
    u = {k: 0.0 for k in STATES}
    u["PATROL"], u["OBSERVE"] = 0.5, 0.5 * h * 1.5
    assert fsm.step(dwell - 0.1, u) is None                     # 最小継続時間の前
    u["OBSERVE"] = 0.5 * (h - 0.01)
    assert fsm.step(dwell + 0.1, u) is None                     # 1.15 倍に届かない
    u["OBSERVE"] = 0.5 * (h + 0.01)
    assert fsm.step(dwell + 0.2, u).dst == "OBSERVE"
    assert fsm.time_to_next(dwell + 0.2) == pytest.approx(dwell)


def test_fsm_petted_interrupts_immediately(cfg: dict) -> None:
    fsm = StateMachine(cfg, "APPROACH", 0.0)
    u = {k: 0.1 for k in STATES}
    u["PETTED"] = 3.0
    assert fsm.step(0.5, u).dst == "PETTED"
    assert fsm.dwell_of("PETTED") == cfg["behavior"]["utility"]["min_dwell_overrides"]["PETTED"]


# ---- 移動制御 --------------------------------------------------------------------
def test_controller_speed_limit_stop_and_no_faster_period(cfg: dict) -> None:
    c = cfg["behavior"]["controller"]
    ctrl = Controller(cfg)
    base_f = cfg["gait"]["presets"][c["gait"]]["temporal_freq_hz"]
    p = pose(600.0, 600.0, 0.0)
    far = ctrl.drive_to(0.0, p, np.array([1100.0, 600.0]), 10_000.0)
    assert far.params.temporal_freq_hz == pytest.approx(base_f)           # 周期は短くしない
    near_person = np.array([600.0 + c["near_person_mm"] - 50, 600.0])
    cmd = ctrl.drive_to(0.0, p, near_person, c["speed_retreat_mm_s"], near_person)
    assert cmd.speed_mm_s == c["near_speed_limit_mm_s"]                  # 1m 以内は 8cm/s
    approach = ctrl.drive_to(0.0, p, np.array([900.0, 900.0]), c["speed_approach_mm_s"])
    retreat = ctrl.drive_to(0.0, p, np.array([900.0, 900.0]), c["speed_retreat_mm_s"])
    assert approach.params.temporal_freq_hz < retreat.params.temporal_freq_hz   # h.
    person = np.array([600.0 + c["head_reach_mm"] + c["stop_distance_mm"] - 1, 600.0])
    stop = ctrl.drive_to(0.0, p, person, c["speed_approach_mm_s"], person, stop_at_person=True)
    assert not stop.moving and stop.blocked


def test_controller_edge_recovery_and_edge_goal(cfg: dict) -> None:
    """マット端では後退しながら中央へ向き直る。人へ近づく場合は端に着いたら止まる。"""
    c = cfg["behavior"]["controller"]
    ctrl = Controller(cfg)
    # 頭先端が端から mat_margin 以内で、外を向いている姿勢
    x = cfg["mat"]["width_mm"] - c["head_reach_mm"] - c["mat_margin_mm"] + 20.0
    p = pose(x, 600.0, 0.0)
    rec = ctrl.drive_to(c["forward_min_s"] + 0.1, p, np.array([3000.0, 600.0]), 100.0)
    assert rec.moving and rec.params.temporal_freq_hz < 0                  # 後退しながら向き直る
    assert ctrl.phase == "back"
    ctrl2 = Controller(cfg)
    goal = ctrl2.drive_to(0.0, p, np.array([3000.0, 600.0]), 100.0, edge_is_goal=True)
    assert not goal.moving and goal.blocked                                # 人の方を向いて端に着いた


# ---- 表現 ------------------------------------------------------------------------
def rig(cfg: dict) -> tuple[Expression, Animator, MockServoBus]:
    clock = FakeClock()
    bus = MockServoBus(cfg, clock=clock)
    bus.connect()
    anim = Animator(cfg)
    return Expression(cfg, anim, Poses(cfg), random.Random(1), bus), anim, bus


def test_look_at_overshoot_and_hold(cfg: dict) -> None:
    x = cfg["behavior"]["expression"]
    ex, anim, _ = rig(cfg)
    assert ex.look_at(0.0, 40.0)
    peak, t = 0.0, 0.0
    while t < 3.0:
        peak = max(peak, anim.update(t)["J8"])
        t += 0.02
    lo, hi = x["look_overshoot_deg"]
    breath = cfg["breath"]["amplitude_deg"]
    assert 40.0 + lo - breath - 0.3 <= peak <= 40.0 + hi + breath + 0.3
    hold_lo, hold_hi = x["look_hold_s"]
    assert not ex.look_at(1.0, 45.0)                                       # ホールド中の小さな変化は無視
    assert ex.hold_until - 0.0 >= hold_lo


def test_tilt_surprise_distraction_and_petted(cfg: dict) -> None:
    x = cfg["behavior"]["expression"]
    ex, anim, bus = rig(cfg)
    ex.tilt(0.0)
    roll = max(abs(anim.update(t)["J9"]) for t in np.arange(0, 1.5, 0.02))
    assert x["tilt_deg"][0] - 3 <= roll <= x["tilt_deg"][1] + 3
    assert ex.pending("untilt")
    ex.surprise(2.0)
    assert anim.frozen
    ex.update(2.0 + x["surprise_freeze_s"] + 1e-6)
    assert not anim.frozen
    lo, hi = x["distraction_interval_s"]
    assert lo <= ex.next_distract_t <= hi
    end = ex.petted(5.0)
    assert end == pytest.approx(5.0 + x["petted_s"])
    assert bus._axes[1].torque_ratio == pytest.approx(
        cfg["poses"]["relax"]["torque_ratio"] * cfg["safety_limits"]["torque"]["software_torque_limit_ratio"])
    ex.update(end + 0.01)
    assert bus._axes[1].torque_ratio == pytest.approx(
        cfg["safety_limits"]["torque"]["software_torque_limit_ratio"])   # 演出が終わっても安全上限まで
