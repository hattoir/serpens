"""STEP 6: シミュレータ上で、人の座標を与えると一連の振る舞いが起きることを確かめる（統合）。"""
from __future__ import annotations

import math

import numpy as np
import pytest

from serpens.config import load_config
from serpens.sim.session import SimSession
from serpens.sim.virtual_camera import SimPerson
from serpens.sim.world import BodyPose


@pytest.fixture()
def cfg() -> dict:
    return load_config()


def run_until(s: SimSession, t_end: float, events: list | None = None, hook=None) -> None:  # type: ignore[no-untyped-def]
    while s.t < t_end:
        if hook:
            hook(s)
        st = s.step()
        if events is not None:
            events.extend((st.t, e) for e in st.events)


def test_reaction_delay_surprise_and_alert(cfg: dict) -> None:
    """b. 0.4〜0.9 秒遅れて反応し、c. 最初の 0.3 秒は全停止、そのあと ALERT。"""
    x = cfg["behavior"]["expression"]
    for seed in range(4):
        s = SimSession(cfg, BodyPose(200.0, 400.0, 0.0), seed=seed)
        run_until(s, 3.0)
        s.people = [SimPerson(900.0, -300.0)]
        t0 = s.t
        ev: list = []
        frozen_seen = []
        run_until(s, t0 + 1.5, ev, hook=lambda ss: frozen_seen.append((ss.t, ss.anim.frozen)))
        t_react = next(t for t, e in ev if e.startswith("反応開始"))
        lo, hi = x["reaction_delay_s"]
        assert lo - 0.05 <= t_react - t0 <= hi + 0.05, seed
        frozen = [t for t, f in frozen_seen if f]
        assert frozen and max(frozen) - min(frozen) == pytest.approx(x["surprise_freeze_s"], abs=0.05)
        assert s.brain.fsm.state == "ALERT"


def test_approach_two_stage_stops_short_and_slow_near_person(cfg: dict) -> None:
    """f. 2段階の接近、1m 以内は 8cm/s 以下、頭先端から 400mm 手前で停止（人がマットの上にいる場合）。"""
    c = cfg["behavior"]["controller"]
    # 尾をマット奥に置き、来場者（y<0）へ向かって十分な距離を進める配置にする
    s = SimSession(cfg, BodyPose(300.0, 1150.0, -math.pi / 2), seed=2)
    person = np.array([600.0, -500.0])
    s.people = [SimPerson(*person)]
    ev: list = []
    track: list[tuple[float, np.ndarray, float]] = []

    cmd_near: list[float] = []

    def rec(ss: SimSession) -> None:
        if ss.snake is not None:
            track.append((ss.t, np.array([ss.snake.x, ss.snake.y]), ss.brain.ctrl.head_distance(ss.snake, person)))
            if ss.brain.drive.moving and np.linalg.norm(track[-1][1] - person) <= c["near_person_mm"]:
                cmd_near.append(ss.brain.drive.speed_mm_s)

    run_until(s, 60.0, ev, hook=rec)
    names = [e for _, e in ev]
    assert "接近: 60% で一時停止" in names, names
    assert min(d for _, _, d in track) >= c["stop_distance_mm"] - 10.0
    assert any("→ENGAGE" in e for e in names)
    # 1m 以内: 指令速度は必ず 8cm/s 以下（気づく前の巡回中も）
    assert cmd_near and max(cmd_near) <= c["near_speed_limit_mm_s"]
    # 実際の速さ（3 秒窓。1 秒窓だと蛇行の横振れを速さと数えてしまう）
    pts = [(t, p) for t, p, _ in track if np.linalg.norm(p - person) <= c["near_person_mm"]]
    win = int(3.0 * cfg["behavior"]["tick_hz"])
    speeds = [np.linalg.norm(p1 - p0) / (t1 - t0) for (t0, p0), (t1, p1) in zip(pts[::win], pts[win::win])]
    assert speeds and max(speeds) <= c["near_speed_limit_mm_s"] * 1.1


def test_touch_makes_it_go_limp(cfg: dict) -> None:
    """i. タッチ → PETTED: トルク 60%・首を下げる・目を暗く、2 秒でトルクが戻る。"""
    s = SimSession(cfg, BodyPose(300.0, 600.0, 0.0), seed=1)
    run_until(s, 2.0)
    s.touch(True)
    run_until(s, 2.3)
    s.touch(False)
    assert s.brain.fsm.state == "PETTED"
    assert s.bus._axes[7].torque_ratio == pytest.approx(
        cfg["poses"]["relax"]["torque_ratio"] * cfg["safety_limits"]["torque"]["software_torque_limit_ratio"])
    assert s.head.eye_rgb_brightness[3] == cfg["behavior"]["eyes"]["PETTED"][3]
    run_until(s, 2.3 + cfg["behavior"]["expression"]["petted_s"] + 0.2)
    assert s.bus._axes[7].torque_ratio == pytest.approx(
        cfg["safety_limits"]["torque"]["software_torque_limit_ratio"])   # 演出が終わっても安全上限まで


def test_hot_servos_make_it_coil_and_rest(cfg: dict) -> None:
    """Energy はサーボ温度から: 熱くなると COIL_REST（尾から順にとぐろ）。"""
    s = SimSession(cfg, BodyPose(300.0, 600.0, 0.0), seed=4,
                   overrides={"mock_servo": {"heat_tau_s": 6.0, "ambient_c": 45.0}})   # 暑い会場を模擬
    ev: list = []
    run_until(s, 60.0, ev)
    assert any("→COIL_REST" in e for _, e in ev)
    assert s.brain.internal.heat_c is not None


def test_lonely_and_uncurious_goes_to_sleep(cfg: dict) -> None:
    s = SimSession(cfg, BodyPose(300.0, 600.0, 0.0), seed=5)
    s.brain.internal._ch["curiosity"].x0 = 0.02
    s.brain.internal._ch["curiosity"].gains = {}
    s.brain.internal.values["curiosity"] = 0.02
    ev: list = []
    run_until(s, 15.0, ev)
    assert s.brain.fsm.state == "SLEEP"
    assert s.head.eye_rgb_brightness[3] == cfg["behavior"]["eyes"]["SLEEP"][3]


@pytest.mark.parametrize("seed", [6, 3, 11])
def test_patrol_keeps_off_the_mat_edge(cfg: dict, seed: int) -> None:
    """巡回中、マット端に押し付けられる（クランプされる）時間は 1% 未満。

    BodyPose は尾端の姿勢。尾を x=150 に置くと頭は x=1000 で、マット（1200）の内側に収まる。
    """
    s = SimSession(cfg, BodyPose(150.0, 600.0, 0.0), seed=seed)
    clamped, moved = [], []
    run_until(s, 90.0, hook=lambda ss: (clamped.append(ss.world.clamped),
                                        moved.append((ss.snake.x, ss.snake.y) if ss.snake else None)))
    assert np.mean(clamped) < 0.01
    pts = np.array([m for m in moved if m is not None])
    travel = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
    assert travel > 5000.0          # 止まって 0% になっていないこと（90秒で 5m 以上動く）


def test_person_rushing_in_causes_retreat(cfg: dict) -> None:
    s = SimSession(cfg, BodyPose(200.0, 600.0, 0.0), seed=7)
    s.people = [SimPerson(600.0, -1000.0)]
    run_until(s, 12.0)
    ev: list = []

    def rush(ss: SimSession) -> None:
        k = min(max(ss.t - 12.0, 0.0), 2.0)
        ss.people = [SimPerson(600.0, -1000.0 + 450.0 * k)]

    run_until(s, 20.0, ev, hook=rush)
    assert any("→RETREAT" in e for _, e in ev)
    assert math.isfinite(s.brain.internal.stress)
