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
    assert travel > 2000.0          # 止まったままになっていないこと（stop-and-go で約半分は静止している）


def test_patrol_is_stop_and_go_and_keeps_breathing(cfg: dict) -> None:
    """巡回は動いては止まる（静止 40〜60% 目標）。止まっている間も呼吸で首が動き続ける。"""
    s = SimSession(cfg, BodyPose(150.0, 600.0, 0.0), seed=6)
    paused, neck_while_paused, gait_while_paused = [], [], []

    def watch(ss: SimSession) -> None:
        if ss.brain.fsm.state != "PATROL":
            return
        paused.append(ss.brain.patrol_paused)
        if ss.brain.patrol_paused and not ss.anim.gait.active:
            neck_while_paused.append(ss.anim.last_output["J7"])
            gait_while_paused.append(ss.anim.last_output["J3"] - ss.anim.last_base["J3"])

    run_until(s, 120.0, hook=watch)
    assert 0.35 <= float(np.mean(paused)) <= 0.70, np.mean(paused)
    assert np.ptp(neck_while_paused) > 1.0            # 呼吸は続いている（物体に戻らない）
    assert max(abs(g) for g in gait_while_paused) < 3.0   # 胴体は歩容していない（呼吸ぶんだけ）


RUSH_MM_S = 1200.0       # 駆け込み（歩行 250〜500mm/s より明らかに速い）
WALK_MM_S = 450.0        # 普通に歩いて近づく来場者


def _come_closer(cfg: dict, speed: float) -> list:
    """検出範囲の中（-700）に立って気づかれた人が、12 秒後に -100 まで speed で近づく。"""
    s = SimSession(cfg, BodyPose(200.0, 600.0, 0.0), seed=7)
    s.people = [SimPerson(600.0, -700.0)]
    run_until(s, 12.0)
    assert s.brain.noticed
    ev: list = []

    def come(ss: SimSession) -> None:
        ss.people = [SimPerson(600.0, min(-700.0 + speed * max(ss.t - 12.0, 0.0), -100.0))]

    run_until(s, 20.0, ev, hook=come)
    assert math.isfinite(s.brain.internal.stress)
    return ev


def test_person_rushing_in_causes_retreat(cfg: dict) -> None:
    assert any("→RETREAT" in e for _, e in _come_closer(cfg, RUSH_MM_S))


def test_person_walking_up_does_not_cause_retreat(cfg: dict) -> None:
    """Bug-1 の回帰: 普通に歩いて近づく来場者から逃げない（旧 gain 0.90 ではここで RETREAT した）。"""
    assert not any("→RETREAT" in e for _, e in _come_closer(cfg, WALK_MM_S))


def test_primary_reaction_is_inside_the_causal_window(cfg: dict) -> None:
    """a. 検出から 0.15〜0.25 秒で目が光り J8 が人の側へピクッと動く。驚きの全停止中も呼吸は続く。"""
    x = cfg["behavior"]["expression"]
    s = SimSession(cfg, BodyPose(200.0, 400.0, 0.0), seed=3)
    run_until(s, 3.0)
    s.people = [SimPerson(900.0, -300.0)]                # 右手前 → 頭から見て右（J8 負）
    t0, ev, trace = s.t, [], []
    run_until(s, t0 + 1.6, ev,
              hook=lambda ss: trace.append((ss.t, ss.anim.last_output["J8"], ss.anim.last_output["J7"],
                                            ss.head.eye_rgb_brightness[3], ss.anim.frozen)))
    t_prim = next(t for t, e in ev if e.startswith("一次反応"))
    lo, hi = x["primary_reaction_delay_s"]
    assert lo - 0.03 <= t_prim - t0 <= hi + 0.03
    t_react = next(t for t, e in ev if e.startswith("反応開始"))
    assert t_prim < t_react                              # 一次反応は「間」より前
    yaw0 = next(j8 for t, j8, *_ in trace if t <= t_prim)
    twitch = [j8 - yaw0 for t, j8, *_ in trace if t_prim < t <= t_prim + 2 * x["primary_twitch_s"]]
    assert min(twitch) <= -x["primary_twitch_deg"] * 0.8    # 人のいる側（右）へ動いた
    assert any(br == x["primary_flash_brightness"] for t, _, _, br, _ in trace if t_prim <= t <= t_prim + x["primary_flash_s"])
    lit = [t for t, _, _, br, _ in trace if t_prim <= t < t_react and br == x["primary_flash_brightness"]]
    assert x["primary_flash_s"] - 0.03 <= max(lit) - min(lit) <= x["primary_flash_s"] + 0.06   # 一瞬だけ光って戻る
    neck_frozen = [j7 for _t, _j8, j7, _br, frozen in trace if frozen]
    assert len(neck_frozen) > 5 and max(neck_frozen) - min(neck_frozen) > 0.1   # 全停止中も呼吸で首が動く


def test_no_threat_posture_while_tracking_a_person(cfg: dict) -> None:
    """人を追跡中は J7 > neck.look_max_deg の姿勢を指令しない（フル鎌首はコブラの打撃直前に見える）。"""
    look_max = float(cfg["neck"]["look_max_deg"])
    s = SimSession(cfg, BodyPose(200.0, 400.0, 0.0), seed=2)
    s.people = [SimPerson(600.0, -400.0)]
    worst = 0.0

    def watch(ss: SimSession) -> None:
        nonlocal worst
        if ss.target is not None:
            worst = max(worst, ss.anim.last_base["J7"])

    run_until(s, 60.0, hook=watch)
    assert s.target is not None
    assert worst <= look_max + 1e-6, worst
    # guard そのもの: 追跡中に伸びを命じても J7 は look_max_deg に抑えられる
    s.brain.expr.person_tracked = True
    s.brain.expr.stretch(s.t)
    run_until(s, s.t + 3.0)
    assert s.anim.last_base["J7"] <= look_max + 1e-6 and s.brain.expr.guarded >= 1


def test_stretch_happens_only_when_alone(cfg: dict) -> None:
    """伸び（フル鎌首）は誰もいない巡回の静止中にだけ出る。"""
    s = SimSession(cfg, BodyPose(200.0, 400.0, 0.0), seed=4)
    full = float(cfg["neck"]["full_rear_min_deg"])
    reached = []
    run_until(s, 150.0, hook=lambda ss: reached.append(ss.anim.last_base["J7"] >= full - 1.0))
    assert any("stretch" == n for _t, n in s.brain.expr.log)
    assert any(reached)
