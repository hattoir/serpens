"""CSAR（Child-Safe Attention Rules）の実行条件。User 採用 2026-09-29、数値は Engineering の案（config floor_watch.csar）。
**模擬**（KINEMATIC_SIM・合成画像）。子どもの近さは Home AI からの「近い」の知らせ（安全側だけ受け取る）で与える。"""
from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from serpens.config import load_config
from serpens.floorwatch.csar import FAR, NEAR, UNKNOWN, ChildProximity
from serpens.floorwatch.mission import InspectMission
from serpens.floorwatch.scene import FloorObject, FloorScene
from tests.test_floorwatch_slice import ahead, rig, run


@pytest.fixture(scope="module")
def cfg() -> dict:
    return load_config()


# ---- 子どもの近さ -------------------------------------------------------------------------------
def test_nothing_seen_is_unknown_and_unknown_is_not_far(cfg: dict) -> None:
    cp = ChildProximity.from_cfg(cfg)
    assert cp.state(0.0) == UNKNOWN and not cp.allows_attention_to_object(0.0)


def test_far_is_trusted_only_for_a_short_time_after_looking(cfg: dict) -> None:
    cp = ChildProximity.from_cfg(cfg)
    cp.observe(10.0, None, sensing=True)
    assert cp.state(10.0 + cp.far_trust_s * 0.9) == FAR
    assert cp.state(10.0 + cp.far_trust_s * 1.1) == UNKNOWN          # 床を向いたまま長く信じない
    cp.observe(20.0, None, sensing=False)                             # 見られない状態の「いない」は FAR の根拠にしない
    assert cp.state(20.0) == UNKNOWN


def test_anyone_within_the_near_distance_counts_as_a_child_and_holds(cfg: dict) -> None:
    cp = ChildProximity.from_cfg(cfg)
    cp.observe(0.0, cp.near_m * 0.5, sensing=True)                    # 子どもと大人は区別できない → 近い人は NEAR
    cp.observe(0.5, None, sensing=True)                               # すぐ見えなくなっても
    assert cp.state(0.5) == NEAR and cp.state(cp.near_hold_s * 0.5) == NEAR
    assert cp.state(cp.near_hold_s + 0.1) != NEAR                     # 保持の時間が過ぎたら NEAR ではない


def test_a_home_ai_hint_can_only_make_it_nearer(cfg: dict) -> None:
    cp = ChildProximity.from_cfg(cfg)
    cp.observe(0.0, None, sensing=True)
    cp.hint_near(0.0, 10.0)
    assert cp.state(1.0) == NEAR                                      # 「近い」は受け取る（「遠い」で上書きする口は無い）


# ---- mission: 撮影（閃光）は子どもが遠いときだけ ------------------------------------------------------
class _Loco:
    def __init__(self) -> None:
        self.anim = SimpleNamespace(gait=SimpleNamespace(stop=lambda immediate=False: None))
        self.ctrl = SimpleNamespace(creep=lambda *a, **k: ("creep", a, k), drive_to=lambda *a, **k: ("drive",))
        self.stops: list[str] = []

    def stop(self, reason: str) -> None:
        self.stops.append(reason)

    def set_drive(self, *a, **k) -> None:
        pass


def _mission(cfg: dict, ok: dict) -> tuple[InspectMission, list[int]]:
    shots: list[int] = []
    m = InspectMission(cfg, _Loco(), np.zeros(2), lambda: (shots.append(1) or {}), lambda f: [], 0.0,
                       attention_ok=lambda t: ok["v"])
    m._enter("SETTLE", 0.0)
    return m, shots


def test_capture_waits_while_a_child_may_be_near_and_resumes_when_far(cfg: dict) -> None:
    ok = {"v": False}
    m, shots = _mission(cfg, ok)
    snake = SimpleNamespace(x=0.0, y=0.0)
    t = 0.0
    for _ in range(40):                                               # 静止 → 撮影の手前で待つ
        t += 0.1
        m.tick(t, snake, None)
    assert m.phase == "WAIT_CHILD" and not shots
    ok["v"] = True
    for _ in range(40):
        t += 0.1
        m.tick(t, snake, None)
    assert shots, m.events


def test_a_child_arriving_mid_capture_stops_the_capture(cfg: dict) -> None:
    ok = {"v": True}
    m, shots = _mission(cfg, ok)
    snake = SimpleNamespace(x=0.0, y=0.0)
    t = 0.0
    while m.phase != "CAPTURE":
        t += 0.1
        m.tick(t, snake, None)
    ok["v"] = False
    t += 0.1
    m.tick(t, snake, None)
    assert m.phase == "WAIT_CHILD" and not shots


def test_capture_is_deferred_after_the_wait_limit(cfg: dict) -> None:
    ok = {"v": False}
    m, shots = _mission(cfg, ok)
    snake = SimpleNamespace(x=0.0, y=0.0)
    t = 0.0
    limit = float(cfg["floor_watch"]["csar"]["capture_wait_s"])
    while m.active and t < limit + 30:
        t += 0.1
        m.tick(t, snake, None)
    assert m.phase == "DEFERRED" and "CSAR" in m.reason and not shots


# ---- 縦一本（模擬） -------------------------------------------------------------------------------
def _ready(ex, home, cfg, scene, tmp_path):
    for _ in range(5):
        ex.tick()
    return ex


def test_inspect_with_a_child_near_does_not_flash_and_ends_unverified(cfg: dict, tmp_path: Path) -> None:
    scene = FloorScene()
    session, ex, ep, home = rig(cfg, scene, tmp_path)
    _ready(ex, home, cfg, scene, tmp_path)
    target = ahead(ex, 0.20)
    scene.objects.append(FloorObject("button_cell", target[0], target[1], 20.0, 3.2, 0.9, True))
    shots = []
    real = ex._capture
    ex._capture = lambda: (shots.append(1), real())[1]
    tid = home.task("inspect_point", target={"x_m": target[0], "y_m": target[1], "yaw_rad": 0.0})
    ex.csar.hint_near(session.t, 1e6)                                  # ずっと子どもが近い
    assert run(ex, home, tid) == "failed"
    assert "CSAR" in home.last_reason(tid) and not shots               # 物を照らしていない


def test_highlight_point_is_refused_while_a_child_may_be_near(cfg: dict, tmp_path: Path) -> None:
    scene = FloorScene()
    session, ex, ep, home = rig(cfg, scene, tmp_path)
    _ready(ex, home, cfg, scene, tmp_path)
    ex.csar.hint_near(session.t, 1e6)
    target = ahead(ex, 0.20)
    tid = home.task("highlight_point", target={"x_m": target[0], "y_m": target[1], "yaw_rad": 0.0})
    assert run(ex, home, tid, max_ticks=10) == "failed" and "子どもが近い" in home.last_reason(tid)


def test_highlight_point_when_far_is_only_unimplemented_not_a_child_refusal(cfg: dict, tmp_path: Path) -> None:
    scene = FloorScene()
    session, ex, ep, home = rig(cfg, scene, tmp_path)
    _ready(ex, home, cfg, scene, tmp_path)
    assert ex.csar.state(session.t) == FAR
    target = ahead(ex, 0.20)
    tid = home.task("highlight_point", target={"x_m": target[0], "y_m": target[1], "yaw_rad": 0.0})
    assert run(ex, home, tid, max_ticks=10) == "failed" and "子どもが近い" not in home.last_reason(tid)


def test_after_a_finding_the_robot_backs_away_from_the_object(cfg: dict, tmp_path: Path) -> None:
    scene = FloorScene()
    session, ex, ep, home = rig(cfg, scene, tmp_path)
    _ready(ex, home, cfg, scene, tmp_path)
    target = ahead(ex, 0.20)
    scene.objects.append(FloorObject("washer", target[0], target[1], 20.0, 1.5, 0.9, True))
    tid = home.task("inspect_point", target={"x_m": target[0], "y_m": target[1], "yaw_rad": 0.0})
    at_report = None
    for _ in range(6000):
        ex.tick()
        if at_report is None and ex._reported:
            at_report = np.array(session.world.head_tip()[:2])
        st = home.statuses_of(tid)
        if st and st[-1] in ("done", "failed"):
            break
    assert home.statuses_of(tid)[-1] == "done", home.last_reason(tid)
    assert at_report is not None                                       # 離れる前に知らせている（R4）
    obj_mm = ex.frame.to_world(np.array(target))
    before = float(np.linalg.norm(at_report - obj_mm))
    after = float(np.linalg.norm(np.array(session.world.head_tip()[:2]) - obj_mm))
    assert after > before + 0.5 * float(cfg["floor_watch"]["csar"]["retreat_mm"])   # 物から離れた（R3）


def test_home_ai_child_near_hint_in_the_task_blocks_highlight_even_when_the_robot_sees_nobody(cfg: dict, tmp_path: Path) -> None:
    """Task の child_near: true は安全側にだけ効く（Serpens 自身は近くに人を見ていなくても、物を指さない）。false は何も変えない。"""
    from serpens.api.validate import validate_task
    scene = FloorScene()
    session, ex, ep, home = rig(cfg, scene, tmp_path)
    _ready(ex, home, cfg, scene, tmp_path)
    target = ahead(ex, 0.20)
    msg = {"v": 1, "id": "t-child-near-000001", "t_ms": 1, "source": "home_ai", "task": "highlight_point", "frame_id": "home",
           "map_version": "m", "target": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}, "child_near": True}
    assert validate_task(msg) == []
    assert ex.csar.state(session.t) == FAR                             # 模擬では人がいない → 遠い
    tid = home.task("highlight_point", target={"x_m": target[0], "y_m": target[1], "yaw_rad": 0.0}, child_near=True)
    assert run(ex, home, tid, max_ticks=10) == "failed" and "子どもが近い" in home.last_reason(tid)
    assert ex.csar.state(session.t) == NEAR
