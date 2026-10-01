"""patrol_route（各点で inspect）と highlight_point（物 → 人 → 物）の模擬（LB-E-014 / SE-E7）。**KINEMATIC_SIM / 合成画像。実機・照明の制御は未接続。**
CSAR（子どもが近いかもしれないときは物を指さない・照らさない）が最優先。"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from serpens.config import load_config
from serpens.floorwatch.csar import FAR
from serpens.floorwatch.highlight import DONE, FAILED, OBJECT1, HighlightMission
from serpens.floorwatch.scene import FloorObject, FloorScene
from tests.test_floorwatch_slice import ahead, rig


@pytest.fixture(scope="module")
def cfg() -> dict:
    return load_config()


# ---- HighlightMission 単体 ---------------------------------------------------------------------------------
def test_gesture_is_object_person_object_and_ends_by_duration() -> None:
    seen: list[str] = []
    ok = True
    h = HighlightMission(np.array([100.0, 0.0]), 10.0, lambda t: ok, look_s=1.0)
    person = np.array([0.0, 200.0])
    t = 0.0
    while h.active and t < 20:
        h.tick(t, lambda xy: (seen.append(f"{xy[0]:.0f},{xy[1]:.0f}") or True), person)
        t += 0.1
    assert h.phase == DONE and "物 → 人 → 物" in h.reason
    order = [s for i, s in enumerate(seen) if i == 0 or s != seen[i - 1]]
    assert order == ["100,0", "0,200", "100,0"]


def test_person_phase_is_skipped_when_no_person_is_visible() -> None:
    seen = []
    h = HighlightMission(np.array([100.0, 0.0]), 10.0, lambda t: True, look_s=1.0)
    t = 0.0
    while h.active and t < 20:
        h.tick(t, lambda xy: (seen.append(tuple(xy)) or True), None)
        t += 0.1
    assert h.phase == DONE and set(seen) == {(100.0, 0.0)}


def test_duration_cuts_the_gesture_short() -> None:
    h = HighlightMission(np.array([100.0, 0.0]), 1.2, lambda t: True, look_s=1.0)
    t = 0.0
    while h.active and t < 5:
        h.tick(t, lambda xy: True, np.array([0.0, 200.0]))
        t += 0.1
    assert h.phase == DONE and h.log[-1][0] <= 1.4


def test_child_gate_closing_midway_stops_pointing_immediately() -> None:
    state = {"ok": True}
    h = HighlightMission(np.array([100.0, 0.0]), 10.0, lambda t: state["ok"], look_s=1.0)
    h.tick(0.0, lambda xy: True, None)
    assert h.phase == OBJECT1
    state["ok"] = False
    h.tick(0.1, lambda xy: pytest.fail("子どもが近いのに物へ向けた"), None)
    assert h.phase == FAILED and "CSAR" in h.reason


def test_target_outside_the_head_yaw_range_fails_without_turning() -> None:
    h = HighlightMission(np.array([100.0, 0.0]), 10.0, lambda t: True)
    h.tick(0.0, lambda xy: False, None)
    assert h.phase == FAILED and "範囲外" in h.reason


# ---- executor との接続 ---------------------------------------------------------------------------------------
def _ready(cfg: dict, tmp_path: Path, washer_at: float | None = None):
    """washer_at: 頭の先から進行方向にこの距離（m）に、ワッシャーを置く（**ticks の前に**置く。slice の試験と同じ順）。"""
    scene = FloorScene()
    session, ex, ep, home = rig(cfg, scene, tmp_path)
    target = None
    if washer_at is not None:
        target = ahead(ex, washer_at)
        scene.objects.append(FloorObject("washer", target[0], target[1], 20.0, 1.5, 0.9, True))
    for _ in range(5):
        ex.tick()
    if washer_at is not None:
        return scene, session, ex, home, target
    return scene, session, ex, home


def _run(ex, home, tid: str, max_ticks: int = 8000) -> str:
    for _ in range(max_ticks):
        ex.tick()
        st = home.statuses_of(tid)
        if st and st[-1] in ("done", "failed", "aborted", "rejected"):
            return st[-1]
    return "timeout"


def test_highlight_point_far_child_is_done_and_the_body_does_not_walk(cfg: dict, tmp_path: Path) -> None:
    scene, session, ex, home = _ready(cfg, tmp_path)
    assert ex.csar.state(session.t) == FAR
    before = session.world.snake_pose()
    target = ahead(ex, 0.15)
    tid = home.task("highlight_point", target={"x_m": target[0], "y_m": target[1], "yaw_rad": 0.0}, duration_s=6.0)
    assert _run(ex, home, tid) == "done"
    after = session.world.snake_pose()
    assert np.hypot(after[0] - before[0], after[1] - before[1]) < 30.0           # 胴は歩いていない（頭ヨーだけ）


def test_highlight_point_is_refused_when_a_child_may_be_near_and_does_nothing(cfg: dict, tmp_path: Path) -> None:
    scene, session, ex, home = _ready(cfg, tmp_path)
    ex.csar.hint_near(session.t, 1e6)
    called = []
    ex._look_toward = lambda t, xy: (called.append(1) or True)
    target = ahead(ex, 0.15)
    tid = home.task("highlight_point", target={"x_m": target[0], "y_m": target[1], "yaw_rad": 0.0})
    assert _run(ex, home, tid, max_ticks=50) == "failed" and "子どもが近い" in home.last_reason(tid) and not called


def test_highlight_stops_when_a_child_approaches_mid_gesture(cfg: dict, tmp_path: Path) -> None:
    scene, session, ex, home = _ready(cfg, tmp_path)
    target = ahead(ex, 0.15)
    tid = home.task("highlight_point", target={"x_m": target[0], "y_m": target[1], "yaw_rad": 0.0}, duration_s=30.0)
    for _ in range(40):
        ex.tick()
    assert not home.statuses_of(tid)[-1] in ("done", "failed")
    ex.csar.hint_near(session.t, 1e6)                                            # 途中で子どもが近づく
    assert _run(ex, home, tid, max_ticks=200) == "failed" and "CSAR" in home.last_reason(tid)


def test_patrol_route_inspects_each_waypoint_in_order_and_reports_findings(cfg: dict, tmp_path: Path) -> None:
    scene, session, ex, home, p1 = _ready(cfg, tmp_path, washer_at=0.20)
    tid = home.task("patrol_route", waypoints=[{"x_m": float(p1[0]), "y_m": float(p1[1]), "yaw_rad": 0.0}])
    assert home.statuses_of(tid) == ["accepted"]
    assert _run(ex, home, tid) == "done"
    assert "1/1" in home.last_reason(tid)
    findings = [e["finding"] for e in home.events if e["event"] == "floor_finding"]
    assert findings and all(f["task_id"] == tid for f in findings)


def test_patrol_route_continues_after_a_deferred_point_and_says_which_were_unverified(cfg: dict, tmp_path: Path) -> None:
    """子どもが近いままだと、各点の撮影は後回し（未確認）。全部後回しなら failed で、理由に各点の事情が入る。"""
    scene, session, ex, home = _ready(cfg, tmp_path)
    ex.csar.hint_near(session.t, 1e6)
    p1 = ahead(ex, 0.15)
    p2 = ahead(ex, 0.25)
    tid = home.task("patrol_route", waypoints=[{"x_m": float(p1[0]), "y_m": float(p1[1]), "yaw_rad": 0.0}, {"x_m": float(p2[0]), "y_m": float(p2[1]), "yaw_rad": 0.0}])
    assert _run(ex, home, tid, max_ticks=20000) == "failed"
    assert "どの地点も確認できなかった" in home.last_reason(tid) and "CSAR" in home.last_reason(tid)
    assert home.last_reason(tid).count(";") >= 1                               # 2 点とも記録（; 区切り）


def test_stop_during_a_route_aborts_it(cfg: dict, tmp_path: Path) -> None:
    scene, session, ex, home = _ready(cfg, tmp_path)
    p1 = ahead(ex, 0.30)
    tid = home.task("patrol_route", waypoints=[{"x_m": float(p1[0]), "y_m": float(p1[1]), "yaw_rad": 0.0}])
    for _ in range(30):
        ex.tick()
    home.task("stop")
    for _ in range(50):
        ex.tick()
    assert ex._route is None and ex.mission is None and home.statuses_of(tid)[-1] in ("aborted", "failed")
