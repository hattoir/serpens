"""Phase 4: 模擬画像の Vision → World State → 行動 → 仮想機体 の閉ループと、Vision の故障注入。

真値は採点にだけ使う。数値の合格線は**模擬画像での目安**で、実カメラの性能ではない（SIMULATED）。
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from serpens.config import load_config
from serpens.perception.aruco_locator import ArucoLocator
from serpens.perception.snake_pose import SnakePoseTracker
from serpens.safety import AutonomyInputs, DriveState, autonomy_blockers
from serpens.sim.session import SimSession
from serpens.sim.sim_vision import SimVisionObserver, VisionFaults, Window
from serpens.sim.virtual_camera import VirtualCamera
from serpens.sim.world import World
from serpens.world_state import PoseSource
from simulation.bridge import world_state_from_session
from simulation.virtual_person import scenario
from simulation.vision_loop import START, VisionLoop
from tests.helpers import steady_patrol

FAR_GHOST = (100.0, 1100.0)          # マットの奥の隅（胴体から遠い）


@pytest.fixture(scope="module")
def cfg() -> dict:
    return steady_patrol(load_config())     # 歩容が出ている最中の性質を見る（stop-and-go は別の試験）


def run(cfg: dict, faults: VisionFaults | None = None, seconds: float = 10.0) -> VisionLoop:
    loop = VisionLoop(cfg, faults, people=scenario(cfg))
    assert loop.start_when_seen(), loop.session.blockers()
    loop.run(seconds)
    return loop


# ---- 部品 ------------------------------------------------------------------------------
def test_duplicate_marker_ids_are_not_used(cfg: dict) -> None:
    """同じ ID が2枚写ったら、検出順で偶然どちらかを選ばず「見えていない」とする。"""
    cam = VirtualCamera(cfg)
    loc = ArucoLocator(cfg, cam.homography)
    world = World(cfg, START)
    img = cam.render(world, [])
    assert loc.observe(img).neck_mm is not None
    cam.draw_extra_marker(img, "neck", np.array(FAR_GHOST), world.marker_tangent("neck"))
    obs = loc.observe(img)
    assert obs.duplicate_ids == (int(cfg["markers"]["neck_id"]),)
    assert obs.neck_mm is None and obs.tail_mm is not None


def test_tracker_rejects_jumps_and_needs_both_markers_to_recover(cfg: dict) -> None:
    tr = SnakePoseTracker(cfg)
    neck, tail = np.array([600.0, 300.0]), np.array([600.0, 800.0])
    assert tr.update(0.0, neck, tail, 0.0) is not None
    p = tr.update(0.1, np.array(FAR_GHOST), None, 0.0)          # 0.1s で 900mm 以上は飛ばない
    assert (p.x, p.y) == (600.0, 300.0) and p.t == 0.0 and tr.rejected_jumps == 1
    assert tr.update(0.2, neck + [5.0, 0.0], None, 0.0).t == 0.2   # もっともらしい動きは通す
    late = 0.2 + tr.stale_after_s + 0.1
    assert tr.update(late, np.array(FAR_GHOST), None, 0.0).t == 0.2   # 位置不明から首だけでは戻らない
    back = tr.update(late + 0.1, neck, tail, 0.0)
    assert back.t == pytest.approx(late + 0.1) and back.neck_seen and back.tail_seen


def test_vision_session_waits_until_the_pose_is_seen(cfg: dict) -> None:
    """Vision の模擬は待機から始まり、自己位置が入るまで開始できない。真値の模擬は従来どおり。"""
    s = SimSession(cfg, START, seed=1, observer=SimVisionObserver(cfg))
    assert s.pose_source == "aruco_sim" and s.stop.state is DriveState.HOLD
    assert any("自己位置" in w for w in s.blockers())
    ok, _ = s.request_start("test")
    assert not ok
    truth = SimSession(cfg, START, seed=1)
    assert truth.blockers() == [] and truth.stop.moving_allowed


def test_simulated_vision_never_satisfies_the_real_start_condition(cfg: dict) -> None:
    """模擬画像の ArUco（aruco_sim）では実機の自律走行を許可しない。"""
    why = autonomy_blockers(cfg, AutonomyInputs(True, "aruco_sim", 0.1, True, True, True, 9, 9, 0.2))
    assert any("実観測ではない" in w for w in why)


# ---- 閉ループ ---------------------------------------------------------------------------
def test_clean_vision_closes_the_loop(cfg: dict) -> None:
    """故障なし: 画像から推定した位置で走り、見失わない。World State は ARUCO（模擬）。"""
    loop = run(cfg)
    m = loop.metrics.summary()
    assert loop.metrics.stops == [] and loop.session.stop.moving_allowed
    assert m["pos_err_p95_mm"] < 60.0, m             # 検出誤差 + 遅れ（模擬画像での目安）
    assert m["heading_err_p95_deg"] < 10.0, m
    assert m["age_max_s"] <= loop.stale_after_s, m
    assert m["off_mat_steps"] == 0
    assert m["travelled_mm"] > 300.0, m               # 立っているだけでなく、実際に走った
    assert loop.vision.frames >= 9 * 10               # 10Hz で撮っている
    ws = world_state_from_session(loop.session)
    assert ws.robot.source is PoseSource.ARUCO and ws.robot.simulated
    assert PoseSource.GROUND_TRUTH_SIM not in ws.sources
    assert ws.any_vision and not ws.any_real_vision
    assert all(p.source is PoseSource.PERSON_DETECTOR and p.simulated for p in ws.people)


def test_camera_blackout_stops_and_does_not_auto_resume(cfg: dict) -> None:
    """カメラが止まったら自己位置の古さで停止し、映像が戻っても勝手に走り出さない。"""
    loop = run(cfg, VisionFaults(blackout=[Window(5.0, 7.0)]))
    m = loop.metrics
    assert len(m.stops) == 1 and "自己位置" in m.stops[0]
    assert m.moved_while_stale_mm < 50.0, m.moved_while_stale_mm   # 止まるまでの惰性だけ
    assert loop.session.stop.state is DriveState.HOLD             # 映像は 7s で戻っている
    assert not loop.snake.device.driving
    assert not loop.session.request_start("操作")[0]              # 位置は戻ったが、人の確認が先
    assert any("確認待ち" in w for w in loop.session.pose_blockers())
    assert loop.session.acknowledge_pose("操作")[0]
    assert loop.session.request_start("操作")[0]                  # 確認 → 開始操作で再開


def test_dropout_and_neck_occlusion_are_tolerated(cfg: dict) -> None:
    """3割のフレーム欠落と、首マーカを2秒隠すことでは止まらない（尾から推定）。"""
    faults = VisionFaults(dropout_ratio=0.3, occlude=[(Window(5.0, 7.0), "neck")])
    loop = run(cfg, faults)
    m = loop.metrics.summary()
    assert loop.metrics.stops == [], loop.metrics.stops
    assert loop.vision.dropped > 0
    assert m["age_max_s"] <= loop.stale_after_s and m["pos_err_p95_mm"] < 80.0, m


def test_far_ghost_marker_with_hidden_neck_is_rejected_and_stops(cfg: dict) -> None:
    """本物の首が隠れ、遠くに偽の首マーカがある: 位置を飛ばさず、見失いとして止まる。"""
    faults = VisionFaults(occlude=[(Window(5.0, 8.0), "neck")],
                          ghost_marker=[(Window(5.0, 8.0), "neck", FAR_GHOST)])
    loop = run(cfg, faults, seconds=8.0)
    assert loop.session.snake_tracker.rejected_jumps > 0
    assert len(loop.metrics.stops) == 1
    assert not loop.snake.device.driving
    assert loop.metrics.moved_while_stale_mm < 50.0


def test_ghost_person_is_reported_as_a_detection_not_truth(cfg: dict) -> None:
    """偽の人は検出として World State に出る（真値とは別）。行動がそれを追うのは既知の限界。"""
    loop = run(cfg, VisionFaults(ghost_person=[(Window(1.0, 20.0), (300.0, -300.0))]), seconds=4.0)
    ws = world_state_from_session(loop.session)
    xs = [(round(p.x_mm), round(p.y_mm)) for p in ws.people]
    assert (300, -300) in xs
    assert all(p.source is PoseSource.PERSON_DETECTOR for p in ws.people)
    truth = [(p.x_mm, p.y_mm) for p in loop.session.people]
    assert all(math.hypot(x - 300.0, y + 300.0) > 1.0 for x, y in truth)
