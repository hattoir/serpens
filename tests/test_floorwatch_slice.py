"""フェーズ 5 の縦一本（**すべて模擬**）: Home AI モック → inspect_point → 移動（KINEMATIC_SIM）→ 撮影（合成）→ 判定 →
floor_finding（推定した自己位置 + σ）→ Home AI モックが受けて通知する。stop は途中で mission を捨てる。
"""
from __future__ import annotations

import copy
import math
from pathlib import Path

import numpy as np
import pytest

from serpens.api.bridge import LoopbackBroker
from serpens.api.endpoint import Endpoint
from serpens.config import load_config
from serpens.floorwatch.executor import FloorWatchExecutor
from serpens.floorwatch.scene import FloorObject, FloorScene, FloorStain
from serpens.localization.tag_map import TagMap
from serpens.sim.session import SimSession
from serpens.sim.world import BodyPose
from tests.mocks.home_ai_mock import HomeAiMock


START = BodyPose(150.0, 150.0, math.pi / 4)      # 尾をマットの隅に置き、斜め前に進む余地を作る


def rig(cfg: dict, scene: FloorScene, tmp_path: Path, seed: int = 3):
    cfg = copy.deepcopy(cfg)
    cfg["floor_watch"]["camera"]["floor_mode"] = "synthetic_ref"            # 合成は設計値の解像度で（実機モードは UXGA）
    session = SimSession(cfg, start=START, seed=seed)
    session.step()
    tm = TagMap.from_cfg(cfg)
    ex = FloorWatchExecutor(cfg, session, scene, tm, np.random.default_rng(seed), findings_dir=tmp_path / "findings")
    broker = LoopbackBroker()
    clock = lambda: int(session.t * 1000)  # noqa: E731
    ep = Endpoint(broker, ex, map_version=tm.map_version, data_source="SIMULATION", clock_ms=clock)
    ex.attach(ep)
    home = HomeAiMock(broker, map_version=tm.map_version, clock_ms=clock)
    return session, ex, ep, home


def ahead(ex, dist_m: float, side_m: float = 0.0) -> np.ndarray:
    """いまの頭先端から進行方向に dist_m（左に side_m）の home 座標。"""
    head, yaw = ex._head()
    return head + dist_m * np.array([math.cos(yaw), math.sin(yaw)]) + side_m * np.array([-math.sin(yaw), math.cos(yaw)])


def run(ex, home, task_id: str, max_ticks: int = 6000) -> str:
    for _ in range(max_ticks):
        ex.tick()
        st = home.statuses_of(task_id)
        if st and st[-1] in ("done", "failed", "aborted", "rejected"):
            return st[-1]
    return "timeout"


@pytest.fixture(scope="module")
def cfg() -> dict:
    return load_config()


def test_inspect_point_round_trip_reports_a_washer_with_estimated_pose(cfg: dict, tmp_path: Path) -> None:
    scene = FloorScene()
    session, ex, ep, home = rig(cfg, scene, tmp_path)
    target = ahead(ex, 0.20)                                                  # 頭の 20cm 先を見に行く
    obj = target.copy()
    scene.objects.append(FloorObject("washer", obj[0], obj[1], 20.0, 1.5, 0.9, True))
    first = home.task("inspect_point", target={"x_m": target[0], "y_m": target[1], "yaw_rad": 0.0})
    assert home.statuses_of(first) == ["rejected"] and "未初期化" in home.last_reason(first)   # タグをまだ見ていない
    for _ in range(5):
        ex.tick()
    assert ex.est.estimate(session.t).source == "APRILTAG"                    # 正面の壁のタグで初期化
    tid = home.task("inspect_point", target={"x_m": target[0], "y_m": target[1], "yaw_rad": 0.0})
    assert home.statuses_of(tid) == ["accepted"]
    assert run(ex, home, tid) == "done", home.last_reason(tid)
    findings = [e["finding"] for e in home.events if e["event"] == "floor_finding"]
    assert findings, "floor_finding が出ていない"
    f = findings[0]
    assert f["frame_id"] == "home" and f["map_version"] == ex.tag_map.map_version and f["task_id"] == tid
    assert f["risk"]["mandatory_notify"] and "metal_disc" in f["risk"]["critical_kinds"]  # 鏡面の円盤 → 必ず通知
    assert f["size"]["height_mm"] is None and f["size"]["height_reason"] == "specular_break"
    assert f["pose"]["pose_source"] in ("APRILTAG", "ODOMETRY_IMU") and 0 < f["pose"]["sigma_xy_m"] < 0.3
    assert math.hypot(f["pose"]["x_m"] - obj[0], f["pose"]["y_m"] - obj[1]) < 3 * f["pose"]["sigma_xy_m"] + 0.05
    assert home.notified and any(f["finding_id"] in n for n in home.notified)
    for p in f["photos"]:
        assert Path(p["crop_path"]).exists() and p["w_px"] < 640                # 切り抜きだけ（原画像は保存しない）
    for _ in range(100):                                                       # 歩容の振幅が消えるまで
        ex.tick()
    assert not session.anim.gait.active and not session.brain.loco.drive.moving and ep.active_task_id is None
    assert home.safety_fresh() and home.safety["mode"] == "DRIVING"           # 機体の状態が写っている（周期送信で新しい）


def test_empty_spot_finishes_without_finding_and_stain_is_not_reported(cfg: dict, tmp_path: Path) -> None:
    scene = FloorScene()
    session, ex, ep, home = rig(cfg, scene, tmp_path, seed=4)
    target = ahead(ex, 0.18, 0.03)
    scene.stains.append(FloorStain(target[0], target[1], 18.0, 0.3))
    for _ in range(5):
        ex.tick()
    tid = home.task("inspect_point", target={"x_m": target[0], "y_m": target[1], "yaw_rad": 0.0})
    assert run(ex, home, tid) == "done"
    assert not [e for e in home.events if e["event"] == "floor_finding"]
    assert "候補 0 件" in home.last_reason(tid)


def test_stop_during_goto_aborts_and_locks_until_operator(cfg: dict, tmp_path: Path) -> None:
    session, ex, ep, home = rig(cfg, FloorScene(), tmp_path, seed=5)
    for _ in range(5):
        ex.tick()
    far = ahead(ex, 0.45, -0.2)
    tid = home.task("inspect_point", target={"x_m": far[0], "y_m": far[1], "yaw_rad": 0.0})
    for _ in range(40):
        ex.tick()
    assert ex.mission is not None and ex.mission.phase == "GOTO"
    stop = home.task("stop", reason="人が来た")
    assert home.statuses_of(stop) == ["accepted"] and home.statuses_of(tid)[-1] == "aborted"
    for _ in range(20):
        ex.tick()
    assert ex.mission is None and not session.brain.loco.drive.moving and not session.stop.moving_allowed
    assert home.safety["mode"] == "ARMED_HOLD" and home.safety["stop_reason"] == "OPERATOR"
    near = ahead(ex, 0.15)
    again = home.task("inspect_point", target={"x_m": near[0], "y_m": near[1], "yaw_rad": 0.0})
    assert home.statuses_of(again) == ["rejected"] and "人の操作" in home.last_reason(again)
    ep.operator_resume()                                                        # API のロック解除（人）
    held = home.task("inspect_point", target={"x_m": near[0], "y_m": near[1], "yaw_rad": 0.0})
    assert home.statuses_of(held) == ["rejected"] and "停止中" in home.last_reason(held)   # 機体はまだ HOLD
    assert session.request_start(source="操作")[0]                                # 機体側の開始操作（人）
    ok = home.task("inspect_point", target={"x_m": near[0], "y_m": near[1], "yaw_rad": 0.0})
    assert home.statuses_of(ok) == ["accepted"]
    unsupported = home.task("return_dock")
    assert run(ex, home, ok) == "done" and home.statuses_of(unsupported)[-1] in ("accepted", "failed", "running")
