"""実画像経路の観測器（`CameraObserver`）: 録画ファイル → デコード → ArUco → 自己位置。

カメラは無いので、仮想カメラで描いた画像を**動画ファイルに書き出してから**読み直す
（圧縮・デコード・ファイル入力の経路を通す）。真値は採点にだけ使う。SIMULATED。
"""
from __future__ import annotations

import math
from pathlib import Path

import cv2
import numpy as np
import pytest

from serpens.config import load_config
from serpens.motion.gait import GaitEngine
from serpens.perception.camera_observer import CameraObserver, source_kind
from serpens.safety import DriveState
from serpens.sim.session import SimSession
from serpens.sim.virtual_camera import VirtualCamera
from serpens.sim.world import BodyPose, World

FPS = 10.0
SECONDS = 3.0
START = BodyPose(600.0, 600.0, -math.pi / 2)


class Clock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


@pytest.fixture(scope="module")
def cfg() -> dict:
    return load_config()


@pytest.fixture(scope="module")
def recording(cfg: dict, tmp_path_factory: pytest.TempPathFactory) -> tuple[Path, Path, list[np.ndarray]]:
    """歩容で動くヘビを仮想カメラで撮った動画と、その校正ファイルと、フレームごとの首の真値。"""
    d = tmp_path_factory.mktemp("rec")
    cam = VirtualCamera(cfg)
    video, homography = d / "rec.avi", d / "floor.json"
    cam.homography.save(homography)
    w = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"MJPG"), FPS, (cam.w, cam.h))
    assert w.isOpened()
    world, gait = World(cfg, START), GaitEngine(cfg)
    gait.start("forward")
    dt, truth, t = float(cfg["sim"]["dt_s"]), [], 0.0
    for _ in range(int(SECONDS * FPS)):
        for _ in range(int(round(1.0 / FPS / dt))):
            t += dt
            world.step(gait.update(t), dt)
        w.write(cam.render(world, []))
        truth.append(world.marker_xy("neck").copy())
    w.release()
    return video, homography, truth


def test_source_kind_only_trusts_a_live_camera() -> None:
    assert source_kind("0") == "aruco"
    assert source_kind("rec.avi") == "aruco_file" and source_kind("shot.png") == "aruco_file"


def test_recorded_video_yields_poses_close_to_truth(cfg: dict, recording: tuple) -> None:
    """録画 → デコード → 検出 → 床座標 が真値の近くに出る（圧縮込み）。撮影時刻が付く。"""
    video, homography, truth = recording
    clock = Clock()
    obs = CameraObserver(cfg, str(video), clock, homography_path=homography, want_people=False)
    assert obs.pose_source == "aruco_file" and obs.notes == []
    errs, seen = [], 0
    for i, neck in enumerate(truth):
        clock.t = (i + 1) / FPS
        o = obs.step()
        assert o is not None and o.t_capture == clock.t
        assert obs.observe(None, clock.t) is o and obs.observe(None, clock.t) is None   # 1回だけ消費
        if o.neck_mm is not None:
            seen += 1
            errs.append(float(np.linalg.norm(o.neck_mm - neck)))
    assert seen >= 0.9 * len(truth), seen
    assert np.percentile(errs, 95) < 25.0, errs        # 圧縮を通した模擬画像での目安
    assert obs.step() is None                          # 最後まで読んだら観測なし（ループしない）
    obs.release()


def test_session_with_recording_waits_for_acknowledgement(cfg: dict, recording: tuple) -> None:
    """録画を観測器にしたセッションは待機から始まり、位置の確認（K）→ 開始操作で走る。"""
    video, homography, _ = recording
    s = SimSession(cfg, START, seed=1)
    obs = CameraObserver(cfg, str(video), s.clock, homography_path=homography, want_people=False)
    s = SimSession(cfg, START, seed=1, observer=obs, clock=s.clock)
    assert s.pose_source == "aruco_file" and s.stop.state is DriveState.HOLD
    assert not s.request_start("test")[0]
    for _ in range(5):
        obs.step()
        s.step()
    assert s.snake is not None
    assert any("確認待ち" in w for w in s.blockers())
    assert not s.request_start("test")[0]
    ok, msg = s.acknowledge_pose("test")
    assert ok and "x=" in msg
    assert s.request_start("test")[0] and s.stop.moving_allowed
    obs.release()


def test_missing_calibration_gives_no_pose_but_keeps_video(cfg: dict, recording: tuple, tmp_path: Path) -> None:
    video, _, _ = recording
    obs = CameraObserver(cfg, str(video), Clock(), homography_path=tmp_path / "none.json", want_people=False)
    assert any("校正" in n for n in obs.notes)
    assert obs.step() is None and obs.frame is not None
    obs.release()


def test_source_kind_for_real_camera_reaches_the_start_condition(cfg: dict) -> None:
    """カメラ番号なら pose_source は "aruco"（実機の開始条件が要求する出どころ）。録画は違う。"""
    from serpens.safety import AutonomyInputs, autonomy_blockers

    file_why = autonomy_blockers(cfg, AutonomyInputs(True, "aruco_file", 0.1, True, True, True, 9, 9, 0.2))
    assert any("実観測ではない" in w for w in file_why)
