"""STEP 5: 知覚（ホモグラフィ・ArUco・人の追跡）のテスト。"""
from __future__ import annotations

import math
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

from serpens.config import load_config
from serpens.perception.aruco_locator import ArucoLocator
from serpens.perception.homography import FloorHomography, apparent_floor_point, correct_height
from serpens.perception.person_detector import PersonDetection, PersonTracker
from serpens.perception.snake_pose import wrap_pi
from serpens.sim.virtual_camera import SimPerson, SimPersonDetector, VirtualCamera
from serpens.sim.world import BodyPose, World

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture()
def cfg() -> dict:
    c = load_config()
    c["camera"]["position_mm"] = c["virtual_camera"]["position_mm"]
    return c


# ---- ホモグラフィ ------------------------------------------------------------------
def test_homography_roundtrip_and_save(cfg: dict, tmp_path: Path) -> None:
    h = FloorHomography.from_clicks(cfg, cfg["virtual_camera"]["mat_corners_px"], (1920, 1080))
    corners = h.image_to_floor(np.array(cfg["virtual_camera"]["mat_corners_px"], float))
    assert np.allclose(corners, [[0, 0], [1200, 0], [1200, 1200], [0, 1200]], atol=1e-6)
    pts = np.array([[123.0, 456.0], [-300.0, 1500.0]])
    assert np.allclose(h.image_to_floor(h.floor_to_image(pts)), pts, atol=1e-6)
    p = tmp_path / "日本語フォルダ" / "h.json"
    h.save(p)
    h2 = FloorHomography.load(p)
    assert np.allclose(h2.H, h.H) and h2.image_size == (1920, 1080)


def test_height_parallax_correction_is_inverse() -> None:
    cam = [600.0, -900.0, 1500.0]
    true = np.array([900.0, 700.0])
    seen = apparent_floor_point(true, 50.0, cam)
    assert np.linalg.norm(seen - true) > 20                    # 補正しないと数十 mm ずれる
    assert np.allclose(correct_height(seen, 50.0, cam), true)
    assert np.allclose(correct_height(seen, 50.0, None), seen)


# ---- ArUco -------------------------------------------------------------------------
@pytest.mark.parametrize("pose", [BodyPose(150, 600, 0.0), BodyPose(200, 1000, 0.0), BodyPose(1000, 150, math.pi / 2),
                                  BodyPose(1100, 900, math.pi)])
def test_aruco_locates_markers_on_virtual_camera(cfg: dict, pose: BodyPose) -> None:
    """仮想カメラの画像から、首と尾のマーカを数 mm 以内で床座標にする（視差補正込み）。"""
    cam = VirtualCamera(cfg)
    w = World(cfg, pose)
    img = cam.render(w, [])
    loc = ArucoLocator(cfg, cam.homography)
    snake, obs = loc.update(img, 0.0, 10.0)
    assert np.linalg.norm(obs.neck_mm - w.marker_xy("neck")) < 5.0
    assert np.linalg.norm(obs.tail_mm - w.marker_xy("tail")) < 5.0
    assert snake is not None and abs(wrap_pi(snake.theta_body_raw - w.snake_pose()[2])) < math.radians(2)
    assert snake.theta_head == pytest.approx(snake.theta_body + math.radians(10.0))


def test_aruco_rejects_impossible_gap(cfg: dict) -> None:
    cam = VirtualCamera(cfg)
    loc = ArucoLocator(cfg, cam.homography)
    loc.max_gap_mm = 100.0
    obs = loc.observe(cam.render(World(cfg, BodyPose(150, 600, 0.0)), []))
    assert obs.rejected_gap and obs.tail_mm is None and obs.neck_mm is not None


def test_make_aruco_sheet_is_detectable(tmp_path: Path) -> None:
    out = tmp_path / "sheet.png"
    subprocess.run([sys.executable, str(ROOT / "tools" / "make_aruco.py"), "--out", str(out), "--spares", "0"],
                   check=True, capture_output=True, cwd=ROOT)
    g = cv2.imdecode(np.fromfile(str(out), np.uint8), cv2.IMREAD_GRAYSCALE)   # imread は日本語パス不可
    d = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    corners, ids, _ = d.detectMarkers(g)
    assert sorted(ids.flatten().tolist()) == [0, 1]
    side_px = np.linalg.norm(corners[0].reshape(4, 2)[0] - corners[0].reshape(4, 2)[1])
    assert side_px == pytest.approx(40 / 25.4 * 300, rel=0.02)            # 40mm @ 300dpi


# ---- 人 ----------------------------------------------------------------------------
def det(x: float, y: float) -> PersonDetection:
    return PersonDetection((0, 0, 1, 1), 1.0, np.array([x, y]))


def test_sim_person_detector_foot_to_floor(cfg: dict) -> None:
    cam = VirtualCamera(cfg)
    cam.render(World(cfg), [SimPerson(1500.0, 900.0), SimPerson(-400.0, 700.0)])
    found = SimPersonDetector(cam, cam.homography).detect(np.zeros(1))
    assert len(found) == 2
    assert min(np.linalg.norm(d.floor_mm - [1500, 900]) for d in found) < 2.0


def test_tracker_nearest_hysteresis_hold_and_range(cfg: dict) -> None:
    tr = PersonTracker(cfg)
    snake = np.array([600.0, 600.0])
    sw, hold = cfg["person"]["switch_hysteresis_s"], cfg["person"]["hold_s"]
    far_away = det(600.0, 600.0 + cfg["person"]["max_range_mm"] + 700)
    assert tr.update(0.0, [far_away], snake) is None                    # 範囲外は無視
    a, b = (1500.0, 600.0), (600.0, 1500.0)
    assert np.allclose(tr.update(0.1, [det(*a), det(*b)], snake).floor_mm, a)   # 同距離 → 先に見つけた方
    near_b = (600.0, 1100.0)                                            # B が近づく
    t = 0.2
    while t < 0.2 + sw - 0.1:
        assert np.allclose(tr.update(t, [det(*a), det(*near_b)], snake).floor_mm, a)   # まだ切り替えない
        t += 0.1
    t += 0.2
    assert np.allclose(tr.update(t, [det(*a), det(*near_b)], snake).floor_mm, near_b)
    assert tr.switches == 1
    t_last = t
    t += hold - 0.2
    assert np.allclose(tr.update(t, [], snake).floor_mm, near_b)        # 0人でも hold_s は保持
    t = t_last + hold + 0.2
    assert tr.update(t, [], snake) is None


def test_tracker_follows_moving_target(cfg: dict) -> None:
    tr = PersonTracker(cfg)
    for k in range(20):
        p = tr.update(k * 0.1, [det(1500.0 - 20 * k, 600.0)], None)
    assert p.floor_mm[0] == pytest.approx(1500.0 - 20 * 19)
