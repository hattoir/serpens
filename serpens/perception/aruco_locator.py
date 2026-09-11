"""ArUco マーカ2枚（尾 ID0 / 首 ID1）からヘビの位置姿勢を求める。

画像 → マーカ検出 → マーカ中心を床座標へ（ホモグラフィ + 高さの視差補正）
     → SnakePoseTracker（位置 = 首マーカ、θ_body = 尾→首 のローパス、θ_head = θ_body + J8）
片方しか見えないときの扱いは SnakePoseTracker を参照。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import cv2
import numpy as np

from serpens.perception.homography import FloorHomography, correct_height
from serpens.perception.snake_pose import SnakePose, SnakePoseTracker


@dataclass
class MarkerObservation:
    """1フレームの検出結果。"""

    neck_mm: np.ndarray | None = None
    tail_mm: np.ndarray | None = None
    corners_px: dict[int, np.ndarray] = field(default_factory=dict)   # 描画用
    rejected_gap: bool = False     # 2枚の距離が離れすぎていて捨てた


def make_detector(cfg: dict[str, Any]) -> cv2.aruco.ArucoDetector:
    """config の辞書で ArUco 検出器を作る。"""
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg["markers"]["dictionary"]))
    params = cv2.aruco.DetectorParameters()
    for name, value in cfg["aruco"]["detector_params"].items():
        if not hasattr(params, name):
            raise ValueError(f"aruco.detector_params: 未知のパラメータ {name}")
        setattr(params, name, type(getattr(params, name))(value))
    return cv2.aruco.ArucoDetector(dictionary, params)


class ArucoLocator:
    """画像からヘビの位置姿勢を出す。"""

    def __init__(self, cfg: dict[str, Any], homography: FloorHomography) -> None:
        m, a = cfg["markers"], cfg["aruco"]
        self.homography = homography
        self.detector = make_detector(cfg)
        self.tail_id, self.neck_id = int(m["tail_id"]), int(m["neck_id"])
        self.height_mm = float(a["marker_height_mm"])
        self.max_gap_mm = float(a["max_marker_gap_mm"])
        pos = cfg["camera"]["position_mm"]
        self.camera_mm = None if pos is None else [float(v) for v in pos]
        self.tracker = SnakePoseTracker(cfg)

    def observe(self, frame: np.ndarray) -> MarkerObservation:
        """マーカを検出して床座標にする（追跡はしない）。"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        corners, ids, _rej = self.detector.detectMarkers(gray)
        obs = MarkerObservation()
        if ids is None:
            return obs
        for c, i in zip(corners, ids.flatten()):
            obs.corners_px[int(i)] = c.reshape(4, 2)
        for mid, attr in ((self.neck_id, "neck_mm"), (self.tail_id, "tail_mm")):
            if mid in obs.corners_px:
                center = obs.corners_px[mid].mean(axis=0)
                p = self.homography.image_to_floor(center)[0]
                setattr(obs, attr, correct_height(p, self.height_mm, self.camera_mm))
        if obs.neck_mm is not None and obs.tail_mm is not None:
            if np.linalg.norm(obs.neck_mm - obs.tail_mm) > self.max_gap_mm:
                obs.tail_mm = None           # どちらかが誤検出。位置に使う首を残し、向きは前回値を保持
                obs.rejected_gap = True
        return obs

    def update(self, frame: np.ndarray, t: float, j8_deg: float) -> tuple[SnakePose | None, MarkerObservation]:
        """1フレーム処理して、推定した位置姿勢を返す。"""
        obs = self.observe(frame)
        pose = self.tracker.update(t, obs.neck_mm, obs.tail_mm, j8_deg)
        return pose, obs
