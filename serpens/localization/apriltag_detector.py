"""頭カメラの画像から AprilTag（tag36h11）を検出し、機体座標の (距離, 方位, タグ面の向き) にする。

依存は opencv-contrib の `cv2.aruco`（DICT_APRILTAG_36h11）だけ。新しい依存は足さない。
カメラは機体基準点から forward_m 前・height_m 上にあり、光軸は首の俯角 pitch_deg だけ下を向く（タグを探すときは
`tag_search_pitch_deg`。首を動かすと変わるので毎回渡す）。内部パラメータ（f_px 等）は **FOV 65° 未確認 → ASSUMED**。

カメラ座標（OpenCV: x 右、y 下、z 前）→ 機体座標（x 前、y 左、z 上）:
    R_bc の列 = カメラ軸の機体座標 = [ (0,-1,0), (-sin p, 0, -cos p), (cos p, 0, -sin p) ]
タグ座標（cv2.aruco: 面の外向き = +z）→ タグの面の向きは R_bc · R_ct[:, 2] の水平成分。
**実カメラでの検出率・精度は未測定（HARDWARE_UNVERIFIED）。テストは合成画像。**
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from serpens.localization.estimator import TagObservation

FAMILIES = {"tag36h11": cv2.aruco.DICT_APRILTAG_36h11, "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
            "tag16h5": cv2.aruco.DICT_APRILTAG_16h5}


def rot_body_from_cam(pitch_deg: float) -> np.ndarray:
    p = math.radians(pitch_deg)
    return np.array([[0.0, -math.sin(p), math.cos(p)],
                     [-1.0, 0.0, 0.0],
                     [0.0, -math.cos(p), -math.sin(p)]])


@dataclass
class HeadCamera:
    """タグ検出に使う頭カメラの内部・外部パラメータ（config localization.camera）。"""

    f_px: float
    cx: float
    cy: float
    width_px: int
    height_px: int
    height_m: float
    forward_m: float

    @classmethod
    def from_cfg(cls, cfg: dict[str, Any]) -> "HeadCamera":
        c = cfg["localization"]["camera"]
        w, h = int(c["width_px"]), int(c["height_px"])
        return cls(float(c["f_px"]), w / 2.0, h / 2.0, w, h, float(c["height_m"]), float(c["forward_m"]))

    @property
    def K(self) -> np.ndarray:
        return np.array([[self.f_px, 0.0, self.cx], [0.0, self.f_px, self.cy], [0.0, 0.0, 1.0]])

    def cam_from_body(self, p_body: np.ndarray, pitch_deg: float) -> np.ndarray:
        """機体座標の点 → カメラ座標（合成画像のテスト用）。"""
        R = rot_body_from_cam(pitch_deg)
        return R.T @ (p_body - np.array([self.forward_m, 0.0, self.height_m]))

    def body_from_cam(self, p_cam: np.ndarray, pitch_deg: float) -> np.ndarray:
        return rot_body_from_cam(pitch_deg) @ p_cam + np.array([self.forward_m, 0.0, self.height_m])


class AprilTagDetector:
    def __init__(self, cfg: dict[str, Any], tag_size_m: float, family: str = "tag36h11") -> None:
        self.cam = HeadCamera.from_cfg(cfg)
        self.size = float(tag_size_m)
        self.dictionary = cv2.aruco.getPredefinedDictionary(FAMILIES[family])
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)
        h = self.size / 2
        self.obj = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float64)   # 左上から時計回り

    def detect(self, gray: np.ndarray, t_s: float, pitch_deg: float) -> list[TagObservation]:
        corners, ids, _rej = self.detector.detectMarkers(gray)
        out: list[TagObservation] = []
        if ids is None:
            return out
        for c, tid in zip(corners, ids.ravel()):
            ok, rvec, tvec = cv2.solvePnP(self.obj, c.reshape(4, 2).astype(np.float64), self.cam.K, None,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            R_ct, _ = cv2.Rodrigues(rvec)
            p_b = self.cam.body_from_cam(tvec.ravel(), pitch_deg)
            n_b = rot_body_from_cam(pitch_deg) @ R_ct[:, 2]                 # タグの面の外向き（機体座標）
            out.append(TagObservation(int(tid), float(math.hypot(p_b[0], p_b[1])), float(math.atan2(p_b[1], p_b[0])),
                                      float(math.atan2(n_b[1], n_b[0])), t_s, simulated=False))
        return out

    def render_synthetic(self, tag_id: int, p_body: np.ndarray, yaw_body: float, pitch_deg: float,
                         background: int = 200) -> np.ndarray:
        """**テスト用**: 機体座標に置いたタグ（中心 p_body、面の向き yaw_body、垂直に立てる）をこのカメラで見た合成画像。"""
        n = np.array([math.cos(yaw_body), math.sin(yaw_body), 0.0])          # 面の外向き = タグ +z
        right = np.array([-math.sin(yaw_body), math.cos(yaw_body), 0.0])      # up × n = 見る側から見て右 = タグ +x
        up = np.array([0.0, 0.0, 1.0])                                        # タグ +y
        h = self.size / 2
        corners_b = np.array([p_body + h * (sx * right + sy * up) for sx, sy in ((-1, 1), (1, 1), (1, -1), (-1, -1))])
        corners_c = np.array([self.cam.cam_from_body(pb, pitch_deg) for pb in corners_b])
        img_pts = (corners_c[:, :2] / corners_c[:, 2:3]) * self.cam.f_px + np.array([self.cam.cx, self.cam.cy])
        px = 200
        marker = cv2.aruco.generateImageMarker(self.dictionary, int(tag_id), px, borderBits=1)
        pad = px // 8
        marker = cv2.copyMakeBorder(marker, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)   # 白い余白
        src = np.array([[pad, pad], [pad + px, pad], [pad + px, pad + px], [pad, pad + px]], dtype=np.float32)
        H, _ = cv2.findHomography(src, img_pts.astype(np.float32))
        canvas = np.full((self.cam.height_px, self.cam.width_px), background, np.uint8)
        warped = cv2.warpPerspective(marker, H, (self.cam.width_px, self.cam.height_px), borderValue=0)
        mask = cv2.warpPerspective(np.full_like(marker, 255), H, (self.cam.width_px, self.cam.height_px), borderValue=0)
        canvas[mask > 127] = warped[mask > 127]
        return canvas
