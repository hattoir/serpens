"""仮想カメラ: シミュレータの世界を斜め上から撮った画像を作る（実機・カメラなしで知覚を試す）。

- マット・ヘビの胴体・ArUco マーカ（本物のマーカ画像を胴体上面に貼る）・人（シルエット）を描く
- マーカは高さ aruco.marker_height_mm にあるものとして視差込みで描くので、
  ArucoLocator の視差補正も試せる
- SimPersonDetector は描いた人の bbox をそのまま「検出結果」として返す（YOLO の代わり）
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from serpens.perception.homography import FloorHomography, apparent_floor_point
from serpens.perception.person_detector import PersonDetection, detections_from_boxes
from serpens.sim.world import World

BG_BGR = (60, 60, 60)
MAT_BGR = (170, 205, 225)
GRID_BGR = (140, 170, 190)
BODY_BGR = (40, 110, 40)
PERSON_BGR = (90, 60, 160)
GRID_STEP_MM = 200.0
MARKER_CELLS = 6          # 4x4 + 黒枠 1 セルずつ
WHITE = 255


@dataclass(frozen=True)
class SimPerson:
    """シミュレータ上の人（足元の床座標）。"""

    x_mm: float
    y_mm: float


class VirtualCamera:
    """シミュレータの世界 → 画像。"""

    def __init__(self, cfg: dict[str, Any]) -> None:
        v, m = cfg["virtual_camera"], cfg["markers"]
        self.w, self.h = int(v["width_px"]), int(v["height_px"])
        self.homography = FloorHomography.from_clicks(cfg, v["mat_corners_px"], (self.w, self.h))
        self.camera_mm = [float(x) for x in v["position_mm"]]
        self.person_h = float(v["person_height_mm"])
        self.person_w = float(v["person_width_mm"])
        self.marker_mm = float(m["size_mm"])
        self.margin_mm = float(cfg["aruco"]["print_margin_mm"])
        self.marker_h = float(cfg["aruco"]["marker_height_mm"])
        self.body_d = float(cfg["body"]["diameter_mm"])
        self.mat = np.array([[0, 0], [cfg["mat"]["width_mm"], 0],
                             [cfg["mat"]["width_mm"], cfg["mat"]["depth_mm"]], [0, cfg["mat"]["depth_mm"]]], float)
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, m["dictionary"]))
        px = MARKER_CELLS * 20
        self._tex = {k: self._marker_texture(d, int(m[k]), px) for k in ("tail_id", "neck_id")}
        self.person_boxes: list[tuple[float, float, float, float]] = []

    def _marker_texture(self, d: Any, marker_id: int, px: int) -> np.ndarray:
        """マーカ＋白い余白の画像。"""
        img = cv2.aruco.generateImageMarker(d, marker_id, px)
        pad = int(round(px * self.margin_mm / self.marker_mm))
        return cv2.copyMakeBorder(img, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=WHITE)

    def _to_px(self, floor_xy: np.ndarray, height_mm: float = 0.0) -> np.ndarray:
        p = np.asarray(floor_xy, float).reshape(-1, 2)
        if height_mm:
            p = np.array([apparent_floor_point(q, height_mm, self.camera_mm) for q in p])
        return self.homography.floor_to_image(p)

    def render(self, world: World, people: list[SimPerson]) -> np.ndarray:
        """1枚描く。self.person_boxes に描いた人の bbox を残す。"""
        img = np.full((self.h, self.w, 3), BG_BGR, np.uint8)
        cv2.fillPoly(img, [self._to_px(self.mat).astype(np.int32)], MAT_BGR)
        for g in np.arange(GRID_STEP_MM, self.mat[2, 0], GRID_STEP_MM):
            a, b = self._to_px(np.array([[g, 0], [g, self.mat[2, 1]]]))
            cv2.line(img, tuple(a.astype(int)), tuple(b.astype(int)), GRID_BGR, 1)
        for g in np.arange(GRID_STEP_MM, self.mat[2, 1], GRID_STEP_MM):
            a, b = self._to_px(np.array([[0, g], [self.mat[2, 0], g]]))
            cv2.line(img, tuple(a.astype(int)), tuple(b.astype(int)), GRID_BGR, 1)
        self._draw_body(img, world)
        for key, which in (("tail_id", "tail"), ("neck_id", "neck")):
            self._draw_marker(img, world, which, self._tex[key])
        self.person_boxes = [self._draw_person(img, p) for p in sorted(people, key=lambda p: -p.y_mm)]
        return img

    def _draw_body(self, img: np.ndarray, world: World) -> None:
        pts = world.world_points()
        px = self._to_px(pts[:, :2], self.body_d / 2)
        scale = self.homography.px_per_mm_at(pts[len(pts) // 2, :2])
        cv2.polylines(img, [px.astype(np.int32)], False, BODY_BGR, max(int(self.body_d * scale), 2), cv2.LINE_AA)

    def _draw_marker(self, img: np.ndarray, world: World, which: str, tex: np.ndarray) -> None:
        """胴体上面にマーカを貼る（胴体の接線方向に向ける）。"""
        self._draw_marker_at(img, world.marker_xy(which), world.marker_tangent(which), tex)

    def draw_extra_marker(self, img: np.ndarray, which: str, floor_xy: np.ndarray, tangent: np.ndarray) -> None:
        """**故障注入用**: 胴体と無関係な場所に同じ ID のマーカを描く（誤検出の再現）。"""
        key = "neck_id" if which == "neck" else "tail_id"
        self._draw_marker_at(img, np.asarray(floor_xy, float), np.asarray(tangent, float), self._tex[key])

    def occlude(self, img: np.ndarray, floor_xy: np.ndarray, radius_mm: float) -> None:
        """**故障注入用**: 床の一点の上（マーカの高さ）を塗りつぶす（手や物で隠れた状態）。"""
        c = self._to_px(np.asarray(floor_xy, float), self.marker_h)[0]
        r = radius_mm * self.homography.px_per_mm_at(np.asarray(floor_xy, float))
        cv2.circle(img, (int(c[0]), int(c[1])), max(int(r), 1), PERSON_BGR, -1)

    def _draw_marker_at(self, img: np.ndarray, c: np.ndarray, t: np.ndarray, tex: np.ndarray) -> None:
        n = np.array([-t[1], t[0]])
        half = (self.marker_mm / 2 + self.margin_mm)
        corners = np.array([c - t * half + n * half, c + t * half + n * half,
                            c + t * half - n * half, c - t * half - n * half])
        dst = self._to_px(corners, self.marker_h).astype(np.float32)
        s = tex.shape[0]
        src = np.array([[0, 0], [s, 0], [s, s], [0, s]], np.float32)
        M = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(tex, M, (self.w, self.h), flags=cv2.INTER_LINEAR, borderValue=0)
        mask = cv2.warpPerspective(np.full_like(tex, WHITE), M, (self.w, self.h), flags=cv2.INTER_NEAREST)
        img[mask > 0] = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)[mask > 0]

    def _draw_person(self, img: np.ndarray, p: SimPerson) -> tuple[float, float, float, float]:
        foot = self._to_px(np.array([p.x_mm, p.y_mm]))[0]
        s = self.homography.px_per_mm_at(np.array([p.x_mm, p.y_mm]))
        w, h = self.person_w * s, self.person_h * s
        x1, y1, x2, y2 = foot[0] - w / 2, foot[1] - h, foot[0] + w / 2, foot[1]
        head_r = int(w * 0.28)
        cv2.rectangle(img, (int(x1 + w * 0.15), int(y1 + 2 * head_r)), (int(x2 - w * 0.15), int(y2)), PERSON_BGR, -1)
        cv2.circle(img, (int(foot[0]), int(y1 + head_r)), head_r, PERSON_BGR, -1)
        return x1, y1, x2, y2


class SimPersonDetector:
    """仮想カメラで描いた人の bbox を、そのまま検出結果として返す（YOLO の代わり）。"""

    def __init__(self, cam: VirtualCamera, homography: FloorHomography) -> None:
        self.cam = cam
        self.homography = homography

    def detect(self, _frame: np.ndarray) -> list[PersonDetection]:
        boxes = [b for b in self.cam.person_boxes if 0 <= (b[0] + b[2]) / 2 < self.cam.w and 0 <= b[3] < self.cam.h]
        if not boxes:
            return []
        return detections_from_boxes(np.array(boxes), np.ones(len(boxes)), self.homography)
