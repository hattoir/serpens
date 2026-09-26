"""模擬の床（**SIMULATED**）: 座標系 home に置いた小物と、機体の世界座標（mm、マット）との対応。

  HomeFrame      … 世界（mm、マット。原点 = マット左手前）↔ home（m。マット中心を localization.sim.mat_center_home_m に置く）
  FloorScene     … home に置いた物（種類・直径・高さ・反射率・鏡面）と汚れ、タグが見えない領域（家具の下）
  capture_synthetic … 頭の位置と向きから、頭カメラの床座標（x 右、y 前、mm）に物を移して 5 枚を合成する
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from serpens.floorwatch.synthetic import Disc, Renderer, Scene, Stain


@dataclass(frozen=True)
class FloorObject:
    kind: str
    x_m: float
    y_m: float
    diameter_mm: float
    height_mm: float
    albedo: float = 0.8
    specular: bool = False


@dataclass(frozen=True)
class FloorStain:
    x_m: float
    y_m: float
    diameter_mm: float = 18.0
    albedo: float = 0.3


@dataclass
class FloorScene:
    objects: list[FloorObject] = field(default_factory=list)
    stains: list[FloorStain] = field(default_factory=list)
    blind_boxes: list[tuple[float, float, float, float]] = field(default_factory=list)   # (x0, x1, y0, y1) タグが見えない
    seed: int = 0

    def blind(self, x_m: float, y_m: float) -> bool:
        return any(x0 <= x_m <= x1 and y0 <= y_m <= y1 for x0, x1, y0, y1 in self.blind_boxes)


class HomeFrame:
    """世界（mm）↔ home（m）。回転は無し（マットの x = 部屋の x）。"""

    def __init__(self, cfg: dict[str, Any], mat_center_mm: np.ndarray) -> None:
        self.center_mm = np.asarray(mat_center_mm, float)
        self.center_home = np.array([float(v) for v in cfg["localization"]["sim"]["mat_center_home_m"]])

    def to_home(self, xy_mm: np.ndarray) -> np.ndarray:
        return (np.asarray(xy_mm, float) - self.center_mm) / 1000.0 + self.center_home

    def to_world(self, xy_m: np.ndarray) -> np.ndarray:
        return (np.asarray(xy_m, float) - self.center_home) * 1000.0 + self.center_mm


def objects_in_camera(scene: FloorScene, head_xy_m: np.ndarray, head_yaw_rad: float,
                      reach_mm: float) -> tuple[list[Disc], list[Stain]]:
    """頭の位置・向きから見た床座標（x 右、y 前、mm）。reach_mm 以内の物だけ。"""
    c, s = math.cos(head_yaw_rad), math.sin(head_yaw_rad)

    def rel(x_m: float, y_m: float) -> tuple[float, float]:
        dx, dy = (x_m - head_xy_m[0]) * 1000.0, (y_m - head_xy_m[1]) * 1000.0
        fwd, left = dx * c + dy * s, -dx * s + dy * c
        return -left, fwd                                   # カメラ x = 右
    discs = [Disc(*rel(o.x_m, o.y_m), o.diameter_mm, o.height_mm, o.albedo, o.specular, o.kind)
             for o in scene.objects if math.hypot(o.x_m - head_xy_m[0], o.y_m - head_xy_m[1]) * 1000.0 <= reach_mm]
    stains = [Stain(*rel(st.x_m, st.y_m), st.diameter_mm, st.albedo)
              for st in scene.stains if math.hypot(st.x_m - head_xy_m[0], st.y_m - head_xy_m[1]) * 1000.0 <= reach_mm]
    return discs, stains


def capture_synthetic(renderer: Renderer, scene: FloorScene, head_xy_m: np.ndarray, head_yaw_rad: float,
                      reach_mm: float) -> dict[str, np.ndarray]:
    discs, stains = objects_in_camera(scene, head_xy_m, head_yaw_rad, reach_mm)
    return renderer.render(Scene(discs, stains, seed=scene.seed))


def camera_to_home(floor_xy_mm: tuple[float, float], head_xy_m: np.ndarray, head_yaw_rad: float) -> np.ndarray:
    """検出した候補の床座標（x 右、y 前、mm）→ home（m）。"""
    right, fwd = float(floor_xy_mm[0]), float(floor_xy_mm[1])
    c, s = math.cos(head_yaw_rad), math.sin(head_yaw_rad)
    dx, dy = fwd * c + right * s, fwd * s - right * c
    return np.asarray(head_xy_m, float) + np.array([dx, dy]) / 1000.0
