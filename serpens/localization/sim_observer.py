"""合成のタグ観測（**模擬。実カメラではない**）: 真の姿勢と地図から、頭カメラに見えるはずのタグを雑音付きで返す。

見える条件: 距離が [min, max]、方位が視野の半分以内、タグの面を max_view_angle 以内の角度で見ている。
blind=True（家具の下）なら何も見えない。sim_miss_prob で見えるはずのタグをたまに見逃す。
歩容オドメトリの合成（`GaitOdometrySim`）: 1 周期の前進量 × 位相の進み。滑りの真値は
`slip_frac`（推定器には知らせない。推定器の割合が正直かを試すため）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from serpens.localization.estimator import TagObservation, wrap
from serpens.localization.tag_map import TagMap


@dataclass
class SimTagObserver:
    cfg: dict[str, Any]
    tag_map: TagMap
    rng: np.random.Generator

    def __post_init__(self) -> None:
        at = self.cfg["localization"]["apriltag"]
        self.min_r, self.max_r = float(at["min_range_m"]), float(at["max_range_m"])
        self.half_fov = math.radians(float(self.cfg["localization"]["camera"]["fov_deg"])) / 2
        self.max_view = math.radians(float(at["max_view_angle_deg"]))
        self.s_r, self.s_b, self.s_y = float(at["sigma_range_frac"]), float(at["sigma_bearing_rad"]), float(at["sigma_yaw_rad"])
        self.miss = float(at["sim_miss_prob"])

    def visible(self, x: float, y: float, yaw: float) -> list[tuple[int, float, float, float]]:
        """雑音なしの (id, range, bearing, rel_yaw)。"""
        out = []
        for tag in self.tag_map.tags.values():
            dx, dy = tag.x_m - x, tag.y_m - y
            r = math.hypot(dx, dy)
            if not (self.min_r <= r <= self.max_r):
                continue
            bearing = wrap(math.atan2(dy, dx) - yaw)
            if abs(bearing) > self.half_fov:
                continue
            fx, fy = tag.facing()
            view = math.acos(max(-1.0, min(1.0, (-dx * fx - dy * fy) / r)))     # タグの面 → 機体 の角度
            if view > self.max_view:
                continue
            out.append((tag.id, r, bearing, wrap(tag.yaw_rad - yaw)))
        return out

    def observe(self, t: float, x: float, y: float, yaw: float, blind: bool = False) -> list[TagObservation]:
        if blind:
            return []
        obs = []
        for tid, r, b, ry in self.visible(x, y, yaw):
            if self.rng.random() < self.miss:
                continue
            obs.append(TagObservation(tid, r + self.rng.normal(0, self.s_r * r), b + self.rng.normal(0, self.s_b),
                                      wrap(ry + self.rng.normal(0, self.s_y)), t, simulated=True))
        return obs


@dataclass
class GaitOdometrySim:
    """歩容の位相から前進量を作る（推定器への入力）と、真の移動（滑り込み）を別に返す。"""

    advance_per_cycle_m: float
    slip_frac: float                       # 真の前進 = 指令 × (1 − slip)。推定器は知らない
    yaw_sigma_true_rad: float = 0.0        # 真の回転に乗る雑音（推定器は知らない）

    def step(self, dphase_rad: float, dyaw_cmd: float, rng: np.random.Generator) -> tuple[float, float, float, float]:
        """(推定器へ渡す ds, 推定器へ渡す dyaw, 真の ds, 真の dyaw)。"""
        ds = self.advance_per_cycle_m * dphase_rad / (2 * math.pi)
        true_ds = ds * (1.0 - self.slip_frac)
        true_dyaw = dyaw_cmd + (rng.normal(0, self.yaw_sigma_true_rad) if self.yaw_sigma_true_rad > 0 else 0.0)
        return ds, dyaw_cmd, true_ds, true_dyaw
