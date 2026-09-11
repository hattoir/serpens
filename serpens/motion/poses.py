"""姿勢プリセット。

関節の役割:
  J1〜J6 = 胴体の水平ヨー / J7 = 首ピッチ（持ち上げ・うなずき）
  J8 = 頭ヨー（左右を見る） / J9 = 頭ロール（首かしげ）

J7 を 90° 近くまで倒すと J8 のヨー軸が世界座標で roll になってしまうため、
人を見るとき（head_look）の J7 は neck.look_min_deg〜look_max_deg に制限する。
neck.full_rear_min_deg〜max_deg は「真上から見下ろすフル鎌首」演出専用。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from serpens.motion.kinematics import Chain, forward

Pose = dict[str, float]

NECK, HEAD_YAW, HEAD_ROLL = "J7", "J8", "J9"


@dataclass(frozen=True)
class PoseCommand:
    """姿勢＋トルク上限（脱力など、角度以外も変えるとき）。"""

    angles: Pose
    torque_ratio: float | None = None
    duration_s: float | None = None


class Poses:
    """config の poses / neck からプリセット姿勢を作る。"""

    def __init__(self, cfg: dict[str, Any]) -> None:
        self._p = cfg["poses"]
        self._neck = cfg["neck"]
        self._limits = {j["name"]: (float(j["min_deg"]), float(j["max_deg"])) for j in cfg["joints"]}
        self._chain = Chain.from_cfg(cfg)

    def _clamp(self, name: str, deg: float) -> float:
        lo, hi = self._limits[name]
        return min(max(deg, lo), hi)

    def _from_cfg(self, d: dict[str, Any]) -> Pose:
        return {k: self._clamp(k, float(v)) for k, v in d.items()}

    # ---- プリセット -----------------------------------------------------------
    def home(self) -> Pose:
        """まっすぐ・頭は床。"""
        return self._from_cfg(self._p["home"])

    def coil(self) -> Pose:
        """とぐろ（J1〜J6 で平面の渦）。"""
        return self._from_cfg(self._p["coil"])

    def rear_up(self, angle_deg: float | None = None) -> Pose:
        """鎌首。J7 = angle_deg、胴体は横倒れしにくい S 字。

        J7 は 0〜full_rear_max_deg にクランプする。人を見る用途では head_look の neck_deg を使うこと。
        """
        r = self._p["rear_up"]
        a = float(r["default_angle_deg"]) if angle_deg is None else float(angle_deg)
        a = min(max(a, 0.0), float(self._neck["full_rear_max_deg"]))
        pose = self._from_cfg(r["body"])
        pose[NECK] = self._clamp(NECK, a)
        return pose

    def full_rear_up(self) -> Pose:
        """真上から見下ろすフル鎌首（演出専用）。"""
        return self.rear_up(float(self._neck["full_rear_min_deg"]))

    def head_look(self, yaw_deg: float, roll_deg: float = 0.0, neck_deg: float | None = None) -> Pose:
        """頭を向ける。neck_deg を与えたときは「人を見る」範囲にクランプする。"""
        pose: Pose = {HEAD_YAW: self._clamp(HEAD_YAW, yaw_deg), HEAD_ROLL: self._clamp(HEAD_ROLL, roll_deg)}
        if neck_deg is not None:
            pose[NECK] = self.clamp_look_neck(neck_deg)
        return pose

    def clamp_look_neck(self, neck_deg: float) -> float:
        """人を見るときの J7 の範囲に収める。"""
        lo, hi = float(self._neck["look_min_deg"]), float(self._neck["look_max_deg"])
        return min(max(neck_deg, lo), hi)

    def relax(self, torque_ratio: float | None = None) -> PoseCommand:
        """脱力: 首を下げ、トルクを落とす（タッチされたとき）。"""
        r = self._p["relax"]
        ratio = float(r["torque_ratio"]) if torque_ratio is None else float(torque_ratio)
        pose = {NECK: self._clamp(NECK, float(r["neck_deg"])),
                HEAD_ROLL: self._clamp(HEAD_ROLL, float(r["head_roll_deg"]))}
        return PoseCommand(pose, ratio, float(r["duration_s"]))

    # ---- 幾何 -----------------------------------------------------------------
    def points(self, pose: Pose) -> np.ndarray:
        """姿勢の各点（尾端, J1…J9, 頭先端）の3次元位置 [mm]。"""
        pts, _ = forward(self._chain, pose)
        return pts

    def head_height_mm(self, pose: Pose) -> float:
        """頭先端の床からの高さ [mm]（尾端の高さを 0 とする）。"""
        return float(self.points(pose)[-1][2])
