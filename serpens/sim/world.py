"""2D シミュレータ: 関節角の列から、ヘビの床上の位置姿勢を求める。

物理エンジンは使わず、次の運動学だけで推進を再現する:
  1. 受動輪のあるリンクは「自分の接線方向にしか動けない」（横滑りしない＝非ホロノミック拘束）
  2. 全リンクの拘束をまとめて最小二乗で満たす、胴体全体の剛体速度 (vx, vy, ω) を毎ステップ解く
  3. その速度で全体を並進・回転させる
胴体の形の変化（関節角の変化）に対して横滑りしないように全体が動く結果、
波が頭→尾へ伝わるとヘビは前へ進む。

座標:
  ボディ座標 … 尾端 = 原点、尾側リンクの向き = +x（kinematics.forward と同じ）
  世界座標   … マット左手前が原点、単位 mm。ボディ座標の原点位置 (X, Y) と向き Θ で置く
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from serpens.motion.kinematics import Chain, forward
from serpens.motion.poses import Pose

NECK_INDEX = 7    # points[7] = J7（首。ArUco マーカを貼る位置）
TAIL_INDEX = 0    # points[0] = 尾端


@dataclass
class BodyPose:
    """ボディ座標原点（尾端）の世界座標での位置と向き。"""

    x: float
    y: float
    theta: float   # [rad]


def _rot2(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


class World:
    """展示台の上のヘビ1匹。"""

    def __init__(self, cfg: dict[str, Any], pose: BodyPose | None = None) -> None:
        s = cfg["sim"]
        self.chain = Chain.from_cfg(cfg)
        self.mat_w = float(cfg["mat"]["width_mm"])
        self.mat_d = float(cfg["mat"]["depth_mm"])
        self.radius = float(cfg["body"]["diameter_mm"]) / 2.0
        self.wheel_links = [int(k) for k in s["wheel_links"]]
        self.contact_h = float(s["contact_height_mm"])
        self.rcond = float(s["lstsq_rcond"])
        self.tan_ratio = float(s["tangential_drag_ratio"])
        self.pose = pose or BodyPose(float(s["start_tail_x_mm"]), float(s["start_tail_y_mm"]),
                                     math.radians(float(s["start_theta_deg"])))
        self.angles: Pose = {}
        self._pts_body = forward(self.chain, self.angles)[0]
        self.clamped = False       # 直前のステップでマット端に当たったか
        self.travel_mm = 0.0       # 重心の総移動距離

    # ---- 更新 -----------------------------------------------------------------
    def step(self, angles: Pose, dt: float) -> None:
        """関節角を angles に変えたときの、全体の動きを dt 秒ぶん積分する。"""
        new_pts = forward(self.chain, angles)[0]
        c_before = self.centroid()
        vx, vy, om = self._solve_body_velocity(self._pts_body, new_pts, dt)
        # ボディ座標で求めた速度を世界座標へ（形の変化の中点で回す）
        th_mid = self.pose.theta + 0.5 * om * dt
        v_world = _rot2(th_mid) @ np.array([vx, vy])
        self.pose = BodyPose(self.pose.x + v_world[0] * dt, self.pose.y + v_world[1] * dt,
                             self.pose.theta + om * dt)
        self.angles = dict(angles)
        self._pts_body = new_pts
        self._clamp_to_mat()
        self.travel_mm += float(np.linalg.norm(self.centroid() - c_before))

    def _solve_body_velocity(self, before: np.ndarray, after: np.ndarray, dt: float) -> tuple[float, float, float]:
        """横滑りしない条件 n_k·(V + ω×c_k + ċ_k) = 0 を最小二乗で解く。

        tangential_drag_ratio > 0 のときは、接線方向の速度 u_k·v にも小さな重みで
        「動きにくさ」を加える（転がり抵抗。0 なら接線方向は完全に自由）。
        """
        rows, rhs = [], []
        tan_w = math.sqrt(self.tan_ratio)
        for k in self.wheel_links:
            a0, a1 = after[k], after[k + 1]
            mid = 0.5 * (a0 + a1)
            if mid[2] > self.contact_h:
                continue                          # 浮いているリンクは拘束しない
            d = a1[:2] - a0[:2]
            length = float(np.linalg.norm(d))
            if length <= 0.0:
                continue
            u = d / length
            n = np.array([-u[1], u[0]])
            c = mid[:2]
            shape_vel = (mid[:2] - 0.5 * (before[k][:2] + before[k + 1][:2])) / dt
            w = math.sqrt(length)                 # 長いリンクほど車輪の効きが強い（重み）
            for vec, wt in ((n, w), (u, w * tan_w)):
                if wt > 0.0:
                    rows.append(wt * np.array([vec[0], vec[1], -vec[0] * c[1] + vec[1] * c[0]]))
                    rhs.append(-wt * float(vec @ shape_vel))
        if not rows:
            return 0.0, 0.0, 0.0
        sol, *_ = np.linalg.lstsq(np.array(rows), np.array(rhs), rcond=self.rcond)
        return float(sol[0]), float(sol[1]), float(sol[2])

    def _clamp_to_mat(self) -> None:
        """胴体のどこかがマットからはみ出したら、全体を内側へ押し戻す。"""
        pts = self.world_points()[:, :2]
        r = self.radius
        dx = max(0.0, r - pts[:, 0].min()) - max(0.0, pts[:, 0].max() - (self.mat_w - r))
        dy = max(0.0, r - pts[:, 1].min()) - max(0.0, pts[:, 1].max() - (self.mat_d - r))
        self.clamped = dx != 0.0 or dy != 0.0
        if self.clamped:
            self.pose = BodyPose(self.pose.x + dx, self.pose.y + dy, self.pose.theta)

    # ---- 取得 -----------------------------------------------------------------
    def world_points(self) -> np.ndarray:
        """尾端, J1…J9, 頭先端 の世界座標 [mm]。shape (12, 3)"""
        R = _rot2(self.pose.theta)
        xy = self._pts_body[:, :2] @ R.T + np.array([self.pose.x, self.pose.y])
        return np.column_stack([xy, self._pts_body[:, 2]])

    def centroid(self) -> np.ndarray:
        """床に接しているリンク中点の平均（ヘビの「重心」代わり）[mm]。"""
        pts = self.world_points()
        mids = [(pts[k] + pts[k + 1]) / 2 for k in self.wheel_links]
        return np.mean(np.array(mids)[:, :2], axis=0)

    def snake_pose(self) -> tuple[float, float, float]:
        """ヘビの (x, y, θ)。位置 = 首（J7）、向き = 尾端→首（ArUco 2枚で測るものと同じ定義）。"""
        pts = self.world_points()
        neck, tail = pts[NECK_INDEX, :2], pts[TAIL_INDEX, :2]
        d = neck - tail
        return float(neck[0]), float(neck[1]), math.atan2(float(d[1]), float(d[0]))

    def head_tip(self) -> np.ndarray:
        """頭先端の世界座標 [mm]（x, y, z）。"""
        return self.world_points()[-1]
