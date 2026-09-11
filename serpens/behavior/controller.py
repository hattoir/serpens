"""移動制御: 目標点 → 歩容パラメータ（旋回オフセット γ0 と時間周波数）。

  - 向き: γ0 = heading_gain × (目標の方位 − θ_body[1周期の移動平均])、±max_turn_deg にクランプ
  - 速さ: 周波数 f = 速さ / advance_per_cycle_mm。プリセットの周波数を上限にする
          （速さのために周期を短くはしない。遅くするときだけ周期を延ばす）
  - 人の near_person_mm 以内では最高速度 near_speed_limit_mm_s
  - 頭先端から人まで stop_distance_mm で必ず停止。歩容は止めてから blend_s かけて振幅が消えるので、
    そのぶんの惰性（速さ × blend_s × coast_ratio）だけ早めに止める
  - 首の先読み点がマット端（mat_margin_mm）を出そうなら、その場で前進をやめ、
    後退しながら中央へ向き直る（後退では γ の符号と回る向きが逆になる）
  - ただし edge_is_goal=True（マットの外にいる人へ近づく）で、目標の方を向いたまま端に来たときは
    「これ以上近づけない」として止まる（blocked=True）
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from serpens.motion.gait import GaitParams
from serpens.perception.snake_pose import SnakePose, wrap_pi


@dataclass(frozen=True)
class DriveCommand:
    """移動の指令。moving=False なら歩容を止める。"""

    moving: bool
    params: GaitParams | None = None
    gamma0_deg: float = 0.0
    speed_mm_s: float = 0.0
    reason: str = ""
    blocked: bool = False       # マット端に来て、目標にこれ以上近づけない


class Controller:
    """目標点へ向かう移動の指令を作る。"""

    def __init__(self, cfg: dict[str, Any]) -> None:
        c = cfg["behavior"]["controller"]
        self.c = c
        self.base = GaitParams.from_cfg(cfg["gait"]["presets"][c["gait"]])
        self.w, self.d = float(cfg["mat"]["width_mm"]), float(cfg["mat"]["depth_mm"])
        self.recovering = False
        self.coast_s = float(cfg["gait"]["blend_s"]) * float(c["coast_ratio"])

    # ---- 補助 -----------------------------------------------------------------
    def head_distance(self, pose: SnakePose, person_xy: np.ndarray) -> float:
        """頭先端（首から head_reach_mm 先と近似）から人までの距離 [mm]。"""
        return float(np.linalg.norm(np.asarray(person_xy) - [pose.x, pose.y])) - float(self.c["head_reach_mm"])

    def speed_limit(self, pose: SnakePose, person_xy: np.ndarray | None, wanted: float) -> float:
        """人の近くでは最高速度を制限する。"""
        if person_xy is not None and np.linalg.norm(np.asarray(person_xy) - [pose.x, pose.y]) <= float(self.c["near_person_mm"]):
            return min(wanted, float(self.c["near_speed_limit_mm_s"]))
        return wanted

    def _params(self, speed: float, backward: bool = False) -> GaitParams:
        f_max = abs(self.base.temporal_freq_hz)
        f = min(max(speed / float(self.c["advance_per_cycle_mm"]), float(self.c["min_temporal_freq_hz"])), f_max)
        return replace(self.base, temporal_freq_hz=-f if backward else f, turn_bias_deg=0.0)

    def _heading_error(self, pose: SnakePose, target: np.ndarray) -> float:
        b = math.atan2(target[1] - pose.y, target[0] - pose.x)
        return math.degrees(wrap_pi(b - pose.theta_body))

    def near_edge(self, pose: SnakePose) -> bool:
        """首の先読み点がマットの内側（margin を除く）から出るか。"""
        la, m = float(self.c["lookahead_mm"]), float(self.c["mat_margin_mm"])
        x = pose.x + la * math.cos(pose.theta_body)
        y = pose.y + la * math.sin(pose.theta_body)
        return not (m <= x <= self.w - m and m <= y <= self.d - m)

    def center(self) -> np.ndarray:
        return np.array([self.w / 2, self.d / 2])

    # ---- 指令 -----------------------------------------------------------------
    def drive_to(self, pose: SnakePose, target_xy: np.ndarray, speed_mm_s: float,
                 person_xy: np.ndarray | None = None, stop_at_person: bool = False,
                 edge_is_goal: bool = False) -> DriveCommand:
        """target_xy へ向かう指令。"""
        c = self.c
        speed = self.speed_limit(pose, person_xy, speed_mm_s)
        if person_xy is not None and stop_at_person and                 self.head_distance(pose, person_xy) <= float(c["stop_distance_mm"]) + speed * self.coast_s:
            return DriveCommand(False, reason=f"停止: 人の {c['stop_distance_mm']:.0f}mm 手前", blocked=True)
        err = self._heading_error(pose, np.asarray(target_xy, float))
        ok = float(c["recover_heading_ok_deg"])
        if edge_is_goal and not self.recovering and self.near_edge(pose) and abs(err) <= ok:
            return DriveCommand(False, reason="マット端に着いた（これ以上近づけない）", blocked=True)
        err_center = self._heading_error(pose, self.center())
        if self.recovering or self.near_edge(pose):
            if abs(err_center) <= ok:
                self.recovering = False
            else:
                self.recovering = True
                g = -math.copysign(float(c["max_turn_deg"]), err_center)     # 後退なので逆符号
                return DriveCommand(True, self._params(speed, backward=True), g, speed, "マット端: 後退しながら中央へ向き直る")
        g = max(-float(c["max_turn_deg"]), min(float(c["max_turn_deg"]), float(c["heading_gain"]) * err))
        return DriveCommand(True, self._params(speed), g, speed, f"目標へ（向きの誤差 {err:+.0f}°）")

    def arrived(self, pose: SnakePose, target_xy: np.ndarray) -> bool:
        return float(np.linalg.norm(np.asarray(target_xy) - [pose.x, pose.y])) <= float(self.c["arrive_mm"])

    def room_toward(self, pose: SnakePose, target_xy: np.ndarray) -> float:
        """target の方向へ、首の先読み点がマット端（margin）に届くまで進める距離 [mm]。"""
        start = np.array([pose.x, pose.y])
        d = np.asarray(target_xy, float) - start
        n = float(np.linalg.norm(d))
        if n == 0.0:
            return 0.0
        u = d / n
        la, m = float(self.c["lookahead_mm"]), float(self.c["mat_margin_mm"])
        lim = np.inf
        for k, (lo, hi) in enumerate(((m, self.w - m), (m, self.d - m))):
            if u[k] > 1e-9:
                lim = min(lim, (hi - start[k]) / u[k])
            elif u[k] < -1e-9:
                lim = min(lim, (lo - start[k]) / u[k])
        return max(float(lim) - la, 0.0)

    def retreat_point(self, pose: SnakePose, person_xy: np.ndarray) -> np.ndarray:
        """人から離れる方向の点（マットの内側にクランプ）。"""
        away = np.array([pose.x, pose.y]) - np.asarray(person_xy, float)
        n = float(np.linalg.norm(away))
        away = away / n if n > 0 else np.array([1.0, 0.0])
        p = np.array([pose.x, pose.y]) + away * float(self.c["retreat_distance_mm"])
        m = float(self.c["mat_margin_mm"])
        return np.clip(p, [m, m], [self.w - m, self.d - m])
