"""仮想の来場者（Behavior / Follow の検証用ターゲット）。

**人物 AI ではない。** 決まった軌跡で動くだけの的で、行動エンジンが
「気づく → 見る → 近づく → 観察する → ついていく」を出せるかを確かめるために置く。

軌跡:
  STATIONARY … 立ち止まっている
  WALKING    … 展示台の前を横切らずに近づいてくる
  LEAVING    … 近くから離れていく
  CROSSING   … 左から右へ通り過ぎる

座標は `config/robot.yaml` の `mat`（展示台）基準。来場者は y<0 側から来る。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from serpens.world_state import Pose2D, PoseSource

TRAJECTORIES = ("STATIONARY", "WALKING", "LEAVING", "CROSSING")


@dataclass(frozen=True)
class VirtualPerson:
    """1人ぶんの軌跡。`at(t)` で位置を返す。"""

    kind: str
    start_xy: tuple[float, float]
    speed_mm_s: float = 500.0
    heading_rad: float = 0.0
    appear_s: float = 0.0                 # この時刻から現れる
    disappear_s: float = math.inf         # この時刻に消える
    stop_at_y_mm: float | None = None     # ここまで来たら立ち止まる（展示では台の手前で止まる）
    stop_at_x_mm: float | None = None     # CROSSING: ここまで横へ動いたら立ち止まる（決定的瞬間の来場者）

    def visible(self, t: float) -> bool:
        return self.appear_s <= t < self.disappear_s

    def at(self, t: float) -> tuple[float, float]:
        """時刻 t の床座標 [mm]。"""
        dt = max(t - self.appear_s, 0.0)
        x0, y0 = self.start_xy
        if self.kind == "STATIONARY":
            return x0, y0
        d = self.speed_mm_s * dt
        if self.kind == "WALKING":         # 台へ向かって近づき、手前で立ち止まる
            y = y0 + d
            if self.stop_at_y_mm is not None:
                y = min(y, self.stop_at_y_mm)
            return x0, y
        if self.kind == "LEAVING":         # 台から離れる
            return x0, y0 - d
        if self.kind == "CROSSING":        # 左から右へ（stop_at_x_mm があればそこで止まる）
            x = x0 + d
            return (x if self.stop_at_x_mm is None else min(x, self.stop_at_x_mm)), y0
        raise ValueError(f"未知の軌跡: {self.kind}（{TRAJECTORIES}）")

    def pose(self, t: float) -> Pose2D:
        """**Ground Truth。** Vision の成功として数えない。"""
        x, y = self.at(t)
        return Pose2D(x, y, self.heading_rad, PoseSource.GROUND_TRUTH_SIM, at_s=t, confidence=1.0)


def visitor(cfg: dict[str, Any], kind: str = "WALKING", *, appear_s: float = 0.0,
            lane_mm: float | None = None, speed_mm_s: float = 500.0) -> VirtualPerson:
    """展示レイアウトに合わせた来場者を1人作る（来場者は y<0 側）。"""
    width = float(cfg["mat"]["width_mm"])
    x = width / 2.0 if lane_mm is None else lane_mm
    if kind == "LEAVING":
        start = (x, -400.0)
    elif kind == "CROSSING":
        start = (-300.0, -500.0)
    elif kind == "STATIONARY":
        start = (x, -600.0)
    else:                                   # WALKING
        start = (x, -1600.0)
    stop_y = -450.0 if kind == "WALKING" else None      # 台の手前で立ち止まる
    return VirtualPerson(kind=kind, start_xy=start, speed_mm_s=speed_mm_s, appear_s=appear_s,
                         stop_at_y_mm=stop_y)


def scenario(cfg: dict[str, Any]) -> list[VirtualPerson]:
    """展示のひと通り: 誰もいない → 近づいてくる → しばらく居る → 去る。"""
    return [
        visitor(cfg, "WALKING", appear_s=3.0, speed_mm_s=350.0),
    ]
