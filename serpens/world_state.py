"""World State — 上位（行動・計画・GUI）が見る「世界の見え方」。

**姿勢には必ず出どころ（`PoseSource`）を持たせる。**
シミュレーションの Ground Truth を「Vision が成功した」として数えないため。

    GROUND_TRUTH_SIM … シミュレータが知っている真値（**Vision の成果ではない**）
    ARUCO            … マーカから推定した自己位置
    PERSON_DETECTOR  … 人物検出から推定した人の位置
    DEAD_RECKONING   … 指令からの推測（観測なし）
    UNKNOWN          … 出どころ不明（使わない）

Phase 4 でカメラを繋いだら、**同じ `WorldState` の中身が `ARUCO` / `PERSON_DETECTOR` へ
差し替わるだけ**で、行動側は変わらない。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum


class PoseSource(str, Enum):
    """姿勢がどこから来たか。"""

    GROUND_TRUTH_SIM = "GROUND_TRUTH_SIM"
    ARUCO = "ARUCO"
    PERSON_DETECTOR = "PERSON_DETECTOR"
    DEAD_RECKONING = "DEAD_RECKONING"
    UNKNOWN = "UNKNOWN"

    @property
    def is_vision(self) -> bool:
        """カメラから得た推定か（**Ground Truth は含まない**）。"""
        return self in (PoseSource.ARUCO, PoseSource.PERSON_DETECTOR)


@dataclass(frozen=True)
class Pose2D:
    """床の上の姿勢 [mm, rad] と、その出どころ。"""

    x_mm: float
    y_mm: float
    theta_rad: float = 0.0
    source: PoseSource = PoseSource.UNKNOWN
    at_s: float = 0.0                     # いつの観測か
    confidence: float = 1.0               # 0〜1（Vision では検出スコア）
    simulated: bool = False               # 模擬画像から得た推定（ARUCO でも実カメラではない）

    def distance_to(self, other: "Pose2D") -> float:
        return math.hypot(self.x_mm - other.x_mm, self.y_mm - other.y_mm)

    def age_s(self, now: float) -> float:
        return max(now - self.at_s, 0.0)


@dataclass
class WorldState:
    """その時点の世界。**上位はここだけを見る。**"""

    t: float = 0.0
    robot: Pose2D | None = None           # ヘビ（首マーカ基準の自己位置）
    head: Pose2D | None = None            # 頭の向き（θ_head）
    people: list[Pose2D] = field(default_factory=list)
    target: Pose2D | None = None          # いま相手にしている人
    notes: list[str] = field(default_factory=list)

    @property
    def sources(self) -> set[PoseSource]:
        poses = [p for p in (self.robot, self.head, self.target) if p is not None]
        return {p.source for p in poses} | {p.source for p in self.people}

    @property
    def any_real_vision(self) -> bool:
        """**実カメラ**由来の推定が入っているか（模擬画像の Vision は数えない）。"""
        poses = [p for p in (self.robot, self.head, self.target) if p is not None] + self.people
        return any(p.source.is_vision and not p.simulated for p in poses)

    @property
    def any_vision(self) -> bool:
        """**Vision 由来の推定が1つでも入っているか。** 模擬の真値だけなら False。"""
        return any(s.is_vision for s in self.sources)

    def stale(self, now: float, max_age_s: float) -> list[str]:
        """古くなった観測の名前（表示・開始条件の判断に使う）。"""
        out = []
        for name, pose in (("robot", self.robot), ("head", self.head), ("target", self.target)):
            if pose is not None and pose.age_s(now) > max_age_s:
                out.append(name)
        return out
