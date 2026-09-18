"""補間の素材: ease 関数・行き過ぎ（overshoot）・予備動作と減衰振動つきの1関節トラック。

Keyframe に足せる「動きの語彙」（anticipation / follow-through: Takayama, Dooley & Ju, HRI 2011）:
  anticipate … 本動作の前に進行方向と**逆**へ少し引く（予備動作）
  settle     … 到達後、減衰振動で止まる（follow-through）
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum


class Easing(Enum):
    """補間の種類。"""

    IN_OUT = "ease_in_out"
    OUT = "ease_out"
    OUT_OVERSHOOT = "ease_out_overshoot"


def ease_in_out(p: float) -> float:
    """3次の ease-in-out（0→1）。"""
    p = min(max(p, 0.0), 1.0)
    return 4.0 * p ** 3 if p < 0.5 else 1.0 - (-2.0 * p + 2.0) ** 3 / 2.0


def ease_out(p: float) -> float:
    """3次の ease-out（0→1）。"""
    p = min(max(p, 0.0), 1.0)
    return 1.0 - (1.0 - p) ** 3


# 各補間の「最大の傾き / 平均の傾き」（3次式なので 3）と、最大の2階微分（ease-out の p=0 で 6）
EASE_PEAK_SLOPE = 3.0
EASE_PEAK_CURVATURE = 6.0
SETTLE_SPAN_TAUS = 4.0            # 減衰振動をこの時定数ぶん続けてから終える（e^-4 ≈ 2%）


def min_duration_s(start: float, end: float, easing: Easing, overshoot_deg: float,
                   peak_ratio: float, max_speed_dps: float) -> float:
    """最高角速度が max_speed_dps を超えないために必要な最短時間。"""
    if easing is Easing.OUT_OVERSHOOT and end != start:
        return EASE_PEAK_SLOPE * (abs(end - start) + abs(overshoot_deg)) / (peak_ratio * max_speed_dps)
    return EASE_PEAK_SLOPE * abs(end - start) / max_speed_dps


def min_duration_for_accel_s(travel_deg: float, lever_mm: float, max_accel_mps2: float) -> float:
    """先端の加速度が max_accel を超えないための最短時間（3次 ease の最大2階微分 6·Δ/T² から）。"""
    if travel_deg <= 0.0 or lever_mm <= 0.0 or max_accel_mps2 <= 0.0:
        return 0.0
    travel_m = math.radians(travel_deg) * lever_mm / 1000.0
    return math.sqrt(EASE_PEAK_CURVATURE * travel_m / max_accel_mps2)


def overshoot_value(start: float, end: float, p: float, overshoot_deg: float, peak_ratio: float) -> float:
    """行き過ぎ付きの補間値。

    0〜peak_ratio: start → end + overshoot（ease-out）
    peak_ratio〜1: そこから end に戻る（ease-in-out）
    """
    if end == start or overshoot_deg == 0.0:
        return start + (end - start) * ease_out(p)
    peak = end + math.copysign(overshoot_deg, end - start)
    if p < peak_ratio:
        return start + (peak - start) * ease_out(p / peak_ratio)
    return peak + (end - peak) * ease_in_out((p - peak_ratio) / (1.0 - peak_ratio))


@dataclass
class Track:
    """1関節の補間。予備動作（lead 秒で pull だけ逆へ）→ 本動作（duration）→ 減衰振動（settle）。"""

    start: float
    end: float
    t0: float
    duration: float
    easing: Easing
    overshoot_deg: float
    lead: float = 0.0            # 予備動作の時間 [s]
    pull: float = 0.0            # 予備動作で引く量（符号つき。start からの差）
    settle_amp: float = 0.0      # 減衰振動の初期振幅 [deg]
    settle_tau: float = 0.0      # 時定数 [s]
    settle_hz: float = 0.0

    @property
    def t_arrive(self) -> float:
        return self.t0 + self.lead + self.duration

    @property
    def t_done(self) -> float:
        return self.t_arrive + (SETTLE_SPAN_TAUS * self.settle_tau if self.settle_amp else 0.0)

    def value(self, t: float, peak_ratio: float) -> float:
        if t <= self.t0:
            return self.start
        if self.lead > 0.0 and t < self.t0 + self.lead:
            return self.start + self.pull * ease_out((t - self.t0) / self.lead)
        s0 = self.start + self.pull
        p = 1.0 if self.duration <= 0 else (t - self.t0 - self.lead) / self.duration
        if p >= 1.0:
            v = self.end
        elif self.easing is Easing.OUT_OVERSHOOT:
            v = overshoot_value(s0, self.end, p, self.overshoot_deg, peak_ratio)
        else:
            f = ease_out(p) if self.easing is Easing.OUT else ease_in_out(p)
            v = s0 + (self.end - s0) * f
        if self.settle_amp and t > self.t_arrive:
            tau = t - self.t_arrive
            v += self.settle_amp * math.exp(-tau / self.settle_tau) * math.sin(2.0 * math.pi * self.settle_hz * tau)
        return v
