"""シミュレータで「1周期あたり何 mm 進むか」を測る（実機との比較・校正用）。"""
from __future__ import annotations

import copy
import math
from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from serpens.motion.gait import GaitEngine, GaitParams, gait_period_s
from serpens.sim.world import BodyPose, World

FAR_MAT_MM = 1.0e6     # 測定中にマット端に当たらないよう、マットを十分広くする


@dataclass(frozen=True)
class Advance:
    """1周期あたりの結果。"""

    per_cycle_mm: float      # 重心の正味の移動量
    turn_deg: float          # 向きの変化


def gait_with_period(cfg: dict[str, Any], gait: str, period_s: float | None) -> GaitParams:
    """プリセットの周期だけ差し替える（向き＝前進/後退は保つ）。"""
    p = GaitParams.from_cfg(cfg["gait"]["presets"][gait])
    if period_s is None:
        return p
    return replace(p, temporal_freq_hz=math.copysign(1.0 / period_s, p.temporal_freq_hz))


def per_cycle_advance(cfg: dict[str, Any], params: GaitParams,
                      warmup_cycles: float, measure_cycles: float) -> Advance:
    """助走ののち measure_cycles 周期歩かせ、1周期あたりの移動量と回転を返す（指令角そのまま）。"""
    c = copy.deepcopy(cfg)
    c["mat"]["width_mm"] = c["mat"]["depth_mm"] = FAR_MAT_MM
    dt = float(c["sim"]["dt_s"])
    w = World(c, BodyPose(FAR_MAT_MM / 2, FAR_MAT_MM / 2, 0.0))
    eng = GaitEngine(c)
    eng.start(params)
    T = gait_period_s(params)
    t = 0.0
    for _ in range(int(round(warmup_cycles * T / dt))):
        t += dt
        w.step(eng.update(t), dt)
    c0, th0 = w.centroid(), w.pose.theta
    for _ in range(int(round(measure_cycles * T / dt))):
        t += dt
        w.step(eng.update(t), dt)
    d = float(np.linalg.norm(w.centroid() - c0)) / measure_cycles
    return Advance(d, math.degrees(w.pose.theta - th0) / measure_cycles)
