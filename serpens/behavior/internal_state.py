"""内部状態: Curiosity / Affection / Energy / Stress / Attention。

Curiosity・Affection・Stress・Attention は一次遅れ系:
    dx/dt = -(x - x0)/τ + Σ gain × 刺激        （0〜1 にクランプ）
Energy だけはサーボの最高温度から決める（熱い = 疲れている）:
    T ≤ temp_fresh → 1.0、T ≥ temp_tired → 0.0、その間は直線。表示用に τ でなめらかにする
係数はすべて config の behavior.internal / behavior.energy。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

STATE_NAMES = ("curiosity", "affection", "stress", "attention")
STIMULI = ("presence", "proximity", "approach", "touch", "novelty", "looking", "alone")


@dataclass
class Stimuli:
    """1周期ぶんの刺激（すべて 0〜1）。"""

    presence: float = 0.0
    proximity: float = 0.0
    approach: float = 0.0
    touch: float = 0.0
    novelty: float = 0.0
    looking: float = 0.0
    alone: float = 0.0

    def get(self, name: str) -> float:
        return float(getattr(self, name))


@dataclass
class _Channel:
    x0: float
    tau_s: float
    gains: dict[str, float] = field(default_factory=dict)


def energy_from_temperature(temp_c: float, fresh_c: float, tired_c: float) -> float:
    """最高温度 → Energy（0〜1）。"""
    if tired_c <= fresh_c:
        raise ValueError("behavior.energy: temp_tired_c は temp_fresh_c より高くすること")
    return min(max((tired_c - temp_c) / (tired_c - fresh_c), 0.0), 1.0)


class InternalState:
    """内部状態の保持と更新。"""

    def __init__(self, cfg: dict[str, Any]) -> None:
        b = cfg["behavior"]
        self._ch: dict[str, _Channel] = {}
        for name in STATE_NAMES:
            c = b["internal"][name]
            gains = {k: float(v) for k, v in c["gains"].items()}
            unknown = set(gains) - set(STIMULI)
            if unknown:
                raise ValueError(f"behavior.internal.{name}: 未知の刺激 {unknown}")
            self._ch[name] = _Channel(float(c["x0"]), float(c["tau_s"]), gains)
        e = b["energy"]
        self._fresh, self._tired, self._e_tau = float(e["temp_fresh_c"]), float(e["temp_tired_c"]), float(e["tau_s"])
        self.values: dict[str, float] = {n: ch.x0 for n, ch in self._ch.items()}
        self.energy = 1.0
        self.heat_c: float | None = None
        self.last_stimuli = Stimuli()

    def __getattr__(self, name: str) -> float:
        if name in STATE_NAMES:
            return self.__dict__["values"][name]
        raise AttributeError(name)

    def update(self, dt: float, s: Stimuli, max_temp_c: float | None) -> None:
        """dt 秒ぶん進める。max_temp_c はサーボの最高温度（読めなければ None で据え置き）。"""
        self.last_stimuli = s
        for name, ch in self._ch.items():
            x = self.values[name]
            dx = -(x - ch.x0) / ch.tau_s + sum(g * s.get(k) for k, g in ch.gains.items())
            self.values[name] = min(max(x + dx * dt, 0.0), 1.0)
        if max_temp_c is not None:
            self.heat_c = max_temp_c
            target = energy_from_temperature(max_temp_c, self._fresh, self._tired)
            a = 1.0 - math.exp(-dt / self._e_tau) if self._e_tau > 0 else 1.0
            self.energy += a * (target - self.energy)

    def snapshot(self) -> dict[str, float]:
        """GUI・ログ用。"""
        d = dict(self.values)
        d["energy"] = self.energy
        return d
