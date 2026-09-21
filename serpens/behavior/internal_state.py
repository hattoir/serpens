"""内部状態: Curiosity / Affection / Stress / Attention / Familiarity / Sleepiness と、活動由来の Energy。

各状態は一次遅れ系（仕様は docs/internal_state_model.md）:
    dx/dt = −(x − baseline)/τ + Σ gain × 刺激        （0〜saturation にクランプ）
    τ = decay_tau_s（x が baseline より上にあるとき。刺激が消えたあと下がる速さ）
      = rise_tau_s （x が baseline より下にあるとき。回復して上がる速さ）
    gains の負値は抑制（inhibition）。
    kick(name, amount) は一度きりの出来事（駆け込み・退避・人の入れ替わり）で一段動かす。

Energy は**キャラクターの状態**（活動の疲れ）: energy = 1 − fatigue、
    d(fatigue)/dt = gain_move × (歩容が動いている) − fatigue / tau_recover。
サーボ温度は**機械の状態**で、ここでは Energy に混ぜない（旧実装は min(温度, 活動) にしていた）。
温度は brain の安全層（過熱 → COIL_REST_HEAT）と、その手前の「休ませたい」要求（thermal.rest_request_c）に使う。
係数はすべて config の behavior.internal / behavior.energy。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

STATE_NAMES = ("curiosity", "affection", "stress", "attention", "familiarity", "sleepiness")
STIMULI = ("presence", "proximity", "approach", "touch", "novelty", "looking", "alone", "calm")


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
    calm: float = 0.0              # 人がいて、近づいて来ず、Stress が低い（安全なやりとりが続いている）

    def get(self, name: str) -> float:
        return float(getattr(self, name))


@dataclass
class Channel:
    """1つの内部状態のダイナミクス。"""

    baseline: float
    rise_tau_s: float
    decay_tau_s: float
    saturation: float
    gains: dict[str, float] = field(default_factory=dict)

    @staticmethod
    def from_cfg(name: str, c: dict[str, Any]) -> "Channel":
        gains = {k: float(v) for k, v in c.get("gains", {}).items()}
        unknown = set(gains) - set(STIMULI)
        if unknown:
            raise ValueError(f"behavior.internal.{name}: 未知の刺激 {unknown}")
        ch = Channel(float(c["baseline"]), float(c["rise_tau_s"]), float(c["decay_tau_s"]),
                     float(c.get("saturation", 1.0)), gains)
        if not 0.0 <= ch.baseline <= ch.saturation <= 1.0:
            raise ValueError(f"behavior.internal.{name}: 0 ≤ baseline ≤ saturation ≤ 1 でない")
        if ch.rise_tau_s <= 0 or ch.decay_tau_s <= 0:
            raise ValueError(f"behavior.internal.{name}: tau は正")
        return ch

    def step(self, x: float, dt: float, s: Stimuli) -> float:
        tau = self.decay_tau_s if x > self.baseline else self.rise_tau_s
        dx = -(x - self.baseline) / tau + sum(g * s.get(k) for k, g in self.gains.items())
        return min(max(x + dx * dt, 0.0), self.saturation)


def energy_from_temperature(temp_c: float, fresh_c: float, tired_c: float) -> float:
    """最高温度 → 0〜1（表示・休憩要求の目安。**Energy には混ぜない**）。"""
    if tired_c <= fresh_c:
        raise ValueError("behavior.thermal: rest_request_c は fresh_c より高くすること")
    return min(max((tired_c - temp_c) / (tired_c - fresh_c), 0.0), 1.0)


class InternalState:
    """内部状態の保持と更新。"""

    def __init__(self, cfg: dict[str, Any]) -> None:
        b = cfg["behavior"]
        self._ch: dict[str, Channel] = {n: Channel.from_cfg(n, b["internal"][n]) for n in STATE_NAMES}
        e, th = b["energy"], b["thermal"]
        self._f_gain = float(e["fatigue"]["gain_move"])
        self._f_tau = float(e["fatigue"]["tau_recover_s"])
        self._t_fresh, self._t_rest = float(th["fresh_c"]), float(th["rest_request_c"])
        self.values: dict[str, float] = {n: ch.baseline for n, ch in self._ch.items()}
        self.fatigue = 0.0                  # 活動による疲れ（0〜1）
        self.heat_c: float | None = None    # サーボ最高温度（機械の状態。表示と休憩要求だけに使う）
        self.last_stimuli = Stimuli()

    def __getattr__(self, name: str) -> float:
        if name in STATE_NAMES:
            return self.__dict__["values"][name]
        raise AttributeError(name)

    @property
    def energy(self) -> float:
        """キャラクターの元気（活動由来のみ）。"""
        return 1.0 - self.fatigue

    @property
    def thermal_rest_request(self) -> bool:
        """サーボが休ませたい温度か（安全層の過熱停止より手前。Behavior 側は REST を優先しやすくなるだけ）。"""
        return self.heat_c is not None and self.heat_c >= self._t_rest

    def channel(self, name: str) -> Channel:
        return self._ch[name]

    def update(self, dt: float, s: Stimuli, max_temp_c: float | None, moving: bool = False) -> None:
        """dt 秒ぶん進める。max_temp_c はサーボの最高温度（読めなければ None で据え置き）。"""
        self.last_stimuli = s
        for name, ch in self._ch.items():
            self.values[name] = ch.step(self.values[name], dt, s)
        if max_temp_c is not None:
            self.heat_c = max_temp_c
        df = (self._f_gain if moving else 0.0) - self.fatigue / self._f_tau
        self.fatigue = min(max(self.fatigue + df * dt, 0.0), 1.0)

    def kick(self, name: str, amount: float) -> None:
        """一度きりの出来事（驚きなど）で値を一段動かす。0〜saturation にクランプ。"""
        self.values[name] = min(max(self.values[name] + amount, 0.0), self._ch[name].saturation)

    def snapshot(self) -> dict[str, float]:
        """GUI・ログ用。"""
        d = dict(self.values)
        d["energy"] = self.energy
        d["fatigue"] = self.fatigue
        return d
