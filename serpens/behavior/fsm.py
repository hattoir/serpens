"""9状態の有限状態機械。どの状態へ移るかは効用で決め、切替の条件をここで守る。

  - 効用最大の状態を候補にする
  - 最小継続時間（min_dwell_s、状態ごとの上書きあり）が過ぎるまでは切り替えない
  - 候補の効用が「今の状態の効用 × hysteresis」を超えたときだけ切り替える
  - interrupt_states（PETTED, ALERT）は最小継続時間を待たずに割り込める。
    割り込み状態どうしは、config で先に書いた方が優先（PETTED は ALERT にも割り込める）
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from serpens.behavior.utility import STATES


@dataclass(frozen=True)
class Transition:
    """状態遷移の記録。"""

    t: float
    src: str
    dst: str
    reason: str


class StateMachine:
    """状態と、切替のルール。"""

    def __init__(self, cfg: dict[str, Any], initial: str = "PATROL", t0: float = 0.0) -> None:
        u = cfg["behavior"]["utility"]
        self.hysteresis = float(u["hysteresis"])
        self.min_dwell = float(u["min_dwell_s"])
        self.dwell_overrides = {k: float(v) for k, v in u.get("min_dwell_overrides", {}).items()}
        self.interrupts = set(u["interrupt_states"])
        self._priority = {s: i for i, s in enumerate(u["interrupt_states"])}   # 先に書いた方が優先
        unknown = (self.interrupts | set(self.dwell_overrides)) - set(STATES)
        if unknown:
            raise ValueError(f"behavior.utility: 未知の状態 {unknown}")
        self.state = initial
        self.entered_t = t0
        self.history: list[Transition] = []

    def dwell_of(self, state: str) -> float:
        return self.dwell_overrides.get(state, self.min_dwell)

    def time_in_state(self, t: float) -> float:
        return t - self.entered_t

    def time_to_next(self, t: float) -> float:
        """次に（通常の）切替ができるまでの秒数。"""
        return max(self.dwell_of(self.state) - self.time_in_state(t), 0.0)

    def step(self, t: float, utilities: dict[str, float]) -> Transition | None:
        """効用を見て、必要なら切り替える。切り替えたら Transition を返す。"""
        best = max(utilities, key=lambda k: utilities[k])
        if best == self.state:
            return None
        cur = utilities.get(self.state, 0.0)
        if utilities[best] <= self.hysteresis * cur:
            return None
        if best in self.interrupts and (self.state not in self.interrupts
                                        or self._priority[best] < self._priority[self.state]):
            return self._switch(t, best, "割り込み")
        if self.time_to_next(t) > 0.0:
            return None
        return self._switch(t, best, f"{utilities[best]:.2f} > {self.hysteresis}×{cur:.2f}")

    def force(self, t: float, state: str, reason: str) -> Transition | None:
        """ルールを無視して切り替える（安全停止など）。"""
        if state == self.state:
            return None
        return self._switch(t, state, reason)

    def _switch(self, t: float, dst: str, reason: str) -> Transition:
        tr = Transition(t, self.state, dst, reason)
        self.history.append(tr)
        self.state, self.entered_t = dst, t
        return tr
