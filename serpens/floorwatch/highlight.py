"""highlight_point（発見した物の位置を身体で示す: 物 → 人 → 物）の段取り。**KINEMATIC_SIM。実機・照明の制御は未接続。**

出典: `docs/task_event_api.md`（highlight_point）、CSAR（`serpens/floorwatch/csar.py`。User 採用 2026-09-29。**物を指す・照らすのは子どもが FAR のときだけ**。R1・R2・R5）。
動き: 頭ヨーだけで向ける（胴は動かさない）。物を `look_s` 見る → 人（いれば）を `look_s` 見る → 物を `look_s` 見る。合計は `duration_s`（Task、最大 300 s）で打ち切る。
**安全側の規則**（疑わしいときは止める）:
  - 子どもが近い（NEAR / UNKNOWN）と**始めない**し、途中で近づいたら**その場でやめる**（物を指さない。executor が人の方を向く = CSAR R5）。
  - 頭ヨーの範囲（`floor_watch.csar.look_at_person_max_deg` と同じ上限）を超えて向けられない地点は `failed`（無理に回さない）。
  - 人が見えないときは、人を見る段を飛ばす（物 → 物）。
照明（ライン光・閃光）は使わない（模擬に無い。CSAR で子どもが近いと禁止のものも、ここでは扱わない）。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np

OBJECT1, PERSON, OBJECT2, DONE, FAILED = "OBJECT1", "PERSON", "OBJECT2", "DONE", "FAILED"


@dataclass
class HighlightMission:
    target_mm: np.ndarray                                   # 物の位置（world, mm）
    duration_s: float
    attention_ok: Callable[[float], bool]                   # CSAR: 物を指してよいか（FAR のときだけ True）
    look_s: float = 1.5
    t0: float | None = None
    phase: str = OBJECT1
    reason: str = ""
    _phase_t: float = 0.0
    log: list[tuple[float, str]] = field(default_factory=list)

    @property
    def active(self) -> bool:
        return self.phase in (OBJECT1, PERSON, OBJECT2)

    def tick(self, t: float, look_at: Callable[[np.ndarray], bool], person_mm: np.ndarray | None) -> None:
        """1 周期。look_at(xy) = その点へ頭を向ける（上限内で向けられたら True）。"""
        if not self.active:
            return
        if self.t0 is None:
            self.t0, self._phase_t = t, t
        if not self.attention_ok(t):
            self._fail(t, "CSAR: 子どもが近いかもしれない（または確かめられない）ので、物を指し示すのをやめた")
            return
        if t - self.t0 >= self.duration_s:
            self._set(t, DONE, "指定の時間（duration_s）で終わった")
            return
        target = self.target_mm if self.phase in (OBJECT1, OBJECT2) else person_mm
        if self.phase == PERSON and person_mm is None:
            self._advance(t)                                  # 人が見えない = 人を見る段を飛ばす
            return
        if not look_at(np.asarray(target, float)):
            self._fail(t, "頭ヨーの範囲外で、地点へ向けられない（無理に回さない）")
            return
        if t - self._phase_t >= self.look_s:
            self._advance(t)

    def _advance(self, t: float) -> None:
        nxt = {OBJECT1: PERSON, PERSON: OBJECT2, OBJECT2: DONE}[self.phase]
        self._set(t, nxt, "" if nxt != DONE else "物 → 人 → 物を示した")

    def _set(self, t: float, phase: str, reason: str) -> None:
        self.phase, self._phase_t = phase, t
        if reason:
            self.reason = reason
        self.log.append((t, phase))

    def _fail(self, t: float, reason: str) -> None:
        self._set(t, FAILED, reason)

    def abort(self, reason: str) -> None:
        self.phase, self.reason = FAILED, reason
