"""フード（口の前の開放底フード = 案 L2 の「フード昇降」）の端の検出（1 ビット）と、0.6 s のタイムアウト。**模擬 + 純粋なロジック。実機・ファームは未接続。**

出典: ENTRY-D-0015 (5)（案 H = 磁石 φ3 + ホール素子 2 個、上 / 下の 1 ビット）、REQUESTS R-017 / R-023、`ai-shared/HARDWARE_TODO.md` HT-013、`simulation/hardware_gaps/HG-S3_torque_limiter/flank_v.md` §F。
決まり（R-017）:
  **昇降の指令から `endstop_timeout_s`（0.6 s）で「下がりきった / 上がりきった」が立たなければ、フードの駆動を保持し、前進を止め、記録する。**
安全側の規則（すべて「疑わしいときは止める」。緩める方向のものは無い）:
  - 立たない = 偽（False）だけでなく、読めていない（None）も「立っていない」。
  - 下と上が**同時に立つ**のは物理的にありえない = センサーの故障として止める。
  - 下がりきった後に下のビットが落ちた（指令なし）= 意図しない持ち上がり。止める。
  - 止まったら**ラッチ**する。原因が消えても自動で再開しない（`acknowledge` = 人の操作だけ。機体の緊急停止ラッチと同じ考え方）。
  - 一定時間（`endstop_stale_s`）読み出しが無い = 止める（読めない = 立っていない）。
ビットの出どころは頭の XIAO（SG90 の位置は返らないので、機構側のビットだけが頼り）。**頭の XIAO のファームはリポジトリに無い**ので、ここは仕様と試験のベクトルを置く段階（ファームへの接続は未）。
`HARDWARE_VERIFIED = 0`。0.6 s は Design / Engineering の仮置き（ASSUMED。HT-013 の SG90 の動作時間と検出遅れの実測で見直す）。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

UNKNOWN, DOWN, UP, MOVING_DOWN, MOVING_UP, STUCK = "UNKNOWN", "DOWN", "UP", "MOVING_DOWN", "MOVING_UP", "STUCK"
TARGETS = (DOWN, UP)


@dataclass
class HoodEvent:
    t: float
    kind: str          # command / settled / stuck / acknowledge / rejected
    state: str
    reason: str = ""


class HoodMonitor:
    """フードの状態（端の 1 ビット ×2）と、タイムアウトの判定。時計は呼び出し側が渡す（模擬の時刻でも実時刻でも同じ）。"""

    def __init__(self, endstop_timeout_s: float = 0.6, endstop_stale_s: float = 0.6) -> None:
        if endstop_timeout_s <= 0 or endstop_stale_s <= 0:
            raise ValueError("タイムアウトは正の値")
        self.timeout_s = float(endstop_timeout_s)
        self.stale_s = float(endstop_stale_s)
        self.state: str = UNKNOWN
        self.events: list[HoodEvent] = []
        self._target: str | None = None
        self._deadline: float | None = None
        self._last_read_t: float | None = None

    @classmethod
    def from_cfg(cls, cfg: dict[str, Any]) -> "HoodMonitor":
        h = cfg["floor_watch"]["hood"]
        return cls(float(h["endstop_timeout_s"]), float(h["endstop_stale_s"]))

    # ---- 出力（フード・前進への許可）-------------------------------------------------
    @property
    def drive_enabled(self) -> bool:
        """フードの駆動（SG90 への指令）を出してよいか。STUCK のあいだは出さない（保持）。"""
        return self.state != STUCK

    @property
    def forward_inhibit(self) -> bool:
        """前進（歩容）を止めるべきか。STUCK のあいだ True（ラッチ。`acknowledge` まで続く）。"""
        return self.state == STUCK

    @property
    def intake_push_allowed(self) -> bool:
        """物を押し込む動き（口の前進）をしてよいか。フードが**完全に下がっている**ときだけ。"""
        return self.state == DOWN

    @property
    def stuck_reason(self) -> str:
        for e in reversed(self.events):
            if e.kind == "stuck":
                return e.reason
        return ""

    # ---- 入力 -------------------------------------------------------------------------
    def command(self, target: str, now: float) -> bool:
        """昇降の指令。受理したら True。STUCK のあいだは拒否（人の `acknowledge` が先）。"""
        if target not in TARGETS:
            raise ValueError(f"target は {TARGETS}")
        if self.state == STUCK:
            self._log(now, "rejected", f"STUCK のため {target} の指令を受けない")
            return False
        if self.state == target:
            return True                                    # すでにその端。何もしない
        self.state = MOVING_DOWN if target == DOWN else MOVING_UP
        self._target = target
        self._deadline = now + self.timeout_s
        self._log(now, "command", f"{target} へ（期限 {self._deadline:.3f}）")
        return True

    def update(self, down_bit: bool | None, up_bit: bool | None, now: float) -> str:
        """端のビットを 1 回分入れる（None = 読めなかった）。新しい状態を返す。"""
        if self.state == STUCK:
            return self.state
        if down_bit is not None or up_bit is not None:
            self._last_read_t = now
        if down_bit is True and up_bit is True:
            return self._stuck(now, "下と上の端のビットが同時に立った（センサーの故障。物理的にありえない）")
        if self.state in (MOVING_DOWN, MOVING_UP):
            want_bit = down_bit if self.state == MOVING_DOWN else up_bit
            if want_bit is True:
                self.state = self._target or UNKNOWN
                self._target = self._deadline = None
                self._log(now, "settled", f"{self.state} のビットが立った")
            elif self._deadline is not None and now >= self._deadline:
                shown = "読めなかった（None）" if want_bit is None else "立たなかった（False）"
                return self._stuck(now, f"指令から {self.timeout_s:g} s で {self._target} のビットが{shown}")
            return self.state
        if self.state in (DOWN, UP):
            held = down_bit if self.state == DOWN else up_bit
            if held is False:
                return self._stuck(now, f"{self.state} のビットが指令なしに落ちた（意図しない動き）")
        if self.state == UNKNOWN:
            if down_bit is True:
                self.state = DOWN
                self._log(now, "settled", "最初の読みで DOWN")
            elif up_bit is True:
                self.state = UP
                self._log(now, "settled", "最初の読みで UP")
        self._check_stale(now)
        return self.state

    def tick(self, now: float) -> str:
        """読み出しが無い周期の時間経過だけを進める（期限・古さの判定）。"""
        if self.state in (MOVING_DOWN, MOVING_UP) and self._deadline is not None and now >= self._deadline:
            return self._stuck(now, f"指令から {self.timeout_s:g} s で {self._target} のビットが読めなかった（None）")
        if self.state != STUCK:
            self._check_stale(now)
        return self.state

    def acknowledge(self, now: float) -> bool:
        """STUCK のラッチを人が解く。状態は UNKNOWN に戻り、新しい読みで確かめ直す（自動では呼ばない）。"""
        if self.state != STUCK:
            return False
        self.state = UNKNOWN
        self._target = self._deadline = None
        self._last_read_t = None
        self._log(now, "acknowledge", "人の操作でラッチを解除（UNKNOWN から確かめ直す）")
        return True

    # ---- 内部 -------------------------------------------------------------------------
    def _check_stale(self, now: float) -> None:
        if self.state in (DOWN, UP) and self._last_read_t is not None and now - self._last_read_t >= self.stale_s:
            self._stuck(now, f"{self.stale_s:g} s 以上、端のビットの読み出しが無い（読めない = 立っていない）")

    def _stuck(self, now: float, reason: str) -> str:
        self.state = STUCK
        self._target = self._deadline = None
        self._log(now, "stuck", reason)
        return self.state

    def _log(self, t: float, kind: str, reason: str) -> None:
        self.events.append(HoodEvent(float(t), kind, self.state, reason))


@dataclass
class MockHood:
    """フードの模擬（機構 + 端のビット）。`delay_s` で動き切る。`stuck=True` は動かない（固着）、`bit_dead=True` はビットが読めない。
    **模擬であって実機の挙動ではない**（SG90 の動作時間・ホール素子の遅れは HT-013 で実測する）。"""
    delay_s: float = 0.25
    stuck: bool = False
    bit_dead: bool = False
    position: str = UP
    _target: str | None = field(default=None, repr=False)
    _t_cmd: float | None = field(default=None, repr=False)
    log: list[tuple[float, str]] = field(default_factory=list)

    def drive(self, target: str, now: float) -> None:
        if target in TARGETS and target != self.position:
            self._target, self._t_cmd = target, now
            self.log.append((now, f"drive {target}"))

    def read(self, now: float) -> tuple[bool | None, bool | None]:
        """(下のビット, 上のビット)。動き切っていない間は両方 False（途中）。"""
        if self._target is not None and not self.stuck and self._t_cmd is not None and now - self._t_cmd >= self.delay_s:
            self.position, self._target, self._t_cmd = self._target, None, None
        if self.bit_dead:
            return None, None
        mid = self._target is not None
        return (self.position == DOWN and not mid), (self.position == UP and not mid)


def run_sequence(monitor: HoodMonitor, hood: MockHood, target: str, t0: float, duration_s: float, dt: float = 0.05,
                 on_step: Callable[[float, HoodMonitor], None] | None = None) -> float:
    """指令 → 駆動 → 周期ごとにビットを読む、を duration_s だけ進める。終わりの時刻を返す。"""
    if monitor.command(target, t0) and monitor.drive_enabled:
        hood.drive(target, t0)
    t = t0
    while t < t0 + duration_s:
        t += dt
        monitor.update(*hood.read(t), t)
        if on_step is not None:
            on_step(t, monitor)
    return t
