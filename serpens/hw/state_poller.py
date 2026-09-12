"""サーボ状態の定期読み出しと、値の鮮度の管理。

**古い値やモックの値を実機の正常状態として扱わない。** 軸ごとに最後に読めた時刻を持ち、
behavior.safety.telemetry.stale_after_s より古い軸は「不明」として扱う（GUI も灰色で出す）。

SYNC READ が使えないと9軸を1つずつ読むことになり遅いので、頻度を分ける:
  位置        … servo_poll.position_hz_sync（一括）/ position_hz_fallback（個別）
  負荷・電圧・温度 … servo_poll.thermal_hz（Energy と GUI 用。温度はゆっくりしか変わらない）
poll() は毎制御周期呼んでよい。時刻が来たものだけ実際にバスを読む。
"""
from __future__ import annotations

import time
from typing import Any, Callable

from serpens.hw.servo_bus import ServoBus, ServoCommError, ServoState

Clock = Callable[[], float]


class ServoStatePoller:
    """位置と熱系を別々の頻度で読み、最新値を保持する。"""

    def __init__(self, bus: ServoBus, cfg: dict[str, Any], clock: Clock | None = None) -> None:
        p = cfg["servo_poll"]
        self.bus = bus
        self._clock: Clock = clock or time.monotonic
        self._pos_hz_fast = float(p["position_hz_sync"])
        self._pos_hz_slow = float(p["position_hz_fallback"])
        self._thermal_period = 1.0 / float(p["thermal_hz"])
        self._next_pos = self._next_thermal = self._clock()
        self.positions: dict[int, float] = {}
        self.states: dict[int, ServoState] = {}
        self.errors = 0
        self.last_error: str = ""
        self.stale_after_s = float(cfg["behavior"]["safety"]["telemetry"]["stale_after_s"])
        self.seen_at: dict[int, float] = {}          # 軸ごとに最後に読めた時刻
        self.source = type(bus).__name__             # 値の出どころ（実機かモックか）

    @property
    def position_hz(self) -> float:
        """現在の位置読み出し頻度。"""
        return self._pos_hz_fast if self.bus.fast_reads else self._pos_hz_slow

    def poll(self) -> None:
        """時刻が来ていれば読む。通信エラーは数えるだけで止めない。"""
        now = self._clock()
        try:
            if now >= self._next_thermal:
                got = self.bus.sync_read_states()
                self.states.update(got)
                self.positions.update({s: st.pos_deg for s, st in got.items()})
                self._mark_seen(got, now)
                self._next_thermal = now + self._thermal_period
                self._next_pos = now + 1.0 / self.position_hz
            elif now >= self._next_pos:
                got = self.bus.read_positions()
                self.positions.update(got)
                self._mark_seen(got, now)
                self._next_pos = now + 1.0 / self.position_hz
        except ServoCommError as e:
            self.errors += 1
            self.last_error = str(e)

    # ---- 鮮度 -------------------------------------------------------------------
    def _mark_seen(self, got: dict[int, object], now: float) -> None:
        for sid in got:
            self.seen_at[sid] = now

    def age_s(self, servo_id: int, now: float | None = None) -> float | None:
        """その軸の値が何秒前のものか。一度も読めていなければ None。"""
        t = self._clock() if now is None else now
        seen = self.seen_at.get(servo_id)
        return None if seen is None else max(t - seen, 0.0)

    def is_fresh(self, servo_id: int, now: float | None = None) -> bool:
        """stale_after_s 以内に読めているか。"""
        age = self.age_s(servo_id, now)
        return age is not None and age <= self.stale_after_s

    def fresh_states(self, now: float | None = None) -> dict[int, ServoState]:
        """鮮度が保証できる軸だけの状態。"""
        return {sid: st for sid, st in self.states.items() if self.is_fresh(sid, now)}

    def missing_axes(self, now: float | None = None) -> list[int]:
        """未取得または古い軸。"""
        return [sid for sid in self.bus.ids if not self.is_fresh(sid, now)]

    def newest_age_s(self, now: float | None = None) -> float | None:
        """いちばん新しい軸の古さ（全軸未取得なら None）。"""
        ages = [a for a in (self.age_s(sid, now) for sid in self.bus.ids) if a is not None]
        return min(ages) if ages else None

    def max_temperature_c(self, now: float | None = None) -> float | None:
        """全軸の最高温度（Energy の計算に使う）。**古い値は使わない**。"""
        return max((st.temp_c for st in self.fresh_states(now).values()), default=None)

    def max_abs_load(self, now: float | None = None) -> float | None:
        """負荷率の最大（掴まれ検知に使う）。古い値は使わない。"""
        loads = [abs(st.load) for st in self.fresh_states(now).values()]
        return max(loads) if loads else None
