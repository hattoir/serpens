"""サーボ状態の定期読み出し。

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

    @property
    def position_hz(self) -> float:
        """現在の位置読み出し頻度。"""
        return self._pos_hz_fast if self.bus.fast_reads else self._pos_hz_slow

    def poll(self) -> None:
        """時刻が来ていれば読む。通信エラーは数えるだけで止めない。"""
        now = self._clock()
        try:
            if now >= self._next_thermal:
                self.states.update(self.bus.sync_read_states())
                self.positions.update({s: st.pos_deg for s, st in self.states.items()})
                self._next_thermal = now + self._thermal_period
                self._next_pos = now + 1.0 / self.position_hz
            elif now >= self._next_pos:
                self.positions.update(self.bus.read_positions())
                self._next_pos = now + 1.0 / self.position_hz
        except ServoCommError:
            self.errors += 1

    def max_temperature_c(self) -> float | None:
        """全軸の最高温度（Energy の計算に使う）。"""
        return max((st.temp_c for st in self.states.values()), default=None)
