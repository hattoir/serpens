"""サーボ状態ポーラーのテスト: 位置と熱系を別頻度で読む。"""
from __future__ import annotations

import pytest

from serpens.config import load_config
from serpens.hw.mock_bus import MockServoBus
from serpens.hw.state_poller import ServoStatePoller
from tests.helpers import FakeClock


class CountingBus(MockServoBus):
    """読み出し回数を数え、SYNC READ の有無を切り替えられるモック。"""

    def __init__(self, cfg: dict, clock: FakeClock, fast: bool) -> None:
        super().__init__(cfg, clock=clock)
        self.fast = fast
        self.n_pos = self.n_full = 0

    @property
    def fast_reads(self) -> bool:
        return self.fast

    def read_positions(self, ids=None):  # type: ignore[no-untyped-def]
        self.n_pos += 1
        return {s: st.pos_deg for s, st in super()._read_states(list(ids or self.ids)).items()}

    def sync_read_states(self, ids=None):  # type: ignore[no-untyped-def]
        self.n_full += 1
        return super().sync_read_states(ids)


@pytest.mark.parametrize("fast, key", [(True, "position_hz_sync"), (False, "position_hz_fallback")])
def test_poll_rates(fast: bool, key: str) -> None:
    cfg = load_config()
    clock = FakeClock()
    bus = CountingBus(cfg, clock, fast)
    bus.connect()
    poller = ServoStatePoller(bus, cfg, clock)
    for _ in range(2000):          # 10 秒間、5ms ごとに poll
        poller.poll()
        clock.advance(0.005)
    p = cfg["servo_poll"]
    reads = bus.n_pos + bus.n_full
    assert reads == pytest.approx(10 * p[key], rel=0.1)
    assert bus.n_full == pytest.approx(10 * p["thermal_hz"], abs=1)
    assert poller.max_temperature_c() == pytest.approx(cfg["mock_servo"]["ambient_c"], abs=0.5)
    assert set(poller.positions) == set(bus.ids)
