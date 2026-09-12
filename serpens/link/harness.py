"""偽時計で PC 側（client）と機体側（device）を噛み合わせる足場。

試験（tests/test_phase2_*.py）と実測の記録（tools/link_check.py）が同じものを使う。
実時間で動かすと待ち時間ばかりになるので、**時刻は呼び出し側が進める**。
"""
from __future__ import annotations

from typing import Any, Callable

from serpens.link.client import LinkClient
from serpens.link.device import SimulatedDevice
from serpens.link.transport import LoopbackTransport


class LinkHarness:
    """PC ⇄ 偽経路 ⇄ 機体。1ステップ = 機体の制御1周期。"""

    def __init__(self, cfg: dict[str, Any], now: float = 0.0, chunk: int = 0) -> None:
        self.cfg = cfg
        self.now = now
        self.dt = 1.0 / float(cfg["link"]["control_hz"])
        self.device = SimulatedDevice(cfg, now=now)
        self.tr = LoopbackTransport(self.device, chunk=chunk)
        self.client = LinkClient(self.tr, cfg, now=now)

    def step(self) -> None:
        """時刻を1周期進め、PC → 機体 → PC の順に1往復させる。"""
        self.now += self.dt
        self.client.update(self.now)
        self.tr.pump(self.now)

    def advance(self, seconds: float) -> None:
        """指定の時間だけ進める。"""
        for _ in range(max(int(round(seconds / self.dt)), 0)):
            self.step()

    def run_until(self, pred: Callable[[], bool], timeout_s: float = 5.0) -> float | None:
        """条件が成り立つまで進め、掛かった時間を返す（成らなければ None）。"""
        t0 = self.now
        for _ in range(max(int(round(timeout_s / self.dt)), 1)):
            self.step()
            if pred():
                return self.now - t0
        return None

    def start_driving(self, amplitude_deg: float = 30.0, spatial_freq_deg: float = 60.0,
                      temporal_freq_hz: float = 0.5, gamma_deg: float = 0.0,
                      settle_s: float = 0.5) -> bool:
        """人の操作にあたる手順: heartbeat を確立 → ARM → DRIVE。走り出したら True。"""
        self.advance(0.2)                                    # heartbeat を先に通す
        self.client.arm(self.now)
        self.advance(0.1)
        self.client.set_drive(amplitude_deg, spatial_freq_deg, temporal_freq_hz, gamma_deg)
        self.advance(settle_s)
        return self.device.driving

    def moved_deg(self, seconds: float) -> float:
        """この先 `seconds` の間に出力が動いた最大角（停止の確認に使う）。"""
        before = dict(self.device.output)
        self.advance(seconds)
        after = self.device.output
        return max(abs(after[k] - before[k]) for k in before)
