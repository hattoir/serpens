"""上位から見た2つのヘビ。**同じ `RobotInterface` なので、行動・安全・GUI は書き換えずに差し替わる。**

    RobotInterface
     ├── SimulatedSnake … PC → Virtual ESP32 → Virtual Servo Bus → Simulated Serpens
     └── RealSnake      … PC → ESP32-S3 → STS3215

違いは**経路だけ**（偽経路か実シリアルか）。機体側の状態機械・上限・watchdog は同じ仕様で、
`SimulatedSnake` は `serpens/link/device.py`、`RealSnake` は `firmware/serpens_esp32/` が実装する。

検証レベル（docs/verification_status.md）:
  - `SimulatedSnake` … SIMULATED / SOFTWARE_VERIFIED
  - `RealSnake` … **HARDWARE_UNVERIFIED**（実機が1台も無く、一度も通信していない）
"""
from __future__ import annotations

from typing import Any, Callable

from serpens.link.client import LinkClient
from serpens.link.device import SimulatedDevice
from serpens.link.faults import FaultInjector
from serpens.link.robot import LinkRobot
from serpens.link.transport import LoopbackTransport, SerialTransport


class SimulatedSnake(LinkRobot):
    """仮想 ESP32 と仮想サーボを相手にするヘビ。**実機が1台も無くても全部動く。**

    故障注入（`self.faults` / `self.device.inject_axis` / `self.transport.unplug()` …）で、
    通信断・再起動・サーボ異常を再現できる。
    """

    def __init__(self, cfg: dict[str, Any], clock: Callable[[], float], *, now: float = 0.0,
                 chunk: int = 0, faults: FaultInjector | None = None, boot_id: int = 1) -> None:
        self.device = SimulatedDevice(cfg, now=now, boot_id=boot_id)
        self.transport = LoopbackTransport(self.device, chunk=chunk, faults=faults)
        super().__init__(cfg, LinkClient(self.transport, cfg, now=now), clock,
                         pump=self.transport.pump)

    @property
    def faults(self) -> FaultInjector:
        """経路に起こす異常（loss / delay / 順序入れ替え / 重複 / CRC 破損）。"""
        return self.transport.faults

    def reboot(self) -> None:
        """機体だけ電源を入れ直す。**PC は自動で走行を再開しない。**"""
        self.transport.reboot_device(self._clock())


class RealSnake(LinkRobot):
    """実機の ESP32-S3（USB CDC）を相手にするヘビ。

    **HARDWARE_UNVERIFIED。** ファームは未コンパイル・未書き込みで、この経路は
    一度も実際の COM ポートで動かしていない。最初に試す手順は
    `firmware/serpens_esp32/README.md` の「実機で最初に確かめること」。
    """

    def __init__(self, cfg: dict[str, Any], clock: Callable[[], float], port: str,
                 baudrate: int = 921600) -> None:
        self.transport = SerialTransport(port, baudrate)
        self.port = port
        super().__init__(cfg, LinkClient(self.transport, cfg), clock, pump=None)
