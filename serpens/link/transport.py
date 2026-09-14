"""駆動リンクの経路。実シリアル（pyserial）と、試験用の偽経路。

偽経路は **同じ SimulatedDevice を相手にする** ので、PC 側の実装を一切変えずに
「USB を抜く」「PC を強制終了する」「フレームを壊す・重複させる」「機体を再起動する」を再現できる
（完了条件 2/3/6/7）。実機での実測は tools/link_check.py と docs/phase2_acceptance.md。
"""
from __future__ import annotations

from typing import Any, Protocol

from serpens.link.faults import FaultInjector


MAX_CATCHUP_S = 1.0        # 時計が飛んだときに機体を回し続けない上限（試験で時刻を大きく進めた場合）


class Transport(Protocol):
    """PC 側から見た経路。読みは非ブロッキング（無ければ空）。"""

    connected: bool

    def write(self, data: bytes) -> None: ...

    def read(self) -> bytes: ...

    def close(self) -> None: ...


class LoopbackTransport:
    """偽経路。`pump(now)` を呼ぶまで機体は動かないので、時間を完全に制御できる。"""

    def __init__(self, device: Any, chunk: int = 0, faults: FaultInjector | None = None) -> None:
        self.device = device
        self.chunk = chunk              # 0 = 一度に届く。>0 ならこのバイト数ずつ刻んで届く
        self.connected = True
        self.faults = faults or FaultInjector()   # 落とす・遅らせる・壊す・重複・順序入れ替え
        self.pc_alive = True            # False = PC のプロセスが落ちた（送信が止まる）
        self._queue: list[tuple[float, bytes]] = []   # (届く時刻, バイト列)
        self._to_dev = bytearray()
        self._to_pc = bytearray()
        self._now = 0.0
        self._last_tick = 0.0
        self.tx_bytes = 0
        self.rx_bytes = 0

    # 故障注入の実体は FaultInjector。短く書けるように別名も残す
    @property
    def drop_next(self) -> int:
        return self.faults.drop_next

    @drop_next.setter
    def drop_next(self, n: int) -> None:
        self.faults.drop_next = n

    @property
    def duplicate_next(self) -> int:
        return self.faults.duplicate_next

    @duplicate_next.setter
    def duplicate_next(self, n: int) -> None:
        self.faults.duplicate_next = n

    @property
    def corrupt_next(self) -> int:
        return self.faults.corrupt_next

    @corrupt_next.setter
    def corrupt_next(self, n: int) -> None:
        self.faults.corrupt_next = n

    # ---- PC 側の口 -------------------------------------------------------------------
    def write(self, data: bytes) -> None:
        """PC → 機体。USB を抜いていたり PC が落ちていたら、そのまま消える。"""
        if not self.connected or not self.pc_alive:
            return
        self._queue.extend(self.faults.on_write(data, self._now))
        self.tx_bytes += len(data)

    def read(self) -> bytes:
        """機体 → PC。"""
        if not self.connected:
            self._to_pc.clear()
            return b""
        out = bytes(self._to_pc)
        self._to_pc.clear()
        self.rx_bytes += len(out)
        return out

    def close(self) -> None:
        self.connected = False

    # ---- 機体側を進める ---------------------------------------------------------------
    def pump(self, now: float) -> None:
        """溜まったバイトを機体へ渡し、機体の周期処理を1回まわす。"""
        self._now = now
        ready = [e for e in self._queue if e[0] <= now]     # 遅延・順序入れ替えはここで効く
        self._queue = [e for e in self._queue if e[0] > now]
        for _, chunk in ready:
            self._to_dev += chunk
        take = len(self._to_dev) if self.chunk <= 0 else min(self.chunk, len(self._to_dev))
        data, self._to_dev = bytes(self._to_dev[:take]), self._to_dev[take:]
        out = b""
        # **機体は自分の制御周期で回る**（PC のループ周期に引きずられない）。
        # これをしないと、PC が 50Hz で回すだけで機体が「周期超過」を誤検出する
        dt = float(getattr(self.device, "ctrl_dt", 0.0) or 0.0)
        t = max(self._last_tick, now - MAX_CATCHUP_S)
        while dt > 0.0 and t + dt < now:
            t += dt
            out += self.device.tick(t)
        self._last_tick = t
        if self.connected and data:
            out += self.device.feed(data, now)
        out += self.device.tick(now)
        self._last_tick = now
        if self.connected:
            self._to_pc += out

    # ---- 事故の再現 -------------------------------------------------------------------
    def unplug(self) -> None:
        """USB を抜く（完了条件 3）。機体は動き続けるが、以後の指令は届かない。"""
        self.connected = False
        self._queue.clear()
        self._to_dev.clear()
        self._to_pc.clear()

    def plug(self) -> None:
        """挿し直す。**機体の状態は変わらない**（緊急停止は解除されない。完了条件 9）。"""
        self.connected = True

    def kill_pc(self) -> None:
        """PC のプロセスが落ちる（完了条件 2）。線は繋がったままで送信だけ止まる。"""
        self.pc_alive = False

    def reboot_device(self, now: float) -> None:
        """機体だけ電源が入り直す（完了条件 6）。"""
        self.device.reboot(now)
        self._queue.clear()
        self._to_dev.clear()


class SerialTransport:
    """実機。ESP32-S3 のネイティブ USB CDC（ボーレートは無視されるが pyserial には必要）。

    **未検証**: 実機が無いため、この経路は実際の COM ポートで動かしていない。
    """

    def __init__(self, port: str, baudrate: int = 921600, timeout: float = 0.0) -> None:
        import serial                              # 実機のときだけ要る

        self._ser = serial.Serial(port, baudrate=baudrate, timeout=timeout, write_timeout=0.5)
        self.port = port
        self.connected = True

    def write(self, data: bytes) -> None:
        try:
            self._ser.write(data)
        except Exception:                           # 抜去・切断は「届かなかった」として扱う
            self.connected = False

    def read(self) -> bytes:
        try:
            n = self._ser.in_waiting
            return self._ser.read(n) if n else b""
        except Exception:
            self.connected = False
            return b""

    def close(self) -> None:
        self.connected = False
        try:
            self._ser.close()
        except Exception:
            pass
