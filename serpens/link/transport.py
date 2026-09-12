"""駆動リンクの経路。実シリアル（pyserial）と、試験用の偽経路。

偽経路は **同じ SimulatedDevice を相手にする** ので、PC 側の実装を一切変えずに
「USB を抜く」「PC を強制終了する」「フレームを壊す・重複させる」「機体を再起動する」を再現できる
（完了条件 2/3/6/7）。実機での実測は tools/link_check.py と docs/phase2_acceptance.md。
"""
from __future__ import annotations

from typing import Any, Protocol


class Transport(Protocol):
    """PC 側から見た経路。読みは非ブロッキング（無ければ空）。"""

    connected: bool

    def write(self, data: bytes) -> None: ...

    def read(self) -> bytes: ...

    def close(self) -> None: ...


class LoopbackTransport:
    """偽経路。`pump(now)` を呼ぶまで機体は動かないので、時間を完全に制御できる。"""

    def __init__(self, device: Any, chunk: int = 0) -> None:
        self.device = device
        self.chunk = chunk              # 0 = 一度に届く。>0 ならこのバイト数ずつ刻んで届く
        self.connected = True
        self.drop_next = 0              # 次の n フレームぶんの書き込みを捨てる
        self.duplicate_next = 0         # 次の n 回の書き込みを二重に届ける
        self.corrupt_next = 0           # 次の n 回の書き込みの末尾1バイトを壊す
        self.pc_alive = True            # False = PC のプロセスが落ちた（送信が止まる）
        self._to_dev = bytearray()
        self._to_pc = bytearray()
        self.tx_bytes = 0
        self.rx_bytes = 0

    # ---- PC 側の口 -------------------------------------------------------------------
    def write(self, data: bytes) -> None:
        """PC → 機体。USB を抜いていたり PC が落ちていたら、そのまま消える。"""
        if not self.connected or not self.pc_alive:
            return
        if self.drop_next > 0:
            self.drop_next -= 1
            return
        if self.corrupt_next > 0:
            self.corrupt_next -= 1
            data = data[:-1] + bytes([data[-1] ^ 0xFF])
        self._to_dev += data
        if self.duplicate_next > 0:
            self.duplicate_next -= 1
            self._to_dev += data
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
        take = len(self._to_dev) if self.chunk <= 0 else min(self.chunk, len(self._to_dev))
        data, self._to_dev = bytes(self._to_dev[:take]), self._to_dev[take:]
        out = b""
        if self.connected and data:
            out += self.device.feed(data, now)
        out += self.device.tick(now)
        if self.connected:
            self._to_pc += out

    # ---- 事故の再現 -------------------------------------------------------------------
    def unplug(self) -> None:
        """USB を抜く（完了条件 3）。機体は動き続けるが、以後の指令は届かない。"""
        self.connected = False
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
