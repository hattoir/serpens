"""テスト共通の道具。"""
from __future__ import annotations


class FakeClock:
    """手で進める時計。"""

    def __init__(self, t: float = 0.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


class FakePort:
    """scservo_sdk の PortHandler の代わり。送信バイトを記録し、用意した応答を返す。"""

    def __init__(self, _name: str = "FAKE") -> None:
        self.is_using = False
        self.is_open = False
        self.written: list[list[int]] = []
        self.rx = bytearray()

    def openPort(self) -> bool:  # noqa: N802 - SDK の名前に合わせる
        self.is_open = True
        return True

    def setBaudRate(self, _baud: int) -> bool:  # noqa: N802
        return True

    def closePort(self) -> None:  # noqa: N802
        self.is_open = False

    def clearPort(self) -> None:  # noqa: N802
        pass

    def writePort(self, packet: list[int]) -> int:  # noqa: N802
        self.written.append(list(packet))
        return len(packet)

    def readPort(self, length: int) -> bytes:  # noqa: N802
        out, self.rx = bytes(self.rx[:length]), self.rx[length:]
        return out

    def getBytesAvailable(self) -> int:  # noqa: N802
        return len(self.rx)

    def setPacketTimeout(self, _n: int) -> None:  # noqa: N802
        pass

    def setPacketTimeoutMillis(self, _ms: float) -> None:  # noqa: N802
        pass

    def isPacketTimeout(self) -> bool:  # noqa: N802
        return len(self.rx) == 0

    def queue_hex(self, hex_str: str) -> None:
        """応答バイト列を16進文字列で積む。"""
        self.rx += bytes.fromhex(hex_str)


def hexbytes(hex_str: str) -> list[int]:
    """'FF FF 01' → [255, 255, 1]"""
    return list(bytes.fromhex(hex_str))


def checksum(body: list[int]) -> int:
    """プロトコル資料 [P] §1.1 のチェックサム: ~(ID+LEN+INST+PARAM...) の下位1バイト。"""
    return ~sum(body) & 0xFF
