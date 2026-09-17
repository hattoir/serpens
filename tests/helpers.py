"""テスト共通の道具。"""
from __future__ import annotations

from serpens.hw.mock_bus import MockServoBus
from serpens.hw.servo_bus import ServoCommError


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


class RecordingBus(MockServoBus):
    """モックの上に「何を送ったか」の記録を足したバス。

    実機バスの代わりに注入して、停止指令が出力先まで届いたか・脱力したかを数える。
    fail_after を指定すると、その回数を超えた読み出しで通信エラーを起こす（欠損・鮮度の試験用）。
    """

    def __init__(self, cfg: dict, clock=None, fail_after: int | None = None) -> None:  # type: ignore[no-untyped-def]
        super().__init__(cfg, clock=clock)
        self.goal_calls: list[dict[int, float]] = []
        self.torque_calls: list[tuple[int, bool]] = []
        self.read_calls = 0
        self.id_changes: list[tuple[int, int]] = []
        self.scans: list[list[int]] = []
        self.disconnects: list[bool] = []
        self.fail_after = fail_after

    def _set_goals(self, goals):  # type: ignore[no-untyped-def]
        self.goal_calls.append({sid: g.deg for sid, g in goals.items()})
        super()._set_goals(goals)

    def set_torque(self, servo_id: int, on: bool) -> None:
        self.torque_calls.append((servo_id, on))
        super().set_torque(servo_id, on)

    def _read_states(self, ids):  # type: ignore[no-untyped-def]
        self.read_calls += 1
        if self.fail_after is not None and self.read_calls > self.fail_after:
            raise ServoCommError("テスト: 応答なし")
        return super()._read_states(ids)

    def set_servo_id(self, old_id: int, new_id: int) -> bool:
        self.id_changes.append((old_id, new_id))
        return super().set_servo_id(old_id, new_id)

    def scan(self, ids=None):  # type: ignore[no-untyped-def]
        self.scans.append(list(ids) if ids is not None else list(self.ids))
        return super().scan(ids)

    def disconnect(self, torque_off: bool = True) -> None:
        self.disconnects.append(torque_off)
        super().disconnect(torque_off=torque_off)

    @property
    def last_pose(self) -> dict[int, float]:
        return self.goal_calls[-1] if self.goal_calls else {}


def steady_patrol(cfg: dict) -> dict:
    """巡回の stop-and-go を切った設定の写し（静止 0 秒 = 歩き続ける）。

    リンク・Vision など**歩容が出ている最中の性質**を確かめる試験用。
    間欠移動そのものは tests/test_behavior_sim.py で確かめる。
    """
    import copy

    out = copy.deepcopy(cfg)
    out["behavior"]["controller"]["patrol_pause_s"] = [0.0, 0.0]
    return out
