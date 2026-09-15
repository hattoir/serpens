"""駆動リンクのフレーム形式と指令の定義（docs/link_protocol.md v1）。

  0xA5 0x5A | ver(1) | type(1) | seq(2) | len(1) | payload(len) | crc16(2)

ここは純粋な組み立て・解釈だけを持つ（通信もタイマも持たない）ので、そのまま試験できる。
**PC 側（client.py / device.py）と ESP32 ファームは、この定義だけを根拠に実装する。**
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

SOF = b"\xa5\x5a"
VERSION = 1
MAX_PAYLOAD = 192              # テレメトリ 9軸 = 125 バイト。len は u8 なので 255 まで拡張できる
SEQ_MOD = 1 << 16
SEQ_FORWARD_WINDOW = 4096      # (seq - last) % 65536 がこの範囲なら「進んだ」
FMT_HEADER = "<BBHB"           # ver, type, seq, len（SOF の直後。CRC の対象はここから payload まで）
HEADER_LEN = 5                 # ver(1)+type(1)+seq(2)+len(1)
CRC_LEN = 2
FRAME_OVERHEAD = len(SOF) + HEADER_LEN + CRC_LEN     # payload 以外のバイト数 = 9


class Cmd(IntEnum):
    """PC → 機体。"""

    HEARTBEAT = 0x01
    ARM = 0x02
    DISARM = 0x03
    DRIVE = 0x10
    HEAD = 0x11
    POSE = 0x12
    BREATH = 0x13
    BODY = 0x14        # 胴体の姿勢（とぐろ・鎌首）。歩容中は拒否される
    TORQUE = 0x15      # トルク比（脱力の演出）
    STOP = 0x20
    EMERGENCY = 0x21
    CLEAR_FAULT = 0x22
    LIMITS = 0x30
    PING = 0x40


class Rep(IntEnum):
    """機体 → PC。"""

    TELEMETRY = 0x80
    ACK = 0x81
    NACK = 0x82
    EVENT = 0x83


class Nack(IntEnum):
    """拒否の理由。"""

    BAD_CRC = 1
    BAD_VERSION = 2
    BAD_LENGTH = 3
    STALE_SEQ = 4
    OUT_OF_RANGE = 5
    DISARMED = 6
    LATCHED = 7
    UNKNOWN_CMD = 8
    NO_HEARTBEAT = 9
    NONCE_REUSED = 10
    BUSY = 11          # いまの状態では受け付けられない（歩容中の BODY など）
    STALE_BOOT = 12    # 知らない起動の機体を ARM しようとした（再起動後の自動再開を防ぐ）


class State(IntEnum):
    """機体の状態（docs/link_protocol.md §4）。**ファームと同じ7状態を持つ。**"""

    BOOT = 0               # 電源投入直後。まだ heartbeat を受けていない
    DISARMED = 1           # 待機。指令は受けるが動かない
    ARMED_HOLD = 2         # 走行可だが歩容は無い（現在姿勢を保持）
    DRIVING = 3            # 歩容を生成して動いている
    FAULT_HOLD = 4         # 異常で動きを止めた（トルクは保持）。復帰は DISARMED 経由
    EMERGENCY_LATCHED = 5  # 緊急停止。ラッチ。CLEAR_FAULT でのみ解ける
    TORQUE_DISABLED = 6    # 明示的な脱力（STOP mode=disable）

    @property
    def armed(self) -> bool:
        """走行の指令を受け付ける状態か。"""
        return self in (State.ARMED_HOLD, State.DRIVING)

    @property
    def stopped(self) -> bool:
        """異常で止まっている状態か。"""
        return self in (State.FAULT_HOLD, State.EMERGENCY_LATCHED)


class Source(IntEnum):
    """テレメトリの値の出どころ。**模擬と実測を混ぜないための印。**"""

    SIMULATION = 0
    HARDWARE = 1


class StopReason(IntEnum):
    """停止理由（docs/link_protocol.md §4）。"""

    NONE = 0
    OPERATOR_STOP = 1
    DRIVE_TTL = 2
    HEARTBEAT_LOST = 3
    EMERGENCY_CMD = 4
    OVERHEAT = 5
    OVERCURRENT = 6
    SERVO_FAULT = 7
    OUT_OF_RANGE = 8
    BOOT = 9


class Flag(IntEnum):
    """テレメトリの flags（u16）。"""

    DRIVING = 1 << 0
    BREATHING = 1 << 1
    HEARTBEAT_OK = 1 << 2
    DRIVE_VALID = 1 << 3
    TORQUE_ON = 1 << 4
    ARMED = 1 << 5
    EMERGENCY_LATCHED = 1 << 6
    SIMULATED = 1 << 7          # **この値はシミュレーション由来**（実測ではない）
    SERVO_MISSING = 1 << 8      # 応答しない軸がある
    OVERRUN = 1 << 9            # 制御周期を超えた


class StopMode(IntEnum):
    HOLD = 0
    DISABLE_TORQUE = 1


REASON_JA = {
    StopReason.NONE: "なし",
    StopReason.OPERATOR_STOP: "PC からの停止指令",
    StopReason.DRIVE_TTL: "DRIVE の期限切れ",
    StopReason.HEARTBEAT_LOST: "heartbeat 途絶（USB 断・PC 停止を含む）",
    StopReason.EMERGENCY_CMD: "PC からの緊急停止",
    StopReason.OVERHEAT: "過熱",
    StopReason.OVERCURRENT: "過電流",
    StopReason.SERVO_FAULT: "サーボ異常",
    StopReason.OUT_OF_RANGE: "上限外の指令",
    StopReason.BOOT: "起動直後（未 ARM）",
}
STATE_JA = {State.BOOT: "起動直後（BOOT）", State.DISARMED: "待機（DISARMED）",
            State.ARMED_HOLD: "走行可・保持（ARMED_HOLD）", State.DRIVING: "走行中（DRIVING）",
            State.FAULT_HOLD: "異常で保持（FAULT_HOLD）",
            State.EMERGENCY_LATCHED: "緊急停止（ラッチ）", State.TORQUE_DISABLED: "脱力（TORQUE_DISABLED）"}

# payload の書式（struct）。長さ検査にも使う
FMT_DRIVE = "<Hhhhh"      # ttl_ms, amp 0.1°, spatial 0.1°, freq 0.001Hz(符号), gamma 0.1°
FMT_HEAD = "<HhhhH"       # ttl_ms, j7, j8, j9 (0.1°), speed 0.1°/s
FMT_BODY = "<H6hH"        # ttl_ms, 胴体ヨー 6軸 (0.1°), speed 0.1°/s
FMT_TORQUE = "<H"         # トルク比 0.001（0=不可、1000=100%）
FMT_ARM = "<H"            # ARM が名指しする boot_id（**PC が見ている機体と一致すること**）
FMT_STOP = "<BB"          # mode, reason
# テレメトリ v2。boot_id, uptime_ms, state, stop_reason, last_rx_seq, last_drive_seq, flags,
# heartbeat_age_ms, drive_age_ms, drive_ttl_remaining_ms, loop_period_us, overruns, source, n_axes
FMT_TELEM_HEAD = "<HIBBHHHHHHHHBB"
FMT_AXIS = "<hhhBBBH"     # pos 0.1°, vel 0.1°/s, load 0.001, temp ℃, volt 0.1V, fault, current mA
AGE_MAX_MS = 0xFFFF       # これ以上古い値は飽和させる（u16）
PAYLOAD_LEN = {Cmd.HEARTBEAT: 0, Cmd.ARM: struct.calcsize(FMT_ARM), Cmd.DISARM: 0,
               Cmd.PING: 0,
               Cmd.DRIVE: struct.calcsize(FMT_DRIVE), Cmd.HEAD: struct.calcsize(FMT_HEAD),
               Cmd.BODY: struct.calcsize(FMT_BODY), Cmd.TORQUE: struct.calcsize(FMT_TORQUE),
               Cmd.POSE: 1, Cmd.BREATH: 1, Cmd.STOP: 2, Cmd.EMERGENCY: 1, Cmd.CLEAR_FAULT: 4}


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE（初期値 0xFFFF, 多項式 0x1021）。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(frame_type: int, seq: int, payload: bytes = b"") -> bytes:
    """1フレームを組み立てる。"""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload が長すぎます: {len(payload)} > {MAX_PAYLOAD}")
    body = struct.pack(FMT_HEADER, VERSION, int(frame_type), seq % SEQ_MOD, len(payload)) + payload
    return SOF + body + struct.pack("<H", crc16(body))


@dataclass(frozen=True)
class Frame:
    """受信した1フレーム。"""

    type: int
    seq: int
    payload: bytes
    crc_ok: bool = True
    version: int = VERSION

    @property
    def length_ok(self) -> bool:
        """type に対して payload の長さが合っているか。"""
        want = PAYLOAD_LEN.get(Cmd(self.type)) if self.type in set(Cmd) else None
        return want is None or len(self.payload) == want


class FrameReader:
    """バイト列を少しずつ入れてフレームを取り出す（分割・混入に耐える）。"""

    def __init__(self, max_buffer: int = 4096) -> None:
        self._buf = bytearray()
        self._max = max_buffer
        self.dropped_bytes = 0          # 同期が取れず捨てたバイト数
        self.bad_crc = 0

    def feed(self, data: bytes) -> list[Frame]:
        """受信バイトを足して、取り出せたフレームを返す（CRC 不一致も crc_ok=False で返す）。"""
        self._buf += data
        if len(self._buf) > self._max:                      # 壊れ続けてもメモリを食わない
            self.dropped_bytes += len(self._buf) - self._max
            del self._buf[:-self._max]
        out: list[Frame] = []
        while True:
            i = self._buf.find(SOF)
            if i < 0:
                self.dropped_bytes += max(len(self._buf) - 1, 0)
                del self._buf[:-1]                          # SOF の片側が残るかもしれない
                return out
            if i:
                self.dropped_bytes += i
                del self._buf[:i]
            if len(self._buf) < FRAME_OVERHEAD:
                return out
            ver, ftype, seq, plen = struct.unpack_from(FMT_HEADER, self._buf, len(SOF))
            if plen > MAX_PAYLOAD:
                self.dropped_bytes += 2
                del self._buf[:2]                           # 偽の SOF として捨てる
                continue
            total = FRAME_OVERHEAD + plen
            if len(self._buf) < total:
                return out
            body_start, body_end = len(SOF), len(SOF) + HEADER_LEN + plen
            body = bytes(self._buf[body_start:body_end])
            got = struct.unpack_from("<H", self._buf, body_end)[0]
            ok = got == crc16(body)
            if not ok:
                self.bad_crc += 1
            payload = bytes(self._buf[body_start + HEADER_LEN:body_end])
            out.append(Frame(ftype, seq, payload, ok, ver))
            del self._buf[:total]


def seq_is_forward(seq: int, last_seq: int | None) -> bool:
    """seq が進んでいるか（重複・巻き戻りを弾く）。"""
    if last_seq is None:
        return True
    return 1 <= (seq - last_seq) % SEQ_MOD <= SEQ_FORWARD_WINDOW
