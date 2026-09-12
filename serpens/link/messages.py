"""指令・テレメトリの payload（docs/link_protocol.md §2/§3）。

フレーム（protocol.py）と分けてある。ここは struct の詰め替えだけで、範囲検査や状態は持たない。
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

from serpens.link.protocol import (FMT_AXIS, FMT_DRIVE, FMT_HEAD, FMT_STOP, FMT_TELEM_HEAD,
                                   REASON_JA, STATE_JA, Flag, State, StopReason)

FMT_ACK = "<HB"        # ack_seq, type
FMT_NACK = "<HBB"      # ack_seq, type, reason
FMT_EVENT = "<BB"      # code, detail


def pack_ack(seq: int, cmd: int) -> bytes:
    return struct.pack(FMT_ACK, seq, int(cmd))


def unpack_ack(payload: bytes) -> tuple[int, int]:
    return struct.unpack(FMT_ACK, payload)          # type: ignore[return-value]


def pack_nack(seq: int, cmd: int, reason: int) -> bytes:
    return struct.pack(FMT_NACK, seq, int(cmd), int(reason))


def unpack_nack(payload: bytes) -> tuple[int, int, int]:
    return struct.unpack(FMT_NACK, payload)         # type: ignore[return-value]


def pack_stop(mode: int, reason: int) -> bytes:
    return struct.pack(FMT_STOP, int(mode), int(reason))


def unpack_stop(payload: bytes) -> tuple[int, int]:
    return struct.unpack(FMT_STOP, payload)         # type: ignore[return-value]


def pack_nonce(nonce: int) -> bytes:
    """CLEAR_FAULT の nonce（u32）。同じ値の再利用は機体が拒否する。"""
    return struct.pack("<I", nonce & 0xFFFFFFFF)


def unpack_nonce(payload: bytes) -> int:
    return int(struct.unpack("<I", payload)[0])


def pack_event(code: int, detail: int = 0) -> bytes:
    return struct.pack(FMT_EVENT, int(code), int(detail))


def unpack_event(payload: bytes) -> tuple[int, int]:
    return struct.unpack(FMT_EVENT, payload)        # type: ignore[return-value]


@dataclass(frozen=True)
class Drive:
    """歩容の指令（角度は度、周波数は Hz。負の周波数は後退）。"""

    ttl_ms: int
    amplitude_deg: float
    spatial_freq_deg: float
    temporal_freq_hz: float
    gamma_deg: float

    def pack(self) -> bytes:
        return struct.pack(FMT_DRIVE, int(self.ttl_ms), round(self.amplitude_deg * 10),
                           round(self.spatial_freq_deg * 10), round(self.temporal_freq_hz * 1000),
                           round(self.gamma_deg * 10))

    @staticmethod
    def unpack(payload: bytes) -> "Drive":
        ttl, amp, spa, freq, gam = struct.unpack(FMT_DRIVE, payload)
        return Drive(ttl, amp / 10, spa / 10, freq / 1000, gam / 10)


@dataclass(frozen=True)
class Head:
    """頭部の目標角。"""

    ttl_ms: int
    j7_deg: float
    j8_deg: float
    j9_deg: float
    speed_dps: float

    def pack(self) -> bytes:
        return struct.pack(FMT_HEAD, int(self.ttl_ms), round(self.j7_deg * 10), round(self.j8_deg * 10),
                           round(self.j9_deg * 10), round(self.speed_dps * 10))

    @staticmethod
    def unpack(payload: bytes) -> "Head":
        ttl, j7, j8, j9, spd = struct.unpack(FMT_HEAD, payload)
        return Head(ttl, j7 / 10, j8 / 10, j9 / 10, spd / 10)


@dataclass(frozen=True)
class AxisTelemetry:
    """1軸ぶんの状態。current_ma が None なら測れていない。"""

    pos_deg: float
    load: float
    temp_c: int
    volt_v: float
    fault: int
    current_ma: int | None


@dataclass(frozen=True)
class Telemetry:
    """機体の状態一式。"""

    boot_id: int
    uptime_ms: int
    state: State
    stop_reason: StopReason
    last_seq: int
    flags: int
    axes: list[AxisTelemetry]

    @property
    def driving(self) -> bool:
        return bool(self.flags & Flag.DRIVING)

    @property
    def torque_on(self) -> bool:
        return bool(self.flags & Flag.TORQUE_ON)

    @property
    def reason_ja(self) -> str:
        return REASON_JA.get(self.stop_reason, str(self.stop_reason))

    @property
    def state_ja(self) -> str:
        return STATE_JA.get(self.state, str(self.state))

    def pack(self) -> bytes:
        out = struct.pack(FMT_TELEM_HEAD, self.boot_id, self.uptime_ms, int(self.state),
                          int(self.stop_reason), self.last_seq, self.flags, len(self.axes))
        for a in self.axes:
            out += struct.pack(FMT_AXIS, round(a.pos_deg * 10), round(a.load * 1000), int(a.temp_c),
                               round(a.volt_v * 10), a.fault,
                               0xFFFF if a.current_ma is None else min(int(a.current_ma), 0xFFFE))
        return out

    @staticmethod
    def unpack(payload: bytes) -> "Telemetry":
        head = struct.calcsize(FMT_TELEM_HEAD)
        boot, up, state, reason, last_seq, flags, n = struct.unpack_from(FMT_TELEM_HEAD, payload)
        size = struct.calcsize(FMT_AXIS)
        axes = []
        for k in range(n):
            pos, load, temp, volt, fault, cur = struct.unpack_from(FMT_AXIS, payload, head + k * size)
            axes.append(AxisTelemetry(pos / 10, load / 1000, temp, volt / 10, fault,
                                      None if cur == 0xFFFF else cur))
        return Telemetry(boot, up, State(state), StopReason(reason), last_seq, flags, axes)
