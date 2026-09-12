"""Phase 2: フレーム形式（docs/link_protocol.md §1）のテスト。

分割・混入・CRC 破損・重複 seq に耐えることを、通信路なしで確かめる。
"""
from __future__ import annotations

import pytest

from serpens.link import messages as m
from serpens.link.protocol import (FRAME_OVERHEAD, MAX_PAYLOAD, SEQ_FORWARD_WINDOW, SEQ_MOD, Cmd,
                                   FrameReader, Rep, State, StopReason, crc16, encode,
                                   seq_is_forward)

DRIVE = m.Drive(300, 30.0, 60.0, 0.5, -20.0)


def test_crc_is_ccitt_false() -> None:
    """CRC-16/CCITT-FALSE の既知値（"123456789" → 0x29B1）。"""
    assert crc16(b"123456789") == 0x29B1


def test_roundtrip() -> None:
    frame = encode(Cmd.DRIVE, 7, DRIVE.pack())
    assert len(frame) == FRAME_OVERHEAD + len(DRIVE.pack())
    fr = FrameReader().feed(frame)[0]
    assert fr.type == Cmd.DRIVE and fr.seq == 7 and fr.crc_ok and fr.length_ok
    assert m.Drive.unpack(fr.payload) == DRIVE


def test_split_and_garbage() -> None:
    """途中で切れても、前にゴミが付いても復帰する。"""
    frame = encode(Cmd.DRIVE, 1, DRIVE.pack())
    rd = FrameReader()
    assert rd.feed(b"\x00\xff\xa5" + frame[:5]) == []      # 偽 SOF + 途中まで
    out = rd.feed(frame[5:])
    assert len(out) == 1 and out[0].crc_ok
    assert rd.dropped_bytes == 3


def test_bad_crc_and_length() -> None:
    bad = bytearray(encode(Cmd.DRIVE, 2, DRIVE.pack()))
    bad[-1] ^= 0xFF
    rd = FrameReader()
    fr = rd.feed(bytes(bad))[0]
    assert not fr.crc_ok and rd.bad_crc == 1
    short = FrameReader().feed(encode(Cmd.DRIVE, 3, b"\x00\x01"))[0]
    assert short.crc_ok and not short.length_ok            # CRC は合うが型に対して短い


def test_two_frames_and_oversize() -> None:
    rd = FrameReader()
    out = rd.feed(encode(Cmd.HEARTBEAT, 1) + encode(Cmd.PING, 2))
    assert [f.type for f in out] == [Cmd.HEARTBEAT, Cmd.PING]
    with pytest.raises(ValueError):
        encode(Cmd.DRIVE, 4, b"\x00" * (MAX_PAYLOAD + 1))


def test_seq_forward() -> None:
    """進んだものだけ受け付ける（重複・巻き戻りは拒否）。"""
    assert seq_is_forward(1, None) and seq_is_forward(9, 8)
    assert not seq_is_forward(8, 8)                        # 重複
    assert not seq_is_forward(7, 8)                        # 巻き戻り
    assert seq_is_forward(3, SEQ_MOD - 2)                  # 65534 → 3（巡回）
    assert not seq_is_forward(8 + SEQ_FORWARD_WINDOW + 1, 8)   # 飛びすぎは拒否


def test_payload_roundtrips() -> None:
    head = m.Head(300, 60.0, -20.0, 5.0, 90.0)
    assert m.Head.unpack(head.pack()) == head
    assert m.unpack_stop(m.pack_stop(1, 4)) == (1, 4)
    assert m.unpack_nonce(m.pack_nonce(0xDEADBEEF)) == 0xDEADBEEF
    assert m.unpack_nack(m.pack_nack(9, Cmd.DRIVE, 5)) == (9, int(Cmd.DRIVE), 5)


def test_telemetry_roundtrip() -> None:
    axes = [m.AxisTelemetry(1.5 * k, 0.02 * k, 30 + k, 11.8, 0, None) for k in range(9)]
    tel = m.Telemetry(7, 123456, State.ARMED, StopReason.DRIVE_TTL, 42, 0b10101, axes)
    payload = tel.pack()
    assert len(payload) == 93 <= MAX_PAYLOAD
    got = m.Telemetry.unpack(FrameReader().feed(encode(Rep.TELEMETRY, 0, payload))[0].payload)
    assert got.boot_id == 7 and got.state is State.ARMED and len(got.axes) == 9
    assert got.axes[8].pos_deg == pytest.approx(12.0) and got.axes[8].current_ma is None
    assert "期限切れ" in got.reason_ja and "走行可" in got.state_ja
