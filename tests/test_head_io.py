"""頭部 I/O のテスト。MockHeadIO の出力と実機用パーサが同じであることを確認する。"""
from __future__ import annotations

import pytest

from serpens.config import load_config
from serpens.hw.head_io import (
    MockHeadIO,
    SensorFrame,
    SerialHeadIO,
    format_eye,
    format_mode,
    format_sensor_line,
    parse_ping_reply,
    parse_sensor_line,
)
from tests.helpers import FakeClock


@pytest.fixture()
def cfg() -> dict:
    return load_config()


@pytest.fixture()
def h(cfg: dict) -> dict:
    return cfg["head_io"]


def parse(line: str, h: dict) -> SensorFrame | None:
    return parse_sensor_line(line, h["tof_max_mm"], h["seq_modulo"])


def test_parse_spec_example(h: dict) -> None:
    assert parse("S,124533,412,0,1,842", h) == SensorFrame(124533, 412, False, True, 842)
    assert parse("S,0,0,1,0,65535\r\n", h) == SensorFrame(0, 0, True, False, 65535)


@pytest.mark.parametrize("line", [
    "", "S", "S,1,2,3", "S,1,2,0,1,2,3", "X,1,2,0,1,2",
    "S,1,4001,0,0,1",       # ToF 範囲外
    "S,1,-5,0,0,1",
    "S,1,100,2,0,1",        # タッチは 0/1 のみ
    "S,1,100,0,0,65536",    # seq 範囲外
    "S,abc,100,0,0,1", "S,1.5,100,0,0,1", "\x00\xff garbage",
])
def test_invalid_lines_are_dropped_silently(line: str, h: dict) -> None:
    assert parse(line, h) is None


def test_roundtrip_and_commands(h: dict) -> None:
    f = SensorFrame(123, 4000, True, True, 7)
    assert parse(format_sensor_line(f), h) == f
    assert format_eye(255, 300, -1, 128) == "E,255,255,0,128\n"
    assert format_mode(2) == "M,2\n"
    assert parse_ping_reply("P,ok,1.2.0") == "1.2.0"
    assert parse_ping_reply("P,ng,1.2.0") is None


def test_mock_generates_10hz_through_same_parser(cfg: dict) -> None:
    """モックは 10Hz で S 行を作り、同じパーサを通って latest に入る。"""
    clock = FakeClock()
    io = MockHeadIO(cfg, clock=clock)
    io.touch_back = True
    io.tof_mm = 380
    clock.advance(1.0)
    io.poll()
    assert io.latest is not None
    assert io.latest.seq == cfg["head_io"]["sensor_rate_hz"]   # 0.0〜1.0 秒で 11 本 → 最後の seq=10
    assert io.latest.tof_mm == 380 and io.latest.touch_back and not io.latest.touch_head
    assert io.dropped_lines == 0
    assert io.link_ok()


def test_link_lost_is_reported_but_not_fatal(cfg: dict) -> None:
    clock = FakeClock()
    io = MockHeadIO(cfg, clock=clock)
    clock.advance(0.5)
    assert io.link_ok()
    io.connected = False
    clock.advance(cfg["head_io"]["link_timeout_s"] + 0.2)
    assert not io.link_ok()
    io.set_eye(1, 2, 3, 4)          # 断線中に送っても例外にならない
    io.connected = True
    clock.advance(0.2)
    assert io.link_ok()


def test_esp32_failsafe_returns_to_breath(cfg: dict) -> None:
    """3秒コマンドが来なければ ESP32 側で目を呼吸モードに戻す。"""
    modes = cfg["head_io"]["eye_modes"]
    failsafe = cfg["head_io"]["esp32_failsafe_s"]
    clock = FakeClock()
    io = MockHeadIO(cfg, clock=clock)
    io.set_mode(modes["on"])
    clock.advance(failsafe - 0.1)
    io.poll()
    assert io.eye_mode == modes["on"]
    clock.advance(0.2)
    io.poll()
    assert io.eye_mode == modes["breath"]


def test_mock_ping_and_eye(cfg: dict) -> None:
    io = MockHeadIO(cfg, clock=FakeClock())
    io.ping()
    assert io.fw_version == cfg["head_io"]["mock"]["fw_version"]
    io.set_eye(10, 20, 30, 40)
    assert io.eye_rgb_brightness == (10, 20, 30, 40)
    assert io.tx_log == ["P\n", "E,10,20,30,40\n"]


def test_serial_head_io_survives_missing_port(cfg: dict) -> None:
    """存在しないポートでも例外で落ちず、断線として報告される。"""
    io = SerialHeadIO(cfg, "COM_DOES_NOT_EXIST")
    try:
        assert not io.link_ok()
        io.set_mode(1)
    finally:
        io.close()
