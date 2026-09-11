"""FeetechServoBus のテスト（実機なし）。

期待値は docs/sts3215_registers.md の出典 [P]（プロトコル資料）の実例パケットから取っている。
"""
from __future__ import annotations

import math

import pytest

from serpens.config import load_config
from serpens.hw.feetech_bus import (
    FeetechServoBus,
    decode_sign_magnitude,
    decode_state_block,
    deg_to_step,
    dps2_to_accel_reg,
    dps_to_step_s,
    ratio_to_torque_limit,
    step_to_deg,
)
from serpens.hw.servo_bus import Goal, JointSpec
from tests.helpers import FakePort, checksum, hexbytes


@pytest.fixture()
def cfg() -> dict:
    return load_config()


@pytest.fixture()
def s(cfg: dict) -> dict:
    return cfg["servo"]


def joint(direction: int = 1, offset: float = 0.0) -> JointSpec:
    return JointSpec("JX", 1, "yaw", 0.0, direction, offset, -180.0, 180.0)


@pytest.fixture()
def bus(cfg: dict) -> tuple[FeetechServoBus, FakePort]:
    ports: list[FakePort] = []

    def factory(name: str) -> FakePort:
        ports.append(FakePort(name))
        return ports[-1]

    b = FeetechServoBus(cfg, "COM_TEST", port_factory=factory)
    b.connect()
    return b, ports[0]


# ---- 単位変換 ------------------------------------------------------------------
def test_deg_step_conversion(s: dict) -> None:
    """中立 2047、4096 step/360°、向き反転とオフセット、範囲クランプ。"""
    assert deg_to_step(0.0, joint(), s) == 2047
    assert deg_to_step(90.0, joint(), s) == 2047 + 1024
    assert deg_to_step(90.0, joint(direction=-1), s) == 2047 - 1024
    assert deg_to_step(0.0, joint(offset=0.087890625), s) == 2048
    assert deg_to_step(270.0, joint(), s) == 4095
    for d in (-45.0, 0.0, 12.3, 90.0):
        for jd in (joint(), joint(-1, 3.0)):
            assert step_to_deg(deg_to_step(d, jd, s), jd, s) == pytest.approx(d, abs=0.05)


def test_speed_and_accel_registers(s: dict) -> None:
    assert dps_to_step_s(360.0, s) == 3400          # 4096 は上限 3400 に丸め
    assert dps_to_step_s(90.0, s) == 1024
    assert dps_to_step_s(0.0, s) == 1               # 0 は書かない
    assert dps2_to_accel_reg(8.7890625 * 10, s) == 10   # [M]: 1 = 100 step/s² = 8.79°/s²
    assert dps2_to_accel_reg(math.inf, s) == 254
    assert dps2_to_accel_reg(0.0, s) == 1


def test_sign_magnitude_and_torque_limit(s: dict) -> None:
    assert decode_sign_magnitude(0x8000 | 100, 15) == -100
    assert decode_sign_magnitude(100, 15) == 100
    assert ratio_to_torque_limit(0.6, s) == 600
    assert ratio_to_torque_limit(1.5, s) == 1000


def test_decode_state_block_protocol_example8(s: dict) -> None:
    """[P] Example 8 の SYNC READ 応答データ（56番地から8バイト）を解釈する。"""
    st1 = decode_state_block(hexbytes("00 08 00 00 00 00 79 1E"), joint(), s)
    assert st1.pos_deg == pytest.approx(360.0 / 4096)     # 2048 step
    assert st1.volt == pytest.approx(12.1)                 # 0x79 = 121 × 0.1V
    assert st1.temp_c == pytest.approx(30.0)               # 0x1E
    st2 = decode_state_block(hexbytes("FF 07 00 00 00 00 77 23"), joint(), s)
    assert st2.pos_deg == pytest.approx(0.0)               # 2047 = 中立
    assert st2.volt == pytest.approx(11.9)
    assert st2.temp_c == pytest.approx(35.0)


# ---- パケット（SDK が資料どおりのバイト列を出すか） -----------------------------
def test_sdk_write_matches_protocol_example4(bus: tuple[FeetechServoBus, FakePort]) -> None:
    """[P] Example 4: ID1 を位置2048・時間0・速度1000 に → FF FF 01 09 03 2A 00 08 00 00 E8 03 D5"""
    b, port = bus
    port.queue_hex("FF FF 01 02 00 FC")
    b._handler().writeTxRx(1, 0x2A, 6, hexbytes("00 08 00 00 E8 03"))
    assert port.written[-1] == hexbytes("FF FF 01 09 03 2A 00 08 00 00 E8 03 D5")


def test_set_goal_packet_layout(bus: tuple[FeetechServoBus, FakePort]) -> None:
    """set_goal → 41番地から [加速度, 位置L, 位置H, 時間L, 時間H, 速度L, 速度H]（リトルエンディアン）。"""
    b, port = bus
    port.queue_hex("FF FF 01 02 00 FC")
    b.set_goal(1, 90.0, 87.890625, 8.7890625 * 20)   # 3071 step, 1000 step/s, 加速度 20
    body = [0x01, 0x0A, 0x03, 0x29, 20, 0xFF, 0x0B, 0x00, 0x00, 0xE8, 0x03]
    assert port.written[-1] == [0xFF, 0xFF, *body, checksum(body)]


def test_sync_set_goals_packet(bus: tuple[FeetechServoBus, FakePort]) -> None:
    """複数軸は SYNC WRITE（ID=0xFE, 0x83, 先頭番地 0x29, 1軸 7 バイト）。"""
    b, port = bus
    b.sync_set_goals({1: Goal(0.0, 87.890625, math.inf), 2: Goal(0.0, 87.890625, math.inf)})
    pkt = port.written[-1]
    assert pkt[:7] == [0xFF, 0xFF, 0xFE, (7 + 1) * 2 + 4, 0x83, 0x29, 7]
    assert pkt[7:15] == [1, 254, 0xFF, 0x07, 0, 0, 0xE8, 0x03]
    assert pkt[15:23] == [2, 254, 0xFF, 0x07, 0, 0, 0xE8, 0x03]
    assert pkt[-1] == checksum(pkt[2:-1])


def test_read_state_via_individual_read(bus: tuple[FeetechServoBus, FakePort]) -> None:
    """READ 56番地 8バイト → [P] Example 8 の ID1 応答パケットを返させて解釈する。"""
    b, port = bus
    port.queue_hex("FF FF 01 0A 00 00 08 00 00 00 00 79 1E 55")
    st = b.read_state(1)
    assert port.written[-1] == [0xFF, 0xFF, 1, 4, 0x02, 0x38, 8, checksum([1, 4, 0x02, 0x38, 8])]
    assert st.volt == pytest.approx(12.1) and st.temp_c == pytest.approx(30.0)


def test_sync_read_falls_back_when_no_reply(bus: tuple[FeetechServoBus, FakePort]) -> None:
    """SYNC READ に誰も応答しなければ、以後は個別 READ に切り替える。"""
    b, port = bus
    states = b.sync_read_states([1, 2])
    assert states == {}
    assert b._use_sync_read is False
    assert port.written[0][4] == 0x82                   # 最初は SYNC READ を送っている
