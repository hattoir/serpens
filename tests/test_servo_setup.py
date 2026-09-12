"""STEP 8: 実機用ツール（tools/servo_setup.py）とレジスタ層のテスト。

実機はまだ無いので、すべてモックバス相手に確認する。
"""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

from serpens.config import load_config
from serpens.hw import registers as reg
from serpens.hw.mock_bus import MockServoBus
from tests.helpers import FakeClock

ROOT = Path(__file__).resolve().parent.parent
TOOL = ROOT / "tools" / "servo_setup.py"


@pytest.fixture()
def cfg() -> dict:
    return load_config()


@pytest.fixture()
def clock() -> FakeClock:
    return FakeClock()


@pytest.fixture()
def bus(cfg: dict, clock: FakeClock) -> MockServoBus:
    b = MockServoBus(cfg, clock=clock)
    b.connect()
    return b


def run_tool(script: str) -> str:
    """メニューへの入力を流し込んでツールを動かし、出力を返す。"""
    env = {**os.environ, "PYTHONIOENCODING": "utf-8", "PYTHONPATH": str(ROOT)}
    p = subprocess.run([sys.executable, str(TOOL)], input=script, capture_output=True,
                       text=True, encoding="utf-8", env=env, cwd=ROOT, timeout=180)
    assert p.returncode == 0, p.stderr
    return p.stdout


# ---- レジスタ定義 ------------------------------------------------------------------
def test_register_definitions(cfg: dict) -> None:
    assert reg.MAX_VOLTAGE.describe(80) == "8.0V"          # 資料どおりの初期値（12V の罠）
    assert reg.MAX_TEMP.describe(70) == "70℃"
    assert reg.status_text(0) == "正常"
    assert "電圧" in reg.status_text(0b1) and "過負荷" in reg.status_text(0b100000)
    assert reg.MAX_VOLTAGE.addr in reg.EEPROM_ADDRS and reg.PRESENT_LOAD.addr not in reg.EEPROM_ADDRS


# ---- モックのレジスタ --------------------------------------------------------------
def test_mock_defaults_reproduce_the_8v_trap(bus: MockServoBus) -> None:
    """初期値は資料どおり 8.0V。12V を入れると動かない、という罠を再現する。"""
    assert bus.read_register(1, reg.MAX_VOLTAGE.addr, 1) == 80
    assert bus.read_register(1, reg.MAX_TEMP.addr, 1) == 70


def test_mock_eeprom_needs_unlock(bus: MockServoBus) -> None:
    """EEPROM はロックを外さないと保存されない（実機と同じ失敗をする）。"""
    assert not bus.write_register(1, reg.MAX_VOLTAGE.addr, 1, 140)          # 手順を踏まない書き込み
    assert bus.read_register(1, reg.MAX_VOLTAGE.addr, 1) == 80
    assert bus.write_register(1, reg.MAX_VOLTAGE.addr, 1, 140, eeprom=True)  # 正しい手順
    assert bus.read_register(1, reg.MAX_VOLTAGE.addr, 1) == 140


def test_mock_id_change_and_scan(bus: MockServoBus) -> None:
    assert bus.scan([1, 2, 99]) == [1, 2]
    assert bus.set_servo_id(9, 12)
    assert bus.ping(12) and not bus.ping(9)
    assert bus.read_register(12, reg.ID.addr, 1) == 12
    assert not bus.set_servo_id(12, 1)          # 既にある ID にはしない


def test_mock_load_sign_bit_follows_hypothesis(bus: MockServoBus, cfg: dict, clock: FakeClock) -> None:
    """負荷の生値は「下位10bit = 大きさ、bit10 = 方向」で作られる（符号ビット検証モードの練習用）。"""
    bit = int(cfg["servo"]["load_sign_bit"])
    bus.set_torque(1, True)
    bus.set_external_load(1, -0.5)
    clock.advance(0.05)                      # モックは経過時間ぶんだけ積分する
    raw = bus.read_register(1, reg.PRESENT_LOAD.addr, 2)
    assert raw & (1 << bit)
    assert (raw & ((1 << bit) - 1)) == pytest.approx(500, abs=5)
    bus.set_external_load(1, 0.5)
    clock.advance(0.05)
    assert not (bus.read_register(1, reg.PRESENT_LOAD.addr, 2) or 0) & (1 << bit)


def test_mock_supports_sync_read(bus: MockServoBus) -> None:
    assert bus.supports_sync_read()


# ---- ツール本体（モック相手に全メニュー） ----------------------------------------------
def test_tool_scan_and_voltage_menu() -> None:
    out = run_tool("2\n1-9\n3\n14.0\n4.5\n0\n")
    assert "応答: [1, 2, 3, 4, 5, 6, 7, 8, 9]" in out
    assert "最高入力電圧 8.0V < 電源 12.0V" in out          # 罠の警告が出る
    assert "PING に応答するのに動かない場合は、まずここを疑ってください" in out
    assert out.count("最高入力電圧 → 14.0V　OK") == 9
    assert out.count("最低入力電圧 → 4.5V　OK") == 9


def test_tool_sign_check_sync_read_and_hold() -> None:
    out = run_tool("6\n1\n7\n8\n\n0\n")
    assert "bit10 が方向を表しています" in out
    assert "SYNC READ: 対応しています" in out
    assert "全軸を中立" in out and "トルクを切りました" in out
    assert "切断しました" in out


def test_tool_id_assignment_can_be_skipped() -> None:
    out = run_tool("4\nq\n" + "s\n" * 9 + "0\n")
    assert "ID の一括設定" in out or "順に振ります" in out
