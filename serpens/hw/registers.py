"""STS3215 のレジスタ定義（出典は docs/sts3215_registers.md）。

モックと実機の両方がここを参照する。番地・バイト数・初期値は資料どおりで、推測は入れていない。
"""
from __future__ import annotations

from dataclasses import dataclass

LOCK_ADDR = 55          # 0 を書くと EEPROM への書き込みが保存される / 1 で保護
TORQUE_ADDR = 40


@dataclass(frozen=True)
class Register:
    """1つのレジスタ。"""

    addr: int
    size: int
    name: str
    unit: str = ""
    default: int | None = None
    eeprom: bool = False

    def describe(self, raw: int) -> str:
        """人が読む形にする（0.1V 単位などを展開）。"""
        if self.unit == "0.1V":
            return f"{raw * 0.1:.1f}V"
        if self.unit == "℃":
            return f"{raw}℃"
        return f"{raw}{self.unit}"


# 実機ツールで使うものだけ（docs/sts3215_registers.md §4）
FIRMWARE_MAJOR = Register(0, 1, "ファームウェア主版", eeprom=True)
MODEL = Register(3, 2, "サーボ型番", eeprom=True)
ID = Register(5, 1, "ID", default=1, eeprom=True)
BAUD_RATE = Register(6, 1, "ボーレート番号", default=0, eeprom=True)
MIN_ANGLE = Register(9, 2, "最小角度制限", "step", 0, eeprom=True)
MAX_ANGLE = Register(11, 2, "最大角度制限", "step", 4095, eeprom=True)
MAX_TEMP = Register(13, 1, "最高温度", "℃", 70, eeprom=True)
MAX_VOLTAGE = Register(14, 1, "最高入力電圧", "0.1V", 80, eeprom=True)
MIN_VOLTAGE = Register(15, 1, "最低入力電圧", "0.1V", 40, eeprom=True)
MAX_TORQUE = Register(16, 2, "最大トルク", "", 1000, eeprom=True)
MODE = Register(33, 1, "動作モード", "", 0, eeprom=True)
TORQUE_ENABLE = Register(TORQUE_ADDR, 1, "トルクスイッチ", "", 0)
GOAL_POSITION = Register(42, 2, "目標位置", "step")
PRESENT_POSITION = Register(56, 2, "現在位置", "step")
PRESENT_SPEED = Register(58, 2, "現在速度", "step/s")
PRESENT_LOAD = Register(60, 2, "現在負荷", "")
PRESENT_VOLTAGE = Register(62, 1, "現在電圧", "0.1V")
PRESENT_TEMPERATURE = Register(63, 1, "現在温度", "℃")
STATUS = Register(65, 1, "サーボ状態", "")
MOVING = Register(66, 1, "移動中", "")

# 起動時に確認する EEPROM（README「トラブルシュート」の順）
STARTUP_CHECK = (MAX_TEMP, MAX_VOLTAGE, MIN_VOLTAGE, MAX_TORQUE, MODE, MIN_ANGLE, MAX_ANGLE)
# サーボ状態のビット（65番地）
STATUS_BITS = ("電圧", "センサ", "温度", "電流", "角度", "過負荷")


def status_text(raw: int) -> str:
    """サーボ状態のビットを日本語にする。"""
    bad = [n for i, n in enumerate(STATUS_BITS) if raw & (1 << i)]
    return "正常" if not bad else "エラー: " + " / ".join(bad)


# EEPROM 領域の番地（ロックを外さないと保存されない）
EEPROM_ADDRS = frozenset(r.addr for r in (FIRMWARE_MAJOR, MODEL, ID, BAUD_RATE, MIN_ANGLE, MAX_ANGLE,
                                          MAX_TEMP, MAX_VOLTAGE, MIN_VOLTAGE, MAX_TORQUE, MODE))
