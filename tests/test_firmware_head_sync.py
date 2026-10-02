"""頭の XIAO ファームの骨組み（`firmware/serpens_head_xiao/`）と Python（`serpens/hw/head_io.py`・config）の値・行の形の一致（LB-E-082）。

**C++ としては実行していない**（ホストに C++ コンパイラが無い）。ここで確かめるのは、ヘッダの**文字列と定数**が Python の写しになっていること。
ESP32 向けのコンパイルは `tools/build_firmware.py --sketch firmware/serpens_head_xiao`。HARDWARE_UNVERIFIED。
"""
from __future__ import annotations

import re
from pathlib import Path

import yaml

from serpens.hw.head_io import SensorFrame, format_sensor_line, parse_sensor_line

ROOT = Path(__file__).resolve().parent.parent
FW = ROOT / "firmware" / "serpens_head_xiao"
CFG = yaml.safe_load((ROOT / "config" / "robot.yaml").read_text(encoding="utf-8"))["head_io"]


def _define(name: str) -> str:
    m = re.search(rf"^#define\s+{name}\s+(\S+)", (FW / "head_config.h").read_text(encoding="utf-8"), re.M)
    assert m, name
    return m.group(1)


def test_constants_match_head_io_config():
    assert int(_define("HEAD_BAUD")) == int(CFG["baudrate"])
    assert int(_define("HEAD_SENSOR_HZ")) == int(CFG["sensor_rate_hz"])
    assert int(_define("HEAD_SEQ_MODULO")) == int(CFG["seq_modulo"])


def test_sensor_line_format_is_the_python_format():
    text = (FW / "head_logic.h").read_text(encoding="utf-8")
    m = re.search(r'snprintf\(buf, n, "(S,[^"]+)"', text)
    assert m
    c_fmt = m.group(1).replace("\\n", "\n")
    py = format_sensor_line(SensorFrame(1234, 567, True, False, 89))
    # C の書式（%lu / %d）に同じ値を入れて Python の出力と一致する
    filled = c_fmt.replace("%lu", "{}", 1).replace("%d", "{}", 1).replace("%d", "{}", 1).replace("%d", "{}", 1).replace("%lu", "{}", 1).format(1234, 567, 1, 0, 89)
    assert filled == py
    assert parse_sensor_line(filled, int(CFG["tof_max_mm"]), int(CFG["seq_modulo"])) is not None


def test_ping_reply_format():
    assert '"P,ok,%s\\n"' in (FW / "head_logic.h").read_text(encoding="utf-8")


def test_gpio_assignment_is_k0001():
    # K-0001 (1): D0 XSHUT_L / D1 XSHUT_R / D2 LED_L / D3 LED_R / D4 SDA / D5 SCL（User 承認 2026-10-01）
    want = {"HEAD_PIN_XSHUT_L": "D0", "HEAD_PIN_XSHUT_R": "D1", "HEAD_PIN_LED_L": "D2", "HEAD_PIN_LED_R": "D3", "HEAD_PIN_SDA": "D4", "HEAD_PIN_SCL": "D5"}
    for k, v in want.items():
        assert _define(k) == v, k
    # ピンが無いもの（タッチ・フードのビット）は使わない（-1）。勝手にピンを決めない
    for k in ("HEAD_PIN_TOUCH_HEAD", "HEAD_PIN_TOUCH_BACK", "HEAD_PIN_HOOD_DOWN", "HEAD_PIN_HOOD_UP"):
        assert _define(k) == "-1", k


def test_tof_reference_registers_are_the_datasheet_ones():
    assert int(_define("TOF_REG_MODEL_ID"), 16) == 0x010F and int(_define("TOF_MODEL_ID"), 16) == 0xEA and int(_define("TOF_MODULE_TYPE"), 16) == 0xCC
    assert int(_define("TOF_ADDR_DEFAULT"), 16) == 0x29                      # 0x52（8 bit 表記）の 7 bit
    assert int(_define("LED_PWM_HZ")) > 20000                                 # 可聴帯の外


def test_led_is_off_at_startup_and_command_is_clamped():
    ino = (FW / "serpens_head_xiao.ino").read_text(encoding="utf-8")
    setup = ino[ino.index("void setup()"):ino.index("void loop()")]
    assert "ledWrite(0, 0)" in setup and "ledWrite(1, 0)" in setup            # 起動時は消灯
    assert "hlLedDuty(c.a, LED_DUTY_MAX)" in ino and "hlLedDuty(c.b, LED_DUTY_MAX)" in ino
