"""firmware のサーボ層の正解のバイト列（`servo_bus_vectors.h`）の試験（SE-E9a、LB-E-016 の後始末）。

ヘッダは `tools/gen_servo_vectors.py` がベンダー SDK（scservo_sdk）の送信バイト列から生成する。ここでは
  (1) ヘッダが SDK の出力と一致している（古くなっていない）、
  (2) ヘッダの各パケットが、プロトコルの式（長さ・チェックサム）だけで独立に組んだものと一致する、
  (3) トルク制限のレジスタ値が config の比（robot.yaml の software_torque_limit_ratio）と一致する、
を確かめる。**C++ としては実行していない**（ホストに C++ コンパイラが無い。ESP32 向けのコンパイルのみ）。HARDWARE_UNVERIFIED。
"""
from __future__ import annotations

import importlib.util
import re
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parent.parent
HEADER = ROOT / "firmware" / "serpens_esp32" / "servo_bus_vectors.h"


def _gen():
    spec = importlib.util.spec_from_file_location("gen_servo_vectors", ROOT / "tools" / "gen_servo_vectors.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _header_arrays() -> dict[str, list[int]]:
    text = HEADER.read_text(encoding="utf-8")
    out = {}
    for name, body in re.findall(r"static const uint8_t SBV_(\w+)\[\] = \{([^}]*)\};", text):
        out[name] = [int(x, 16) for x in body.replace(" ", "").split(",") if x]
    return out


def _defines() -> dict[str, int]:
    return {k: int(v) for k, v in re.findall(r"#define (SBV_\w+) (\d+)", HEADER.read_text(encoding="utf-8"))}


def _packet(sid: int, instr: int, params: list[int]) -> list[int]:
    body = [sid, len(params) + 2, instr] + params
    return [0xFF, 0xFF] + body + [(~sum(body)) & 0xFF]


def test_header_matches_vendor_sdk():
    pytest.importorskip("scservo_sdk")
    g = _gen()
    assert g.render(g.sdk_vectors()) .replace("\r\n", "\n") == HEADER.read_text(encoding="utf-8").replace("\r\n", "\n")


def test_packets_match_protocol_formula():
    a, d = _header_arrays(), _defines()
    assert a["WRITE1"] == _packet(d["SBV_W1_ID"], 0x03, [d["SBV_W1_ADDR"], d["SBV_W1_VAL"]])
    v2 = d["SBV_W2_VAL"]
    assert a["WRITE2"] == _packet(d["SBV_W2_ID"], 0x03, [d["SBV_W2_ADDR"], v2 & 0xFF, (v2 >> 8) & 0xFF])
    assert a["READ_STATE"] == _packet(d["SBV_R_ID"], 0x02, [d["SBV_R_ADDR"], d["SBV_R_LEN"]])


def test_status_packet_checksum_and_data():
    a, d = _header_arrays(), _defines()
    pkt, data = a["STATUS"], a["STATUS_DATA"]
    assert pkt[:2] == [0xFF, 0xFF] and pkt[2] == d["SBV_STATUS_ID"]
    assert pkt[3] == len(data) + 2 and pkt[4] == 0
    assert pkt[5:-1] == data
    assert pkt[-1] == (~sum(pkt[2:-1])) & 0xFF


def test_sync_write_layout_and_checksum():
    a, d = _header_arrays(), _defines()
    pkt = a["SYNC_WRITE"]
    n = d["SBV_SYNC_N"]
    assert pkt[:2] == [0xFF, 0xFF] and pkt[2] == 0xFE and pkt[4] == 0x83      # broadcast + SYNC_WRITE
    assert pkt[5] == 41 and pkt[6] == 7                                       # 先頭アドレス（加速度）、1 軸のデータ長
    assert pkt[3] == (7 + 1) * n + 4
    assert pkt[-1] == (~sum(pkt[2:-1])) & 0xFF
    assert len(pkt) == pkt[3] + 4


def test_torque_limit_register_matches_config_ratio():
    d = _defines()
    cfg = yaml.safe_load((ROOT / "config" / "robot.yaml").read_text(encoding="utf-8"))

    def find(node):
        if isinstance(node, dict):
            if "software_torque_limit_ratio" in node:
                return node["software_torque_limit_ratio"]
            for v in node.values():
                r = find(v)
                if r is not None:
                    return r
        return None

    ratio = find(cfg)
    assert ratio is not None
    assert d["SBV_W2_VAL"] == round(ratio * 1000)
