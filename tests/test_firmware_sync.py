"""ファームウェア（C++）と Python 実装が**同じ値**を持っていることを照合する。

このリポジトリには C++ コンパイラが無いので、ファームは**コンパイルできない**
（`firmware/serpens_esp32/README.md`: HARDWARE_UNVERIFIED）。
せめて「三箇所（config/robot.yaml / serpens/link/protocol.py / firmware/*.h）が食い違っていない」
ことだけは機械的に確かめる。**ここがズレると実機で必ず事故になる。**
"""
from __future__ import annotations

import re
from pathlib import Path

import pytest

from serpens.config import load_config
from serpens.link.protocol import (MAX_PAYLOAD, PAYLOAD_LEN, SEQ_FORWARD_WINDOW, VERSION, Cmd, Flag,
                                   Nack, Rep, State, StopReason)

FW = Path(__file__).resolve().parent.parent / "firmware" / "serpens_esp32"


def source(name: str) -> str:
    return (FW / name).read_text(encoding="utf-8")


def enum_values(text: str, prefix: str) -> dict[str, int]:
    """`PREFIX_NAME = 値` を拾う（1 << n の形も解く）。"""
    out: dict[str, int] = {}
    for name, expr in re.findall(rf"\b{prefix}_([A-Z0-9_]+)\s*=\s*([^,;}}]+)", text):
        e = expr.strip()
        m = re.fullmatch(r"1\s*<<\s*(\d+)", e)
        out[name] = (1 << int(m.group(1))) if m else int(e, 0)
    return out


def const_value(text: str, name: str) -> int:
    m = re.search(rf"\b{name}\s*=\s*([0-9xXa-fA-F]+)", text)
    assert m, f"{name} がファームに無い"
    return int(m.group(1), 0)


@pytest.fixture()
def link_h() -> str:
    return source("link.h")


@pytest.fixture()
def config_h() -> str:
    return source("config.h")


# ---- プロトコル -----------------------------------------------------------------------
def test_frame_constants_match(link_h: str) -> None:
    assert const_value(link_h, "LINK_VERSION") == VERSION
    assert const_value(link_h, "MAX_PAYLOAD") == MAX_PAYLOAD
    assert const_value(link_h, "SEQ_FORWARD_WINDOW") == SEQ_FORWARD_WINDOW
    assert const_value(link_h, "SOF0") == 0xA5 and const_value(link_h, "SOF1") == 0x5A


def test_command_and_reply_codes_match(link_h: str) -> None:
    fw = enum_values(link_h, "CMD")
    for cmd in Cmd:
        assert fw.get(cmd.name) == int(cmd), f"{cmd.name}: ファーム {fw.get(cmd.name)} ≠ {int(cmd)}"
    rep = enum_values(link_h, "REP")
    for r in Rep:
        assert rep.get(r.name) == int(r), r.name


def test_state_machine_matches(link_h: str) -> None:
    """**7状態の値が一致していること。** ここがズレると停止状態を取り違える。"""
    fw = enum_values(link_h, "ST")
    for st in State:
        assert fw.get(st.name) == int(st), f"{st.name}: ファーム {fw.get(st.name)} ≠ {int(st)}"


def test_stop_reasons_and_nacks_match(link_h: str) -> None:
    fw = enum_values(link_h, "SR")
    for r in StopReason:
        assert fw.get(r.name) == int(r), r.name
    nack = enum_values(link_h, "NACK")
    for n in Nack:
        assert nack.get(n.name) == int(n), n.name


def test_flags_match(link_h: str) -> None:
    fw = enum_values(link_h, "FL")
    for f in Flag:
        assert fw.get(f.name) == int(f), f"{f.name}: ファーム {fw.get(f.name)} ≠ {int(f)}"


def test_payload_lengths_match(link_h: str) -> None:
    """type ごとの payload 長。ズレると相手のフレームを取りこぼす。"""
    block = link_h[link_h.index("payloadLenFor"):]
    block = block[:block.index("\n}")]
    fw: dict[str, int] = {}
    for line in block.splitlines():
        names = re.findall(r"CMD_([A-Z_]+):", line)
        m = re.search(r"return\s+(\d+)", line)
        if names and m:
            for n in names:
                fw[n] = int(m.group(1))
    for cmd, want in PAYLOAD_LEN.items():
        assert fw.get(cmd.name) == want, f"{cmd.name}: ファーム {fw.get(cmd.name)} ≠ {want}"


# ---- 機体の設定 -----------------------------------------------------------------------
def test_timings_match_config(config_h: str) -> None:
    cfg = load_config()["link"]
    assert const_value(config_h, "HEARTBEAT_TIMEOUT_MS") == cfg["heartbeat_timeout_ms"]
    assert const_value(config_h, "DRIVE_TTL_MAX_MS") == cfg["drive_ttl_max_ms"]
    assert const_value(config_h, "DRIVE_DISARM_MS") == cfg["drive_disarm_ms"]
    assert const_value(config_h, "CONTROL_PERIOD_MS") == round(1000 / cfg["control_hz"])
    assert const_value(config_h, "TELEMETRY_PERIOD_MS") == round(1000 / cfg["telemetry_hz"])
    assert const_value(config_h, "FAULT_TEMP_LIMIT_C") == cfg["faults"]["temp_limit_c"]


def test_command_limits_match_config(config_h: str) -> None:
    lim = load_config()["link"]["limits"]
    pairs = {"LIMIT_AMPLITUDE_DEG": "amplitude_deg", "LIMIT_SPATIAL_DEG": "spatial_freq_deg",
             "LIMIT_TEMPORAL_HZ": "temporal_freq_hz", "LIMIT_GAMMA_DEG": "gamma_deg",
             "LIMIT_HEAD_SPEED_DPS": "head_speed_dps", "LIMIT_BODY_SPEED_DPS": "body_speed_dps",
             "LIMIT_YAW_SUM_DEG": "yaw_sum_deg"}
    for c_name, key in pairs.items():
        m = re.search(rf"{c_name}\s*=\s*([0-9.]+)f", config_h)
        assert m, f"{c_name} がファームに無い"
        assert float(m.group(1)) == pytest.approx(float(lim[key])), c_name


def test_joint_table_matches_config(config_h: str) -> None:
    """**可動域の写し間違いを見つける。** ファームの JOINTS[] は operational limit を持つ。"""
    cfg = load_config()
    rows = re.findall(r"\{\s*(\d+)\s*,\s*(-?[\d.]+)f\s*,\s*(-?[\d.]+)f\s*,\s*(-?[\d.]+)f", config_h)
    assert len(rows) == len(cfg["joints"]), f"軸数が違う: ファーム {len(rows)}"
    for row, j in zip(rows, cfg["joints"]):
        sid, lo, hi, spd = int(row[0]), float(row[1]), float(row[2]), float(row[3])
        assert sid == int(j["servo_id"]), j["name"]
        assert lo == pytest.approx(float(j["min_deg"])), f"{j['name']} の下限"
        assert hi == pytest.approx(float(j["max_deg"])), f"{j['name']} の上限"
        assert spd == pytest.approx(float(j["max_speed_dps"])), f"{j['name']} の速度"


def test_yaw_chain_matches_config(config_h: str) -> None:
    """巻ける角の合計に使う軸（axis: yaw）の添字がファームと config で同じ。"""
    cfg = load_config()
    m = re.search(r"YAW_CHAIN\[N_YAW_CHAIN\]\s*=\s*\{([^}]+)\}", config_h)
    assert m, "YAW_CHAIN がファームに無い"
    fw = [int(x) for x in m.group(1).split(",")]
    want = [i for i, j in enumerate(cfg["joints"]) if j["axis"] == "yaw"]
    assert fw == want
    n = re.search(r"#define N_YAW_CHAIN (\d+)", config_h)
    assert n and int(n.group(1)) == len(want)


def test_home_pose_matches_config(config_h: str) -> None:
    cfg = load_config()
    m = re.search(r"HOME_DEG\[N_AXES\]\s*=\s*\{([^}]+)\}", config_h)
    assert m, "HOME_DEG がファームに無い"
    fw = [float(x.strip().rstrip("f")) for x in m.group(1).split(",")]
    home = cfg["poses"]["home"]
    want = [float(home.get(j["name"], 0.0)) for j in cfg["joints"]]
    assert fw == pytest.approx(want)


def test_telemetry_sizes_match(link_h: str) -> None:
    """テレメトリの大きさがファームと Python で一致する（C++ 側は static_assert でも守る）。"""
    import struct

    from serpens.link.protocol import FMT_AXIS, FMT_TELEM_HEAD

    assert const_value(link_h, "TELEM_HEAD_BYTES") == struct.calcsize(FMT_TELEM_HEAD)
    assert const_value(link_h, "TELEM_AXIS_BYTES") == struct.calcsize(FMT_AXIS)


def test_float_literals_are_valid_cpp(config_h: str) -> None:
    """**C++ として妥当な float リテラルであること。**

    生成器が `0f` を吐いてビルドが落ちた（2026-09-16）。`0.0f` でなければならない。
    """
    for lit in re.findall(r"[-+]?\d+(?:\.\d+)?f", config_h):
        assert "." in lit, f"不正な float リテラル: {lit}（0f ではなく 0.0f）"


def test_firmware_declares_hardware_source() -> None:
    """**実機ファームは SIMULATED を立てず、SRC_HARDWARE を返す。**"""
    ino = source("serpens_esp32.ino")
    assert "SRC_HARDWARE" in ino
    assert "FL_SIMULATED" not in ino.replace("**FL_SIMULATED は実機では立てない**", "")
