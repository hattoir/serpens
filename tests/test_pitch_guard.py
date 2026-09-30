"""Floor Watch の頭（J7）の範囲・速さの強制（R-009。`serpens/motion/pitch_guard.py`、機体側 `device_motion.py`、PC 側 `client.py`、ファーム `config.h` / `.ino`）。

範囲 [−4°, +3°]・窓の端の手前 ≤ 40 °/s は **PROVISIONAL（シミュレーション・prior）**。既定はオフ（HT-001 の実機確認の後に有効化）。
**ファームの C++ はこの環境ではコンパイルしていない**（arduino-cli 無し）。ここでは定数の一致と、フックの存在を文字列で確かめるだけ。
"""
from __future__ import annotations

import copy
import logging
import re
from pathlib import Path

import pytest

from serpens.config import load_config
from serpens.link import messages as m
from serpens.link.device_motion import DeviceMotion
from serpens.link.harness import LinkHarness
from serpens.link.protocol import Nack
from serpens.motion.pitch_guard import PitchGuard, PitchGuardConfig

FW = Path(__file__).resolve().parent.parent / "firmware" / "serpens_esp32"


@pytest.fixture()
def cfg() -> dict:
    return load_config()


def enforced(cfg: dict) -> dict:
    c = copy.deepcopy(cfg)
    c["neck"]["floor_watch_enforce"] = True
    return c


# ---- 設定 -------------------------------------------------------------------------------
def test_default_is_off_and_the_existing_j7_limits_are_unchanged(cfg: dict) -> None:
    assert cfg["neck"]["floor_watch_enforce"] is False                 # HT-001 の実機確認まで有効にしない
    j7 = next(j for j in cfg["joints"] if j["name"] == "J7")
    assert (j7["min_deg"], j7["max_deg"]) == (-8.0, 90.0)              # 既存の J7 の範囲は変えない
    assert not DeviceMotion(cfg).pitch.enabled
    assert PitchGuardConfig.from_cfg(cfg).stop == (-4.0, 3.0)


def test_inconsistent_ranges_are_refused(cfg: dict) -> None:
    bad = copy.deepcopy(cfg)
    bad["neck"]["floor_watch_soft_deg"] = [-5.0, 1.0]                   # 窓がストッパーの外
    with pytest.raises(ValueError):
        PitchGuardConfig.from_cfg(bad)
    assert PitchGuard.from_cfg({"neck": {}}).enabled is False           # 設定が無ければガードなし


def test_disabled_guard_changes_nothing(cfg: dict) -> None:
    g = PitchGuard.from_cfg(cfg)
    assert g.check_head_range(60.0) and g.clamp_target(60.0) == 60.0 and g.speed_limit(0.0, 60.0, 120.0) == 120.0
    assert g.prevet_head(60.0, 120.0).deg == 60.0 and g.events == []


# ---- 範囲外 -----------------------------------------------------------------------------
def test_out_of_range_head_is_rejected_and_logged(cfg: dict, caplog) -> None:
    g = PitchGuard.from_cfg(cfg, enabled=True)
    with caplog.at_level(logging.WARNING, logger="serpens.pitch_guard"):
        assert g.check_head_range(-4.0) and g.check_head_range(3.0)     # 端は範囲内
        assert not g.check_head_range(3.01) and not g.check_head_range(-4.01) and not g.check_head_range(8.0)
    assert [e["kind"] for e in g.events] == ["REJECT"] * 3
    assert "REJECT" in caplog.text


def test_pose_and_breath_are_clamped_and_logged(cfg: dict) -> None:
    mo = DeviceMotion(enforced(cfg))
    mo.set_pose({"J7": 8.0})                                            # home の J7 = 8°
    assert mo.target["J7"] == 3.0
    assert mo.pitch.events[-1]["kind"] == "CLAMP" and mo.pitch.events[-1]["requested"] == 8.0
    mo.goals["J7"] = 3.0
    peak = max(mo.output(t * 0.01, True)["J7"] for t in range(600))     # 呼吸 ±5°（周期 6 s 想定）を足しても、範囲の上端を超えない
    low = min(mo.output(t * 0.01, True)["J7"] for t in range(600))
    assert peak <= 3.0 + 1e-9 and low >= -4.0 - 1e-9


# ---- 速さ -------------------------------------------------------------------------------
def test_speed_limit_ramps_down_to_the_near_limit_speed_at_the_window_edge(cfg: dict) -> None:
    g = PitchGuard.from_cfg(cfg, enabled=True)
    assert g.speed_limit(0.0, 0.5, 120.0) == 120.0                      # 窓の内側だけの移動は制限しない
    assert g.speed_limit(1.5, 2.5, 120.0) == 40.0                       # 区間の中は上限
    assert g.speed_limit(2.5, 0.0, 120.0) == 40.0                       # 区間から戻るときも上限
    assert g.speed_limit(0.0, 2.5, 120.0) == pytest.approx(60.0, abs=0.01)   # 窓の端まで 1° → √(40² + 2·1000·1)
    assert g.speed_limit(1.0, 2.5, 120.0) == pytest.approx(40.0, abs=0.01)   # 窓の端では上限に一致
    assert g.speed_limit(-1.0, -3.0, 120.0) == pytest.approx(60.0, abs=0.01)


def test_step_never_exceeds_the_near_limit_speed_inside_the_band(cfg: dict) -> None:
    """窓の内側から +3° へ 120 °/s で指令しても、区間（+1°〜+3°）の中の実際の速さは 40 °/s（離散化の 5% 以内）を超えない。"""
    mo = DeviceMotion(enforced(cfg))
    mo.goals["J7"] = 0.0
    mo.target["J7"] = 3.0
    mo.speed["J7"] = 120.0
    dt = 1.0 / float(cfg["link"]["control_hz"])
    worst = 0.0
    for _ in range(600):
        before = mo.goals["J7"]
        mo.step(dt, None)
        if before > cfg["neck"]["floor_watch_soft_deg"][1]:              # 区間の中
            worst = max(worst, abs(mo.goals["J7"] - before) / dt)
    assert mo.goals["J7"] == pytest.approx(3.0)
    assert 0.0 < worst <= 40.0 * 1.05
    assert mo.goals["J7"] <= 3.0 + 1e-9                                  # ストッパーの範囲を超えない


def test_head_command_speed_is_capped_when_the_target_is_in_the_band(cfg: dict) -> None:
    mo = DeviceMotion(enforced(cfg))
    mo.goals["J7"] = 1.5                                                # すでに区間の中
    h = m.Head(500, 2.5, 0.0, 0.0, 120.0)
    assert mo.head_ok(h)
    mo.set_head(h)
    assert mo.speed["J7"] == 40.0
    assert any(e["kind"] == "SPEED" for e in mo.pitch.events)


# ---- 端から端（PC ⇄ 偽経路 ⇄ 機体）--------------------------------------------------------
def _armed(h: LinkHarness) -> None:
    h.advance(0.2)
    h.client.arm(h.now)
    h.advance(0.1)


def test_device_rejects_out_of_range_head_even_if_the_pc_does_not_check(cfg: dict) -> None:
    """PC を信頼しない: PC 側のガードを外しても、機体が NACK OUT_OF_RANGE で拒否し、目標を変えない。"""
    h = LinkHarness(enforced(cfg))
    h.client.pitch = PitchGuard(None)                                   # PC 側は検査しない
    _armed(h)
    before = h.device.mo.target["J7"]
    h.client.head(h.now, j7=4.0, j8=0.0, j9=0.0, speed_dps=30.0, ttl_ms=500)
    h.advance(0.1)
    assert any(n[1].name == "HEAD" and n[2] is Nack.OUT_OF_RANGE for n in h.client.nacks)
    assert h.device.mo.target["J7"] == before


def test_pc_clamps_and_logs_and_the_device_accepts_the_clamped_value(cfg: dict) -> None:
    h = LinkHarness(enforced(cfg))
    _armed(h)
    h.client.head(h.now, j7=4.0, j8=0.0, j9=0.0, speed_dps=30.0, ttl_ms=500)
    h.advance(0.1)
    assert not any(n[2] is Nack.OUT_OF_RANGE for n in h.client.nacks)
    assert h.device.mo.target["J7"] == pytest.approx(3.0, abs=0.05)
    assert h.client.pitch.events and h.client.pitch.events[0]["kind"] == "CLAMP"


def test_default_config_still_accepts_a_wide_head_command(cfg: dict) -> None:
    """既定（オフ）では従来どおり J7 = 60° も通る（人を見る姿勢。既存の動作を壊さない）。"""
    h = LinkHarness(cfg)
    _armed(h)
    h.client.head(h.now, j7=60.0, j8=0.0, j9=0.0, speed_dps=60.0, ttl_ms=500)
    h.advance(0.1)
    assert not h.client.nacks and h.device.mo.target["J7"] == 60.0


# ---- ファーム（C++ の写し。コンパイルはしていない）------------------------------------------
def test_firmware_constants_match_the_config_and_guard_is_off_by_default(cfg: dict) -> None:
    text = (FW / "config.h").read_text(encoding="utf-8")

    def f(name: str) -> float:
        return float(re.search(rf"\b{name}\s*=\s*(-?[0-9.]+)f?", text).group(1))

    n = cfg["neck"]
    assert re.search(r"FW_PITCH_GUARD_ENABLED\s*=\s*false", text) and n["floor_watch_enforce"] is False
    assert (f("FW_PITCH_SOFT_LO_DEG"), f("FW_PITCH_SOFT_HI_DEG")) == tuple(n["floor_watch_soft_deg"])
    assert (f("FW_PITCH_STOP_LO_DEG"), f("FW_PITCH_STOP_HI_DEG")) == tuple(n["floor_watch_stop_deg"])
    assert f("FW_PITCH_NEAR_SPEED_DPS") == n["floor_watch_near_limit_speed_dps"]
    assert f("FW_PITCH_DECEL_DPS2") == n["floor_watch_decel_dps2"]
    names = [j["name"] for j in cfg["joints"]]
    assert int(re.search(r"FW_PITCH_AXIS\s*=\s*(\d+)", text).group(1)) == names.index("J7")
    assert re.search(r"\{7,\s*-8\.0f,\s*90\.0f", text)                                      # 既存の J7 の範囲は変えていない


def test_firmware_has_the_guard_hooks_in_head_pose_and_control() -> None:
    ino = (FW / "serpens_esp32.ino").read_text(encoding="utf-8")
    assert "pitchInStop(a[FW_PITCH_AXIS - N_BODY])" in ino and "gPitchRejects++" in ino      # HEAD: 範囲外は拒否
    assert "t = pitchClamp(t)" in ino                                                        # POSE: クランプ
    assert "tgt = pitchClamp(tgt)" in ino and "spd = pitchSpeedLimit(gGoal[i], tgt, spd)" in ino  # 制御周期: 範囲と速さ
    assert ino.count("FW_PITCH_GUARD_ENABLED") >= 3                                          # どのフックも有効フラグで囲まれている
