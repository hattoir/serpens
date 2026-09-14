"""Belly（腹側の交換パーツ）・摩擦プロファイル・歩容パラメータ・サーボ版数の設定。

**摩擦はどれも未実測（HARDWARE_UNVERIFIED）。** ここで確かめるのは
「設定が効くこと」と「仮値を現実として扱っていないこと」であって、現実の摩擦ではない。
"""
from __future__ import annotations

import copy

import pytest

from serpens.config import load_config
from serpens.motion.gait import BODY_AXIS, GaitParams, body_joint_names
from serpens.sim.measure import per_cycle_advance
from serpens.sim.world import World

FORWARD = GaitParams(30.0, 60.0, 0.5, 0.0)


@pytest.fixture()
def cfg() -> dict:
    return load_config()


def with_belly(cfg: dict, type_: str, profile: str) -> dict:
    c = copy.deepcopy(cfg)
    c["belly"]["type"], c["belly"]["friction_profile"] = type_, profile
    return c


# ---- Belly / 摩擦 ----------------------------------------------------------------------
def test_default_is_wheel_belly_and_matches_previous_model(cfg: dict) -> None:
    """既定（Wheel / MEDIUM）は、belly を導入する前と同じ動きのままであること。"""
    assert cfg["belly"]["type"] == "wheel" and cfg["belly"]["friction_profile"] == "MEDIUM"
    prof = cfg["belly"]["profiles"]["wheel"]["MEDIUM"]
    assert prof["lateral"] == 1.0
    assert prof["tangential"] == pytest.approx(0.02)     # 2026-09-12 までと同じ推定値
    assert "tangential_drag_ratio" not in cfg["sim"], "摩擦の値が2か所にある"
    adv = per_cycle_advance(cfg, FORWARD, 2.0, 3.0)
    assert adv.per_cycle_mm == pytest.approx(384.0, abs=15.0), "接触モデルが変わっている"


def test_friction_profiles_change_propulsion(cfg: dict) -> None:
    """摩擦が強いほど進まない（プロファイルが効いている）。"""
    low = per_cycle_advance(with_belly(cfg, "wheel", "LOW"), FORWARD, 2.0, 3.0).per_cycle_mm
    mid = per_cycle_advance(with_belly(cfg, "wheel", "MEDIUM"), FORWARD, 2.0, 3.0).per_cycle_mm
    high = per_cycle_advance(with_belly(cfg, "wheel", "HIGH"), FORWARD, 2.0, 3.0).per_cycle_mm
    assert low > mid > high


def test_snake_belly_needs_anisotropy_to_move(cfg: dict) -> None:
    """鱗の腹（車輪なし）は、異方性が無いとほとんど進まない。

    **これは仮の摩擦係数から出た予測であって、実測ではない。**
    実機で Snake Belly を試すときに最初に確かめる仮説。
    """
    wheel = per_cycle_advance(with_belly(cfg, "wheel", "MEDIUM"), FORWARD, 2.0, 3.0).per_cycle_mm
    iso = per_cycle_advance(with_belly(cfg, "snake", "MEDIUM"), FORWARD, 2.0, 3.0).per_cycle_mm
    aniso = per_cycle_advance(with_belly(cfg, "snake", "ANISOTROPIC"), FORWARD, 2.0, 3.0).per_cycle_mm
    assert iso < wheel * 0.5, "車輪なしなのに車輪並みに進んでいる"
    assert aniso > iso * 2.0, "異方性が効いていない"


def test_snake_belly_puts_every_body_link_on_the_floor(cfg: dict) -> None:
    """車輪が無いので、胴体のリンク全部が接地する（拘束の作られ方が変わる）。"""
    wheel_world = World(with_belly(cfg, "wheel", "MEDIUM"))
    snake_world = World(with_belly(cfg, "snake", "MEDIUM"))
    assert all(w == 1.0 for _, w, _ in wheel_world._contacts[:len(wheel_world.body_links)])
    assert all(w < 1.0 for _, w, _ in snake_world._contacts[:len(snake_world.body_links)]), \
        "鱗の腹なのに横滑りしない拘束になっている"
    assert wheel_world.body_links == snake_world.body_links


def test_friction_values_are_marked_unverified() -> None:
    """**仮値であることが設定ファイルに書いてある**（実測と取り違えないため）。"""
    text = (load_config.__module__ and open("config/robot.yaml", encoding="utf-8").read())
    belly = text[text.index("belly:") - 600:text.index("belly:")]
    assert "未実測" in belly and "HARDWARE_UNVERIFIED" in belly


# ---- 歩容パラメータ --------------------------------------------------------------------
def test_gait_parameters_are_configurable(cfg: dict) -> None:
    g = cfg["gait"]
    for key in ("phase_offset_deg", "pitch_amplitude_deg", "yaw_pitch_phase_deg",
                "turn_profile", "turn_ramp_periods", "blend_s"):
        assert key in g, key
    for name, p in g["presets"].items():
        for key in ("amplitude_deg", "spatial_freq_deg", "temporal_freq_hz", "turn_bias_deg"):
            assert key in p, f"{name}.{key}"


def test_pitch_amplitude_stays_zero_without_body_pitch_axes(cfg: dict) -> None:
    """**垂直波は封印。** 胴体に pitch 軸が無いので出せないし、出せると巻き付きが可能になる。

    構想設計書 16章「螺旋歩容のコードを存在させない」。機構が直交2軸構成になったら、
    安全設計を見直したうえでこのテストを書き換えること。
    """
    axes = {j["name"]: j["axis"] for j in cfg["joints"]}
    body_pitch = [n for n in body_joint_names(cfg) if axes[n] != BODY_AXIS]
    assert not body_pitch, "胴体に pitch 軸が入った。安全設計の見直しが要る"
    assert float(cfg["gait"]["pitch_amplitude_deg"]) == 0.0, "この機体では垂直波を出せない"


# ---- サーボの版数 ----------------------------------------------------------------------
def test_selected_servo_variant_has_a_known_stall_torque(cfg: dict) -> None:
    """**未確認のトルクで安全上限を計算しない。**

    第一候補は STS3215 7.4V 1:191 だが、その版のストールトルクを確認できていない。
    variant を切り替えるなら、先に実測かデータシートで値を埋めること。
    """
    s = cfg["servo"]
    variant = s["variants"][s["variant"]]
    assert variant["stall_torque_kgfcm"] is not None, (
        f"{s['variant']} のストールトルクが未確認。安全のトルク上限を計算できない")
    assert variant["stall_torque_kgfcm"] == pytest.approx(s["stall_torque_kgfcm"])
    assert variant["supply_voltage_v"] == pytest.approx(s["supply_voltage_v"])


def test_candidate_variant_is_listed_even_if_unknown(cfg: dict) -> None:
    """候補（7.4V 1:191）が設定に載っていて、未確認だと分かること。"""
    v = cfg["servo"]["variants"]
    assert "7.4V_1:191" in v
    assert v["7.4V_1:191"]["stall_torque_kgfcm"] is None
    assert not any(x["verified"] for x in v.values()), "実機で確認していないのに verified になっている"
