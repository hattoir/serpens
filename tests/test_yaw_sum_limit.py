"""胴体ヨーの角度合計の上限（link.limits.yaw_sum_deg）。**体が輪を作って手首・首を囲い込めない。**

DEC-USER-0002（2026-09-29）: 暫定 145°（PROVISIONAL / SAFETY_UNVERIFIED）。
機体側（DeviceMotion）が DRIVE / BODY を拒否し、POSE と最終出力（呼吸込み）は比例で縮める。
"""
from __future__ import annotations

import pytest

from serpens.config import load_config
from serpens.link.device_motion import DeviceMotion, max_contiguous_sum
from serpens.link.messages import Body, Drive


@pytest.fixture()
def mo() -> DeviceMotion:
    return DeviceMotion(load_config())


def test_limit_is_the_user_decision() -> None:
    assert float(load_config()["link"]["limits"]["yaw_sum_deg"]) == pytest.approx(145.0)


def test_max_contiguous_sum() -> None:
    assert max_contiguous_sum([10.0, 20.0, -5.0, 30.0]) == pytest.approx(55.0)
    assert max_contiguous_sum([-40.0, -40.0, 30.0]) == pytest.approx(80.0)
    assert max_contiguous_sum([]) == 0.0


def test_standard_gaits_are_accepted(mo: DeviceMotion) -> None:
    """展示・巡回の歩容（旋回込み）は上限の内側。"""
    for p in load_config()["gait"]["presets"].values():
        d = Drive(500, float(p["amplitude_deg"]), float(p["spatial_freq_deg"]), float(p["temporal_freq_hz"]),
                  float(p.get("turn_bias_deg", 0.0)))
        assert mo.drive_ok(d), p


def test_encircling_drive_is_rejected(mo: DeviceMotion) -> None:
    """全関節が同じ向きに曲がる歩容（波長が長い + 大きな旋回）は拒否する。"""
    d = Drive(500, 20.0, 5.0, 0.5, 30.0)                   # 振幅 20 + γ 30 = 50（各関節の上限内）
    assert d.amplitude_deg + d.gamma_deg <= 50.0
    assert mo.drive_yaw_sum(d) > 145.0
    assert not mo.drive_ok(d)


def test_encircling_body_is_rejected(mo: DeviceMotion) -> None:
    n = len(mo.body)
    ok = Body(500, tuple([145.0 / n - 0.1] * n), 60.0)
    bad = Body(500, tuple([30.0] * n), 60.0)               # 各関節は ±50° の内側、合計 180°
    assert mo.body_ok(ok)
    assert not mo.body_ok(bad)


def test_pose_and_output_are_scaled_down(mo: DeviceMotion) -> None:
    """POSE の形が上限を超えていたら同じ比で縮める。呼吸を足した最終出力も上限以下。"""
    mo.set_pose({n: 45.0 for n in mo.body})
    assert max_contiguous_sum([mo.target[n] for n in mo.body]) == pytest.approx(145.0)
    for n in mo.body:
        mo.goals[n] = 145.0 / len(mo.body)                   # ちょうど上限
    for t in (0.0, 0.5, 1.0, 1.5, 2.0, 3.0):
        out = mo.output(t, breathing=True)
        assert max_contiguous_sum([out[n] for n in mo.body]) <= 145.0 + 1e-9


def test_head_yaw_counts_toward_the_limit(mo: DeviceMotion) -> None:
    """頭ヨーも巻ける角に効く（安全側: 首 pitch によらず同じ平面とみなす）。"""
    assert "J8" in mo.yaw_chain and "J7" not in mo.yaw_chain and "J9" not in mo.yaw_chain
    for n in mo.body:
        mo.goals[n] = 140.0 / len(mo.body)
    mo.goals["J8"] = 60.0
    out = mo.output(0.0, breathing=False)
    assert max_contiguous_sum([out[n] for n in mo.yaw_chain]) <= 145.0 + 1e-9


def test_servo_bus_enforces_the_limit_even_for_partial_writes() -> None:
    """PC 直結の経路も含む最下層（ServoBus）でも上限を守る。一部の軸だけ書く指令でも、前の姿勢と合わせて判定する。"""
    from serpens.hw.mock_bus import MockServoBus
    from serpens.hw.servo_bus import Goal

    cfg = load_config()
    bus = MockServoBus(cfg)
    bus.connect()
    ids = {j["name"]: int(j["servo_id"]) for j in cfg["joints"]}
    body = [f"J{k}" for k in range(1, 7)]
    bus.sync_set_goals({ids[n]: Goal(22.0, 100.0, 1000.0) for n in body})          # 132°（上限内）
    assert max_contiguous_sum([bus._last_deg[ids[n]] for n in body]) == pytest.approx(132.0)
    bus.set_goal(ids["J8"], 60.0, 90.0, 1000.0)                                      # 足すと 192° → J8 だけ縮む
    chain = [bus._last_deg.get(ids[n], 0.0) for n in body + ["J8"]]
    assert max_contiguous_sum(chain) <= 145.0 + 1e-6
    assert bus._last_deg[ids["J8"]] == pytest.approx(13.0, abs=0.01)                 # 胴体（先に書いた軸）は動かさない
    bus.set_goal(ids["J8"], -40.0, 90.0, 1000.0)                                     # 逆向きは囲い込まない → そのまま
    assert bus._last_deg[ids["J8"]] == pytest.approx(-40.0)
