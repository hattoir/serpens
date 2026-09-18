"""STEP 3: 歩容と姿勢のテスト。"""
from __future__ import annotations

import copy
import math

import numpy as np
import pytest

from serpens.config import load_config
from serpens.motion.gait import (GaitEngine, GaitParams, body_joint_names, gait_angles,
                                 gait_period_s, turn_weights)
from serpens.motion.kinematics import min_self_clearance
from serpens.motion.poses import Poses

SAMPLES = 200


@pytest.fixture()
def cfg() -> dict:
    return load_config()


@pytest.fixture()
def names(cfg: dict) -> list[str]:
    return body_joint_names(cfg)


def preset(cfg: dict, name: str) -> GaitParams:
    return GaitParams.from_cfg(cfg["gait"]["presets"][name])


def sweep(p: GaitParams, names: list[str], profile: str = "uniform") -> np.ndarray:
    """1周期ぶんの角度列。shape (SAMPLES, 6)"""
    T = gait_period_s(p)
    return np.array([[gait_angles(p, T * k / SAMPLES, names, profile)[n] for n in names] for k in range(SAMPLES)])


def limits(cfg: dict) -> dict[str, tuple[float, float]]:
    return {j["name"]: (j["min_deg"], j["max_deg"]) for j in cfg["joints"]}


def test_body_joints_are_j1_to_j6(names: list[str]) -> None:
    assert names == ["J1", "J2", "J3", "J4", "J5", "J6"]


def test_forward_gait_shape(cfg: dict, names: list[str]) -> None:
    """全関節が振幅 A・平均 0 で振れ、周期的で、リミット内。"""
    p = preset(cfg, "forward")
    a = sweep(p, names)
    assert np.allclose(a.max(axis=0), p.amplitude_deg, atol=0.1)
    assert np.allclose(a.mean(axis=0), 0.0, atol=1e-6)
    T = gait_period_s(p)
    assert gait_angles(p, 0.3, names) == pytest.approx(gait_angles(p, 0.3 + T, names))
    lim = limits(cfg)
    assert all(lim[n][0] <= v <= lim[n][1] for row in a for n, v in zip(names, row))


def test_wave_travels_head_to_tail_when_forward(cfg: dict, names: list[str]) -> None:
    """α(n, t + Ω/ω) = α(n+1, t)：頭側の関節の動きを、少し遅れて尾側がなぞる。"""
    for name, direction in (("forward", 1), ("backward", -1)):
        p = preset(cfg, name)
        lag = math.radians(p.spatial_freq_deg) / (2 * math.pi * p.temporal_freq_hz)
        t = 0.37
        now, later = gait_angles(p, t, names), gait_angles(p, t + lag, names)
        for n in range(len(names) - 1):
            assert later[names[n]] == pytest.approx(now[names[n + 1]], abs=1e-9)
        assert (lag > 0) == (direction > 0)


def test_turn_profiles(cfg: dict, names: list[str]) -> None:
    """uniform: 全関節に γ0。head_weighted: γ(n) = γ0·n/N（尾 0 → 頭 γ0）。"""
    p = GaitParams(30.0, 60.0, 0.5, 12.0)
    assert np.allclose(sweep(p, names, "uniform").mean(axis=0), 12.0, atol=1e-6)
    hw = sweep(p, names, "head_weighted").mean(axis=0)
    assert np.allclose(hw, [12.0 * n / 5 for n in range(6)], atol=1e-6)
    assert turn_weights(6, "head_weighted")[0] == 0.0 and turn_weights(6, "head_weighted")[-1] == 1.0
    with pytest.raises(ValueError):
        turn_weights(6, "gradient")


def test_turn_presets_within_limits(cfg: dict, names: list[str]) -> None:
    lim = limits(cfg)
    for g in ("turn_left", "turn_right"):
        a = sweep(preset(cfg, g), names, cfg["gait"]["turn_profile"])
        assert all(lim[n][0] <= v <= lim[n][1] for row in a for n, v in zip(names, row))


def test_turn_offset_ramps_over_one_period(cfg: dict) -> None:
    """γ0 はステップで変わらず、turn_ramp_periods 周期かけて移る。"""
    eng = GaitEngine(cfg)
    dt = 0.02
    eng.start("forward")
    t = 0.0
    for _ in range(100):
        t += dt
        eng.update(t)
    eng.set_turn(20.0)
    T = gait_period_s(eng.params) * cfg["gait"]["turn_ramp_periods"]
    seen = []
    for _ in range(int(T / dt) + 5):
        t += dt
        eng.update(t)
        seen.append(eng.gamma0)
    assert seen[0] < 1.0                                  # 急には曲がらない
    assert seen[len(seen) // 2] == pytest.approx(10.0, abs=2.0)
    assert seen[-1] == pytest.approx(20.0)
    assert all(b >= a for a, b in zip(seen, seen[1:]))


def test_engine_blends_in_and_out(cfg: dict) -> None:
    """開始直後は小さく、blend_s 後に全振幅、停止すると 0 に戻る。位相は飛ばない。"""
    eng = GaitEngine(cfg)
    blend = cfg["gait"]["blend_s"]
    dt = 0.02
    eng.start("forward")
    trace, t = [], 0.0
    for _ in range(int(6 / dt)):
        trace.append(eng.update(t))
        t += dt
    assert max(abs(v) for v in trace[1].values()) < 1.0
    full = max(abs(v) for tr in trace[int(blend / dt) + 10:] for v in tr.values())
    assert full == pytest.approx(preset(cfg, "forward").amplitude_deg, abs=1.0)
    eng.start("turn_left")
    prev = eng.update(t)
    nxt = eng.update(t + dt)
    assert max(abs(nxt[k] - prev[k]) for k in prev) < 5.0
    eng.stop()
    for _ in range(int((blend + 0.1) / dt)):
        t += dt
        out = eng.update(t)
    assert out == {} and not eng.active


# ---- 姿勢 ------------------------------------------------------------------------
def test_presets_within_soft_limits(cfg: dict) -> None:
    poses = Poses(cfg)
    lim = limits(cfg)
    all_poses = [poses.home(), poses.rest(), poses.stretch(), poses.head_look(80, 50, 90),
                 poses.relax().angles] + [poses.rear_up(60, b) for b in poses.rear_up_bases]
    for pose in all_poses:
        for n, v in pose.items():
            assert lim[n][0] <= v <= lim[n][1], (n, v)
    assert poses.home()["J7"] == 8 and all(v == 0 for k, v in poses.home().items() if k != "J7")


def test_rest_arc_leaves_room_for_breathing(cfg: dict) -> None:
    """とぐろ ± 呼吸振幅 でもソフトリミット内。"""
    amp = cfg["breath"]["amplitude_by_axis"]
    lim = limits(cfg)
    for n, v in Poses(cfg).rest().items():
        assert lim[n][0] <= v - amp[n] and v + amp[n] <= lim[n][1], n


def test_rest_arc_has_no_self_intersection(cfg: dict) -> None:
    """R03 の休憩姿勢: 合計 200° 以上曲げ、呼吸で ±振れても中心線間隔が下限以上。

    曲げ量は software_operational_limit（現在 ±50° CONDITIONAL、CAD R03 由来）で決まる。
    **旧とぐろ（329°巻き）は R03 では作れない。** `legacy_poses` を参照。
    """
    poses = Poses(cfg)
    sc = cfg["self_collision"]
    coil = poses.rest()
    assert sum(coil[f"J{k}"] for k in range(1, 7)) > 200
    amp = cfg["breath"]["amplitude_by_axis"]
    for d in (-1.0, 0.0, 1.0):
        pose = {k: v + d * amp[k] for k, v in coil.items()}
        pts = poses.points(pose)
        clr = min_self_clearance(pts[:, :2], sc["arc_skip_mm"], sc["sample_mm"])
        assert clr >= sc["min_clearance_mm"] >= cfg["body"]["diameter_mm"], (d, clr)
    tight = {f"J{k}": 90.0 for k in range(1, 7)}
    assert min_self_clearance(poses.points(tight)[:, :2], sc["arc_skip_mm"], sc["sample_mm"]) < sc["min_clearance_mm"]


def test_rear_up_bases_have_no_self_intersection(cfg: dict) -> None:
    poses = Poses(cfg)
    sc = cfg["self_collision"]
    assert set(poses.rear_up_bases) == {"arc"}                      # s_curve は威嚇に見えるので legacy へ
    assert "s_curve" in cfg["legacy_poses"]
    for b in poses.rear_up_bases:
        pts = poses.points(poses.rear_up(60, b))
        assert min_self_clearance(pts[:, :2], sc["arc_skip_mm"], sc["sample_mm"]) >= sc["min_clearance_mm"]


@pytest.mark.parametrize("angle, height", [(55, 147), (65, 163), (85, 179)])
def test_rear_up_height(cfg: dict, angle: float, height: float) -> None:
    """鎌首の高さ = (J7→頭先端 180mm) × sin(J7)。"""
    poses = Poses(cfg)
    assert poses.head_height_mm(poses.rear_up(angle)) == pytest.approx(height, abs=1.0)


def test_head_look_limits_neck_to_look_range(cfg: dict) -> None:
    poses = Poses(cfg)
    nk = cfg["neck"]
    assert poses.head_look(10, 0, 90)["J7"] == nk["look_max_deg"]
    assert poses.head_look(10, 0, 20)["J7"] == nk["look_min_deg"]
    assert "J7" not in poses.head_look(10, 5)
    assert poses.head_look(30, 15) == {"J8": 30, "J9": 15}
    assert poses.head_look(120, -60) == {"J8": 80, "J9": -35}      # ソフトリミット
    assert poses.stretch()["J7"] == nk["full_rear_min_deg"]


def test_relax_command(cfg: dict) -> None:
    r = Poses(cfg).relax()
    assert r.torque_ratio == cfg["poses"]["relax"]["torque_ratio"]
    assert r.duration_s == cfg["poses"]["relax"]["duration_s"]
    assert r.angles["J7"] == cfg["poses"]["relax"]["neck_deg"]


def test_config_copy_is_independent(cfg: dict) -> None:
    c = copy.deepcopy(cfg)
    c["gait"]["turn_profile"] = "uniform"
    assert GaitEngine(c).profile == "uniform" and GaitEngine(cfg).profile == cfg["gait"]["turn_profile"]
