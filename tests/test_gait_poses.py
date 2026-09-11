"""STEP 3: 歩容と姿勢のテスト。"""
from __future__ import annotations

import math

import numpy as np
import pytest

from serpens.config import load_config
from serpens.motion.gait import GaitEngine, GaitParams, body_joint_names, gait_angles, gait_period_s
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


def sweep(p: GaitParams, names: list[str]) -> np.ndarray:
    """1周期ぶんの角度列。shape (SAMPLES, 6)"""
    T = gait_period_s(p)
    return np.array([[gait_angles(p, T * k / SAMPLES, names)[n] for n in names] for k in range(SAMPLES)])


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
    lim = {j["name"]: (j["min_deg"], j["max_deg"]) for j in cfg["joints"]}
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
        assert (lag > 0) == (direction > 0)   # 後退では向きが逆


def test_turn_bias_and_amplitude_gradient(cfg: dict, names: list[str]) -> None:
    left, right = preset(cfg, "turn_left"), preset(cfg, "turn_right")
    assert np.allclose(sweep(left, names).mean(axis=0), left.turn_bias_deg, atol=1e-6)
    assert np.allclose(sweep(right, names).mean(axis=0), right.turn_bias_deg, atol=1e-6)
    grad = GaitParams(30.0, 60.0, 0.5, amp_gradient=0.5)
    amp = sweep(grad, names).max(axis=0)
    assert amp[-1] > amp[0]                        # 頭側ほど大きい
    assert amp[-1] / amp[0] == pytest.approx(1.25 / 0.75, rel=0.01)


def test_engine_blends_in_and_out(cfg: dict) -> None:
    """開始直後は小さく、blend_s 後に全振幅、停止すると 0 に戻る。位相は飛ばない。"""
    eng = GaitEngine(cfg)
    blend = cfg["gait"]["blend_s"]
    dt = 0.02
    eng.start("forward")
    trace = []
    t = 0.0
    for _ in range(int(6 / dt)):
        trace.append(eng.update(t))
        t += dt
    first = max(abs(v) for v in trace[1].values())
    assert first < 1.0
    full = max(abs(v) for tr in trace[int(blend / dt) + 10:] for v in tr.values())
    assert full == pytest.approx(preset(cfg, "forward").amplitude_deg, abs=1.0)
    eng.start("turn_left")                           # 途中で切替
    prev = eng.update(t)
    nxt = eng.update(t + dt)
    assert max(abs(nxt[k] - prev[k]) for k in prev) < 5.0
    eng.stop()
    for _ in range(int((blend + 0.1) / dt)):
        t += dt
        out = eng.update(t)
    assert out == {} and not eng.active


# ---- 姿勢 ------------------------------------------------------------------------
def test_presets_within_limits(cfg: dict) -> None:
    poses = Poses(cfg)
    lim = {j["name"]: (j["min_deg"], j["max_deg"]) for j in cfg["joints"]}
    for pose in (poses.home(), poses.coil(), poses.rear_up(), poses.full_rear_up(),
                 poses.head_look(80, 50, 90), poses.relax().angles):
        for n, v in pose.items():
            assert lim[n][0] <= v <= lim[n][1], (n, v)
    assert all(v == 0 for v in poses.home().values())


def test_coil_has_no_self_intersection(cfg: dict) -> None:
    """とぐろ: J1〜J6 だけで 300° 以上巻き、胴体どうしが直径以上離れている。"""
    poses = Poses(cfg)
    sc = cfg["self_collision"]
    coil = poses.coil()
    assert all(coil[n] == 0 for n in ("J7", "J8", "J9"))
    assert sum(coil[f"J{k}"] for k in range(1, 7)) > 300
    pts = poses.points(coil)
    assert np.allclose(pts[:, 2], 0.0)             # 平面
    clr = min_self_clearance(pts[:, :2], sc["arc_skip_mm"], sc["sample_mm"])
    assert clr >= sc["min_clearance_mm"] >= cfg["body"]["diameter_mm"]
    # 巻きすぎると干渉することも検出できる（チェック自体の検証）
    tight = {f"J{k}": 90.0 for k in range(1, 7)}
    assert min_self_clearance(poses.points(tight)[:, :2], sc["arc_skip_mm"], sc["sample_mm"]) < sc["min_clearance_mm"]


@pytest.mark.parametrize("angle, height", [(55, 147), (65, 163), (85, 179)])
def test_rear_up_height(cfg: dict, angle: float, height: float) -> None:
    """鎌首の高さ = (J7→頭先端 180mm) × sin(J7)。"""
    h = Poses(cfg).head_height_mm(Poses(cfg).rear_up(angle))
    assert h == pytest.approx(height, abs=1.0)


def test_head_look_limits_neck_to_look_range(cfg: dict) -> None:
    poses = Poses(cfg)
    nk = cfg["neck"]
    assert poses.head_look(10, 0, 90)["J7"] == nk["look_max_deg"]
    assert poses.head_look(10, 0, 20)["J7"] == nk["look_min_deg"]
    assert "J7" not in poses.head_look(10, 5)
    look = poses.head_look(30, 15)
    assert look == {"J8": 30, "J9": 15}
    assert poses.full_rear_up()["J7"] == nk["full_rear_min_deg"]


def test_relax_command(cfg: dict) -> None:
    r = Poses(cfg).relax()
    assert r.torque_ratio == cfg["poses"]["relax"]["torque_ratio"]
    assert r.duration_s == cfg["poses"]["relax"]["duration_s"]
    assert r.angles["J7"] == cfg["poses"]["relax"]["neck_deg"]
