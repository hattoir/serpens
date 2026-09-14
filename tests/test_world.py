"""STEP 4: 2D シミュレータのテスト。"""
from __future__ import annotations

import copy
import math

import numpy as np
import pytest

from serpens.config import load_config
from serpens.motion.animator import Animator, coil_keyframe
from serpens.motion.gait import GaitEngine, GaitParams, gait_period_s  # noqa: F401
from serpens.motion.kinematics import min_self_clearance
from serpens.motion.poses import Poses
from serpens.sim.world import BodyPose, World

WARMUP_CYCLES = 3
MEASURE_CYCLES = 2


@pytest.fixture()
def cfg() -> dict:
    return load_config()


@pytest.fixture()
def big(cfg: dict) -> dict:
    """マット端に当たらないよう、マットだけ広くした設定（推進量の測定用）。"""
    c = copy.deepcopy(cfg)
    c["mat"]["width_mm"] = c["mat"]["depth_mm"] = 1.0e5
    return c


def walk(cfg: dict, gait: str | GaitParams, start: BodyPose) -> tuple[World, float, float, np.ndarray]:
    """助走してから MEASURE_CYCLES 周期歩かせる。戻り値: (world, 1周期の前進量, 1周期の回転[deg], 進行方向)"""
    dt = cfg["sim"]["dt_s"]
    w, eng = World(cfg, start), GaitEngine(cfg)
    eng.start(gait)
    T = gait_period_s(eng.params)
    t = 0.0
    for _ in range(int(WARMUP_CYCLES * T / dt)):
        t += dt
        w.step(eng.update(t), dt)
    c0, th0 = w.centroid(), w.pose.theta
    for _ in range(int(round(MEASURE_CYCLES * T / dt))):
        t += dt
        w.step(eng.update(t), dt)
    d = (w.centroid() - c0) / MEASURE_CYCLES
    return w, float(np.linalg.norm(d)), math.degrees(w.pose.theta - th0) / MEASURE_CYCLES, d


def test_straight_body_does_not_move(cfg: dict) -> None:
    w = World(cfg)
    before = w.world_points().copy()
    for _ in range(100):
        w.step({}, cfg["sim"]["dt_s"])
    assert np.allclose(w.world_points(), before)


def test_forward_moves_toward_head(big: dict) -> None:
    """前進歩容で頭の方向（+x）へ進む。1周期あたりの前進量は胴体の波長程度（滑りなしの理想値）。"""
    w, per_cycle, dth, d = walk(big, "forward", BodyPose(5e4, 5e4, 0.0))
    assert d[0] > 0 and abs(d[1]) < 0.1 * d[0]
    assert abs(dth) < 0.5
    wavelength = 360.0 / big["gait"]["presets"]["forward"]["spatial_freq_deg"] * 95.0
    assert 0.6 * wavelength < per_cycle < wavelength


def test_backward_moves_toward_tail(big: dict) -> None:
    _, _, _, d = walk(big, "backward", BodyPose(5e4, 5e4, 0.0))
    assert d[0] < 0


@pytest.mark.parametrize("gait, sign", [("turn_left", 1), ("turn_right", -1)])
def test_turning(big: dict, gait: str, sign: int) -> None:
    """turn_bias_deg > 0 で左（反時計回り）に、< 0 で右に曲がる。"""
    _, _, dth, _ = walk(big, gait, BodyPose(5e4, 5e4, 0.0))
    assert sign * dth > 10.0


def test_stays_inside_mat(cfg: dict) -> None:
    """壁に向かって歩き続けても、胴体はマットの外に出ない。"""
    w, *_ = walk(cfg, "forward", BodyPose(200.0, 600.0, 0.0))
    for _ in range(3):
        w2, *_ = walk(cfg, "turn_left", w.pose)
        w = w2
    pts = w.world_points()
    r = cfg["body"]["diameter_mm"] / 2
    assert pts[:, 0].min() >= r - 1e-6 and pts[:, 0].max() <= cfg["mat"]["width_mm"] - r + 1e-6
    assert pts[:, 1].min() >= r - 1e-6 and pts[:, 1].max() <= cfg["mat"]["depth_mm"] - r + 1e-6


def test_coil_in_sim(cfg: dict) -> None:
    """まっすぐ → とぐろ（尾から順）。巻き終わった形は自己干渉がなく、マット内にある。"""
    dt = cfg["sim"]["dt_s"]
    w, anim, poses = World(cfg, BodyPose(350.0, 400.0, 0.0)), Animator(cfg), Poses(cfg)
    anim.set_breathing(False)
    anim._breath_env = 0.0
    dur = anim.play(coil_keyframe(poses), 0.0)
    t = 0.0
    while t < dur + 0.3:
        t += dt
        w.step(anim.update(t), dt)
    sc = cfg["self_collision"]
    pts = w.world_points()
    assert min_self_clearance(pts[:, :2], sc["arc_skip_mm"], sc["sample_mm"]) >= sc["min_clearance_mm"]
    assert 0 <= pts[:, 0].min() and pts[:, 0].max() <= cfg["mat"]["width_mm"]
    assert 0 <= pts[:, 1].min() and pts[:, 1].max() <= cfg["mat"]["depth_mm"]


def test_tangential_drag_limits_gliding(big: dict) -> None:
    """転がり抵抗 0.02 で、全関節同時のとぐろ化による「滑走」が減り、前進の低下は 2 割未満。"""
    from serpens.motion.animator import ease_in_out

    coil = Poses(big).coil()
    dt = big["sim"]["dt_s"]
    travel, fwd = [], []
    for ratio in (0.0, 0.02):
        c = copy.deepcopy(big)
        c["belly"]["profiles"]["wheel"]["MEDIUM"]["tangential"] = ratio
        w = World(c, BodyPose(5e4, 5e4, 0.0))
        c0 = w.centroid()
        for k in range(301):
            w.step({j: v * ease_in_out(k / 300) for j, v in coil.items()}, dt)
        travel.append(float(np.linalg.norm(w.centroid() - c0)))
        fwd.append(walk(c, "forward", BodyPose(5e4, 5e4, 0.0))[1])
    assert big["belly"]["profiles"]["wheel"]["MEDIUM"]["tangential"] == 0.02
    assert travel[1] < 0.7 * travel[0]       # 14輪+頭パッドでは 394mm → 226mm（2026-09-12 測定）
    assert fwd[1] > 0.8 * fwd[0]


def test_head_pad_is_isotropic_and_lifts_off(cfg: dict) -> None:
    """頭部はパッド（等方の軽い摩擦）。床にあれば頭を振ると胴体が少し動き、鎌首で浮けば動かない。"""
    dt = cfg["sim"]["dt_s"]
    assert 7 not in cfg["sim"]["wheel_links"] and cfg["sim"]["pad_links"] == [9]
    moved = []
    for neck in (60.0, 0.0):
        w = World(cfg, BodyPose(200.0, 600.0, 0.0))
        w.step({"J7": neck}, dt)
        p0 = w.pose
        for k in range(200):
            w.step({"J7": neck, "J8": 40.0 * math.sin(k * dt * 2 * math.pi)}, dt)
        moved.append(math.hypot(w.pose.x - p0.x, w.pose.y - p0.y) + abs(w.pose.theta - p0.theta) * 1000)
    assert moved[0] < 1e-6
    assert moved[1] > 0.1


def test_snake_pose_definition(cfg: dict) -> None:
    """生の (x, y, θ_body) = 首マーカの位置と、尾マーカ→首マーカの向き。首マーカは J7 より胴体側。"""
    w = World(cfg, BodyPose(100.0, 200.0, math.radians(90)))
    x, y, th = w.snake_pose()
    m = cfg["markers"]
    j7 = [j["x_mm"] for j in cfg["joints"] if j["name"] == "J7"][0]
    assert m["neck_x_mm"] < j7
    assert (x, y) == pytest.approx((100.0, 200.0 + m["neck_x_mm"]))
    assert tuple(w.marker_xy("tail")) == pytest.approx((100.0, 200.0 + m["tail_x_mm"]))
    assert th == pytest.approx(math.radians(90))
    w.step({"J7": 70.0}, cfg["sim"]["dt_s"])            # 鎌首を上げても首マーカは動かない
    assert w.snake_pose()[:2] == pytest.approx((x, y))
