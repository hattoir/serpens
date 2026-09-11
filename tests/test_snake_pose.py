"""ヘビの位置姿勢推定（θ_body の生値とローパス、θ_head、片方しか見えないとき）のテスト。"""
from __future__ import annotations

import math

import numpy as np
import pytest

from serpens.config import load_config
from serpens.motion.gait import GaitEngine
from serpens.perception.snake_pose import SnakePoseTracker, wrap_pi
from serpens.sim.world import BodyPose, World


@pytest.fixture()
def cfg() -> dict:
    return load_config()


def test_tau_is_one_gait_period(cfg: dict) -> None:
    tr = SnakePoseTracker(cfg)
    f = cfg["gait"]["presets"][cfg["snake_pose"]["reference_gait"]]["temporal_freq_hz"]
    assert tr.tau_s == pytest.approx(cfg["snake_pose"]["heading_tau_periods"] / f)


def test_lowpass_suppresses_serpentine_wobble(cfg: dict) -> None:
    """蛇行中、生の θ_body は大きく振れるが、ローパス後は振れが小さい。"""
    dt = 0.02
    w, eng, tr = World(cfg, BodyPose(100.0, 600.0, 0.0)), GaitEngine(cfg), SnakePoseTracker(cfg)
    eng.start("forward")
    raw, filt, t = [], [], 0.0
    for _ in range(int(6 / dt)):
        t += dt
        ang = eng.update(t)
        for _ in range(2):
            w.step(ang, dt / 2)
        p = tr.update(t, w.marker_xy("neck"), w.marker_xy("tail"), ang.get("J8", 0.0))
        raw.append(p.theta_body_raw)
        filt.append(p.theta_body)
    raw_a, filt_a = np.array(raw[-100:]), np.array(filt[-100:])
    assert np.ptp(filt_a) < 0.5 * np.ptp(raw_a)


def test_theta_head_adds_j8(cfg: dict) -> None:
    tr = SnakePoseTracker(cfg)
    p = tr.update(0.0, np.array([600.0, 0.0]), np.array([0.0, 0.0]), 30.0)
    assert p.theta_body == pytest.approx(0.0)
    assert p.theta_head == pytest.approx(math.radians(30))


def test_one_marker_cases(cfg: dict) -> None:
    """首だけ → 位置更新・向き保持。尾だけ → 前回の距離と向きで首を推定。両方× → 前回値保持。"""
    tr = SnakePoseTracker(cfg)
    assert tr.update(0.0, np.array([600.0, 0.0]), None, 0.0) is None     # 最初は向き不明
    tr.update(0.1, np.array([600.0, 100.0]), np.array([0.0, 100.0]), 0.0)
    p = tr.update(0.2, np.array([650.0, 120.0]), None, 0.0)
    assert (p.x, p.y) == (650.0, 120.0) and p.theta_body_raw == pytest.approx(0.0) and not p.tail_seen
    p = tr.update(0.3, None, np.array([10.0, 100.0]), 0.0)
    assert p.estimated and (p.x, p.y) == pytest.approx((610.0, 100.0))
    p = tr.update(0.4, None, None, 10.0)
    assert (p.x, p.y) == pytest.approx((610.0, 100.0)) and p.t == 0.3 and not p.neck_seen
    assert tr.stale(0.3 + cfg["snake_pose"]["stale_after_s"] + 0.1)
    assert not tr.stale(0.35)


def test_wrap_and_lowpass_across_pi(cfg: dict) -> None:
    """±π をまたいでもローパスが反対側へ飛ばない。"""
    tr = SnakePoseTracker(cfg)
    for k, th in enumerate([math.pi - 0.05, -math.pi + 0.05] * 5):
        p = tr.update(k * 0.02, np.array([math.cos(th), math.sin(th)]) * 600, np.zeros(2), 0.0)
    assert abs(wrap_pi(p.theta_body - math.pi)) < 0.1
