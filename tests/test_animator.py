"""STEP 3: アニメーター（補間・呼吸・フリーズ）のテスト。"""
from __future__ import annotations

import numpy as np
import pytest

from serpens.config import load_config
from serpens.hw.mock_bus import MockServoBus
from serpens.motion.animator import Animator, Easing, Keyframe, ease_in_out, ease_out, overshoot_value
from serpens.motion.poses import Poses
from tests.helpers import FakeClock

DT = 0.02   # 50Hz


@pytest.fixture()
def cfg() -> dict:
    return load_config()


def run(anim: Animator, t0: float, t1: float) -> tuple[float, list[dict[str, float]]]:
    out, t = [], t0
    while t < t1 - 1e-9:
        out.append(anim.update(t))
        t += DT
    return t, out


def test_easing_endpoints_and_monotonic() -> None:
    ps = np.linspace(0, 1, 101)
    for f in (ease_in_out, ease_out):
        v = [f(p) for p in ps]
        assert v[0] == 0 and v[-1] == pytest.approx(1)
        assert all(b >= a for a, b in zip(v, v[1:]))
    assert ease_out(0.5) > ease_in_out(0.5) - 1e-9   # ease-out は前半が速い


def test_overshoot_peaks_then_returns() -> None:
    v = [overshoot_value(0.0, 40.0, p, 4.0, 0.75) for p in np.linspace(0, 1, 401)]
    assert max(v) == pytest.approx(44.0, abs=0.01)
    assert v[-1] == pytest.approx(40.0)
    neg = [overshoot_value(10.0, -20.0, p, 3.0, 0.75) for p in np.linspace(0, 1, 401)]
    assert min(neg) == pytest.approx(-23.0, abs=0.01)


def test_keyframe_reaches_target(cfg: dict) -> None:
    anim = Animator(cfg)
    anim.set_breathing(False)
    anim.set_pose_now({n: 0.0 for n in anim.names})
    anim._breath_env = 0.0
    anim.play(Keyframe({"J8": 40.0}, 1.0, Easing.OUT_OVERSHOOT, 4.0), 0.0)
    t, trace = run(anim, 0.0, 1.5)
    j8 = [p["J8"] for p in trace]
    assert max(j8) == pytest.approx(44.0, abs=0.3)
    assert j8[-1] == pytest.approx(40.0)
    assert not anim.busy(t)


def test_too_fast_keyframe_is_stretched(cfg: dict) -> None:
    """60° を 0.1 秒で、は無理なので最高角速度に収まるよう時間が延びる。"""
    anim = Animator(cfg)
    vmax = cfg["animator"]["max_joint_speed_dps"]
    dur = anim.play(Keyframe({"J8": 60.0}, 0.1), 0.0)
    assert dur == pytest.approx(3 * 60.0 / vmax)
    assert anim.play(Keyframe({"J8": 61.0}, 2.0), 0.0) == 2.0     # 十分遅ければそのまま


def test_breathing_on_all_axes(cfg: dict) -> None:
    """呼吸: 全軸が ±2°・周期 4 秒で振れる。OFF にすると止まる。

    ただしリミットは越えない（J7 は下限 0° = 頭が床なので、ホーム姿勢では下側が切れる）。
    """
    b = cfg["breath"]
    anim = Animator(cfg)
    t, trace = run(anim, 0.0, 2 * b["period_s"])
    arr = np.array([[p[n] for n in anim.names] for p in trace])
    lo = np.array([max(-b["amplitude_deg"], j["min_deg"]) for j in cfg["joints"]])
    assert np.allclose(arr.max(axis=0), b["amplitude_deg"], atol=0.05)
    assert np.allclose(arr.min(axis=0), lo, atol=0.05)
    k = int(b["period_s"] / DT)
    assert np.allclose(arr[10], arr[10 + k], atol=1e-6)      # 周期 4 秒
    anim.set_breathing(False)
    t, trace = run(anim, t, t + b["fade_s"] + 1.0)
    tail = np.array([[p[n] for n in anim.names] for p in trace[-20:]])
    assert np.allclose(tail, 0.0, atol=1e-9)


def test_freeze_holds_everything_and_resumes_smoothly(cfg: dict) -> None:
    anim = Animator(cfg)
    anim.gait.start("forward")
    t, _ = run(anim, 0.0, 3.0)
    before = anim.update(t)
    anim.freeze(t)
    t2, held = run(anim, t + DT, t + 0.3)
    assert all(h == before for h in held)                     # 呼吸も含めて完全停止
    anim.unfreeze(t2)
    after = anim.update(t2)
    assert max(abs(after[n] - before[n]) for n in anim.names) < 3.0   # 続きから（飛ばない）


def test_scenario_9axis_sequence_is_reasonable(cfg: dict) -> None:
    """home → 前進 → 停止 → とぐろ → 鎌首 → 左を見る → 脱力 の角度列が、
    リミット内・なめらか（50Hz 1周期の変化がサーボ最高速以下）で、各段階で期待どおり変化する。"""
    poses = Poses(cfg)
    anim = Animator(cfg)
    vmax = cfg["servo"]["speed_max_step_s"] * 360 / cfg["servo"]["steps_per_rev"]
    log: list[dict[str, float]] = []
    t = 0.0

    def seg(dur: float) -> list[dict[str, float]]:
        nonlocal t
        t, tr = run(anim, t, t + dur)
        log.extend(tr)
        return tr

    anim.play(Keyframe(poses.home(), 0.5), t); seg(1.0)
    anim.gait.start("forward"); walk = seg(4.0)
    anim.gait.stop(); seg(1.5)
    anim.play(Keyframe(poses.coil(), 2.0), t); coil = seg(2.5)
    anim.play(Keyframe(poses.rear_up(60), 1.5), t); rear = seg(2.0)
    anim.play(Keyframe(poses.head_look(40, 15), 0.8, Easing.OUT_OVERSHOOT, 4.0), t); look = seg(1.5)
    anim.play(Keyframe(poses.relax().angles, 0.5), t); relax = seg(1.0)

    lim = {j["name"]: (j["min_deg"], j["max_deg"]) for j in cfg["joints"]}
    arr = np.array([[p[n] for n in anim.names] for p in log])
    for i, n in enumerate(anim.names):
        assert arr[:, i].min() >= lim[n][0] and arr[:, i].max() <= lim[n][1]
    assert np.abs(np.diff(arr, axis=0)).max() <= vmax * DT
    assert max(abs(p["J3"]) for p in walk[-50:]) > 20           # 歩いている
    assert all(abs(p["J7"]) < 3 for p in walk)                   # 首は動かさない
    assert coil[-1]["J1"] == pytest.approx(poses.coil()["J1"], abs=2.5)
    assert rear[-1]["J7"] == pytest.approx(60, abs=2.5)
    assert look[-1]["J8"] == pytest.approx(40, abs=2.5) and max(p["J8"] for p in look) > 42
    assert relax[-1]["J7"] == pytest.approx(cfg["poses"]["relax"]["neck_deg"], abs=2.5)


def test_send_to_mock_bus(cfg: dict) -> None:
    clock = FakeClock()
    bus = MockServoBus(cfg, clock=clock)
    bus.connect()
    for s in bus.ids:
        bus.set_torque(s, True)
    anim = Animator(cfg)
    anim.set_pose_now(Poses(cfg).coil())
    anim.send(bus, anim.update(0.0))
    clock.advance(3.0)
    assert bus.read_state(1).pos_deg == pytest.approx(Poses(cfg).coil()["J1"], abs=2.5)
