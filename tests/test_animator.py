"""STEP 3: アニメーター（補間・呼吸・フリーズ）のテスト。"""
from __future__ import annotations

import numpy as np
import pytest

from serpens.config import load_config
from serpens.hw.mock_bus import MockServoBus
from serpens.motion.animator import (Animator, Easing, Keyframe, rest_keyframe, ease_in_out, ease_out,
                                     overshoot_value)
from serpens.motion.poses import Poses
from tests.helpers import FakeClock

DT = 0.02   # 50Hz


@pytest.fixture()
def cfg() -> dict:
    return load_config()


def vmax(cfg: dict) -> dict[str, float]:
    return {j["name"]: j["max_speed_dps"] for j in cfg["joints"]}


def run(anim: Animator, t0: float, t1: float) -> tuple[float, list[dict[str, float]]]:
    out, t = [], t0
    while t < t1 - 1e-9:
        out.append(anim.update(t))
        t += DT
    return t, out


def quiet(cfg: dict) -> Animator:
    """呼吸なしのアニメーター。"""
    anim = Animator(cfg)
    anim.set_breathing(False)
    anim._breath_env = 0.0
    return anim


def test_easing_endpoints_and_monotonic() -> None:
    ps = np.linspace(0, 1, 101)
    for f in (ease_in_out, ease_out):
        v = [f(p) for p in ps]
        assert v[0] == 0 and v[-1] == pytest.approx(1)
        assert all(b >= a for a, b in zip(v, v[1:]))
    assert ease_out(0.5) > ease_in_out(0.5) - 1e-9


def test_overshoot_peaks_then_returns() -> None:
    v = [overshoot_value(0.0, 40.0, p, 4.0, 0.75) for p in np.linspace(0, 1, 401)]
    assert max(v) == pytest.approx(44.0, abs=0.01)
    assert v[-1] == pytest.approx(40.0)
    neg = [overshoot_value(10.0, -20.0, p, 3.0, 0.75) for p in np.linspace(0, 1, 401)]
    assert min(neg) == pytest.approx(-23.0, abs=0.01)


def test_keyframe_reaches_target_with_overshoot(cfg: dict) -> None:
    anim = quiet(cfg)
    dur = anim.play(Keyframe({"J8": 40.0}, 1.0, Easing.OUT_OVERSHOOT, 4.0), 0.0)
    t, trace = run(anim, 0.0, dur + 0.2)
    j8 = [p["J8"] for p in trace]
    assert max(j8) == pytest.approx(44.0, abs=0.3)
    assert j8[-1] == pytest.approx(40.0)
    assert not anim.busy(t)


def test_per_axis_speed_limit_stretches_keyframe(cfg: dict) -> None:
    """軸ごとの最高角速度: 頭（J8 90°/s）は胴体（J3 240°/s）より同じ角度でも時間がかかる。"""
    v = vmax(cfg)
    assert v["J3"] == 240 and v["J7"] == 120 and v["J8"] == 90 and v["J9"] == 90
    assert quiet(cfg).play(Keyframe({"J3": 60.0}, 0.1), 0.0) == pytest.approx(3 * 60 / v["J3"])
    assert quiet(cfg).play(Keyframe({"J8": 60.0}, 0.1), 0.0) == pytest.approx(3 * 60 / v["J8"])
    assert quiet(cfg).play(Keyframe({"J8": 30.0}, 2.0), 0.0) == 2.0


def test_staggered_coil_goes_tail_first(cfg: dict) -> None:
    """とぐろは尾(J1)から順に巻き、頭が最後。"""
    anim = quiet(cfg)
    kf = rest_keyframe(Poses(cfg))
    seq = cfg["poses"]["coil_sequence"]
    total = anim.play(kf, 0.0)
    assert total == pytest.approx(seq["per_joint_s"] + seq["stagger_s"] * (len(seq["order"]) - 1))
    _, trace = run(anim, 0.0, seq["stagger_s"] * 2.5)
    assert trace[-1]["J1"] > 20 and abs(trace[-1]["J4"]) < 1e-9      # J4 はまだ動いていない
    _, trace = run(anim, seq["stagger_s"] * 2.5, total + 0.1)
    assert trace[-1]["J6"] == pytest.approx(Poses(cfg).rest()["J6"])


def test_breathing_on_all_axes(cfg: dict) -> None:
    """呼吸: 各軸が軸ごとの振幅・周期 4 秒で振れる（J7 もホーム +8° なので両側に振れる）。
    位相は尾→頭へ phase_step_deg ずつ遅れる（同位相だと胴体が C 字に丸まって「震え」に見える）。OFF で止まる。"""
    b = cfg["breath"]
    home = cfg["poses"]["home"]
    anim = Animator(cfg)
    t, trace = run(anim, 0.0, 2 * b["period_s"])
    arr = np.array([[p[n] - home[n] for n in anim.names] for p in trace])
    amp = np.array([b["amplitude_by_axis"][n] for n in anim.names])
    assert np.allclose(arr.max(axis=0), amp, atol=0.05)
    assert np.allclose(arr.min(axis=0), -amp, atol=0.05)
    assert b["amplitude_by_axis"]["J7"] >= 2 * max(b["amplitude_by_axis"][n] for n in anim.names if n != "J7")
    lag = int(round(b["phase_step_deg"] / 360.0 * b["period_s"] / DT))
    assert 0 < lag and np.allclose(arr[lag:lag + 50, 0] / amp[0], arr[:50, 1] / amp[1], atol=0.02)  # J2 は J1 より遅れる
    k = int(b["period_s"] / DT)
    assert np.allclose(arr[10], arr[10 + k], atol=1e-6)
    anim.set_breathing(False)
    t, trace = run(anim, t, t + b["fade_s"] + 1.0)
    tail = np.array([[p[n] - home[n] for n in anim.names] for p in trace[-20:]])
    assert np.allclose(tail, 0.0, atol=1e-9)


def test_freeze_holds_everything_and_resumes_smoothly(cfg: dict) -> None:
    anim = Animator(cfg)
    anim.gait.start("forward")
    t, _ = run(anim, 0.0, 3.0)
    before = anim.update(t)
    anim.freeze(t)
    t2, held = run(anim, t + DT, t + 0.3)
    assert all(h == before for h in held)
    anim.unfreeze(t2)
    after = anim.update(t2)
    assert max(abs(after[n] - before[n]) for n in anim.names) < 3.0


def test_scenario_9axis_sequence_is_reasonable(cfg: dict) -> None:
    """home → 前進 → 停止 → とぐろ → 鎌首 → 左を見る → 脱力 の角度列が、
    ソフトリミット内・なめらか（各軸 50Hz 1周期の変化がその軸の上限速度＋歩容・呼吸ぶん以下）で、
    各段階で期待どおり変化する。"""
    poses = Poses(cfg)
    anim = Animator(cfg)
    log: list[dict[str, float]] = []
    t = 0.0

    def seg(dur: float) -> list[dict[str, float]]:
        nonlocal t
        t, tr = run(anim, t, t + dur)
        log.extend(tr)
        return tr

    seg(anim.play(Keyframe(poses.home(), 0.5), t) + 0.2)
    anim.gait.start("forward"); walk = seg(4.0)
    anim.gait.stop(); seg(1.5)
    coil = seg(anim.play(rest_keyframe(poses), t) + 0.2)
    rear = seg(anim.play(Keyframe(poses.rear_up(60), 1.5), t) + 0.2)
    look = seg(anim.play(Keyframe(poses.head_look(40, 15), 0.8, Easing.OUT_OVERSHOOT, 4.0), t) + 0.2)
    relax = seg(anim.play(Keyframe(poses.relax().angles, 0.5), t) + 0.2)

    lim = {j["name"]: (j["min_deg"], j["max_deg"]) for j in cfg["joints"]}
    arr = np.array([[p[n] for n in anim.names] for p in log])
    v = vmax(cfg)
    extra = 2 * np.pi * (max(cfg["breath"]["amplitude_by_axis"].values()) / cfg["breath"]["period_s"]
                         + cfg["gait"]["presets"]["forward"]["amplitude_deg"] * 0.5)
    for i, n in enumerate(anim.names):
        assert arr[:, i].min() >= lim[n][0] and arr[:, i].max() <= lim[n][1]
        assert np.abs(np.diff(arr[:, i])).max() <= (v[n] + extra) * DT, n
    assert max(abs(p["J3"]) for p in walk[-50:]) > 20
    assert all(abs(p["J7"] - 8) < cfg["breath"]["amplitude_by_axis"]["J7"] + 1 for p in walk)   # 前進中の首は呼吸ぶんだけ
    assert coil[-1]["J1"] == pytest.approx(poses.rest()["J1"], abs=2.5)
    neck_breath = cfg["breath"]["amplitude_by_axis"]["J7"] + 0.5      # J7 は呼吸で大きく上下する
    assert rear[-1]["J7"] == pytest.approx(60, abs=neck_breath)
    assert look[-1]["J8"] == pytest.approx(40, abs=2.5) and max(p["J8"] for p in look) > 42
    assert relax[-1]["J7"] == pytest.approx(cfg["poses"]["relax"]["neck_deg"], abs=neck_breath)


def test_send_uses_per_axis_speed(cfg: dict) -> None:
    clock = FakeClock()
    bus = MockServoBus(cfg, clock=clock)
    bus.connect()
    for s in bus.ids:
        bus.set_torque(s, True)
    anim = Animator(cfg)
    anim.set_pose_now({"J1": 60.0, "J8": 60.0})
    anim.send(bus, anim.update(0.0))
    clock.advance(0.3)
    st = bus.sync_read_states()
    assert st[1].pos_deg > st[8].pos_deg + 15           # 頭（90°/s）は胴体（240°/s）より遅い
    assert st[8].pos_deg <= vmax(cfg)["J8"] * 0.3 + 0.5


def test_anticipation_pulls_back_first_and_speed_limit_includes_it(cfg: dict) -> None:
    """予備動作: 本動作の前に逆へ引く。速度上限の計算に予備動作ぶんが入る（時間が延びる）。"""
    anim = quiet(cfg)
    kf = Keyframe({"J8": 40.0}, 0.6, Easing.OUT, anticipate_deg=6.0, anticipate_lead_s=0.2)
    total = anim.play(kf, 0.0)
    t, trace = run(anim, 0.0, total + 0.1)
    j8 = [p["J8"] for p in trace]
    assert min(j8[:10]) < -3.0                       # まず逆（負）へ
    assert j8[-1] == pytest.approx(40.0)
    v = vmax(cfg)["J8"]
    assert np.abs(np.diff(j8)).max() <= v * DT * 1.01
    plain = anim.play(Keyframe({"J8": 0.0}, 0.6, Easing.OUT), t)
    assert total > plain                              # 予備動作の時間ぶん長い


def test_settle_oscillates_and_decays_on_tail_joints(cfg: dict) -> None:
    """follow-through: 到達後に尾側の関節が減衰振動して止まる。振幅は軸の速度上限に収める。"""
    anim = quiet(cfg)
    kf = Keyframe({"J7": 60.0}, 0.8, Easing.IN_OUT, settle_amp_deg=5.0, settle_tau_s=0.3, settle_joints=("J1", "J2"))
    arrive = anim.play(kf, 0.0)
    _, trace = run(anim, arrive, arrive + 1.5)
    j1 = np.array([p["J1"] for p in trace])
    assert np.abs(j1[:15]).max() > 2.0                # 到達直後は揺れている
    assert np.abs(j1[-10:]).max() < 0.2               # 4τ 後には止まっている
    assert (np.diff(np.sign(j1[:40])) != 0).sum() >= 2   # 振動（符号が変わる）
    assert np.abs(np.diff(j1)).max() <= vmax(cfg)["J1"] * DT * 1.01
    assert all(p["J3"] == 0.0 for p in trace)         # 指定外の関節は揺れない


def test_head_rise_time_and_acceleration_are_bounded(cfg: dict) -> None:
    """頭の動きは立ち上がり 200ms 以上（5° 以上のとき）・先端加速度 1G 以下。2.5° のピクッは速いまま。"""
    a = cfg["animator"]
    anim = quiet(cfg)
    fast = anim.play(Keyframe({"J8": 40.0}, 0.05, Easing.OUT), 0.0)
    assert fast >= a["head_min_rise_s"]
    lever = cfg["body"]["head_tip_x_mm"] - next(j["x_mm"] for j in cfg["joints"] if j["name"] == "J7")
    t, trace = run(anim, 0.0, fast + 0.1)
    x = np.radians([p["J8"] for p in trace]) * lever / 1000.0
    accel = np.abs(np.diff(x, 2)).max() / DT ** 2
    assert accel <= a["head_max_accel_g"] * 9.80665 * 1.05
    anim.set_pose_now({"J8": 0.0})
    twitch = anim.play(Keyframe({"J8": 2.5}, 0.12, Easing.OUT), t)
    assert twitch < a["head_min_rise_s"]              # 微小な動きは下限を課さない
    body = anim.play(Keyframe({"J3": 10.0}, 0.05, Easing.OUT), t)
    assert body < a["head_min_rise_s"]                # 胴体は対象外（速度上限だけ）
