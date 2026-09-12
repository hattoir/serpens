"""Phase 1: 停止の分離の検証（通常停止 / 緊急停止 / シミュレーション一時停止）。

  - 通常停止は歩容を即時に止め、**保持指令を出力先まで送り続ける**。監視は続く
  - 停止で勝手にホーム姿勢へ動かさない。脱力は明示操作のときだけ
  - 緊急停止はラッチし、通常操作や**予約済みの演出では解除・再始動しない**
実機は動かさない（モックと偽通信のみ）。
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import pytest

from serpens import keys
from serpens.config import load_config
from serpens.runner import ControlLoop
from serpens.safety import DriveState, StopSupervisor
from serpens.sim.session import SimSession
from serpens.sim.virtual_camera import SimPerson
from serpens.sim.world import BodyPose
from tests.helpers import FakeClock, RecordingBus

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

START = BodyPose(200.0, 400.0, 0.0)


@pytest.fixture()
def cfg() -> dict:
    return load_config()


def make_session(cfg: dict, real: bool = False, fail_after: int | None = None,
                 head: object | None = None) -> tuple[SimSession, RecordingBus]:
    bus = RecordingBus(cfg, fail_after=fail_after)
    bus.connect()
    for sid in bus.ids:
        bus.set_torque(sid, True)
    s = SimSession(cfg, START, seed=1, bus=bus, head=head, robot_is_real=real)
    bus.goal_calls.clear()
    bus.torque_calls.clear()
    return s, bus


# ---- 2. 停止が出力先まで届く ----------------------------------------------------------
def test_normal_stop_reaches_the_bus_and_keeps_monitoring(cfg: dict) -> None:
    """通常停止: 歩容は即時停止、保持指令は送り続ける、脱力はしない、監視は続く。"""
    s, bus = make_session(cfg)
    for _ in range(20):
        s.step()
    assert s.anim.gait.active
    loop = ControlLoop(s, realtime=False)
    loop.request_stop("テスト: 通常停止")
    assert not s.anim.gait.active, "停止しても歩容が動いている"
    sent_before, reads_before = len(bus.goal_calls), bus.read_calls
    torque_before = list(bus.torque_calls)
    poses = []
    for _ in range(60):
        s.step()
        poses.append(tuple(round(v, 6) for v in s.anim.last_output.values()))
    assert len(bus.goal_calls) > sent_before, "停止中に保持指令が出力へ届いていない"
    assert bus.read_calls > reads_before, "停止中に監視が止まっている"
    assert bus.torque_calls == torque_before, "通常停止で勝手に脱力している"
    assert len(set(poses)) == 1, "停止中に姿勢が動いている"


def test_disable_torque_only_when_asked(cfg: dict) -> None:
    """脱力は明示操作のときだけ。走行中には受け付けない。"""
    s, bus = make_session(cfg)
    s.step()
    assert not s.request_disable_torque("テスト", "test"), "走行中に脱力を受け付けた"
    s.request_stop("テスト", "test")
    assert s.request_disable_torque("テスト: 明示操作", "test")
    s.enforce_stop_output()
    assert (1, False) in bus.torque_calls and s.stop.state is DriveState.DISABLED


def test_stop_does_not_move_to_home(cfg: dict) -> None:
    """停止でホーム姿勢へ動かさない（現在姿勢のまま保持する）。"""
    s, bus = make_session(cfg)
    for _ in range(40):
        s.step()
    pose_at_stop = dict(s.anim.last_output)
    s.request_stop("テスト", "test")
    for _ in range(40):
        s.step()
    home = s.brain.poses.home()
    moved = {k: (pose_at_stop[k], v) for k, v in s.anim.last_output.items() if abs(v - pose_at_stop[k]) > 1e-6}
    assert not moved, f"停止後に姿勢が変わった: {moved}"
    assert any(abs(pose_at_stop[k] - home.get(k, 0.0)) > 1.0 for k in home), "そもそもホームと同じ姿勢だった"


# ---- 3. 演出の再始動と緊急停止のラッチ ------------------------------------------------
def test_scheduled_expression_cannot_restart_motion(cfg: dict) -> None:
    """停止時に予約済みの演出を捨て、解除後も勝手に動き出さない。"""
    s, bus = make_session(cfg)
    s.people = [SimPerson(600.0, -400.0)]
    for _ in range(30):                              # 反応の予約（react / look など）が入る
        s.step()
    s.request_stop("テスト", "test")
    s.enforce_stop_output()
    assert s.brain.expr._events == [], "予約済みの演出が残っている"
    held = tuple(round(v, 6) for v in s.anim.last_output.values())
    for _ in range(200):                             # 4秒ぶん（反応遅延 0.4〜0.9s を十分に超える）
        s.step()
    assert tuple(round(v, 6) for v in s.anim.last_output.values()) == held


def test_emergency_latches_and_normal_keys_do_not_release(cfg: dict) -> None:
    """緊急停止はラッチし、通常操作（G/R/H/D/S）では解除されない。"""
    s, bus = make_session(cfg)
    loop = ControlLoop(s, realtime=False)
    for _ in range(10):
        s.step()
    loop.request_emergency("テスト: 緊急停止")
    assert s.stop.latched and s.stop.state is DriveState.EMERGENCY
    for k in ("g", "r", "h", "d", "s"):
        keys.handle(loop, k)
        assert s.stop.latched, f"{k} キーでラッチが外れた"
        assert not s.stop.moving_allowed
    pose = tuple(round(v, 6) for v in s.anim.last_output.values())
    for _ in range(100):
        s.step()
    assert tuple(round(v, 6) for v in s.anim.last_output.values()) == pose
    msg = keys.handle(loop, "u")                      # 明示解除 → 待機（走行しない）
    assert "解除" in msg and not s.stop.latched
    assert s.stop.state is DriveState.HOLD and not s.stop.moving_allowed
    assert "開始" in keys.handle(loop, "g")           # シミュレーションは開始できる
    assert s.stop.moving_allowed


def test_stop_supervisor_rules(cfg: dict) -> None:
    """停止の規則そのもの（ラッチ中は stop / disable_torque / start を受け付けない）。"""
    sup = StopSupervisor(cfg, FakeClock(), initial=DriveState.RUN)
    assert sup.moving_allowed
    sup.emergency("テスト", "test")
    assert not sup.stop("通常停止", "test") and not sup.disable_torque("脱力", "test")
    ok, why = sup.start([], "test")
    assert not ok and "ラッチ" in why[0]
    assert sup.clear_emergency("test") and sup.state is DriveState.HOLD
    assert not sup.moving_allowed
    assert sup.start([], "test")[0] and sup.moving_allowed
    assert [e.state for e in sup.events][:3] == ["RUN", "EMERGENCY", "HOLD"]
