"""Phase 1: 実機接続の配線・終了処理・開始条件・状態の鮮度・補助ツールの検証。

  - 実機用の構築経路で、監視・指令・トルク操作が**同じ接続先**を参照する
  - 例外・終了時に「停止要求 → join → 切断」が通り、失敗が報告される
  - 観測不明・期限切れでは**実機の自律走行を許可しない**
  - 古い温度やモック値を実機の正常状態として扱わない
  - servo_setup の q / s で**サーボ ID を書き換えない**
実機は動かさない（モックと偽通信のみ）。
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import pytest

from serpens import app as app_mod
from serpens.config import load_config
from serpens.hw.head_io import MockHeadIO
from serpens.runner import ControlLoop
from serpens.safety import AutonomyInputs, DriveState, autonomy_blockers
from serpens.sim.session import SimSession
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


# ---- 1. 参照の一致 -------------------------------------------------------------------
def test_injected_bus_is_shared_by_monitor_command_and_torque(cfg: dict) -> None:
    """注入したバスを、poller（監視）と expression（トルク操作）も使う。"""
    s, bus = make_session(cfg)
    assert s.bus is bus
    assert s.poller.bus is bus                       # 状態監視
    assert s.brain.expr.bus is bus                   # トルク操作（脱力）
    s.step()
    assert bus.goal_calls, "位置指令が注入したバスへ届いていない"


def test_real_build_path_shares_one_bus_and_no_mock_head(cfg: dict, monkeypatch: pytest.MonkeyPatch) -> None:
    """--bus feetech の構築経路で、全参照が一致し、頭部にモックを使わない。"""
    fake = RecordingBus(cfg)
    fake.connect()
    monkeypatch.setattr(app_mod, "_open_real_bus", lambda cfg_, port: fake)
    args = argparse.Namespace(bus="feetech", port="FAKE", head_port=None, camera=None, seed=1,
                              robot="direct", link_port=None)
    cfg_out, session, loop, (source, cam) = app_mod.build(args)
    try:
        assert session.bus is fake
        assert session.poller.bus is fake
        assert session.brain.expr.bus is fake
        assert session.head is None, "実機経路で MockHeadIO を実センサーとして使っている"
        assert not isinstance(session.head, MockHeadIO)
        assert session.robot_is_real and session.stop.state is DriveState.HOLD  # 実機は待機から
    finally:
        cam.release()
        loop.shutdown()


# ---- 4. 例外・終了時の後処理 ---------------------------------------------------------
def test_control_loop_exception_triggers_emergency_and_is_reported(cfg: dict) -> None:
    """制御スレッドの例外で緊急停止し、理由が fault と shutdown の戻り値に出る。"""
    s, bus = make_session(cfg)
    calls = {"n": 0}
    real_step = s.step

    def boom() -> None:
        calls["n"] += 1
        if calls["n"] > 3:
            raise RuntimeError("テストの故障")
        real_step()

    s.step = boom                                     # type: ignore[method-assign]
    loop = ControlLoop(s, realtime=False)
    loop.start()
    deadline = time.monotonic() + 10.0                # 全体テストと同時に走っても足りる余裕
    while not loop.fault and time.monotonic() < deadline:
        time.sleep(0.01)
    assert "テストの故障" in loop.fault, "例外が fault として記録されていない"
    assert s.stop.latched and s.stop.state is DriveState.EMERGENCY
    assert loop.latest().fault == loop.fault          # GUI へ伝わる
    errors = loop.shutdown()
    assert any("テストの故障" in e for e in errors)
    assert not loop.is_alive()
    assert bus.disconnects == [False], "切断時に既定で脱力している（config: disable_torque_on_exit）"


def test_shutdown_stops_joins_and_disconnects(cfg: dict) -> None:
    """正常終了でも 停止 → join → 切断 を通る。"""
    s, bus = make_session(cfg)
    loop = ControlLoop(s, realtime=False)
    loop.start()
    time.sleep(0.05)
    errors = loop.shutdown()
    assert errors == []
    assert not loop.is_alive()
    assert bus.disconnects and not bus._connected
    assert s.stop.state is DriveState.HOLD


def test_build_failure_closes_what_was_opened(cfg: dict, monkeypatch: pytest.MonkeyPatch) -> None:
    """構築途中で失敗したら、開いたバスを閉じてから例外を返す。"""
    fake = RecordingBus(cfg)
    fake.connect()
    monkeypatch.setattr(app_mod, "_open_real_bus", lambda cfg_, port: fake)
    monkeypatch.setattr(app_mod, "SimSession", lambda *a, **k: (_ for _ in ()).throw(RuntimeError("構築失敗")))
    args = argparse.Namespace(bus="feetech", port="FAKE", head_port=None, camera=None, seed=1,
                              robot="direct", link_port=None)
    with pytest.raises(RuntimeError, match="構築失敗"):
        app_mod.build(args)
    assert fake.disconnects == [False] and not fake._connected


def test_sim_camera_release_stops_thread(cfg: dict) -> None:
    s, _bus = make_session(cfg)
    cam = app_mod.SimCamera(cfg, s)
    cam.start()
    time.sleep(0.05)
    cam.release()
    assert not cam.is_alive()


# ---- 5. 実機の自律走行の開始条件 -------------------------------------------------------
def test_real_session_starts_in_hold_and_refuses_to_start(cfg: dict) -> None:
    """実機は待機から始まり、実観測・校正・駆動リンクが無い間は開始できない。"""
    s, bus = make_session(cfg, real=True)
    assert s.stop.state is DriveState.HOLD
    for _ in range(10):
        s.step()
    ok, why = s.request_start("test")
    assert not ok
    joined = " / ".join(why)
    assert "自己位置" in joined and "駆動リンク" in joined
    assert not s.stop.moving_allowed
    assert bus.goal_calls, "停止中でも保持指令は出る"
    assert all(len(g) == len(bus.ids) for g in bus.goal_calls)


def test_sim_session_keeps_working(cfg: dict) -> None:
    """シミュレーションのデモは従来どおり動く（開始条件で止めない）。"""
    s, _bus = make_session(cfg)
    assert s.blockers() == [] and s.stop.moving_allowed
    for _ in range(50):
        s.step()
    assert s.status is not None and s.anim.gait.active


def test_autonomy_blockers_list_reasons(cfg: dict) -> None:
    """不足している条件が個別に言葉で出る。"""
    bad = AutonomyInputs(True, "sim", None, False, False, False, 0, 9, None)
    why = autonomy_blockers(cfg, bad)
    assert len(why) == 5, why          # 自己位置・校正・駆動リンク・トルク上限・テレメトリ
    ok = AutonomyInputs(True, "aruco", 0.1, True, True, True, 9, 9, 0.2)
    assert autonomy_blockers(cfg, ok) == []
    stale = AutonomyInputs(True, "aruco", 5.0, True, True, True, 9, 9, 9.0)
    assert any("古い" in w for w in autonomy_blockers(cfg, stale))


# ---- 6. 状態の欠損・鮮度 --------------------------------------------------------------
def test_stale_telemetry_is_not_reported_as_normal(cfg: dict) -> None:
    """応答が止まったら、古い温度を最新として扱わない。"""
    clock = FakeClock()
    bus = RecordingBus(cfg, clock=clock, fail_after=1)   # 最初の1回だけ応答する
    bus.connect()
    s = SimSession(cfg, START, seed=1, bus=bus)
    s.clock = clock                                   # セッションと同じ時計にする
    s.poller._clock = clock
    s.step()
    assert s.poller.max_temperature_c(clock.t) is not None
    assert s.poller.missing_axes(clock.t) == []
    for _ in range(3):                                # 以後は応答なし（例外は数えるだけ）
        clock.t += float(cfg["behavior"]["safety"]["telemetry"]["stale_after_s"]) + 0.5
        s.poller.poll()
    assert s.poller.errors > 0 and "応答なし" in s.poller.last_error
    assert s.poller.max_temperature_c(clock.t) is None, "古い温度を現在値として返している"
    assert s.poller.missing_axes(clock.t) == bus.ids
    assert s.poller.fresh_states(clock.t) == {}
    assert s.poller.source == "RecordingBus"          # 値の出どころが分かる


def test_snapshot_exposes_freshness_and_drive_state(cfg: dict) -> None:
    """GUI へ渡す内容に、駆動状態・出どころ・古い軸・開始条件が入っている。"""
    s, _bus = make_session(cfg, real=True)
    loop = ControlLoop(s, realtime=False)
    s.step()
    loop._publish()
    snap = loop.latest()
    assert snap.drive_state == "HOLD" and "停止" in snap.drive_text
    assert snap.telemetry_source == "RecordingBus"
    assert snap.blockers and snap.head_link == "なし"
    assert set(snap.servo_age_s) == set(s.bus.ids)


# ---- 7. servo_setup の q / s -----------------------------------------------------------
def test_assign_ids_q_aborts_without_writing(cfg: dict, monkeypatch: pytest.MonkeyPatch) -> None:
    import servo_setup

    bus = RecordingBus(cfg)
    bus.connect()
    monkeypatch.setattr(servo_setup, "ask", lambda *a, **k: "q")
    servo_setup.assign_ids(bus, cfg)
    assert bus.id_changes == [], "q で中断したのに ID を書き換えた"
    assert bus.scans == [], "q で中断したのにスキャンした"


def test_assign_ids_s_skips_without_writing(cfg: dict, monkeypatch: pytest.MonkeyPatch) -> None:
    import servo_setup

    bus = RecordingBus(cfg)
    bus.connect()
    monkeypatch.setattr(servo_setup, "ask", lambda *a, **k: "s")
    servo_setup.assign_ids(bus, cfg)
    assert bus.id_changes == [] and bus.scans == []


def test_assign_ids_enter_does_write(cfg: dict, monkeypatch: pytest.MonkeyPatch) -> None:
    """Enter のときだけスキャンして ID を振る（1個だけ応答する状況を作る）。"""
    import servo_setup

    bus = RecordingBus(cfg)
    bus.connect()
    for sid in list(bus.ids):
        if sid != 5:
            bus._axes.pop(sid)
            bus.joints.pop(sid)
    monkeypatch.setattr(servo_setup, "ask", lambda *a, **k: "")
    servo_setup.assign_ids(bus, cfg)
    assert bus.id_changes and bus.id_changes[0][0] == 5
