"""フェーズ 1 追加確認 8〜11（2026-09-26）: operator_resume は MQTT から到達しない / safety_state の周期送信と
受け側時計での失効 / 正常終了は自分で OFFLINE、再接続で今の状態を上書き。ハード不要（Loopback）。本物のブローカーは
tests/test_mqtt_live.py。
"""
from __future__ import annotations

from serpens.api.endpoint import Endpoint
from tests.mocks.home_ai_mock import SAFETY_MAX_AGE_MS
from tests.test_task_event_api import MAP, rig  # noqa: F401  (fixture)


def test_safety_state_is_published_periodically_under_the_receiver_deadline(rig) -> None:
    """9: 周期 < 受け側の失効。tick を回し続ければ受け側は常に fresh。"""
    broker, ep, ex, home, clock = rig
    assert ep.safety_period_ms < SAFETY_MAX_AGE_MS
    ep.safety_state("DISARMED", "BOOT", latched=False)
    n0 = home.safety_count
    for _ in range(20):
        clock.t += ep.safety_period_ms // 2
        ep.tick()
        assert home.safety_fresh()
    assert home.safety_count - n0 == 10                                            # 周期ごとに 1 回（半周期では出ない）


def test_operator_resume_is_unreachable_from_mqtt(rig) -> None:
    """8: 再開は人の操作だけ。Task に見せかけても、余分なフィールドでも拒否され、ロックは残る。"""
    broker, ep, ex, home, clock = rig
    home.task("stop")
    assert ep.locked_reason is not None
    for msg in ({"task": "operator_resume"}, {"task": "resume"}, {"task": "return_dock", "operator_resume": True},
                {"task": "clear_fault"}, {"task": "arm"}):
        tid = home.task(msg.pop("task"), **msg)
        assert home.statuses_of(tid) == ["rejected"] and "人の操作" in home.last_reason(tid)
        assert ep.locked_reason is not None
    assert ex.started == []
    ep.operator_resume()                                                            # 人の操作（MQTT ではない）
    assert ep.locked_reason is None


def test_graceful_close_publishes_offline_and_reconnect_overwrites_it(rig) -> None:
    """10: 正常終了は LWT が出ないので自分で OFFLINE を retain。再接続では今の状態で retain を上書き。"""
    broker, ep, ex, home, clock = rig
    ep.safety_state("DRIVING", "NONE", latched=False)
    ep.close()
    last = [p for p in broker.log if p.retain][-1].payload
    assert last["mode"] == "OFFLINE" and last["stop_reason"] == "OPERATOR" and home.safety["mode"] == "OFFLINE"
    ep2 = Endpoint(broker, ex, map_version=MAP, clock_ms=clock)                    # 立ち上げ直し（retain は OFFLINE のまま）
    ep2.safety_state("DISARMED", "BOOT", latched=False)
    broker.simulate_disconnect()                                                    # 落ちて OFFLINE が retain に残る
    assert home.safety["mode"] == "OFFLINE"
    broker.simulate_reconnect()                                                     # 戻った瞬間に今の状態で上書き
    assert home.safety["mode"] == "DISARMED" and [p for p in broker.log if p.retain][-1].payload["mode"] == "DISARMED"


def test_nothing_overwrites_the_offline_after_a_graceful_close(rig) -> None:
    """10 補: close の途中・後に再接続（announce）や周期送信（tick）が走っても、OFFLINE の後に古い状態を出さない。
    announce は last_safety が空だと BOOT を作って出すので、close が状態を捨てるだけでは防げない。"""
    broker, ep, ex, home, clock = rig
    ep.safety_state("DRIVING", "NONE", latched=False)
    ep.close()
    broker.simulate_reconnect()
    clock.t += ep.safety_period_ms * 3
    assert not ep.tick()
    last = [p for p in broker.log if p.retain][-1].payload
    assert last["mode"] == "OFFLINE" and home.safety["mode"] == "OFFLINE"
