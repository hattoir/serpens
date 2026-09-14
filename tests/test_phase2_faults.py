"""故障注入。**確かめたいのは「正常時に動く」ことではなく「異常時に勝手に走り続けない」こと。**

経路（loss / delay / 順序入れ替え）・サーボ（応答なし / 過熱 / 過負荷）・
指令値（範囲外 / NaN / Inf）・機体（制御周期の超過）の4層。
すべて SIMULATED。実機での挙動は `docs/verification_status.md` を見ること。
"""
from __future__ import annotations

import math

import pytest

from serpens.config import load_config
from serpens.link import messages as m
from serpens.link.harness import LinkHarness
from serpens.link.protocol import Flag, Nack, State, StopReason


@pytest.fixture()
def cfg() -> dict:
    return load_config()


def driving_harness(cfg: dict) -> LinkHarness:
    h = LinkHarness(cfg)
    assert h.start_driving(), "走行を開始できなかった"
    return h


def nacks(h: LinkHarness) -> set[Nack]:
    return {n[2] for n in h.client.nacks}


# ---- 経路 ------------------------------------------------------------------------------
def test_total_packet_loss_stops_the_machine(cfg: dict) -> None:
    """全部落ちたら止まる（TTL → heartbeat の二段）。"""
    h = driving_harness(cfg)
    h.tr.faults.drop_ratio = 1.0
    took = h.run_until(lambda: not h.device.driving, 2.0)
    assert took is not None and h.tr.faults.dropped > 0
    h.advance(1.0)
    assert h.device.state is State.FAULT_HOLD
    assert h.device.stop_reason is StopReason.HEARTBEAT_LOST
    assert h.moved_deg(0.5) < 1e-9


def test_partial_packet_loss_keeps_running(cfg: dict) -> None:
    """3割落ちても走り続ける（落ちるたびに止まっては展示にならない）。"""
    h = driving_harness(cfg)
    h.tr.faults.drop_ratio = 0.3
    h.advance(3.0)
    assert h.device.driving, "取りこぼしに弱すぎる"
    assert h.tr.faults.dropped > 0


def test_large_delay_is_treated_as_link_loss(cfg: dict) -> None:
    """遅延が heartbeat タイムアウトを超えたら、届いていても「古い」として止まる。"""
    h = driving_harness(cfg)
    h.tr.faults.delay_s = (cfg["link"]["heartbeat_timeout_ms"] / 1000.0) * 2.0
    took = h.run_until(lambda: not h.device.driving, 3.0)
    assert took is not None
    assert h.device.stop_reason in (StopReason.HEARTBEAT_LOST, StopReason.DRIVE_TTL)
    assert h.moved_deg(0.5) < 1e-9


def test_out_of_order_frames_are_rejected_not_applied(cfg: dict) -> None:
    """順序が入れ替わっても、古い方は実行されない（`seq` の判定）。"""
    h = driving_harness(cfg)
    h.tr.faults.reorder_next = 6
    h.advance(1.5)
    assert h.tr.faults.reordered > 0
    assert Nack.STALE_SEQ in nacks(h), "入れ替わった古いフレームが素通りした"
    assert h.device.last_seq is not None


def test_duplicate_and_corruption_together(cfg: dict) -> None:
    """重複と CRC 破損が同時に来ても、状態は壊れない。"""
    h = driving_harness(cfg)
    h.tr.faults.duplicate_next = 4                      # まず重複だけ
    h.advance(0.6)
    h.tr.faults.corrupt_next = 4                        # 次に CRC 破損だけ
    h.advance(0.6)
    assert {Nack.STALE_SEQ, Nack.BAD_CRC} <= nacks(h)
    assert h.device.state in (State.DRIVING, State.ARMED_HOLD)


# ---- サーボ ----------------------------------------------------------------------------
def test_servo_offline_stops_the_machine(cfg: dict) -> None:
    """軸が応答しなくなったら走行をやめる。**残りの軸で走り続けない。**"""
    h = driving_harness(cfg)
    h.device.inject_axis("J3", offline=True)
    took = h.run_until(lambda: not h.device.driving, 1.0)
    assert took is not None, "応答しない軸があるのに走り続けた"
    assert h.device.state is State.FAULT_HOLD
    assert h.device.stop_reason is StopReason.SERVO_FAULT
    h.advance(0.3)                                      # 新しいテレメトリが届くまで
    tel = h.client.telemetry
    assert tel is not None and len(tel.axes) == len(cfg["joints"]) - 1
    assert tel.flags & Flag.SERVO_MISSING


def test_offline_servo_blocks_restart_until_it_answers(cfg: dict) -> None:
    """応答が戻るまで待機へも戻さない（戻ったら DISARMED。走行には ARM が要る）。"""
    h = driving_harness(cfg)
    h.device.inject_axis("J3", offline=True)
    h.advance(0.5)
    h.client.arm(h.now)                                 # 人が開始しようとしても
    h.advance(0.3)
    assert h.device.state is State.FAULT_HOLD and Nack.BUSY in nacks(h)
    h.device.inject_axis("J3", offline=False)
    h.advance(0.5)
    assert h.device.state is State.DISARMED, "応答が戻ったら待機へ"
    assert not h.device.driving


def test_servo_overtemperature_latches(cfg: dict) -> None:
    """過熱はラッチ。冷えても自動では戻らない。"""
    h = driving_harness(cfg)
    h.device.inject_axis("J2", temp_c=cfg["link"]["faults"]["temp_limit_c"] + 5)
    h.advance(0.3)
    assert h.device.state is State.EMERGENCY_LATCHED
    assert h.device.stop_reason is StopReason.OVERHEAT
    h.device.inject_axis("J2", temp_c=25.0)
    h.advance(1.0)
    assert h.device.state is State.EMERGENCY_LATCHED, "冷えたら勝手に復帰した"


def test_servo_fault_bit_latches(cfg: dict) -> None:
    h = driving_harness(cfg)
    h.device.inject_axis("J5", fault=0b100)
    h.advance(0.3)
    assert h.device.state is State.EMERGENCY_LATCHED
    assert h.device.stop_reason is StopReason.SERVO_FAULT


def test_servo_overload_is_visible_to_the_pc(cfg: dict) -> None:
    """過負荷はテレメトリに出る（掴まれ検知の材料）。**まだ機体は自動で止めない。**

    構想設計書16章は「接触検知で 20ms 以内に脱力」を求めている。未達
    （`docs/safety_limits.md` §3）。ここでは「見えている」ことだけを確かめる。
    """
    h = driving_harness(cfg)
    h.device.inject_axis("J4", load=0.9)
    h.advance(0.5)
    tel = h.client.telemetry
    assert tel is not None
    assert max(abs(a.load) for a in tel.axes) > 0.3, "外力がテレメトリに出ていない"


# ---- 指令値 ----------------------------------------------------------------------------
BAD_VALUES = [("NaN", float("nan")), ("+Inf", float("inf")), ("-Inf", float("-inf"))]


@pytest.mark.parametrize("label,bad", BAD_VALUES, ids=[b[0] for b in BAD_VALUES])
def test_nan_and_inf_never_reach_the_wire(cfg: dict, label: str, bad: float) -> None:
    """壊れた数値は**送る前に**捨てる（通信スレッドを落とさない・機体を動かさない）。"""
    h = driving_harness(cfg)
    h.client.stop(h.now)
    h.advance(0.3)
    sent_before = h.tr.tx_bytes
    h.client.set_drive(bad, 60.0, 0.5)
    assert h.client.drive is None and h.client.rejected, label
    assert h.client.head(h.now, bad, 0.0, 0.0, 60.0) == -1
    assert h.client.body(h.now, (bad,) * 6, 120.0) == -1
    h.advance(0.5)
    assert not h.device.driving
    assert h.tr.tx_bytes - sent_before < 200, "壊れた値を送ろうとしている"


def test_invalid_angle_and_velocity_are_rejected_by_the_device(cfg: dict) -> None:
    """機体側でも弾く（PC の検査を通り抜けても動かない）。"""
    h = driving_harness(cfg)
    h.client.clear_drive()
    h.advance(0.5)
    h.client.arm(h.now)
    h.advance(0.1)
    before = dict(h.device.output)
    h.client.head(h.now, 200.0, 0.0, 0.0, 60.0)          # 可動範囲外
    h.client.body(h.now, (89.0,) * 6, 120.0)             # operational limit 外
    assert h.client.head(h.now, 30.0, 0.0, 0.0, 9000.0) == -1   # 速度が u16 に入らない
    assert h.client.rejected, "桁あふれを記録していない"
    h.advance(0.5)
    assert Nack.OUT_OF_RANGE in nacks(h)
    assert max(abs(h.device.output[k] - before[k]) for k in before) < 1e-9


def test_finite_helper_catches_every_bad_value() -> None:
    from serpens.link.faults import finite

    assert finite(1.0, -2.5, 0)
    assert not finite(float("nan"))
    assert not finite(1.0, float("inf"))
    assert not finite(-math.inf, 0.0)


# ---- 機体 ------------------------------------------------------------------------------
def test_control_loop_overrun_is_reported(cfg: dict) -> None:
    """制御周期を超えたら数えて PC へ知らせる（遅い処理を隠さない）。"""
    h = driving_harness(cfg)
    assert h.device.overruns == 0
    h.now += 0.5                                        # 機体が 0.5 秒固まった
    h.device.tick(h.now)
    h.advance(0.3)
    assert h.device.overruns >= 1
    tel = h.client.telemetry
    assert tel is not None and tel.flags & Flag.OVERRUN
    assert tel.loop_period_us > cfg["link"]["control_hz"] * 10


def test_telemetry_says_the_values_are_simulated(cfg: dict) -> None:
    """**模擬の値であることが PC 側で分かる。** 実測と混ぜないための印。"""
    h = driving_harness(cfg)
    h.advance(0.3)
    tel = h.client.telemetry
    assert tel is not None and tel.simulated
    assert tel.flags & Flag.SIMULATED
