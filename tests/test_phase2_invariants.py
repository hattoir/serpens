"""機体側の**不変条件**を1か所に固定する（Phase 2 の契約）。

個々の振る舞いは `test_phase2_device.py` / `test_phase2_safety.py` / `test_phase2_faults.py`
が詳しく確かめている。ここは「守られていなければ実機で事故になる」項目だけを、
契約として短く並べたもの。**このファイルが落ちたら、設計に戻る。**

すべて SIMULATED（模擬機体）。実機の値は `docs/verification_status.md`。
"""
from __future__ import annotations

import pytest

from serpens.config import load_config
from serpens.link.harness import LinkHarness
from serpens.link.protocol import Cmd, Flag, Nack, State, StopReason


@pytest.fixture()
def cfg() -> dict:
    return load_config()


@pytest.fixture()
def h(cfg: dict) -> LinkHarness:
    return LinkHarness(cfg)


def nacks(h: LinkHarness) -> set[Nack]:
    return {n[2] for n in h.client.nacks}


def driving(h: LinkHarness) -> LinkHarness:
    assert h.start_driving(), "走行を開始できなかった"
    return h


# ---- 起動と ARM ------------------------------------------------------------------------
def test_boot_state_is_not_movable(h: LinkHarness) -> None:
    """1. 起動直後は BOOT。heartbeat を受けて DISARMED。**どちらも動かない。**"""
    assert h.device.state is State.BOOT and not h.device.driving
    h.advance(0.3)
    assert h.device.state is State.DISARMED and not h.device.driving


def test_drive_without_arm_is_rejected(h: LinkHarness) -> None:
    """2. ARM していない機体は DRIVE を実行しない（PC が送っても、機体が拒否する）。"""
    from serpens.link import messages as m
    from serpens.link.protocol import FrameReader, Rep, encode

    h.advance(0.3)
    h.client.set_drive(30.0, 60.0, 0.5)
    h.advance(0.5)
    assert not h.device.driving
    assert h.client.drive is None, "PC が通らない指令を送り続けている"
    # PC 側の遠慮を飛ばして、機体が自分で拒否することを確かめる
    rd = FrameReader()
    reply = rd.feed(h.device.feed(encode(Cmd.DRIVE, h.client._seq + 1,
                                         m.Drive(300, 30.0, 60.0, 0.5, 0.0).pack()), h.now))[0]
    assert reply.type == Rep.NACK and m.unpack_nack(reply.payload)[2] == Nack.DISARMED
    h.advance(0.3)
    assert not h.device.driving


def test_arm_alone_does_not_move(h: LinkHarness) -> None:
    """ARM だけでは動かない（ARMED_HOLD）。動くには有効な DRIVE が要る。"""
    h.advance(0.2)
    h.client.arm(h.now)
    h.advance(0.3)
    assert h.device.state is State.ARMED_HOLD and not h.device.driving
    assert h.moved_deg(0.5) < 1e-9


# ---- 期限（heartbeat と DRIVE は独立） ---------------------------------------------------
def test_heartbeat_timeout_holds(cfg: dict) -> None:
    """3. heartbeat が途絶えたら保持（FAULT_HOLD）。DRIVE が届いていても止まる。"""
    h = driving(LinkHarness(cfg))
    h.client.hb_dt = 1e9
    assert h.run_until(lambda: not h.device.driving, 2.0) is not None
    assert h.device.state is State.FAULT_HOLD
    assert h.device.stop_reason is StopReason.HEARTBEAT_LOST


def test_drive_ttl_timeout_holds(cfg: dict) -> None:
    """4. DRIVE の更新が止まったら保持（ARMED_HOLD）。heartbeat は生きていてよい。"""
    h = driving(LinkHarness(cfg))
    h.client.clear_drive()
    assert h.run_until(lambda: not h.device.driving, 2.0) is not None
    assert h.device.state is State.ARMED_HOLD
    assert h.device.stop_reason is StopReason.DRIVE_TTL
    assert h.device._hb_fresh(h.now), "heartbeat まで切れている"


def test_heartbeat_does_not_extend_the_drive_ttl(cfg: dict) -> None:
    """5. **heartbeat は DRIVE の期限を延ばさない。**

    延ばしてしまうと「行動が止まっても heartbeat だけで走り続ける」ことになる。
    """
    h = driving(LinkHarness(cfg))
    h.client.clear_drive()                       # DRIVE だけ止める（heartbeat は 10Hz のまま）
    last_drive = h.device._drive_at
    assert h.run_until(lambda: not h.device.driving, 2.0) is not None
    ttl_s = cfg["link"]["drive_ttl_ms"] / 1000.0
    assert h.now - last_drive == pytest.approx(ttl_s, abs=2.0 * h.dt)
    assert h.device._hb_at is not None and h.now - h.device._hb_at < ttl_s


def test_drive_does_not_extend_the_heartbeat(cfg: dict) -> None:
    """逆も同じ。DRIVE が届いても heartbeat の期限は延びない。"""
    h = driving(LinkHarness(cfg))
    h.client.hb_dt = 1e9                         # heartbeat だけ止める（DRIVE は届き続ける）
    took = h.run_until(lambda: not h.device.driving, 2.0)
    assert took is not None
    assert took == pytest.approx(cfg["link"]["heartbeat_timeout_ms"] / 1000.0, abs=0.25)


# ---- 復帰の段差 ------------------------------------------------------------------------
def test_fault_cause_alone_does_not_resume(cfg: dict) -> None:
    """6. 原因が消えただけでは走行に戻らない（FAULT_HOLD → DISARMED まで）。"""
    h = driving(LinkHarness(cfg))
    h.client.hb_dt = 1e9
    h.run_until(lambda: h.device.state is State.FAULT_HOLD, 2.0)
    h.client.hb_dt = 1.0 / cfg["link"]["heartbeat_hz"]          # heartbeat が戻る
    h.advance(1.0)
    assert h.device.state is State.DISARMED, "待機より先へ勝手に進んだ"
    assert not h.device.driving


def test_clear_fault_returns_to_disarmed_and_requires_rearm(cfg: dict) -> None:
    """7/8. CLEAR_FAULT は DISARMED まで。走行には ARM + DRIVE が要る。"""
    h = driving(LinkHarness(cfg))
    h.client.emergency(h.now)
    h.advance(0.2)
    assert h.device.state is State.EMERGENCY_LATCHED
    h.client.clear_fault(h.now)
    h.advance(0.2)
    assert h.device.state is State.DISARMED
    h.client.set_drive(30.0, 60.0, 0.5)                        # ARM 無し
    h.advance(0.5)
    assert not h.device.driving
    h.client.arm(h.now)
    h.advance(0.1)
    h.client.set_drive(30.0, 60.0, 0.5)
    h.advance(0.4)
    assert h.device.driving


def test_emergency_survives_reconnect(cfg: dict) -> None:
    """9. 緊急停止は再接続で解除されない。"""
    h = driving(LinkHarness(cfg))
    h.client.emergency(h.now)
    h.advance(0.2)
    h.tr.unplug()
    h.advance(0.5)
    h.tr.plug()
    h.advance(0.5)
    assert h.device.state is State.EMERGENCY_LATCHED


def test_reboot_does_not_auto_resume(cfg: dict) -> None:
    """10. boot_id が変わっても自動で走り出さない。"""
    h = driving(LinkHarness(cfg))
    boot0 = h.device.boot_id
    h.tr.reboot_device(h.now)
    h.advance(3.0)
    assert h.device.boot_id != boot0 and not h.device.driving
    assert h.client.rebooted


# ---- テレメトリの正直さ ------------------------------------------------------------------
def test_missing_servo_is_not_filled_with_zero(cfg: dict) -> None:
    """11. **応答しない軸を 0 で埋めない。** 欠けたまま返し、印を立てる。"""
    h = driving(LinkHarness(cfg))
    h.device.inject_axis("J4", offline=True)
    h.advance(0.5)
    tel = h.client.telemetry
    assert tel is not None
    assert len(tel.axes) == len(cfg["joints"]) - 1, "欠けた軸が 0 で埋められている"
    assert tel.flags & Flag.SERVO_MISSING
    assert tel.simulated, "模擬の値である印が消えている"


def test_missing_axis_while_driving_goes_to_fault_hold(cfg: dict) -> None:
    """12. 走行中に必要な軸が欠けたら FAULT_HOLD。残りの軸で走り続けない。"""
    h = driving(LinkHarness(cfg))
    h.device.inject_axis("J2", offline=True)
    assert h.run_until(lambda: not h.device.driving, 1.0) is not None
    assert h.device.state is State.FAULT_HOLD
    assert h.device.stop_reason is StopReason.SERVO_FAULT


# ---- 壊れた値 --------------------------------------------------------------------------
@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf"), 9e9])
def test_bad_values_never_reach_the_transport(cfg: dict, bad: float) -> None:
    """13. NaN / Inf / 桁あふれは**経路へ出る前に**捨てる（送信側も落とさない）。"""
    h = driving(LinkHarness(cfg))
    h.client.stop(h.now)
    h.advance(0.3)
    before = h.tr.tx_bytes
    h.client.set_drive(bad, 60.0, 0.5)
    h.client.head(h.now, bad, 0.0, 0.0, 60.0)
    h.client.body(h.now, (bad,) * 6, 120.0)
    h.advance(0.3)
    assert h.client.rejected, "壊れた値を素通りさせた"
    assert not h.device.driving
    assert h.tr.tx_bytes - before < 200, "壊れた値を送ろうとしている"


def test_stop_never_moves_to_home(cfg: dict) -> None:
    """停止で勝手にホーム姿勢へ動かさない（停止中に動くのが最も危険）。"""
    h = driving(LinkHarness(cfg))
    h.advance(1.0)
    before = dict(h.device.output)
    assert any(abs(v) > 5.0 for v in before.values()), "そもそも曲がっていない"
    h.client.stop(h.now)
    h.advance(1.5)
    assert max(abs(h.device.output[k] - before[k]) for k in before) < 1e-9


def test_normal_stop_keeps_torque(cfg: dict) -> None:
    """通常停止でトルクを切らない。脱力は STOP(mode=disable) のときだけ。"""
    h = driving(LinkHarness(cfg))
    h.client.stop(h.now)
    h.advance(0.3)
    assert h.device.torque_on and h.device.state is State.DISARMED
    h.client.stop(h.now, disable_torque=True)
    h.advance(0.3)
    assert not h.device.torque_on and h.device.state is State.TORQUE_DISABLED


def test_arm_must_name_the_boot_id(cfg: dict) -> None:
    """**再起動とすれ違った ARM を機体が拒否する。**（2026-09-16 に見つけた穴の回帰試験）

    PC 側の「再起動に気付いたら ARM しない」だけでは足りなかった。再起動の telemetry が
    届く前に出た ARM が通り、**人の操作なしに 240ms で走行が再開していた**。
    ARM に boot_id を持たせ、知らない起動の機体は ARM できないようにした。
    """
    from serpens.link import messages as m
    from serpens.link.protocol import FrameReader, Rep, encode

    h = LinkHarness(cfg)
    h.advance(0.3)
    rd = FrameReader()
    stale = rd.feed(h.device.feed(encode(Cmd.ARM, h.client._seq + 1,
                                         m.pack_arm(h.device.boot_id + 1)), h.now))[0]
    assert stale.type == Rep.NACK and m.unpack_nack(stale.payload)[2] == Nack.STALE_BOOT
    assert h.device.state is State.DISARMED, "知らない起動の機体を ARM してしまった"
    ok = rd.feed(h.device.feed(encode(Cmd.ARM, h.client._seq + 2,
                                      m.pack_arm(h.device.boot_id)), h.now))[0]
    assert ok.type == Rep.ACK and h.device.state is State.ARMED_HOLD


def test_client_will_not_arm_a_machine_it_has_not_seen(cfg: dict) -> None:
    """テレメトリを一度も受けていない機体は ARM しない（どの機体か分かっていない）。"""
    h = LinkHarness(cfg)
    assert h.client.boot_id is None
    assert h.client.arm(h.now) == -1
    assert h.client.rejected
