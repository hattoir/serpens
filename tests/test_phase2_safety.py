"""Phase 2 完了条件 2 / 3 / 5 / 6 / 8 / 9 / 10 / 12: 機体が自分で止まること。

**PC を殺す・USB を抜く・heartbeat を止める**を偽経路で再現する。実機での実測は
tools/link_check.py と docs/phase2_acceptance.md（条件 13/14/15）。
"""
from __future__ import annotations

import pytest

from serpens.config import load_config
from serpens.link import messages as m
from serpens.link.harness import LinkHarness
from serpens.link.protocol import Cmd, FrameReader, Nack, Rep, State, StopReason, encode

HB_TIMEOUT_MARGIN_S = 0.25      # timeout + 制御周期 + テレメトリ周期ぶんの余裕


@pytest.fixture()
def cfg() -> dict:
    return load_config()


def timeout_s(cfg: dict) -> float:
    return cfg["link"]["heartbeat_timeout_ms"] / 1000.0


def driving_harness(cfg: dict) -> LinkHarness:
    h = LinkHarness(cfg)
    assert h.start_driving(), "走行を開始できなかった"
    return h


# ---- 完了条件 2 / 3: PC が落ちる・USB が抜ける ------------------------------------------
def test_pc_process_killed_leads_to_hold(cfg: dict) -> None:
    """PC のプロセスが消えても、機体が自分で保持へ入る（線は繋がったまま）。

    止まり方は二段構え。まず **DRIVE の TTL（300ms）** が切れて保持に入り、続いて
    **heartbeat タイムアウト（400ms）** で待機へ落ちる。速い方が先に効く。
    """
    h = driving_harness(cfg)
    h.tr.kill_pc()
    took = h.run_until(lambda: not h.device.driving, timeout_s=2.0)
    assert took is not None
    assert took == pytest.approx(cfg["link"]["drive_ttl_ms"] / 1000.0, abs=HB_TIMEOUT_MARGIN_S)
    assert h.device.stop_reason is StopReason.DRIVE_TTL       # 先に効くのは TTL
    h.advance(timeout_s(cfg))
    assert h.device.stop_reason is StopReason.HEARTBEAT_LOST  # 続いて heartbeat 途絶
    assert h.device.state is State.DISARMED
    assert h.moved_deg(1.0) < 1e-9, "PC が死んだ後に動いた"
    assert h.device.torque_on, "保持なのでトルクは入ったまま（脱力は明示指令のときだけ）"


def test_usb_unplug_leads_to_hold(cfg: dict) -> None:
    """USB を抜いても同じ経路で止まる。"""
    h = driving_harness(cfg)
    h.tr.unplug()
    took = h.run_until(lambda: not h.device.driving, timeout_s=2.0)
    assert took is not None
    assert took == pytest.approx(cfg["link"]["drive_ttl_ms"] / 1000.0, abs=HB_TIMEOUT_MARGIN_S)
    h.advance(timeout_s(cfg))
    assert h.device.stop_reason is StopReason.HEARTBEAT_LOST
    assert h.device.state is State.DISARMED
    assert h.moved_deg(1.0) < 1e-9


def test_pc_sees_link_loss(cfg: dict) -> None:
    """PC 側もリンク断に気付く（テレメトリが古くなる）。"""
    h = driving_harness(cfg)
    assert h.client.link_ok(h.now)
    h.tr.unplug()
    h.advance(1.0)
    assert not h.client.link_ok(h.now)
    assert "前の情報" in h.client.status_text(h.now)


def test_reconnect_does_not_restart_motion(cfg: dict) -> None:
    """挿し直しただけでは走り出さない（ARM + DRIVE が要る）。"""
    h = driving_harness(cfg)
    h.tr.unplug()
    h.advance(1.0)
    h.tr.plug()
    h.advance(1.0)
    assert not h.device.driving and h.device.state is State.DISARMED


# ---- 完了条件 5: heartbeat だけ止める（DRIVE は届き続ける） -------------------------------
def test_heartbeat_loss_stops_even_while_drive_arrives(cfg: dict) -> None:
    h = driving_harness(cfg)
    h.client.hb_dt = 1e9                         # heartbeat を送るのをやめる（DRIVE は続く）
    took = h.run_until(lambda: not h.device.driving, timeout_s=2.0)
    assert took is not None and took == pytest.approx(timeout_s(cfg), abs=HB_TIMEOUT_MARGIN_S)
    assert h.device.stop_reason is StopReason.HEARTBEAT_LOST
    assert h.moved_deg(0.5) < 1e-9


def test_drive_without_heartbeat_is_rejected(cfg: dict) -> None:
    """heartbeat が切れた後の DRIVE は、届いても実行されない。"""
    h = driving_harness(cfg)
    h.client.hb_dt = 1e9
    h.advance(timeout_s(cfg) + 0.3)
    rd = FrameReader()                           # PC がまだ走らせようとしている状況を直に作る
    reply = rd.feed(h.device.feed(encode(Cmd.DRIVE, h.client._seq + 1,
                                         m.Drive(300, 30.0, 60.0, 0.5, 0.0).pack()), h.now))[0]
    assert m.unpack_nack(reply.payload)[2] == Nack.DISARMED
    h.advance(0.3)
    assert not h.device.driving


def test_arm_requires_heartbeat(cfg: dict) -> None:
    """heartbeat が来ていない機体は ARM できない（電源投入直後の暴走を防ぐ）。"""
    h = LinkHarness(cfg)
    rd = FrameReader()
    reply = rd.feed(h.device.feed(encode(Cmd.ARM, 1, b""), h.now))[0]
    assert reply.type == Rep.NACK and m.unpack_nack(reply.payload)[2] == Nack.NO_HEARTBEAT
    assert h.device.state is State.DISARMED


# ---- 完了条件 6: 機体の再起動で自動再開しない --------------------------------------------
def test_device_reboot_starts_disarmed(cfg: dict) -> None:
    h = driving_harness(cfg)
    boot0 = h.device.boot_id
    h.tr.reboot_device(h.now)
    h.advance(1.5)                               # PC は heartbeat と DRIVE を送り続けている
    assert h.device.boot_id != boot0
    assert h.device.state is State.DISARMED and not h.device.driving
    assert h.client.rebooted, "PC が再起動に気付いていない"
    assert h.client.drive is None, "PC が走行指令を送り続けている"
    h.advance(2.0)
    assert not h.device.driving, "再起動後に勝手に走り出した"


def test_after_reboot_operator_must_arm(cfg: dict) -> None:
    h = driving_harness(cfg)
    h.tr.reboot_device(h.now)
    h.advance(1.0)
    h.client.arm(h.now)                          # 人の操作
    h.advance(0.1)
    h.client.set_drive(30.0, 60.0, 0.5)
    h.advance(0.4)
    assert h.device.driving


# ---- 完了条件 8 / 9: 緊急停止はラッチ。再接続で解除されない -------------------------------
def test_emergency_latches_on_device(cfg: dict) -> None:
    h = driving_harness(cfg)
    h.client.emergency(h.now)
    h.advance(0.1)
    assert h.device.state is State.EMERGENCY
    assert h.device.stop_reason is StopReason.EMERGENCY_CMD
    assert not h.device.driving and h.moved_deg(1.0) < 1e-9
    h.client.arm(h.now)                          # ARM しても解除されない
    h.advance(0.1)
    h.client.set_drive(30.0, 60.0, 0.5)
    h.advance(0.5)
    assert h.device.state is State.EMERGENCY and not h.device.driving
    assert Nack.LATCHED in {n[2] for n in h.client.nacks}


def test_emergency_survives_reconnect(cfg: dict) -> None:
    """USB を抜き差ししても、PC を立ち上げ直しても解除されない（完了条件 9）。"""
    h = driving_harness(cfg)
    h.client.emergency(h.now)
    h.advance(0.2)
    h.tr.unplug()
    h.advance(1.0)
    h.tr.plug()
    h.advance(0.5)
    assert h.device.state is State.EMERGENCY
    h.client = type(h.client)(h.tr, cfg, now=h.now)     # PC 側だけ作り直す（再起動に相当）
    h.advance(1.0)
    assert h.device.state is State.EMERGENCY and h.client.latched
    assert h.client.telemetry is not None and "緊急停止" in h.client.telemetry.state_ja


def test_stop_command_does_not_clear_latch(cfg: dict) -> None:
    h = driving_harness(cfg)
    h.client.emergency(h.now)
    h.advance(0.2)
    h.client.stop(h.now)
    h.advance(0.2)
    assert h.device.state is State.EMERGENCY
    assert h.device.stop_reason is StopReason.EMERGENCY_CMD, "停止理由が上書きされた"


# ---- 完了条件 10: CLEAR_FAULT は待機へ戻すだけ -------------------------------------------
def test_clear_fault_returns_to_disarmed_only(cfg: dict) -> None:
    h = driving_harness(cfg)
    h.client.emergency(h.now)
    h.advance(0.2)
    h.client.clear_fault(h.now, nonce=0x1234)
    h.advance(0.2)
    assert h.device.state is State.DISARMED
    h.client.set_drive(30.0, 60.0, 0.5)          # ARM 無しの DRIVE では動かない
    h.advance(0.5)
    assert not h.device.driving
    h.client.arm(h.now)                          # ARM + DRIVE で初めて動く
    h.advance(0.1)
    h.client.set_drive(30.0, 60.0, 0.5)
    h.advance(0.4)
    assert h.device.driving


def test_clear_fault_nonce_cannot_be_replayed(cfg: dict) -> None:
    """同じ nonce の CLEAR_FAULT を録音・再送しても解除できない。"""
    h = driving_harness(cfg)
    h.client.emergency(h.now)
    h.advance(0.2)
    h.client.clear_fault(h.now, nonce=0xABCD)
    h.advance(0.2)
    h.client.emergency(h.now)                    # もう一度ラッチさせる
    h.advance(0.2)
    rd = FrameReader()
    reply = rd.feed(h.device.feed(encode(Cmd.CLEAR_FAULT, h.client._seq + 1,
                                         m.pack_nonce(0xABCD)), h.now))[0]
    assert reply.type == Rep.NACK and m.unpack_nack(reply.payload)[2] == Nack.NONCE_REUSED
    assert h.device.state is State.EMERGENCY


# ---- 完了条件 12: 停止理由が PC から確認できる -------------------------------------------
def test_all_stop_reasons_are_visible_from_pc(cfg: dict) -> None:
    """6 種類の停止理由を起こし、PC 側のテレメトリ・履歴から日本語で読めることを確かめる。"""
    seen: dict[StopReason, str] = {}

    def record(h: LinkHarness) -> None:
        tel = h.client.telemetry
        assert tel is not None, "テレメトリが届いていない"
        seen[tel.stop_reason] = tel.reason_ja

    h = driving_harness(cfg)
    h.client.stop(h.now)                                   # 1) PC からの停止
    h.advance(0.3)
    record(h)

    h.client.arm(h.now)
    h.advance(0.1)
    h.client.set_drive(30.0, 60.0, 0.5)
    h.advance(0.4)
    h.client.clear_drive()                                 # 2) DRIVE の期限切れ
    h.advance(0.6)
    record(h)

    h.client.arm(h.now)
    h.advance(0.1)
    h.client.set_drive(30.0, 60.0, 0.5)
    h.advance(0.4)
    h.client.hb_dt = 1e9                                   # 3) heartbeat 途絶
    h.advance(timeout_s(cfg) + 0.4)
    record(h)
    h.client.hb_dt = 1.0 / cfg["link"]["heartbeat_hz"]
    h.advance(0.3)

    h.client.emergency(h.now)                              # 4) 緊急停止
    h.advance(0.3)
    record(h)
    h.client.clear_fault(h.now)
    h.advance(0.3)

    h.device.inject_axis("J3", temp_c=cfg["link"]["faults"]["temp_limit_c"] + 5)  # 5) 過熱
    h.advance(0.3)
    record(h)
    h.device.inject_axis("J3", temp_c=25.0)
    h.client.clear_fault(h.now)
    h.advance(0.3)

    h.device.inject_axis("J5", fault=0b100)                # 6) サーボ異常
    h.advance(0.3)
    record(h)

    want = {StopReason.OPERATOR_STOP, StopReason.DRIVE_TTL, StopReason.HEARTBEAT_LOST,
            StopReason.EMERGENCY_CMD, StopReason.OVERHEAT, StopReason.SERVO_FAULT}
    assert want <= set(seen), f"PC から読めない停止理由がある: {want - set(seen)}"
    assert all(txt and not txt.isdigit() for txt in seen.values()), "日本語になっていない"
    assert len(h.client.reason_history_text()) >= len(want), "履歴に残っていない"


def test_device_faults_latch_as_emergency(cfg: dict) -> None:
    """機体自身が見つけた異常（過熱）はラッチする。冷えても自動では戻らない。"""
    h = driving_harness(cfg)
    h.device.inject_axis("J2", temp_c=cfg["link"]["faults"]["temp_limit_c"] + 1)
    h.advance(0.2)
    assert h.device.state is State.EMERGENCY and h.device.stop_reason is StopReason.OVERHEAT
    h.device.inject_axis("J2", temp_c=25.0)
    h.advance(1.0)
    assert h.device.state is State.EMERGENCY, "冷えたら勝手に復帰した"


def test_stop_with_disable_torque_is_explicit(cfg: dict) -> None:
    """脱力は STOP(mode=disable) のときだけ。通常の停止ではトルクを切らない。"""
    h = driving_harness(cfg)
    h.client.stop(h.now)
    h.advance(0.2)
    assert h.device.torque_on
    h.client.stop(h.now, disable_torque=True)
    h.advance(0.2)
    assert not h.device.torque_on
