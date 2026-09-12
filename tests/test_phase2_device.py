"""Phase 2 完了条件 1 / 4 / 7 / 11: 歩容の一致・期限・古い指令・上限。

偽時計 + 偽経路 + シミュレート ESP32（同じ仕様のファームを書くための参照実装）で確かめる。
"""
from __future__ import annotations

import math

import pytest

from serpens.config import load_config
from serpens.link import messages as m
from serpens.link.harness import LinkHarness
from serpens.link.protocol import Cmd, FrameReader, Nack, Rep, State, StopReason, encode
from serpens.motion.gait import GaitParams, angles_at_phase, body_joint_names


@pytest.fixture()
def cfg() -> dict:
    return load_config()


def nack_reasons(h: LinkHarness) -> list[Nack]:
    return [n[2] for n in h.client.nacks]


# ---- 完了条件 1: PC は DRIVE を送るだけ。歩容は機体が作る -------------------------------
def test_device_generates_gait_matching_reference(cfg: dict) -> None:
    """機体の出力が serpens/motion/gait.py の式と一致する（PC は角度列を送っていない）。"""
    h = LinkHarness(cfg)
    assert h.start_driving(amplitude_deg=30.0, spatial_freq_deg=60.0, temporal_freq_hz=0.5,
                           gamma_deg=-12.0)
    names = body_joint_names(cfg)
    p = GaitParams(30.0, 60.0, 0.5)
    base_phase, base_t = h.device.mo.phase, h.device._last_ctrl
    worst = 0.0
    for _ in range(200):                       # 2 秒ぶん
        h.step()
        phase = base_phase + 2.0 * math.pi * p.temporal_freq_hz * (h.device._last_ctrl - base_t)
        want = angles_at_phase(p, phase, names, -12.0, cfg["gait"]["turn_profile"])
        worst = max(worst, max(abs(h.device.goals[n] - want[n]) for n in names))
    assert worst < 1e-9, f"機体の歩容が参照式とずれた: {worst}"
    # PC が送ったのは DRIVE だけ（1フレーム 14 バイト）で、角度列ではない
    assert h.client.drive is not None and len(h.client.drive.pack()) == 10


def test_drive_frame_carries_only_parameters(cfg: dict) -> None:
    """DRIVE は 5 個の数値だけ。機体側で 6 軸ぶんの角度になる。"""
    h = LinkHarness(cfg)
    h.start_driving()
    assert h.device.driving and h.device.mo.phase != 0.0
    spread = max(h.device.goals[n] for n in h.device.mo.body) - min(
        h.device.goals[n] for n in h.device.mo.body)
    assert spread > 10.0, "6軸が同じ角度になっている（進行波になっていない）"


def test_turn_uses_gamma_offset_only(cfg: dict) -> None:
    """旋回はオフセット γ だけ。頭側ほど大きい（head_weighted）。"""
    h = LinkHarness(cfg)
    h.start_driving(gamma_deg=20.0, temporal_freq_hz=0.5)
    period = 2.0
    acc = {n: 0.0 for n in h.device.mo.body}
    for _ in range(int(period / h.dt)):        # 1周期ぶんの平均 = γ(n)
        h.step()
        for n in acc:
            acc[n] += h.device.goals[n] * h.dt
    mean = {n: v / period for n, v in acc.items()}
    assert mean["J1"] == pytest.approx(0.0, abs=0.5)
    assert mean["J6"] == pytest.approx(20.0, abs=0.5)


# ---- 完了条件 4: DRIVE だけ止めたら止まる（heartbeat は生きている） ----------------------
def test_drive_ttl_stops_motion_while_heartbeat_alive(cfg: dict) -> None:
    h = LinkHarness(cfg)
    assert h.start_driving()
    h.client.clear_drive()                     # DRIVE の送信だけ止める
    last_drive_at = h.device._drive_at
    assert h.run_until(lambda: not h.device.driving, timeout_s=2.0) is not None
    ttl_s = cfg["link"]["drive_ttl_ms"] / 1000.0
    assert h.now - last_drive_at == pytest.approx(ttl_s, abs=1.5 * h.dt), "最後の DRIVE から TTL 後に止まる"
    assert h.device.stop_reason is StopReason.DRIVE_TTL
    assert h.device.state is State.ARMED, "TTL 切れは保持まで（いきなり待機へ落とさない）"
    assert h.device._hb_fresh(h.now), "heartbeat は生きている"
    assert h.moved_deg(0.5) < 1e-9, "停止後に動いた"
    assert h.device.torque_on, "保持なのでトルクは入れたまま"


def test_drive_silence_disarms(cfg: dict) -> None:
    """保持のまま放置すると待機へ落ちる（再開には ARM + DRIVE が要る）。"""
    h = LinkHarness(cfg)
    h.start_driving()
    h.client.clear_drive()
    h.advance(cfg["link"]["drive_disarm_ms"] / 1000.0 + 0.3)
    assert h.device.state is State.DISARMED
    rd = FrameReader()                         # ARM なしで DRIVE を送り直しても（新しい seq で）
    reply = rd.feed(h.device.feed(encode(Cmd.DRIVE, h.client._seq + 1,
                                         m.Drive(300, 30.0, 60.0, 0.5, 0.0).pack()), h.now))[0]
    assert m.unpack_nack(reply.payload)[2] == Nack.DISARMED
    h.advance(0.3)
    assert not h.device.driving


# ---- 完了条件 7: 古い・重複したパケットで走り出さない -------------------------------------
def test_replayed_drive_is_rejected(cfg: dict) -> None:
    h = LinkHarness(cfg)
    assert h.start_driving()
    captured = encode(Cmd.DRIVE, h.client._seq, m.Drive(300, 30.0, 60.0, 0.5, 0.0).pack())
    h.client.stop(h.now)                       # 停止（保持）
    h.advance(0.2)
    assert not h.device.driving and h.device.state is State.DISARMED
    rd = FrameReader()
    replies = rd.feed(h.device.feed(captured, h.now))   # 記録しておいた古いフレームを再送
    assert replies[0].type == Rep.NACK
    assert m.unpack_nack(replies[0].payload)[2] == Nack.STALE_SEQ
    assert not h.device.driving
    assert h.moved_deg(0.3) < 1e-9


def test_duplicate_frames_do_not_double_apply(cfg: dict) -> None:
    """経路が二重に届けても、2通目は無視される。"""
    h = LinkHarness(cfg)
    h.start_driving()
    h.tr.duplicate_next = 5
    h.advance(0.5)
    assert h.device.driving
    assert Nack.STALE_SEQ in nack_reasons(h), "重複が素通りしている"


def test_corrupted_frames_are_rejected(cfg: dict) -> None:
    h = LinkHarness(cfg)
    h.start_driving()
    h.tr.corrupt_next = 3
    h.advance(0.4)
    assert Nack.BAD_CRC in nack_reasons(h)
    assert h.device.driving, "壊れたフレームで止まってはいない（TTL 内なので保持でなく継続）"


def test_split_stream_still_works(cfg: dict) -> None:
    """1バイトずつ細切れに届いても通る。"""
    h = LinkHarness(cfg, chunk=1)
    h.advance(3.0)
    h.client.arm(h.now)
    h.advance(1.0)
    h.client.set_drive(30.0, 60.0, 0.5)
    h.advance(3.0)
    assert h.device.driving


# ---- 完了条件 11: 上限外の値を機体が拒否する ---------------------------------------------
BAD_DRIVES = [
    ("振幅が上限超え", m.Drive(300, 80.0, 60.0, 0.5, 0.0)),
    ("周波数が速すぎ", m.Drive(300, 30.0, 60.0, 3.0, 0.0)),
    ("旋回が大きすぎ", m.Drive(300, 30.0, 60.0, 0.5, 45.0)),
    ("空間周波数が 0", m.Drive(300, 30.0, 0.0, 0.5, 0.0)),
    ("TTL が 0（期限なし）", m.Drive(0, 30.0, 60.0, 0.5, 0.0)),
    ("TTL が長すぎ", m.Drive(5000, 30.0, 60.0, 0.5, 0.0)),
]


@pytest.mark.parametrize("label,bad", BAD_DRIVES, ids=[b[0] for b in BAD_DRIVES])
def test_device_rejects_out_of_range_drive(cfg: dict, label: str, bad: m.Drive) -> None:
    """拒否のうえ、状態も出力も変えないこと。"""
    h = LinkHarness(cfg)
    h.advance(0.2)
    h.client.arm(h.now)
    h.advance(0.1)
    before = dict(h.device.output)
    rd = FrameReader()
    reply = rd.feed(h.device.feed(encode(Cmd.DRIVE, h.client._seq + 1, bad.pack()), h.now))[0]
    assert reply.type == Rep.NACK and m.unpack_nack(reply.payload)[2] == Nack.OUT_OF_RANGE, label
    assert not h.device.driving and h.device.state is State.ARMED
    h.advance(0.3)
    assert max(abs(h.device.output[k] - before[k]) for k in before) < 1e-9, label


BAD_HEADS = [
    ("J7 が可動範囲外", m.Head(300, 120.0, 0.0, 0.0, 60.0)),
    ("J8 が可動範囲外", m.Head(300, 30.0, 95.0, 0.0, 60.0)),
    ("J9 が可動範囲外", m.Head(300, 30.0, 0.0, 60.0, 60.0)),
    ("速度が上限超え", m.Head(300, 30.0, 0.0, 0.0, 400.0)),
    ("速度が 0", m.Head(300, 30.0, 0.0, 0.0, 0.0)),
]


@pytest.mark.parametrize("label,bad", BAD_HEADS, ids=[b[0] for b in BAD_HEADS])
def test_device_rejects_out_of_range_head(cfg: dict, label: str, bad: m.Head) -> None:
    h = LinkHarness(cfg)
    h.advance(0.2)
    h.client.arm(h.now)
    h.advance(0.1)
    before = dict(h.device.output)
    rd = FrameReader()
    reply = rd.feed(h.device.feed(encode(Cmd.HEAD, h.client._seq + 1, bad.pack()), h.now))[0]
    assert reply.type == Rep.NACK and m.unpack_nack(reply.payload)[2] == Nack.OUT_OF_RANGE, label
    h.advance(0.3)
    assert max(abs(h.device.output[k] - before[k]) for k in before) < 1e-9, label


def test_valid_head_moves_within_limits(cfg: dict) -> None:
    """正しい HEAD は通り、可動範囲内で動く。期限切れでその場に止まる。"""
    h = LinkHarness(cfg)
    h.advance(0.2)
    h.client.arm(h.now)
    h.advance(0.1)
    h.client.head(h.now, j7=60.0, j8=-30.0, j9=10.0, speed_dps=60.0, ttl_ms=500)
    h.advance(0.4)
    assert 8.0 < h.device.goals["J7"] < 60.0, "速度制限つきで動いている途中のはず"
    h.advance(0.4)                              # TTL 切れ
    stopped = dict(h.device.goals)
    h.advance(0.5)
    assert all(abs(h.device.goals[k] - stopped[k]) < 1e-9 for k in stopped)
    assert h.device.goals["J7"] <= cfg["joints"][6]["max_deg"]


def test_amplitude_plus_turn_must_fit_soft_limit(cfg: dict) -> None:
    """振幅と旋回オフセットの**合計**がソフトリミット（85°）を超える組み合わせも拒否する。

    既定の上限（振幅 40 + γ 30 = 70）では届かないが、上限を緩めたときの最後の砦。
    """
    import copy

    from serpens.link.device_motion import DeviceMotion

    loose = copy.deepcopy(cfg)
    loose["link"]["limits"]["amplitude_deg"] = 90.0
    loose["link"]["limits"]["gamma_deg"] = 90.0
    mo = DeviceMotion(loose)
    assert mo.drive_ok(m.Drive(300, 60.0, 60.0, 0.5, 20.0))        # 80 ≤ 85
    assert not mo.drive_ok(m.Drive(300, 60.0, 60.0, 0.5, 30.0))    # 90 > 85


def test_unknown_command_and_bad_length(cfg: dict) -> None:
    h = LinkHarness(cfg)
    h.advance(0.2)
    rd = FrameReader()
    bad_type = rd.feed(h.device.feed(encode(0x7F, h.client._seq + 1, b""), h.now))[0]
    assert m.unpack_nack(bad_type.payload)[2] == Nack.UNKNOWN_CMD
    bad_len = rd.feed(h.device.feed(encode(Cmd.DRIVE, 30001, b"\x00\x01"), h.now))[0]
    assert m.unpack_nack(bad_len.payload)[2] == Nack.BAD_LENGTH
