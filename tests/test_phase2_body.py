"""Phase 2 の追加指令: BODY（胴体の姿勢）と TORQUE（脱力の演出）。

とぐろ・鎌首・脱力は展示の見せ場なので、駆動リンク越しでも出せなければならない。
ただし**胴体の持ち主を二つにしない**（歩容中の BODY は機体が拒否する）。
"""
from __future__ import annotations

import pytest

from serpens.config import load_config
from serpens.link import messages as m
from serpens.link.harness import LinkHarness
from serpens.link.protocol import Nack


@pytest.fixture()
def cfg() -> dict:
    return load_config()


def armed(cfg: dict) -> LinkHarness:
    h = LinkHarness(cfg)
    h.advance(0.2)
    h.client.arm(h.now)
    h.advance(0.1)
    return h


def nacks(h: LinkHarness) -> set[Nack]:
    return {n[2] for n in h.client.nacks}


def test_body_pose_moves_body_when_not_driving(cfg: dict) -> None:
    """歩容を使っていないときは、胴体の姿勢を機体が速度制限つきで作る。"""
    h = armed(cfg)
    coil = cfg["poses"]["coil"]
    angles = tuple(float(coil[n]) for n in h.device.mo.body)
    h.client.body(h.now, angles, 20.0, ttl_ms=800)      # ゆっくり動かして途中を見る
    h.advance(0.5)
    assert 0.0 < h.device.goals["J1"] < angles[0], "速度制限つきで移っている途中のはず"
    h.advance(0.5)                                   # TTL 切れ
    stopped = dict(h.device.goals)
    h.advance(0.5)
    assert all(abs(h.device.goals[k] - stopped[k]) < 1e-9 for k in stopped), "期限切れ後に動いた"


def test_body_reaches_pose_when_refreshed(cfg: dict) -> None:
    """期限を更新し続ければ、とぐろの角度まで到達する。"""
    h = armed(cfg)
    coil = cfg["poses"]["coil"]
    angles = tuple(float(coil[n]) for n in h.device.mo.body)
    for _ in range(40):
        h.client.body(h.now, angles, 120.0, ttl_ms=500)
        h.advance(0.1)
    for n, want in zip(h.device.mo.body, angles):
        assert h.device.goals[n] == pytest.approx(want, abs=0.5)


def test_body_is_rejected_while_driving(cfg: dict) -> None:
    """歩容中の BODY は拒否する（胴体の持ち主を二つにしない）。"""
    h = LinkHarness(cfg)
    assert h.start_driving()
    h.client.body(h.now, (80.0,) * 6, 120.0)
    h.advance(0.3)
    assert Nack.BUSY in nacks(h)
    assert h.device.driving, "歩容が止まった"


BAD_BODIES = [
    ("可動範囲外", ((90.0, 0.0, 0.0, 0.0, 0.0, 0.0), 120.0, 300)),
    ("速度が上限超え", ((30.0,) * 6, 500.0, 300)),
    ("TTL が 0", ((30.0,) * 6, 120.0, 0)),
]


@pytest.mark.parametrize("label,bad", BAD_BODIES, ids=[b[0] for b in BAD_BODIES])
def test_device_rejects_out_of_range_body(cfg: dict, label: str, bad: tuple) -> None:
    h = armed(cfg)
    before = dict(h.device.output)
    angles, speed, ttl = bad
    h.client.body(h.now, angles, speed, ttl_ms=ttl)
    h.advance(0.3)
    assert Nack.OUT_OF_RANGE in nacks(h), label
    assert max(abs(h.device.output[k] - before[k]) for k in before) < 1e-9, label


def test_torque_ratio(cfg: dict) -> None:
    """脱力の演出（トルク比）。範囲外は拒否し、停止で 100% に戻る。"""
    h = armed(cfg)
    h.client.torque(h.now, 0.6)
    h.advance(0.2)
    assert h.device.torque_ratio == pytest.approx(0.6)
    h.client.torque(h.now, 0.0)                      # 0 は「脱力」ではなく無効値（STOP を使う）
    h.advance(0.2)
    assert Nack.OUT_OF_RANGE in nacks(h)
    assert h.device.torque_ratio == pytest.approx(0.6), "拒否で状態が変わった"
    h.client.stop(h.now)
    h.advance(0.2)
    assert h.device.torque_ratio == 1.0, "停止しても脱力が残っている"
