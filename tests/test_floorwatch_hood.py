"""フードの端の 1 ビットと 0.6 s のタイムアウト（R-017 / R-023、ENTRY-D-0015 (5)）。**すべて模擬（MockHood）。実機・ファームは未接続。HARDWARE_VERIFIED = 0。**
昇降の指令から 0.6 s で「下がりきった」が立たなければ、フードの駆動を保持し、前進を止め、記録する（ラッチ。人の acknowledge まで再開しない）。"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from serpens.config import load_config
from serpens.floorwatch.hood import DOWN, MOVING_DOWN, STUCK, UNKNOWN, UP, HoodMonitor, MockHood, run_sequence
from serpens.floorwatch.scene import FloorObject, FloorScene
from tests.test_floorwatch_slice import ahead, rig


@pytest.fixture(scope="module")
def cfg() -> dict:
    return load_config()


def test_config_has_the_06s_timeout_as_an_assumption(cfg: dict) -> None:
    m = HoodMonitor.from_cfg(cfg)
    assert m.timeout_s == 0.6 and m.stale_s == 0.6


def test_hood_that_settles_in_time_is_down_and_does_not_stop_forward_motion() -> None:
    m, h = HoodMonitor(0.6, 0.6), MockHood(delay_s=0.25, position=UP)
    m.update(False, True, 0.0)
    assert m.state == UP
    run_sequence(m, h, DOWN, 0.1, 0.5)
    assert m.state == DOWN and m.intake_push_allowed and not m.forward_inhibit and m.drive_enabled
    assert [e.kind for e in m.events] == ["settled", "command", "settled"]


def test_timeout_stops_forward_holds_the_drive_and_records_the_reason() -> None:
    m, h = HoodMonitor(0.6, 0.6), MockHood(delay_s=0.25, stuck=True, position=UP)
    m.update(False, True, 0.0)
    m.command(DOWN, 1.0)
    h.drive(DOWN, 1.0)
    assert m.state == MOVING_DOWN
    m.update(*h.read(1.59), 1.59)
    assert m.state == MOVING_DOWN and not m.forward_inhibit        # 0.59 s ではまだ待つ
    m.update(*h.read(1.60), 1.60)
    assert m.state == STUCK and m.forward_inhibit and not m.drive_enabled and not m.intake_push_allowed
    assert "0.6" in m.stuck_reason and "DOWN" in m.stuck_reason


def test_unreadable_bit_at_the_deadline_counts_as_not_standing() -> None:
    """読めない（None）は「立っていない」。偽（False）と同じ扱い（疑わしいときは止める）。"""
    m, h = HoodMonitor(0.6, 0.6), MockHood(delay_s=0.1, bit_dead=True, position=UP)
    m.command(DOWN, 0.0)
    h.drive(DOWN, 0.0)
    for t in (0.2, 0.4, 0.59):
        m.update(*h.read(t), t)
        assert m.state == MOVING_DOWN
    m.tick(0.6)                                                      # 読み出しが無い周期でも期限は来る
    assert m.state == STUCK and "None" in m.stuck_reason


def test_both_bits_at_once_is_a_sensor_fault() -> None:
    m = HoodMonitor()
    m.update(True, True, 0.0)
    assert m.state == STUCK and "同時" in m.stuck_reason


def test_bit_dropping_without_a_command_is_an_unintended_motion() -> None:
    m = HoodMonitor()
    m.update(True, False, 0.0)
    assert m.state == DOWN
    m.update(False, False, 0.1)
    assert m.state == STUCK and "指令なし" in m.stuck_reason


def test_stuck_latches_until_a_human_acknowledges() -> None:
    m = HoodMonitor()
    m.command(DOWN, 0.0)
    m.update(False, False, 0.6)
    assert m.state == STUCK
    for t in (0.7, 0.8):                                             # 原因が消えた（下のビットが立った）ように見えても、自動では戻らない
        m.update(True, False, t)
        assert m.state == STUCK
    assert m.command(UP, 0.9) is False and m.events[-1].kind == "rejected"
    assert m.acknowledge(1.0) is True and m.state == UNKNOWN
    m.update(True, False, 1.1)
    assert m.state == DOWN
    assert m.acknowledge(1.2) is False                               # STUCK でなければ何もしない


def test_missing_reads_for_the_stale_time_stop_a_settled_hood() -> None:
    m = HoodMonitor(0.6, 0.6)
    m.update(True, False, 0.0)
    m.tick(0.5)
    assert m.state == DOWN
    m.tick(0.6)
    assert m.state == STUCK and "読み出しが無い" in m.stuck_reason


def test_invalid_arguments_are_rejected() -> None:
    with pytest.raises(ValueError):
        HoodMonitor(0.0, 0.6)
    with pytest.raises(ValueError):
        HoodMonitor().command("SIDEWAYS", 0.0)


# ---- executor との接続（模擬の縦一本）---------------------------------------------------------
def _ready(cfg: dict, tmp_path: Path):
    scene = FloorScene()
    session, ex, ep, home = rig(cfg, scene, tmp_path)
    target = ahead(ex, 0.20)
    scene.objects.append(FloorObject("washer", target[0], target[1], 20.0, 1.5, 0.9, True))
    for _ in range(5):
        ex.tick()
    return session, ex, home, target


def _run(ex, home, tid: str, max_ticks: int = 6000) -> str:
    for _ in range(max_ticks):
        ex.tick()
        st = home.statuses_of(tid)
        if st and st[-1] in ("done", "failed", "aborted", "rejected"):
            return st[-1]
    return "timeout"


def test_stuck_hood_fails_the_running_task_and_blocks_new_ones(cfg: dict, tmp_path: Path) -> None:
    session, ex, home, target = _ready(cfg, tmp_path)
    mon, hood = HoodMonitor.from_cfg(cfg), MockHood(stuck=True, position=UP)
    ex.attach_hood(mon, hood.read)
    mon.update(False, True, session.t)
    mon.command(DOWN, session.t)
    hood.drive(DOWN, session.t)
    tid = home.task("inspect_point", target={"x_m": target[0], "y_m": target[1], "yaw_rad": 0.0})
    assert home.statuses_of(tid) == ["accepted"]
    assert _run(ex, home, tid) == "failed"
    assert "フードの端のビット" in home.last_reason(tid)
    assert mon.state == STUCK and mon.forward_inhibit
    again = home.task("inspect_point", target={"x_m": target[0], "y_m": target[1], "yaw_rad": 0.0})
    assert home.statuses_of(again) == ["rejected"] and "フード" in home.last_reason(again)


def test_without_the_hood_check_the_same_task_completes(cfg: dict, tmp_path: Path) -> None:
    """対照: フードを繋がなければ（既定）、同じ Task は終わる。固着の検出が Task を止めていることの確認。"""
    session, ex, home, target = _ready(cfg, tmp_path)
    tid = home.task("inspect_point", target={"x_m": target[0], "y_m": target[1], "yaw_rad": 0.0})
    assert _run(ex, home, tid) == "done"


def test_healthy_hood_does_not_disturb_the_task(cfg: dict, tmp_path: Path) -> None:
    session, ex, home, target = _ready(cfg, tmp_path)
    mon, hood = HoodMonitor.from_cfg(cfg), MockHood(delay_s=0.25, position=UP)
    ex.attach_hood(mon, hood.read)
    mon.update(False, True, session.t)
    mon.command(DOWN, session.t)
    hood.drive(DOWN, session.t)
    tid = home.task("inspect_point", target={"x_m": target[0], "y_m": target[1], "yaw_rad": 0.0})
    assert _run(ex, home, tid) == "done" and mon.state == DOWN
