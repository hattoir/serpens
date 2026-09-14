"""STEP 7: 制御ループ・文言・GUI（オフスクリーン）のテスト。"""
from __future__ import annotations

import os
import time

import numpy as np
import pytest

from serpens.behavior.brain import BrainStatus
from serpens.config import load_config
from serpens.gui.wording import head_distance_mm, sentence
from serpens.keys import KEY_HELP, handle
from serpens.runner import ControlLoop, LoopStats
from serpens.sim.session import SimSession
from serpens.sim.world import BodyPose

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


@pytest.fixture()
def cfg() -> dict:
    return load_config()


def status(**kw) -> BrainStatus:  # type: ignore[no-untyped-def]
    base = dict(t=1.0, state="PATROL", state_ja="巡回", thought="", time_to_next_s=0.0,
                utilities={"PATROL": 0.5, "SLEEP": 0.2, "APPROACH": 0.1}, internal={}, heat_c=30.0,
                drive="目標へ", noticed=False, safety="")
    base.update(kw)
    return BrainStatus(**base)


# ---- 文言 --------------------------------------------------------------------------
def test_sentence_priorities(cfg: dict) -> None:
    assert "冷えるまで休む" in sentence(status(safety="過熱 58.0℃", heat_c=58.0), cfg, None)
    assert "力を抜いて待つ" in sentence(status(safety="掴まれた（負荷 0.90）"), cfg, None)
    assert "撫でられている" in sentence(status(state="PETTED", state_ja="撫でられ"), cfg, None)
    s = sentence(status(drive="停止: 人の 400mm 手前"), cfg, 412.0)
    assert "412mm" in s and "これ以上は近づかない" in s
    assert "台の端" in sentence(status(drive="マット端: 後退しながら中央へ向き直る"), cfg, None)


def test_sentence_is_japanese_not_raw_numbers(cfg: dict) -> None:
    """効用の数値だけを並べず、理由が日本語で書かれていること。"""
    st = status(state="APPROACH", state_ja="接近",
                utilities={"APPROACH": 0.82, "COIL_REST_MOOD": 0.31, "PATROL": 0.2})
    s = sentence(st, cfg, 900.0)
    assert "好奇心 0.82" in s and "上回ったので" in s and "近づくことにした" in s
    held = sentence(status(state="OBSERVE", state_ja="観察", time_to_next_s=2.3,
                           utilities={"OBSERVE": 0.4, "APPROACH": 0.9}), cfg, None)
    assert "観察を続ける" in held and "2.3 秒" in held


def test_head_distance(cfg: dict) -> None:
    d = head_distance_mm((600.0, 600.0), (600.0, 0.0), cfg)
    assert d == pytest.approx(600.0 - cfg["behavior"]["controller"]["head_reach_mm"])
    assert head_distance_mm(None, (0.0, 0.0), cfg) is None


# ---- 制御ループ ---------------------------------------------------------------------
def test_loop_stats() -> None:
    s = LoopStats(50.0)
    for dt in (0.02, 0.021, 0.05):
        s.add(dt)
    assert s.count == 3 and s.worst_ms == pytest.approx(50.0)
    assert s.mean_ms == pytest.approx((20 + 21 + 50) / 3)
    assert s.late_count == 1                       # 24ms 超えは 1 回


def test_control_loop_keeps_period(cfg: dict) -> None:
    """制御ループが目標周期（50Hz）で回る。"""
    session = SimSession(cfg, BodyPose(150.0, 600.0, 0.0), seed=1)
    loop = ControlLoop(session, realtime=True)
    loop.start()
    time.sleep(2.0)
    loop.stop()
    loop.join(timeout=2.0)
    s = loop.stats
    assert s.count > 80                            # 2 秒で 100 回前後
    assert 18.0 <= s.mean_ms <= 23.0
    assert s.late_count <= s.count * 0.05
    snap = loop.latest()
    assert snap.status is not None and snap.points is not None and snap.stats is s


def test_keys(cfg: dict) -> None:
    session = SimSession(cfg, BodyPose(150.0, 600.0, 0.0), seed=1)
    loop = ControlLoop(session, realtime=False)
    for _ in range(5):
        session.step()
    assert "ホーム" in handle(loop, "r")
    assert session.brain.fsm.state == "PATROL"
    assert "デモ" in handle(loop, "d") and session.people
    assert "消した" in handle(loop, "d") and not session.people
    # 停止は3種類。S は機体の通常停止（一時停止ではない）、P がシミュレーションの一時停止
    assert "通常停止" in handle(loop, "s") and not session.stop.moving_allowed
    assert not loop.paused, "S で制御ループを止めてはいけない（監視は続ける）"
    assert "開始" in handle(loop, "g") and session.stop.moving_allowed
    assert "一時停止" in handle(loop, "p") and loop.paused
    assert "解除" in handle(loop, "p") and not loop.paused
    assert "緊急停止" in handle(loop, "e") and session.stop.latched
    assert "停止中" in handle(loop, "h"), "緊急停止中に姿勢を変えようとした"
    assert "解除" in handle(loop, "u") and not session.stop.latched
    assert "tools" in handle(loop, "c")
    handle(loop, "g")
    handle(loop, "t")
    session.step()
    assert session.head.touch_head
    assert len(KEY_HELP) >= 6


# ---- GUI（オフスクリーンで描画できること） ---------------------------------------------
def test_gui_paints_offscreen(cfg: dict) -> None:
    from PySide6.QtWidgets import QApplication

    from serpens.gui.app import MainWindow

    session = SimSession(cfg, BodyPose(150.0, 600.0, 0.0), seed=1)
    for _ in range(10):
        session.step()
    loop = ControlLoop(session, realtime=False)
    loop._publish()
    app = QApplication.instance() or QApplication([])
    win = MainWindow(loop, cfg, lambda: np.zeros((90, 160, 3), np.uint8))
    win.resize(1280, 1000)                       # 関節ペインを足したぶん縦に伸びた
    win.refresh()
    img = win.grab().toImage()
    assert img.width() == 1280 and img.height() == 1000
    # 関節ペインが指令角と実測角を受け取っていること（描けているかは目視）
    assert win.joints.snap is not None and win.joints.names[0] == cfg["joints"][0]["name"]
    win.close()
    app.processEvents()
