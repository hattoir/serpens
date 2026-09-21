"""EXHIBITION profile（config/profile_exhibition.yaml）: 体験弧と Fear 監査。**SIMULATED。**

体験弧 NOTICE → LOOK → HESITATE/TRACK → APPROACH → OBSERVE → ENGAGE → RELAX → LEAVE/REST が
固定スクリプトでなく Utility / 内部状態 / grammar から 60〜90 秒で起きること。
"""
from __future__ import annotations

import math

import pytest

from serpens.config import load_config
from serpens.sim.session import SimSession
from serpens.sim.world import BodyPose
from simulation.bridge import drive_people
from simulation.virtual_person import VirtualPerson

PROFILE = "config/profile_exhibition.yaml"
START = BodyPose(600.0, 1100.0, -math.pi / 2)        # 来場者側を向いて台の中央


@pytest.fixture(scope="module")
def cfg() -> dict:
    return load_config(overlay=PROFILE)


def visitor_crossing(appear_s: float = 5.0, leave_s: float = 55.0) -> VirtualPerson:
    """決定的瞬間の来場者: 横へ歩いて来て、正面で立ち止まり、しばらくして去る。"""
    return VirtualPerson("CROSSING", (-200.0, -500.0), 350.0, appear_s=appear_s, disappear_s=leave_s,
                         stop_at_x_mm=650.0)


def run_arc(cfg: dict, seed: int, seconds: float = 90.0) -> tuple[list[tuple[float, str]], list[tuple[float, str]]]:
    s = SimSession(cfg, START, seed=seed)
    walker = visitor_crossing()
    states: list[tuple[float, str]] = []
    while s.t < seconds:
        drive_people(s, [walker], s.t)
        st = s.step()
        if not states or states[-1][1] != st.state:
            states.append((s.t, st.state))
    return states, [(t, n.split()[0]) for t, n in s.brain.expr.log]


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_experience_arc_emerges_within_90s(cfg: dict, seed: int) -> None:
    states, prims = run_arc(cfg, seed)
    seq = [n for _, n in states]
    first = {n: t for t, n in reversed(states)}
    twitch = next(t for t, n in prims if n == "twitch")
    assert 0.15 <= twitch - 5.0 <= 0.30 + 0.05                   # NOTICE: 一次反応（BEHAVIOR_INTERNAL）
    for a, b in (("ALERT", "OBSERVE"), ("OBSERVE", "APPROACH"), ("APPROACH", "ENGAGE")):
        assert a in first and b in first and first[a] < first[b], (a, b, states)   # LOOK → TRACK → APPROACH → ENGAGE
    assert any(n == "sag" and first["ENGAGE"] < t < 55.0 for t, n in prims), prims   # RELAX（かかわった後に力を抜く）
    assert seq[-1] in ("PATROL", "COIL_REST_MOOD", "SLEEP") and states[-1][0] > 55.0   # LEAVE → REST
    assert first["ENGAGE"] < 60.0
    assert "RETREAT" not in seq                                    # 歩いて来る来場者から逃げない


def test_profile_has_no_threat_display_toward_people(cfg: dict) -> None:
    """Fear 監査: 人に向けた S 字・フル鎌首・速い首振りを使わない。"""
    nk, x = cfg["neck"], cfg["behavior"]["expression"]
    assert "s_curve" not in cfg["poses"]["rear_up"]["bases"]
    assert x["alert_neck_deg"] <= nk["look_max_deg"] and x["engage_neck_deg"] <= nk["look_max_deg"]
    assert x["look_duration_s"] >= 0.8                            # 速い首振りをしない
    g = cfg["behavior"]["grammar"]
    for state in ("ALERT", "OBSERVE", "ENGAGE", "APPROACH", "RETREAT", "PETTED"):
        items = [*g[state].get("on_enter", []), *g[state].get("while", [])]
        assert all(it["do"] != "stretch" for it in items), state    # フル鎌首は人がいる状態で出さない
    assert cfg["behavior"]["controller"]["speed_approach_mm_s"] <= 50.0   # 直進突撃にしない（忍び寄り）
    assert cfg["animator"]["head_max_accel_g"] <= 1.0


def test_neck_never_exceeds_look_range_while_tracking(cfg: dict) -> None:
    s = SimSession(cfg, START, seed=1)
    walker = visitor_crossing()
    worst = 0.0
    while s.t < 50.0:
        drive_people(s, [walker], s.t)
        s.step()
        if s.target is not None:
            worst = max(worst, s.anim.last_base["J7"])
    assert worst <= cfg["neck"]["look_max_deg"] + 1e-6
