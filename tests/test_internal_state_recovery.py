"""内部状態 6 種の回復可能性（property test）。

Bug-1 で起きた「state = 1 → 複数の効用が同時に 0 → 回復不能」の構造を、全状態について機械的に確かめる:
  1. どの状態も、刺激を止めれば有限時間で baseline へ戻る（飽和からでも）
  2. どの状態が飽和していても、もっともらしい文脈（誰もいない / 遠い / 近い / 撫でられ）で
     正の効用を持つ状態が 2 つ以上あり、効用が単一の状態に独占されない
  3. Stress が飽和しても、近くにいる人への ENGAGE / OBSERVE は 0 にならない（saturation < 1）
"""
from __future__ import annotations

import random

import pytest

from serpens.behavior.internal_state import STATE_NAMES, InternalState, Stimuli
from serpens.behavior.utility import Context, UtilityModel
from serpens.config import load_config

CONTEXTS = {
    "alone": Context(False, None, 0.0, 0.0),
    "far": Context(True, 1200.0, 0.0, 0.0),
    "near": Context(True, 300.0, 0.0, 0.0),
    "touch": Context(True, 300.0, 1.0, 0.0),
    "novel": Context(True, 900.0, 0.0, 1.0),
}
LIVE = [s for s in ("SLEEP", "PATROL", "OBSERVE", "APPROACH", "ENGAGE", "PETTED", "RETREAT", "COIL_REST_MOOD")]


@pytest.fixture(scope="module")
def cfg() -> dict:
    return load_config()


@pytest.mark.parametrize("name", STATE_NAMES)
def test_every_state_recovers_from_saturation_without_stimulus(cfg: dict, name: str) -> None:
    st = InternalState(cfg)
    ch = st.channel(name)
    for extreme in (ch.saturation, 0.0):
        st.values[name] = extreme
        for _ in range(int(5 * max(ch.decay_tau_s, ch.rise_tau_s) / 0.05)):
            st.update(0.05, Stimuli(), None)
        assert abs(st.values[name] - ch.baseline) <= 0.05 * max(ch.saturation, 1e-9) + 1e-6, (name, extreme, st.values[name])


def test_energy_recovers_after_rest(cfg: dict) -> None:
    st = InternalState(cfg)
    st.fatigue = 1.0
    tau = cfg["behavior"]["energy"]["fatigue"]["tau_recover_s"]
    for _ in range(int(5 * tau / 0.05)):
        st.update(0.05, Stimuli(), None, moving=False)
    assert st.energy > 0.95


@pytest.mark.parametrize("name", STATE_NAMES + ("energy",))
@pytest.mark.parametrize("extreme", ("max", "min"))
def test_no_state_extreme_leaves_a_single_reachable_action(cfg: dict, name: str, extreme: str) -> None:
    """ある状態が飽和/枯渇しても、外界（文脈）が変われば選ばれる行動が変わる（1 つの行動に固定されない）。

    加えて、人が近くにいる文脈では正の効用が 2 つ以上ある（社会的な状態が同時に消えない）。
    """
    u = UtilityModel(cfg, random.Random(0))
    u.noise = 0.0
    st = InternalState(cfg)
    if name == "energy":
        st.fatigue = 1.0 if extreme == "max" else 0.0
    else:
        st.values[name] = st.channel(name).saturation if extreme == "max" else 0.0
    winners = set()
    for label, ctx in CONTEXTS.items():
        raw = u.evaluate(st, ctx, 0.0).raw
        positive = [k for k in LIVE if raw[k] > 1e-9]
        assert positive, (name, extreme, label, raw)
        winners.add(max(positive, key=lambda k: raw[k]))
        if label in ("near", "touch"):
            assert len(positive) >= 2, (name, extreme, label, raw)
    assert len(winners) >= 2, (name, extreme, winners)


def test_stress_saturation_keeps_social_states_alive(cfg: dict) -> None:
    """Bug-1 の構造: Stress 飽和で (1−s) が 0 になり APPROACH/ENGAGE/OBSERVE が同時に消える、を起こさない。"""
    u = UtilityModel(cfg, random.Random(0))
    u.noise = 0.0
    st = InternalState(cfg)
    st.values["stress"] = st.channel("stress").saturation
    assert st.channel("stress").saturation < 1.0
    raw = u.evaluate(st, CONTEXTS["near"], 0.0).raw
    assert raw["ENGAGE"] > 0.0 and raw["OBSERVE"] > 0.0
    assert raw["RETREAT"] > raw["ENGAGE"]                    # 飽和中は退避が勝つのは正しい
    # 刺激が止まれば、decay_tau の数倍で退避が負ける（回復経路がある）
    for _ in range(int(3 * st.channel("stress").decay_tau_s / 0.05)):
        st.update(0.05, Stimuli(presence=1.0, proximity=0.8), None)
    raw = u.evaluate(st, CONTEXTS["near"], 0.0).raw
    assert raw["ENGAGE"] > raw["RETREAT"]


def test_familiarity_rises_with_calm_contact_and_drops_on_events(cfg: dict) -> None:
    st = InternalState(cfg)
    for _ in range(int(60.0 / 0.05)):
        st.update(0.05, Stimuli(presence=1.0, proximity=0.7, calm=1.0), None)
    assert 0.2 <= st.familiarity <= st.channel("familiarity").saturation
    before = st.familiarity
    st.kick("familiarity", cfg["behavior"]["kicks"]["retreat_familiarity"])
    assert st.familiarity < before
    for _ in range(int(2 * st.channel("familiarity").decay_tau_s / 0.05)):
        st.update(0.05, Stimuli(alone=1.0), None)
    assert st.familiarity < 0.2 * before + 0.05                # 誰もいないと抜けていく


def test_sleepiness_builds_alone_and_clears_with_people(cfg: dict) -> None:
    st = InternalState(cfg)
    for _ in range(int(300.0 / 0.05)):
        st.update(0.05, Stimuli(alone=1.0), None)
    sleepy = st.sleepiness
    assert sleepy > 0.5
    for _ in range(int(20.0 / 0.05)):
        st.update(0.05, Stimuli(presence=1.0, novelty=0.5), None)
    assert st.sleepiness < sleepy * 0.5
