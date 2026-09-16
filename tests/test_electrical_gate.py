"""ELECTRICAL_SAFETY_GATE — 電気の実測が無い間は実機の自律走行が必ず止まること。"""
from __future__ import annotations

import copy

import pytest

from serpens.config import load_config
from serpens.electrical_gate import REQUIRED_ITEMS, electrical_gate_blockers, gate_items, gate_open
from serpens.safety import AutonomyInputs, autonomy_blockers

READY = AutonomyInputs(True, "aruco", 0.1, True, True, True, 9, 9, 0.2)


@pytest.fixture(scope="module")
def cfg() -> dict:
    return load_config()


def _complete(cfg: dict, evidence: str | None = "2026-xx-xx 測定器/条件") -> dict:
    out = copy.deepcopy(cfg)
    for item in out["safety_limits"]["electrical_safety_gate"].values():
        item.update(status="COMPLETE", evidence=evidence)
    return out


def test_gate_is_incomplete_today(cfg: dict) -> None:
    """実測ゼロの現在、全項目が未完了として並ぶ（HARDWARE_VERIFIED は 0 件）。"""
    items = gate_items(cfg)
    assert [i.key for i in items] == list(REQUIRED_ITEMS)
    assert not any(i.passed for i in items)
    assert all(i.evidence is None for i in items)
    assert not gate_open(cfg)
    assert len(electrical_gate_blockers(cfg)) == len(REQUIRED_ITEMS)


def test_required_items_cover_the_charter(cfg: dict) -> None:
    """要求された8項目（電流・容量・過電流・発熱・独立遮断・物理停止・温度・実停止時間）。"""
    assert set(REQUIRED_ITEMS) == {
        "real_current", "power_capacity", "overcurrent_protection", "wiring_heat",
        "independent_power_cut", "physical_estop", "servo_temperature", "real_stop_time"}


def test_real_autonomy_is_blocked_by_the_gate(cfg: dict) -> None:
    """他の条件がそろっても、ゲートが閉じていれば実機は走らない。"""
    why = autonomy_blockers(cfg, READY)
    assert why and all("電気安全ゲート" in w for w in why)


def test_simulation_is_not_blocked(cfg: dict) -> None:
    """模擬は従来どおり動く（モックファースト）。"""
    sim = AutonomyInputs(False, "sim", None, False, False, True, 9, 9, None)
    assert autonomy_blockers(cfg, sim) == []


def test_complete_without_evidence_does_not_pass(cfg: dict) -> None:
    """状態だけ COMPLETE にしても、証拠が無ければ通らない。"""
    for ev in (None, "", "   "):
        c = _complete(cfg, ev)
        assert not gate_open(c)
        assert all("証拠が無い" in w for w in electrical_gate_blockers(c))


def test_one_missing_item_keeps_gate_closed(cfg: dict) -> None:
    c = _complete(cfg)
    assert gate_open(c) and autonomy_blockers(c, READY) == []
    c["safety_limits"]["electrical_safety_gate"]["physical_estop"].update(status="INCOMPLETE")
    assert not gate_open(c)
    assert any("physical_estop" in w for w in autonomy_blockers(c, READY))


def test_malformed_gate_is_an_error(cfg: dict) -> None:
    """項目の削除・未知の状態は黙って通さない。"""
    c = copy.deepcopy(cfg)
    del c["safety_limits"]["electrical_safety_gate"]["real_stop_time"]
    with pytest.raises(ValueError):
        gate_items(c)
    c = copy.deepcopy(cfg)
    c["safety_limits"]["electrical_safety_gate"]["wiring_heat"]["status"] = "SIMULATED"
    with pytest.raises(ValueError):
        gate_items(c)
