"""指の挟み込みの判定（simulation/pinch_gap.py）。基準は config safety_limits.pinch（ASTM F963-11 4.18.1 と構想設計書の厳しい側）。"""
from __future__ import annotations

import pytest

from serpens.config import load_config
from simulation.pinch_gap import PinchRule, classify, max_constant_gap_mm, min_constant_gap_mm


@pytest.fixture(scope="module")
def rule() -> PinchRule:
    return PinchRule.from_cfg(load_config())


def test_the_rule_is_the_stricter_of_astm_and_the_concept_document(rule: PinchRule) -> None:
    assert rule.small_below_mm <= 5.0          # ASTM F963-11 4.18.1: 5mm の棒が入らない（構想設計書の 8mm は緩い）
    assert rule.large_from_mm >= 25.0          # 構想設計書 16 章（ASTM の 13mm より厳しい）


def test_a_gap_closing_through_the_band_fails(rule: PinchRule) -> None:
    rows = [{"angle_deg": a, "gap_mm": g} for a, g in [(0, 28.0), (20, 18.0), (40, 8.8), (50, 4.0)]]
    assert classify(rows, rule)["verdict"] == "FAIL"


def test_a_constant_small_gap_passes_only_with_tolerance_margin(rule: PinchRule) -> None:
    hi = max_constant_gap_mm(rule)
    ok = classify([{"angle_deg": a, "gap_mm": hi - 0.1} for a in range(0, 55, 5)], rule)
    bad = classify([{"angle_deg": a, "gap_mm": hi + 0.1} for a in range(0, 55, 5)], rule)
    assert ok["verdict"] == "PASS_SMALL" and bad["verdict"] == "FAIL"


def test_a_constant_gap_that_would_rub_is_reported(rule: PinchRule) -> None:
    r = classify([{"angle_deg": 0, "gap_mm": min_constant_gap_mm(rule) * 0.5}], rule)
    assert r["verdict"] == "PASS_SMALL" and r["rubbing_angles"] == [0]


def test_always_wide_passes_large(rule: PinchRule) -> None:
    assert classify([{"angle_deg": a, "gap_mm": 40.0} for a in (0, 25, 50)], rule)["verdict"] == "PASS_LARGE"
