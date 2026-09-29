"""二重の安全層（ソフトの上限 + 機械式トルクリミッター）と、暫定の接触しきい値（PROVISIONAL / SAFETY_UNVERIFIED）。

DEC-USER-0002（2026-09-29）。しきい値は**規格・子どもの計測値から導いた暫定値**で、確定ではない。
このテストは「値が正しい」ことではなく、**導出の整合と、緩める方向の変更を検出すること**を守る。
"""
from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from serpens.config import load_config

ROOT = Path(__file__).resolve().parent.parent
GAPS = ROOT / "simulation" / "hardware_gaps"


@pytest.fixture(scope="module")
def cfg() -> dict:
    return load_config()


@pytest.fixture(scope="module")
def thr() -> dict:
    return yaml.safe_load((GAPS / "safety_thresholds.yaml").read_text(encoding="utf-8"))


def test_mechanical_limiter_sits_above_the_software_cap_with_tolerance(cfg: dict) -> None:
    """個体差 −30% を見込んでも、ソフトの上限（0.45 N·m）より下では滑らない = 通常の移動で不要に滑らない。"""
    t = cfg["safety_limits"]["torque"]
    m = t["mechanical_limiter"]
    lo, hi = (float(x) for x in m["slip_nm_window"])
    base, tol = float(m["slip_nm_baseline"]), float(m["slip_tolerance"])
    assert lo <= base <= hi
    assert base * (1.0 - tol) >= float(t["software_torque_limit_nm"]), "リミッターがソフトの上限より先に滑りうる"
    assert base <= float(cfg["safety_limits"]["torque_nm_max"]), "構想設計書 16 章の 1.2 N·m を超えている"
    assert m["hardware_verified"] is False and m["status"] == "DESIGN_PROVISIONAL"
    assert m["layout"] == "output_side", "入力側では外力が歯車を通り、歯車を守れない"


def test_limiter_is_independent_of_the_software_ratio(cfg: dict) -> None:
    """二重の層: リミッターの滑りトルクはレジスタ比から計算しない（ソフトが壊れても値が変わらない）。"""
    t = cfg["safety_limits"]["torque"]
    stall = float(t["stall_torque_reference_nm"])
    # レジスタ比 × ストール = ソフトの上限。リミッターはこれより大きい別の値
    assert float(t["software_torque_limit_ratio"]) * stall == pytest.approx(float(t["software_torque_limit_nm"]), abs=0.01)
    assert float(t["mechanical_limiter"]["slip_nm_baseline"]) > float(t["software_torque_limit_nm"])


def test_thresholds_are_marked_provisional(thr: dict) -> None:
    assert thr["status"] == "PROVISIONAL_SAFETY_UNVERIFIED"
    assert thr["unverified"], "未確認の一覧が空 = 確定したように見える"


def test_every_baseline_is_the_smallest_candidate(thr: dict) -> None:
    """候補が複数あれば安全側（最小）を baseline にする（DEC-USER-0002 §2）。"""
    for region, v in thr["quasi_static_contact_force_n"].items():
        assert float(v["baseline"]) == pytest.approx(min(float(c["value"]) for c in v["candidates"])), region


def test_adult_values_are_scaled_not_copied(thr: dict) -> None:
    """成人の規格値（ISO/TS 15066 の 65〜160 N）をそのまま子どもの baseline にしていない。"""
    for region, v in thr["quasi_static_contact_force_n"].items():
        assert float(v["baseline"]) < 20.0, f"{region}: 成人の値が転用されている疑い"
    assert float(thr["pressure_limit_n_per_cm2"]["baseline"]) < 20.0
    s = thr["pain_scaling"]
    assert s["factor_conservative"] == pytest.approx(
        s["child_pressure_pain_thenar_n_per_cm2"]["mean_minus_1sd"] / s["adult_iso15066_thenar_n_per_cm2"], abs=0.001)


def test_impact_is_not_relaxed_by_the_adult_factor(thr: dict) -> None:
    assert float(thr["transient_impact_factor"]["baseline"]) <= float(thr["transient_impact_factor"]["iso15066_adult"])


def test_anthropometry_matches_the_recorded_circumferences() -> None:
    """直径 = 円周 / π（円断面の仮定）。首・手首の最小と大きい側。"""
    import math
    a = yaml.safe_load((GAPS / "child_anthropometry.yaml").read_text(encoding="utf-8"))["diameter_mm"]
    assert a["neck"]["min"] == pytest.approx(200.0 / math.pi, abs=0.1)
    assert a["neck"]["p95_6to7y"] == pytest.approx(278.0 / math.pi, abs=0.1)
    assert a["wrist"]["min"] == pytest.approx(92.0 / math.pi, abs=0.1)
    assert a["wrist"]["p95_6to7y"] == pytest.approx(136.0 / math.pi, abs=0.1)


def test_yaw_sum_leaves_room_below_the_reentrant_pocket(cfg: dict) -> None:
    """角度合計の上限は、C 字の口が袋小路になる 180° を下回る（頭ヨーも鎖に含めて掛かる）。"""
    lim = float(cfg["link"]["limits"]["yaw_sum_deg"])
    assert lim < 180.0
    assert lim == pytest.approx(145.0), "暫定 145°（DEC-USER-0002）から変えるなら DECISIONS.md に理由と evidence を残す"
