"""頭スキッドで体の前側を支えるときの J1 の負荷（`simulation/hardware_gaps/HG-S3_torque_limiter/skid_support.py`）。ANALYTIC + Monte Carlo（prior）。f・割増・腕は ASSUMED。"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope="module")
def sk():
    spec = importlib.util.spec_from_file_location("skid_support", ROOT / "simulation/hardware_gaps/HG-S3_torque_limiter/skid_support.py")
    m = importlib.util.module_from_spec(spec)
    sys.modules["skid_support"] = m
    spec.loader.exec_module(m)
    return m


def test_zero_support_leaves_only_the_heads_own_moment(sk) -> None:
    t = sk.j1_torque_nm(0.0, 1200.0, 1.5, 44.0, 200.0, 19.0)
    assert t == pytest.approx(0.2 * 9.81 * 0.019, rel=1e-9)


def test_torque_grows_linearly_with_the_supported_fraction(sk) -> None:
    a, b, c = (sk.j1_torque_nm(f, 1200.0, 1.5, 44.0, 200.0, 19.0) for f in (0.1, 0.2, 0.3))
    assert (c - b) == pytest.approx(b - a, rel=1e-9) and c > b > a


def test_boundary_fraction_inverts_the_torque(sk) -> None:
    f = sk.f_for_fraction_of_cap(0.75, 1200.0, 1.5, 44.0, 200.0, 19.0)
    assert sk.j1_torque_nm(f, 1200.0, 1.5, 44.0, 200.0, 19.0) == pytest.approx(0.75 * sk.SOFT_CAP_NM, rel=1e-9)


def test_worst_inputs_reach_the_soft_cap_before_half_the_body_is_supported(sk) -> None:
    """最悪の入力（全体最大・割増 2・腕 48 mm・頭最大）では、頭が体の重さの約 3 割を受けるだけで J1 の静的トルクがソフトの上限（0.45 N·m）に届く。
    Design に f の上限を依頼する根拠（R-027）。"""
    m = sk.masses()
    f = sk.f_for_fraction_of_cap(1.0, m["total_hi"], sk.K_DYN[1], sk.L_SKID_MM[1], m["head_hi"], m["cog_mm"])
    assert 0.25 < f < 0.40
    assert sk.f_for_fraction_of_cap(1.0, m["total_lo"], sk.K_DYN[0], sk.L_SKID_MM[0], m["head_lo"], m["cog_mm"]) > 0.9   # 最良の入力ならほぼ全体でも届かない


def test_monte_carlo_is_reproducible_and_ordered(sk) -> None:
    a, b = sk.mc(n=4000), sk.mc(n=4000)
    assert a["p50"] == b["p50"] and a["p95"] == b["p95"]
    assert a["p50"] < a["p95"] <= a["max"] and a["p_gt"][0.5] >= a["p_gt"][0.75] >= a["p_gt"][1.0]
