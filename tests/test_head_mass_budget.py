"""頭の質量予算（`simulation/hardware_gaps/HG-S3_torque_limiter/head_mass_budget.py`）。ANALYTIC + ACTUATOR_MODEL_SIM。Design の殻の質量は Design の見積もり、電子部品は ASSUMED。"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent


def _load(name: str, rel: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / rel)
    m = importlib.util.module_from_spec(spec)
    sys.modules[name] = m
    spec.loader.exec_module(m)
    return m


@pytest.fixture(scope="module")
def hb():
    return _load("head_mass_budget", "simulation/hardware_gaps/HG-S3_torque_limiter/head_mass_budget.py")


@pytest.fixture(scope="module")
def opt():
    return _load("j1_range_opt_for_mass", "simulation/hardware_gaps/HG-H1_actuator/j1_range_opt.py")


def test_zero_increment_reproduces_the_config_baseline(hb) -> None:
    """増分 0 のとき、config の `neck_lifted_mass` 200 g × 重心 104 mm = 0.204 N·m（docs/safety_limits.md の 0.20 N·m と同じ）。"""
    assert hb.static_torque_nm(0.0, 19.0) == pytest.approx(0.2 * 9.81 * 0.104, rel=1e-9)
    assert hb.static_torque_nm(0.0, 19.0) == pytest.approx(0.204, abs=0.001)


def test_static_torque_grows_with_mass_and_the_far_lever_is_the_conservative_bound(hb) -> None:
    prev = 0.0
    for shell in sorted(hb.SHELL_G):
        d = hb.head_total_g(shell, 15.0) - hb.budget()["head_total"]
        near, far = hb.static_torque_nm(d, hb.DESIGN_COG_MM), hb.static_torque_nm(d, hb.CFG_COG_MM)
        assert far > near > hb.static_torque_nm(0.0, 19.0) and far > prev
        prev = far


def test_shell_masses_are_designs_estimate_and_exceed_the_90g_budget(hb) -> None:
    assert hb.SHELL_G == {1.2: 137.0, 1.6: 169.0, 2.0: 199.0, 2.5: 234.0}      # ENTRY-D-0015 (4)
    assert all(m > hb.budget()["head_total"] for m in hb.SHELL_G.values())      # 90 g の予算を満たさない（Design の指摘）


def test_total_mass_table_stays_inside_the_config_cap(hb) -> None:
    """全体の最大（頭 254 g + 追加の最大）でも `mass_total_g_max` 内。上限そのものは変えない。"""
    t = hb.total_mass_g(max(hb.SHELL_G.values()) + 20.0 - hb.budget()["head_total"])
    assert t["min"] < t["max"] < t["cap"] == 1700.0


def test_cog_y_limit_is_inversely_proportional_to_mass(hb) -> None:
    assert hb.y_limit_mm(97.0, 1.0) == pytest.approx(4.2, abs=0.05)             # 既存の `4.2 mm × k`（HT-012）
    assert hb.y_limit_mm(194.0, 1.0) == pytest.approx(hb.y_limit_mm(97.0, 1.0) / 2.0, rel=1e-9)


def test_default_load_inertia_does_not_change_the_stopper_model(opt) -> None:
    """`j_load_kgm2` の既定 0 は従来の計算と同じ（回帰）。足すと衝撃が増える。"""
    base = opt.stopper_mc(n=3000, omega=120.0, decel_deg=0.0, e_mpa=26.0, t_mm=3.0)
    same = opt.stopper_mc(n=3000, omega=120.0, decel_deg=0.0, e_mpa=26.0, t_mm=3.0, j_load_kgm2=0.0)
    more = opt.stopper_mc(n=3000, omega=120.0, decel_deg=0.0, e_mpa=26.0, t_mm=3.0, j_load_kgm2=2.75e-3)
    assert base == same
    assert more["impact_p50"] > base["impact_p50"] and more["sf_p05"] < base["sf_p05"]


def test_load_inertia_with_design_cog_is_small_compared_with_the_rotor_prior(hb) -> None:
    j = hb.load_inertia_kgm2(254.0, hb.DESIGN_COG_MM)
    assert j < 1.0e-3 * 0.1                                                      # ロータ反映慣性 prior の下限（1e-3）の 1 割未満
    assert hb.load_inertia_kgm2(254.0, hb.CFG_COG_MM) > 1.0e-3 * 2.0             # 保守側の重心では下限の 2 倍以上（無視できない）


def test_design_v3_head_is_113_8_g_plus_electronics_and_its_cog_is_16_2_mm_ahead_of_j1(hb) -> None:
    """ENTRY-D-0021: シェル 77.7 + 取り込み 23.8 + スキッド 10.3 = 113.8 g、重心 x −198.0（J1 軸 −181.8 → 16.2 mm）、y +3.1。D-0015 の 137〜234 g は全体の値だった。"""
    # Design の報告: 内訳 77.7 + 23.8 + 10.3 = 111.8 g に対し、合計は 113.8 g（2 g の差は Design の内訳に無い部品。ENTRY-E-0017 で確認を依頼）。合計の値を使う
    assert hb.V3_HEAD_G == 113.8 and abs(hb.V3_HEAD_G - (77.7 + 23.8 + 10.3) - 2.0) < 1e-9 and hb.V3_COG_X_MM == pytest.approx(16.2) and hb.V3_COG_Y_MM == 3.1
    m = hb.V3_HEAD_G + hb.V3_ELEC_G[1]
    assert 1.5 < m / hb.budget()["head_total"] < 1.65
    assert hb.static_torque_nm(m - 90.0, 104.0) / hb.SOFT_CAP_NM < 0.6                           # 保守側でも上限の 6 割弱
    assert 3.1 / hb.y_limit_mm(m, 1.0) == pytest.approx(1.09, abs=0.02)                            # k ≥ 約 1.1 N/mm が要る（余裕が小さい）
