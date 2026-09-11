"""STEP 4.5: fit_sim の CSV 読み込みと同定ロジック（シミュレータは回さない）。"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import fit_sim  # noqa: E402

from serpens.config import load_config  # noqa: E402

HEADER = "gait,period_s,cycles,distance_mm,floor,date,note\n"


def test_read_runs_utf8_and_cp932_and_skips_bad_rows(tmp_path: Path) -> None:
    presets = load_config()["gait"]["presets"]
    body = HEADER + "forward,2.0,5,1500,felt,2026-09-12,フェルト\nunknown,2,5,100,felt,,\nforward,x,5,1,felt,,\n"
    for enc in ("utf-8-sig", "cp932"):
        p = tmp_path / f"runs_{enc}.csv"
        p.write_text(body, encoding=enc)
        runs = fit_sim.read_runs(p, presets)
        assert len(runs) == 1 and runs[0].per_cycle_mm == pytest.approx(300.0) and runs[0].note == "フェルト"


def test_sample_csv_is_valid() -> None:
    root = Path(__file__).resolve().parent.parent
    runs = fit_sim.read_runs(root / "data" / "real_runs.csv", load_config()["gait"]["presets"])
    assert runs


def test_fit_recovers_ratio_from_synthetic_curve() -> None:
    """単調減少の曲線上の点を実測とすれば、その ratio が戻る（格子の間も補間）。"""
    ratios = np.linspace(0.0, 0.2, 21)
    curve = 450.0 * np.exp(-8.0 * ratios)
    true = 0.057
    run = fit_sim.Run("forward", 2.0, 4.0, 4 * 450.0 * np.exp(-8.0 * true), "f", "", "")
    best, loss, resid = fit_sim.fit([run], {("forward", 2.0): curve}, ratios)
    assert best == pytest.approx(true, abs=0.003)
    assert abs(resid[0]) < 3.0
