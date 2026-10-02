"""HG-H1 の実測の取り込み口（`simulation/hardware_gaps/HG-H1_actuator/run.py` の `ingest`）を合成データで確かめる（LB-E-055 の B-完了の条件）。

**実測ではない。** 与えた真の値（電流の傾き・ストール・熱時定数・上限）が当てはめで戻ること、足りない測定が「問題」に出ることを確かめる。
"""
from __future__ import annotations

import csv
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
HEADER = ["date", "operator", "servo_serial", "supply_v", "current_limit_a", "test_id", "condition", "arm_mm", "load_g", "torque_nm_calc", "current_a",
          "load_reg", "position_deg", "speed_dps", "temp_c", "voltage_reg_v", "duration_s", "instrument", "note"]


@pytest.fixture()
def h1(tmp_path: Path, monkeypatch):
    spec = importlib.util.spec_from_file_location("hg_h1_run", ROOT / "simulation" / "hardware_gaps" / "HG-H1_actuator" / "run.py")
    m = importlib.util.module_from_spec(spec)
    sys.modules["hg_h1_run"] = m
    spec.loader.exec_module(m)
    monkeypatch.setattr(m, "HERE", tmp_path)
    monkeypatch.setattr(m, "ROOT", tmp_path)
    return m


def _write(path: Path, rows: list[dict]) -> Path:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=HEADER)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in HEADER})
    return path


def test_ingest_recovers_the_fitted_values(h1, tmp_path: Path) -> None:
    kt, i0, stall_nm, stall_a, tau, rise, cap_nm = 2.4, 0.15, 2.7, 3.9, 600.0, 18.0, 0.46
    rows = [{"test_id": "4", "torque_nm_calc": t, "current_a": kt * t + i0} for t in (0.0, 0.1, 0.2, 0.3, 0.4)]
    rows += [{"test_id": "5", "torque_nm_calc": stall_nm, "current_a": stall_a}]
    rows += [{"test_id": "6", "duration_s": s, "temp_c": 25.0 + rise * (1 - math.exp(-s / tau))} for s in (60, 150, 300, 600, 900, 1500, 2400)]
    rows += [{"test_id": "7", "torque_nm_calc": cap_nm}]
    out = h1.ingest(_write(tmp_path / "h1.csv", rows))
    fit = json.loads(next((tmp_path / "results" / "measured").glob("*_fit.json")).read_text(encoding="utf-8"))
    f, problems = fit["fits"], fit["problems"]
    assert f["kt_a_per_nm"] == pytest.approx(kt, rel=1e-6) and f["no_load_current_a"] == pytest.approx(i0, rel=1e-6)
    assert f["stall_torque_nm"] == stall_nm and f["stall_current_a"] == stall_a
    assert f["thermal_time_constant_s"] == pytest.approx(tau, rel=0.05) and f["thermal_rise_c"] == pytest.approx(rise, rel=0.05)
    assert f["cap_error_vs_0p45"] == pytest.approx(cap_nm / 0.45 - 1.0)
    assert problems == [] and out.name.endswith("_decision.md")
    text = out.read_text(encoding="utf-8")
    assert "config は自動で書き換えない" in text                         # 安全に関わる値を自動で書き換えない（提案だけ）


def test_missing_measurements_are_reported_as_problems_and_nothing_is_invented(h1, tmp_path: Path) -> None:
    out = h1.ingest(_write(tmp_path / "h1.csv", [{"test_id": "4", "torque_nm_calc": 0.1, "current_a": 0.4}]))
    fit = json.loads(next((tmp_path / "results" / "measured").glob("*_fit.json")).read_text(encoding="utf-8"))
    assert fit["fits"] == {} and len(fit["problems"]) == 3                # 電流 3 点未満・ストール無し・温度 4 点未満
    assert "recommended_ratio" not in out.read_text(encoding="utf-8")
