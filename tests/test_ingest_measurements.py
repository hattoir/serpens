"""実測の取り込み口（`tools/ingest_measurements.py`）。**合成データで、与えた値が戻ってくること**を確かめる（実測は 0 件。FAKE の入力）。"""
from __future__ import annotations

import csv
import hashlib
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope="module")
def im():
    spec = importlib.util.spec_from_file_location("ingest_measurements", ROOT / "tools" / "ingest_measurements.py")
    m = importlib.util.module_from_spec(spec)
    sys.modules["ingest_measurements"] = m
    spec.loader.exec_module(m)
    return m


def write(path: Path, header: list[str], rows: list[dict]) -> Path:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=header)
        w.writeheader()
        w.writerows(rows)
    return path


def test_templates_list_the_columns(im, capsys) -> None:
    assert im.main(["--template", "stop_coast"]) == 0
    assert capsys.readouterr().out.strip() == ",".join(im.COLUMNS["stop_coast"])


def test_stop_coast_recovers_the_time_constant(im, tmp_path: Path) -> None:
    rng = np.random.default_rng(1)
    rows = [{"trial": i, "speed_mm_s": v, "coast_mm": 0.9 * v + rng.normal(0, 2.0)} for i, v in enumerate([30, 50, 80, 120] * 6)]
    res, md = im.ingest("stop_coast", write(tmp_path / "t.csv", im.COLUMNS["stop_coast"], rows), tmp_path / "out")
    assert res["tau_s"] == pytest.approx(0.9, rel=0.03) and md.exists() and (tmp_path / "out" / "stop_coast_t.json").exists()
    assert res["config"]["stop_distance_mm"] == 400.0 and res["n_speeds"] == 4


def test_capture_time_compares_with_far_trust(im, tmp_path: Path) -> None:
    rng = np.random.default_rng(2)
    rows = [{"trial": i, "settle_s": rng.normal(1.0, 0.1), "capture_s": rng.normal(1.6, 0.1)} for i in range(60)]
    res, _ = im.ingest("capture_time", write(tmp_path / "c.csv", im.COLUMNS["capture_time"], rows), tmp_path / "out")
    assert res["total"]["mean"] == pytest.approx(2.6, abs=0.1) and res["config"]["far_trust_s"] == 3.0
    assert res["exceeds_far_trust_fraction"] < 0.05 and 0.8 < res["p95_total_over_far_trust"] < 1.0


def test_gait_slip_by_floor(im, tmp_path: Path) -> None:
    rows = [{"trial": i, "floor": "WOOD", "commanded_advance_mm": 100, "measured_advance_mm": 88} for i in range(5)]
    rows += [{"trial": 10 + i, "floor": "RUG", "commanded_advance_mm": 100, "measured_advance_mm": 75} for i in range(5)]
    res, _ = im.ingest("gait_slip", write(tmp_path / "g.csv", im.COLUMNS["gait_slip"], rows), tmp_path / "out")
    assert res["by_floor"]["WOOD"]["mean"] == pytest.approx(0.12) and res["by_floor"]["RUG"]["mean"] == pytest.approx(0.25)
    assert res["config"]["odometry_sigma_along_frac"] == 0.15


def test_tag_detect_rates_with_confidence_interval(im, tmp_path: Path) -> None:
    rows = [{"trial": i, "distance_mm": 300, "detected": 1} for i in range(20)]
    rows += [{"trial": 100 + i, "distance_mm": 1100, "detected": int(i % 2 == 0)} for i in range(20)]
    res, _ = im.ingest("tag_detect", write(tmp_path / "d.csv", im.COLUMNS["tag_detect"], rows), tmp_path / "out")
    near, far = res["by_distance"]["250-500 mm"], res["by_distance"]["1000-1250 mm"]
    assert near["rate"] == 1.0 and near["ci95"][0] > 0.8 and far["rate"] == 0.5 and far["ci95"][0] < 0.5 < far["ci95"][1]


def test_imu_recovers_the_drift(im, tmp_path: Path) -> None:
    rng = np.random.default_rng(3)
    t = np.arange(0, 300, 1.0)
    yaw = np.degrees(0.003 * t + rng.normal(0, 0.01, t.size))
    rows = [{"trial": 0, "t_s": float(a), "yaw_deg": float(b), "ref_yaw_deg": 0.0} for a, b in zip(t, yaw)]
    res, _ = im.ingest("imu", write(tmp_path / "i.csv", im.COLUMNS["imu"], rows), tmp_path / "out")
    assert res["drift_rad_per_s"] == pytest.approx(0.003, rel=0.05)
    assert res["drift_ratio_to_config"] == pytest.approx(1.5, rel=0.06) and res["noise_sd_rad"] == pytest.approx(0.01, rel=0.2)


def test_contact_load_compares_with_the_provisional_threshold_without_a_verdict(im, tmp_path: Path) -> None:
    rows = [{"trial": i, "location": "hand_finger", "force_n": f, "instrument": "scale"} for i, f in enumerate([2.0, 3.0, 4.0])]
    res, md = im.ingest("contact_load", write(tmp_path / "f.csv", im.COLUMNS["contact_load"], rows), tmp_path / "out")
    s = res["by_location"]["hand_finger"]
    assert s["provisional_threshold_n"] == 5.7 and s["max_over_threshold"] == pytest.approx(4.0 / 5.7)
    text = md.read_text(encoding="utf-8")
    assert "SAFETY_UNVERIFIED" in text and "合格" not in text.replace("合否ではない", "")


def test_current_summarizes_without_touching_the_gate(im, tmp_path: Path) -> None:
    rows = [{"trial": i, "state": "idle", "current_a": 0.2, "peak_a": 0.3} for i in range(3)] + [{"trial": 9, "state": "stall", "current_a": 2.0, "peak_a": 2.7}]
    res, _ = im.ingest("current", write(tmp_path / "a.csv", im.COLUMNS["current"], rows), tmp_path / "out")
    assert res["by_state"]["stall"]["peak_a"] == 2.7 and res["by_state"]["idle"]["n"] == 3
    assert "COMPLETE" in res["note"] and "GATE" in res["note"]


def test_blank_cells_are_dropped_not_zero_filled_and_missing_columns_fail(im, tmp_path: Path) -> None:
    rows = [{"trial": 0, "speed_mm_s": 50, "coast_mm": 40}, {"trial": 1, "speed_mm_s": 50, "coast_mm": ""}]
    res, _ = im.ingest("stop_coast", write(tmp_path / "b.csv", im.COLUMNS["stop_coast"], rows), tmp_path / "out")
    assert res["coast"]["n"] == 1 and res["coast"]["mean"] == 40.0
    with pytest.raises(ValueError, match="必須の列"):
        im.ingest("gait_slip", write(tmp_path / "bad.csv", ["trial"], [{"trial": 1}]), tmp_path / "out")


def test_ingest_does_not_modify_the_config(im, tmp_path: Path) -> None:
    p = ROOT / "config" / "robot.yaml"
    before = hashlib.sha256(p.read_bytes()).hexdigest()
    rows = [{"trial": 0, "speed_mm_s": 50, "coast_mm": 40}]
    im.ingest("stop_coast", write(tmp_path / "x.csv", im.COLUMNS["stop_coast"], rows), tmp_path / "out")
    assert hashlib.sha256(p.read_bytes()).hexdigest() == before
