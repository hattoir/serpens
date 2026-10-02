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


# ---- ELECTRICAL_SAFETY_GATE の材料（LB-E-046〜053）--------------------------------------------------------------
def test_power_sag_reports_drop_and_margin_to_the_assumed_servo_minimum(im, tmp_path: Path) -> None:
    rows = [{"trial": i, "event": "all_axes_start", "v_nominal": 7.4, "v_min": 6.4 + 0.05 * i, "brownout": 0} for i in range(4)]
    rows += [{"trial": 9, "event": "stall", "v_nominal": 7.4, "v_min": 5.5, "brownout": 1}]
    res, _ = im.ingest("power_sag", write(tmp_path / "p.csv", im.COLUMNS["power_sag"], rows), tmp_path / "out")
    a, s = res["by_event"]["all_axes_start"], res["by_event"]["stall"]
    assert a["drop_v"]["max"] == pytest.approx(1.0) and a["margin_to_servo_min_v"] == pytest.approx(0.4) and a["brownouts"] == 0
    assert s["margin_to_servo_min_v"] < 0 and s["brownouts"] == 1 and res["assumed_servo_min_v"] == 6.0


def test_trip_counts_trials_that_did_not_trip(im, tmp_path: Path) -> None:
    rows = [{"trial": 0, "device": "polyswitch", "i_set_a": 3.0, "trip_a": 3.2, "trip_ms": 800, "tripped": 1},
            {"trial": 1, "device": "polyswitch", "i_set_a": 3.0, "trip_a": 3.1, "trip_ms": 900, "tripped": 1},
            {"trial": 2, "device": "polyswitch", "i_set_a": 3.0, "tripped": 0}]
    res, _ = im.ingest("trip", write(tmp_path / "t.csv", im.COLUMNS["trip"], rows), tmp_path / "out")
    assert res["not_tripped_trials"] == 1 and res["by_device"]["polyswitch"]["tripped_fraction"] == pytest.approx(2 / 3)


def test_thermal_recovers_steady_rise_and_time_constant(im, tmp_path: Path) -> None:
    t = np.arange(0, 60, 1.0)
    temp = 25.0 + 18.0 * (1.0 - np.exp(-t / 8.0))
    rows = [{"trial": 0, "point": "connector", "t_min": float(a), "temp_c": float(b), "ambient_c": 25.0, "current_a": 2.0} for a, b in zip(t, temp)]
    res, _ = im.ingest("thermal", write(tmp_path / "h.csv", im.COLUMNS["thermal"], rows), tmp_path / "out")
    p = res["by_point"]["connector"]
    assert p["delta_t_steady_c"] == pytest.approx(18.0, abs=0.3) and p["tau_min"] == pytest.approx(8.0, rel=0.1)


def test_servo_temp_finds_the_stop_temperature_against_the_config_limit(im, tmp_path: Path) -> None:
    t = np.arange(0, 600, 10.0)
    temp = 25.0 + 0.07 * t
    rows = [{"trial": 0, "t_s": float(a), "temp_c": float(b), "load": 0.5, "stopped": int(b >= 60.0)} for a, b in zip(t, temp)]
    res, _ = im.ingest("servo_temp", write(tmp_path / "s.csv", im.COLUMNS["servo_temp"], rows), tmp_path / "out")
    assert res["rise_c_per_min"] == pytest.approx(0.07 * 60.0, rel=0.02)
    assert res["stopped_at_c"] is not None and 60.0 <= res["stopped_at_c"] < 61.0 and res["stopped_before_limit"] is True
    assert res["config"]["link_faults_temp_limit_c"] == 60.0


def test_stop_time_groups_by_trigger_and_marks_esp32_independent_trials(im, tmp_path: Path) -> None:
    rows = [{"trial": i, "trigger": "estop", "what": "supply_cut", "t_trigger_ms": 1000, "t_effect_ms": 1000 + 8 + i, "esp32_running": 0} for i in range(5)]
    rows += [{"trial": 10 + i, "trigger": "comm_loss", "what": "motion_stop", "t_trigger_ms": 0, "t_effect_ms": 380 + i, "esp32_running": 1} for i in range(5)]
    res, _ = im.ingest("stop_time", write(tmp_path / "st.csv", im.COLUMNS["stop_time"], rows), tmp_path / "out")
    e, c = res["by_trigger"]["estop/supply_cut"], res["by_trigger"]["comm_loss/motion_stop"]
    assert e["ms"]["mean"] == pytest.approx(10.0) and e["trials_with_esp32_stopped"] == 5 and c["trials_with_esp32_stopped"] == 0
    assert res["config"]["heartbeat_timeout_ms"] == 400.0 and "合否ではない" in res["note"]


def test_power_sag_estimates_the_total_source_resistance(im, tmp_path: Path) -> None:
    rows = [{"trial": i, "event": "stall", "v_nominal": 7.4, "v_min": 7.4 - 0.20 * ip, "i_peak_a": ip, "brownout": 0} for i, ip in enumerate([1.0, 1.5, 2.0, 2.5])]
    res, _ = im.ingest("power_sag", write(tmp_path / "r.csv", im.COLUMNS["power_sag"], rows), tmp_path / "out")
    assert res["by_event"]["stall"]["r_total_ohm"]["mean"] == pytest.approx(0.20)


def test_exposure_groups_by_setting_and_counts_saturated_frames(im, tmp_path: Path) -> None:
    rows = [{"trial": i, "light": "bright", "floor": "flooring", "object": "cr2032", "exposure_mode": "fixed", "gain": 2.5, "saturated_pct": 80 + i % 5, "diameter_px": 30, "detected": 0}
            for i in range(10)]
    rows += [{"trial": 100 + i, "light": "bright", "floor": "flooring", "object": "cr2032", "exposure_mode": "auto", "gain": "", "saturated_pct": 1.0 + i % 3, "diameter_px": 14, "detected": 1}
             for i in range(10)]
    res, _ = im.ingest("exposure", write(tmp_path / "e.csv", im.COLUMNS["exposure"], rows), tmp_path / "out")
    fx, au = res["by_exposure"]["fixed/gain=2.5"], res["by_exposure"]["auto"]
    assert fx["n"] == 10 and fx["frac_over_pct"] == 1.0 and fx["detect"]["rate"] == 0.0
    assert au["n"] == 10 and au["frac_over_pct"] == 0.0 and au["detect"]["rate"] == 1.0 and au["saturated_pct"]["max"] < 5


def _j1_rows_dead_backlash(d_steps: int, backlash_steps: float, motor_side: bool = True) -> list[dict]:
    rows = []
    for g in range(-5, 6):                                                      # J1-1: 目標 ±5 ステップ、上から / 下から。読み値は目標から最大 d_steps ずれる
        for dr, sgn in (("up", 1), ("down", -1)):
            rows.append({"trial": len(rows), "test_id": "J1-1", "direction": dr, "goal_step": g, "servo_present_step": g + sgn * (abs(g) % (d_steps + 1)), "output_mm": ""})
    mm_per_step = 44.0 * math.radians(360.0 / 4096.0)
    for g in (-10, 0, 10):                                                      # J1-2: 同じ目標へ上から / 下から。出力側の差 = backlash_steps × 1 ステップの弧長
        for dr, sgn in (("up", 0.5), ("down", -0.5)):
            rows.append({"trial": len(rows), "test_id": "J1-2", "direction": dr, "goal_step": g, "servo_present_step": g if motor_side else g + sgn * backlash_steps,
                         "output_mm": sgn * backlash_steps * mm_per_step})
    return rows


def test_j1_dead_band_and_backlash_are_recovered_and_replace_the_prior(im, tmp_path: Path) -> None:
    rows = _j1_rows_dead_backlash(3, 6.0, motor_side=True)
    res, _ = im.ingest("j1_dead_backlash", write(tmp_path / "j.csv", im.COLUMNS["j1_dead_backlash"], rows), tmp_path / "out")
    assert res["dead_band_steps"] == 3.0
    assert abs(res["backlash_steps"] - 6.0) < 1e-6 and res["encoder_on_motor_side"] is True
    assert abs(res["position_error_steps"] - (0.5 + 3 + 6.0)) < 1e-9
    # 位置の誤差が大きいほど、窓の端の手前で出せる速さは小さくなる（ストッパーが窓の端に近い候補ほど先に成り立たなくなる）
    res0, _ = im.ingest("j1_dead_backlash", write(tmp_path / "j0.csv", im.COLUMNS["j1_dead_backlash"], _j1_rows_dead_backlash(0, 0.0)), tmp_path / "out0")
    for k, v in res["edges"].items():
        assert v["cap_dps_at_conservative_decel"] <= res0["edges"][k]["cap_dps_at_conservative_decel"] + 1e-9
    assert any(v["feasible"] for v in res0["edges"].values()) and not all(v["feasible"] for v in res["edges"].values())


def test_j1_backlash_on_the_output_encoder_is_not_added(im, tmp_path: Path) -> None:
    rows = _j1_rows_dead_backlash(2, 6.0, motor_side=False)
    res, _ = im.ingest("j1_dead_backlash", write(tmp_path / "j.csv", im.COLUMNS["j1_dead_backlash"], rows), tmp_path / "out")
    assert res["encoder_on_motor_side"] is False and abs(res["position_error_steps"] - (0.5 + res["dead_band_steps"])) < 1e-9


def test_j1_push_subtracts_the_torque_off_baseline(im, tmp_path: Path) -> None:
    rows = [{"test_id": "J1-3", "register_torque_limit": 0, "torque_on": 0, "force_n_mean": 1.0, "force_n_sd": 0.01, "n_samples": 10},
            {"test_id": "J1-3", "register_torque_limit": 8, "torque_on": 1, "force_n_mean": 1.5, "force_n_sd": 0.01, "n_samples": 10},
            {"test_id": "J1-3", "register_torque_limit": 20, "torque_on": 1, "force_n_mean": 4.2, "force_n_sd": 0.01, "n_samples": 10}]
    res, _ = im.ingest("j1_push", write(tmp_path / "p.csv", im.COLUMNS["j1_push"], rows), tmp_path / "out")
    assert abs(res["by_register"]["8"]["servo_force_n"] - 0.5) < 1e-9 and abs(res["by_register"]["20"]["ratio_to_2p8_n"] - 3.2 / 2.8) < 1e-9


def test_j1_stopper_peaks_are_grouped_and_compared_with_the_pin(im, tmp_path: Path) -> None:
    rows = [{"speed_dps": 120, "damper": "tpu", "decel_deg": 3, "torque_limit_reg": 167, "peak_n": p, "rise_ms": 2, "load_cell_fn_hz": 800} for p in (30, 40, 50)]
    rows += [{"speed_dps": 120, "damper": "none", "decel_deg": 0, "torque_limit_reg": 167, "peak_n": 120, "rise_ms": 1, "load_cell_fn_hz": 800}]
    res, _ = im.ingest("j1_stopper", write(tmp_path / "s.csv", im.COLUMNS["j1_stopper"], rows), tmp_path / "out")
    a, b = res["by_setting"]["tpu/decel=3"], res["by_setting"]["none/decel=0"]
    assert a["n"] == 3 and a["peak_n"]["max"] == 50.0 and b["max_over_steel_pin_capacity"] > a["max_over_steel_pin_capacity"]
    assert abs(a["max_over_steel_pin_capacity"] - 50.0 / res["steel_pin_capacity_n_at_200mpa"]) < 1e-9


def test_tof_floor_counts_missed_cliffs_separately_and_valid_fraction(im, tmp_path: Path) -> None:
    rows = [{"test": "T6-2", "surface": "wood", "scenario": "floor", "distance_mm": 45, "range_status": 0, "classified": "floor"} for _ in range(18)]
    rows += [{"test": "T6-2", "surface": "wood", "scenario": "floor", "distance_mm": 45, "range_status": 0, "classified": "cliff"} for _ in range(2)]
    rows += [{"test": "T6-2", "surface": "edge", "scenario": "cliff", "distance_mm": 90, "range_status": 0, "classified": "cliff"} for _ in range(19)]
    rows += [{"test": "T6-2", "surface": "edge", "scenario": "cliff", "distance_mm": 50, "range_status": 0, "classified": "floor"}]
    rows += [{"test": "T6-4", "surface": "black", "scenario": "floor", "distance_mm": "", "range_status": 4, "classified": "cliff"} for _ in range(10)]
    res, _ = im.ingest("tof_floor", write(tmp_path / "t.csv", im.COLUMNS["tof_floor"], rows), tmp_path / "out")
    w, e, b = res["by_surface"]["T6-2/wood"], res["by_surface"]["T6-2/edge"], res["by_surface"]["T6-4/black"]
    assert w["false_cliff_rate"] == 0.1 and w["valid_fraction"] == 1.0
    assert e["cliff_missed"]["k"] == 1 and e["cliff_missed"]["n"] == 20 and e["cliff_missed"]["ci95"][1] > 0.05
    assert b["valid_fraction"] == 0.0 and b["false_cliff_rate"] == 1.0                      # 黒い床で無効 → 崖と誤判定（止まる側 = 安全側）


def test_limiter_slip_deviation_and_window_fraction(im, tmp_path: Path) -> None:
    vals = [0.8, 0.9, 1.0, 0.7, 0.85, 0.95, 0.75, 0.65, 1.05, 0.8]
    rows = [{"specimen": i, "direction": "cw", "slip_torque_nm": v, "temp_c": 25, "cycle": 0} for i, v in enumerate(vals)]
    rows += [{"specimen": i, "direction": "cw", "slip_torque_nm": v * 0.5, "temp_c": 60, "cycle": 0} for i, v in enumerate(vals)]
    res, _ = im.ingest("limiter_slip", write(tmp_path / "l.csv", im.COLUMNS["limiter_slip"], rows), tmp_path / "out")
    a, b = res["by_condition"]["cw/T=25/cycle=0"], res["by_condition"]["cw/T=60/cycle=0"]
    assert abs(a["mean_nm"] - float(np.mean(vals))) < 1e-9 and a["n"] == 10
    assert a["in_window_fraction"] == 0.8 and b["in_window_fraction"] == 0.0               # 65 と 105 は窓の外 / 60 ℃では全部下回る
    assert a["deviation_over_window_closing_prior"] is False


def test_cable_cycle_finds_first_wear_and_break(im, tmp_path: Path) -> None:
    rows = [{"trial": i, "cycles": c, "continuity_ohm": 0.10 + (0.02 if c >= 700 else 0.0), "wear": 1 if c >= 500 else 0, "broken": 1 if c >= 900 else 0}
            for i, c in enumerate((0, 100, 300, 500, 700, 900, 1000))]
    res, _ = im.ingest("cable_cycle", write(tmp_path / "c.csv", im.COLUMNS["cable_cycle"], rows), tmp_path / "out")
    assert res["max_cycles"] == 1000 and res["first_wear_at_cycles"] == 500 and res["first_break_at_cycles"] == 900
    assert abs(res["continuity_change_frac"] - 0.2) < 1e-9


def test_hood_edge_pressure_is_peak_over_area(im, tmp_path: Path) -> None:
    rows = [{"trial": i, "edge": "tpu_lip", "contact_area_mm2": 40.0, "drop_height_mm": 10, "peak_n": 8.0} for i in range(5)]
    rows += [{"trial": 10 + i, "edge": "sharp", "contact_area_mm2": 4.0, "drop_height_mm": 10, "peak_n": 8.0} for i in range(5)]
    res, _ = im.ingest("hood_edge", write(tmp_path / "h.csv", im.COLUMNS["hood_edge"], rows), tmp_path / "out")
    a, b = res["by_edge"]["tpu_lip"], res["by_edge"]["sharp"]
    assert abs(a["mean_pressure_kpa"]["mean"] - 200.0) < 1e-9 and abs(b["mean_pressure_kpa"]["mean"] - 2000.0) < 1e-9
    assert a["contact_area_mm2"]["n"] == 5
