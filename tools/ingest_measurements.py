"""実機の測定記録（CSV）を 1 コマンドで解析する取り込み口（LB-E-013 / SE-E6）。**実測が来たときの受け口。今は合成データでの試験だけ（実測 0 件）。**

    python tools/ingest_measurements.py stop_coast   <csv>
    python tools/ingest_measurements.py capture_time <csv>
    python tools/ingest_measurements.py gait_slip    <csv>
    python tools/ingest_measurements.py tag_detect   <csv>
    python tools/ingest_measurements.py imu          <csv>
    python tools/ingest_measurements.py contact_load <csv>
    python tools/ingest_measurements.py current      <csv>
    python tools/ingest_measurements.py power_sag    <csv>      # 電圧降下・突入（GATE power_capacity）
    python tools/ingest_measurements.py trip         <csv>      # 過電流保護の動作（GATE overcurrent_protection）
    python tools/ingest_measurements.py thermal      <csv>      # 配線・コネクタの発熱（GATE wiring_heat）
    python tools/ingest_measurements.py servo_temp   <csv>      # サーボの温度上昇と停止（GATE servo_temperature）
    python tools/ingest_measurements.py stop_time    <csv>      # 電源遮断・緊急停止・通信断から止まるまで（GATE independent_power_cut / physical_estop / real_stop_time）
    python tools/ingest_measurements.py --template <kind>      # 列の雛形を出す

出力: `simulation/results/measured/<kind>_<csv の日付またはファイル名>.md`（と `.json`）。**設定ファイルは書き換えない**（`config/robot.yaml` との比較だけ。変更が要るなら Engineering が根拠つきの別 commit にする）。
**検証区分は上げない**: この道具は「その個体・その条件の実測の要約」を出す。`HARDWARE_VERIFIED` と書けるのは、測定の日付・条件・測定器・生データのファイルが揃った記録だけ（`global/VERIFICATION_LEVELS.md`）。
`ELECTRICAL_SAFETY_GATE` の状態は**書き換えない**（`serpens/electrical_gate.py` は証拠つきの記録を要求する。ここは要約を出すだけ）。接触の力は暫定しきい値（PROVISIONAL / SAFETY_UNVERIFIED）との比較で、**合否・安全の語は使わない**。
CSV の共通の列: `date`, `operator`, `note`（空欄は空のまま。0 で埋めない）。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any, Callable

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

OUT_DIR = ROOT / "simulation" / "results" / "measured"
CAUTION = ("**実機の測定の要約（その個体・その条件に限る）。設定は書き換えていない。5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）。"
           "要約から `HARDWARE_VERIFIED` と書くには、日付・条件・測定器・生データのファイルが要る。**")

COLUMNS: dict[str, list[str]] = {
    "stop_coast": ["date", "operator", "trial", "speed_mm_s", "floor", "stop_cmd_t_s", "stopped_t_s", "coast_mm", "note"],
    "capture_time": ["date", "operator", "trial", "settle_s", "capture_s", "n_frames", "note"],
    "gait_slip": ["date", "operator", "trial", "floor", "commanded_advance_mm", "measured_advance_mm", "note"],
    "tag_detect": ["date", "operator", "trial", "distance_mm", "angle_deg", "light", "detected", "note"],
    "imu": ["date", "operator", "trial", "t_s", "yaw_deg", "ref_yaw_deg", "note"],
    "contact_load": ["date", "operator", "trial", "location", "force_n", "instrument", "note"],
    "current": ["date", "operator", "trial", "state", "supply_v", "current_a", "peak_a", "note"],
    "power_sag": ["date", "operator", "trial", "event", "v_nominal", "v_min", "i_peak_a", "duration_ms", "brownout", "note"],
    "trip": ["date", "operator", "trial", "device", "i_set_a", "trip_a", "trip_ms", "tripped", "note"],
    "thermal": ["date", "operator", "trial", "point", "t_min", "temp_c", "ambient_c", "current_a", "note"],
    "servo_temp": ["date", "operator", "trial", "t_s", "temp_c", "load", "stopped", "note"],
    "stop_time": ["date", "operator", "trial", "trigger", "what", "t_trigger_ms", "t_effect_ms", "esp32_running", "note"],
    "exposure": ["date", "operator", "trial", "light", "floor", "object", "exposure_mode", "gain", "saturated_pct", "diameter_px", "detected", "note"],
    "j1_dead_backlash": ["date", "operator", "trial", "test_id", "direction", "goal_step", "servo_present_step", "output_mm", "note"],
    "j1_push": ["date", "operator", "test_id", "register_torque_limit", "torque_on", "force_n_mean", "force_n_sd", "n_samples", "note"],
    "j1_stopper": ["date", "operator", "speed_dps", "damper", "decel_deg", "torque_limit_reg", "peak_n", "rise_ms", "load_cell_fn_hz", "note"],
    "tof_floor": ["date", "operator", "trial", "test", "surface", "height_mm", "speed_mm_s", "scenario", "distance_mm", "range_status", "classified", "note"],
    "limiter_slip": ["date", "operator", "specimen", "direction", "slip_torque_nm", "temp_c", "cycle", "note"],
    "cable_cycle": ["date", "operator", "trial", "cycles", "continuity_ohm", "wear", "broken", "note"],
    "hood_edge": ["date", "operator", "trial", "edge", "contact_area_mm2", "drop_height_mm", "peak_n", "note"],
}


# ---- 共通 ----------------------------------------------------------------------------------------------------
def load_csv(path: Path, kind: str) -> list[dict[str, str]]:
    rows = list(csv.DictReader(open(path, encoding="utf-8-sig")))
    need = {"stop_coast": ["speed_mm_s", "coast_mm"], "capture_time": ["settle_s", "capture_s"], "gait_slip": ["commanded_advance_mm", "measured_advance_mm"],
            "tag_detect": ["distance_mm", "detected"], "imu": ["t_s", "yaw_deg", "ref_yaw_deg"], "contact_load": ["location", "force_n"],
            "current": ["state", "current_a"], "power_sag": ["event", "v_nominal", "v_min"], "trip": ["device", "i_set_a", "tripped"],
            "thermal": ["point", "t_min", "temp_c"], "servo_temp": ["t_s", "temp_c"], "stop_time": ["trigger", "what", "t_trigger_ms", "t_effect_ms"], "exposure": ["exposure_mode", "saturated_pct"], "j1_dead_backlash": ["test_id", "direction", "goal_step", "servo_present_step"],
            "j1_push": ["register_torque_limit", "torque_on", "force_n_mean"], "j1_stopper": ["damper", "peak_n"],
            "tof_floor": ["test", "surface", "distance_mm"], "limiter_slip": ["specimen", "slip_torque_nm"], "cable_cycle": ["cycles"], "hood_edge": ["edge", "contact_area_mm2"]}[kind]
    if not rows:
        raise ValueError(f"{path}: 行が無い")
    missing = [c for c in need if c not in rows[0]]
    if missing:
        raise ValueError(f"{path}: 必須の列が無い: {missing}（`--template {kind}` で雛形）")
    return rows


def num(r: dict[str, str], k: str) -> float | None:
    v = (r.get(k) or "").strip()
    return float(v) if v else None


def col(rows: list[dict[str, str]], k: str) -> np.ndarray:
    """空欄は捨てる（0 で埋めない）。"""
    return np.array([x for x in (num(r, k) for r in rows) if x is not None and math.isfinite(x)], float)


def stats(a: np.ndarray) -> dict[str, float]:
    if a.size == 0:
        return {"n": 0}
    return {"n": int(a.size), "mean": float(a.mean()), "sd": float(a.std(ddof=1)) if a.size > 1 else 0.0, "p50": float(np.percentile(a, 50)),
            "p95": float(np.percentile(a, 95)), "max": float(a.max()), "min": float(a.min())}


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def cfg() -> dict[str, Any]:
    from serpens.config import load_config
    return load_config()


def fmt(x: float, nd: int = 2) -> str:
    return f"{x:.{nd}f}"


# ---- 種類ごと ----------------------------------------------------------------------------------------------------
def a_stop_coast(rows: list[dict[str, str]], c: dict[str, Any]) -> dict[str, Any]:
    """停止指令から止まるまでの惰行距離。速さに比例するとして傾き（= 実効の停止の時定数 [s]）を最小二乗で。"""
    v, d = col(rows, "speed_mm_s"), col(rows, "coast_mm")
    n = min(v.size, d.size)
    v, d = v[:n], d[:n]
    tau = float((v @ d) / (v @ v)) if n and (v @ v) > 0 else float("nan")        # coast = v τ（原点を通る）
    pred = tau * v
    resid = d - pred
    gait_blend = float(c["gait"]["blend_s"]) if "gait" in c and "blend_s" in c["gait"] else None
    stop_d = float(c["behavior"]["controller"]["stop_distance_mm"])
    coast_ratio = float(c["behavior"]["controller"]["coast_ratio"])
    ref_tau = None if gait_blend is None else gait_blend * coast_ratio
    return {"coast": stats(d), "tau_s": tau, "resid_sd_mm": float(resid.std(ddof=1)) if n > 2 else None, "n_speeds": int(len(set(v.tolist()))),
            "config": {"stop_distance_mm": stop_d, "coast_ratio": coast_ratio, "gait_blend_s": gait_blend, "tau_config_s": ref_tau,
                       "mission_coast_frac": float(c["floor_watch"]["mission"]["coast_frac"])},
            "ratio_to_config": None if not ref_tau or not math.isfinite(tau) else tau / ref_tau,
            "p95_over_stop_distance": float(np.percentile(d, 95)) / stop_d if d.size else None}


def a_capture_time(rows: list[dict[str, str]], c: dict[str, Any]) -> dict[str, Any]:
    """撮影（静止 settle + 5 枚の撮影 capture）の所要時間。`far_trust_s`（「近くに人がいない」を信じる時間）の根拠。"""
    st, cp = col(rows, "settle_s"), col(rows, "capture_s")
    n = min(st.size, cp.size)
    total = st[:n] + cp[:n]
    far = float(c["floor_watch"]["csar"]["far_trust_s"])
    return {"settle": stats(st), "capture": stats(cp), "total": stats(total),
            "config": {"far_trust_s": far, "settle_s": float(c["floor_watch"]["mission"]["settle_s"]), "capture_s": float(c["floor_watch"]["mission"]["capture_s"])},
            "p95_total_over_far_trust": float(np.percentile(total, 95)) / far if n else None,
            "exceeds_far_trust_fraction": float(np.mean(total > far)) if n else None}


def a_gait_slip(rows: list[dict[str, str]], c: dict[str, Any]) -> dict[str, Any]:
    """歩容の滑り = 1 − 実測の前進 / 指令の前進。床ごとに。"""
    by: dict[str, list[float]] = {}
    for r in rows:
        cmd, meas = num(r, "commanded_advance_mm"), num(r, "measured_advance_mm")
        if cmd and meas is not None and cmd > 0:
            by.setdefault((r.get("floor") or "?").strip() or "?", []).append(1.0 - meas / cmd)
    allv = np.array([x for v in by.values() for x in v])
    return {"all": stats(allv), "by_floor": {k: stats(np.array(v)) for k, v in by.items()},
            "config": {"odometry_sigma_along_frac": float(c["localization"]["odometry"]["sigma_along_frac"]), "sim_assumed_slips": [0.05, 0.10, 0.20]}}


def a_tag_detect(rows: list[dict[str, str]], c: dict[str, Any]) -> dict[str, Any]:
    """AprilTag の検出率を距離の帯ごとに（Wilson 95%）。"""
    bins: dict[int, list[int]] = {}
    for r in rows:
        d, hit = num(r, "distance_mm"), (r.get("detected") or "").strip()
        if d is None or hit not in ("0", "1"):
            continue
        b = int(d // 250) * 250
        bins.setdefault(b, []).append(int(hit))
    out = {}
    for b in sorted(bins):
        k, n = sum(bins[b]), len(bins[b])
        lo, hi = wilson(k, n)
        out[f"{b}-{b + 250} mm"] = {"n": n, "rate": k / n, "ci95": [lo, hi]}
    return {"by_distance": out}


def a_imu(rows: list[dict[str, str]], c: dict[str, Any]) -> dict[str, Any]:
    """IMU の yaw と基準の差: 誤差の σ と、ドリフト（時間に対する一次の傾き [rad/s]）。"""
    t = col(rows, "t_s")
    err = np.radians(col(rows, "yaw_deg") - col(rows, "ref_yaw_deg"))
    n = min(t.size, err.size)
    t, err = t[:n], err[:n]
    err = (err + math.pi) % (2 * math.pi) - math.pi
    slope = float(np.polyfit(t, err, 1)[0]) if n > 2 and np.ptp(t) > 0 else float("nan")
    resid = err - (np.polyval(np.polyfit(t, err, 1), t) if n > 2 and np.ptp(t) > 0 else 0.0)
    imu = c["localization"]["imu"]
    return {"error_rad": stats(err), "drift_rad_per_s": slope, "noise_sd_rad": float(resid.std(ddof=1)) if n > 2 else None,
            "config": {"yaw_sigma_rad": float(imu["yaw_sigma_rad"]), "yaw_drift_rad_per_s": float(imu["yaw_drift_rad_per_s"])},
            "drift_ratio_to_config": None if not math.isfinite(slope) else abs(slope) / float(imu["yaw_drift_rad_per_s"])}


def a_contact_load(rows: list[dict[str, str]], c: dict[str, Any]) -> dict[str, Any]:
    """接触の力を場所ごとに。暫定しきい値（safety_thresholds.yaml。PROVISIONAL）との比（合否ではない）。"""
    th = yaml.safe_load((ROOT / "simulation" / "hardware_gaps" / "safety_thresholds.yaml").read_text(encoding="utf-8"))["quasi_static_contact_force_n"]
    base = {k: float(v["baseline"]) for k, v in th.items() if isinstance(v, dict) and "baseline" in v}
    by: dict[str, list[float]] = {}
    for r in rows:
        f = num(r, "force_n")
        if f is not None:
            by.setdefault((r.get("location") or "?").strip(), []).append(f)
    out = {}
    for loc, v in by.items():
        s = stats(np.array(v))
        key = next((k for k in base if k in loc or loc in k), None)
        s["provisional_threshold_n"] = base.get(key) if key else None
        s["max_over_threshold"] = (s["max"] / base[key]) if key else None
        out[loc] = s
    return {"by_location": out, "thresholds": base}


def a_current(rows: list[dict[str, str]], c: dict[str, Any]) -> dict[str, Any]:
    """電流を状態ごとに（待機・歩容・ストール等）。GATE の `real_current` / `power_capacity` の材料。**GATE の状態は書き換えない**。"""
    by: dict[str, list[float]] = {}
    peak: dict[str, float] = {}
    for r in rows:
        i, p = num(r, "current_a"), num(r, "peak_a")
        s = (r.get("state") or "?").strip()
        if i is not None:
            by.setdefault(s, []).append(i)
        if p is not None:
            peak[s] = max(peak.get(s, 0.0), p)
    out = {s: {**stats(np.array(v)), "peak_a": peak.get(s)} for s, v in by.items()}
    gate = c["safety_limits"]["electrical_safety_gate"]
    return {"by_state": out, "gate_status": {k: (v.get("status") if isinstance(v, dict) else v) for k, v in gate.items()} if isinstance(gate, dict) else str(gate),
            "note": "GATE の項目を COMPLETE にするには、日付・条件・測定器の証拠（`serpens/electrical_gate.py`）が要る。この要約は証拠の代わりにならない"}


SERVO_MIN_V = 6.0          # Waveshare Wiki の ST3215（7.4V 版の動作下限の目安）。**ASSUMED**（C044 の資料は未確認。レジスタ 15 の初期値は 4.0 V）


def a_power_sag(rows: list[dict[str, str]], c: dict[str, Any]) -> dict[str, Any]:
    """電圧降下: 事象（全軸同時の起動・ストール等）ごとの最小電圧と降下量、ブラウンアウト（リセット）の有無。"""
    by: dict[str, dict[str, Any]] = {}
    for r in rows:
        ev, vn, vm = (r.get("event") or "?").strip(), num(r, "v_nominal"), num(r, "v_min")
        if vn is None or vm is None:
            continue
        d = by.setdefault(ev, {"drop": [], "vmin": [], "brownout": 0, "n": 0, "r": []})
        d["drop"].append(vn - vm)
        ip = num(r, "i_peak_a")
        if ip is not None and ip > 0:
            d["r"].append((vn - vm) / ip)                              # 電源の全抵抗 R ≈ 降下 / ピーク電流（HG-E1 の判断境界と同じ量）
        d["vmin"].append(vm)
        d["n"] += 1
        d["brownout"] += int((r.get("brownout") or "").strip() in ("1", "true", "True"))
    out = {ev: {"n": d["n"], "drop_v": stats(np.array(d["drop"])), "v_min": stats(np.array(d["vmin"])), "brownouts": d["brownout"],
                "margin_to_servo_min_v": float(min(d["vmin"])) - SERVO_MIN_V,
                "r_total_ohm": stats(np.array(d["r"]))} for ev, d in by.items()}
    return {"by_event": out, "assumed_servo_min_v": SERVO_MIN_V, "note": "動作下限 6.0 V は ASSUMED（資料の確認が要る = OQ-0103）。余裕が負 = 下限を下回った測定がある"}


def a_trip(rows: list[dict[str, str]], c: dict[str, Any]) -> dict[str, Any]:
    """過電流保護: 設定電流ごとに、動作した割合と動作電流・動作時間。**動作しなかった試行が 1 つでもあれば、その設定は保護として働いていない**（数えるだけ。判定はしない）。"""
    by: dict[str, dict[str, list[float]]] = {}
    not_tripped = 0
    for r in rows:
        dev = (r.get("device") or "?").strip()
        d = by.setdefault(dev, {"i_set": [], "trip_a": [], "trip_ms": [], "tripped": []})
        tr = (r.get("tripped") or "").strip() in ("1", "true", "True")
        d["tripped"].append(float(tr))
        not_tripped += int(not tr)
        for k, col_ in (("i_set", "i_set_a"), ("trip_a", "trip_a"), ("trip_ms", "trip_ms")):
            v = num(r, col_)
            if v is not None:
                d[k].append(v)
    out = {dev: {"n": len(d["tripped"]), "tripped_fraction": float(np.mean(d["tripped"])), "i_set": stats(np.array(d["i_set"])),
                 "trip_a": stats(np.array(d["trip_a"])), "trip_ms": stats(np.array(d["trip_ms"]))} for dev, d in by.items()}
    return {"by_device": out, "not_tripped_trials": not_tripped}


def a_thermal(rows: list[dict[str, str]], c: dict[str, Any]) -> dict[str, Any]:
    """配線・コネクタの発熱: 点ごとに、定常の温度上昇 ΔT（最後の 20% の平均 − 周囲）と、指数の時定数（1 − exp(−t/τ) の当てはめ）。"""
    by: dict[str, list[tuple[float, float, float]]] = {}
    for r in rows:
        tm, tc, amb = num(r, "t_min"), num(r, "temp_c"), num(r, "ambient_c")
        if tm is not None and tc is not None:
            by.setdefault((r.get("point") or "?").strip(), []).append((tm, tc, 0.0 if amb is None else amb))
    out = {}
    for pt, v in by.items():
        v.sort()
        tm = np.array([x[0] for x in v])
        dT = np.array([x[1] - x[2] for x in v])
        tail = dT[int(0.8 * len(dT)):] if len(dT) >= 5 else dT
        dT_inf = float(tail.mean())
        tau = float("nan")
        if len(dT) >= 5 and dT_inf > 0.5:
            f = np.clip(dT / dT_inf, 1e-6, 0.999)
            ok = f < 0.95
            if ok.sum() >= 3:
                y = -np.log(1.0 - f[ok])
                tau = float(1.0 / np.polyfit(tm[ok], y, 1)[0]) if np.polyfit(tm[ok], y, 1)[0] > 0 else float("nan")
        out[pt] = {"n": len(v), "delta_t_steady_c": dT_inf, "tau_min": tau, "max_temp_c": float(max(x[1] for x in v))}
    return {"by_point": out}


def a_servo_temp(rows: list[dict[str, str]], c: dict[str, Any]) -> dict[str, Any]:
    """サーボの温度上昇: 最高温度、停止した温度（`stopped` = 1 の最初の行）、config の停止温度との比較。"""
    t, temp = col(rows, "t_s"), col(rows, "temp_c")
    n = min(t.size, temp.size)
    t, temp = t[:n], temp[:n]
    stop_c = None
    for r in rows:
        if (r.get("stopped") or "").strip() in ("1", "true", "True") and num(r, "temp_c") is not None:
            stop_c = num(r, "temp_c")
            break
    slope = float(np.polyfit(t, temp, 1)[0]) if n > 2 and np.ptp(t) > 0 else float("nan")
    cfgv = {"link_faults_temp_limit_c": float(c["link"]["faults"]["temp_limit_c"]), "servo_temperature_limit_c": float(c["servo"]["temperature_limit_c"]),
            "safety_limits_servo_temp_stop_c": float(c["safety_limits"]["servo_temp_stop_c"])}
    return {"max_temp_c": float(temp.max()) if n else None, "rise_c_per_min": slope * 60.0 if math.isfinite(slope) else None, "stopped_at_c": stop_c,
            "config": cfgv, "stopped_before_limit": None if stop_c is None else bool(stop_c <= cfgv["link_faults_temp_limit_c"] + 1.0),
            "note": "停止温度が config の 60 ℃（機体側の上限）の近傍で止まったかを見る。**GATE servo_temperature の状態は書き換えない**"}


def a_stop_time(rows: list[dict[str, str]], c: dict[str, Any]) -> dict[str, Any]:
    """停止までの時間 = t_effect − t_trigger（ms）。きっかけ（estop / comm_loss / contact / esp32_hang）と、何が止まったか（supply_cut = サーボ電源の遮断 / motion_stop = 動きの停止）ごとに。
    `esp32_running` = 0 の行は ESP32 を止めた（ハングさせた）状態の試験 = **独立した遮断**の確認。"""
    by: dict[str, dict[str, Any]] = {}
    for r in rows:
        a, b = num(r, "t_trigger_ms"), num(r, "t_effect_ms")
        if a is None or b is None:
            continue
        key = f"{(r.get('trigger') or '?').strip()}/{(r.get('what') or '?').strip()}"
        d = by.setdefault(key, {"dt": [], "esp32_off": 0})
        d["dt"].append(b - a)
        d["esp32_off"] += int((r.get("esp32_running") or "").strip() in ("0", "false", "False"))
    cfgv = {"heartbeat_timeout_ms": float(c["link"]["heartbeat_timeout_ms"]), "drive_ttl_ms": float(c["link"]["drive_ttl_ms"]),
            "contact_release_ms": float(c["safety_limits"]["contact_release_ms"])}
    out = {k: {"n": len(d["dt"]), "ms": stats(np.array(d["dt"])), "trials_with_esp32_stopped": d["esp32_off"]} for k, d in by.items()}
    return {"by_trigger": out, "config": cfgv,
            "note": "`contact_release_ms` 20 は要求そのものが未確定（`contact_release_requirements.md`）。この要約は比較の材料で、合否ではない"}


def a_exposure(rows: list[dict[str, str]], c: dict[str, Any]) -> dict[str, Any]:
    """実機カメラの露出と飽和: 露出の設定（`exposure_mode` + `gain`）ごとに、飽和画素の割合の分布と、検出の成否（Wilson 95%）。
    `over_pct` の割合 = 飽和率がその値を超えたフレームの割合（**「撮り直し」の保護の要否の材料**。しきい値 50% は ENTRY-E-0018 (4) の案で、config には無い）。"""
    over_pct = 50.0
    by: dict[str, dict[str, list]] = {}
    for r in rows:
        sat = num(r, "saturated_pct")
        if sat is None:
            continue
        g = (r.get("gain") or "").strip()
        key = f"{(r.get('exposure_mode') or '?').strip()}" + (f"/gain={g}" if g else "")
        d = by.setdefault(key, {"sat": [], "hit": []})
        d["sat"].append(sat)
        hit = (r.get("detected") or "").strip()
        if hit in ("0", "1"):
            d["hit"].append(int(hit))
    out = {}
    for k, d in sorted(by.items()):
        s = np.array(d["sat"])
        e = {"n": int(s.size), "saturated_pct": stats(s), "frac_over_pct": float(np.mean(s > over_pct))}
        if d["hit"]:
            kk, n = sum(d["hit"]), len(d["hit"])
            lo, hi = wilson(kk, n)
            e["detect"] = {"n": n, "rate": kk / n, "ci95": [lo, hi]}
        out[k] = e
    return {"by_exposure": out, "over_pct": over_pct,
            "note": "合成（HG-H2）では gain 2.5 で通常光フレームの 71〜87% が飽和し検出が欠落した。**実機の自動露出がこれを抑えるか**を見る材料。合否ではない"}


# ---- J1（頭ピッチ）の実測（HT-002 / 003 / 004 / 012。LB-E-059）-----------------------------------------------
J1_ARM_MM = 44.0                                            # J1 軸から横スキッドの前端 / ダイヤルゲージまでの腕 [mm]（j1_head_pitch.md §1。ASSUMED。実測で置き換える）
J1_STEP_DEG = 360.0 / 4096.0


def _j1_opt():
    """`simulation/hardware_gaps/HG-H1_actuator/j1_range_opt.py` を読む（範囲の候補・窓の端の速さの式を再利用。式を二重に持たない）。"""
    import importlib.util
    p = ROOT / "simulation" / "hardware_gaps" / "HG-H1_actuator" / "j1_range_opt.py"
    spec = importlib.util.spec_from_file_location("j1_range_opt", p)
    m = importlib.util.module_from_spec(spec)
    sys.modules["j1_range_opt"] = m
    spec.loader.exec_module(m)
    return m


def a_j1_dead_backlash(rows: list[dict[str, str]], c: dict[str, Any]) -> dict[str, Any]:
    """J1-1（不感帯 D）と J1-2（バックラッシ B）→ 範囲の端の「減速できる速さ」を、**測った D・B で**再計算する（prior の置き換え）。
    D = J1-1 の |読み値 − 目標| の最大（ステップ。量子化を含む上界）。B = J1-2 の、同じ目標へ上から / 下から着いたときの**出力側の**差（ダイヤルゲージ、腕 44 mm）。
    エンコーダがモーター側なら（サーボの読み値の差 < 出力の差の半分）B は位置の誤差に足される。出力側なら B は読み値に出ているので D に含まれる。"""
    o = _j1_opt()
    errs: dict[str, list[float]] = {}
    for r in rows:
        if (r.get("test_id") or "").strip().upper() != "J1-1":
            continue
        g, p = num(r, "goal_step"), num(r, "servo_present_step")
        if g is None or p is None:
            continue
        errs.setdefault((r.get("direction") or "?").strip().lower(), []).append(abs(p - g))
    d_by_dir = {k: float(max(v)) for k, v in errs.items()}
    d_hat = max(d_by_dir.values()) if d_by_dir else None
    by_goal: dict[float, dict[str, list[tuple[float, float]]]] = {}
    for r in rows:
        if (r.get("test_id") or "").strip().upper() != "J1-2":
            continue
        g, s, out = num(r, "goal_step"), num(r, "servo_present_step"), num(r, "output_mm")
        if g is None or s is None or out is None:
            continue
        by_goal.setdefault(g, {}).setdefault((r.get("direction") or "?").strip().lower(), []).append((s, out))
    b_out, b_servo = [], []
    for g, dd in by_goal.items():
        if "up" in dd and "down" in dd:
            mo = lambda k, i: float(np.mean([x[i] for x in dd[k]]))
            b_out.append(abs(mo("up", 1) - mo("down", 1)) / J1_ARM_MM * 180.0 / math.pi / J1_STEP_DEG)           # mm → rad → ° → ステップ
            b_servo.append(abs(mo("up", 0) - mo("down", 0)))
    b_hat = float(np.mean(b_out)) if b_out else None
    motor_side = None if not b_out else bool(np.mean(b_servo) < 0.5 * np.mean(b_out))
    res: dict[str, Any] = {"dead_band_steps": d_hat, "dead_band_by_direction": d_by_dir, "backlash_steps": b_hat, "backlash_n_goals": len(b_out),
                           "encoder_on_motor_side": motor_side, "arm_mm": J1_ARM_MM}
    if d_hat is None:
        res["note"] = "J1-1 の行が無い（D が決まらない）"
        return res
    m_steps = 0.5 + d_hat + (b_hat if (motor_side and b_hat is not None) else 0.0)
    m_deg = m_steps * J1_STEP_DEG
    cap = {}
    for side, cands in (("hi", o.HI_CANDIDATES), ("lo", o.LO_CANDIDATES)):
        for e in cands:
            w = o.speed_cap_dps(e, side, m_deg, o.A_DECEL_MIN)
            cap[f"{side}:{e:g}"] = {"cap_dps_at_conservative_decel": w, "feasible": bool(w >= o.V_NEAR_MIN)}
    res.update({"position_error_steps": m_steps, "position_error_deg": m_deg, "decel_conservative_dps2": o.A_DECEL_MIN, "v_near_min_dps": o.V_NEAR_MIN, "edges": cap,
                "note": "prior（D 0〜4、B 0〜8 ステップ、エンコーダがモーター側の確率 0.5）を測った値に置き換えた再計算。減速度は保守側（1000 °/s²。最大 2233）。**判定ではなく、範囲を決め直す材料**"})
    return res


def a_j1_push(rows: list[dict[str, str]], c: dict[str, Any]) -> dict[str, Any]:
    """J1-3: 上限レジスタごとの押す力（サーボ分 = トルク ON − OFF）。暫定しきい値 5.7 N（PROVISIONAL）の半分 2.8 N と頭の重さ（約 1 N）との比を添える（合否ではない）。"""
    off = [num(r, "force_n_mean") for r in rows if (r.get("torque_on") or "").strip() in ("0", "false", "False") and num(r, "force_n_mean") is not None]
    base = float(np.mean(off)) if off else None
    by: dict[str, list[float]] = {}
    for r in rows:
        if (r.get("torque_on") or "").strip() not in ("1", "true", "True"):
            continue
        f = num(r, "force_n_mean")
        if f is None:
            continue
        by.setdefault((r.get("register_torque_limit") or "?").strip(), []).append(f)
    out = {}
    for k, v in sorted(by.items(), key=lambda kv: float(kv[0]) if kv[0].replace(".", "", 1).isdigit() else 1e9):
        f = float(np.mean(v)) - (base or 0.0)
        out[k] = {"n": len(v), "servo_force_n": f, "ratio_to_2p8_n": f / 2.8, "ratio_to_head_weight_1n": f / 1.0}
    return {"torque_off_baseline_n": base, "by_register": out, "provisional_threshold_n": 5.7,
            "note": "5.7 N は PROVISIONAL / SAFETY_UNVERIFIED。OFF の基準が無いと servo_force は ON の値そのまま（過大になりうる）"}


def a_j1_stopper(rows: list[dict[str, str]], c: dict[str, Any]) -> dict[str, Any]:
    """ストッパーの衝撃: (緩衝材, 減速開始角) ごとのピーク荷重 N。ピン（鋼 φ3、曲げ）の許容荷重の prior（σ 200 MPa の下限）との比を添える（合否ではない）。"""
    o = _j1_opt()
    by: dict[str, list[float]] = {}
    for r in rows:
        pk = num(r, "peak_n")
        if pk is None:
            continue
        by.setdefault(f"{(r.get('damper') or '?').strip()}/decel={(r.get('decel_deg') or '?').strip()}", []).append(pk)
    cap = o.pin_capacity("steel", 200.0)
    out = {k: {"n": len(v), "peak_n": stats(np.array(v)), "max_over_steel_pin_capacity": float(max(v) / cap)} for k, v in sorted(by.items())}
    return {"by_setting": out, "steel_pin_capacity_n_at_200mpa": cap,
            "note": "ピンの許容荷重は prior（σ 200〜350 MPa の下限、曲げ・片持ち）。ロードセルの固有振動数 `load_cell_fn_hz` がピーク幅（`rise_ms`）より低いとピークを過小に読む。**合否ではない**"}


# ---- ToF の崖判定（HT-005 / T6）、リミッターの滑りトルク（HG-S3 L1・L2）、ケーブルの耐久（HG-C1 C4）----------------
LIMITER_WINDOW_NM = (0.7, 1.0)               # config / HG-S3 の窓（暫定。PROVISIONAL）
LIMITER_TOL_CLOSE = 0.40                      # 個体差がこれを超えると窓が閉じる（HG-S3 の拡張 LB-E-042、dyn 2 の prior。**prior**。実測の個体差で置き換える）


def a_tof_floor(rows: list[dict[str, str]], c: dict[str, Any]) -> dict[str, Any]:
    """T6-1 / T6-2 / T6-4: 面（床・縁・黒・鏡面…）ごとに、有効な読みの割合・距離の分布と、床 / 崖の判定の誤り。
    **崖を床と判定した（見逃し = 危険側）**を別に数える。`scenario` = floor / cliff（実際の状態）、`classified` = floor / cliff / invalid（読みからの判定）。"""
    by: dict[str, dict[str, Any]] = {}
    for r in rows:
        key = f"{(r.get('test') or '?').strip()}/{(r.get('surface') or '?').strip()}"
        d = by.setdefault(key, {"dist": [], "n": 0, "valid": 0, "floor_n": 0, "floor_as_cliff": 0, "cliff_n": 0, "cliff_as_floor": 0})
        d["n"] += 1
        st = (r.get("range_status") or "").strip()
        dist = num(r, "distance_mm")
        if st in ("0", "") and dist is not None and dist > 0:
            d["valid"] += 1
            d["dist"].append(dist)
        sc, cl = (r.get("scenario") or "").strip().lower(), (r.get("classified") or "").strip().lower()
        if sc == "floor" and cl:
            d["floor_n"] += 1
            d["floor_as_cliff"] += int(cl == "cliff")
        if sc == "cliff" and cl:
            d["cliff_n"] += 1
            d["cliff_as_floor"] += int(cl == "floor")
    out = {}
    for k, d in sorted(by.items()):
        e: dict[str, Any] = {"n": d["n"], "valid_fraction": d["valid"] / d["n"], "distance_mm": stats(np.array(d["dist"]))}
        if d["floor_n"]:
            e["false_cliff_rate"] = d["floor_as_cliff"] / d["floor_n"]
        if d["cliff_n"]:
            lo, hi = wilson(d["cliff_as_floor"], d["cliff_n"])
            e["cliff_missed"] = {"n": d["cliff_n"], "k": d["cliff_as_floor"], "rate": d["cliff_as_floor"] / d["cliff_n"], "ci95": [lo, hi]}
        out[k] = e
    return {"by_surface": out, "note": "崖の見逃し（危険側）の率。見逃しが 0 でも n が小さければ上限は 0 ではない（ci95 の上側を見る）。**合否ではない**。3V3 駆動では Wi-Fi 送信・撮影・LED 100 %・ToF のピークを同時にしない（K-0002）"}


def a_limiter_slip(rows: list[dict[str, str]], c: dict[str, Any]) -> dict[str, Any]:
    """L1 / L2: 滑りトルクの値と個体差。公称（平均）・標準偏差・個体差（最大の偏差 / 平均）と、窓 0.7〜1.0 N·m に入る個体の割合。
    温度・繰り返しごとに分ける。個体差が `LIMITER_TOL_CLOSE`（prior）を超えると窓が閉じる、という HG-S3 の読みとの比較（**合否ではない**）。"""
    by: dict[str, list[float]] = {}
    for r in rows:
        x = num(r, "slip_torque_nm")
        if x is None:
            continue
        key = f"{(r.get('direction') or 'any').strip()}/T={(r.get('temp_c') or '?').strip()}/cycle={(r.get('cycle') or '0').strip()}"
        by.setdefault(key, []).append(x)
    out = {}
    for k, v in sorted(by.items()):
        a = np.array(v)
        mean = float(a.mean())
        dev = float(np.max(np.abs(a - mean)) / mean) if mean > 0 else float("nan")
        out[k] = {"n": int(a.size), "mean_nm": mean, "sd_nm": float(a.std(ddof=1)) if a.size > 1 else 0.0, "max_deviation_frac": dev,
                  "in_window_fraction": float(np.mean((a >= LIMITER_WINDOW_NM[0]) & (a <= LIMITER_WINDOW_NM[1]))),
                  "deviation_over_window_closing_prior": bool(dev > LIMITER_TOL_CLOSE)}
    return {"by_condition": out, "window_nm": list(LIMITER_WINDOW_NM), "window_closing_tolerance_prior": LIMITER_TOL_CLOSE,
            "note": "窓は PROVISIONAL。n が小さい（試験片 10 個未満）と個体差を過小に見る。歯車の破断トルク（L3）は別（資料 / 破壊試験 = D）"}


def a_cable_cycle(rows: list[dict[str, str]], c: dict[str, Any]) -> dict[str, Any]:
    """C4: ±50° 往復の耐久。最大の回数、導通（抵抗）の変化、被覆の擦れ（wear 0/1/2）・断線の最初の回数。`verified_with_cable` の材料（**User 承認と記録が要る**）。"""
    cyc, ohm, wear_first, break_first = [], [], None, None
    for r in sorted(rows, key=lambda r: num(r, "cycles") or 0.0):
        n = num(r, "cycles")
        if n is None:
            continue
        cyc.append(n)
        o = num(r, "continuity_ohm")
        if o is not None:
            ohm.append(o)
        w = (r.get("wear") or "").strip()
        if w in ("1", "2") and wear_first is None:
            wear_first = n
        if (r.get("broken") or "").strip() in ("1", "true", "True") and break_first is None:
            break_first = n
    return {"max_cycles": max(cyc) if cyc else None, "first_wear_at_cycles": wear_first, "first_break_at_cycles": break_first,
            "continuity_change_frac": (max(ohm) - ohm[0]) / ohm[0] if len(ohm) > 1 and ohm[0] > 0 else None,
            "target_cycles": 1000,
            "note": "目標は 1000 往復（0.5 Hz で約 33 分）。擦れも断線も導通の変化も無いことが `joint_limit_policy.verified_with_cable` の材料。**この取り込みは合否を出さない・config を書き換えない**"}


def a_hood_edge(rows: list[dict[str, str]], c: dict[str, Any]) -> dict[str, Any]:
    """HT-006: フード昇降の縁の接触面積（感圧紙の読み）と落下ピーク。縁（edge）ごとに、接触面積・ピーク荷重の分布と、ピーク ÷ 面積 = 平均圧力 [kPa]（N / mm² = MPa なので ×1000）。
    **しきい値は置かない**（力・圧力の安全のしきい値は PROVISIONAL / SAFETY_UNVERIFIED。User が決める）。合否ではない。"""
    by: dict[str, dict[str, list[float]]] = {}
    for r in rows:
        key = (r.get("edge") or "?").strip()
        d = by.setdefault(key, {"area": [], "peak": [], "p": []})
        a, p = num(r, "contact_area_mm2"), num(r, "peak_n")
        if a is not None:
            d["area"].append(a)
        if p is not None:
            d["peak"].append(p)
        if a is not None and p is not None and a > 0:
            d["p"].append(p / a * 1000.0)
    out = {k: {"contact_area_mm2": stats(np.array(d["area"])), "peak_n": stats(np.array(d["peak"])), "mean_pressure_kpa": stats(np.array(d["p"]))} for k, d in sorted(by.items())}
    return {"by_edge": out, "note": "圧力のしきい値は無い（PROVISIONAL）。面積が小さく荷重が大きいほど平均圧力が大きい。感圧紙の読みは濃度から圧力への換算の誤差が大きい（フィルムの仕様の範囲）"}


ANALYZERS: dict[str, Callable[[list[dict[str, str]], dict[str, Any]], dict[str, Any]]] = {
    "stop_coast": a_stop_coast, "capture_time": a_capture_time, "gait_slip": a_gait_slip, "tag_detect": a_tag_detect,
    "imu": a_imu, "contact_load": a_contact_load, "current": a_current,
    "power_sag": a_power_sag, "trip": a_trip, "thermal": a_thermal, "servo_temp": a_servo_temp, "stop_time": a_stop_time, "exposure": a_exposure,
    "j1_dead_backlash": a_j1_dead_backlash, "j1_push": a_j1_push, "j1_stopper": a_j1_stopper,
    "tof_floor": a_tof_floor, "limiter_slip": a_limiter_slip, "cable_cycle": a_cable_cycle, "hood_edge": a_hood_edge,
}


def render(kind: str, path: Path, res: dict[str, Any]) -> str:
    L = [f"# 測定の取り込み: {kind}（{path.name}）\n", CAUTION + "\n", "```json", json.dumps(res, ensure_ascii=False, indent=1, default=float), "```\n"]
    if kind == "stop_coast":
        L.append(f"- 実効の停止の時定数 τ = {fmt(res['tau_s'], 3)} s（coast = 速さ × τ。config の gait.blend_s × coast_ratio = {res['config']['tau_config_s']} s と比べる）。"
                 f"p95 の惰行 / 停止距離 {res['config']['stop_distance_mm']:g} mm = {fmt(res['p95_over_stop_distance'] or float('nan'), 2)}。")
    if kind == "capture_time":
        L.append(f"- 撮影の所要時間（静止 + 撮影）の p95 = {fmt(res['total'].get('p95', float('nan')))} s。`far_trust_s` = {res['config']['far_trust_s']:g} s を超える割合 = {fmt((res['exceeds_far_trust_fraction'] or 0) * 100, 0)}%。")
    return "\n".join(L) + "\n"


def ingest(kind: str, path: Path, out_dir: Path = OUT_DIR) -> tuple[dict[str, Any], Path]:
    rows = load_csv(path, kind)
    res = ANALYZERS[kind](rows, cfg())
    res["source_file"] = str(path).replace("\\", "/")
    res["n_rows"] = len(rows)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f"{kind}_{path.stem}"
    (out_dir / f"{stem}.json").write_text(json.dumps(res, ensure_ascii=False, indent=1, default=float), encoding="utf-8")
    md = out_dir / f"{stem}.md"
    md.write_text(render(kind, path, res), encoding="utf-8", newline="\n")
    return res, md


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("kind", nargs="?", choices=sorted(ANALYZERS))
    ap.add_argument("csv", nargs="?", type=Path)
    ap.add_argument("--template", metavar="KIND", choices=sorted(ANALYZERS))
    ap.add_argument("--out-dir", type=Path, default=OUT_DIR)
    a = ap.parse_args(argv)
    if a.template:
        print(",".join(COLUMNS[a.template]))
        return 0
    if not a.kind or not a.csv:
        ap.error("kind と csv が要る")
    res, md = ingest(a.kind, a.csv, a.out_dir)
    print(f"wrote {md}（{res['n_rows']} 行）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
