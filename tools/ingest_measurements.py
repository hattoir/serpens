"""実機の測定記録（CSV）を 1 コマンドで解析する取り込み口（LB-E-013 / SE-E6）。**実測が来たときの受け口。今は合成データでの試験だけ（実測 0 件）。**

    python tools/ingest_measurements.py stop_coast   <csv>
    python tools/ingest_measurements.py capture_time <csv>
    python tools/ingest_measurements.py gait_slip    <csv>
    python tools/ingest_measurements.py tag_detect   <csv>
    python tools/ingest_measurements.py imu          <csv>
    python tools/ingest_measurements.py contact_load <csv>
    python tools/ingest_measurements.py current      <csv>
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
}


# ---- 共通 ----------------------------------------------------------------------------------------------------
def load_csv(path: Path, kind: str) -> list[dict[str, str]]:
    rows = list(csv.DictReader(open(path, encoding="utf-8-sig")))
    need = {"stop_coast": ["speed_mm_s", "coast_mm"], "capture_time": ["settle_s", "capture_s"], "gait_slip": ["commanded_advance_mm", "measured_advance_mm"],
            "tag_detect": ["distance_mm", "detected"], "imu": ["t_s", "yaw_deg", "ref_yaw_deg"], "contact_load": ["location", "force_n"],
            "current": ["state", "current_a"]}[kind]
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


ANALYZERS: dict[str, Callable[[list[dict[str, str]], dict[str, Any]], dict[str, Any]]] = {
    "stop_coast": a_stop_coast, "capture_time": a_capture_time, "gait_slip": a_gait_slip, "tag_detect": a_tag_detect,
    "imu": a_imu, "contact_load": a_contact_load, "current": a_current,
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
