"""SE-E13 / LB-E-039: HG-H2 の 1 変数ずつの掃引（OAT）の範囲を、nominal から最も遠い値のさらに 2 倍（正の量は 0 を下回らない）まで広げ、
各変数の「合格する範囲」（基準を満たす値）の端が動くかを見る。拡張した値は現実的でないものを含む（境界を見つけるための探索）。SYNTHETIC_VISION_SIM（合成画像）。実画像の性能ではない。
    python expand_oat_range.py [--workers 4]
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run as R  # noqa: E402

EXTRA = {                                     # nominal からさらに遠い値（現在の掃引の最も遠い値の約 2 倍の距離）
    "fov_h_deg": [30.0, 120.0], "width_px": [320, 3200], "cam_height_mm": [10.0, 90.0], "cam_pitch_deg": [5.0, 75.0],
    "distortion_k1": [-0.7], "focus_mm": [30.0, 20000.0], "aperture_mm": [0.25, 3.0], "motion_blur_px": [20.0],
    "exposure_gain": [0.25, 5.0], "read_noise": [20.0], "floor_albedo": [0.1, 0.95], "raking_led_height_mm": [50.0],
    "shadow_factor": [0.05, 0.95], "line_width_mm": [0.5, 12.0], "ambient_lux": [400.0, 1000.0],
}
OUT = R.HERE / "results" / "expand_oat_range_2026-10-02.json"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=4)
    a = ap.parse_args()
    nom = dict(R.A["nominal"])
    conds = [(f"{k}={v}", {**nom, k: v}) for k, vs in EXTRA.items() for v in vs]
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        rows = list(ex.map(R.evaluate_condition, conds))
    base = {}
    with (R.HERE / "results" / "oat.csv").open(encoding="utf-8") as fp:
        for r in csv.DictReader(fp):
            base[r["tag"]] = r
    out = {"extra": [], "pass_range": {}}
    for r in rows:
        out["extra"].append({"tag": r["tag"], "ok": bool(R.ok(r)), "recall_patrol": r["recall_patrol"], "recall_inspect_online": r["recall_inspect_online"],
                             "false_alarm_patrol": r["false_alarm_patrol"], "missed": r["missed"]})
    ok_by_var: dict[str, list[tuple[float, bool]]] = {}
    for k, vals in R.SW["oat"].items():
        for v in vals:
            rr = base.get(f"{k}={v}")
            if rr is not None:
                ok_by_var.setdefault(k, []).append((float(v), float(rr["recall_patrol"]) >= R.A["criteria"]["min_recall_patrol"] and float(rr["recall_inspect_online"]) >= R.A["criteria"]["min_recall_inspect"] and float(rr["false_alarm_patrol"]) <= R.A["criteria"]["max_false_alarm"] and float(rr["false_alarm_inspect"]) <= R.A["criteria"]["max_false_alarm"]))
        ok_by_var.setdefault(k, []).append((float(nom[k]), True))
    for e in out["extra"]:
        k, v = e["tag"].split("=")
        ok_by_var.setdefault(k, []).append((float(v), e["ok"]))
    for k, lst in ok_by_var.items():
        lst.sort()
        out["pass_range"][k] = {"nominal": nom[k], "pass": [v for v, ok in lst if ok], "fail": [v for v, ok in lst if not ok]}
    OUT.write_text(json.dumps(out, indent=1, default=float), encoding="utf-8")
    for k, v in out["pass_range"].items():
        print(k, "nominal", v["nominal"], "pass", v["pass"], "fail", v["fail"])


if __name__ == "__main__":
    main()
