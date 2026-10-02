"""SE-E13 / LB-E-041: HG-S2 の角度合計の上限の掃引を 145〜180° から 145〜290°（約 2 倍）へ広げ、判断（145° を維持）が動くかを見る。
同じ歩容の集合（`locomotion()`）を 1 回走らせ、上限ごとに集計し直す。指標 = 直進の最速の変化 [mm/s] と最小旋回半径の短縮の割合。
5% 未満しか動かなければ「これ以上拡張しない」（dropped、指標の値つき）。YAW_SUM_TRADEOFF_SIM（平面摩擦モデル）。実測ではない。
    python expand_limit_range.py
"""
from __future__ import annotations

import functools
import json
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run as R  # noqa: E402

LIMITS = (145.0, 180.0, 215.0, 250.0, 290.0)
OUT = R.HERE / "results" / "expand_limit_range_2026-10-02.json"


def main() -> None:
    R.ProcessPoolExecutor = functools.partial(ProcessPoolExecutor, max_workers=4)      # 12 並列はメモリが足りない（HG-H2 で経験）
    rows = R.locomotion()
    max_sum = max(r["yaw_sum"] for r in rows)
    res = {lim: R.summarize_loco(rows, lim) for lim in LIMITS}
    base = {(x["config"], x["law"], x["ratio"]): x for x in res[145.0]}
    out = {"max_yaw_sum_of_any_gait_deg": max_sum, "limits": {}}
    for lim in LIMITS[1:]:
        dv, frac, dr = 0.0, 0.0, 0.0
        for x in res[lim]:
            b = base[(x["config"], x["law"], x["ratio"])]
            dv = max(dv, abs(x["speed_max"] - b["speed_max"]))
            if math.isfinite(x["turn_radius_min_mm"]) and math.isfinite(b["turn_radius_min_mm"]) and b["turn_radius_min_mm"] > 0:
                dr = max(dr, b["turn_radius_min_mm"] - x["turn_radius_min_mm"])
                frac = max(frac, (b["turn_radius_min_mm"] - x["turn_radius_min_mm"]) / b["turn_radius_min_mm"])
        out["limits"][f"{lim:g}"] = {"max_speed_change_mm_s": dv, "max_turn_radius_reduction_mm": dr, "max_turn_radius_reduction_frac": frac}
    OUT.write_text(json.dumps(out, indent=1), encoding="utf-8")
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
