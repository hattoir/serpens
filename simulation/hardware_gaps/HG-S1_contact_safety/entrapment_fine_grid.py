"""囲い込み検査の格子を細かくした追試（LB-E-084、SE-E13 の打ち切り規則）。

`entrapment_check.py` の格子 GRID_DEG（9 値）に 12.5° 刻みの値を足し、同じ `sweep` で走査する。
判断境界の指標 = (a) 袋小路が最初に現れる頭ヨー、(b) 首の太い側 88.5 mm との差の最小 [mm]（±45° と ±60°）。
元の格子の結果（v2）と比べ、境界が 5% 未満しか動かなければ「これ以上細かくしない」（dropped、指標の値つき）。

GEOMETRY_SIM（平面・剛体・CAD_CONCEPT の寸法）。実物で未確認（HARDWARE_VERIFIED = 0）。
実行: python entrapment_fine_grid.py（4 並列。形の数が増えるので数十分かかる）
"""
from __future__ import annotations

import itertools
import json
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import entrapment_check as ec  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
FINE_STEP = 12.5
OUT_JSON = ROOT / "simulation" / "results" / "entrapment_fine_grid_2026-10-02.json"
V2_JSON = ROOT / "simulation" / "results" / "entrapment_check_2026-10-01.json"


def fine_values() -> tuple[float, ...]:
    steps = [i * FINE_STEP for i in range(-4, 5)]
    return tuple(sorted(set(ec.GRID_DEG) | set(steps)))


def fine_configs() -> list[tuple[float, ...]]:
    vals = fine_values()
    return [tuple(float(x) for x in c) for c in itertools.product(vals, repeat=4)
            if ec.max_contiguous_sum(c) <= ec.BODY_SUM_MAX_DEG + 1e-9]


def sweep_signed_fine(head_yaw_deg: float) -> dict:
    return ec.sweep(head_yaw_deg, fine_configs())


def summarize(per_phi: dict) -> dict:
    out = {}
    for phi, pks in per_phi.items():
        if not pks:
            out[phi] = None
            continue
        mouth = min(p["mouth_mm"] for p in pks)
        out[phi] = {"pockets": len(pks), "min_mouth_mm": mouth,
                    "margin_to_neck_mm": mouth - ec.PARTS_MM["首"][1],
                    "max_d_in_mm": max(p["d_in_mm"] for p in pks)}
    return out


def main() -> None:
    jobs = [s for phi in ec.HEAD_YAW_DEG for s in ((phi,) if phi == 0 else (phi, -phi))]
    with ProcessPoolExecutor(max_workers=4) as ex:
        done = dict(zip(jobs, ex.map(sweep_signed_fine, jobs)))
    per_phi: dict[float, list] = {}
    n_cfg = {}
    for phi in ec.HEAD_YAW_DEG:
        signs = (phi,) if phi == 0 else (phi, -phi)
        per_phi[phi] = [p for s in signs for p in done[s]["pockets"]]
        n_cfg[phi] = sum(done[s]["n"] for s in signs)
    fine = summarize(per_phi)
    v2 = json.loads(V2_JSON.read_text(encoding="utf-8"))["summary"]
    rows, shifts = [], []
    for phi in ec.HEAD_YAW_DEG:
        a, b = v2.get(f"{phi}"), fine[phi]
        rows.append({"phi": phi, "forms": n_cfg[phi], "v2": a, "fine": b})
        if a and b:
            shifts.append((phi, (b["margin_to_neck_mm"] - a["margin_to_neck_mm"])))
    first_v2 = next((float(k) for k, v in v2.items() if v), None)
    first_fine = next((phi for phi in ec.HEAD_YAW_DEG if fine[phi]), None)
    OUT_JSON.write_text(json.dumps({"fine_values_deg": fine_values(), "rows": rows, "first_pocket_v2": first_v2,
                                    "first_pocket_fine": first_fine, "margin_shift_mm": shifts},
                                   ensure_ascii=False, default=float, indent=1), encoding="utf-8")
    print("values:", fine_values())
    print("first pocket v2 / fine:", first_v2, first_fine)
    for r in rows:
        print(r["phi"], r["forms"], "v2:", r["v2"] and round(r["v2"]["margin_to_neck_mm"], 1),
              "fine:", r["fine"] and round(r["fine"]["margin_to_neck_mm"], 1),
              "pockets", r["v2"] and r["v2"]["pockets"], "->", r["fine"] and r["fine"]["pockets"])


if __name__ == "__main__":
    main()
