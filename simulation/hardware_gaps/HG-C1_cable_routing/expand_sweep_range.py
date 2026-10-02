"""SE-E13 / LB-E-043: HG-C1 のケーブルの掃引範囲を 2 倍にして、成立する経路（軸からの横のずれ e・アンカー距離 a・高さ dz）の境界が動くかを見る。
指標 = 曲げ・空間の両方を満たす条件が存在する最大の e [mm]、その e での最小のたわみ。5% 未満しか動かなければ「これ以上拡張しない」（dropped）。
CABLE_GEOMETRY_SIM（幾何のみ）。実ケーブルの剛性・摩擦・ねじれは無い。実測ではない。
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run as R  # noqa: E402

OUT = R.HERE / "results" / "expand_sweep_range_2026-10-02.json"


def boundary(anchor, asym, lat, dz) -> dict:
    lim = float(R.A["joint"]["mechanical_limit_deg"])
    rows = [R.evaluate(a, a + d, e, z, float(R.A["cable"]["margin_mm"]), lim) for a in anchor for d in asym for e in lat for z in dz]
    ok = [r for r in rows if r["ok_bend"] and r["ok_space"]]
    e_max = max((r["e"] for r in ok), default=None)
    best = min((r for r in ok), key=lambda r: r["bow_max_mm"], default=None)
    return {"n": len(rows), "n_ok": len(ok), "max_feasible_e_mm": e_max, "best_bow_mm": None if best is None else best["bow_max_mm"],
            "feasible_e": sorted({r["e"] for r in ok}), "feasible_a": sorted({r["a"] for r in ok}), "feasible_dz": sorted({r["dz"] for r in ok})}


def main() -> None:
    sw = R.A["sweep"]
    base = boundary(sw["anchor_mm"], sw["anchor_asym_mm"], sw["lateral_offset_mm"], sw["dz_mm"])
    wide = boundary([a * 1.0 for a in (7.5, 15, 20, 25, 30, 35, 45, 60, 90)], [0.0, 15.0, 30.0],
                    [0.0, 2.5, 5.0, 7.5, 10.0, 12.5, 15.0, 20.0, 28.5, 35.0, 50.0, 70.0], [0.0, 10.0, 20.0, 40.0])
    out = {"base": base, "wide": wide}
    OUT.write_text(json.dumps(out, indent=1), encoding="utf-8")
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
