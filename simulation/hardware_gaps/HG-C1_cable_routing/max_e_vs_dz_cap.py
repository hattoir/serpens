"""SE-E13 / LB-E-043（続き）: 成立する横のずれ e の最大が、アンカーの高さの差 dz の上限にどう依存するか（`expand_sweep_range.py` で境界が 5% 以上動いたので、動く理由を調べる）。
CABLE_GEOMETRY_SIM（幾何のみ）。実測ではない。dz の上限は Design の CAD の量（外装の内側にどれだけ高さの差を取れるか）。
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import expand_sweep_range as X  # noqa: E402

OUT = X.R.HERE / "results" / "max_e_vs_dz_cap_2026-10-02.json"


def main() -> None:
    a = list(np.arange(2.5, 100, 2.5))
    asym = [0.0, 5.0, 10.0, 20.0, 40.0]
    e = list(np.arange(0, 24.1, 0.5))
    tab = {}
    for cap in (0, 10, 20, 30, 40, 60, 80, 120):
        dz = [float(x) for x in np.arange(0, cap + 0.1, 5)] if cap else [0.0]
        r = X.boundary(a, asym, e, dz)
        tab[cap] = {"max_feasible_e_mm": None if r["max_feasible_e_mm"] is None else float(r["max_feasible_e_mm"]), "n_ok": r["n_ok"], "best_bow_mm": r["best_bow_mm"]}
        print(cap, tab[cap])
    OUT.write_text(json.dumps(tab, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
