"""SE-E13 / LB-E-042: HG-S3 のリミッターの滑りトルクの窓（0.7〜1.0 N·m）を、掃引の範囲を約 2 倍（滑りトルク 0.4〜1.5 → 0.1〜3.0、
過渡の倍率 1〜2 → 1〜4、個体差 ±30% → ±60%）にして取り直し、窓の下限・上限が 5% 動くかを見る。
下限 = 不要な滑りが基準（5%）以下になる最小の滑りトルク（最悪の個体差・過渡）、上限 = 歯車を守れる確率が基準（0.95）以上の最大の滑りトルク（最悪の個体差）。
TORQUE_LIMITER_SWEEP_SIM（prior ASSUMPTION）。実測ではない。
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run as R  # noqa: E402

OUT = R.HERE / "results" / "expand_window_range_2026-10-02.json"


def window(slips, dyns, tols) -> dict:
    R.A["sweep"]["slip_torque_nm"], R.A["sweep"]["dynamic_factor"], R.A["sweep"]["slip_tolerance"] = list(slips), list(dyns), list(tols)
    o = R.run()
    crit = R.A["criteria"]
    lows, ups = {}, {}
    for dyn in dyns:
        for tl in tols:
            ok = [r["slip"] for r in o["nuisance"] if r["dyn"] == dyn and abs(r["tol"] - tl) < 1e-9 and r["p_nuisance"] <= float(crit["max_nuisance_slip"])]
            lows[f"dyn={dyn:g},tol={tl:+.2f}"] = min(ok) if ok else None
    for tl in tols:
        ok = [r["slip"] for r in o["gear"] if abs(r["tol"] - tl) < 1e-9 and r["p_protect_design"] >= float(crit["min_gear_protect"])]
        ups[f"tol={tl:+.2f}"] = max(ok) if ok else None
    return {"lower_bound_nm": lows, "upper_bound_nm": ups}


def main() -> None:
    fine = list(np.round(np.arange(0.1, 3.0001, 0.05), 3))
    base = window([s for s in fine if 0.4 <= s <= 1.5], [1.0, 1.5, 2.0], [-0.30, 0.0, 0.30])
    wide = window(fine, [1.0, 1.5, 2.0, 3.0, 4.0], [-0.60, -0.30, 0.0, 0.30, 0.60])
    shift = {}
    for k, v in base["lower_bound_nm"].items():
        w = wide["lower_bound_nm"].get(k)
        shift[f"lower/{k}"] = None if (v is None or w is None) else (w - v) / v
    for k, v in base["upper_bound_nm"].items():
        w = wide["upper_bound_nm"].get(k)
        shift[f"upper/{k}"] = None if (v is None or w is None) else (w - v) / v
    OUT.write_text(json.dumps({"base": base, "wide": wide, "shift": shift}, indent=1), encoding="utf-8")
    print(json.dumps({"base": base, "wide": wide, "shift": shift}, indent=1))


if __name__ == "__main__":
    main()
