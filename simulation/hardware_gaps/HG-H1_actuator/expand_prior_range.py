"""SE-E13 / LB-E-038: HG-H1 の prior の幅を約 2 倍にして（中心から両側へ 2 倍。正の量は 0 を下回らない）、判断の指標が 5% 動くかを見る。
指標 = (a) e = 0 で歩容を追従できる確率が 90% 以上になる最大のピーク需要 [N·m]（シナリオを需要の順に並べて内挿）、
(b) 脱力で外から回すトルクが上限より大きい確率、(c) 衝撃力 p50 [N]、(d) 稼働 p05 [分]（最重のシナリオ）。ACTUATOR_MODEL_SIM（prior）。実測ではない。
"""
from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run as R  # noqa: E402

OUT = R.HERE / "results" / "expand_prior_range_2026-10-02.json"
KEYS = ("stall_torque_nm_at_7v4", "no_load_rpm_at_7v4", "stall_current_a_at_7v4", "no_load_current_a", "thermal_resistance_k_per_w",
        "thermal_time_constant_s", "reflected_inertia_kgm2", "backdrive_torque_nm")


def widen(lo_hi, factor=2.0):
    lo, hi = float(lo_hi[0]), float(lo_hi[1])
    c, h = (lo + hi) / 2, (hi - lo) / 2
    return [max(c - factor * h, lo * 0.1), c + factor * h]


def metrics(A) -> dict:
    R.A.clear()
    R.A.update(copy.deepcopy(A))
    rng = np.random.default_rng(20260929)
    pr = R.sample_priors(rng, 3000)
    scs = R.demand_scenarios()
    pts = []
    for sc in scs:
        ev = R.evaluate(sc, pr)
        pts.append((float(sc["tau"].max()), float(np.mean(ev["margin_nm"][pr["cap_err"] == 0.0] >= 0)), ev))
    pts.sort(key=lambda x: x[0])
    ok = [p[0] for p in pts if p[1] >= 0.9]
    heavy = max(pts, key=lambda x: x[0])[2]
    return {"max_peak_demand_trackable_p90_nm": max(ok) if ok else None,
            "p_backdrive_gt_cap": float(np.mean(heavy["backdrive_nm"] > heavy["cap_nm"])),
            "f_impact_p50_n": float(np.percentile(heavy["f_impact_n"], 50)),
            "runtime_p05_min_heaviest": float(np.percentile(heavy["runtime_min"], 5))}


def main() -> None:
    A0 = copy.deepcopy(R.A)
    base = metrics(A0)
    A1 = copy.deepcopy(A0)
    for k in KEYS:
        A1["servo"][k] = widen(A1["servo"][k])
    A1["safety"]["contact_stiffness_n_per_mm"] = widen(A1["safety"]["contact_stiffness_n_per_mm"])
    A1["control"]["cap_model_error"] = [-0.80, -0.40, -0.20, 0.0, 0.20, 0.40, 0.80]
    wide = metrics(A1)
    shift = {k: (None if (base[k] in (None, 0) or wide[k] is None) else (wide[k] - base[k]) / base[k]) for k in base}
    OUT.write_text(json.dumps({"base": base, "wide": wide, "shift": shift}, indent=1), encoding="utf-8")
    print(json.dumps({"base": base, "wide": wide, "shift": shift}, indent=1))


if __name__ == "__main__":
    main()
