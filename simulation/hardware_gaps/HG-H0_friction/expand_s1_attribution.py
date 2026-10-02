"""SE-E13 / LB-E-037（続き）: `expand_s1_range.py` で境界（最小の比）が 19〜49% 下がった。原因が (A) 比の刻みを細かくしたこと、(B) 波数 0.5 を足したこと、
(C) 波数 2.0 を足したこと のどれかを分ける。比 1〜160（56 点）で、波数の集合だけを変えて 3 通り。PLANAR_FRICTION_SIM。実測ではない。
    python expand_s1_attribution.py [--workers 10]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run as R  # noqa: E402

OUT = R.HERE / "results" / "expand_s1_attribution_2026-10-02.json"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=10)
    a = ap.parse_args()
    base_waves = list(R.SWEEP["ratio_sweep"]["waves"])
    R.SWEEP["ratio_sweep"]["ratios"] = {"log_min": 1.0, "log_max": 160.0, "n": 56}
    out = {}
    for name, waves in (("A_fine_grid_only", base_waves), ("B_plus_0.5", sorted(set(base_waves) | {0.5})), ("C_plus_2.0", sorted(set(base_waves) | {2.0}))):
        R.SWEEP["ratio_sweep"]["waves"] = waves
        out[name] = {"waves": waves, "boundaries": R.s1_boundaries(R.s1_ratio_sweep(a.workers, False))}
        print(name, json.dumps({k: {c: (None if v is None else round(v, 3)) for c, v in d.items() if ":" not in c} for k, d in out[name]["boundaries"].items()}))
    OUT.write_text(json.dumps(out, indent=1, default=float), encoding="utf-8")


if __name__ == "__main__":
    main()
