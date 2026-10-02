"""SE-E13 / LB-E-037: HG-H0 の摩擦比の判断境界を、掃引範囲を 2 倍にして取り直す（比の上限 80 → 160、刻みを細かく 28 → 56、波数に 0.5 / 2.0 を追加）。
境界（Head Yaw / Body Yaw が足りる最小の比）が 5% 以内で動かなければ「これ以上拡張しない」（dropped、指標の値つき）。PLANAR_FRICTION_SIM。実測ではない。
    python expand_s1_range.py [--workers 4]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run as R  # noqa: E402

OUT = R.HERE / "results" / "expand_s1_range_2026-10-02.json"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=4)
    a = ap.parse_args()
    base_cfg = json.loads(json.dumps(R.SWEEP["ratio_sweep"]))
    base = R.s1_boundaries(R.s1_ratio_sweep(a.workers, False))
    R.SWEEP["ratio_sweep"]["ratios"] = {"log_min": 1.0, "log_max": 160.0, "n": 56}
    R.SWEEP["ratio_sweep"]["waves"] = sorted(set(base_cfg["waves"]) | {0.5, 2.0})
    wide = R.s1_boundaries(R.s1_ratio_sweep(a.workers, False))
    out = {"base": base, "wide": wide, "shift": {}}
    for law in base:
        for k, v in base[law].items():
            w = wide[law][k]
            out["shift"][f"{law}/{k}"] = None if (v is None or w is None) else (w - v) / v
    OUT.write_text(json.dumps(out, ensure_ascii=False, indent=1, default=float), encoding="utf-8")
    for k, v in out["shift"].items():
        print(k, base[k.split("/")[0]][k.split("/")[1]], wide[k.split("/")[0]][k.split("/")[1]], None if v is None else f"{v:+.1%}")


if __name__ == "__main__":
    main()
