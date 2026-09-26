r"""フェーズ 3 の完成条件「タグを見失ってからの誤差を数値で」を模擬で出す（**KINEMATIC_SIM 相当。実機ではない**）。

    .\.venv\Scripts\python.exe tools\localization_sim.py --seeds 5 --seconds 300

部屋（config/tags.yaml）を周回し、家具の下（blind）でタグが見えない。滑り（真値）を変えて、
  - 最後のタグ補正から d m 走った時点の 実誤差の最大 / σ
  - σ が正直か（|誤差| ≤ 3σ の割合）
  - 開始条件（localization.start）が σ で塞がるまでの走行距離
を output/localization_sim.md に書く。滑りの真値と推定器の割合（sigma_along_frac）は実機で校正するまで ASSUMED。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from serpens.config import load_config  # noqa: E402
from serpens.localization.sim_run import run_loop_sim  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--seconds", type=float, default=300.0)
    ap.add_argument("--slips", type=str, default="0.05,0.10,0.20", help="真の滑り（前進量に対する割合）")
    ap.add_argument("--out", type=Path, default=Path("output/localization_sim.md"))
    args = ap.parse_args()
    cfg = load_config()
    od, st = cfg["localization"]["odometry"], cfg["localization"]["start"]
    lines = ["# 自己位置の模擬評価（KINEMATIC_SIM 相当。source = SIMULATED、実機ではない）", "",
             f"推定器の前進誤差の割合 sigma_along_frac = {od['sigma_along_frac']}（ASSUMED）、開始条件 σ_xy ≤ {st['max_sigma_xy_m']} m、"
             f"σ_yaw ≤ {st['max_sigma_yaw_rad']} rad。周回 {args.seconds:.0f} s × {args.seeds} seeds。", "",
             "| 真の滑り | 補正数 / 棄却 | \\|誤差\\| ≤ 3σ | 0.5 m 見えない: 最大誤差 / σ | 1.0 m: 最大誤差 / σ | 2.0 m: 最大誤差 / σ | 塞がるまでの距離 |",
             "|---|---|---|---|---|---|---|"]
    for slip in (float(s) for s in args.slips.split(",")):
        runs = [run_loop_sim(cfg, seed=s, seconds=args.seconds, slip_frac=slip) for s in range(args.seeds)]
        cons = np.mean([r.consistency(3.0) for r in runs])
        corr, rej = sum(r.corrections for r in runs), sum(r.rejected for r in runs)
        cells = []
        for d in (0.5, 1.0, 2.0):
            pairs = [r.error_at_blind_distance(d) for r in runs]
            e = max((p[0] for p in pairs if p[0] == p[0]), default=float("nan"))
            s = float(np.nanmedian([p[1] for p in pairs]))
            cells.append(f"{e:.2f} / {s:.2f} m")
        blk = [r.blind_distance_until_blocked() for r in runs]
        blk_s = f"{min(b for b in blk if b is not None):.2f} m" if any(b is not None for b in blk) else "塞がらず"
        lines.append(f"| {slip:.0%} | {corr} / {rej} | {cons:.0%} | " + " | ".join(cells) + f" | {blk_s} |")
    lines += ["", "読み方: 滑りが推定器の割合（15%）を超えると σ が誤差を下回る（正直でなくなる）。実機で滑りを測って割合を直す。",
              "「塞がるまでの距離」= タグを見ずに走れる距離。家具の下の inspect_point はこの範囲で計画する。"]
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
