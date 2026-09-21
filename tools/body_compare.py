r"""胴体ヨー 6 / 8 / 10 軸の比較表（KINEMATIC_SIM）。波数の掃引と速度を揃えた条件を CSV と Markdown に出す。

    .\.venv\Scripts\python.exe tools\body_compare.py
    .\.venv\Scripts\python.exe tools\body_compare.py --configs yaw6,yaw8,yaw10 --out output\body_compare.csv

**身体構成の決定はしない。** 人の評価（docs/human_pilot.md）と並べるための数値。
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from simulation.body_compare import CONFIGS, SOURCE, sweep  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--configs", default=",".join(CONFIGS))
    ap.add_argument("--amplitude", type=float, default=30.0)
    ap.add_argument("--freq", type=float, default=0.5)
    ap.add_argument("--out", type=Path, default=Path("output/body_compare.csv"))
    args = ap.parse_args()
    rows = sweep(args.configs.split(","), args.amplitude, args.freq)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", newline="", encoding="utf-8") as fp:
        w = csv.DictWriter(fp, fieldnames=list(rows[0].as_dict()))
        w.writeheader()
        for r in rows:
            w.writerow(r.as_dict())
    md = args.out.with_suffix(".md")
    with md.open("w", encoding="utf-8") as fp:
        fp.write(f"# 胴体ヨー構成の比較（source = {SOURCE}。摩擦・サーボ応答は模擬。**決定用ではない**）\n\n")
        fp.write("| config | N | link mm | 全長 mm | 条件 | 狙い波数 | Ω° | f Hz | 可視波数 | 前進 mm/周期 | 前進 mm/s | 旋回 °/周期 | 関節速度 °/s | 関節加速度 °/s² | 上限 |\n")
        fp.write("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n")
        for r in rows:
            fp.write(f"| {r.config} | {r.body_axes} | {r.link_mm:.0f} | {r.length_mm:.0f} | {r.condition} | {r.target_waves:.1f} | "
                     f"{r.spatial_freq_deg:.0f} | {r.temporal_freq_hz:.2f} | {r.visible_waves:.2f} | {r.forward_mm_per_cycle:.0f} | "
                     f"{r.forward_mm_s:.0f} | {r.turn_deg_per_cycle:.1f} | {r.peak_joint_speed_dps:.0f} | {r.peak_joint_accel_dps2:.0f} | "
                     f"{'clipped' if r.clipped else ''} |\n")
    print(f"{args.out} / {md}: {len(rows)} 行（{SOURCE}）")
    for r in rows:
        if r.condition == "RAW":
            print(f"{r.config:14s} 波数狙い {r.target_waves:.1f} Ω={r.spatial_freq_deg:5.1f}  可視 {r.visible_waves:.2f}  "
                  f"前進 {r.forward_mm_per_cycle:5.0f}mm/周期 {r.forward_mm_s:4.0f}mm/s  旋回 {r.turn_deg_per_cycle:5.1f}°")
    return 0


if __name__ == "__main__":
    sys.exit(main())
