r"""H0 摩擦クーポンの実測 CSV を解析し、MuJoCo で推進を予測して Markdown に出す。

    .\.venv\Scripts\python.exe tools\h0_friction.py hardware\prototypes\H0_friction\h0_friction_20261001.csv
    → simulation/results/h0_friction_20261001.md

摩擦係数は実測（HARDWARE_VERIFIED はクーポン × 床の組み合わせに限る）。前進の予測は PHYSICS_SIM。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from simulation.floorwatch_body import CONFIGS, MIN_PATROL_SPEED_MM_S  # noqa: E402
from simulation.h0_friction import DIRECTIONS, load_csv, predict, verdict  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--seconds", type=float, default=8.0)
    args = ap.parse_args()
    out = args.out or Path("simulation/results") / (args.csv.stem + ".md")

    groups = load_csv(args.csv)
    complete = [g for g in groups if g.complete()]
    preds = [p for g in complete for p in predict(g, seconds=args.seconds)]

    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8") as fp:
        fp.write(f"# H0 摩擦クーポンの結果（入力: `{args.csv.as_posix()}`）\n\n")
        fp.write("摩擦係数 = 実測（傾斜法なら静止摩擦 tanθ）。前進の予測 = **PHYSICS_SIM**（質量・kp・接触形状は未実測）。\n\n")
        fp.write("## 摩擦係数（平均 [最小〜最大] n）\n\n| 床 | クーポン | "
                 + " | ".join(DIRECTIONS) + " | 横/前 | 横/(前後平均) |\n|---|---|---|---|---|---|---|\n")
        for g in groups:
            cells = [f"{g.mu[d].mean:.3f} [{g.mu[d].lo:.3f}〜{g.mu[d].hi:.3f}] n={g.mu[d].n}" if d in g.mu else "—"
                     for d in DIRECTIONS]
            ratios = (f"{g.ratio_optimistic:.2f} | {g.ratio_conservative:.2f}" if g.complete() else "— | —")
            fp.write(f"| {g.floor_id} | {g.coupon} | " + " | ".join(cells) + f" | {ratios} |\n")
        missing = [g for g in groups if not g.complete()]
        if missing:
            fp.write("\n3 方向そろっていない組み合わせは予測に使っていない: "
                     + ", ".join(f"{g.floor_id}/{g.coupon}" for g in missing) + "\n")
        fp.write(f"\n## 前進の予測 mm/s（PHYSICS_SIM、各構成で歩容を選び直した最良。目安 {MIN_PATROL_SPEED_MM_S:.0f} mm/s は ASSUMPTION）\n\n")
        fp.write("| 床 | クーポン | 前後の μ の取り方 | " + " | ".join(CONFIGS) + " |\n|---|---|---|"
                 + "---|" * len(CONFIGS) + "\n")
        for p in preds:
            fp.write(f"| {p.floor_id} | {p.coupon} | {p.variant} ({p.along:.3f} / 横 {p.across:.3f}) | "
                     + " | ".join(f"{p.speed_mm_s[c]:.0f}" for c in CONFIGS) + " |\n")
        if preds:
            fp.write("\n## 床ごとの読み（**提案。正式 Decision ではない**）\n\n| 床 | 保守側 | 楽観側 |\n|---|---|---|\n")
            cons, opt = verdict(preds, "conservative"), verdict(preds, "optimistic")
            for floor in cons:
                fp.write(f"| {floor} | {cons[floor]} | {opt[floor]} |\n")
            fp.write("\nPROPULSION_OK = Pure Snake 継続・6 本目は Head Yaw 寄りで評価 / NEEDS_BODY_YAW = 6 本目は Body Yaw 寄り / "
                     "SNAKE_INSUFFICIENT = その床では Wheel Belly / Hybrid の候補\n")
    print(f"{len(groups)} 組（うち予測 {len(complete)} 組）→ {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
