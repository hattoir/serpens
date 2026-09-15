"""歩容パラメータの掃引（MuJoCo 版 / Stage I）。**source は MUJOCO_SIM。**

簡易シミュレータの `tools/gait_sweep.py`（KINEMATIC_SIM）とは**別物**。結果を混ぜない。

単一のスコアで「最強の歩容」を決めない。展示機なので、

  FAST（速い） / SMOOTH（滑らか） / LOW_LOAD（サーボに優しい） /
  SNAKE_LIKE（まっすぐ素直に進む） / TIGHT_TURN（よく曲がる）

の代表候補と、**Pareto 非劣解**（他のどれにも全面的に負けていない組み合わせ）を残す。
実機試験へ持っていく候補を 5〜10 個に絞るための道具。

    .\\.venv\\Scripts\\python.exe tools\\gait_sweep_mujoco.py
    .\\.venv\\Scripts\\python.exe tools\\gait_sweep_mujoco.py --belly SNAKE_ANISOTROPIC --seconds 5
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from serpens.config import load_config  # noqa: E402
from serpens.motion.gait import GaitParams, body_joint_names  # noqa: E402

# 小さいほど良い指標（Pareto の向きをここで宣言する）
LOWER_IS_BETTER = ("energy_j", "joint_accel_rms_dps2", "abs_lateral_mm", "torque_saturation")
HIGHER_IS_BETTER = ("forward_mm",)


def snake_like(row: dict) -> float:
    """**蛇らしさの代用値**（小さいほど素直）。横ずれの割合 + 頭の上下動。

    実機では人が見て決める。ここは候補を絞るための目安にすぎない。
    """
    fwd = max(abs(row["forward_mm"]), 1.0)
    return row["abs_lateral_mm"] / fwd + row["head_height_std_mm"] / 20.0


def pareto(rows: list[dict]) -> list[dict]:
    """非劣解（他のどれにも全面的に負けていない行）を返す。"""
    keep = []
    for a in rows:
        dominated = False
        for b in rows:
            if a is b:
                continue
            not_worse = (all(b[k] <= a[k] for k in LOWER_IS_BETTER)
                         and all(b[k] >= a[k] for k in HIGHER_IS_BETTER))
            better = (any(b[k] < a[k] for k in LOWER_IS_BETTER)
                      or any(b[k] > a[k] for k in HIGHER_IS_BETTER))
            if not_worse and better:
                dominated = True
                break
        if not dominated:
            keep.append(a)
    return keep


def main() -> int:
    ap = argparse.ArgumentParser(description="歩容の掃引（MuJoCo）")
    ap.add_argument("--amplitude", default="20,30,40")
    ap.add_argument("--spatial", default="45,60,75")
    ap.add_argument("--freq", default="0.3,0.5")
    ap.add_argument("--belly", default="WHEEL", choices=["WHEEL", "SNAKE_ISOTROPIC", "SNAKE_ANISOTROPIC"])
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", type=Path, default=Path("output/gait_sweep_mujoco.csv"))
    args = ap.parse_args()

    try:
        from simulation.mujoco.model import build_mjcf
        from simulation.mujoco.runner import run_gait
    except ImportError as e:
        print(f"MuJoCo が入っていません（任意依存）: {e}\n"
              "  .venv/Scripts/python.exe -m pip install -r requirements-sim3d.txt")
        return 2

    cfg = load_config()
    spec = build_mjcf(cfg, args.belly)
    body_max = min(float(j["max_deg"]) for j in cfg["joints"] if j["name"] in body_joint_names(cfg))
    gamma = float(cfg["gait"]["turn_full_scale_deg"])
    print(f"belly={args.belly} model={spec.digest} seed={args.seed} "
          f"（**MUJOCO_SIM。摩擦もサーボ応答も未実測**）")
    print("   A   Ω    f | 前進mm 横mm 旋回° 追従° 飽和% E(J) 頭σmm")

    rows: list[dict] = []
    for a in [float(x) for x in args.amplitude.split(",")]:
        for om in [float(x) for x in args.spatial.split(",")]:
            for f in [float(x) for x in args.freq.split(",")]:
                if a + gamma > body_max:
                    print(f"{a:5.0f}{om:5.0f}{f:5.2f} | 機体が拒否する（振幅+旋回が可動域外）")
                    continue
                p = GaitParams(a, om, f)
                straight = run_gait(cfg, args.belly, p, seconds=args.seconds, seed=args.seed, spec=spec)
                turned = run_gait(cfg, args.belly, p, gamma_deg=gamma, seconds=args.seconds,
                                  seed=args.seed, spec=spec)
                row = straight.as_row()
                row.pop("notes", None)
                row["abs_lateral_mm"] = abs(straight.lateral_mm)
                row["turn_deg_with_gamma"] = abs(turned.turn_deg)
                row["snake_like"] = round(snake_like(row), 3)
                rows.append(row)
                print(f"{a:5.0f}{om:5.0f}{f:5.2f} | {straight.forward_mm:6.0f} "
                      f"{straight.lateral_mm:5.0f} {abs(turned.turn_deg):5.1f} "
                      f"{straight.tracking_error_rms_deg:5.2f} {straight.torque_saturation*100:5.1f} "
                      f"{straight.energy_j:4.1f} {straight.head_height_std_mm:5.1f}")
    if not rows:
        return 1

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", newline="", encoding="utf-8") as fp:
        w = csv.DictWriter(fp, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    picks = {
        "FAST": max(rows, key=lambda r: r["forward_mm"]),
        "SMOOTH": min(rows, key=lambda r: r["joint_accel_rms_dps2"]),
        "LOW_LOAD": min(rows, key=lambda r: (r["torque_saturation"], r["energy_j"])),
        "SNAKE_LIKE": min(rows, key=snake_like),
        "TIGHT_TURN": max(rows, key=lambda r: r["turn_deg_with_gamma"]),
    }
    front = pareto(rows)
    print(f"\n{args.out} に {len(rows)} 行。Pareto 非劣解 {len(front)} 件")
    print("代表候補（実機で試す順に並べ替えて使う）:")
    for label, r in picks.items():
        print(f"  {label:11s} A={r['amplitude_deg']:.0f} Ω={r['spatial_freq_deg']:.0f} "
              f"f={r['temporal_freq_hz']:.2f} → 前進{r['forward_mm']:.0f}mm "
              f"旋回{r['turn_deg_with_gamma']:.0f}° E{r['energy_j']:.1f}J 蛇らしさ{r['snake_like']:.2f}")
    print("**この順位は MUJOCO_SIM のもの。** サーボ応答と摩擦を実機で同定するまで確定しない。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
