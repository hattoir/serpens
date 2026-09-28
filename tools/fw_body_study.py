r"""Floor Watch 身体構成・腹面・トルク上限の感度を MuJoCo で測り、CSV と Markdown に出す（MUJOCO_SIM）。

    .\.venv\Scripts\python.exe tools\fw_body_study.py
    .\.venv\Scripts\python.exe tools\fw_body_study.py --out simulation\results\fw_body_study

**構成を決めない。** 実物で何を測れば決まるかを絞るための感度表。摩擦・質量・サーボ応答はすべて未実測。
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from simulation.floorwatch_body import (  # noqa: E402
    CONFIGS, GAIT_GRID_AMPLITUDE_DEG, GAIT_GRID_WAVES, MIN_PATROL_SPEED_MM_S, SOURCE, StudyRow, best_gait,
    neck_static_torque_nm, run_case)
from simulation.mujoco.belly import PROFILES, BellyProfile, sweep_profiles  # noqa: E402

ALONG = [0.05, 0.10, 0.20, 0.30]      # 前後の摩擦（掃引。**実測値ではない**）
ACROSS = [0.30, 0.50, 0.80]           # 横の摩擦
TORQUES = [0.30, 0.45, 0.90, 1.50]    # N·m。0.45 が現行ソフト上限（C044 参照値由来）
NECK_MASS_G = [60, 90, 120, 165, 200]
NECK_COG_MM = [25, 35, 55, 75]


def _fmt(v: object) -> str:
    if v is None:
        return "—"
    if isinstance(v, float):
        return f"{v:.3f}" if abs(v) < 10 else f"{v:.0f}"
    return str(v)


def _table(rows: list[StudyRow], cols: list[str]) -> str:
    head = "| " + " | ".join(cols) + " |\n|" + "---|" * len(cols) + "\n"
    return head + "".join("| " + " | ".join(_fmt(r.as_dict()[c]) for c in cols) + " |\n" for r in rows)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path("simulation/results/fw_body_study"))
    ap.add_argument("--seconds", type=float, default=8.0)
    args = ap.parse_args()
    s = args.seconds
    base = ["WHEEL", "SNAKE_ISOTROPIC", "SNAKE_ANISOTROPIC"]

    # E1: 構成 × 腹面ごとに歩容を選び直した最良（1 つの歩容で比べると構成の差より歩容の相性が出る）
    e1 = [best_gait(c, PROFILES[b], seconds=s) for c in CONFIGS for b in base]
    gait = {r.config: (r.amplitude_deg, r.waves) for r in e1 if r.belly == "SNAKE_ANISOTROPIC"}

    def at(c: str, belly: BellyProfile, **kw: float) -> StudyRow:
        a, w = gait[c]
        return run_case(c, belly, amplitude_deg=a, waves=w, seconds=s, **kw)

    e2 = [at(c, p) for c in CONFIGS for p in sweep_profiles(ALONG, ACROSS)]
    e3 = [at(c, PROFILES["SNAKE_ANISOTROPIC"], torque_limit_nm=t) for c in CONFIGS for t in TORQUES]
    # E4: 旋回は同じ歩容の直進との差で見る（直進でも開始位相による向きのずれが残る）
    e4 = [at(c, PROFILES["SNAKE_ANISOTROPIC"], gamma_deg=g) for c in CONFIGS for g in (0.0, 20.0, -20.0)]
    all_rows = e1 + e2 + e3 + e4

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.with_suffix(".csv").open("w", newline="", encoding="utf-8") as fp:
        w = csv.DictWriter(fp, fieldnames=list(all_rows[0].as_dict()))
        w.writeheader()
        for r in all_rows:
            w.writerow(r.as_dict())

    main_cols = ["config", "belly", "amplitude_deg", "waves", "body_axes", "mass_kg", "speed_mm_s", "body_lengths_per_s",
                 "lateral_slip_ratio", "energy_j_per_m", "peak_torque_nm", "torque_saturation",
                 "tracking_error_rms_deg", "turn_deg"]
    with args.out.with_suffix(".md").open("w", encoding="utf-8") as fp:
        fp.write(f"# Floor Watch 身体構成の感度（source = {SOURCE}。**決定用ではない**）\n\n")
        fp.write("摩擦・質量・サーボ応答（kp）はすべて未実測。serpenoid 0.5 Hz、"
                 f"{s:.0f} 秒。質量は overlay の ASSUMPTION。energy は機械仕事で電気消費ではない。"
                 "turn_deg は直進指令でも出る向きのずれ（開始位相の影響。実機では IMU で補正する前提）。\n\n")
        fp.write(f"## E1 構成 × 腹面（トルク上限 0.45 N·m。振幅 {GAIT_GRID_AMPLITUDE_DEG} × 波数 {GAIT_GRID_WAVES} "
                 "から最速を選んだ。E2〜E4 は各構成の SNAKE_ANISOTROPIC の歩容で固定）\n\n" + _table(e1, main_cols))
        fp.write("\n## E2 腹面の摩擦掃引（前後 along × 横 across、上限 0.45 N·m）\n\n")
        fp.write(_table(e2, ["config", "slide_along", "slide_across", "anisotropy", "speed_mm_s",
                             "lateral_slip_ratio", "torque_saturation", "tracking_error_rms_deg"]))
        ok = [r for r in e2 if r.speed_mm_s >= MIN_PATROL_SPEED_MM_S]
        fp.write(f"\n巡回の目安 {MIN_PATROL_SPEED_MM_S:.0f} mm/s（ASSUMPTION）以上: {len(ok)}/{len(e2)} 条件。"
                 f" 最小の異方性: {min((r.anisotropy for r in ok), default=float('nan')):.1f}\n")
        fp.write("\n## E3 トルク上限の感度（SNAKE_ANISOTROPIC）\n\n")
        fp.write(_table(e3, ["config", "torque_limit_nm", "speed_mm_s", "peak_torque_nm",
                             "torque_saturation", "tracking_error_rms_deg", "energy_j_per_m"]))
        fp.write("\n## E4 旋回（γ0 = 0 / +20 / −20°、head_weighted、SNAKE_ANISOTROPIC）\n\n")
        fp.write(_table(e4, ["config", "gamma_deg", "turn_deg", "turn_radius_mm", "speed_mm_s",
                             "torque_saturation"]))
        fp.write("\n| config | 左 +20° の直進との差 ° | 右 −20° の直進との差 ° |\n|---|---|---|\n")
        for c in CONFIGS:
            t = {r.gamma_deg: r.turn_deg for r in e4 if r.config == c}
            fp.write(f"| {c} | {t[20.0] - t[0.0]:+.1f} | {t[-20.0] - t[0.0]:+.1f} |\n")
        fp.write("\n## E5 首 J1 の静的保持トルク（ANALYTIC、τ = m g d、水平 = 最大）\n\n")
        fp.write("| 頭側の質量 g | " + " | ".join(f"d={d}mm" for d in NECK_COG_MM) + " |\n|"
                 + "---|" * (len(NECK_COG_MM) + 1) + "\n")
        for m in NECK_MASS_G:
            fp.write(f"| {m} | " + " | ".join(f"{neck_static_torque_nm(m, d, 0.0):.3f}" for d in NECK_COG_MM)
                     + " |\n")
        fp.write("\n単位 N·m。現行ソフト上限 0.45 N·m（C044 参照値由来、未実測）。動的な持ち上げ・床の反力・"
                 "頭スキッドで体を支える荷重は含まない。\n")
    print(f"{len(all_rows)} runs → {args.out.with_suffix('.md')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
