"""歩容パラメータを掃引して、速さ以外も含めて比べる（SIMULATED）。

展示機なので「最速」だけでは決められない。見るのは6つ:

  速さ / 旋回性 / 蛇らしさ（尾が頭の軌跡をなぞるか）/ 滑らかさ（関節加速度）/
  サーボ負荷（最高角速度の使用率）/ 消費電力の目安（関節速度の総和。**ワットではない**）

**すべてシミュレーション上の値**で、摩擦は未実測（docs/verification_status.md）。
実機が来たら tools/fit_sim.py で摩擦を同定し、ここを回し直して候補を絞る。

    .\\.venv\\Scripts\\python.exe tools\\gait_sweep.py
    .\\.venv\\Scripts\\python.exe tools\\gait_sweep.py --amplitude 20,25,30 --freq 0.3,0.5 --belly snake
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from serpens.config import load_config  # noqa: E402
from serpens.motion.gait import GaitEngine, GaitParams, body_joint_names, gait_period_s  # noqa: E402
from serpens.sim.world import BodyPose, World  # noqa: E402

FAR_MM = 1.0e6              # 測定中にマット端へ当たらないよう広くする
WARMUP_CYCLES = 2.0
MEASURE_CYCLES = 3.0


@dataclass(frozen=True)
class Score:
    """1つの歩容パラメータに対する評価。"""

    speed_mm_s: float           # 前進速度
    turn_deg_cycle: float       # γ0 を入れたときの旋回率
    trail_error_mm: float       # 尾が頭の軌跡からどれだけ外れるか（小さいほど蛇らしい）
    smoothness_dps2: float      # 関節角加速度の RMS（小さいほど滑らか）
    speed_use: float            # 最高角速度に対する使用率（1.0 で上限）
    power_proxy_dps: float      # 関節速度の総和の平均（**消費電力の目安。ワットではない**）


def _run(cfg: dict, p: GaitParams, cycles: float, gamma0: float) -> tuple[World, list, list, list, list]:
    """歩かせて、関節角・頭の軌跡・尾の軌跡・向きを記録する。"""
    c = dict(cfg)
    c["mat"] = dict(cfg["mat"], width_mm=FAR_MM, depth_mm=FAR_MM)
    dt = float(cfg["sim"]["dt_s"])
    w = World(c, BodyPose(FAR_MM / 2, FAR_MM / 2, 0.0))
    eng = GaitEngine(c)
    eng.start(p, gamma0)
    angles, head, tail, theta = [], [], [], []
    t = 0.0
    for _ in range(int(round(cycles * gait_period_s(p) / dt))):
        t += dt
        a = eng.update(t)
        w.step(a, dt)
        angles.append(dict(a))
        pts = w.world_points()
        head.append(pts[-1][:2])           # 頭先端
        tail.append(pts[0][:2])            # 尾端
        theta.append(w.pose.theta)
    return w, angles, head, tail, theta


def _trail_error(head: list, tail: list, skip: int) -> float:
    """尾の位置が、頭が通った軌跡からどれだけ離れているか [mm]。

    skip は「頭が体長ぶん進むのにかかる時間」。それより前に頭が通った点とだけ比べる。
    """
    skip = max(min(skip, len(head) - 2), 1)
    if len(head) <= skip:
        return float("nan")
    path = np.array(head[:-skip])
    errs = []
    for q in tail[skip:]:
        d = np.linalg.norm(path - np.array(q), axis=1)
        errs.append(float(d.min()))
    return float(np.mean(errs)) if errs else float("nan")


def evaluate(cfg: dict, p: GaitParams, gamma0: float) -> Score:
    """1つのパラメータを評価する。"""
    dt = float(cfg["sim"]["dt_s"])
    names = body_joint_names(cfg)
    vmax = min(float(j["max_speed_dps"]) for j in cfg["joints"] if j["name"] in names)
    warm = int(round(WARMUP_CYCLES * gait_period_s(p) / dt))

    _, angles, head, tail, _ = _run(cfg, p, WARMUP_CYCLES + MEASURE_CYCLES, 0.0)
    seq = np.array([[a[n] for n in names] for a in angles[warm:]])
    vel = np.diff(seq, axis=0) / dt
    acc = np.diff(vel, axis=0) / dt
    dist = float(np.linalg.norm(np.array(head[-1]) - np.array(head[warm])))
    seconds = (len(angles) - warm) * dt
    speed = dist / seconds if seconds else 0.0
    body_mm = float(cfg["body"]["length_mm"])
    lag_s = body_mm / speed if speed > 1e-6 else 1.0          # 頭が体長ぶん進む時間
    _, _, _, _, theta = _run(cfg, p, WARMUP_CYCLES + MEASURE_CYCLES, gamma0)
    turned_deg = math.degrees(theta[-1] - theta[warm])        # 計測区間だけの回転

    return Score(
        speed_mm_s=speed,
        turn_deg_cycle=turned_deg / MEASURE_CYCLES,
        trail_error_mm=_trail_error(head[warm:], tail[warm:], skip=int(lag_s / dt)),
        smoothness_dps2=float(np.sqrt((acc ** 2).mean())) if acc.size else 0.0,
        speed_use=float(np.abs(vel).max() / vmax) if vel.size else 0.0,
        power_proxy_dps=float(np.abs(vel).sum(axis=1).mean()) if vel.size else 0.0,
    )


def main() -> None:
    ap = argparse.ArgumentParser(description="歩容パラメータの掃引（シミュレーション）")
    ap.add_argument("--amplitude", default="20,25,30", help="振幅 A [deg] をカンマ区切りで")
    ap.add_argument("--spatial", default="45,60,75", help="空間周波数 Ω [deg]")
    ap.add_argument("--freq", default="0.3,0.5,0.7", help="時間周波数 [Hz]")
    ap.add_argument("--belly", default=None, choices=["wheel", "snake"])
    ap.add_argument("--profile", default=None, choices=["LOW", "MEDIUM", "HIGH", "ANISOTROPIC"])
    ap.add_argument("--out", type=Path, default=Path("output/gait_sweep.csv"))
    args = ap.parse_args()

    cfg = load_config()
    if args.belly:
        cfg["belly"]["type"] = args.belly
    if args.profile:
        cfg["belly"]["friction_profile"] = args.profile
    gamma0 = float(cfg["gait"]["turn_full_scale_deg"])
    lim = cfg["link"]["limits"]
    body_max = min(float(j["max_deg"]) for j in cfg["joints"] if j["name"] in body_joint_names(cfg))

    rows = []
    print(f"belly={cfg['belly']['type']}/{cfg['belly']['friction_profile']}  "
          f"（**摩擦は未実測。相対比較にのみ使う**）")
    print("  A   Ω    f | 速度mm/s 旋回°/周期 なぞりmm 滑らかさ 速度使用率 電力目安")
    for a in [float(x) for x in args.amplitude.split(",")]:
        for om in [float(x) for x in args.spatial.split(",")]:
            for f in [float(x) for x in args.freq.split(",")]:
                if a + abs(gamma0) > body_max or a > float(lim["amplitude_deg"]):
                    print(f"{a:5.0f}{om:5.0f}{f:5.2f} | 機体が拒否する値（可動域・上限外）")
                    continue
                p = GaitParams(a, om, f)
                s = evaluate(cfg, p, gamma0)
                rows.append(dict(amplitude_deg=a, spatial_deg=om, freq_hz=f,
                                 speed_mm_s=round(s.speed_mm_s, 1),
                                 turn_deg_cycle=round(s.turn_deg_cycle, 1),
                                 trail_error_mm=round(s.trail_error_mm, 1),
                                 smoothness_dps2=round(s.smoothness_dps2, 1),
                                 speed_use=round(s.speed_use, 2),
                                 power_proxy_dps=round(s.power_proxy_dps, 1)))
                print(f"{a:5.0f}{om:5.0f}{f:5.2f} | {s.speed_mm_s:8.1f} {s.turn_deg_cycle:9.1f}"
                      f" {s.trail_error_mm:8.1f} {s.smoothness_dps2:9.0f} {s.speed_use:9.2f}"
                      f" {s.power_proxy_dps:9.0f}")
    if rows:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("w", newline="", encoding="utf-8") as fp:
            wr = csv.DictWriter(fp, fieldnames=list(rows[0]))
            wr.writeheader()
            wr.writerows(rows)
        print(f"\n{args.out} に {len(rows)} 行を書きました。")
        best = min(rows, key=lambda r: r["trail_error_mm"])
        fast = max(rows, key=lambda r: r["speed_mm_s"])
        print(f"最も蛇らしい（なぞり誤差が小さい）: A={best['amplitude_deg']:.0f} "
              f"Ω={best['spatial_deg']:.0f} f={best['freq_hz']}  誤差 {best['trail_error_mm']}mm")
        print(f"最も速い: A={fast['amplitude_deg']:.0f} Ω={fast['spatial_deg']:.0f} "
              f"f={fast['freq_hz']}  {fast['speed_mm_s']}mm/s")
        print("**この順位はシミュレーション上のもの。実機で摩擦を測ってから選ぶこと。**")


if __name__ == "__main__":
    main()
