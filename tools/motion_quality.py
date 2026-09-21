r"""動きの質の評価（蛇らしさ・愛着）。**実機ゼロで回せる。source = KINEMATIC_SIM。**

  頭部軌道の LDJ / 静止率 / 可視波数 / 一次反応レイテンシ / GAR（simulation/quality.py）

    .\.venv\Scripts\python.exe tools\motion_quality.py
    .\.venv\Scripts\python.exe tools\motion_quality.py --seeds 3 --alone 120 --person 60
    .\.venv\Scripts\python.exe tools\motion_quality.py --waves-only --overlay config\robot_yaw8.yaml   # 8軸案の前進量と可視波数

数値の目標（外部レビュー）: 静止率 40〜60% / 可視波数 2 / 一次反応 ≤ 300ms / GAR 0.4〜0.6。
LDJ は試行時間を統制した相対比較にだけ使う。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np  # noqa: E402

from serpens.config import load_config  # noqa: E402
from serpens.motion.gait import GaitParams  # noqa: E402
from serpens.sim.measure import per_cycle_advance  # noqa: E402
from simulation.quality import SOURCE, evaluate, visible_waves  # noqa: E402


def waves_for_presets(cfg: dict, names: list[str]) -> None:
    """歩容プリセットごとに前進量と可視波数（指令角そのまま。サーボ応答なし）。"""
    from serpens.motion.gait import GaitEngine, gait_period_s
    from serpens.sim.world import BodyPose, World

    print(f"{'preset':10s} {'A':>4s} {'Ω':>5s} {'f':>5s}  前進mm/周期  可視波数")
    for name in names:
        p = GaitParams.from_cfg(cfg["gait"]["presets"][name])
        adv = per_cycle_advance(cfg, p, 2, 3).per_cycle_mm
        w = World(cfg, BodyPose(600.0, 600.0, 0.0))
        eng = GaitEngine(cfg)
        eng.start(p)
        dt, T, t, waves = float(cfg["sim"]["dt_s"]), gait_period_s(p), 0.0, []
        for _ in range(int(round(3 * T / dt))):
            t += dt
            w.step(eng.update(t), dt)
            if t > T:
                waves.append(visible_waves(w.world_points()[:, :2], float(cfg["quality"]["wave_min_turn_deg"])))
        print(f"{name:10s} {p.amplitude_deg:4.0f} {p.spatial_freq_deg:5.0f} {p.temporal_freq_hz:5.2f}  "
              f"{adv:10.0f}  {np.mean(waves):8.2f}")


def main() -> int:
    ap = argparse.ArgumentParser(description="動きの質の評価（KINEMATIC_SIM）")
    ap.add_argument("--seeds", type=int, default=2)
    ap.add_argument("--alone", type=float, default=120.0, help="誰もいない巡回の秒数（LDJ はこの長さで統制）")
    ap.add_argument("--person", type=float, default=60.0, help="人が現れてからの秒数")
    ap.add_argument("--overlay", default=None, help="config の上書き（例: config/robot_yaw8.yaml）")
    ap.add_argument("--waves-only", action="store_true", help="歩容プリセットの前進量と可視波数だけ")
    args = ap.parse_args()
    cfg = load_config(overlay=args.overlay)
    n_body = len([j for j in cfg["joints"] if j["axis"] == "yaw"]) - 1
    print(f"source={SOURCE}（簡易シミュレータ。摩擦・サーボ応答は模擬）胴体ヨー {n_body} 軸"
          + (f" overlay={args.overlay}" if args.overlay else ""))
    names = [n for n in ("forward", "one_wave", "stalk") if n in cfg["gait"]["presets"]]
    waves_for_presets(cfg, names)
    if args.waves_only:
        return 0
    rows = []
    for seed in range(1, args.seeds + 1):
        r = evaluate(cfg, args.alone, args.person, seed)
        rows.append(r)
        print(f"seed {seed}: 静止率 {r.still_ratio:.2f}  LDJ {r.head_ldj:.2f}  可視波数 {r.visible_waves:.2f}  "
              f"一次反応 {r.primary_latency_s * 1000:.0f}ms（+カメラ予算 {r.vision_budget_s * 1000:.0f}ms）  "
              f"GAR {r.gar:.2f}  {' '.join(r.notes)}")
    keys = ("still_ratio", "head_ldj", "visible_waves", "primary_latency_s", "gar")
    mean = {k: float(np.nanmean([getattr(r, k) for r in rows])) for k in keys}
    print("平均:", {k: round(v, 3) for k, v in mean.items()})
    print("目標: 静止率 0.40〜0.60 / 可視波数 2（9軸案は 1 が上限） / 一次反応 0.30s 以下 / GAR 0.40〜0.60")
    return 0


if __name__ == "__main__":
    sys.exit(main())
