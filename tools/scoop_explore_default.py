"""探索（成立性調査）: 全 36 形状 × 4 対象物 × 2 床（20 mm/s、位置ずれ 0 / 5 mm、N = 3）。既定の設定でどれだけ成功するか。

    .\.venv\Scripts\python.exe tools\scoop_explore_default.py

結果は simulation/results/scoop_exploration_2026-09-29.md。MUJOCO_SIM。実物の試験片の結果が出るまで「候補」。
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from simulation.scoop.model import all_shapes, load_config  # noqa: E402
from simulation.scoop.sweep import make_cases, run_cases, summarize  # noqa: E402


def main() -> None:
    cfg = load_config()
    shapes = all_shapes(cfg)
    cases = make_cases(cfg, shapes, 3, speeds=[20.0], offsets=[0.0, 5.0])
    print(len(cases), "cases", flush=True)
    rows = run_cases(cases, workers=14)
    for b in summarize(rows, ("obj", "floor")):
        print(b["obj"], b["floor"], "rate", round(b["rate"], 3),
              {o: b[f"n_{o}"] for o in ("success", "pushed_ahead", "lateral", "under", "pinched", "on_ramp", "other") if b[f"n_{o}"]},
              "push_mean", round(b["push_mean_mm"], 1))
    succ = [r for r in rows if r["success"]]
    print("total success", len(succ), "of", len(rows))
    for r in succ[:10]:
        print(r["shape"], r["obj"], r["floor"], r["speed"], r["offset"], r["trial"])


if __name__ == "__main__":
    main()
