"""探索（成立性調査）: 解析との照合（傾斜板の保持条件 tan α ≤ μ）と、押して逃げる原因の切り分け。

    .\.venv\Scripts\python.exe tools\scoop_explore_sanity.py

結果は simulation/results/scoop_exploration_2026-09-29.md。MUJOCO_SIM。実物の試験片の結果が出るまで「候補」。
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from simulation.scoop.model import Shape, load_config  # noqa: E402
from simulation.scoop.runner import place_on_ramp_and_release, run_episode  # noqa: E402


def main() -> None:
    cfg = load_config()
    print("== 1. 解析の照合: 傾斜板の上の物は tan(alpha) <= mu なら保持される（scoop mu = 0.3、tan = 0.141 / 0.213 / 0.325）")
    for a in (8.0, 12.0, 18.0):
        sh = Shape(0.6, a, False, 30.0)
        for obj in ("coin_1yen", "crumb_cube"):
            for mu in (0.3, 0.5):
                r = place_on_ramp_and_release(cfg, sh, obj, "flooring", rel_x_mm=15.0, seconds=1.0, scoop_mu=mu)
                import math
                hold = math.tan(math.radians(a)) <= mu
                print(f"  alpha {a:4.0f} {obj:10s} mu {mu}: moved {r['moved_mm']:7.2f} mm  (解析: {'保持' if hold else '滑る'})")
    print("== 2. 押して逃げる原因の切り分け（形 t0.4/a8/open/w30、コイン、20 mm/s、位置ずれ 0）")
    sh = Shape(0.4, 8.0, False, 30.0)
    base = dict(shape=sh, obj="coin_1yen", floor="flooring", speed_mm_s=20.0, y_offset_mm=0.0, seed=1)
    tests = [("既定", {}), ("先端の浮き 0（床に接する）", {"clearance_mm": 0.0}), ("スコップ μ 0.0", {"scoop_mu": 0.0}), ("スコップ μ 1.0", {"scoop_mu": 1.0}),
             ("床 mat（μ 0.6）", {"floor": "mat"})]
    for name, kw in tests:
        b = {**base, **kw}
        r = run_episode(cfg, b["shape"], b["obj"], b["floor"], b["speed_mm_s"], b["y_offset_mm"], b["seed"],
                        **{k: v for k, v in kw.items() if k in ("clearance_mm", "scoop_mu")})
        print(f"  {name:22s}: {r.outcome:13s} 物の前進 {r.forward_disp_mm:6.1f} mm  先端より前 {r.ahead_of_tip_mm:5.1f} mm")
    print("== 3. 床の摩擦を極端に上げる（物 – 床 μ 3.0）と、物は先端に押されるのをやめるか")
    cfg2 = load_config()
    cfg2["floors"]["sticky"] = {"friction": 3.0}
    r = run_episode(cfg2, sh, "coin_1yen", "sticky", 20.0, 0.0, 1)
    print(f"  μ 3.0: {r.outcome} 物の前進 {r.forward_disp_mm:.1f} mm")
    print("== 4. とても遅い前進（2 mm/s）")
    r = run_episode(cfg, sh, "coin_1yen", "flooring", 2.0, 0.0, 1)
    print(f"  2 mm/s: {r.outcome} 物の前進 {r.forward_disp_mm:.1f} mm")


if __name__ == "__main__":
    main()
