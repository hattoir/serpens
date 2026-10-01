"""探索（成立性調査）: コインの縁の丸み（0 / 0.1 / 0.3 / 0.5 mm）× 形 × 床 × 速度。

    .\.venv\Scripts\python.exe tools\scoop_explore_fillet.py

結果は simulation/results/scoop_exploration_2026-09-29.md。MUJOCO_SIM。実物の試験片の結果が出るまで「候補」。
"""
import sys
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from simulation.scoop.model import Shape, load_config  # noqa: E402
from simulation.scoop.runner import run_episode  # noqa: E402

CFG = None


def job(args):
    global CFG
    if CFG is None:
        CFG = load_config()
    shape, obj, floor, speed, fillet, seed, mu_s = args
    r = run_episode(CFG, shape, obj, floor, speed, 0.0, seed, rim_fillet_mm=fillet, scoop_mu=mu_s)
    return (shape.key, obj, floor, speed, fillet, mu_s, r.outcome, r.forward_disp_mm)


def main() -> None:
    shapes = [Shape(0.4, 8.0, False, 30.0), Shape(0.6, 12.0, False, 30.0), Shape(1.0, 18.0, False, 30.0)]
    jobs = [(s, "coin_1yen", f, v, fil, seed, None) for s in shapes for f in ("flooring", "mat") for v in (10.0, 40.0)
            for fil in (0.0, 0.1, 0.3, 0.5) for seed in range(6)]
    with ProcessPoolExecutor(max_workers=14) as ex:
        res = list(ex.map(job, jobs, chunksize=4))
    tab = {}
    for key, obj, floor, speed, fil, mu_s, out, fwd in res:
        tab.setdefault((fil, floor), Counter())[out] += 1
    print("rim fillet mm / floor -> outcomes (6 trials x 3 shapes x 2 speeds = 36 each)")
    for k in sorted(tab):
        print(" ", k, dict(tab[k]))
    print("by shape (fillet 0.3, mat):")
    for s in shapes:
        c = Counter(out for key, obj, floor, speed, fil, mu_s, out, fwd in res if key == s.key and fil == 0.3 and floor == "mat")
        print(" ", s.key, dict(c))


if __name__ == "__main__":
    main()
