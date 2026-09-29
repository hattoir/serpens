"""探索（成立性調査）: 薄い先端（0.02〜0.4 mm）× 縁の丸み × スコップと物の摩擦。物理的に成功がありうる範囲を探す。

    .\.venv\Scripts\python.exe tools\scoop_explore_thintip.py

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


def job(a):
    global CFG
    if CFG is None:
        CFG = load_config()
    tip, fillet, mu_s, floor, obj, seed = a
    r = run_episode(CFG, Shape(tip, 8.0, False, 30.0), obj, floor, 20.0, 0.0, seed, rim_fillet_mm=fillet, clearance_mm=0.0,
                    front_face="vertical", scoop_mu=mu_s)
    return (tip, fillet, mu_s, floor, obj, r.outcome)


def main():
    jobs = [(tip, fil, mu, fl, "coin_1yen", s) for tip in (0.02, 0.05, 0.1, 0.2, 0.4) for fil in (0.3, 0.5) for mu in (0.3, 0.1, 0.03)
            for fl in ("flooring", "mat") for s in range(4)]
    with ProcessPoolExecutor(max_workers=14) as ex:
        res = list(ex.map(job, jobs, chunksize=4))
    print(len(res), "episodes; any success:", any(r[5] == "success" for r in res))
    for tip in (0.02, 0.05, 0.1, 0.2, 0.4):
        for fil in (0.3, 0.5):
            for mu in (0.3, 0.1, 0.03):
                c = Counter(r[5] for r in res if r[0] == tip and r[1] == fil and r[2] == mu)
                if set(c) != {"pushed_ahead"}:
                    print(f"  tip {tip} fillet {fil} mu_scoop {mu}: {dict(c)}")


if __name__ == "__main__":
    main()
