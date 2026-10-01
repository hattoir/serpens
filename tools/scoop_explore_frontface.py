"""探索（成立性調査）: 先端の前面（板に垂直 / 床に垂直）× 先端厚 0.2〜1.0 × 縁の丸み × 先端の浮き。

    .\.venv\Scripts\python.exe tools\scoop_explore_frontface.py

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
    shape, obj, floor, speed, fillet, clearance, face, seed = args
    r = run_episode(CFG, shape, obj, floor, speed, 0.0, seed, rim_fillet_mm=fillet, clearance_mm=clearance, front_face=face)
    return (shape.tip_mm, shape.alpha_deg, obj, floor, speed, fillet, clearance, face, r.outcome)


def main() -> None:
    jobs = []
    for face in ("perpendicular", "vertical"):
        for tip in (0.2, 0.4, 0.6, 1.0):
            for alpha in (8.0, 18.0):
                for fillet in (0.0, 0.3):
                    for floor in ("flooring", "mat"):
                        for clearance in (0.0, 0.1):
                            for seed in range(4):
                                jobs.append((Shape(tip, alpha, False, 30.0), "coin_1yen", floor, 20.0, fillet, clearance, face, seed))
    with ProcessPoolExecutor(max_workers=14) as ex:
        res = list(ex.map(job, jobs, chunksize=4))
    print(len(res), "episodes")
    for face in ("perpendicular", "vertical"):
        for fillet in (0.0, 0.3):
            for floor in ("flooring", "mat"):
                c = Counter(r[8] for r in res if r[7] == face and r[5] == fillet and r[3] == floor)
                print(f"  front={face:13s} fillet={fillet} floor={floor:8s}: {dict(c)}")
    print("successes by tip thickness / clearance (vertical front, fillet 0.3):")
    for tip in (0.2, 0.4, 0.6, 1.0):
        for cl in (0.0, 0.1):
            for fl in ("flooring", "mat"):
                c = Counter(r[8] for r in res if r[7] == "vertical" and r[5] == 0.3 and r[0] == tip and r[6] == cl and r[3] == fl)
                print(f"  tip {tip} clearance {cl} {fl:8s}: {dict(c)}")


if __name__ == "__main__":
    main()
