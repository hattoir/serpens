"""探索（成立性調査）: 傾斜板に乗ったコインが、その後どこに落ち着くか。

    .\.venv\Scripts\python.exe tools\scoop_explore_ramp_rest.py

結果は simulation/results/scoop_exploration_2026-09-29.md。MUJOCO_SIM。実物の試験片の結果が出るまで「候補」。
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from simulation.scoop.model import Shape, load_config
from simulation.scoop.runner import run_episode

cfg = load_config()
for mu in (0.3, 0.6):
    for speed in (10.0, 40.0):
        for seed in range(3):
            r = run_episode(cfg, Shape(0.05, 8.0, False, 30.0), "coin_1yen", "mat", speed, 0.0, seed, rim_fillet_mm=0.3,
                            clearance_mm=0.0, front_face="vertical", scoop_mu=mu)
            print(f"scoop_mu {mu} v {speed:4.0f} seed {seed}: {r.outcome:12s} rel_x {r.rel_x_mm:6.1f} mm (傾斜板の上端 29.7、空間は 29.7〜59.7)  trig {r.triggered}  fwd {r.forward_disp_mm:.1f}")
