r"""模擬 Vision の閉ループを故障ごとに走らせて、真値との差と停止の様子を表にする（Phase 4）。

**source = KINEMATIC_SIM。** 画像は仮想カメラ、検出は本物の ArUco。
実カメラの性能ではない。実カメラが来たら同じ表を実測で作り直す。

    .\.venv\Scripts\python.exe tools\vision_check.py
    .\.venv\Scripts\python.exe tools\vision_check.py --seconds 20 --only clean,blackout
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from serpens.config import load_config  # noqa: E402
from serpens.sim.sim_vision import VisionFaults, Window  # noqa: E402
from simulation.virtual_person import scenario  # noqa: E402
from simulation.vision_loop import VisionLoop  # noqa: E402

FAR_GHOST = (100.0, 1100.0)
NEAR_GHOST = (650.0, 250.0)


def cases() -> dict[str, VisionFaults]:
    """故障の一覧（5〜8 秒の間に起こす）。"""
    w = Window(5.0, 8.0)
    return {
        "clean": VisionFaults(),
        "pixel_noise": VisionFaults(pixel_noise_std=12.0),
        "dropout_30%": VisionFaults(dropout_ratio=0.3),
        "occlude_neck": VisionFaults(occlude=[(w, "neck")]),
        "occlude_tail": VisionFaults(occlude=[(w, "tail")]),
        "blackout": VisionFaults(blackout=[w]),
        "ghost_neck(dup)": VisionFaults(ghost_marker=[(w, "neck", FAR_GHOST)]),
        "hidden+far_ghost": VisionFaults(occlude=[(w, "neck")], ghost_marker=[(w, "neck", FAR_GHOST)]),
        "hidden+near_ghost": VisionFaults(occlude=[(w, "neck")], ghost_marker=[(w, "neck", NEAR_GHOST)]),
        "ghost_person": VisionFaults(ghost_person=[(Window(2.0, 30.0), (300.0, -300.0))]),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="模擬 Vision の閉ループ試験")
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--seed", type=int, default=5)
    ap.add_argument("--only", default="", help="カンマ区切りで絞る")
    args = ap.parse_args()
    cfg = load_config()
    only = {s for s in args.only.split(",") if s}
    print("source=KINEMATIC_SIM（仮想カメラ画像 + 本物の ArUco 検出。**実カメラの値ではない**）")
    print(f"{'case':18s} 走行mm 位置p95 位置max 向きp95 古さmax 人p95  停止 飛び棄却 停止後の惰性mm")
    for name, faults in cases().items():
        if only and name not in only:
            continue
        t0 = time.perf_counter()
        loop = VisionLoop(cfg, faults, seed=args.seed, people=scenario(cfg))
        if not loop.start_when_seen():
            print(f"{name:18s} 自己位置が入らず開始できない")
            continue
        s = loop.run(args.seconds).summary()
        print(f"{name:18s} {s['travelled_mm']:6.0f} {s['pos_err_p95_mm']:7.1f} {s['pos_err_max_mm']:7.1f} "
              f"{s['heading_err_p95_deg']:7.1f} {s['age_max_s']:7.2f} {s['target_err_p95_mm']:5.0f} "
              f"{int(s['stops']):4d} {loop.session.snake_tracker.rejected_jumps:8d} "
              f"{s['moved_while_stale_mm']:8.1f}   ({time.perf_counter() - t0:.0f}s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
