"""HG-H2（合成視覚）で、斜め LED を口の前の外側の角へ移した案を、移設前後で比べる（ENTRY-D-0008）。SYNTHETIC_VISION_SIM。

Engineering の `simulation/h2_vision.py`（agent/engineering-vision-sim のスナップショット。読み出しのみ、変更しない）を import し、
LED_LAYOUTS に条件を足して run_trial を回す。Engineering のファイルは変えない。
座標は h2_vision の世界座標（x 右 / y 前 / z 上、カメラ (0, 0, 30) = Design の X −232、Z 30）。Design の X → y = −232 − X（前が +）。
  移設前 design_2x2: 斜め (±40, −20, 7)（X −212）、通常 (±44.7, −12, 21)
  移設後 front_corner: 斜め (±36, +4.5, 6)（X −236.5、|y| 36、Z 6）、通常は同じ
  front_corner_z4 / z9: 同じ角で LED の高さ 4 / 9 mm
  runner_front: 横のスキッドの前端（Design X −226、|y| 40、Z6）→ (±40, −6, 6)
  現行位置 + フード（近似）: design_2x2 の位置のまま、フードの壁が前の床への光線の 88 % を遮る → 斜めの明るさを 12 % にする近似
使い方: python h2_layout_compare.py <snapshot> [--reps 4] [--workers 8]
"""
from __future__ import annotations

import argparse
import csv
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np

CASES = {
    "before_design_2x2": {"layout": "design_2x2", "raking_scale": 1.0},
    "after_front_corner": {"layout": "front_corner", "raking_scale": 1.0},
    "current_pos_with_hood_approx": {"layout": "design_2x2", "raking_scale": 0.12},
    "after_front_corner_z9": {"layout": "front_corner_z9", "raking_scale": 1.0},
    "after_front_corner_z4": {"layout": "front_corner_z4", "raking_scale": 1.0},
    "after_runner_front": {"layout": "runner_front", "raking_scale": 1.0},
}
_C: dict = {}


def _init(snap: str) -> None:
    sys.path.insert(0, snap)
    import os
    os.chdir(snap)
    from serpens.config import load_config
    from serpens.floorwatch.geometry import Camera, LightPlane
    from serpens.floorwatch.synthetic import default_lighting
    import simulation.h2_vision as hv
    hv.LED_LAYOUTS["front_corner"] = {"raking": ((36.0, 4.5, 6.0), (-36.0, 4.5, 6.0)), "normal": ((44.7, -12.0, 21.0), (-44.7, -12.0, 21.0))}
    hv.LED_LAYOUTS["front_corner_z9"] = {"raking": ((36.0, 4.5, 9.0), (-36.0, 4.5, 9.0)), "normal": ((44.7, -12.0, 21.0), (-44.7, -12.0, 21.0))}
    hv.LED_LAYOUTS["front_corner_z4"] = {"raking": ((36.0, 4.5, 4.0), (-36.0, 4.5, 4.0)), "normal": ((44.7, -12.0, 21.0), (-44.7, -12.0, 21.0))}
    hv.LED_LAYOUTS["runner_front"] = {"raking": ((40.0, -6.0, 6.0), (-40.0, -6.0, 6.0)), "normal": ((44.7, -12.0, 21.0), (-44.7, -12.0, 21.0))}
    cfg = load_config()
    _C.update(cfg=cfg, cam=hv.scaled_camera(Camera.from_cfg(cfg), 0.5), plane=LightPlane.design(cfg), lt=default_lighting(cfg), hv=hv)


def _job(a):
    case, target, negative, seed = a
    hv = _C["hv"]
    spec = CASES[case]
    c = hv.Condition(physical_falloff=True, flat_field=True, auto_exposure=True, led_layout=spec["layout"])
    lt = replace(_C["lt"], raking_lux=_C["lt"].raking_lux * spec["raking_scale"])
    t = hv.run_trial(_C["cfg"], _C["cam"], _C["plane"], lt, c, target, seed, blur_scale=0.5, negative=negative)
    return case, target, negative, t


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("snapshot"); ap.add_argument("--reps", type=int, default=4); ap.add_argument("--workers", type=int, default=8)
    a = ap.parse_args()
    sys.path.insert(0, a.snapshot)
    import os
    os.chdir(a.snapshot)
    import simulation.h2_vision as hv
    negs = ("empty", "stain", "seam")
    jobs = []
    for case in CASES:
        k = 9000
        for rep in range(a.reps):
            for tg in hv.TARGETS:
                jobs.append((case, tg, "", k)); k += 1
            for ng in negs:
                jobs.append((case, None, ng, k)); k += 1
    with ProcessPoolExecutor(max_workers=a.workers, initializer=_init, initargs=(a.snapshot,)) as ex:
        res = list(ex.map(_job, jobs, chunksize=4))
    out = {}
    for case in CASES:
        trials = [r[3] for r in res if r[0] == case]
        out[case] = hv.summarize(trials)
    here = Path(__file__).resolve().parents[1] / "results"
    (here / "h2_layout_compare_2026-09-30.json").write_text(__import__("json").dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    for k, v in out.items():
        print(k, {kk: (round(vv, 2) if isinstance(vv, float) else vv) for kk, vv in v.items()})


if __name__ == "__main__":
    main()
