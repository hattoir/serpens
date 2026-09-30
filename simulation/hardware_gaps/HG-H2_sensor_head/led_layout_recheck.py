"""Design の斜め LED 移設（ENTRY-D-0008 (4)）の追試。SYNTHETIC_VISION_SIM（合成画像。実写ではない）。

Design は同じ種（9000〜）で 1 条件 = 対象 11 種 × 8 回 + 陰性 3 種 × 8 回を回した。ここでは **別の種（既定 40000〜）と、多めの繰り返し（既定 16）**で、
Engineering の `simulation/h2_vision.py` の同じ経路（`run_trial`）を回して、結果が保たれるかを見る（追試）。
フード（漏斗の壁）の遮りは HG-H2 のモデルに無い。斜め LED の明るさを 0.5 / 0.25 倍にした感度で「一部が遮られても保てるか」を見る。

    python simulation/hardware_gaps/HG-H2_sensor_head/led_layout_recheck.py --reps 16 --seed 40000
"""
from __future__ import annotations

import argparse
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

CASES = {
    "before_design_2x2": ("design_2x2", 1.0),
    "front_corner": ("front_corner", 1.0),
    "runner_front_z6": ("runner_front", 1.0),
    "runner_front_z4": ("runner_front_z4", 1.0),
    "runner_front_z9": ("runner_front_z9", 1.0),
    "runner_front_z6_raking_x0.5": ("runner_front", 0.5),
    "runner_front_z6_raking_x0.25": ("runner_front", 0.25),
    "current_pos_with_hood_approx_x0.12": ("design_2x2", 0.12),
}
_C: dict = {}


def _init() -> None:
    from serpens.config import load_config
    from serpens.floorwatch.geometry import Camera, LightPlane
    from serpens.floorwatch.synthetic import default_lighting
    import simulation.h2_vision as hv
    cfg = load_config()
    _C.update(cfg=cfg, cam=hv.scaled_camera(Camera.from_cfg(cfg), 0.5), plane=LightPlane.design(cfg), lt=default_lighting(cfg), hv=hv)


def _job(a):
    case, target, negative, seed = a
    hv = _C["hv"]
    layout, scale = CASES[case]
    c = hv.Condition(physical_falloff=True, flat_field=True, auto_exposure=True, led_layout=layout)
    lt = replace(_C["lt"], raking_lux=_C["lt"].raking_lux * scale)
    return case, hv.run_trial(_C["cfg"], _C["cam"], _C["plane"], lt, c, target, seed, blur_scale=0.5, negative=negative)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--reps", type=int, default=16)
    ap.add_argument("--seed", type=int, default=40000)
    ap.add_argument("--workers", type=int, default=8)
    a = ap.parse_args()
    import simulation.h2_vision as hv
    jobs = []
    for case in CASES:
        k = a.seed                                        # 全条件で同じ種の並び（条件間の差を、乱数でなく条件から出す）
        for _ in range(a.reps):
            for tg in hv.TARGETS:
                jobs.append((case, tg, "", k)); k += 1
            for ng in ("empty", "stain", "seam"):
                jobs.append((case, None, ng, k)); k += 1
    with ProcessPoolExecutor(max_workers=a.workers, initializer=_init) as ex:
        res = list(ex.map(_job, jobs, chunksize=4))
    out = {case: hv.summarize([t for c, t in res if c == case]) for case in CASES}
    out["_meta"] = {"reps": a.reps, "seed": a.seed, "n_targets": len(hv.TARGETS), "label": "SYNTHETIC_VISION_SIM"}
    p = Path(__file__).parent / "results" / "led_layout_recheck.json"
    p.parent.mkdir(exist_ok=True)
    p.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    for k, v in out.items():
        if k != "_meta":
            print(f"{k:38s}", {kk: round(vv, 2) if isinstance(vv, float) else vv for kk, vv in v.items() if kk in ("patrol_recall", "inspect_recall", "critical_inspect_recall", "specular_critical_flagged", "false_alarm_patrol", "diameter_rel_err_median")})


if __name__ == "__main__":
    main()
