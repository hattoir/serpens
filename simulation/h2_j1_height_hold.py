r"""H2 VIS-0009: 頭が毛足に沈んだとき、J1 の鼻上げでカメラの高さを戻すと床見はどうなるか。**SYNTHETIC_VISION_SIM。**

    .\.venv\Scripts\python.exe simulation\h2_j1_height_hold.py  → simulation/results/h2_j1_height_hold.md

レンズは J1 の軸の 50.2mm 前・2.35mm 下（Design integration-log ENTRY-0022、CAD_CONCEPT）。沈み s をレンズの高さで打ち消す
鼻上げ θ を解き、「沈んだまま」と「J1 で高さを戻した（代わりに下向きが θ 減る）」を、点光源の照明・照明の較正画像（名目の姿勢）・
狙い直しありで比べる。LED もカメラも頭に固定なので一緒に動く（simulation/h2_vision.py の rigid_point）。
"""
from __future__ import annotations

import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

LENS_AHEAD_MM, LENS_BELOW_MM = 50.2, 2.35
SINKS_MM = (2.0, 4.0, 6.0)
_C: dict = {}


def j1_for_sink(sink_mm: float) -> tuple[float, float]:
    """沈み sink_mm をレンズの高さで打ち消す鼻上げ θ [deg] と、そのときのレンズの前後のずれ [mm]。"""
    f, d = LENS_AHEAD_MM, LENS_BELOW_MM
    lo, hi = 0.0, 45.0
    for _ in range(60):
        th = (lo + hi) / 2
        t = math.radians(th)
        dz = f * math.sin(t) - d * math.cos(t) + d
        lo, hi = (th, hi) if dz < sink_mm else (lo, th)
    t = math.radians(th)
    return th, f * math.cos(t) + d * math.sin(t) - f


def _init() -> None:
    from serpens.config import load_config
    from serpens.floorwatch.geometry import Camera, LightPlane
    from serpens.floorwatch.synthetic import default_lighting
    from simulation.h2_vision import scaled_camera
    cfg = load_config()
    _C.update(cfg=cfg, cam=scaled_camera(Camera.from_cfg(cfg), 0.5), plane=LightPlane.design(cfg), lt=default_lighting(cfg))


def _job(a: tuple) -> tuple:
    from simulation.h2_vision import Condition, run_trial
    mode, sink, tg, seed = a
    base = dict(physical_falloff=True, flat_field=True, auto_exposure=True, reaim=True)
    if mode == "sink":
        c = Condition(cam_height_err_mm=-sink, **base)
    else:
        c = Condition(cam_height_err_mm=0.0, cam_pitch_err_deg=-j1_for_sink(sink)[0], **base)   # 鼻上げ = 下向きが減る
    t = run_trial(_C["cfg"], _C["cam"], _C["plane"], _C["lt"], c, tg, seed, blur_scale=0.5, negative="empty")
    return mode, sink, tg, t.stage_hits["patrol"], t.stage_hits["inspect"], t.critical_flag, t.false_objects["inspect"]


def main() -> int:
    from simulation.h2_vision import TARGETS
    jobs = [(m, s, tg, 5000 + 37 * i + k) for m in ("sink", "j1") for s in SINKS_MM for i, tg in enumerate(TARGETS) for k in range(3)]
    jobs += [(m, s, None, 7000 + k) for m in ("sink", "j1") for s in SINKS_MM for k in range(6)]
    with ProcessPoolExecutor(12, initializer=_init) as ex:
        res = list(ex.map(_job, jobs, chunksize=2))
    out = ["# J1 で頭の高さを保つ（VIS-0009、SYNTHETIC_VISION_SIM。対象 11 種 × 3 回 + 何も無い床 6 回 / 条件）", ""]
    out += [f"- 沈み {s:g}mm → J1 鼻上げ {j1_for_sink(s)[0]:.2f}°" for s in (2.0, 4.0, 6.0, 8.0)]
    out += ["", "| 沈み mm | 対策 | 巡回 | 停止 | 危険物 | metal_disc（鏡面の危険物）| 誤報 停止 |", "|---|---|---|---|---|---|---|"]
    for s in SINKS_MM:
        for m in ("sink", "j1"):
            pos = [r for r in res if r[0] == m and r[1] == s and r[2] is not None]
            neg = [r for r in res if r[0] == m and r[1] == s and r[2] is None]
            crit = [r for r in pos if TARGETS[r[2]][4]]
            spec = [r for r in pos if TARGETS[r[2]][3] and TARGETS[r[2]][4]]
            out.append(f"| {s:g} | {'なし' if m == 'sink' else 'J1 で高さを戻す'} | {np.mean([r[3] for r in pos]):.2f} | "
                       f"{np.mean([r[4] for r in pos]):.2f} | {np.mean([r[4] for r in crit]):.2f} | {np.mean([r[5] for r in spec]):.2f} | "
                       f"{np.mean([r[6] > 0 for r in neg]):.2f} |")
    p = ROOT / "simulation" / "results" / "h2_j1_height_hold.md"
    p.write_text("\n".join(out) + "\n", encoding="utf-8")
    print("\n".join(out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
