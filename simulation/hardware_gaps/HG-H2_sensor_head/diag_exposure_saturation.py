"""LB-E-080: HG-H2 の exposure_gain=2.5 で欠落する button_lr44 の原因を切り分ける（診断。run.py は変更しない）。"""
import sys
sys.stdout.reconfigure(encoding="utf-8")
W = str(__import__("pathlib").Path(__file__).resolve().parents[3])
sys.path.insert(0, W)
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent))
import numpy as np
import run as R
from serpens.floorwatch.detect import detect, MotionError  # noqa

A = R.A
nom = dict(A["nominal"])
base = R.load_config()
for gain in (1.0, 1.5, 2.0, 2.5):
    p = {**nom, "exposure_gain": gain}
    cfg = R.cfg_for(base, p)
    cam = R.camera(p)
    plane = R.LightPlane.design(cfg)
    rend = R.Renderer(cam, plane, R.lighting(p))
    deg = R.Degrader(cam, rend, p)
    rng = np.random.default_rng(int(R.SW["seed"]))
    for name, kind, positive, sc, x, y in R.scenes(p):
        raw = rend.render(sc)
        frames = {k: deg(v, rng) for k, v in raw.items()}
        if name != "button_lr44":
            continue
        sat = {k: float((v >= 255).mean()) for k, v in frames.items()}
        cands, _, _ = detect(frames, cam, plane, cfg, with_line=True)
        objs = [c for c in cands if c.is_object]
        print(f"gain={gain} x={x:+.1f} kind={kind} sat_frac={ {k: round(s,3) for k,s in sat.items()} } nobj={len(objs)} hit={R._nearest(objs, x, y) is not None}",
              [round(c.diameter_mm, 1) for c in objs][:3])

