"""J1 ピッチ（あご・首の前）の姿勢ごとの、幅 5〜12 mm のすき間の r の範囲（J1 の軸 (−181.8, 32.4) からの XZ の距離）。
姿勢 p0（0°）、p7（−5°）、p6（−10°）、p5（−22°）、p4（−45°）、p8（+10°）、p9（+22°）。胴はまっすぐ。
使い方: python gap_pitch.py <folder> <出力の接頭辞> <pose...>
"""
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
import gap_check as g

TF = Path("C:/Users/Public/serpens_gapcheck/transforms.json")
OUT = Path(__file__).resolve().parents[1] / "results"
J1 = (-181.8, 32.4)
ANG = {"p0": 0, "p7": -5, "p6": -10, "p5": -22, "p4": -45, "p8": 10, "p9": 22, "p10": -1, "p11": -2, "p12": -3, "p13": 1, "p14": 2, "p15": 3, "p16": 5, "p17": 7.5}
folder, prefix, poses = Path(sys.argv[1]), sys.argv[2], sys.argv[3:]
rows = {}
for pose in poses:
    box = (-250.0, -110.0, -60.0, 60.0, 0.0, 100.0)
    r = g.analyse_crevices(folder, pose, TF, box, cell=0.5, return_mask=True)
    haz, W = r["_haz"], r["_W"]; idx = np.argwhere(haz)
    row = {"j1v_deg": ANG.get(pose), "hazard_mm3": r["hazard_mm3"], "clusters": []}
    if len(idx):
        z = box[4] + (idx[:, 0] + .5) * 0.5; x = box[0] + (idx[:, 2] + .5) * 0.5; y = box[2] + (idx[:, 1] + .5) * 0.5
        rr = np.hypot(x - J1[0], z - J1[1]); w = W[tuple(idx.T)]
        ang = np.degrees(np.arctan2(z - J1[1], x - J1[0]))
        row.update({"r_mm": [round(float(rr.min()), 1), round(float(rr.max()), 1)], "r_p5_p95_mm": [round(float(np.percentile(rr, 5)), 1), round(float(np.percentile(rr, 95)), 1)],
                    "width_mm": [round(float(w.min()), 1), round(float(w.max()), 1)], "y_mm": [round(float(y.min()), 1), round(float(y.max()), 1)], "z_mm": [round(float(z.min()), 1), round(float(z.max()), 1)],
                    "angle_deg_from_axis_xz": [round(float(ang.min()), 0), round(float(ang.max()), 0)]})
        row["clusters"] = r["hazard_clusters"][:4]
    rows[pose] = row
    print(prefix, pose, row["j1v_deg"], row["hazard_mm3"], row.get("r_mm"), row.get("angle_deg_from_axis_xz"), flush=True)
(OUT / f"{prefix}.json").write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8")
