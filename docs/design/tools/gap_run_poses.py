"""gap_check.analyse_crevices を姿勢ごとに回す（x を 4 つに区切る）。

使い方: python gap_run_poses.py <folder> <出力の接頭辞> <pose...>
  folder: 部品ごとの STL（`KEY__NN.stl`）のあるフォルダ。transforms.json は C:/Users/Public/serpens_gapcheck/
  p0（まっすぐ）は y ±55、曲げた姿勢は y ±170 の箱で調べる（0.5 mm）。
"""
import json, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import gap_check as g

TF = Path("C:/Users/Public/serpens_gapcheck/transforms.json")
OUT = Path(__file__).resolve().parents[1] / "results"
XSEG = [("head_neck", -250, -90), ("J2_J3", -100, 20), ("J3_J4_J5", 10, 150), ("J5_tail", 140, 360)]
loc = Path(sys.argv[1]); prefix = sys.argv[2]
for pose in sys.argv[3:]:
    res = {}
    yh = 55.0 if pose == "p0" else 170.0
    for name, x0, x1 in XSEG:
        box = (float(x0), float(x1), -yh, yh, 0.0, 100.0)
        res[name] = g.analyse_crevices(loc, pose, TF, box, cell=0.5)
        print(prefix, pose, name, res[name]["hazard_mm3"], [(c["x_mm"], c["y_mm"], c["z_mm"], c["volume_mm3"]) for c in res[name]["hazard_clusters"][:4]], flush=True)
    (OUT / f"{prefix}_{pose}.json").write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")
