"""覆いの案（A/B/C）ごとに、θ_E の姿勢での危険体積と、覆いと頭・フードの重なり（mm³）を計算する。R-003 / ENTRY-D-0012。
使い方: python j1_cover_variants_eval.py <出力json> <案名> <cover.stl>
先頭の注意: KNUCKLE DRUM 後も、まっすぐ比で約 20 倍のすき間が残る。背板は現行 CAD では未解決のまま。5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）。安全・合格の語は使わない。数値は CAD_CONCEPT。
姿勢: p20 = θ_E −4°、p15 = −3°、p14 = −2°、p16 = −5°、p0 = 0°、p21 = +4°（頭を上げる。覆いは下側に効かない）。
"""
import sys, json
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import numpy as np
import gap_check as g
SCR = Path("C:/Users/Public/serpens_gapcheck"); TF = SCR / "transforms_r003.json"; L = SCR / "local3"
BOX = (-250.0, -110.0, -60.0, 60.0, 0.0, 100.0)
POSES = {"p20": -4, "p15": -3, "p14": -2, "p16": -5, "p0": 0, "p21": 4}
out, name, stl = sys.argv[1], sys.argv[2], Path(sys.argv[3])
cov = g.read_stl(stl)
g.X0, g.X1, g.Y0, g.Y1, g.Z0, g.Z1 = BOX; g.DX = g.DY = g.DZ = 0.5
covocc = g.occupancy([cov]); vol = float(covocc.sum() * 0.125)
names = sorted(L.glob("*.stl")); res = {"cover": name, "volume_mm3": round(vol), "poses": {}}
for p, te in POSES.items():
    ms = g.load_posed(L, p, TF)
    head = g.occupancy([m for m, n in zip(ms, names) if n.stem.startswith("HEADV")])
    ov = float((covocc & head).sum() * 0.125)
    h0 = g.analyse_crevices(L, p, TF, BOX, cell=0.5)["hazard_mm3"] if name == "A" else None
    h1 = g.analyse_crevices(L, p, TF, BOX, cell=0.5, extra_meshes=[cov])["hazard_mm3"]
    res["poses"][p] = {"theta_E": te, "hazard_with_cover": h1, "cover_head_overlap_mm3": round(ov, 1), **({"hazard_without": h0} if h0 is not None else {})}
    print(name, p, te, res["poses"][p], flush=True)
Path(out).write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")
