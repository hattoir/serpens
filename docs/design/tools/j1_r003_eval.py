"""R-003 (c): J1 の危険体積を θ_E = ±4° で計算する（Design の計算方法 = gap_check.analyse_crevices、0.5 mm、幅 5〜12.5・深さ 8 以上、箱 x −250〜−110 / |y| ≤ 60）。
姿勢: p20 = CAD +4°（θ_E −4°、頭を下げる）、p21 = CAD −4°（θ_E +4°、頭を上げる）。transforms_r003.json は transforms.json に p20/p21（y 軸まわりの回転、軸 (−181.8, z 32.4)）を足したもの。
使い方: python j1_r003_eval.py <出力json> <pose,pose,...> [cover.stl ...]   （cover は首側に固定 = 姿勢の変換を掛けない）
先頭の注意: KNUCKLE DRUM 後も、まっすぐ比で約 20 倍のすき間が残る。背板は現行 CAD では未解決のまま。5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）。安全・合格の語は使わない。数値は CAD_CONCEPT。
"""
import sys, json
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import gap_check as g
SCR = Path("C:/Users/Public/serpens_gapcheck"); TF = SCR / "transforms_r003.json"; L = SCR / "local3"
BOX = (-250.0, -110.0, -60.0, 60.0, 0.0, 100.0)
out, poses, covers = sys.argv[1], sys.argv[2].split(","), [g.read_stl(Path(c)) for c in sys.argv[3:]]
res = {}
for p in poses:
    r = g.analyse_crevices(L, p, TF, BOX, cell=0.5, extra_meshes=covers)
    res[p] = {"hazard_mm3": r["hazard_mm3"], "clusters": r["hazard_clusters"][:3]}
    print(p, r["hazard_mm3"], flush=True)
Path(out).write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")
