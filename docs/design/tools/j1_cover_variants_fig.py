"""覆いの案（なし / A / B / C）を θ_E −4°（CAD +4°）で比べる断面図（y = 0 と y = 20）。危険なすき間 = 青、覆い = ピンク。
先頭の注意: KNUCKLE DRUM 後も、まっすぐ比で約 20 倍（18〜24 倍）のすき間が残る。背板は現行 CAD では未解決のまま。5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）。安全・合格の語は使わない。数値は CAD_CONCEPT。
使い方: python j1_cover_variants_fig.py <出力 png>
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import gap_check as g
plt.rcParams["font.family"] = ["Yu Gothic", "Meiryo", "sans-serif"]
SCR = Path("C:/Users/Public/serpens_gapcheck"); TF = SCR / "transforms_r003.json"; L = SCR / "local3"
BOX = (-250.0, -110.0, -60.0, 60.0, 0.0, 100.0); VIEW = (-160.0, -118.0, 36.0, 64.0)
covers = {"覆いなし": None, "案 A（8° の殻）": SCR / "j1v2_out/J1v2_COVER_A_10.stl", "案 B（5° の短い殻）": SCR / "j1v2_out/J1v2_COVER_B_11.stl", "案 C（丸い襟巻き）": SCR / "j1v2_out/J1v2_COVER_C_12.stl"}
fig, axs = plt.subplots(2, 4, figsize=(24, 12))
pose = "p20"
for c, (name, stl) in enumerate(covers.items()):
    extra = [g.read_stl(stl)] if stl else []
    r = g.analyse_crevices(L, pose, TF, BOX, cell=0.5, extra_meshes=extra, return_mask=True); haz = r["_haz"]
    g.X0, g.X1, g.Y0, g.Y1, g.Z0, g.Z1 = BOX; g.DX = g.DY = g.DZ = 0.5
    names = sorted(L.glob("*.stl")); ms = g.load_posed(L, pose, TF)
    grp = lambda pre: g.occupancy([m for m, n in zip(ms, names) if n.stem.startswith(pre)])
    head, neck = grp("HEADV"), grp("NECKV")
    cov = g.occupancy(extra) if extra else np.zeros_like(head)
    for row, yv in enumerate((0, 20)):
        j = int((yv - BOX[2]) / 0.5)
        img = np.ones(head[:, j, :].shape + (3,)); img[neck[:, j, :]] = (0.62, 0.78, 0.60); img[head[:, j, :]] = (0.98, 0.83, 0.55); img[cov[:, j, :]] = (0.95, 0.35, 0.55); img[haz[:, j, :]] = (0.15, 0.35, 0.95)
        ax = axs[row, c]; ax.imshow(img, origin="lower", extent=(BOX[0], BOX[1], BOX[4], BOX[5]), aspect="equal"); ax.set_xlim(VIEW[0], VIEW[1]); ax.set_ylim(VIEW[2], VIEW[3])
        ax.set_title(f"{name}  y={yv} mm" + (f"  危険 {r['hazard_mm3']} mm³" if row == 0 else ""), fontsize=13)
    print(name, r["hazard_mm3"], flush=True)
fig.suptitle("J1 の上の輪: θ_E −4°（頭を下げる端。CAD の姿勢 +4°）。橙 = 頭、緑 = 首、ピンク = 覆い、青 = 届く危険なすき間（幅 5〜12.5 mm・深さ 8 mm 以上）\n先頭の注意: KNUCKLE DRUM 後も、まっすぐ比で約 20 倍のすき間が残る。背板は未解決。5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）。安全・合格の語は使わない。CAD_CONCEPT", fontsize=12)
fig.tight_layout(rect=(0, 0, 1, 0.93)); fig.savefig(sys.argv[1], dpi=55)
