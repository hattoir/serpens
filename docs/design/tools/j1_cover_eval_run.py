import sys; sys.path.insert(0,"docs/design/tools")
import numpy as np, json
from pathlib import Path
import j1_cover_eval as J, gap_check as g
cov=g.read_stl(Path("C:/Users/Public/serpens_gapcheck/j1cover_out/J1_COVER_UPPER_0.stl"))
res={}
g.X0,g.X1,g.Y0,g.Y1,g.Z0,g.Z1=J.BOX; g.DX=g.DY=g.DZ=J.C
covocc=g.occupancy([cov]); print("cover vol",covocc.sum()*.125,flush=True)
names=sorted(J.L.glob("*.stl"))
for p in ("p8","p16","p15","p14","p0","p13","p7"):
    ms=g.load_posed(J.L,p,J.TF)
    head=g.occupancy([m for m,n in zip(ms,names) if n.stem.startswith("HEADV")])
    ov=float((covocc&head).sum()*.125)
    r0=g.analyse_crevices(J.L,p,J.TF,J.BOX,cell=0.5)["hazard_mm3"]
    r1=g.analyse_crevices(J.L,p,J.TF,J.BOX,cell=0.5,extra_meshes=[cov])["hazard_mm3"]
    res[p]={"angle_cad":J.ANG[p],"hazard_without":r0,"hazard_with_cover":r1,"cover_head_overlap_mm3":ov}
    print(p,res[p],flush=True)
json.dump(res,open("C:/Users/Public/serpens_gapcheck/j1cover_eval.json","w"))
