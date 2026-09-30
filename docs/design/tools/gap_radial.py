"""幅 5〜12 mm を通る「すき間」を、関節ごと・姿勢ごとに、軸からの距離 r の範囲で出す（gap_check.analyse_crevices の hazard を使う）。

ヨー軸（鉛直）: J2 (−95,0)、J3 (0,0)、J4 (95,0)、J5 (190,0)（中立、CAD `robot_fw*` の CONCEPT 値）。姿勢ごとの位置は transforms.json の
親側のリンクの変換で動かす。頭は J1 ピッチ軸 (−181.8, y, z32.4)。r はヨー軸なら XY 平面、J1 なら XZ 平面で測る。
実行: python gap_radial.py <folder> <出力の接頭辞> <pose...>   （カバーなしの p0..p5 なら folder=local2）
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gap_check as g  # noqa: E402

TF = Path("C:/Users/Public/serpens_gapcheck/transforms.json")
OUT = Path(__file__).resolve().parents[1] / "results"
XSEG = [("head_neck", -250, -110), ("J2_J3", -110, 75), ("J4", 75, 170), ("J5_tail", 170, 360)]   # 区切りは節の真ん中（フラップの空洞を箱で切らない）
NEUTRAL = {"J2": (-95.0, 0.0), "J3": (0.0, 0.0), "J4": (95.0, 0.0), "J5": (190.0, 0.0)}
CARRIER = {"J2": "LINK1", "J3": "LINK1", "J4": "LINK3", "J5": "LINK3"}   # 軸の位置を運ぶリンク（動かない側から見て軸がある側）


def axes_for(pose: str) -> dict:
    tf = json.loads(TF.read_text())[pose]
    out = {}
    for j, (x, y) in NEUTRAL.items():
        m = np.array(tf[CARRIER[j]]).reshape(4, 4)
        p = m[:3, :3] @ np.array([x, y, 0.0]) + m[:3, 3] * 10.0
        out[j] = (float(p[0]), float(p[1]))
    m = np.array(tf["NECKV"]).reshape(4, 4)
    p = m[:3, :3] @ np.array([-181.8, 0.0, 32.4]) + m[:3, 3] * 10.0
    out["J1"] = (float(p[0]), float(p[1]), float(p[2]))          # x, y, z。ピッチ軸は頭の向き（ここでは近似で y 方向）
    return out


def radial(res: dict, axes: dict) -> list[dict]:
    haz, W, depth = res["_haz"], res["_W"], res["_depth"]
    box, cell = res["box"], res["cell_mm"]
    idx = np.argwhere(haz)
    if len(idx) == 0:
        return []
    z = box[4] + (idx[:, 0] + 0.5) * cell; y = box[2] + (idx[:, 1] + 0.5) * cell; x = box[0] + (idx[:, 2] + 0.5) * cell
    names = ["J2", "J3", "J4", "J5"]
    d = np.stack([np.hypot(x - axes[n][0], y - axes[n][1]) for n in names])
    j1 = axes["J1"]
    d1 = np.sqrt((x - j1[0]) ** 2 + (y - j1[1]) ** 2 + (z - j1[2]) ** 2)
    near = np.argmin(d, axis=0)
    label = np.array([names[i] for i in near])
    r = d[near, np.arange(len(x))]
    head = d1 < r                                   # J1 の 3D の距離のほうが近い所は J1（あご・首の前）
    label[head] = "J1"
    r[head] = np.hypot(x[head] - j1[0], z[head] - j1[2])          # 側面（XZ）の距離
    w = W[tuple(idx.T)]
    rows = []
    for n in ["J1"] + names:
        m = label == n
        if not m.any():
            continue
        zz = z[m]
        rows.append({"joint": n, "volume_mm3": round(float(m.sum()) * cell ** 3), "r_mm": [round(float(r[m].min()), 1), round(float(r[m].max()), 1)],
                     "r_p5_p95_mm": [round(float(np.percentile(r[m], 5)), 1), round(float(np.percentile(r[m], 95)), 1)],
                     "volume_z_lt77_mm3": round(float((zz < 77).sum()) * cell ** 3), "width_mm": [round(float(w[m].min()), 1), round(float(w[m].max()), 1)], "z_mm": [round(float(z[m].min()), 1), round(float(z[m].max()), 1)],
                     "axis_position_mm": [round(v, 1) for v in axes[n]]})
    return rows


def run(folder: Path, prefix: str, poses: list[str], extra=None) -> None:
    for pose in poses:
        axes = axes_for(pose)
        yh = 55.0 if pose == "p0" else 170.0
        rows = {}
        for name, x0, x1 in XSEG:
            box = (float(x0), float(x1), -yh, yh, 0.0, 100.0)
            ex = extra(pose) if extra else None
            res = g.analyse_crevices(folder, pose, TF, box, cell=0.5, extra_meshes=ex, return_mask=True)
            rows[name] = {"hazard_mm3": res["hazard_mm3"], "by_joint": radial(res, axes)}
            print(prefix, pose, name, res["hazard_mm3"], [(b["joint"], b["volume_mm3"], b["r_mm"]) for b in rows[name]["by_joint"]], flush=True)
        (OUT / f"{prefix}_{pose}.json").write_text(json.dumps({"pose": pose, "axes": axes, "zones": rows}, ensure_ascii=False, indent=1), encoding="utf-8")


if __name__ == "__main__":
    run(Path(sys.argv[1]), sys.argv[2], sys.argv[3:])
