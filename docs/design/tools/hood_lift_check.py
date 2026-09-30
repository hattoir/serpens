"""フード昇降（cloche drop）の干渉体積・リンクの数値 — Fusion なしで、STL とボクセルだけで見積もる。

【先頭の注意】KNUCKLE DRUM 後も、まっすぐ比で約 20 倍（18〜24 倍）のすき間が残る。背板は現行 CAD では未解決のまま。
5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）。安全・合格の語は使わない。数値は CAD_CONCEPT / KINEMATIC_SIM。

使い方:  python hood_lift_check.py   （結果は results/hood_lift_2026-09-30.json）
前提: scratch の local3（HEADV__*.stl、頭の外形）と transforms.json（p0）。Engineering の REF 箱は concept_geometry.SENSORS。
フードの形は fusion_hood_b.py と同じ（口 60→30 / 30、空間 30×30×15、壁 1.6、屋根 1.6、足の帯 3.6×4）。頭の座標 x = −236 + 局所 x。
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
from matplotlib.path import Path as MPath

sys.path.insert(0, str(Path(__file__).parent))
import gap_check as g  # noqa: E402
import concept_geometry as cg  # noqa: E402

SCR = Path("C:/Users/Public/serpens_gapcheck")
OUT = Path(__file__).resolve().parents[1] / "results" / "hood_lift_2026-09-30.json"

MOUTH, FUNNEL, CH_L, CH_W, CH_H = 60.0, 30.0, 30.0, 30.0, 15.0
WALL, FOOT, FOOT_H = 1.6, 3.6, 4.0
H_OUT = CH_H + WALL
X_MOUTH = -236.0


def area(P):
    return 0.5 * sum(P[i][0] * P[(i + 1) % len(P)][1] - P[(i + 1) % len(P)][0] * P[i][1] for i in range(len(P)))


def line_off(p, q, d, cw):
    dx, dy = q[0] - p[0], q[1] - p[1]; L = math.hypot(dx, dy); dx /= L; dy /= L
    nx, ny = (-dy, dx) if cw else (dy, -dx)
    return ((p[0] + nx * d, p[1] + ny * d), (dx, dy))


def isect(l1, l2):
    (p, d), (q, e) = l1, l2
    det = d[0] * (-e[1]) - d[1] * (-e[0])
    t = ((q[0] - p[0]) * (-e[1]) - (q[1] - p[1]) * (-e[0])) / det
    return (p[0] + d[0] * t, p[1] + d[1] * t)


def inner_poly():
    s = (MOUTH / 2 - CH_W / 2) / FUNNEL
    x0 = -3.0
    return [(x0, MOUTH / 2 + s * 3.0), (FUNNEL, CH_W / 2), (FUNNEL + CH_L, CH_W / 2), (FUNNEL + CH_L, -CH_W / 2), (FUNNEL, -CH_W / 2), (x0, -(MOUTH / 2 + s * 3.0))]


def outer_poly(d, front_x=0.0):
    P = inner_poly(); cw = area(P) < 0
    edges = [(P[i], P[i + 1]) for i in range(0, 5)]
    L = [line_off(p, q, d, cw) for p, q in edges]
    pts = []
    p, dd = L[0]; t = (front_x - p[0]) / dd[0]; pts.append((front_x, p[1] + dd[1] * t))
    for i in range(4): pts.append(isect(L[i], L[i + 1]))
    p, dd = L[4]; t = (front_x - p[0]) / dd[0]; pts.append((front_x, p[1] + dd[1] * t))
    return pts


def poly_mask(poly, xs, ys, dx=0.0):
    XX, YY = np.meshgrid(xs, ys, indexing="xy")
    pts = np.c_[(XX - dx - X_MOUTH).ravel(), YY.ravel()]
    return MPath(poly).contains_points(pts).reshape(XX.shape)


def hood_masks(xs, ys, zs, dz, dx=0.0):
    """(cavity, material, envelope) を (nz, ny, nx) のブール配列で返す。dz = 持ち上げ量。"""
    inn = poly_mask(inner_poly(), xs, ys, dx)
    out = poly_mask(outer_poly(WALL), xs, ys, dx)
    foot = poly_mask(outer_poly(FOOT), xs, ys, dx)
    XX = np.meshgrid(xs, ys, indexing="xy")[0]
    front = (XX - dx - X_MOUTH) >= 0                 # 口の面より前（局所 x < 0）は無し
    inn &= front; out &= front; foot &= front
    z = zs - dz
    cav = np.zeros((len(zs), len(ys), len(xs)), bool)
    mat = np.zeros_like(cav); env = np.zeros_like(cav)
    for k, zz in enumerate(z):
        if 0 <= zz < CH_H:
            cav[k] = inn; mat[k] = out & ~inn
            if zz < FOOT_H: mat[k] |= foot & ~inn
        elif CH_H <= zz < H_OUT:
            mat[k] = out
        if 0 <= zz < H_OUT:
            env[k] = out | (foot if zz < FOOT_H else False) | inn
    return cav, mat, env


def main():
    tfp = SCR / "transforms.json"
    folder = SCR / "local3"
    box = (-250.0, -160.0, -50.0, 50.0, 0.0, 70.0)
    g.X0, g.X1, g.Y0, g.Y1, g.Z0, g.Z1 = box; g.DX = g.DY = g.DZ = 0.5
    xs = np.arange(box[0] + 0.25, box[1], 0.5); ys = np.arange(box[2] + 0.25, box[3], 0.5); zs = np.arange(box[4] + 0.25, box[5], 0.5)
    cell = 0.5 ** 3
    meshes = g.load_posed(folder, "p0", tfp)
    allnames = sorted(folder.glob("*.stl"))
    head = {p.stem: m for p, m in zip(allnames, meshes) if p.stem.startswith("HEADV")}
    occ = {k: g.occupancy([m]) for k, m in head.items()}
    head_all = np.zeros_like(next(iter(occ.values())))
    for o in occ.values():
        head_all |= o
    info = {}
    for k, o in occ.items():
        if o.any():
            idx = np.argwhere(o)
            info[k] = {"volume_mm3_in_box": round(float(o.sum() * cell)), "x": [round(float(xs[idx[:, 2].min()]), 1), round(float(xs[idx[:, 2].max()]), 1)],
                       "z": [round(float(zs[idx[:, 0].min()]), 1), round(float(zs[idx[:, 0].max()]), 1)]}
    SENS = dict(cg.SENSORS)
    SENS["J1 servo (REF, Fusion の REF 箱 y ±18)"] = (-190.0, -145.0, -18.0, 18.0, 20.0, 45.0)

    def box_mask(b):
        m = np.zeros_like(head_all)
        i0, i1 = np.searchsorted(xs, b[0]), np.searchsorted(xs, b[1]); j0, j1 = np.searchsorted(ys, b[2]), np.searchsorted(ys, b[3]); k0, k1 = np.searchsorted(zs, b[4]), np.searchsorted(zs, b[5])
        m[k0:k1, j0:j1, i0:i1] = True
        return m

    boxes = {k: box_mask(b) for k, b in SENS.items()}
    res = {"stroke": {}}
    _, base_mat, base_env = hood_masks(xs, ys, zs, 0.0)
    for s in (0, 3, 4, 5, 8, 10):
        L = 20.0                                   # 平行リンクの長さ（仮）: 上下 dz = s、前後 dx = L(1-cosφ) を後ろへ
        dx = L * (1 - math.sqrt(1 - (s / L) ** 2)) if s else 0.0
        sweep = np.zeros_like(base_env)
        for f in np.linspace(0, 1, 11):
            sz = s * f; sx = L * (1 - math.sqrt(1 - (sz / L) ** 2)) if sz else 0.0
            sweep |= hood_masks(xs, ys, zs, sz, sx)[2]
        cav, mat, env = hood_masks(xs, ys, zs, s, dx)
        res["stroke"][str(s)] = {
            "dz_mm": s, "dx_rearward_mm(L=20)": round(dx, 2),
            "envelope_mm3": round(float(env.sum() * cell)), "material_mm3": round(float(mat.sum() * cell)),
            "sweep_envelope_mm3": round(float(sweep.sum() * cell)),
            "sweep_into_head_all_mm3": round(float((sweep & head_all).sum() * cell)),
            "extra_dig_vs_s0_mm3": round(float((sweep & head_all & ~base_env).sum() * cell)),
            "hood_material_at_up_into_head_mm3": round(float((mat & head_all).sum() * cell)),
            "by_ref_box_sweep_mm3": {k: round(float((sweep & b).sum() * cell), 1) for k, b in boxes.items()},
            "by_ref_box_extra_vs_s0_mm3": {k: round(float((sweep & b & ~base_env).sum() * cell), 1) for k, b in boxes.items()},
            "by_head_body_sweep_mm3": {k: round(float((sweep & o).sum() * cell)) for k, o in occ.items() if (sweep & o).any()},
            "by_head_body_extra_vs_s0_mm3": {k: round(float((sweep & o & ~base_env).sum() * cell)) for k, o in occ.items() if (sweep & o & ~base_env).any()},
            "roof_top_z": H_OUT + s}
    res["head_bodies"] = info
    res["ref_boxes"] = {k: list(v) for k, v in SENS.items()}
    m_hood = 0.0114                                # kg（PLA 9.2 cm3 → 11.4 g、E3 REPORT §6）
    kin = {}
    for s in (5, 8, 10):
        rc = s / 2
        kin[str(s)] = {"crank_r_mm(2r=stroke)": rc, "SG90_stall_Nm(ASSUMED 0.12 @4.8V)": 0.12,
                       "peak_force_N_at_crank_pin": round(0.12 / (rc / 1000), 1), "hood_weight_N": round(m_hood * 9.81, 3),
                       "free_fall_speed_m_per_s": round(math.sqrt(2 * 9.81 * s / 1000), 2),
                       "free_fall_energy_mJ": round(m_hood * 9.81 * s, 2), "drive_speed_mm_s(0.5 s)": round(s / 0.5, 1)}
    res["kinematics_assumed"] = kin
    OUT.write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")
    for s, r in res["stroke"].items():
        print(s, {k: v for k, v in r.items() if not k.startswith("by_")})
        print("  REF extra:", {k: v for k, v in r["by_ref_box_extra_vs_s0_mm3"].items() if v})
        print("  REF sweep:", {k: v for k, v in r["by_ref_box_sweep_mm3"].items() if v})
        print("  head extra:", r["by_head_body_extra_vs_s0_mm3"])
    print("head bodies:", json.dumps(info, ensure_ascii=False))


if __name__ == "__main__":
    main()
