"""J1 の頭と首のあいだのすき間に対する「詰め物」案の計算（Fusion なしのボクセル）— 首側の詰め物 F（上げ側）と頭側の詰め物 H（頭に固定）。

【先頭の注意】KNUCKLE DRUM 後も、まっすぐ比で約 20 倍（18〜24 倍）のすき間が残る。背板は現行 CAD では未解決のまま。5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）。安全・合格の語は使わない。
姿勢ラベルは CAD の符号（transforms.json の p*）。**この符号では + が頭を下げる**（p8 = +10 で頭の前面が 9.5 mm 下がる）。詳細 j1_sign_correction_2026-09-30.md。
使い方: python j1_cover_eval.py build   → 詰め物のボクセルを作る（j1cover.npz、STL）
"""
import sys, json
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import gap_check as g
SCR = Path("C:/Users/Public/serpens_gapcheck"); TF = SCR / "transforms.json"; L = SCR / "local3"
BOX = (-250.0, -110.0, -60.0, 60.0, 0.0, 100.0); C = 0.5
POSES = ("p6", "p7", "p12", "p11", "p10", "p0", "p13", "p14", "p15", "p16", "p8")
ANG = {"p0": 0, "p7": -5, "p6": -10, "p8": 10, "p10": -1, "p11": -2, "p12": -3, "p13": 1, "p14": 2, "p15": 3, "p16": 5}

def dil(a, n):
    o = a.copy()
    for _ in range(n):
        p = o.copy()
        for ax in range(3):
            p |= np.roll(o, 1, ax); p |= np.roll(o, -1, ax)
        o = p
    return o

def centers():
    xs = BOX[0] + (np.arange(int((BOX[1] - BOX[0]) / C)) + .5) * C
    ys = BOX[2] + (np.arange(int((BOX[3] - BOX[2]) / C)) + .5) * C
    zs = BOX[4] + (np.arange(int((BOX[5] - BOX[4]) / C)) + .5) * C
    return xs, ys, zs

def pose_matrix(pose, key="HEADV"):
    m = np.array(json.loads(TF.read_text())[pose][key]).reshape(4, 4); R = m[:3, :3]; t = m[:3, 3] * 10.0
    return R, t

def voxel_mesh(mask, origin, cell=C):
    """ボクセルの露出面から三角形メッシュ（外向き法線、(n,3,3) mm）。"""
    tris = []
    nz, ny, nx = mask.shape
    P = np.pad(mask, 1)
    for ax, (da) in enumerate(((1, 0, 0), (0, 1, 0), (0, 0, 1))):
        pass
    idx = np.argwhere(mask)
    def add(quad, flip):
        a, b, c, d = quad
        t = [(a, b, c), (a, c, d)] if not flip else [(a, c, b), (a, d, c)]
        tris.extend(t)
    for (k, j, i) in idx:
        x0, y0, z0 = origin[0] + i * cell, origin[1] + j * cell, origin[2] + k * cell
        x1, y1, z1 = x0 + cell, y0 + cell, z0 + cell
        kk, jj, ii = k + 1, j + 1, i + 1
        if not P[kk, jj, ii + 1]: add(((x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)), False)
        if not P[kk, jj, ii - 1]: add(((x0, y0, z0), (x0, y0, z1), (x0, y1, z1), (x0, y1, z0)), False)
        if not P[kk, jj + 1, ii]: add(((x0, y1, z0), (x0, y1, z1), (x1, y1, z1), (x1, y1, z0)), False)
        if not P[kk, jj - 1, ii]: add(((x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)), False)
        if not P[kk + 1, jj, ii]: add(((x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)), False)
        if not P[kk - 1, jj, ii]: add(((x0, y0, z0), (x0, y1, z0), (x1, y1, z0), (x1, y0, z0)), False)
    return np.array(tris, float)

def write_stl(tris, path):
    import struct
    with open(path, "wb") as f:
        f.write(b"J1 cover study".ljust(80, b" ")); f.write(struct.pack("<I", len(tris)))
        for t in tris:
            n = np.cross(t[1] - t[0], t[2] - t[0]); nn = np.linalg.norm(n); n = n / nn if nn > 0 else n
            f.write(struct.pack("<3f", *n)); f.write(struct.pack("<9f", *t.ravel())); f.write(b"\0\0")
