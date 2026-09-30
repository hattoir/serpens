"""外装の隙間検査: 5〜12 mm の隙間（指が入って挟まれうる幅）が無いか（ENTRY-E-0003 (5) への Design の確認）。

入力: Fusion から書き出した STL（姿勢ごとのフォルダ、単位 mm、世界座標）。標準ライブラリ + numpy だけ。
方法: 各 STL を x 方向の線で切り、三角形の向き（法線の x 成分の符号）で巻き数を数えて占有格子にする（重なる立体も扱える）。
      その格子で、x / y / z の各方向に「両端が固体にはさまれた空きの連続」の長さを測り、5.0 <= L <= 12.0 mm のものを印にする。
      印の連結成分（26 近傍）ごとに、体積・外接箱・位置を出す。

出力の読み方（限界）:
- 外装の外形だけ（内部の部品・配線・ねじ・公差は入っていない）。判定ではなく「見つけて直す」ための道具。
- 格子は x 0.5 mm、y・z 1.0 mm。5 mm ちょうどの前後は格子の丸めで ±1 mm ぶれる（4〜6 mm の帯は「要確認」として別に数える）。
- 「両端が固体」なので、外へ開いた溝（片側が開いている）は入らない。深い溝は開口の幅を、開口の幅が 5〜12 mm の溝として
  別の検査（z 方向の空きの連続の長さ）で拾う。
- 隙間の広さが 12 mm を超える所は指の挟み込みの対象から外している（別の規則: 首・手首が入る開口は Engineering が判定）。

実行:  .venv/Scripts/python.exe docs/design/tools/gap_check.py <STLのフォルダ> [姿勢名 変換json [出力json]]
"""
from __future__ import annotations

import json
import struct
import sys
from pathlib import Path

import numpy as np

X0, X1, DX = -260.0, 360.0, 0.5
Y0, Y1, DY = -62.0, 62.0, 1.0
Z0, Z1, DZ = -2.0, 102.0, 1.0
LO, HI = 5.0, 12.0


def read_stl(path: Path) -> np.ndarray:
    b = path.read_bytes()
    if b[:5] == b"solid" and b"facet" in b[:400]:
        raise ValueError("ASCII STL は未対応: " + path.name)
    n = struct.unpack("<I", b[80:84])[0]
    a = np.frombuffer(b, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n, offset=84)
    return a["v"].astype(np.float64)          # (n, 3 vertices, 3)


def occupancy(meshes: list[np.ndarray]) -> np.ndarray:
    ys = np.arange(Y0 + DY / 2, Y1, DY)
    zs = np.arange(Z0 + DZ / 2, Z1, DZ)
    xs = np.arange(X0 + DX / 2, X1, DX)
    ny, nz, nx = len(ys), len(zs), len(xs)
    cross: dict[int, list[tuple[float, int]]] = {}
    for tri in meshes:
        v0, v1, v2 = tri[:, 0], tri[:, 1], tri[:, 2]
        n = np.cross(v1 - v0, v2 - v0)
        for i in range(len(tri)):
            nxi = n[i, 0]
            if abs(nxi) < 1e-12:
                continue
            y = tri[i, :, 1]; z = tri[i, :, 2]
            j0 = max(0, int(np.floor((y.min() - Y0) / DY - 0.5))); j1 = min(ny - 1, int(np.ceil((y.max() - Y0) / DY - 0.5)))
            k0 = max(0, int(np.floor((z.min() - Z0) / DZ - 0.5))); k1 = min(nz - 1, int(np.ceil((z.max() - Z0) / DZ - 0.5)))
            if j1 < j0 or k1 < k0:
                continue
            jj, kk = np.meshgrid(np.arange(j0, j1 + 1), np.arange(k0, k1 + 1), indexing="ij")
            py, pz = ys[jj], zs[kk]
            # barycentric in (y, z)
            d = (y[1] - y[0]) * (z[2] - z[0]) - (y[2] - y[0]) * (z[1] - z[0])
            if abs(d) < 1e-14:
                continue
            l1 = ((py - y[0]) * (z[2] - z[0]) - (y[2] - y[0]) * (pz - z[0])) / d
            l2 = ((y[1] - y[0]) * (pz - z[0]) - (py - y[0]) * (z[1] - z[0])) / d
            l0 = 1 - l1 - l2
            inside = (l0 >= 0) & (l1 >= 0) & (l2 >= 0)
            if not inside.any():
                continue
            xh = l0 * tri[i, 0, 0] + l1 * tri[i, 1, 0] + l2 * tri[i, 2, 0]
            w = -1 if nxi > 0 else 1               # 法線が +x を向く面 = 出る面（−1）、−x を向く面 = 入る面（+1）
            for a, b_ in zip(jj[inside], kk[inside]):
                pass
            for a, b_, xx in zip(jj[inside], kk[inside], xh[inside]):
                cross.setdefault(int(a) * nz + int(b_), []).append((float(xx), w))
    occ = np.zeros((nz, ny, nx), bool)
    for key, lst in cross.items():
        j, k = divmod(key, nz)
        lst.sort()
        xsx = np.array([c[0] for c in lst]); ws = np.cumsum([c[1] for c in lst])
        inside_pts = np.where(ws > 0)[0]
        for idx in inside_pts:
            xa = xsx[idx]; xb = xsx[idx + 1] if idx + 1 < len(xsx) else xa
            ia = int(np.ceil((xa - X0) / DX - 0.5)); ib = int(np.floor((xb - X0) / DX - 0.5))
            if ib >= ia and ib >= 0 and ia <= nx - 1:      # 箱の外だけの区間は無視（負の添字で行が全部埋まる不具合を直した 2026-09-30）
                occ[k, j, max(ia, 0):min(ib, nx - 1) + 1] = True
    return occ


def bounded_runs(occ: np.ndarray, axis: int, step: float, lo: float, hi: float) -> np.ndarray:
    """axis に沿って、両端が固体にはさまれた空きの連続のうち長さが lo..hi のものを True にした配列を返す。"""
    o = np.moveaxis(occ, axis, -1)
    n = o.shape[-1]
    mark = np.zeros_like(o)
    flat = o.reshape(-1, n); mflat = mark.reshape(-1, n)
    for r in range(flat.shape[0]):
        row = flat[r]
        if not row.any():
            continue
        d = np.diff(np.r_[False, row, False].astype(np.int8))
        ends_solid = np.where(d == -1)[0]           # 固体が終わる位置 = 空きの始まり
        starts_solid = np.where(d == 1)[0]          # 固体が始まる位置 = 空きの終わり
        for e in ends_solid:
            nxt = starts_solid[starts_solid > e]
            if len(nxt) == 0:
                continue
            L = (nxt[0] - e) * step
            if lo <= L <= hi:
                mflat[r, e:nxt[0]] = True
    return np.moveaxis(mark, -1, axis)


def components(mark: np.ndarray, min_vox: int) -> list[dict]:
    idx = np.argwhere(mark)
    if len(idx) == 0:
        return []
    # 10 mm の升に丸めて連結（26 近傍）を数える簡易版: 升ごとの体積を出し、隣接する升をまとめる
    cell = np.array([10 / DZ, 10 / DY, 10 / DX])
    keys = {tuple(k) for k in (idx // cell).astype(int)}
    labels: dict[tuple, int] = {}
    comps: list[list[tuple]] = []
    for k in keys:
        if k in labels:
            continue
        stack = [k]; labels[k] = len(comps); cur = [k]
        while stack:
            a = stack.pop()
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        b = (a[0] + da, a[1] + db, a[2] + dc)
                        if b in keys and b not in labels:
                            labels[b] = len(comps); stack.append(b); cur.append(b)
        comps.append(cur)
    out = []
    for ci, cur in enumerate(comps):
        sel = np.array([labels.get(tuple(k), -1) == ci for k in (idx // cell).astype(int)])
        pts = idx[sel]
        if len(pts) < min_vox:
            continue
        vol = len(pts) * DX * DY * DZ
        mn = pts.min(0); mx = pts.max(0)
        out.append({"volume_mm3": round(vol), "x_mm": [round(X0 + mn[2] * DX), round(X0 + (mx[2] + 1) * DX)], "y_mm": [round(Y0 + mn[1] * DY), round(Y0 + (mx[1] + 1) * DY)], "z_mm": [round(Z0 + mn[0] * DZ), round(Z0 + (mx[0] + 1) * DZ)]})
    out.sort(key=lambda d: -d["volume_mm3"])
    return out


def load_posed(folder: Path, pose: str | None, transforms: Path | None) -> list[np.ndarray]:
    """folder の STL（部品の座標系）に、Fusion から書き出した姿勢ごとの変換（4x4、行優先、平行移動は cm）を掛ける。
    ファイル名 `<KEY>__NN.stl` の KEY が変換の名前（LINK1 など）。"""
    meshes = []
    tf = json.loads(transforms.read_text()) if (pose and transforms) else None
    for p in sorted(folder.glob("*.stl")):
        v = read_stl(p)
        if tf is not None:
            key = p.stem.split("__")[0]
            m = np.array(tf[pose][key]).reshape(4, 4)
            R, t = m[:3, :3], m[:3, 3] * 10.0
            v = (v.reshape(-1, 3) @ R.T + t).reshape(-1, 3, 3)
        meshes.append(v)
    return meshes


def analyse(folder: Path, pose: str | None = None, transforms: Path | None = None) -> dict:
    meshes = load_posed(folder, pose, transforms)
    occ = occupancy(meshes)
    res = {"folder": folder.name, "pose": pose, "solid_volume_cm3": round(occ.sum() * DX * DY * DZ / 1000, 1), "axes": {}}
    for name, axis, step in (("x（前後）", 2, DX), ("y（左右）", 1, DY), ("z（上下）", 0, DZ)):
        mark = bounded_runs(occ, axis, step, LO, HI)
        near = bounded_runs(occ, axis, step, 4.0, LO - 0.01) if False else None
        res["axes"][name] = {"marked_volume_mm3": round(mark.sum() * DX * DY * DZ), "clusters": components(mark, 30)[:12]}
    return res


if __name__ == "__main__":
    folder = Path(sys.argv[1])
    pose = sys.argv[2] if len(sys.argv) > 2 else None
    tfp = Path(sys.argv[3]) if len(sys.argv) > 3 else None
    r = analyse(folder, pose, tfp)
    if len(sys.argv) > 4:
        Path(sys.argv[4]).write_text(json.dumps(r, ensure_ascii=False, indent=2), encoding="utf-8")
    ax = r["axes"]
    print(pose, "solid cm3", r["solid_volume_cm3"])
    for k, v in ax.items():
        print(" ", k, "marked mm3", v["marked_volume_mm3"])
        for c in v["clusters"][:10]:
            print("     ", c)


# ---------------------------------------------------------------------------
# 局所の隙間の幅（13 方向の最小）と、外へつながった空きだけを数える版
# ---------------------------------------------------------------------------
BIG = 30000
DIRS = [d for d in [(dz, dy, dx) for dz in (-1, 0, 1) for dy in (-1, 0, 1) for dx in (-1, 0, 1)] if d != (0, 0, 0) and d > (0, 0, 0) or False]
DIRS = sorted({tuple(d) if tuple(d) > tuple(-np.array(d)) else tuple(-np.array(d)) for d in [(dz, dy, dx) for dz in (-1, 0, 1) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if (dz, dy, dx) != (0, 0, 0)]})


def to_iso(occ: np.ndarray) -> np.ndarray:
    """x を 1 mm に（中心の標本）。y・z は 1 mm のまま。"""
    return occ[:, :, ::2].copy()


def exterior(empty: np.ndarray) -> np.ndarray:
    """境界へ 6 近傍でつながった空きだけを True にする（閉じた空洞は False）。"""
    ext = np.zeros_like(empty)
    ext[0] |= empty[0]; ext[-1] |= empty[-1]; ext[:, 0] |= empty[:, 0]; ext[:, -1] |= empty[:, -1]; ext[:, :, 0] |= empty[:, :, 0]; ext[:, :, -1] |= empty[:, :, -1]
    while True:
        n = ext.copy()
        n[1:] |= ext[:-1]; n[:-1] |= ext[1:]; n[:, 1:] |= ext[:, :-1]; n[:, :-1] |= ext[:, 1:]; n[:, :, 1:] |= ext[:, :, :-1]; n[:, :, :-1] |= ext[:, :, 1:]
        n &= empty
        if (n == ext).all():
            return ext
        ext = n


def forward_count(empty: np.ndarray, d: tuple[int, int, int]) -> np.ndarray:
    """各セルから方向 d に、固体に当たるまで続く空きのセル数（自分を含む）。範囲の外へ出るなら BIG。"""
    a = int(np.argmax(np.abs(d)))
    n = empty.shape[a]
    F = np.zeros(empty.shape, np.int32)
    rng = range(n - 1, -1, -1) if d[a] > 0 else range(n)
    oth = [i for i in range(3) if i != a]
    for i in rng:
        nxt_i = i + d[a]
        sl = [slice(None)] * 3; sl[a] = i
        cur = empty[tuple(sl)]
        if not (0 <= nxt_i < n):
            F[tuple(sl)] = np.where(cur, BIG, 0)
            continue
        sl2 = [slice(None)] * 3; sl2[a] = nxt_i
        prev = F[tuple(sl2)]
        # 面内の残りの成分だけずらす（範囲外は BIG）
        shifted = np.full(prev.shape, BIG, np.int32)
        s_src = [slice(None), slice(None)]; s_dst = [slice(None), slice(None)]
        for ax_local, ax in enumerate(oth):
            step = d[ax]
            m = prev.shape[ax_local]
            if step > 0:
                s_dst[ax_local] = slice(0, m - 1); s_src[ax_local] = slice(1, m)
            elif step < 0:
                s_dst[ax_local] = slice(1, m); s_src[ax_local] = slice(0, m - 1)
        shifted[tuple(s_dst)] = prev[tuple(s_src)]
        F[tuple(sl)] = np.where(cur, np.minimum(shifted + 1, BIG), 0)
    return F


def local_width(empty: np.ndarray) -> np.ndarray:
    """空きセルごとに、13 方向のうち両端が固体にはさまれた最短の長さ（mm）。どの方向も抜けるなら inf。"""
    W = np.full(empty.shape, np.inf)
    for d in DIRS:
        f = forward_count(empty, d)
        b = forward_count(empty, tuple(-x for x in d))
        tot = (f + b - 1).astype(np.float64)
        bounded = (f < BIG // 2) & (b < BIG // 2) & empty
        length = tot * float(np.linalg.norm(d))
        W = np.where(bounded & (length < W), length, W)
    return W


def analyse_width(folder: Path, pose: str | None, transforms: Path | None, lo: float = 5.0, hi: float = 12.0) -> dict:
    meshes = load_posed(folder, pose, transforms)
    occ = to_iso(occupancy(meshes))
    empty = ~occ
    ext = exterior(empty)
    W = local_width(empty)
    W[~ext] = np.inf
    mark = (W >= lo) & (W <= hi)
    near = (W >= lo - 1.5) & (W < lo)           # 3.5〜5 mm（格子の丸め ±1 の要確認帯）
    dxi = 1.0
    def clusters(m):
        idx = np.argwhere(m)
        if len(idx) == 0:
            return []
        cell = 10
        keys = {tuple(k) for k in (idx // cell)}
        seen: dict = {}
        comps = []
        for k in keys:
            if k in seen:
                continue
            stack = [k]; seen[k] = len(comps); cur = [k]
            while stack:
                a = stack.pop()
                for da in (-1, 0, 1):
                    for db in (-1, 0, 1):
                        for dc in (-1, 0, 1):
                            b = (a[0] + da, a[1] + db, a[2] + dc)
                            if b in keys and b not in seen:
                                seen[b] = len(comps); stack.append(b); cur.append(b)
            comps.append(cur)
        out = []
        for ci in range(len(comps)):
            sel = np.array([seen[tuple(k)] == ci for k in (idx // cell)])
            p = idx[sel]
            if len(p) < 40:
                continue
            mn, mx_ = p.min(0), p.max(0)
            out.append({"volume_mm3": int(len(p)), "x_mm": [round(X0 + mn[2] * 2 * DX), round(X0 + (mx_[2] + 1) * 2 * DX)], "y_mm": [round(Y0 + mn[1] * DY), round(Y0 + (mx_[1] + 1) * DY)], "z_mm": [round(Z0 + mn[0] * DZ), round(Z0 + (mx_[0] + 1) * DZ)], "width_mm": [round(float(W[tuple(p.T)].min()), 1), round(float(W[tuple(p.T)].max()), 1)]})
        out.sort(key=lambda d: -d["volume_mm3"])
        return out
    return {"pose": pose, "flag_5_12_mm3": int(mark.sum()), "near_3p5_5_mm3": int(near.sum()), "clusters_5_12": clusters(mark)[:14], "clusters_near": clusters(near)[:8]}


if __name__ == "__main__" and len(sys.argv) > 5 and sys.argv[5] == "width":
    pass


# ---------------------------------------------------------------------------
# 報告用: 外へつながった空きだけ、軸方向の 5〜12 mm の挟み込み（平行な隙間）を、体の幅 |y|<=24 と外側に分けて数える
# ---------------------------------------------------------------------------
def analyse_slots(folder: Path, pose: str, transforms: Path, y_body: float = 24.0) -> dict:
    meshes = load_posed(folder, pose, transforms)
    occ = occupancy(meshes)
    empty = ~occ
    ext = exterior(to_iso(occ) == False)            # 1 mm 等方格子で外への連結を求める
    ext_full = np.repeat(ext, 2, axis=2)[:, :, : occ.shape[2]]
    ys = np.arange(Y0 + DY / 2, Y1, DY)
    res = {"pose": pose, "regions": {}}
    marks = {"x": bounded_runs(occ, 2, DX, LO, HI), "y": bounded_runs(occ, 1, DY, LO, HI), "z": bounded_runs(occ, 0, DZ, LO, HI)}
    for region, ysel in (("body_|y|<=%g" % y_body, np.abs(ys) <= y_body), ("outer_|y|>%g" % y_body, np.abs(ys) > y_body)):
        r = {}
        for ax, m in marks.items():
            mm = m & ext_full
            mm = mm & ysel[None, :, None]
            r[ax] = {"marked_mm3": int(round(mm.sum() * DX * DY * DZ)), "clusters": components(mm, 30)[:8]}
        res["regions"][region] = r
    return res


# ---------------------------------------------------------------------------
# 探針（直径 5 mm の球 = 5 mm 未満の隙間には入れない）で外から届く空きだけを対象にする最終版
# ---------------------------------------------------------------------------
def _ball_offsets(r: float) -> list[tuple[int, int, int]]:
    n = int(np.floor(r))
    return [(i, j, k) for i in range(-n, n + 1) for j in range(-n, n + 1) for k in range(-n, n + 1) if i * i + j * j + k * k <= r * r + 1e-9]


def _shift(a: np.ndarray, off: tuple[int, int, int], fill: bool) -> np.ndarray:
    out = np.full(a.shape, fill, a.dtype)
    src = []; dst = []
    for o, n in zip(off, a.shape):
        if o >= 0:
            src.append(slice(o, n)); dst.append(slice(0, n - o))
        else:
            src.append(slice(0, n + o)); dst.append(slice(-o, n))
    out[tuple(dst)] = a[tuple(src)]
    return out


def probe_accessible(empty_iso: np.ndarray, radius: float = 2.5) -> np.ndarray:
    """直径 2*radius の球の中心が外から届く空きセル（境界の外は空きとみなす）と、その球が触れる空きセルを返す。"""
    offs = _ball_offsets(radius)
    E = empty_iso.copy()
    for off in offs:
        E &= _shift(empty_iso, off, True)                 # 球が全部空きの所だけ
    A = exterior(E)                                       # 外へつながった球の中心
    R = np.zeros_like(empty_iso)
    for off in offs:
        R |= _shift(A, (-off[0], -off[1], -off[2]), False)  # 球の中心から半径以内 = 触れる空き
    return R & empty_iso


def analyse_final(folder: Path, pose: str, transforms: Path, y_body: float = 24.0, probe_d: float = 5.0) -> dict:
    meshes = load_posed(folder, pose, transforms)
    occ = occupancy(meshes)
    occ_iso = to_iso(occ)
    acc = probe_accessible(~occ_iso, probe_d / 2)
    acc_full = np.repeat(acc, 2, axis=2)[:, :, : occ.shape[2]]
    ys = np.arange(Y0 + DY / 2, Y1, DY)
    res = {"pose": pose, "probe_diameter_mm": probe_d, "accessible_empty_cm3": round(float(acc.sum()) / 1000, 1), "regions": {}}
    marks = {"x": bounded_runs(occ, 2, DX, LO, HI), "y": bounded_runs(occ, 1, DY, LO, HI), "z": bounded_runs(occ, 0, DZ, LO, HI)}
    for region, ysel in (("body |y|<=%g" % y_body, np.abs(ys) <= y_body), ("outer |y|>%g" % y_body, np.abs(ys) > y_body)):
        r = {}
        for ax, m in marks.items():
            mm = m & acc_full & ysel[None, :, None]
            r[ax] = {"marked_mm3": int(round(mm.sum() * DX * DY * DZ)), "clusters": components(mm, 30)[:8]}
        res["regions"][region] = r
    return res


def analyse_report(folder: Path, pose: str, transforms: Path, y_body: float = 24.0, probe_d: float = 5.0, w_min: float = 4.9) -> dict:
    """報告用: 探針で届く空きのうち、軸方向の空きの連続が 5〜12 mm で、かつ 13 方向の最短の幅も w_min 以上のものだけ数える
    （斜めの継ぎ目で軸方向の測定が 1/cosθ ぶん大きく出る誤検出を落とす）。"""
    meshes = load_posed(folder, pose, transforms)
    occ = occupancy(meshes)
    occ_iso = to_iso(occ)
    empty = ~occ_iso
    acc = probe_accessible(empty, probe_d / 2)
    W = local_width(empty)
    keep = acc & (W >= w_min)
    keep_full = np.repeat(keep, 2, axis=2)[:, :, : occ.shape[2]]
    ys = np.arange(Y0 + DY / 2, Y1, DY)
    res = {"pose": pose, "probe_diameter_mm": probe_d, "min_true_width_mm": w_min, "regions": {}}
    marks = {"x": bounded_runs(occ, 2, DX, LO, HI), "y": bounded_runs(occ, 1, DY, LO, HI), "z": bounded_runs(occ, 0, DZ, LO, HI)}
    for region, ysel in (("body |y|<=%g" % y_body, np.abs(ys) <= y_body), ("outer |y|>%g" % y_body, np.abs(ys) > y_body)):
        r = {}
        for ax, m in marks.items():
            mm = m & keep_full & ysel[None, :, None]
            r[ax] = {"marked_mm3": int(round(mm.sum() * DX * DY * DZ)), "clusters": components(mm, 30)[:8]}
        res["regions"][region] = r
    return res


# ---------------------------------------------------------------------------
# 細かい格子（0.5 mm 等方）で一部の範囲だけ調べる版（丸めの誤差を小さくする）
# ---------------------------------------------------------------------------
def analyse_fine(folder: Path, pose: str, transforms: Path, box: tuple[float, float, float, float, float, float], cell: float = 0.5,
                 probe_d: float = 5.0, lo: float = 5.0, hi: float = 12.0) -> dict:
    """box = (x0, x1, y0, y1, z0, z1) mm。cell 等方の格子。探針で届く空きのうち、13 方向の最短の幅が lo..hi の所を数える。"""
    global X0, X1, Y0, Y1, Z0, Z1, DX, DY, DZ
    saved = (X0, X1, Y0, Y1, Z0, Z1, DX, DY, DZ)
    X0, X1, Y0, Y1, Z0, Z1 = box; DX = DY = DZ = cell
    try:
        meshes = load_posed(folder, pose, transforms)
        occ = occupancy(meshes)                      # (nz, ny, nx) 等方
    finally:
        X0, X1, Y0, Y1, Z0, Z1, DX, DY, DZ = saved
    empty = ~occ
    acc = probe_accessible(empty, (probe_d / 2) / cell)
    W = local_width(empty) * cell
    W[~acc] = np.inf
    mark = (W >= lo) & (W <= hi)
    near = (W >= lo - 1.0) & (W < lo)
    def clusters(m, min_cells):
        idx = np.argwhere(m)
        if len(idx) == 0:
            return []
        c = int(round(10 / cell))
        keys = {tuple(k) for k in (idx // c)}
        seen: dict = {}; comps = []
        for k in keys:
            if k in seen:
                continue
            stack = [k]; seen[k] = len(comps); cur = [k]
            while stack:
                a = stack.pop()
                for da in (-1, 0, 1):
                    for db in (-1, 0, 1):
                        for dc in (-1, 0, 1):
                            b = (a[0] + da, a[1] + db, a[2] + dc)
                            if b in keys and b not in seen:
                                seen[b] = len(comps); stack.append(b); cur.append(b)
            comps.append(cur)
        lab = np.array([seen[tuple(k)] for k in (idx // c)])
        out = []
        for ci in range(len(comps)):
            p = idx[lab == ci]
            if len(p) < min_cells:
                continue
            mn, mx_ = p.min(0), p.max(0)
            out.append({"volume_mm3": round(len(p) * cell ** 3), "x_mm": [round(box[0] + mn[2] * cell), round(box[0] + (mx_[2] + 1) * cell)], "y_mm": [round(box[2] + mn[1] * cell), round(box[2] + (mx_[1] + 1) * cell)], "z_mm": [round(box[4] + mn[0] * cell), round(box[4] + (mx_[0] + 1) * cell)], "width_mm": [round(float(W[tuple(p.T)].min()), 1), round(float(W[tuple(p.T)].max()), 1)]})
        out.sort(key=lambda d: -d["volume_mm3"])
        return out
    return {"pose": pose, "cell_mm": cell, "box": list(box), "flag_mm3": round(float(mark.sum()) * cell ** 3), "near_mm3": round(float(near.sum()) * cell ** 3),
            "clusters": clusters(mark, int(200 / cell ** 3)), "clusters_near": clusters(near, int(200 / cell ** 3))}


# ---------------------------------------------------------------------------
# 深さのある隙間（すきま）だけを危険として数える最終判定版
#   すきま = 探針（直径 5 mm）が入れる空きのうち、局所の幅が 5〜12.5 mm の所
#   深さ   = 広く開いた所（幅 > 12.5 mm か、どの方向にも抜ける所）から、すきまの中を 6 近傍で何 mm 入ったか
#   危険   = すきまで、深さが depth_min（既定 8 mm）以上
#   → 鈍角の V の入口（浅い）は入らず、平行な深い溝は入る。
# ---------------------------------------------------------------------------
def analyse_crevices(folder: Path, pose: str, transforms: Path, box: tuple[float, float, float, float, float, float], cell: float = 0.5,
                     lo: float = 5.0, hi: float = 12.0, depth_min: float = 8.0, extra_meshes: list | None = None, return_mask: bool = False) -> dict:
    """extra_meshes: 姿勢の変換を掛けない追加の三角形メッシュ（(n,3,3)、mm。姿勢ごとに作ったカバーなど）。return_mask: 結果に hazard の bool 配列と W を入れる。"""
    global X0, X1, Y0, Y1, Z0, Z1, DX, DY, DZ
    saved = (X0, X1, Y0, Y1, Z0, Z1, DX, DY, DZ)
    X0, X1, Y0, Y1, Z0, Z1 = box; DX = DY = DZ = cell
    try:
        occ = occupancy(load_posed(folder, pose, transforms) + list(extra_meshes or []))
    finally:
        X0, X1, Y0, Y1, Z0, Z1, DX, DY, DZ = saved
    empty = ~occ
    W = local_width(empty) * cell
    passable = empty & (W >= lo)                                  # 探針が通れる
    A = exterior(passable)                                        # 外から届く
    crev = A & (W >= lo) & (W <= hi + 0.5)                        # すきま
    open_ = A & ~crev                                             # 広く開いた所
    depth = np.full(empty.shape, -1, np.int32)
    visited = open_.copy()
    frontier = open_.copy()
    steps = int(np.ceil(30.0 / cell))
    for k in range(1, steps + 1):
        n = frontier.copy()
        n[1:] |= frontier[:-1]; n[:-1] |= frontier[1:]; n[:, 1:] |= frontier[:, :-1]; n[:, :-1] |= frontier[:, 1:]; n[:, :, 1:] |= frontier[:, :, :-1]; n[:, :, :-1] |= frontier[:, :, 1:]
        new = n & crev & ~visited
        if not new.any():
            break
        depth[new] = k; visited |= new; frontier = new
    haz = crev & (depth * cell >= depth_min)
    def clusters(m, min_cells):
        idx = np.argwhere(m)
        if len(idx) == 0:
            return []
        c = max(1, int(round(10 / cell)))
        keys = {tuple(k) for k in (idx // c)}
        seen: dict = {}; comps = []
        for k in keys:
            if k in seen:
                continue
            stack = [k]; seen[k] = len(comps); cur = [k]
            while stack:
                a = stack.pop()
                for da in (-1, 0, 1):
                    for db in (-1, 0, 1):
                        for dc in (-1, 0, 1):
                            b = (a[0] + da, a[1] + db, a[2] + dc)
                            if b in keys and b not in seen:
                                seen[b] = len(comps); stack.append(b); cur.append(b)
            comps.append(cur)
        lab = np.array([seen[tuple(k)] for k in (idx // c)])
        out = []
        for ci in range(len(comps)):
            p = idx[lab == ci]
            if len(p) < min_cells:
                continue
            mn, mx_ = p.min(0), p.max(0)
            out.append({"volume_mm3": round(len(p) * cell ** 3), "x_mm": [round(box[0] + mn[2] * cell), round(box[0] + (mx_[2] + 1) * cell)], "y_mm": [round(box[2] + mn[1] * cell), round(box[2] + (mx_[1] + 1) * cell)], "z_mm": [round(box[4] + mn[0] * cell), round(box[4] + (mx_[0] + 1) * cell)], "width_mm": [round(float(W[tuple(p.T)].min()), 1), round(float(W[tuple(p.T)].max()), 1)], "max_depth_mm": round(float(depth[tuple(p.T)].max()) * cell, 1)})
        out.sort(key=lambda d: -d["volume_mm3"])
        return out
    out = {"pose": pose, "cell_mm": cell, "box": list(box), "crevice_mm3": round(float(crev.sum()) * cell ** 3), "hazard_mm3": round(float(haz.sum()) * cell ** 3),
           "hazard_clusters": clusters(haz, max(1, int(30 / cell ** 3))), "max_depth_in_crevices_mm": round(float(depth.max()) * cell, 1)}
    if return_mask:
        out["_haz"] = haz; out["_W"] = W; out["_depth"] = depth
    return out
