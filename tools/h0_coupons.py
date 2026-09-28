r"""H0 摩擦クーポン（腹面インサート候補の試験片）の STL を作る。**CONCEPT。印刷・実測はまだ。**

    .\.venv\Scripts\python.exe tools\h0_coupons.py
    → hardware/prototypes/H0_friction/stl/H0_{FLAT,SCALE,KEEL}.stl

形状は高さ場（heightfield）で作る。上面は平ら、下面（床に当たる面）に模様を付け、外周は下面を丸めて持ち上げる
（カーペットの毛に角が引っかかって「横だけ食いつく」見かけの異方性が出ないように）。
+x が機体の前（頭の方向）。上面に彫った矢印が前を向く。
**上面をベッドに付けて裏返しで印刷する**（床に当たる模様の面が上になり、歯と外周の丸めがサポート無しで出る）。

  FLAT  … 平らな腹（PETG / TPU で印刷して材料の差を見る）
  SCALE … 横向きの非対称のこぎり歯（前へは緩い斜面、後ろへは立った面）= 鱗の向き。前後の差を見る
  KEEL  … 前後方向の細い畝（横ずれを畝で止める）。柔らかい床（ラグ・カーペット）で横の食いつきを見る

寸法は `hardware/prototypes/H0_friction/README.md` の表と同じ（ここが正本）。
"""
from __future__ import annotations

import argparse
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

LENGTH_MM = 60.0          # x（前後）
WIDTH_MM = 40.0           # y（横）
THICK_MM = 4.0            # 平らな部分の厚み（下面の基準 z=0 → 上面 z=4）
EDGE_ROUND_MM = 3.0       # 外周の下面を持ち上げる幅（1/4 円）
RES_MM = 0.25             # 高さ場の刻み
ARROW_DEPTH_MM = 0.6      # 上面の矢印の彫りの深さ（盛らない: 上面を平らにして、上面をベッドに付けて裏返しで印刷する）


@dataclass(frozen=True)
class Pattern:
    name: str
    pitch_mm: float
    depth_mm: float
    note: str


PATTERNS = {
    "FLAT": Pattern("FLAT", 0.0, 0.0, "平ら"),
    "SCALE": Pattern("SCALE", 3.0, 0.6, "横向きのこぎり歯。前へ緩い斜面（2.5mm）、後ろへ立った面（0.5mm）"),
    "KEEL": Pattern("KEEL", 4.0, 0.8, "前後方向の三角の畝。頂点の丸め無し（印刷の 0.4mm ノズルで自然に丸まる）"),
}


def bottom_height(p: Pattern, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """下面の高さ [mm]（0 が床に当たる面。模様の谷と外周の丸めは + 側）。"""
    z = np.zeros_like(x)
    if p.name == "SCALE":
        u = np.mod(x, p.pitch_mm)
        ramp = p.pitch_mm - 0.5
        # 前（+x）へ行くほど下面が床から離れる緩い斜面 → 立った面で床へ戻る（後ろ向きの動きで引っかかる）
        z = np.where(u < ramp, u / ramp * p.depth_mm, (p.pitch_mm - u) / 0.5 * p.depth_mm)
    elif p.name == "KEEL":
        v = np.mod(y, p.pitch_mm) / p.pitch_mm
        z = np.abs(v - 0.5) * 2.0 * p.depth_mm            # 畝の頂点で 0、谷で depth
    # 外周の丸め（辺からの距離 d < R で 1/4 円ぶん持ち上げる）
    d = np.minimum.reduce([x, LENGTH_MM - x, y, WIDTH_MM - y])
    r = EDGE_ROUND_MM
    lift = np.where(d < r, r - np.sqrt(np.clip(r * r - (r - d) ** 2, 0.0, None)), 0.0)
    return np.minimum(z + lift, THICK_MM - 1.0)


def top_height(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """上面 [mm]。前を指す矢印（三角形）を ARROW_DEPTH_MM だけ彫る。"""
    cx, cy = LENGTH_MM * 0.5, WIDTH_MM * 0.5
    tip, base, half = cx + 15.0, cx - 10.0, 9.0
    inside = (x >= base) & (x <= tip) & (np.abs(y - cy) <= half * (tip - x) / (tip - base))
    return np.where(inside, THICK_MM - ARROW_DEPTH_MM, THICK_MM)


def heightfield_mesh(zb: np.ndarray, zt: np.ndarray, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    """下面・上面の高さ場から閉じた三角形メッシュ（N×3×3）を作る。外向きの法線になる順で並べる。"""
    nx, ny = len(xs), len(ys)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    top = np.stack([X, Y, zt], axis=-1)
    bot = np.stack([X, Y, zb], axis=-1)
    tris: list[np.ndarray] = []

    def quads(g: np.ndarray, flip: bool) -> None:
        a, b, c, d = g[:-1, :-1], g[1:, :-1], g[1:, 1:], g[:-1, 1:]
        t1 = np.stack([a, b, c], axis=-2).reshape(-1, 3, 3)
        t2 = np.stack([a, c, d], axis=-2).reshape(-1, 3, 3)
        t = np.concatenate([t1, t2])
        tris.append(t[:, ::-1] if flip else t)

    quads(top, flip=False)                 # 上面: 法線 +z
    quads(bot, flip=True)                  # 下面: 法線 −z

    def wall(tline: np.ndarray, bline: np.ndarray, flip: bool) -> None:
        a, b = tline[:-1], tline[1:]
        c, d = bline[1:], bline[:-1]
        t = np.concatenate([np.stack([a, d, c], 1), np.stack([a, c, b], 1)])
        tris.append(t[:, ::-1] if flip else t)

    wall(top[:, 0], bot[:, 0], flip=False)       # y = 0 の側面
    wall(top[:, -1], bot[:, -1], flip=True)      # y = W
    wall(top[0, :], bot[0, :], flip=True)        # x = 0
    wall(top[-1, :], bot[-1, :], flip=False)     # x = L
    del nx, ny
    return np.concatenate(tris)


def coupon_mesh(p: Pattern, res_mm: float = RES_MM) -> np.ndarray:
    xs = np.linspace(0.0, LENGTH_MM, int(round(LENGTH_MM / res_mm)) + 1)
    ys = np.linspace(0.0, WIDTH_MM, int(round(WIDTH_MM / res_mm)) + 1)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    return heightfield_mesh(bottom_height(p, X, Y), top_height(X, Y), xs, ys)


def signed_volume_mm3(tris: np.ndarray) -> float:
    """閉じた面の体積（発散定理）。外向きに並んでいれば正。"""
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)


def write_binary_stl(path: Path, tris: np.ndarray, header: str) -> None:
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    rec = np.zeros(len(tris), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, tris
    with path.open("wb") as fp:
        fp.write(header.encode("ascii", "replace")[:80].ljust(80, b" "))
        fp.write(struct.pack("<I", len(tris)))
        fp.write(rec.tobytes())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path("hardware/prototypes/H0_friction/stl"))
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    for p in PATTERNS.values():
        tris = coupon_mesh(p)
        path = args.out / f"H0_{p.name}.stl"
        write_binary_stl(path, tris, f"Serpens H0 coupon {p.name} +x=forward mm NOT_VERIFIED")
        print(f"{path}  {len(tris)} tris  volume {signed_volume_mm3(tris) / 1000:.2f} cm3")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
