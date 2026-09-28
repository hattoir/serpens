"""J1（首 pitch）の頭–首のすき間を、側面視（y = 0 の断面）で頭を 0〜45° 上げながら調べる（KINEMATIC_SIM / DESIGN_ESTIMATE）。

比べる形:
- RECOVERY: Fusion の Recovery 版と同じ考え方。頭の J1 より後ろ = r ≤ 32、首の前 = r ≥ 36 の受け、首の上の縁を r47 で逃がす
- VISOR: 提案。頭の後ろ上に頭巾（r 42〜48、角度 15〜90°）。首は J1 まわりを細い舌（r 36〜41）にし、
  頭巾の下側（角度 −30〜15°）は首の外皮（襟、r ≥ 49）が覆う。頭を上げると頭巾は襟の下へすべり込む

判定に使う量（Engineering の安全の判定ではない）:
- pinch_zone_mm2: 頭と首の両方から 12.5 mm 以内にある空き（= 幅 25 mm 以下のすき間）の面積
- max_width_mm: そのすき間の最大の幅（2 × 両側への距離の小さい方）
- 頭を上げるにつれて、幅 8〜25 mm のすき間が狭くなっていくなら「閉じる挟み込み」の候補

出典: 頭と首の断面は concept_geometry.bean_head と Fusion の NECK loft（X−161 Z10〜70、−145 Z8〜78、−120〜−58 Z7〜80）。
J1 軸 (−181.8, 32.35) は CAD.md の ASSUMED 値。
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import concept_geometry as cg  # noqa: E402

AX, AZ = cg.J1_AXIS
STEP = 0.5
HEAD = cg.bean_head()
NECK = ([-161, -145, -120, -58], [10, 8, 7, 7], [70, 78, 80, 80])


def grid():
    xs = np.arange(-90, 100, STEP)
    zs = np.arange(-36, 62, STEP)
    X, Z = np.meshgrid(xs, zs)
    return X, Z                      # J1 基準の相対座標（x は後ろ向き +）


def in_head_profile(xr, zr):
    x = xr + AX; z = zr + AZ
    zb = np.interp(x, HEAD.xs, HEAD.z_bot); zt = np.interp(x, HEAD.xs, HEAD.z_top)
    return (x >= min(HEAD.xs)) & (x <= max(HEAD.xs)) & (z >= zb) & (z <= zt)


def in_neck_profile(xr, zr):
    x = xr + AX; z = zr + AZ
    zb = np.interp(x, NECK[0], NECK[1]); zt = np.interp(x, NECK[0], NECK[2])
    return (x >= NECK[0][0]) & (x <= NECK[0][-1]) & (z >= zb) & (z <= zt)


def polar(xr, zr):
    return np.hypot(xr, zr), np.degrees(np.arctan2(zr, xr))   # 角度: 後ろ水平 0°、真上 90°


def head_mask(kind, xr, zr):
    """頭の座標（頭を上げる前）での頭の領域（y = 0 の断面）。"""
    r, th = polar(xr, zr)
    prof = in_head_profile(xr, zr)
    if kind == "RECOVERY":
        return (prof & (xr <= 0)) | (prof & (r <= 32))
    # VISOR（クレビス）: y = 0 の断面では、J1 のまわり r < 42 は首の舌（とサーボ）の場所。頭はその外側だけ。
    # 頭の後ろ上に頭巾 r42〜48（角度 15〜180°＝前の頭の本体へつながる）
    tongue_sector = ((th >= -30) & (th <= 180)) | (th <= -165)   # 舌の範囲 −30〜150° ＋ 頭を 45° 上げる分（195° = −165°）
    front = prof & (xr <= 0) & ~(tongue_sector & (r < 42))
    hood = (r >= 42) & (r <= 48) & (th >= 15) & (th <= 180)
    return front | hood


def neck_mask(kind, xr, zr):
    r, th = polar(xr, zr)
    prof = in_neck_profile(xr, zr)
    if kind == "RECOVERY":
        base = prof & (r >= 36)
        lip = (xr >= -8.2) & (xr <= 71.8) & (zr >= 12.65) & (r < 47)
        return base & ~lip
    # VISOR: 舌＋サーボの芯 r ≤ 41（角度 −30〜150°、首の外形より前へも伸びて頭の下に入る）、
    # 襟 r ≥ 49（角度 −30〜15°、首の外形の中）、それ以外の角度は首の外形そのまま
    core = (r <= 41) & (th >= -30) & (th <= 150)
    sector = (th >= -30) & (th <= 150)
    collar = prof & (r >= 49) & (th >= -30) & (th <= 15)
    far = prof & (r >= R_NECK_CLEAR)             # 頭巾の届かない外側は首の外形そのまま
    rest = prof & ~sector & (r >= 36)
    return core | collar | far | rest


def rotate_up(xr, zr, phi_deg):
    """頭を φ 上げたとき、世界の点 (xr, zr) が頭の座標ではどこか（逆回転）。前（x<0）が上がる向き。"""
    p = math.radians(phi_deg)
    # 頭の点 (x, z) → 世界 (x cos + z sin, −x sin + z cos)。その逆
    return xr * math.cos(p) - zr * math.sin(p), xr * math.sin(p) + zr * math.cos(p)


def arcs_between(kind, phi, r_values=range(20, 61, 2), th_range=(-60.0, 150.0), dth=0.5):
    """半径 r の円の上で、片側が頭・反対側が首の空き（= 頭と首の間のすき間）の弧の長さ（mm）。"""
    out = []
    ths = np.arange(th_range[0], th_range[1] + dth, dth)
    for r in r_values:
        xr = r * np.cos(np.radians(ths)); zr = r * np.sin(np.radians(ths))
        hx, hz = rotate_up(xr, zr, phi)
        head = head_mask(kind, hx, hz); neck = neck_mask(kind, xr, zr)
        lab = np.where(head, 1, np.where(neck, 2, 0))
        i = 0
        while i < len(lab):
            if lab[i] == 0:
                j = i
                while j < len(lab) and lab[j] == 0: j += 1
                left = lab[i - 1] if i > 0 else 0; right = lab[j] if j < len(lab) else 0
                if {left, right} == {1, 2}:
                    out.append((r, round(float(r * math.radians((j - i) * dth)), 1), round(float(ths[i]), 1)))
                i = j
            else:
                i += 1
    return out


def boundary(mask):
    edge = mask & ~(np.roll(mask, 1, 0) & np.roll(mask, -1, 0) & np.roll(mask, 1, 1) & np.roll(mask, -1, 1))
    return edge


PROBE_R = 4.0
R_NECK_CLEAR = 58.0   # VISOR: 首の外形を J1 からこの半径より内側で逃がす（頭頂 r≈48 と頭巾 r48 の外側）   # 指の模型の半径（直径 8 mm = 構想設計書の「8 mm 以下」の境目）


def accessible(occupied):
    """外から直径 8 mm の棒が届く空きセル（True）。占有の境界からの距離 ≥ 4 mm の中心を外周から塗りつぶし、その 4 mm 以内。"""
    from collections import deque
    H, W = occupied.shape
    occ_pts = np.argwhere(boundary(occupied)).astype(float) * STEP
    empty = np.argwhere(~occupied)
    clear = np.full(occupied.shape, False)
    if len(occ_pts):
        for chunk in np.array_split(empty, max(1, len(empty) // 4000)):
            d = np.sqrt(((chunk[:, None, :].astype(float) * STEP - occ_pts[None, :, :]) ** 2).sum(-1)).min(1)
            ok = chunk[d >= PROBE_R]
            clear[ok[:, 0], ok[:, 1]] = True
    seen = np.full(occupied.shape, False)
    dq = deque()
    for i in range(H):
        for j in (0, W - 1):
            if clear[i, j] and not seen[i, j]: seen[i, j] = True; dq.append((i, j))
    for j in range(W):
        for i in (0, H - 1):
            if clear[i, j] and not seen[i, j]: seen[i, j] = True; dq.append((i, j))
    while dq:
        i, j = dq.popleft()
        for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if 0 <= a < H and 0 <= b < W and clear[a, b] and not seen[a, b]:
                seen[a, b] = True; dq.append((a, b))
    centers = np.argwhere(seen).astype(float) * STEP
    reach = np.full(occupied.shape, False)
    if len(centers):
        for chunk in np.array_split(empty, max(1, len(empty) // 4000)):
            d = np.sqrt(((chunk[:, None, :].astype(float) * STEP - centers[None, ::3, :]) ** 2).sum(-1)).min(1)
            ok = chunk[d <= PROBE_R + STEP]
            reach[ok[:, 0], ok[:, 1]] = True
    return reach


def study(kind):
    X, Z = grid()
    neck = neck_mask(kind, X, Z)
    rows, per_r = [], {}
    for phi in range(0, 46, 5):
        hx, hz = rotate_up(X, Z, phi)
        overlap = float((head_mask(kind, hx, hz) & neck).sum() * STEP * STEP)
        head_g = head_mask(kind, hx, hz)
        reach = accessible(head_g | neck)
        z0, x0 = Z[0, 0], X[0, 0]
        def is_reach(r, th):
            xr, zr = r * math.cos(math.radians(th)), r * math.sin(math.radians(th))
            i = int(round((zr - z0) / STEP)); j = int(round((xr - x0) / STEP))
            return 0 <= i < reach.shape[0] and 0 <= j < reach.shape[1] and bool(reach[i, j])
        arcs = [a for a in arcs_between(kind, phi) if any(is_reach(a[0], a[2] + k) for k in np.arange(0.0, max(a[1] / a[0] * 180 / math.pi, 0.5), 0.5))]
        for r, L, th in arcs:
            per_r.setdefault(r, []).append((phi, L))
        band = [L for _, L, _ in arcs if 8 <= L <= 25]
        rows.append({"phi_deg": phi, "overlap_mm2": round(overlap, 1), "n_gaps": len(arcs),
                     "gaps_8_25_mm": sorted(band)[:4], "min_gap_mm": min([L for _, L, _ in arcs], default=None)})
    # 閉じるすき間: 同じ半径で、φ が増えると弧が縮み、途中で 8〜25 mm を通るもの
    closing = []
    for r, seq in per_r.items():
        seq.sort()
        Ls = [L for _, L in seq]
        if len(Ls) >= 2 and Ls[-1] < Ls[0] - 2 and any(8 <= L <= 25 for L in Ls):
            closing.append({"r_mm": r, "gap_mm_by_phi": seq[:10]})
    return {"kind": kind, "rows": rows, "closing_gaps": closing[:12], "n_closing_radii": len(closing)}


def main():
    out = {"source": "KINEMATIC_SIM（側面視 2D、y = 0 断面、0.5 mm 格子）。安全の判定ではない", "results": [study("RECOVERY"), study("VISOR")]}
    p = cg.RESULTS / "j1_interface_2026-09-29.json"
    p.write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
    for r in out["results"]:
        print(r["kind"], "closing radii:", r["n_closing_radii"])
        for row in r["rows"]:
            print("   ", row)
        for c in r["closing_gaps"][:6]:
            print("    closing", c)


if __name__ == "__main__":
    main()
