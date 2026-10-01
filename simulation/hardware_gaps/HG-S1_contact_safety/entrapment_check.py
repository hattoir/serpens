r"""囲い込み・挟まり（entrapment）検査: 胴 + 頭ヨーの曲げで、子どもの体の部位（指・手首・前腕・首）が**抜けなくなる袋小路**ができるか（LB-E-012 / SE-E5 / USER-DEC-SERPENS-0004）。
**GEOMETRY_SIM（平面・剛体）。CAD_CONCEPT の寸法。実物・実機で未確認。HARDWARE_VERIFIED = 0。5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）で、ここでは使わない。**

    .\.venv\Scripts\python.exe simulation\hardware_gaps\HG-S1_contact_safety\entrapment_check.py

モデル（真上から見た平面。胴と頭を、軸の間のリンクのカプセル = 線分 + 半径で置く）:
  - 寸法（Design の CAD.md、CAD_CONCEPT）: ヨー軸 J2〜J5 は X = −95 / 0 / 95 / 190（軸間 95）、J1（ピッチ）X = −181.8、外装は X −238〜335（全長 573）、胴外装の幅 92
    → 尾端 → J5 = 145、J5→J4 = J4→J3 = J3→J2 = 95、J2 → 頭ヨー軸（J1 の位置）= 86.8、頭ヨー軸 → 頭の先 = 56.2 mm。頭の幅 100 mm（Design の OPEN-001 の記述。**ASSUMED**）
  - 関節: 胴ヨー 4 本（J5, J4, J3, J2。ソフト ±50°）+ 頭ヨー（6 本目の候補。±15 / 30 / 45 / 60°）。連続する胴ヨーの和の最大 ≤ 145°（USER-DEC-0004: 頭ヨーは 145° に含めない独立の軸）
  - 「抜けなくなる」= 直径 d の円（子どもの部位の断面）を、体に触れずに置けて、体に触れずに平面内を動いても外へ出られない（動ける領域が閉じている）。
    円の中心が動ける領域 = {距離変換 ≥ d / 2} の連結成分のうち、画面の端に届かないもの。
    **袋小路は 2 つの数で決まる**: 口の幅 m（これより太い円は出入りできない）と、内側の最大の円の直径 D_in。m < d ≤ D_in の円が抜けなくなる。
  - 部位の太さ（Snyder 1977 の生データ。**2 歳未満は出典なし**）: 指 8.3〜12.7、手首 29.3〜43.3、前腕 41.7〜61.8、首 63.7〜88.5 mm（`child_anthropometry.yaml`）
限界: 平面だけ（高さ方向に持ち上げて抜ける動きは見ない = 抜けられるものも「抜けない」と数える側 = 保守側）。変形（指・外装の柔らかさ）・摩擦・関節の動的な挙動は入れない。
        床の上に寝ている前提。体が持ち上げられた形・とぐろ・自己干渉は別に数える。画素の量子化（粗い走査 2 mm/px・刻み 4 mm、確認 1 mm/px・刻み 1 mm）の誤差がある。
"""
from __future__ import annotations

import itertools
import json
import math
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
LINKS_MM = (145.0, 95.0, 95.0, 95.0, 86.8, 56.2)       # 尾端→J5, J5→J4, J4→J3, J3→J2, J2→頭ヨー軸, 頭ヨー軸→先端
RADII_MM = (46.0, 46.0, 46.0, 46.0, 46.0, 50.0)        # 胴の幅 92（半径 46）、頭の幅 100（半径 50。ASSUMED）
BODY_YAW_SOFT_DEG = 50.0
BODY_SUM_MAX_DEG = 145.0
PARTS_MM = {"指": (8.3, 12.7), "手首": (29.3, 43.3), "前腕": (41.7, 61.8), "首": (63.7, 88.5), "胸": (138.2, 211.0)}
HEAD_YAW_DEG = (0.0, 15.0, 30.0, 45.0, 60.0)
MARGIN_MM = 140
COARSE = {"px": 2.0, "step": 4.0}                      # 全部の形の走査
FINE = {"px": 1.0, "step": 1.0}                        # 見つかった形の確認


def chain_points(body_deg, head_yaw_deg: float = 0.0) -> np.ndarray:
    """尾端から頭の先までの点列（7 点）。関節角は J5, J4, J3, J2, 頭ヨー（点 1〜5 での曲がり）。正 = 左。"""
    angles = list(body_deg) + [head_yaw_deg]
    heading = 0.0
    pts = [np.zeros(2)]
    for i, ln in enumerate(LINKS_MM):
        d = np.array([math.cos(math.radians(heading)), math.sin(math.radians(heading))])
        pts.append(pts[-1] + d * ln)
        if i < len(angles):
            heading += angles[i]
    return np.array(pts)


def max_contiguous_sum(values) -> float:
    best = run_pos = run_neg = 0.0
    for x in values:
        run_pos = max(0.0, run_pos + x)
        run_neg = min(0.0, run_neg + x)
        best = max(best, run_pos, -run_neg)
    return best


def seg_dist(p0, p1, q0, q1) -> float:
    """2 線分の最短距離（2D）。"""
    def pt_seg(p, a, b):
        ab = b - a
        t = 0.0 if not ab.any() else max(0.0, min(1.0, float((p - a) @ ab) / float(ab @ ab)))
        return float(np.linalg.norm(p - (a + t * ab)))

    def cross(a, b, c, d):
        r, s = b - a, d - c
        den = r[0] * s[1] - r[1] * s[0]
        if abs(den) < 1e-12:
            return False
        t = ((c[0] - a[0]) * s[1] - (c[1] - a[1]) * s[0]) / den
        u = ((c[0] - a[0]) * r[1] - (c[1] - a[1]) * r[0]) / den
        return 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0
    if cross(p0, p1, q0, q1):
        return 0.0
    return min(pt_seg(p0, q0, q1), pt_seg(p1, q0, q1), pt_seg(q0, p0, p1), pt_seg(q1, p0, p1))


def clearances(pts: np.ndarray, radii=RADII_MM) -> dict:
    """3 つ以上離れたリンクどうしのすき間（カプセルの表面間 [mm]。負 = 重なり = 自己干渉）の最小。頭（リンク 5）と体の最小も別に。
    1 つおき（j − i = 2）は数えない: 間の短いリンクが半径の和より短いと、丸い端のカプセルが直線でも重なる（モデルの人工物。胴は 1 枚の殻）。"""
    best, best_head = math.inf, math.inf
    for i, j in itertools.combinations(range(len(LINKS_MM)), 2):
        if j - i < 3:
            continue
        g = seg_dist(pts[i], pts[i + 1], pts[j], pts[j + 1]) - radii[i] - radii[j]
        best = min(best, g)
        if j == 5:
            best_head = min(best_head, g)
    return {"min_nonadjacent": best, "min_head_to_body": best_head}


def raster(pts: np.ndarray, radii=RADII_MM, px: float = COARSE["px"]) -> np.ndarray:
    """カプセル列を px mm / 画素で塗る。返り値 = 自由 = 1 のマスク。"""
    lo = pts.min(axis=0) - (max(radii) + MARGIN_MM)
    hi = pts.max(axis=0) + (max(radii) + MARGIN_MM)
    W, H = int(math.ceil((hi[0] - lo[0]) / px)), int(math.ceil((hi[1] - lo[1]) / px))
    img = np.zeros((H, W), np.uint8)

    def pp(p):
        return (int(round((p[0] - lo[0]) / px)), int(round((p[1] - lo[1]) / px)))
    for i in range(len(LINKS_MM)):
        r_px = max(1, int(round(radii[i] / px)))
        cv2.line(img, pp(pts[i]), pp(pts[i + 1]), 1, 2 * r_px)
        cv2.circle(img, pp(pts[i]), r_px, 1, -1)
        cv2.circle(img, pp(pts[i + 1]), r_px, 1, -1)
    return 1 - img


def dist_mm(free: np.ndarray, px: float) -> np.ndarray:
    """自由な画素から体までの距離 [mm]。"""
    return cv2.distanceTransform(free, cv2.DIST_L2, 5) * px


def trapped(dt: np.ndarray, d_mm: float) -> bool:
    """直径 d の円が、体に触れずに置けて、平面内で外へ出られない領域があるか（dt = 距離変換 [mm]）。"""
    comp = (dt >= d_mm / 2.0).astype(np.uint8)
    if not comp.any():
        return False
    n, lab = cv2.connectedComponents(comp, connectivity=8)
    edge = set(np.unique(np.concatenate([lab[0, :], lab[-1, :], lab[:, 0], lab[:, -1]]))) - {0}
    return any(k not in edge for k in range(1, n))


def trap_diameters(pts: np.ndarray, radii=RADII_MM, px: float = COARSE["px"], step: float = COARSE["step"], d_hi: float = 140.0) -> list[float]:
    """抜けなくなる円の直径（step 刻み）のリスト。空 = 袋小路なし。外へ出られない領域が残る間だけ走査する。"""
    dt = dist_mm(raster(pts, radii, px), px)
    out = []
    d = step
    top = float(dt.max()) * 2.0
    while d <= min(d_hi, top):
        if trapped(dt, d):
            out.append(d)
        d += step
    return out


def pocket(pts: np.ndarray, radii=RADII_MM) -> dict | None:
    """袋小路の 2 つの数: 口の幅 m（抜けなくなる最小の直径の手前）と内側の最大の円 D_in（細かい走査）。なければ None。"""
    ds = trap_diameters(pts, radii, FINE["px"], FINE["step"])
    if not ds:
        return None
    return {"mouth_mm": max(ds[0] - FINE["step"], 0.0), "d_in_mm": ds[-1], "trap_range_mm": (ds[0], ds[-1])}


GRID_DEG = (-50.0, -45.0, -36.25, -25.0, 0.0, 25.0, 36.25, 45.0, 50.0)      # 36.25 = 145 / 4（4 軸が同じ向きに 145° ちょうど）、45 + 50 + 50 = 145
WRAP_MIN_DEG = 100.0                                                       # 袋小路は大きく囲む形にしかできない。囲む角（胴 + 頭ヨー）がこれ未満の形は走査しない（小さい形は含まれる）


def body_configs(step_deg: float | None = None) -> list[tuple[float, ...]]:
    """胴ヨー 4 本の格子（連続した和の最大 ≤ 145°）。step_deg を渡すと等間隔の格子（試験用）、なければ 145° ちょうどを含む格子 GRID_DEG。"""
    vals = GRID_DEG if step_deg is None else tuple(float(x) for x in np.arange(-BODY_YAW_SOFT_DEG, BODY_YAW_SOFT_DEG + 1e-9, step_deg))
    return [tuple(float(x) for x in c) for c in itertools.product(vals, repeat=4) if max_contiguous_sum(c) <= BODY_SUM_MAX_DEG + 1e-9]


def sweep(head_yaw_deg: float, cfgs=None) -> dict:
    """ある頭ヨー角で、胴の形ごとに走査する（粗い走査で候補 → 見つかった形だけ細かい走査）。"""
    cfgs = cfgs if cfgs is not None else body_configs()
    cfgs = [c for c in cfgs if max_contiguous_sum(list(c) + [head_yaw_deg]) >= WRAP_MIN_DEG]
    wrap_max, clr, clr_h, overlap = 0.0, math.inf, math.inf, 0
    pockets = []
    for c in cfgs:
        pts = chain_points(c, head_yaw_deg)
        wrap_max = max(wrap_max, max_contiguous_sum(list(c) + [head_yaw_deg]))
        k = clearances(pts)
        clr, clr_h = min(clr, k["min_nonadjacent"]), min(clr_h, k["min_head_to_body"])
        overlap += int(k["min_nonadjacent"] < 0)
        if trap_diameters(pts):
            pk = pocket(pts)
            if pk:
                pockets.append({"body": c, "head_yaw": head_yaw_deg, **pk, "gap_head_body": k["min_head_to_body"], "wrap": max_contiguous_sum(list(c) + [head_yaw_deg])})
    return {"n": len(cfgs), "wrap_max_deg": wrap_max, "min_nonadjacent_mm": clr, "min_head_to_body_mm": clr_h, "self_overlap": overlap, "pockets": pockets}


def sweep_signed(head_yaw_deg: float) -> dict:
    """並列実行用（プロセスごとに格子を作る）。"""
    return sweep(head_yaw_deg, body_configs())


def parts_trapped(pk: dict) -> list[str]:
    """袋小路の（口の幅, 内側の最大）と重なる太さの部位。口より太く、内側に入る太さ = 抜けなくなる。"""
    out = []
    for name, (lo, hi) in PARTS_MM.items():
        if hi > pk["mouth_mm"] and lo <= pk["d_in_mm"]:
            out.append(f"{name}（{max(lo, pk['mouth_mm']):.0f}〜{min(hi, pk['d_in_mm']):.0f} mm の太さ）")
    return out


def swept_area_cm2(head_yaw_max_deg: float, step: float = 1.0) -> float:
    """胴をまっすぐにして、頭ヨーを ±φ の範囲で振ったときに頭が胴の外で掃く面積 [cm²]（頭のカプセルの和集合 − 胴）。"""
    pts0 = chain_points((0.0, 0.0, 0.0, 0.0), 0.0)
    W, H, ox, oy = 900, 700, 300, 300

    def pp(p):
        return (int(round(p[0] + ox)), int(round(p[1] + oy)))
    union = np.zeros((H, W), np.uint8)
    for a in np.arange(-head_yaw_max_deg, head_yaw_max_deg + 1e-9, step):
        pts = chain_points((0.0, 0.0, 0.0, 0.0), float(a))
        cv2.line(union, pp(pts[5]), pp(pts[6]), 1, int(round(2 * RADII_MM[5])))
        cv2.circle(union, pp(pts[5]), int(round(RADII_MM[5])), 1, -1)
        cv2.circle(union, pp(pts[6]), int(round(RADII_MM[5])), 1, -1)
    body = np.zeros((H, W), np.uint8)
    for i in range(5):
        cv2.line(body, pp(pts0[i]), pp(pts0[i + 1]), 1, int(round(2 * RADII_MM[i])))
        cv2.circle(body, pp(pts0[i]), int(round(RADII_MM[i])), 1, -1)
        cv2.circle(body, pp(pts0[i + 1]), int(round(RADII_MM[i])), 1, -1)
    return float(((union == 1) & (body == 0)).sum()) / 100.0


def main() -> None:
    cfgs = body_configs()
    L: list[str] = []
    A = L.append
    A("# 囲い込み・挟まり（entrapment）検査: 胴 + 頭ヨーの曲げ（2026-10-01）\n")
    A("**GEOMETRY_SIM（平面・剛体・CAD_CONCEPT の寸法）。実物・実機で未確認（HARDWARE_VERIFIED = 0）。5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）で、ここでは使わない。安全・合格の語は使わない。** 再現: `simulation/hardware_gaps/HG-S1_contact_safety/entrapment_check.py`。\n")
    A(f"形: 胴ヨー 4 本（J5, J4, J3, J2）を {{−50, −45, −36.25, −25, 0, 25, 36.25, 45, 50}}° の格子（**145° ちょうどの形を含む**。囲む角 100° 以上の形だけ走査）、連続した和の最大 ≤ {BODY_SUM_MAX_DEG:g}°（USER-DEC-0004: 頭ヨーは 145° に含めない独立の軸）で {len(cfgs)} 通り × 頭ヨー ±0 / 15 / 30 / 45 / 60°（左右）。"
      "「抜けなくなる」= 直径 d の円が体に触れずに置けて、平面内で外へ出られない領域がある。**袋小路 = 口の幅 m より太く、内側の最大の円 D_in 以下の太さ**。"
      "部位の太さ（Snyder 1977 の生データ。2 歳未満は出典なし）: 指 8.3〜12.7、手首 29.3〜43.3、前腕 41.7〜61.8、首 63.7〜88.5、胸 138.2〜211.0 mm。\n")
    A("## 0. 検出器の確認（対照）\n")
    straight = chain_points((0.0, 0.0, 0.0, 0.0), 0.0)
    A(f"- まっすぐ: 抜けなくなる円 = {'あり' if trap_diameters(straight) else 'なし'}（なし = 正しい）。")
    slender = (5.0,) * 6
    loop = chain_points((90.0, 90.0, 90.0, 90.0), 0.0)
    pk = pocket(loop, slender)
    A(f"- **対照（細い体 = 半径 5 mm で、胴ヨーを 90° ずつ 4 本 = 360° 巻いた閉じた輪）**: 袋小路 = {pk}（検出器が閉じた輪の内側を見つける確認。実機の体では作れない形）。")
    c0 = chain_points((50.0, 50.0, 45.0, 0.0), 0.0)
    A(f"- 参考（この製品の最大の胴の曲げ 145°。頭ヨーなし）: 抜けなくなる円 = {'あり' if trap_diameters(c0) else 'なし'}。\n")
    A("## 1. 頭ヨーの範囲ごとの結果（USER-DEC-0004 の掃引。粗い走査 2 mm/px・刻み 4 mm → 見つかった形は細かい走査 1 mm/px・刻み 1 mm で確認）\n")
    A("| 頭ヨー ±φ | 形の数 | 最大の囲む角 [°]（胴 + 頭）| 袋小路になる形 | 頭と胴（3 リンク以上離れた）の最小すき間 [mm] | 自己干渉の形 |\n|---|---|---|---|---|---|")
    allp = []
    jobs = [s for phi in HEAD_YAW_DEG for s in ((phi,) if phi == 0 else (phi, -phi))]
    with ProcessPoolExecutor(max_workers=4) as ex:
        done = dict(zip(jobs, ex.map(sweep_signed, jobs)))
    per_phi: dict[float, dict] = {}
    for phi in HEAD_YAW_DEG:
        signs = (phi,) if phi == 0 else (phi, -phi)
        n = ov = 0
        wrap, clr_h = 0.0, math.inf
        pks = []
        for s in signs:
            r = done[s]
            n += r["n"]
            ov += r["self_overlap"]
            wrap = max(wrap, r["wrap_max_deg"])
            clr_h = min(clr_h, r["min_head_to_body_mm"])
            pks += r["pockets"]
        allp += pks
        per_phi[phi] = {"n": n, "wrap": wrap, "pockets": pks, "clr_h": clr_h}
        A(f"| ±{phi:g} | {n} | {wrap:.0f} | {len(pks)} | {clr_h:.1f} | {ov} |")
    A("")
    A("### 頭ヨーの範囲ごとの要約（袋小路の口の幅と、首の太い側 88.5 mm との差）\n")
    A("| 頭ヨー ±φ | 袋小路の形 | 口の幅の最小 [mm] | 首の太い側 88.5 mm との差 [mm]（口 − 88.5。小さいほど余裕が無い）| 内側の最大の円の最大 [mm] | 抜けなくなる部位（形の数）|\n|---|---|---|---|---|---|")
    summary = {}
    for phi in HEAD_YAW_DEG:
        pks = per_phi[phi]["pockets"]
        if not pks:
            A(f"| ±{phi:g} | 0 | — | — | — | なし |")
            summary[phi] = None
            continue
        mouth = min(p["mouth_mm"] for p in pks)
        d_in = max(p["d_in_mm"] for p in pks)
        cnt: dict[str, int] = {}
        for p in pks:
            for nm in parts_trapped(p):
                cnt[nm.split("（")[0]] = cnt.get(nm.split("（")[0], 0) + 1
        A(f"| ±{phi:g} | {len(pks)} | {mouth:.0f} | {mouth - PARTS_MM['首'][1]:+.1f} | {d_in:.0f} | " + ("、".join(f"{k}（{v}）" for k, v in cnt.items()) or "なし") + " |")
        summary[phi] = {"pockets": len(pks), "min_mouth_mm": mouth, "margin_to_neck_mm": mouth - PARTS_MM["首"][1], "max_d_in_mm": d_in, "parts": cnt}
    (ROOT / "simulation" / "results" / "entrapment_check_2026-10-01.json").write_text(json.dumps({"summary": summary, "pockets": allp}, ensure_ascii=False, default=float, indent=1), encoding="utf-8")
    A("")
    if allp:
        A("## 2. 袋小路になった形（口の幅 m と内側の最大の円 D_in。細かい走査）\n")
        A("| 頭ヨー [°] | 胴 J5, J4, J3, J2 [°] | 最大の囲む角 [°] | 口の幅 m [mm] | 内側の最大の円 D_in [mm] | 頭と胴の最小すき間 [mm] | 抜けなくなる部位（m < 太さ ≤ D_in）|\n|---|---|---|---|---|---|---|")
        for p in sorted(allp, key=lambda x: -x["d_in_mm"]):
            A(f"| {p['head_yaw']:+g} | {', '.join(f'{b:+g}' for b in p['body'])} | {p['wrap']:.0f} | {p['mouth_mm']:.0f} | {p['d_in_mm']:.0f} | {p['gap_head_body']:.1f} | {'、'.join(parts_trapped(p)) or 'なし'} |")
        A("")
    A("## 3. 頭ヨーが掃く面積（胴をまっすぐにして、頭ヨーを ±φ で振ったときに、頭が胴の外で掃く面積）\n")
    A("| ±φ | 掃く面積 [cm²] | 頭の先の横の移動 [mm]（56.2 sin φ）|\n|---|---|---|")
    for phi in HEAD_YAW_DEG[1:]:
        A(f"| ±{phi:g} | {swept_area_cm2(phi):.0f} | {LINKS_MM[5] * math.sin(math.radians(phi)):.0f} |")
    A("")
    A("## 4. 読み方と限界\n")
    by_phi = {phi: [p for p in allp if abs(p["head_yaw"]) == phi] for phi in HEAD_YAW_DEG}
    first = next((phi for phi in HEAD_YAW_DEG if by_phi[phi]), None)
    hits = [(p, parts_trapped(p)) for p in allp]
    hit_parts = sorted({n.split("（")[0] for _p, ns in hits for n in ns})
    if first is None:
        A("- **この形の格子では、袋小路（口が内側より細い領域）は見つからなかった。**")
    else:
        A(f"- **袋小路（口の幅が内側の最大の円より細い領域）が最初に現れる頭ヨーの範囲は ±{first:g}°**（この格子・この幅の前提。それより小さい範囲では見つからなかった）。"
          f"胴が同じ向きに大きく曲がり、頭ヨーも同じ向きに切ったとき（囲む角 {min(p['wrap'] for p in allp):.0f}〜{max(p['wrap'] for p in allp):.0f}°）に、頭の先と胴のあいだに口が狭い領域ができる。")
        mouths = sorted(p["mouth_mm"] for p in allp)
        parts_text = (f"**部位の太さ（指・手首・前腕・首・胸）のうち、口より太く内側に入るものは {'、'.join(hit_parts)}**。" if hit_parts else
                      "**部位の太さ（指・手首・前腕・首・胸）で、口より太く内側に入るものは、この格子では無かった**（首の太い側 88.5 mm は口 92 mm 以上より細く、"
                      "胸の細い側 138.2 mm は内側の最大の円との差が数 mm 以内 = 画素の誤差 ±1〜2 mm の範囲で、余裕は無い）。")
        A(f"- 見つかった袋小路の口の幅は {mouths[0]:.0f}〜{mouths[-1]:.0f} mm、内側の最大の円は {min(p['d_in_mm'] for p in allp):.0f}〜{max(p['d_in_mm'] for p in allp):.0f} mm（表 2）。"
          + parts_text + " Primary target（3 歳未満）の寸法は出典が無い（2 歳未満は空白）。")
    A("- 最小すき間の列は、**頭の先と胴のリンクの近づき方**（3 つ以上離れたリンクだけ。隣の隣は丸い端が重なるモデルの人工物なので数えない）。")
    A("- **平面・剛体・格子の形だけ**。(1) 高さ方向に持ち上げて抜ける動きは見ない（保守側）。(2) 形の格子は 25° 刻みで、145° を満たす形の全部ではない。(3) 頭の幅 100 mm・頭ヨー軸の位置（J1 の位置）は ASSUMED。(4) 2 歳未満の太さは出典なし。"
      "(5) 画素の量子化の誤差 ±1〜2 mm（口の幅・内側の最大の円）。(6) 外力で ±55°（機械ストッパー）まで押し込まれる最悪、とぐろ、持ち上げた形は入っていない。**実物の挟み込み（HA-06）と実測が要る。**")
    A("- **Head Yaw の最小の範囲の採否は Human Approval**（USER-DEC-0004: ±15〜60° を掃引して最小の範囲を採用）。この表は材料であって、決定ではない。袋小路が見つからないことも、見つかることも、安全の確定ではない。5〜12 mm の指のすき間と関節のくさび（脇の V）は別の検査。")
    out = ROOT / "simulation" / "results" / "entrapment_check_2026-10-01.md"
    out.write_text("\n".join(L) + "\n", encoding="utf-8", newline="\n")
    print("\n".join(L))
    print("wrote", out)


if __name__ == "__main__":
    main()
