"""口のフード（漏斗 60 → 30、空間 30×30×15、壁 1.6）を頭に付けたときの照明の効きの再計算（ENTRY-D-0006 への回答）。

light_placement.py（頭の殻の影 + 点光源 h/r³）に、フードの壁・屋根・足の帯による遮りを足した。床の帯・照度比の定義は同じ
（レンズ X−234 の前 38〜147 mm、横 ±23 mm、330 点。近い側と遠い側の最大 / 最小）。GEOMETRY_SIM。配光・反射なし。
口の面 = 頭の顔の前面 x = −236 [A]。実行: .venv/Scripts/python.exe docs/design/tools/light_placement_hood.py
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import concept_geometry as cg  # noqa: E402
import light_placement as lp  # noqa: E402

MOUTH_X = -236.0
MOUTH, FUNNEL, CH_L, CH_W, WALL, FOOT, H_OUT, FOOT_H, CH_H = 60.0, 30.0, 30.0, 30.0, 1.6, 3.6, 16.6, 4.0, 15.0


def _area(p):
    return 0.5 * sum(p[i][0] * p[(i + 1) % len(p)][1] - p[(i + 1) % len(p)][0] * p[i][1] for i in range(len(p)))


def _inner():
    s = (MOUTH / 2 - CH_W / 2) / FUNNEL
    x0 = -3.0
    return [(x0, MOUTH / 2 + s * 3), (FUNNEL, CH_W / 2), (FUNNEL + CH_L, CH_W / 2), (FUNNEL + CH_L, -CH_W / 2), (FUNNEL, -CH_W / 2), (x0, -(MOUTH / 2 + s * 3))]


def _off(p, q, d, cw):
    dx, dy = q[0] - p[0], q[1] - p[1]
    L = math.hypot(dx, dy); dx /= L; dy /= L
    nx, ny = (-dy, dx) if cw else (dy, -dx)
    return ((p[0] + nx * d, p[1] + ny * d), (dx, dy))


def _isect(a, b):
    (p, d), (q, e) = a, b
    det = d[0] * (-e[1]) - d[1] * (-e[0])
    t = ((q[0] - p[0]) * (-e[1]) - (q[1] - p[1]) * (-e[0])) / det
    return (p[0] + d[0] * t, p[1] + d[1] * t)


def _outer(d, fx=0.0):
    P = _inner(); cw = _area(P) < 0
    L = [_off(P[i], P[i + 1], d, cw) for i in range(5)]
    pts = []
    p, dd = L[0]; pts.append((fx, p[1] + dd[1] * (fx - p[0]) / dd[0]))
    for i in range(4): pts.append(_isect(L[i], L[i + 1]))
    p, dd = L[4]; pts.append((fx, p[1] + dd[1] * (fx - p[0]) / dd[0]))
    return pts


def _inpoly(pt, poly):
    x, y = pt; c = False
    for i in range(len(poly)):
        x1, y1 = poly[i]; x2, y2 = poly[(i + 1) % len(poly)]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            c = not c
    return c


OW, OF, IN = _outer(WALL), _outer(FOOT), _inner()


def hood_blocks(p) -> bool:
    x, y, z = p; xl = x - MOUTH_X
    if xl < 0:
        return False
    in_cav = _inpoly((xl, y), IN) and z <= CH_H
    return ((_inpoly((xl, y), OW) and z <= H_OUT) or (_inpoly((xl, y), OF) and z <= FOOT_H)) and not in_cav


def lit_hood(src, x, y, with_hood: bool) -> bool:
    if not lp.lit(src, x, y):
        return False
    if not with_hood:
        return True
    n = int(math.dist(src, (x, y, 0)) / 0.5)
    for i in range(n + 1):
        t = i / n
        if hood_blocks((src[0] + (x - src[0]) * t, src[1] + (y - src[1]) * t, src[2] * (1 - t))):
            return False
    return True


def evaluate(sources, with_hood: bool) -> dict:
    none, emin, emax, tot = 0, 1e18, 0.0, 0
    for x in lp.XS:
        for y in lp.YS:
            tot += 1
            e = sum(s[2] / math.dist(s, (x, y, 0)) ** 3 for s in sources if lit_hood(s, x, y, with_hood))
            if e == 0:
                none += 1
            else:
                emin, emax = min(emin, e), max(emax, e)
    return {"unlit_points": f"{none}/{tot}", "near_far_ratio": round(emax / emin, 1) if emin < 1e17 else None}


def pair(x, y, z):
    return [(x, y, z), (x, -y, z)]


CASES = [
    ("斜め ① 現行（頬の下 X−212 |y|40 Z7）: フードなし", pair(-212.0, 40.0, 7.0), False),
    ("斜め ② 現行位置 + フード", pair(-212.0, 40.0, 7.0), True),
    ("斜め ③ 案: 口の前の外側の角（X−236.5 |y|36 Z6）+ フード", pair(-236.5, 36.0, 6.0), True),
    ("斜め ④ 案: 口の前の外側の角（X−234 |y|38 Z6）+ フード", pair(-234.0, 38.0, 6.0), True),
    ("斜め ⑤ 案: X−236.5 |y|36 Z4（低め）+ フード", pair(-236.5, 36.0, 4.0), True),
    ("斜め ⑥ 案: X−236.5 |y|36 Z9（高め）+ フード", pair(-236.5, 36.0, 9.0), True),
    ("通常 ① 現行（口の線 X−220 |y|44.7 Z21）: フードなし", pair(-220.0, 44.7, 21.0), False),
    ("通常 ② 現行位置 + フード", pair(-220.0, 44.7, 21.0), True),
    ("通常 ③ 案: 口の線の前端 X−232 |y|41 Z21 + フード", pair(-232.0, 41.0, 21.0), True),
    ("通常 ④ 案: 口の線の前端 X−234 |y|38 Z21 + フード", pair(-234.0, 38.0, 21.0), True),
]


def main() -> None:
    res = {"source": "GEOMETRY_SIM（点光源 h/r³、頭の影 + フードの遮り。配光なし）", "mouth_x": MOUTH_X, "cases": {}}
    for name, srcs, hood in CASES:
        r = evaluate(srcs, hood)
        res["cases"][name] = {"sources_mm": [[round(v, 1) for v in s] for s in srcs], "with_hood": hood, **r}
        print(name, r, flush=True)
    (cg.RESULTS / "light_placement_hood_2026-09-30.json").write_text(json.dumps(res, ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
