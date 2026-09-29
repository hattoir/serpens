"""頭の照明の位置の比較（ENTRY-0019 への Design の回答、GEOMETRY_SIM）。

Engineering の VIS-0007 と同じ点光源の考え方: 床の照度 E ∝ h / r³（光源の高さ h、床の点までの距離 r）。
光源の向き・配光は入れない（Engineering の結果で「向けても効かない」）。
床の帯: レンズ（X −234）の前 38〜147 mm、横 ±23 mm。頭の形は SD-01A（concept_geometry.bean_head）。
「頭に当たる」: 光源から床の点への直線が、いったん頭の殻の外へ出たあと、また殻に入るもの（鼻先の影）。

実行:  .venv/Scripts/python.exe docs/design/tools/light_placement.py
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import concept_geometry as cg  # noqa: E402

H = cg.bean_head()
LENS_X = -234.0
XS = LENS_X - np.linspace(38, 147, 30)
YS = np.linspace(-23, 23, 11)


def half_width(px: float, pz: float) -> float:
    w, zb, zt = H.section(px)
    h = (zt - zb) / 2; zc = (zt + zb) / 2
    u = abs(pz - zc) / h if h > 0 else 2
    return w * (1 - u ** H.n_super) ** (1 / H.n_super) if u <= 1 else 0.0


def inside(px: float, py: float, pz: float) -> bool:
    return (min(H.xs) <= px <= max(H.xs)) and abs(py) <= half_width(px, pz)


def lit(src, x, y) -> bool:
    left = False
    for t in np.linspace(0, 1, 300):
        p = (src[0] + (x - src[0]) * t, src[1] + (y - src[1]) * t, src[2] * (1 - t))
        ins = inside(*p)
        if not left and not ins:
            left = True
        elif left and ins:
            return False
    return True


def evaluate(sources) -> dict:
    none, emin, emax, tot = 0, 1e18, 0.0, 0
    for x in XS:
        for y in YS:
            tot += 1
            e = sum(s[2] / math.dist(s, (x, y, 0)) ** 3 for s in sources if lit(s, x, y))
            if e == 0:
                none += 1
            else:
                emin, emax = min(emin, e), max(emax, e)
    return {"unlit_points": f"{none}/{tot}", "near_far_ratio": round(emax / emin, 1)}


CASES = {
    "通常: レンズ横 1 灯（FW03 の IR/白 LED の位置）": [(-232.0, 14.0, 28.0)],
    "通常: 口の線 X−220 左右 2 灯（Design 案）": [(-220.0, half_width(-220, 21) + 1, 21.0), (-220.0, -(half_width(-220, 21) + 1), 21.0)],
    "通常: 口の線 X−209 左右 2 灯（下げすぎ）": [(-209.0, half_width(-209, 21) + 1, 21.0), (-209.0, -(half_width(-209, 21) + 1), 21.0)],
    "斜め: あご X−232 左右 2 灯（FW03）": [(-232.0, 18.0, 7.0), (-232.0, -18.0, 7.0)],
    "斜め: 頬の下 X−212 y±40 左右 2 灯（Design 案）": [(-212.0, 40.0, 7.0), (-212.0, -40.0, 7.0)],
}


def main() -> None:
    res = {"source": "GEOMETRY_SIM（点光源 h/r³、配光なし）", "cases": {}}
    for name, srcs in CASES.items():
        res["cases"][name] = {"sources_mm": [[round(v, 1) for v in s] for s in srcs], **evaluate(srcs)}
        print(name, res["cases"][name])
    (cg.RESULTS / "light_placement_2026-09-29.json").write_text(json.dumps(res, ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
