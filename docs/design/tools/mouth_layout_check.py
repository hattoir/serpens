"""口の前（進行方向、口の幅 60 mm の掃引範囲）に、床に触れる・段差になる部品を置かない配置の検査（GEOMETRY_SIM、箱の重なり）。

禁止域 F: 口の面 x = −236 から フードの奥の壁の外 x = −174.4 まで、および口の前（x < −236）、|y| ≤ 34（口 60 + 足の帯 4 の掃引幅）、
  z < 16.6（屋根の上面）。この中に部品の一部が入ると、口へ入る物（高さ 15 以下）が当たる = 段差になりうる。
  外側の許容: |y| ≥ 36（足の帯 34 + 2 mm）、または 奥の壁より後ろ（x ≥ −172）。
部品の箱は Fusion（Recovery 複製、2026-09-30 読み出し）の bbox。REF = Engineering の FW03 の箱（PROVISIONAL）。
実行: .venv/Scripts/python.exe docs/design/tools/mouth_layout_check.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

sys.path.insert(0, str(Path(__file__).resolve().parent))
import concept_geometry as cg  # noqa: E402

XF0, XF1 = -240.0, -174.4        # 禁止域の x（口の前の 4 mm を含む。口の面 −236）
YF = 34.0
ZF = 16.6

# (名前, 由来, x0, x1, y0, y1, z0, z1)  y は |y| の範囲（左右対称に置く）
CURRENT = [
    ("REF 頭のスキッド（Engineering PROVISIONAL）", "REF", -230, -200, 0, 18, 0, 5),
    ("双子スキッド（Design LIGHT+SKID）", "Design", -232, -172, 30, 40, 0, 12),
    ("顎板 2 枚（Design CHIN SHIELD）", "Design", -232, -180, 20, 31, 1, 4),
    ("REF ToF 下向き", "REF", -200, -182, 0, 9, 5, 7),
    ("REF 斜めの LED（FW03 位置）", "REF", -232, -227, 15, 22, 5, 9),
    ("頬の下の LED（Design E3）", "Design", -215, -210, 38, 42, 5, 10),
    ("鼻のカメラ窓の枠（D14）", "Design", -237, -235, 0, 7, 23, 37),
    ("口の線（Design E3）", "Design", -236, -207, 27, 47, 20, 23),
    ("REF IR LED", "REF", -232, -228, 11, 17, 25, 31),
]
PROPOSED = [
    ("横のスキッド（口の左右の外側）", "案", -226, -178, 36, 44, 0, 10),
    ("後ろのスキッド 2 つ（奥の壁の後ろ）", "案", -172, -164, 14, 22, 0, 7),
    ("ToF 下向き（奥の壁の後ろ、2 つのスキッドの間）", "案", -172, -165, 0, 7, 7, 9),
    ("斜めの LED（口の前の外側の角）", "案", -237, -233, 36, 38, 4, 8),
    ("鼻のカメラ窓の枠（D14、変更なし）", "Design", -237, -235, 0, 7, 23, 37),
    ("口の線（Design E3、変更なし）", "Design", -236, -207, 27, 47, 20, 23),
    ("REF IR LED（変更なし）", "REF", -232, -228, 11, 17, 25, 31),
]


def overlap_mm3(p) -> float:
    _, _, x0, x1, y0, y1, z0, z1 = p
    dx = max(0.0, min(x1, XF1) - max(x0, XF0))
    # |y| 範囲が禁止域（|y| ≤ 34）とどれだけ重なるか。左右 2 つ分
    dy = max(0.0, min(y1, YF) - max(y0, 0.0)) * 2
    dz = max(0.0, min(z1, ZF) - max(z0, 0.0))
    return dx * dy * dz


def evaluate(parts):
    rows = []
    for p in parts:
        v = overlap_mm3(p)
        low = p[6] < ZF
        rows.append({"name": p[0], "source": p[1], "x": [p[2], p[3]], "abs_y": [p[4], p[5]], "z": [p[6], p[7]], "intrusion_mm3": round(v, 1), "z_min_mm": p[6],
                     "verdict": "禁止域に入る" if v > 0 else ("外側（許容）" if low else "屋根より上（許容）")})
    return rows


def draw(parts, ax_top, ax_side, title):
    ax_top.set_title(title + " 上から（x 横、y 縦）", fontsize=10)
    ax_top.add_patch(Rectangle((XF0, -YF), XF1 - XF0, 2 * YF, fill=False, ec="red", ls="--", lw=1.2))
    ax_top.text(XF0 + 2, YF + 1.5, "禁止域（|y| ≤ 34、口の前〜奥の壁の外）", color="red", fontsize=8)
    for p in parts:
        col = {"REF": "#7a7a7a", "Design": "#3a7d44", "案": "#1f6fbf"}[p[1]]
        for s in (1, -1):
            y0, y1 = (p[4], p[5]) if s > 0 else (-p[5], -p[4])
            ax_top.add_patch(Rectangle((p[2], y0), p[3] - p[2], y1 - y0, color=col, alpha=0.55 if p[6] < ZF else 0.2))
    ax_top.set_xlim(-245, -150); ax_top.set_ylim(-52, 52); ax_top.set_aspect("equal")
    ax_side.set_title(title + " 横から（x 横、z 縦）", fontsize=10)
    ax_side.add_patch(Rectangle((XF0, 0), XF1 - XF0, ZF, fill=False, ec="red", ls="--", lw=1.2))
    for p in parts:
        col = {"REF": "#7a7a7a", "Design": "#3a7d44", "案": "#1f6fbf"}[p[1]]
        ax_side.add_patch(Rectangle((p[2], p[6]), p[3] - p[2], p[7] - p[6], color=col, alpha=0.55))
    ax_side.axhline(0, color="k", lw=1); ax_side.set_xlim(-245, -150); ax_side.set_ylim(-3, 45); ax_side.set_aspect("equal")


def main() -> None:
    res = {"source": "GEOMETRY_SIM（箱の重なり）", "forbidden_zone": {"x": [XF0, XF1], "abs_y_max": YF, "z_max": ZF}, "current": evaluate(CURRENT), "proposed": evaluate(PROPOSED)}
    res["current_total_intrusion_mm3"] = round(sum(r["intrusion_mm3"] for r in res["current"]), 1)
    res["proposed_total_intrusion_mm3"] = round(sum(r["intrusion_mm3"] for r in res["proposed"]), 1)
    (cg.RESULTS / "mouth_layout_2026-09-30.json").write_text(json.dumps(res, ensure_ascii=False, indent=2), encoding="utf-8")
    plt.rcParams["font.family"] = ["Yu Gothic", "Meiryo", "sans-serif"]
    fig, axs = plt.subplots(2, 2, figsize=(15, 9))
    draw(CURRENT, axs[0, 0], axs[0, 1], "現行"); draw(PROPOSED, axs[1, 0], axs[1, 1], "案")
    fig.tight_layout(); fig.savefig(cg.ASSETS / "mouth_layout_2026-09-30.png", dpi=70)
    for k in ("current", "proposed"):
        print(k)
        for r in res[k]:
            print("  ", r["name"], r["intrusion_mm3"], r["verdict"])
    print(res["current_total_intrusion_mm3"], res["proposed_total_intrusion_mm3"])


if __name__ == "__main__":
    main()
