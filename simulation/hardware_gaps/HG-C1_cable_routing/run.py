r"""HG-C1 関節渡りのケーブル（MODEL GAP）: アンカーの位置と余長で、±可動域の全域が曲げ半径・空間に収まるかを掃引する。

    .\.venv\Scripts\python.exe simulation\hardware_gaps\HG-C1_cable_routing\run.py

モデル（CABLE_GEOMETRY_SIM。**実ケーブルの剛性・摩擦・ねじれは入れていない**）:
  yaw 軸 = 原点の z 軸。固定側アンカー A = (−a, e)、可動側アンカー B = (b, e) を関節角 θ で回す（平面図）。
  ケーブルの自由長 ℓ は一定。弦 c(θ) = |R(θ)B − A|。ℓ ≥ max c（張らない）。余り s = ℓ − c はたわみになる:
    両端固定の座屈形 y = h/2·(1 − cos 2πx/c) で近似 → h = (2/π)·√(c·s)、最小曲げ半径 R_buckle = c² / (2π² h)
  両端の向きのずれ（可動側の出口が θ 回る）による曲げ: R_turn = c / (2 sin(|θ|/2))
  最小曲げ半径 = min(R_buckle, R_turn)。たわみ h は外装の中の空き（横方向）に収まる必要がある。
  e = 0 は「平面図で yaw 軸の上（または下）を通る」経路 = 回っても弦がほとんど変わらない。
"""
from __future__ import annotations

import json
import math
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
A = yaml.safe_load((HERE / "assumptions.yaml").read_text(encoding="utf-8"))
SOURCE = str(A["label"])


def chord_mm(a: float, b: float, e_a: float, e_b: float, theta_deg: float, dz: float = 0.0) -> float:
    th = math.radians(theta_deg)
    bx, by = b * math.cos(th) - e_b * math.sin(th), b * math.sin(th) + e_b * math.cos(th)
    return math.sqrt((bx + a) ** 2 + (by - e_a) ** 2 + dz * dz)


def evaluate(a: float, b: float, e: float, dz: float, margin: float, lim: float) -> dict[str, Any]:
    thetas = np.linspace(-lim, lim, 111)
    c = np.array([chord_mm(a, b, e, e, t, dz) for t in thetas])
    free = float(c.max() + margin)
    s = free - c
    h = (2 / math.pi) * np.sqrt(c * s)
    r_buckle = np.where(h > 1e-9, c ** 2 / (2 * math.pi ** 2 * np.maximum(h, 1e-9)), np.inf)
    with np.errstate(divide="ignore"):
        r_turn = np.where(np.abs(thetas) > 1e-6, c / (2 * np.abs(np.sin(np.radians(thetas) / 2))), np.inf)
    r_min = np.minimum(r_buckle, r_turn)
    i = int(np.argmin(r_min))
    return {"a": a, "b": b, "e": e, "dz": dz, "chord_min": float(c.min()), "chord_max": float(c.max()),
            "length_change": float(c.max() - c.min()), "free_length": free, "bow_max_mm": float(h.max()),
            "r_min_mm": float(r_min[i]), "r_min_at_deg": float(thetas[i]),
            "ok_bend": bool(r_min.min() >= float(A["cable"]["min_bend_radius_mm"])),
            "ok_space": bool(h.max() <= float(A["envelope"]["bow_space_mm"]))}


def main() -> int:
    sw = A["sweep"]
    lim = float(A["joint"]["mechanical_limit_deg"])
    rows = [evaluate(a, a + d, e, dz, float(A["cable"]["margin_mm"]), lim)
            for a in sw["anchor_mm"] for d in sw["anchor_asym_mm"] for e in sw["lateral_offset_mm"] for dz in sw["dz_mm"]]
    # FW04 の記録との照合（A = (−20, −28.5)、B = (35, −28.5)、±55°）
    fw04 = [chord_mm(20, 35, -28.5, -28.5, t) for t in (-55, -50, 0, 50, 55)]
    L = [f"# HG-C1 関節渡りのケーブル — 経路の境界（自動生成 {datetime.now():%Y-%m-%d %H:%M}。source = {SOURCE}）\n",
         "**幾何だけのモデル。実ケーブルの剛性・摩擦・ねじれ・束の太さの変化は入れていない。**\n",
         f"照合: FW04 の仮アンカー（A=(−20,−28.5), B=(35,−28.5)）の弦 = "
         + " / ".join(f"{t:+d}°: {v:.1f}" for t, v in zip((-55, -50, 0, 50, 55), fw04))
         + " mm（`Serpens_配線設計メモ_20260927.md` の 23.509 / 26.526 / 55.000 / 74.207 / 75.424 と一致させる）\n",
         f"条件: 可動域 ±{lim:g}°（機械）、最小曲げ半径 ≥ {A['cable']['min_bend_radius_mm']} mm（D5 束の仮値）、"
         f"たわみ ≤ {A['envelope']['bow_space_mm']} mm（外装の中の空きの仮値）、余長 = 最大の弦 + {A['cable']['margin_mm']} mm\n",
         "## 横のずれ e（平面図で yaw 軸からの距離）ごとの最良（アンカー距離を選び直した場合）\n",
         "| e mm | 長さの変化 mm | 最大のたわみ mm | 最小曲げ半径 mm | 合格する (a, b, dz) の数 / 全体 |",
         "|---|---|---|---|---|"]
    for e in sw["lateral_offset_mm"]:
        g = [r for r in rows if r["e"] == e]
        ok = [r for r in g if r["ok_bend"] and r["ok_space"]]
        best = min(g, key=lambda r: (not (r["ok_bend"] and r["ok_space"]), r["bow_max_mm"]))
        L.append(f"| {e:g} | {best['length_change']:.1f} | {best['bow_max_mm']:.1f} | {best['r_min_mm']:.1f} | {len(ok)} / {len(g)} |")
    L.append("\n## ケーブルの曲げ半径 × 外装の空き ごとの、成立する経路の数（e = 0 / e ≤ 10 / 全体）\n")
    L.append("曲げ半径はケーブルの選び方（D5 の束 R10、細い線を並べる・FFC なら小さい）、空きは外装の設計で変わる。\n")
    L.append("| 曲げ半径 mm \\ 空き mm | " + " | ".join(f"{s:g}" for s in A["sweep"]["bow_space_mm"]) + " |")
    L.append("|---|" + "---|" * len(A["sweep"]["bow_space_mm"]))
    for rb in A["sweep"]["bend_radius_mm"]:
        cells = []
        for sp in A["sweep"]["bow_space_mm"]:
            ok = lambda r: r["r_min_mm"] >= rb and r["bow_max_mm"] <= sp
            n0 = sum(ok(r) for r in rows if r["e"] == 0)
            n10 = sum(ok(r) for r in rows if r["e"] <= 10)
            cells.append(f"{n0} / {n10} / {sum(ok(r) for r in rows)}")
        L.append(f"| {rb:g} | " + " | ".join(cells) + " |")
    n_e = len([r for r in rows if r["e"] == 0])
    L.append(f"\n（e = 0 の条件は {n_e} 通り、全体は {len(rows)} 通り）\n")
    L.append("**読み**: 横通し（FW04 の e = 28.5 mm）は長さの変化が 36 mm 以上で、どの組み合わせでも成立しない。"
             "**関節渡りは yaw 軸の上か下（平面図で軸から 10 mm 以内）を通す**のが成立域（R10 なら軸の真上だけ）。"
             "D5 束・R10 のままだと、空き 12 mm では 1 点（a ≈ 25 mm、高さの差 20 mm）だけの狭い窓になる → "
             "細い線に分ける（R5 程度）か、軸の上に 20 mm 程度の空きを取る\n")
    L.append("\n## 全条件\n\n| a | b | e | dz | 弦 min〜max | 長さの変化 | 自由長 | たわみ max | 最小曲げ半径（角度） | 曲げ | 空間 |")
    L.append("|---|---|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        L.append(f"| {r['a']:g} | {r['b']:g} | {r['e']:g} | {r['dz']:g} | {r['chord_min']:.1f}〜{r['chord_max']:.1f} | {r['length_change']:.1f} | "
                 f"{r['free_length']:.1f} | {r['bow_max_mm']:.1f} | {r['r_min_mm']:.1f}（{r['r_min_at_deg']:+.0f}°） | "
                 f"{'OK' if r['ok_bend'] else 'NG'} | {'OK' if r['ok_space'] else 'NG'} |")
    (HERE / "decision_boundary.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    (HERE / "results").mkdir(exist_ok=True)
    (HERE / "results" / "c1_rows.json").write_text(json.dumps(rows, indent=1), encoding="utf-8")
    print("FW04 check", [round(v, 3) for v in fw04])
    for e in sw["lateral_offset_mm"]:
        g = [r for r in rows if r["e"] == e]
        print(e, sum(r["ok_bend"] and r["ok_space"] for r in g), "/", len(g),
              "min change", round(min(r["length_change"] for r in g), 1), "min bow", round(min(r["bow_max_mm"] for r in g), 1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
