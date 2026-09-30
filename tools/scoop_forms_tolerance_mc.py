"""実物の許容差の確率版: 口の前縁の段差 t とすき間 c を、印刷・シートの厚み・平面度のばらつきとして一様分布で引き、保持率の期待値と 90% 信頼区間を出す。
**「実物で期待できる下限」を 1 表で。** MUJOCO_SIM。実物ではない。

    python tools/scoop_forms_tolerance_mc.py

入力: `simulation/results/scoop_forms_tolerance3_cells.csv`（段差 {0.002, 0.005, 0.01}。縁の丸み 0.3 mm と直角）、`scoop_forms_tolerance_cells.csv`（段差 {0, 0.1, 0.2, 0.5} × すき間 {0, 0.3, 1}、直角の縁）と
      `scoop_forms_tolerance2_cells.csv`（すき間 0.1、縁の丸み 0.3 mm で段差 {0, 0.02, 0.05, 0.1, 0.2, 0.5}、直角で {0.02, 0.05}）。
      漏斗つき・ゲートあり・頭 10 mm/s、位置ずれ 0・5 mm（10 mm ずれの 1 円玉・CR2032 は段差と関係なく失敗するので除く）、N = 30 / 対象物。
分布（**ASSUMED**。User の指示 2026-09-30）: 段差 t ~ U(0, 0.3) mm（A1 の印刷とシートの厚みから）、すき間 c ~ U(0, 0.5) mm（印刷の平面度から）。
方法: 段差 t の関数として、格子点の間の保持率を**下限（小さい方）/ 上限（大きい方）**で置いて期待値の範囲を出す（線形補間は、境界の位置が分からないので使わない）（**すき間 c は結果に効かなかった**: 段差 0 では c = 0 / 0.3 / 1 mm がすべて 100%、段差 ≥ 0.1 では すべて 0%。
      c の格子が無い範囲は、t に対する保持率を c を込みにした値で置き換える）。各格子点の保持率の不確かさは Beta(k + 1, n − k + 1) の事後から引く（4,000 回）。
      期待値 = t の一様分布での積分。90% 信頼区間 = 下限の 5 パーセンタイル〜上限の 95 パーセンタイル。
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path

import numpy as np

RES = Path(__file__).resolve().parents[1] / "simulation" / "results"
OBJS = ["coin_1yen", "battery_cr2032", "bead", "crumb_cube"]
NAME = {"coin_1yen": "1 円玉", "battery_cr2032": "CR2032", "bead": "ビーズ φ8", "crumb_cube": "立方体 10mm"}
T_MAX, C_MAX = 0.3, 0.5                   # ASSUMED（User 2026-09-30）
MAX_OFFSET = 5.0
N_DRAW = 4000


def load(name: str) -> list[dict]:
    p = RES / f"scoop_forms_{name}_cells.csv"
    return list(csv.DictReader(open(p, encoding="utf-8"))) if p.exists() else []


def parse_params(tag: str) -> dict[str, str]:
    body = tag.split("|")[1]
    return dict(kv.split("=", 1) for kv in body.split(","))


def grid(cells: list[dict], fillet: float | None, obj: str | None) -> dict[float, tuple[int, int]]:
    """段差 → (成功, 回数)。すき間は込み。fillet: 0.3 か None（直角）。"""
    out: dict[float, list[int]] = {}
    for x in cells:
        p = parse_params(x["tag"])
        f = float(p["rim_fillet_mm"]) if "rim_fillet_mm" in p else None
        if f != fillet or (obj is not None and x["obj"] != obj) or float(x["offset"]) > MAX_OFFSET:
            continue
        t = float(p.get("plate", 0.0))
        k = out.setdefault(t, [0, 0])
        k[0] += int(x["success"])
        k[1] += int(x["n"])
    return {t: (v[0], v[1]) for t, v in sorted(out.items())}


def expected_bounds(ts: list[float], rates: np.ndarray) -> tuple[float, float]:
    """t ~ U(0, T_MAX) での保持率の期待値の（下限, 上限）。格子点の間で保持率が**どう変わるか分からない**ので、
    区間 (t_i, t_{i+1}) の保持率を、小さい方の値（下限）/ 大きい方の値（上限）で置く。最後の格子より先は最後の値。
    段差 0（板なし）だけが高く、0.002 mm でも 0% のときは、下限 = 0、上限 = 0.002 mm / T_MAX × 保持率（0 のごく近く）。"""
    lo = hi = 0.0
    for i, t0 in enumerate(ts):
        t1 = ts[i + 1] if i + 1 < len(ts) else T_MAX
        t1 = min(t1, T_MAX)
        if t0 >= T_MAX:
            break
        nxt = rates[i + 1] if i + 1 < len(ts) else rates[i]
        lo += (t1 - t0) * min(rates[i], nxt)
        hi += (t1 - t0) * max(rates[i], nxt)
    return lo / T_MAX, hi / T_MAX


def mc(g: dict[float, tuple[int, int]], seed: int = 1) -> tuple[tuple[float, float], tuple[float, float], list[float], list[float]]:
    ts = sorted(g)
    rng = np.random.default_rng(seed)
    ks = np.array([g[t][0] for t in ts])
    ns = np.array([g[t][1] for t in ts])
    draws = rng.beta(ks + 1, ns - ks + 1, size=(N_DRAW, len(ts)))
    lo_hi = np.array([expected_bounds(ts, d) for d in draws])
    point = expected_bounds(ts, ks / ns)
    return point, (float(np.percentile(lo_hi[:, 0], 5)), float(np.percentile(lo_hi[:, 1], 95))), ts, list(ks / ns)


def main() -> None:
    cells = load("tolerance") + load("tolerance2") + load("tolerance3")
    if not cells:
        print("集計 CSV が無い（先に tolerance / tolerance2 を回す）")
        return
    L: list[str] = []
    A = L.append
    A("# 実物の許容差の確率版（段差 t ~ U(0, 0.3) mm、すき間 c ~ U(0, 0.5) mm）\n")
    A("MUJOCO_SIM。**分布は ASSUMED**（User 2026-09-30。段差は A1 の印刷とシートの厚みから、すき間は印刷の平面度から）。すき間 c は結果に効かなかった（段差 0 で c = 0 / 0.3 / 1 mm すべて 100%、段差 ≥ 0.1 でも すべて 0%）ので、期待値は段差 t だけで決まる。\n")
    A("| 縁 | 対象 | 保持率の期待値の範囲（下限〜上限） | 90% 信頼区間（下限の 5% 点〜上限の 95% 点） | 段差ごとの保持率（格子点） |\n|---|---|---|---|---|")
    worst = None
    for fillet, lab in ((None, "直角"), (0.3, "丸み 0.3 mm")):
        for obj in [None] + OBJS:
            g = grid(cells, fillet, obj)
            if len(g) < 2:
                continue
            (plo, phi), (clo, chi), ts, rs = mc(g)
            if obj is None:
                worst = (lab, plo, phi, clo, chi) if worst is None else worst
            A(f"| {lab} | {'4 種合算' if obj is None else NAME[obj]} | {100 * plo:.2f}%〜{100 * phi:.2f}% | {100 * clo:.2f}%〜{100 * chi:.2f}% | "
              + " / ".join(f"t={t:g}: {100 * r:.0f}%" for t, r in zip(ts, rs)) + " |")
    A("\n**実物で期待できる下限（1 行）**: 段差が 0 でない限り保持は 0%（**0.002 mm の段差でも 0%**）。段差 t ~ U(0, 0.3) mm では、保持が得られるのは t がほぼ 0 の一点だけで、"
      "**期待値は 0〜0.7% 程度（下限 0%）**。実物で期待できる下限は**ほぼ 0%**。")
    A("理由（構造的）: 自由に滑る軽い物（1 円玉の床の摩擦は約 3 mN）を頭がゆっくり押すとき、口に剛体の段差があると、物はその段差に**押されて頭と同じ速さで運ばれるだけ**で、段差を越えない"
      "（段差の角が物を持ち上げる力 = 押す力 × 傾き は、物の重さ 約 10 mN に届かない）。段差の大きさ（0.002〜0.5 mm）は効かず、**有る / 無い**で決まる。"
      "0.02 mm 以下は接触のやわらかさ（solref 0.002 s、solimp の幅 0.5 mm）の分解能に近いが、0.002〜0.5 mm のすべてで 0% だった。")
    A("**実物への含意**: 面一（段差 0）を印刷の許容差で保証することはできない。**段差を無くす形**（口の前縁を床に倣う柔らかい薄いリップ、床に押しつける弾性のスカート、リップなし）が要る。"
      "実物では床・物の表面粗さ・弾性が、剛体のシミュレーションより段差を越えやすくする可能性があり、**試験片 B（リップの厚み `plate` を 0 / 0.3 / 0.6 / 1.2 と振る）で確かめる**。\n")
    Path(RES / "scoop_forms_tolerance_mc.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    print("\n".join(L))


if __name__ == "__main__":
    main()
