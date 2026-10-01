"""実物の許容差の確率版: 口の前縁の段差 t とすき間 c を、印刷・シートの厚み・平面度のばらつきとして一様分布で引き、保持率・入る率の期待値と 90% 信頼区間を出す。
**「実物で期待できる下限」を 1 表で。** MUJOCO_SIM。実物ではない。

    python tools/scoop_forms_tolerance_mc.py

分布（**ASSUMED**。User の指示 2026-09-30）: 段差 t ~ U(0, 0.3) mm（A1 の印刷とシートの厚みから）、すき間 c ~ U(0, 0.5) mm（印刷の平面度から）。
**すき間 c は結果に効かなかった**（段差 0 では c = 0 / 0.3 / 1 mm すべて 100%、段差 > 0 では すべて 0%）ので、期待値は段差 t だけで決まる。
3 つの条件を並べる（実物に近いのは「面取りあり、粗さあり」）:
  A 剛体・鋭い直角・平らな床         `scoop_forms_tolerance*_cells.csv`（段差 0〜0.5 mm。直角の縁）
  B 面取りあり・平らな床            `scoop_forms_chamfer*_cells.csv`（面取り角 5 / 10 / 20 / 45°、丸み R 0 / 0.1 / 0.3 mm。すき間 0）
  C 面取りあり・粗さのある床         `scoop_forms_realistic_cells.csv`（面取り 10 / 20°、R 0.1、粗さ ±0.03 / ±0.1 mm）
共通: 漏斗つき・ゲートあり・頭 10 mm/s、位置ずれ 0・5 mm（10 mm ずれの 1 円玉・CR2032 は段差と関係なく失敗するので除く）。t = 0 は「板なし」の結果（100%）。
方法: 段差 t の関数として、格子点の間の保持率を**下限（小さい方）/ 上限（大きい方）**で置いて期待値の範囲を出す（線形補間は、境界の位置が分からないので使わない）。
      各格子点の保持率の不確かさは Beta(k + 1, n − k + 1) の事後から引く（4,000 回）。90% 信頼区間 = 下限の 5 パーセンタイル〜上限の 95 パーセンタイル。
指標は 2 つ: **保持**（閉じ終わりから 2 秒後まで中）と **入る**（物の中心が空間の範囲に一度でも入った。段差に乗って入り口で頭と一緒に運ばれるだけの場合も数える）。
"""
from __future__ import annotations

import csv
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


def grid_generic(cells: list[dict], pred, obj: str | None = None, floor: str | None = None, key: str = "success") -> dict[float, tuple[int, int]]:
    """段差 t → (成功 or 入る の回数, 回数)。pred(params) が True の設計だけ。"""
    out: dict[float, list[int]] = {}
    for x in cells:
        if (obj is not None and x["obj"] != obj) or (floor is not None and x["floor"] != floor) or float(x["offset"]) > MAX_OFFSET:
            continue
        p = parse_params(x["tag"])
        if not pred(p):
            continue
        t = float(p.get("plate", 0.0))
        k = out.setdefault(t, [0, 0])
        k[0] += int(x[key])
        k[1] += int(x["n"])
    return {t: (v[0], v[1]) for t, v in sorted(out.items())}


def expected_bounds(ts: list[float], rates: np.ndarray) -> tuple[float, float]:
    """t ~ U(0, T_MAX) での期待値の（下限, 上限）。区間 (t_i, t_{i+1}) の値を、小さい方（下限）/ 大きい方（上限）で置く。最後の格子より先は最後の値。"""
    lo = hi = 0.0
    for i, t0 in enumerate(ts):
        t1 = min(ts[i + 1] if i + 1 < len(ts) else T_MAX, T_MAX)
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


def with_zero(g: dict[float, tuple[int, int]], zero: tuple[int, int]) -> dict[float, tuple[int, int]]:
    """t = 0（板なし）の結果を足す。"""
    return {0.0: zero, **{t: v for t, v in g.items() if t > 0}}


def line(label: str, g: dict[float, tuple[int, int]]) -> str:
    (plo, phi), (clo, chi), ts, rs = mc(g)
    return (f"| {label} | {100 * plo:.1f}%〜{100 * phi:.1f}% | {100 * clo:.1f}%〜{100 * chi:.1f}% | "
            + " / ".join(f"t={t:g}: {100 * r:.0f}%" for t, r in zip(ts, rs)) + " |")


def main() -> None:
    tol = load("tolerance") + load("tolerance2") + load("tolerance3")
    cham = load("chamfer") + load("chamfer2")
    real = load("realistic")
    if not tol:
        print("集計 CSV が無い（先に tolerance / tolerance2 / tolerance3 を回す）")
        return
    zero = {k: (sum(v[0] for v in [grid_generic(tol, lambda p: float(p.get("plate", 0)) == 0.0, key=k).get(0.0, (0, 0))]),
                sum(v[1] for v in [grid_generic(tol, lambda p: float(p.get("plate", 0)) == 0.0, key=k).get(0.0, (0, 0))])) for k in ("success", "entered")}
    L: list[str] = []
    A = L.append
    A("# 実物の許容差の確率版（段差 t ~ U(0, 0.3) mm、すき間 c ~ U(0, 0.5) mm）— 3 つの条件を並べる\n")
    A("MUJOCO_SIM。**分布は ASSUMED**（User 2026-09-30）。すき間 c は結果に効かなかった（段差 0 で c = 0 / 0.3 / 1 mm すべて 100%、段差 > 0 では すべて 0%）ので、期待値は段差 t だけで決まる。"
      "位置ずれ 0・5 mm、漏斗つき・ゲートあり・10 mm/s。**実物に近いのは C（面取りあり、粗さあり）**。両方を並べる。\n")
    HEAD = "| 条件 | 期待値の範囲（下限〜上限） | 90% 信頼区間 | 段差ごとの率（格子点） |\n|---|---|---|---|"
    for key, title in (("success", "保持"), ("entered", "入る（一度でも空間の範囲に入った）")):
        A(f"\n## {title}\n")
        A(HEAD)
        z = zero[key]
        A(line("A 剛体・鋭い直角・平らな床（4 種合算）", with_zero(grid_generic(tol, lambda p: p.get("rim_fillet_mm") is None, key=key), z)))
        for ang in (90, 45, 20, 10, 5):
            for R in (0.0, 0.1, 0.3):
                g = grid_generic(cham, lambda p, a=ang, r=R: float(p["plate_chamfer_deg"]) == a and float(p["plate_round_mm"]) == r, key=key)
                if g:
                    A(line(f"B 面取り {ang}°{'（=直角）' if ang == 90 else ''}・丸み R {R:g} mm・平らな床（4 種合算）", with_zero(g, z)))
        for ang in (10, 20):
            for amp in (0.03, 0.1):
                g = grid_generic(real, lambda p, a=ang, m=amp: p.get("rim_fillet_mm") is None and float(p["plate_chamfer_deg"]) == a and float(p["rough_mm"]) == m, key=key)
                if g:
                    A(line(f"C 面取り {ang}°・R 0.1・粗さ ±{amp:g} mm（4 種合算）", with_zero(g, z)))
        for amp in (0.03, 0.1):
            g = grid_generic(real, lambda p, m=amp: p.get("rim_fillet_mm") is not None and float(p["rough_mm"]) == m, key=key)
            if g:
                A(line(f"C 面取り 10°・R 0.1・粗さ ±{amp:g} mm・物の縁の丸み 0.3 mm（4 種合算）", with_zero(g, z)))
    # 並べた要約（面取り 10°・R 0.1 を代表に。B は t = 0.002〜0.5 の細かい格子がある設計）
    def rng(pred, cells, key):
        g = grid_generic(cells, pred, key=key)
        (plo, phi), (clo, chi), ts, rs = mc(with_zero(g, zero[key]))
        return plo, phi, clo, chi

    rows = [
        ("A 剛体・直角・平らな床", tol, lambda p: p.get("rim_fillet_mm") is None),
        ("B 面取り 10°・R 0.1・平らな床", cham, lambda p: float(p["plate_chamfer_deg"]) == 10 and float(p["plate_round_mm"]) == 0.1),
        ("C 面取り 10°・R 0.1・粗さ ±0.03 mm", real, lambda p: p.get("rim_fillet_mm") is None and float(p["plate_chamfer_deg"]) == 10 and float(p["rough_mm"]) == 0.03),
        ("C 面取り 10°・R 0.1・粗さ ±0.1 mm", real, lambda p: p.get("rim_fillet_mm") is None and float(p["plate_chamfer_deg"]) == 10 and float(p["rough_mm"]) == 0.1),
        ("C 面取り 10°・R 0.1・粗さ ±0.1 mm・物の縁の丸み 0.3", real, lambda p: p.get("rim_fillet_mm") is not None and float(p["rough_mm"]) == 0.1),
    ]
    A("\nOOO 並べた要約（面取り 10°・R 0.1 を代表に）\n".replace("OOO", "##"))
    A("| 条件 | 保持（範囲 / 90%CI） | 入る（範囲 / 90%CI） |\n|---|---|---|")
    summ = {}
    for label, cells, pred in rows:
        h = rng(pred, cells, "success")
        e = rng(pred, cells, "entered")
        summ[label] = h
        A(f"| {label} | {100 * h[0]:.1f}〜{100 * h[1]:.1f}% / {100 * h[2]:.1f}〜{100 * h[3]:.1f}% | {100 * e[0]:.1f}〜{100 * e[1]:.1f}% / {100 * e[2]:.1f}〜{100 * e[3]:.1f}% |")
    hA, hB, hC3, hC1, hC1f = (summ[r[0]] for r in rows)
    A("\n## 読み（数値は上の表から。格子の間の値は下限/上限で置いた範囲）\n")
    A(f"- **直角・平らな床（A）**: 保持の期待値 {100 * hA[0]:.1f}〜{100 * hA[1]:.1f}%。段差 0.002 mm で 0%（剛体・直角では、段差は 0.002 mm でも保持を消す）。")
    A(f"- **面取りだけ（B）**: 保持の期待値 {100 * hB[0]:.1f}〜{100 * hB[1]:.1f}%。面取りを付けても保持は戻らない。戻るのは「入る」で、乗った物は頭と一緒に運ばれる（§12.5）。")
    A(f"- **面取り＋粗さ（C）**: 粗さ ±0.03 mm で保持 {100 * hC3[0]:.1f}〜{100 * hC3[1]:.1f}%、粗さ ±0.1 mm で **{100 * hC1[0]:.1f}〜{100 * hC1[1]:.1f}%**（物の縁の丸み 0.3 mm を足すと {100 * hC1f[0]:.1f}〜{100 * hC1f[1]:.1f}%）。"
      "**0 でない値が出るのは、段差 0.05 mm・面取り 10 / 20°・粗さ ±0.1 mm の組（保持 44〜59%）だけ**で、0.1 mm 以上の段差ではどの条件も 0%。"
      "下限は、この 1 格子点（t = 0.05）と、t = 0〜0.05 の間を「小さい方」で置いた寄与だけで決まる。")
    A("- **結論**: シミュレーションでは、口の段差は**ほぼ 0（0.05 mm 以下）が必須**。0.1 mm 以上は、面取り・丸み・粗さ・接触設定を変えても保持は 0%。"
      "「実物で期待できる下限」は、面取り＋粗さ（C）でも**約 0〜10%（粗さ ±0.1 mm を仮定した場合だけ約 10%）**で、期待の根拠にはできない。")
    A("- **限界**: 面取りは 0.05〜0.5 mm の格子（B の細かい格子は 10 / 45°・R 0.1 のみ）。B の他の行の上限 16.7% は、t = 0 と 0.05 mm の間（格子の穴）を上限で置いた結果で、面取りの効果ではない。"
      "粗さは「ランダムな高さ場（1 mm 格子）」で、実物の床（フローリング・マット）の粗さの実測ではない。カーペットは対象外。")
    A("- 実物での確認は試験片 B（`hardware_test_plan.md`）。ここでの 0% は**シミュレーションの結果**であり、実物でも 0% という意味ではない（HARDWARE_VERIFIED = 0）。\n")
    Path(RES / "scoop_forms_tolerance_mc.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    print("\n".join(L))


if __name__ == "__main__":
    main()
