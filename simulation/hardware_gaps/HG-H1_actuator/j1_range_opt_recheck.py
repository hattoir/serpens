"""J1（J7）の範囲の最適化の再実行: Design の計算した危険体積（ENTRY-D-0015 (3)）へ差し替えて、**結論が変わるか**を見る（LB-E-002 / R-022）。

    python simulation/hardware_gaps/HG-H1_actuator/j1_range_opt_recheck.py

比較するのは 2 つの入力表だけ（他は同じ。乱数は固定）:
  - 旧: `V_COMPUTED_0930`（+6〜+9° と −7.5°（覆いあり）は対数補間 = Design は計算していない）
  - 新: `V_COMPUTED`（Design の計算値 +6 / +7 / +8 / +9° = 1270 / 1432 / 1584 / 1714、−7.5°（覆いあり）= 106）
出力: `simulation/results/j1_range_opt_recheck_2026-10-01.md`。**ACTUATOR_MODEL_SIM + MUJOCO_SIM + CAD_CONCEPT（Design の見積もり。Engineering 未検証）。実機で未確認。**
結果が悪くても書く。範囲の判断（[−4, +3]）を、この再実行の結果に合わせて変えない（変わるかどうかを報告するだけ）。
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

HERE = Path(__file__).parent
spec = importlib.util.spec_from_file_location("j1_range_opt", HERE / "j1_range_opt.py")
o = importlib.util.module_from_spec(spec)
sys.modules["j1_range_opt"] = o
spec.loader.exec_module(o)

POINTS = (6.0, 7.0, 8.0, 9.0)
RANGES = ((-4.0, 3.0), (-4.0, 4.0), (-4.0, 5.0), (-4.0, 8.0), (-4.0, 9.0), (-5.0, 3.0), (-3.0, 3.0))


def with_table(table: dict):
    """`o.V_COMPUTED` を差し替えて、終わったら戻す（`hazard` などはモジュールの表を読む）。"""
    class _Ctx:
        def __enter__(self_):
            self_.saved = o.V_COMPUTED
            o.V_COMPUTED = table
            return self_

        def __exit__(self_, *a):
            o.V_COMPUTED = self_.saved
    return _Ctx()


def snapshot(table: dict) -> dict:
    """ある入力表での、各点の危険体積・各範囲の最悪値・膝・推奨の範囲。"""
    with with_table(table):
        res = o.main()
        rec = o.recommend(res)
        return {
            "hazard": {t: {cv: o.hazard(t, cv) for cv in (False, True)} for t in POINTS + (-7.5,)},
            "ranges": {r: {cv: o.range_hazard(r[0], r[1], cv) for cv in (False, True)} for r in RANGES},
            "knee": {k: (v["chord_knee"], v["steepest_ratio_at"], round(v["ratio"], 2)) for k, v in res["knee"].items()},
            "recommend": rec,
        }


def run() -> dict:
    return {"old": snapshot(o.V_COMPUTED_0930), "new": snapshot(o.V_COMPUTED)}


def larger_pct(old_v: float, new_v: float) -> float:
    """旧（補間）に対して、Design の計算値が何 % 大きいか（(新 − 旧) / 旧。ENTRY-D-0015 の「補間より 33〜74% 大」と同じ基準）。"""
    return 100.0 * (new_v - old_v) / old_v


def main() -> None:
    d = run()
    old, new = d["old"], d["new"]
    L = []
    A = L.append
    A("# J1 の範囲の再実行: Design の危険体積（ENTRY-D-0015 (3)）への差し替え（2026-10-01）\n")
    A("**ACTUATOR_MODEL_SIM + MUJOCO_SIM + CAD_CONCEPT（Design の見積もり。Engineering 未検証）。実機で未確認（HARDWARE_VERIFIED = 0）。5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）。安全・合格の語は使わない。**\n")
    A("入力表は 2 つだけ変えた（`j1_range_opt.py` の `V_COMPUTED_0930` → `V_COMPUTED`）。出典 = Design の `docs/design/results/j1_hazard_wide_coverA_2026-09-30.json`（ENTRY-D-0015 (3)）。他の入力・乱数の種は同じ。"
      "結果が悪くても書く。範囲の判断は、この再実行の結果に合わせて変えない。\n")
    A("## 1. 補間と Design の計算値（覆いなし / 覆いあり。θ_E: + = 頭を上げる）\n")
    A("| θ_E | 旧（補間）| Design の計算 | 旧の補間に対して Design の値は何 % 大きいか | 出所 |\n|---|---|---|---|---|")
    for t in POINTS:
        ov = old["hazard"][t][False][0]
        nv = new["hazard"][t][False][0]
        A(f"| +{t:g}° | {ov:,.0f}（補間）| **{nv:,.0f}** | +{larger_pct(ov, nv):.0f} % | Design（CAD_CONCEPT）|")
    ov, nv = old["hazard"][-7.5][True][0], new["hazard"][-7.5][True][0]
    A(f"| −7.5°（覆いあり）| {ov:,.0f}（補間）| **{nv:,.0f}** | +{larger_pct(ov, nv):.0f} % | Design（CAD_CONCEPT）|")
    A("")
    A("## 2. 範囲ごとの最悪の危険体積（mm³。覆いなし / 覆いあり）\n")
    A("| 範囲 | 旧 最悪（なし / あり）| 新 最悪（なし / あり）| 旧の端が補間か | 新の端が補間か |\n|---|---|---|---|---|")
    for r in RANGES:
        a, b = old["ranges"][r], new["ranges"][r]
        A(f"| [{r[0]:+g}, {r[1]:+g}] | {a[False]['worst']:,.0f} / {a[True]['worst']:,.0f} | {b[False]['worst']:,.0f} / {b[True]['worst']:,.0f} | "
          f"{'はい' if a[False]['interpolated_ends'] else 'いいえ'} | {'はい' if b[False]['interpolated_ends'] else 'いいえ'} |")
    A("")
    A("## 3. 推奨の範囲（制約を満たす候補のうち最悪の危険体積が最小のもの）と膝\n")
    A("| 項目 | 旧 | 新 | 変わったか |\n|---|---|---|---|")
    for k in ("nocover", "cover"):
        a, b = old["recommend"][k], new["recommend"][k]
        sa, sb = f"[{a['lo']:+g}, {a['hi']:+g}] 最悪 {a['worst']:,.0f}", f"[{b['lo']:+g}, {b['hi']:+g}] 最悪 {b['worst']:,.0f}"
        A(f"| 推奨（{'覆いなし' if k == 'nocover' else '覆いあり'}）| {sa} | {sb} | {'**変わらない**（範囲の端）' if (a['lo'], a['hi']) == (b['lo'], b['hi']) else '**変わった**'} |")
    for k in sorted(old["knee"]):
        A(f"| 膝（{k}。弦の膝 / 比が最大の点 / その比）| {old['knee'][k]} | {new['knee'][k]} | {'変わらない' if old['knee'][k] == new['knee'][k] else '変わった'} |")
    A("")
    same = all((old["recommend"][k]["lo"], old["recommend"][k]["hi"]) == (new["recommend"][k]["lo"], new["recommend"][k]["hi"]) for k in ("nocover", "cover"))
    A("## 4. 読み方（限界つき）\n")
    A(f"- **推奨の範囲の端は{'変わらない' if same else '変わった'}**（上の表）。範囲 [−4, +3] は、上げ側 +3° の端の危険体積（44 mm³）と、位置の誤差 + 減速の余裕で決まる（`recommend`）。"
      "+6〜+9° の値が大きくなっても、+3° より外側の候補が**さらに不利になる**だけで、範囲の端を内側へ動かす根拠にはならない（制約側は同じ）。")
    A("- 補間は +6〜+9° で Design の計算値より小さかった（上の表。Design が ENTRY-D-0015 で書いた「補間より 33〜74% 大」を、旧の入力表の再現で**確認できた**。実際は +74.6 / +62.6 / +48.6 / +32.8 %。Design の「74」は端数の切り捨てと読める）。"
      "**姿勢を残す案（+8° 以上）の最悪は、旧の見積もりでなく +9° の 1,714 mm³**。")
    A("- 下げ側の膝（`lo_cover`）の「比が最大の点」だけが動いた（旧 −10° → 新 −7.5°。比 8.07 → 3.66）。−7.5°（覆いあり）が計算済みの点になったため。**推奨の範囲には影響しない**（下げ側の候補は −2〜−5° まで）。")
    A("- 危険体積は Design の CAD_CONCEPT（外形 STL・0.5 mm ボクセル・内部部品なし・公差なし）で、実際の挟み込みの傷害の大きさを表すとは言えない（体積は指標）。")
    A("- **この再実行は計算の差し替えだけ**。J・k・D・B・e・摩擦は prior のまま（HT-001〜004・012 の実測で置き換える。R-010）。範囲 [−4, +3] は PROVISIONAL のまま。")
    out = HERE.parents[2] / "simulation" / "results" / "j1_range_opt_recheck_2026-10-01.md"
    out.write_text("\n".join(L) + "\n", encoding="utf-8", newline="\n")
    print("\n".join(L))
    print("wrote", out)


if __name__ == "__main__":
    main()
