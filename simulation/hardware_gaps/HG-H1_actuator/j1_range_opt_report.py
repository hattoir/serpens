"""`j1_range_opt.py` の結果を `simulation/results/j1_range_opt_2026-09-30.md`（と CSV・PNG）に書く。

    python simulation/hardware_gaps/HG-H1_actuator/j1_range_opt_report.py
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).parent
spec = importlib.util.spec_from_file_location("j1_range_opt", HERE / "j1_range_opt.py")
o = importlib.util.module_from_spec(spec)
sys.modules["j1_range_opt"] = o
spec.loader.exec_module(o)

RES = o.RES


def fm(x: float) -> str:
    return f"{x:,.0f}"


def main() -> None:
    data = o.write_outputs()
    res, rows, stop = data["main"], data["rows"], data["stopper"]
    fe = res["feasibility_mc"]
    rec = o.recommend(res)
    rec60 = o.recommend(res, v_min=60)
    intake = o.load_intake()
    band = {t: o.intake_band(intake, t) for t in (0.85, 0.90, 0.95)}
    wp = o.intake_working_point_mc(intake)
    sens = res["sensitivity_knee"]
    kn = res["knee"]
    mc = res["margin_control_deg"]
    o.make_plot(RES / "j1_range_opt_pareto.png", rows, rec)
    rc, rn = rec["cover"], rec["nocover"]

    def sf(material, omega, tpu, dec=0.0):
        return next(x for x in stop if x["material"] == material and x["omega_dps"] == omega and x["tpu"] == tpu and x["decel_deg"] == dec)

    L = []
    A = L.append
    A("# J1（頭ピッチ = Engineering の J7）の動作範囲とストッパーの荷重の最適化（2026-09-30）\n")
    A("> **数値はシミュレーション・prior・Design の CAD_CONCEPT の見積もりで、実機で未確認。HARDWARE_VERIFIED ではない。5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）で、ここでは使っていない（機械の強度の見積もり）。安全の確定ではない。結果が悪くても後付けの調整はしていない。**\n")
    A("符号は Engineering（θ_E: + = 頭を上げる）。実機の符号は HT-001（J1-0）で確認するまで未確認。手順・道具は `ai-shared/HARDWARE_TODO.md`。\n")

    A("## 0. 結論（暫定値。戻せる）\n")
    A("| 項目 | 決めた値（PROVISIONAL） | 根拠 |")
    A("|---|---|---|")
    A(f"| **機械ストッパーの範囲（覆いあり）** | **[{rc['lo']:g}°, +{rc['hi']:g}°]** | 制約（窓 −2〜+1° + 位置の誤差 + 減速の余裕 + 取り込みの帯 [{band[0.9][0]:+.1f}°, {band[0.9][1]:+.1f}°]）を満たす中で、最悪の危険体積が最小（{fm(rc['worst'])} mm³）。**上側の膝は +3°**（Design の計算済みの点で +3° → +5° が {kn['hi_cover']['ratio_next']:.0f} 倍に急増: 44 → 601 mm³） |")
    A(f"| 機械ストッパーの範囲（**覆いなし**） | [{rn['lo']:g}°, +{rn['hi']:g}°]（最悪 {fm(rn['worst'])} mm³）| 下側は覆いが無いと −4° で 205 mm³（補間）。覆いを採ると 13 mm³（補間）。**覆いは Design の形の判断（見た目 = User）**。採らない場合、この値のまま進めてよいが下側の危険体積は 約 {fm(rn['worst'] / max(rc['worst'], 1))} 倍 |")
    A("| ソフトの作業窓 | **−2° 〜 +1°**（変えない。Design の提案） | すき間 ≤ 40 mm³。取り込みの帯は [{:+.1f}°, {:+.1f}°]（I ≥ 0.90、全床・摩擦 0.7〜1.4 倍）で、窓の内側 |".format(*band[0.9]))
    A(f"| **窓の端の手前の速さの上限** | **≤ 40 °/s**（窓の端の手前 2° の区間） | 余裕 2° − 位置の誤差（prior の最大 {mc['max']:.2f}°）から、減速度 1,000 °/s² で減速できる速さ（{o.speed_cap_dps(3.0, 'hi', mc['max']):.0f} °/s） |")
    A("| **ストッパーのピン** | **鋼のダウエル φ3（PLA は不可）** + 窓の端に **TPU パッド（厚さ 3〜6 mm、E 3 MPa 級）**| 衝撃 + 静的の合計荷重に対する安全率 p05: 鋼 + TPU 3 mm で {:.1f}（120°/s の衝突。減速なし）、PLA は {:.2f}（SF < 1.5 の確率 {:.0%}） |".format(sf('steel', 120.0, 'E3MPa_t3mm')['sf_p05'], sf('PLA', 120.0, 'E3MPa_t3mm')['sf_p05'], sf('PLA', 120.0, 'E3MPa_t3mm')['p_sf_lt_1p5']))
    A("| 使えなくなる姿勢（副次的なコスト）| **5 グループ**（home / rest_arc / coil = 8°、人を見る 55〜65°、フル鎌首 85〜90°）+ 呼吸（J7 ±5°）| 上限 +3° は、`robot.yaml` の J7（−8〜90°）の動作を **Floor Watch の頭では使えなくする**。既存の J7 の上限は**変えない**（下の §7: 作業モードごとの追加の設定にした）|\n")
    A(f"**上限を +8° 以上にすれば 3 グループ（home / rest_arc / coil）は残る**が、最悪の危険体積は {fm(o.range_hazard(rc['lo'], 8.0, True)['worst'])} mm³（補間）と **約 {o.range_hazard(rc['lo'], 8.0, True)['worst'] / max(rc['worst'], 1):.0f} 倍**。姿勢の損失は副次的なコストとして扱う指示なので、+3° を推す。**姿勢が要るなら、Design の別案（動く覆い / Floor Watch モードだけ効かせる外せるピン）**を R-003 で依頼する。\n")

    A("## 1. 出典（すべて「シミュレーション / prior」。実測なし）\n")
    A("| 入力 | 出典 | ラベル |\n|---|---|---|")
    A("| すき間の危険体積 V(θ) | Design の見積もり: `docs/design/results/gap_pitch_*.json`、`j1_cover_eval_2026-09-30.json`（外形 STL・0.5 mm ボクセル・内部部品なし）。**計算済みの点だけ使い、その間は対数補間（Design は計算していない。表で `補間` と印）** | CAD_CONCEPT / KINEMATIC_SIM |")
    A("| 取り込み割合 I(c) | `agent/engineering-scoop` の `tools/scoop_forms_sweep.py intake_c`（2,400 回。フード + 漏斗 + ゲート、頭 10 mm/s、位置ずれ 0・5 mm、床 = フローリング / マット / 凹凸 ±0.5 mm（絨毯の代用）、摩擦（壁・床）を 0.7 / 1.0 / 1.4 倍。**絨毯は未対応**）| MUJOCO_SIM |")
    A("| θ → すき間 c | Design の幾何: 口の前縁 +0.95 mm/°、前縁が床に触れる θ = −0.106°、押し込むと奥の壁の下が +1.17 mm/°（大きい方を採る = 保守的）| CAD_CONCEPT / ASSUMED |")
    A("| 位置の誤差 δθ | 量子化 ±0.5 ステップ（0.088°）+ 不感帯 D（0〜4 ステップ）+ バックラッシ B（0〜8 ステップ、エンコーダがモーター側の確率 0.5）。**D・B は UNKNOWN（HG-H1 の prior に無い）。HT-004（J1-1 / J1-2）で置き換える** | prior |")
    A("| ストッパーの荷重 | 静的 = 上限（0.167 × ストール 1.4〜3.0 N·m × (1 ± 0.4)）÷ 腕 36〜48.5 mm + 衝撃 ω√(J k)（J = 反射慣性 1e-3〜1e-2 kg·m²、k = TPU パッド E A / t + ピン + 窓の端の直列）。ピン: 片持ち φ3・L 4.5 mm、許容応力 PLA 20〜50 MPa / 鋼 200〜350 MPa | prior（J・k・強度は UNKNOWN。HT-003 で置き換える）|")
    A("| 使えなくなる姿勢 | `config/robot.yaml`（`poses.home` / `poses.rest_arc` / `legacy_poses.coil` の J7 = 8°、`neck.look_*` 55〜65°、`neck.full_rear_*` 85〜90°、`breath.amplitude_by_axis.J7` ±5°）| 設定 |\n")

    A("## 2. 範囲の候補: 下限 −2〜−5°、上限 +1.5〜+10°\n")
    A("最悪の危険体積 [mm³]（範囲の中の最大）。`*` = 端が補間。実現性 = prior（D・B）で、窓の端の手前の速さ ≥ 30 °/s で減速できる確率（余裕 = 端 − 窓の端 − 位置の誤差、減速度 1,000〜2,233 °/s²）。\n")
    A("| 下限 \\ 上限 | " + " | ".join(f"+{h:g}°" for h in o.HI_CANDIDATES) + " |\n|---|" + "---|" * len(o.HI_CANDIDATES))
    A("| **実現性（上限側）** | " + " | ".join(f"{fe['hi'][h]['p_feasible']:.0%}" for h in o.HI_CANDIDATES) + " |")
    for cover in (True, False):
        for lo in o.LO_CANDIDATES:
            cells = []
            for h in o.HI_CANDIDATES:
                r = next(x for x in rows if x["cover"] == cover and x["lo"] == lo and x["hi"] == h)
                cells.append(fm(r["worst"]) + ("*" if r["interpolated_ends"] else ""))
            A(f"| {'覆いあり' if cover else '覆いなし'} {lo:g}°（実現性 {fe['lo'][lo]['p_feasible']:.0%}）| " + " | ".join(cells) + " |")
    A("\n使えなくなる姿勢（5 グループ）: 上限 +1.5〜+7° は **5**（home / rest_arc / coil / 人を見る / フル鎌首）、**+8° 以上は 2**（人を見る / フル鎌首）。呼吸（J7 ±5°）はどの候補でも使えない。\n")
    A("**目的関数**: 最悪の危険体積を最小にする。**制約**: (1) 作業窓 −2〜+1° を含む、(2) 位置の誤差 + 減速の余裕（実現性 ≥ 99%）、(3) 取り込みの帯を含む、(4) ストッパーの安全率（§4）。姿勢の損失は副次的なコスト。")
    A(f"**パレート図**（`j1_range_opt_pareto.png`）: 実現可能な候補のうち、最悪の危険体積と姿勢の損失（5 → 2）で残るのは **[−4, +3]（覆いあり: {fm(rc['worst'])} mm³、損失 5）** と **[−4, +8] 以上（{fm(o.range_hazard(-4.0, 8.0, True)['worst'])} mm³ 級、損失 2）** の 2 群。**膝は上側 +3°**（Design が計算した点 +1 / +2 / +3 / +5 / +10° で、次の点への増加率が最大 = +3° → +5° の {kn['hi_cover']['ratio_next']:.1f} 倍）。")
    A(f"下側の膝は覆いあり −5°（−5° → −10° が {kn['lo_cover']['ratio_next']:.1f} 倍）、覆いなし −3°（{kn['lo_nocover']['ratio_next']:.1f} 倍）。実現性から下限は −4°（覆いあり: 13 mm³ 補間。**Design が計算済みの −5° は 29 mm³**）。\n")

    A("## 3. 取り込み割合 vs 角度（MUJOCO_SIM）\n")
    A("壁の下端のすき間 c の一様な値に対する取り込み割合（位置ずれ 0・5 mm、床 3 種 × 摩擦 9 通りの平均）:\n")
    A("| c [mm] | " + " | ".join(f"{c:g}" for c in (0.0, 0.1, 0.3, 0.6, 1.0, 1.5, 2.0, 3.0, 4.0)) + " |\n|---|" + "---|" * 9)
    for cls in ("flooring", "mat", "bump"):
        ks = [k for k in intake if k[0] == cls]
        A(f"| {cls}{'（凹凸 ±0.5 = 絨毯の代用）' if cls == 'bump' else ''} | " + " | ".join(f"{np.mean([intake[k][c][0] / intake[k][c][1] for k in ks]):.0%}" for c in (0.0, 0.1, 0.3, 0.6, 1.0, 1.5, 2.0, 3.0, 4.0)) + " |")
    A("")
    A(f"- **c ≤ 1.0 mm は、床・摩擦（0.7〜1.4 倍）によらず 100%**。c 1.5 mm 以上は 1 円玉（1.5 mm）が壁の下をくぐって 75〜80%、c 4 mm で 50%（CR2032 3.2 mm も）。すき間 0〜1 mm が効かない §12.1 と一致。**段差 t（口の前縁の板）は 0.002 mm でも 0%**（§12.6）なので、c + t の掃引で意味があるのは t = 0 の c だけ（板を付けない設計が前提）。")
    A(f"- **θ に直すと**: 取り込みの帯（全床・全摩擦で I ≥ 0.85 / 0.90 / 0.95）は **[{band[0.85][0]:+.2f}°, {band[0.85][1]:+.2f}°] / [{band[0.9][0]:+.2f}°, {band[0.9][1]:+.2f}°] / [{band[0.95][0]:+.2f}°, {band[0.95][1]:+.2f}°]**（幅 約 2°、24 ステップ）。上側は口の前縁が c = 1.0〜1.15 mm まで上がる角（+0.95 mm/°）、**下側は前縁が床に押しつけられ、奥の壁の下が 1 mm を超える角（約 −1°）**。")
    A(f"- **作業点（θ_cmd = 0°、床接触の較正で c = 0.1 mm）の取り込み割合**: prior の位置の誤差・足の帯の平らさ ±0.05 mm・摩擦・床の全部で、**平均 {wp['mean']:.1%}、p05 {wp['p05']:.0%}、I ≥ 0.90 の確率 {wp['p_ge_target']:.0%}**。**取り込みは prior の幅にほとんど依存しない**（帯の中に収まる）。ただし帯は ±1° = 11 ステップしかなく、**J1 の位置の誤差（D・B）が 1° を超えるなら帯を外れる**（HT-004 で確認）。\n")

    A("## 4. ストッパーの荷重（ω 60〜180 °/s、TPU パッド、減速）\n")
    A("衝撃 + 静的（既定の上限 0.167 の場合。作業中の上限 8/1000 なら静的は 1 N 未満）の合計荷重と、φ3 ピンの片持ち曲げの許容荷重の比（安全率 SF）。prior の 4,000 点。減速は端の手前 `decel` 度から 1,000 °/s²（保守）で。\n")
    A("| ピン | ω [°/s] | TPU パッド | 減速 [°] | 衝突の速さ [°/s] | 衝撃 p95 [N] | 合計荷重 p95 [N] | **SF p05** | SF < 1.5 の確率 |\n|---|---|---|---|---|---|---|---|---|")
    for x in stop:
        if x["decel_deg"] == 0.0 and x["omega_dps"] in (60.0, 120.0, 180.0) and x["tpu"] in ("なし", "E3MPa_t1mm", "E3MPa_t3mm", "E3MPa_t6mm"):
            A(f"| {x['material']} | {x['omega_dps']:g} | {x['tpu']} | {x['decel_deg']:g} | {x['omega_impact_dps']:.0f} | {x['impact_p95']:.1f} | {x['load_p95']:.1f} | **{x['sf_p05']:.2f}** | {x['p_sf_lt_1p5']:.0%} |")
    A("")
    A("**端の手前の減速の効果**（鋼、TPU 3 mm、ω = 120 °/s、減速度 1,000 °/s²）:\n")
    A("| 減速を始める角 [°] | 衝突の速さ [°/s] | 衝撃 p95 [N] | SF p05 |\n|---|---|---|---|")
    for dec in (0.0, 1.0, 2.0, 3.0, 5.0):
        x = sf("steel", 120.0, "E3MPa_t3mm", dec)
        A(f"| {dec:g} | {x['omega_impact_dps']:.0f} | {x['impact_p95']:.1f} | {x['sf_p05']:.2f} |")
    A("")
    A(f"- **PLA のピンは、どの条件でも SF p05 が 1.1 以下で足りない**（120°/s・TPU 3 mm でも SF p05 {sf('PLA', 120.0, 'E3MPa_t3mm')['sf_p05']:.2f}、SF < 1.5 の確率 {sf('PLA', 120.0, 'E3MPa_t3mm')['p_sf_lt_1p5']:.0%}。60°/s・TPU 6 mm でも {sf('PLA', 60.0, 'E3MPa_t6mm')['sf_p05']:.2f}）。**鋼のダウエルが必須**。")
    A(f"- **鋼 φ3**: 120°/s の衝突（減速なし = ソフトのリミットが効かない故障）でも SF p05 {sf('steel', 120.0, 'なし')['sf_p05']:.1f}（TPU なし）〜 {sf('steel', 120.0, 'E3MPa_t6mm')['sf_p05']:.1f}（TPU 6 mm）。**180°/s・TPU なしでは {sf('steel', 180.0, 'なし')['sf_p05']:.2f}（SF < 1.5 の確率 {sf('steel', 180.0, 'なし')['p_sf_lt_1p5']:.0%}）** → TPU パッドと速さの制限が要る。")
    A("- **TPU パッドは厚さが効く**: 薄い（1 mm）と PLA 並みに硬く（k = E A / t = 32〜108 N/mm）、衝撃はほとんど減らない。**厚さ 3〜6 mm、E 3 MPa 級（k 5〜11 N/mm）で衝撃が 1/3〜1/4**。前回（ENTRY-E-0009）の「TPU の緩衝で 4.7〜15 N」は k = 5 N/mm を仮定した値で、**1 mm のパッドでは成り立たない**。訂正する。")
    A("- **減速の効果は、距離が要る**: 1,000 °/s² で 120 → 60 °/s に落とすには 5.4°、120 → 5 °/s に 7.2° が要る。**上限 +3°（余裕 2°）では 43 °/s までしか減速できない**（位置の誤差の最大 {:.2f}° を引く）→ **窓の端の手前の速さの上限 40 °/s** を設定にする。".format(mc["max"]))
    A(f"- **静的荷重**: 既定の上限で 2.9〜19.5 N、上限が効かない場合 29〜83 N（steel 147 N。SF 1.8〜5）。**上限のレジスタが効いているかは HT-002 / HT-003 で確認するまで未確認**。\n")

    A("## 5. prior の幅への依存（モンテカルロ）\n")
    A("| 量 | prior | 結果 | 結論が変わる条件 |\n|---|---|---|---|")
    A(f"| 位置の誤差 δθ の最悪側（量子化 + D + B） | D 0〜4、B 0〜8（ENC_MOTOR 0.5）| p50 {mc['p50']:.2f}° / p95 {mc['p95']:.2f}° / 最大 {mc['max']:.2f}° | J1-1 / J1-2 の実測で D + B が小さければ +2° / −3° まで狭められる（実現性 {fe['hi'][2.0]['p_feasible']:.0%}） |")
    A(f"| 上限 +2°（危険 10 mm³）の実現性 | 上の prior | **{fe['hi'][2.0]['p_feasible']:.0%}**（+3° は {fe['hi'][3.0]['p_feasible']:.0%}、+1.5° は {fe['hi'][1.5]['p_feasible']:.0%}）| D + B が ≤ 約 6 ステップ（0.5°）と実測されたら +2° |")
    A(f"| 窓の端の手前の速さの下限（30 → 60 °/s）| 30 °/s | 30: [{rc['lo']:g}, +{rc['hi']:g}] → 60: [{(rec60['cover'] or {}).get('lo', float('nan')):g}, +{(rec60['cover'] or {}).get('hi', float('nan')):g}]（最悪 {fm((rec60['cover'] or {}).get('worst', float('nan')))} mm³）。90 °/s 以上は下限の候補（−5°）では成り立たない | 30 °/s で足りるか（J1 は小さな補正だけ）。速い動きが要るなら範囲を広げる代わりに危険体積が増える |")
    A(f"| 取り込み割合（作業点）| 摩擦 0.7〜1.4 倍・床 3 種・δθ・平らさ ±0.05 | 平均 {wp['mean']:.1%}、p05 {wp['p05']:.0%} | 帯（±1°）を外れる = 位置の誤差が 1° 超（D + B > 11 ステップ）|")
    A(f"| 危険体積の不確かさ（Design の見積もり）| V(+5°) を 1/1, 1/2, 1/4, 1/8 に | 膝: {sens[1.0]['knee']:g}° / {sens[2.0]['knee']:g}° / {sens[4.0]['knee']:g}° / {sens[8.0]['knee']:g}° | **V(+5°) が Design の値の 1/4 以下（150 mm³ 以下）なら、膝は +5° に動く**（+4° は Design が未計算 = R-003 で依頼）|")
    A("| ピンの強度・J・k | PLA 20〜50 / 鋼 200〜350 MPa、J 1e-3〜1e-2、k = TPU + ピン + 窓 | 鋼 + TPU 3 mm: SF p05 4.7（120°/s）| J が prior の上限の 10 倍（1e-1）だと衝撃は約 3 倍になり、SF p05 は 1.5 前後まで下がる（見積もり。HT-003 で J・k を測る） |\n")

    A("## 6. 選んだ案と「膝」\n")
    A("- **推す案: [−4°, +3°]（覆いあり）、作業窓 −2〜+1°、窓の端の手前 2° で ≤ 40 °/s、鋼ダウエル φ3 + TPU パッド（厚さ 3〜6 mm）**。最悪の危険体積 44 mm³（+3° の上の輪）。")
    A("- **膝の手前 = +3°**: +3° → +5° で危険体積が 44 → 601 mm³（13.7 倍）に急増する。+3° は、位置の誤差の最大 + 減速の余裕を 100% の prior で満たす最小の上限（+2° は 85%）。")
    A("- **覆いなしの場合**: 下側が −4° で 205 mm³（補間）になり、覆いを採る場合の約 5 倍。覆い（Design の STL、襟が前へ 7 mm 伸びる）は**見た目の判断（User）**。採らないなら、下限を −3°（105 mm³）にして実現性 85% を受け入れる（D + B の実測後に確定）。")
    A("- **姿勢の損失**（5 グループ + 呼吸）は、**Floor Watch の頭（ストッパーが物理的に効く）に限る**。`robot.yaml` の既存の J7（−8〜90°）は変えない。表情・鎌首を残す必要が出たら、Design の別案（動く覆い / 外せるピン）。\n")

    A("## 7. 設定の変更（別コミット）と、戻し方\n")
    A("- **変えるファイル**: `config/robot.yaml` の `neck:` に**追加の設定**（既存の J7 の `min_deg` / `max_deg` は変えない）:")
    A("  - `floor_watch_soft_deg: [-2.0, 1.0]`（作業窓。Design）")
    A("  - `floor_watch_stop_deg: [-4.0, 3.0]`（機械ストッパー。PROVISIONAL。覆いあり）")
    A("  - `floor_watch_near_limit_speed_dps: 40`（窓の端の手前 2° の区間の速さの上限。PROVISIONAL）")
    A("- **戻し方**: 上の 3 行を消す（他のコードは参照していない。強制する配線は R-009 で別途。**今は設定の値だけで、動作は変わらない**）。")
    A("- **再判定の手順**: (1) HT-004（J1-1 / J1-2）で D・B を測る → `assumptions.yaml` に足す → `python simulation/hardware_gaps/HG-H1_actuator/j1_range_opt_report.py`。(2) HT-002 / HT-003 で上限の効き・J・k を測る → 同様。(3) Design が +4° と −4° の危険体積を計算したら `V_COMPUTED` に足す。\n")

    A("## 8. Design への形状条件（TO: Design。ENTRY-E-0011）\n")
    A("1. **ストッパーの窓（θ_E）**: 範囲 **−4° 〜 +3°**（φ_pin − θ_CAD の変換は Design の符号で。窓の端 = 範囲の両端 + ピンの半角）。旧案（−5〜+10° / −5〜+3°）は使わない。**実機の符号の確認（HT-001）の後に切る**。")
    A("2. **ピン: 鋼のダウエル φ3（PLA 不可）**。片持ち 4.5 mm、窓の端に **TPU パッド（厚さ 3〜6 mm、幅 3.6、E 3 MPa 級）**。窓の端面（PLA 2 mm）の支圧・たわみは Design の確認。")
    A("3. **上の輪の覆い**（870 mm³）を採るか（見た目 = User）。採らない場合の下側の危険体積は 205 mm³（−4°）。")
    A("4. **+4° と −4° の危険体積を計算**してほしい（補間の点。膝の位置の確認）。+5° の値が Design の 1/4 以下なら膝が動く。")
    A("5. 範囲が **+3° に狭まる**ので、下のくさび（頭を上げる側）は 44 mm³ 以下。**上げ側の覆いは不要**（動く覆いも不要）。姿勢（home 8° など）を残す要求が出たら、外せるピンの概念を依頼。\n")

    A("## 9. 限界\n")
    A("- 危険体積は Design の CAD_CONCEPT（外形 STL・ボクセル・内部部品なし・公差なし）。**+4° などは補間**。危険体積が実際の挟み込みの傷害の大きさを表すとは言えない（体積は指標）。")
    A("- 取り込み割合の θ → c の写像は Design の幾何（ASSUMED）。前縁が床に触れたあとの押し込み（弾性・スキッドのたわみ）は入っていない。絨毯は未対応。")
    A("- 衝撃は ω√(J k) の式（HG-H1 と同じ）。J・k・ピンの強度・減速度は prior。**HT-002 / HT-003 の実測が要る**。")
    A("- 「姿勢の損失」は robot.yaml の J7 の目標との比較で、実際の行動の使われ方（頻度）は見ていない。\n")
    (RES / "j1_range_opt_2026-09-30.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    print("\n".join(L))


if __name__ == "__main__":
    main()
