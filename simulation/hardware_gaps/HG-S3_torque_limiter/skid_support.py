r"""頭スキッドで体の前側を支えるときの J1（Engineering の J7）の負荷（OQ-0105 / LB-E-010）。**ANALYTIC + Monte Carlo（prior）。PROVISIONAL。実機で未確認。**

    .\.venv\Scripts\python.exe simulation\hardware_gaps\HG-S3_torque_limiter\skid_support.py

問い: 横スキッドの前端が床に触れて**体の前側の重さの一部を受ける**とき、J1 の保持トルクはどこまで上がるか。
モデル（静的。剛体・摩擦なし・鉛直の力だけ）:
  τ_J1 = W_skid × L_skid + m_head × g × l_cog
    W_skid  = 頭スキッドが受ける重さ = f × M_total × g × k_dyn
    f       = 体の重さのうち頭スキッドが受ける割合（**未知**。0〜0.5 を掃引。0.5 は「前半分が頭で支えられる」極端な仮定）
    k_dyn   = 段差・揺れの動的な割増（1〜2。ASSUMED。実測なし）
    L_skid  = J1 軸からスキッド前端までの腕（40〜48 mm。ASSUMED。HT-002 の 44.2 mm の前後）
    m_head, l_cog = Design の頭（殻 + 電子部品 147〜254 g、重心 19 mm）。`head_mass_budget.py` と同じ入力
  頭の自重の向きとスキッドの反力の向きは、最悪の組み合わせとして**足し合わせる**（保守側）。
入力の出典: 質量の合計 = `config/robot.yaml` の `mass_budget_g`（1000 g）+ 頭の増分 + `mass_additions_g`（PROPOSED）。**Design の CAD にスキッドの荷重の分担の計算は無い**（f は Design の領分: 重心・接触点の配置）。
読み方: **これは f を決めない**。f がいくつなら J1 のソフトの上限（0.45 N·m）の何 % か、という判断境界を出す。5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）で、ここでは使わない。
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
G = 9.81
SOFT_CAP_NM = 0.45
L_SKID_MM = (40.0, 48.0)          # J1 軸からスキッド前端までの腕（ASSUMED。HT-002 の 44.2 mm の前後）
K_DYN = (1.0, 2.0)                # 動的な割増（ASSUMED）
F_MAX = 0.5                       # 頭スキッドが受ける体の重さの割合の上限（ASSUMED の極端な仮定）


def _load(name: str, rel: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / rel)
    m = importlib.util.module_from_spec(spec)
    sys.modules[name] = m
    spec.loader.exec_module(m)
    return m


def masses() -> dict:
    """頭の質量・重心・全体質量の入力。**2026-10-05（R-030）: Design の Fusion v3（113.8 g、重心 J1 軸の 16.2 mm 前）+ 電子部品 25〜30 g（ASSUMED）= 139〜144 g に更新**
    （以前は D-0015 の殻 137〜234 g + 電子 10〜20 g = 147〜254 g、重心 19 mm。D-0020 / D-0021 で撤回された値）。"""
    hb = _load("head_mass_budget_for_skid", "simulation/hardware_gaps/HG-S3_torque_limiter/head_mass_budget.py")
    b = hb.budget()
    heads = [hb.V3_HEAD_G + e for e in hb.V3_ELEC_G]
    t_lo = hb.total_mass_g(min(heads) - b["head_total"])["min"]
    t_hi = hb.total_mass_g(max(heads) - b["head_total"])["max"]
    return {"head_lo": min(heads), "head_hi": max(heads), "total_lo": t_lo, "total_hi": t_hi, "cog_mm": hb.V3_COG_X_MM}


def j1_torque_nm(f: float, total_g: float, k_dyn: float, l_skid_mm: float, head_g: float, cog_mm: float) -> float:
    """f = 体の重さのうち頭スキッドが受ける割合。τ = f M g k L + m_head g l_cog（最悪の足し合わせ）。"""
    return f * total_g / 1000.0 * G * k_dyn * l_skid_mm / 1000.0 + head_g / 1000.0 * G * cog_mm / 1000.0


def f_for_fraction_of_cap(frac: float, total_g: float, k_dyn: float, l_skid_mm: float, head_g: float, cog_mm: float) -> float:
    """J1 の静的トルクがソフトの上限の frac になる f（> 1 なら体の全重量でも届かない）。"""
    tau_head = head_g / 1000.0 * G * cog_mm / 1000.0
    return (frac * SOFT_CAP_NM - tau_head) / (total_g / 1000.0 * G * k_dyn * l_skid_mm / 1000.0)


def mc(n: int = 20000, seed: int = 7) -> dict:
    m = masses()
    r = np.random.default_rng(seed)
    f = r.uniform(0.0, F_MAX, n)
    kd = r.uniform(*K_DYN, n)
    ls = r.uniform(*L_SKID_MM, n)
    hg = r.uniform(m["head_lo"], m["head_hi"], n)
    tg = r.uniform(m["total_lo"], m["total_hi"], n)
    tau = f * tg / 1000.0 * G * kd * ls / 1000.0 + hg / 1000.0 * G * m["cog_mm"] / 1000.0
    return {"tau": tau, "p50": float(np.percentile(tau, 50)), "p95": float(np.percentile(tau, 95)), "max": float(tau.max()),
            "p_gt": {x: float(np.mean(tau > x * SOFT_CAP_NM)) for x in (0.5, 0.75, 1.0)}, "m": m}


def main() -> None:
    m = masses()
    res = mc()
    L: list[str] = []
    A = L.append
    A("# 頭スキッドで体の前側を支えるときの J1 の負荷（OQ-0105。2026-10-01）\n")
    A("**ANALYTIC + Monte Carlo（prior）。PROVISIONAL。実機で未確認（HARDWARE_VERIFIED = 0）。f（頭スキッドが受ける体の重さの割合）・k_dyn・L_skid は ASSUMED。"
      "5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）で、ここでは使わない。安全・合格の語は使わない。** 再現: `simulation/hardware_gaps/HG-S3_torque_limiter/skid_support.py`。\n")
    A(f"入力: 頭 {m['head_lo']:g}〜{m['head_hi']:g} g（Design の Fusion v3 113.8 g + 電子 25〜30 g = ASSUMED。2026-10-05 に更新。重心 {m['cog_mm']:g} mm）、全体 {m['total_lo']:.0f}〜{m['total_hi']:.0f} g（`head_mass_budget.py`）、"
      f"スキッド前端の腕 {L_SKID_MM[0]:g}〜{L_SKID_MM[1]:g} mm、動的な割増 {K_DYN[0]:g}〜{K_DYN[1]:g}、f = 0〜{F_MAX:g}。\n")
    A("## 1. f を掃引した J1 の静的トルク（中央値の入力: 全体・頭・腕・割増の中央値）\n")
    tg, hg = (m["total_lo"] + m["total_hi"]) / 2, (m["head_lo"] + m["head_hi"]) / 2
    ls, kd = sum(L_SKID_MM) / 2, sum(K_DYN) / 2
    A(f"（全体 {tg:.0f} g、頭 {hg:.0f} g、腕 {ls:g} mm、割増 {kd:g}）\n")
    A("| f | J1 の静的トルク [N·m] | ソフトの上限 0.45 N·m に対する割合 |\n|---|---|---|")
    for f in (0.0, 0.1, 0.2, 0.3, 0.4, 0.5):
        t = j1_torque_nm(f, tg, kd, ls, hg, m["cog_mm"])
        A(f"| {f:g} | {t:.3f} | {100 * t / SOFT_CAP_NM:.0f}% |")
    A("")
    A("## 2. 判断境界: J1 の静的トルクが上限の X % になる f\n")
    A("| X | f（中央値の入力）| f（最悪の入力: 全体最大・割増 2・腕 48 mm・頭最大）| f（最良の入力）|\n|---|---|---|---|")
    for x in (0.5, 0.75, 1.0):
        mid = f_for_fraction_of_cap(x, tg, kd, ls, hg, m["cog_mm"])
        worst = f_for_fraction_of_cap(x, m["total_hi"], K_DYN[1], L_SKID_MM[1], m["head_hi"], m["cog_mm"])
        best = f_for_fraction_of_cap(x, m["total_lo"], K_DYN[0], L_SKID_MM[0], m["head_lo"], m["cog_mm"])
        A(f"| {int(100 * x)}% | {mid:.2f} | {worst:.2f} | {best:.2f} |")
    A("\n（f > 1 は体の全重量でも届かない = その入力では上限に達しない。）\n")
    A("## 3. Monte Carlo（prior。20,000 点、seed 固定）\n")
    A(f"f ~ U(0, {F_MAX:g})、割増・腕・頭・全体は一様。J1 の静的トルク: p50 **{res['p50']:.3f}**、p95 **{res['p95']:.3f}**、最大 {res['max']:.3f} N·m。"
      f"上限 0.45 N·m の 50% を超える確率 {res['p_gt'][0.5]:.3f}、75% を超える確率 {res['p_gt'][0.75]:.3f}、100% を超える確率 {res['p_gt'][1.0]:.3f}。\n")
    A("## 4. 読み方と限界\n")
    A(f"- f = {F_MAX:g}（前半分が頭スキッドで支えられる極端な仮定）・割増 2・腕 48 mm・最大の頭でも、静的トルクは上限の **{100 * j1_torque_nm(F_MAX, m['total_hi'], K_DYN[1], L_SKID_MM[1], m['head_hi'], m['cog_mm']) / SOFT_CAP_NM:.0f}%**（表 2 の境界を見る）。")
    A("- **f は Design の領分（重心・接触点・脚の配置）で、CAD に分担の計算が無い**。この計算は f を決めない。f の上限を Design に依頼する（R-027）。")
    A("- 静的・鉛直の力だけ。横スキッドの摩擦（床との水平の力 × 腕）、頭が床で滑るときの動的なトルク、サーボのバックドライブ（HT-004）は入っていない。**実測が要る**（HT-002 の押す力の装置で、頭の荷重を段階的に増やす）。")
    A("- 静的トルクがソフトの上限以内でも、**保持（ストールに近い）状態が続くと温度・電流が上がる**（HG-H1。H1 の実測まで未評価）。")
    out = ROOT / "simulation" / "results" / "skid_support_j1_2026-10-01.md"
    out.write_text("\n".join(L) + "\n", encoding="utf-8", newline="\n")
    print("\n".join(L))
    print("wrote", out)


# ---- スキッド底の高さの差 δ と、頭スキッドが受ける割合 f（R-031。2026-10-05）-----------------------------------
def skid_share(ks: float, kb: float, delta_mm: float, wf_n: float) -> float:
    """体の前側の重さ wf_n [N] を、スキッド（押し込み剛性 ks [N/mm]）と足の帯（kb [N/mm]）が**並列**に床で受けるとき、スキッドの分担 f。
    前側を剛体とみなし、帯の底を基準にスキッド底が δ [mm] 低い（δ > 0 = スキッドが出ている、δ < 0 = 上げてある）。共通の沈み z（帯）で
    F_b = kb z、F_s = ks max(0, z + δ)、F_s + F_b = wf → F_s = ks (wf + kb δ) / (ks + kb)（スキッドが接しているとき）、f = F_s / wf を [0, 1] に切る。
    **ANALYTIC。剛体・並列ばね・床は線形の仮定（ASSUMED）。k は未測定（HT-012）。実機で未確認。**"""
    f = ks / (ks + kb) * (1.0 + kb * delta_mm / wf_n)
    return max(0.0, min(1.0, f))


def delta_report() -> str:
    m = masses()
    wf = 0.5 * m["total_hi"] / 1000.0 * G                              # 体の前半分 × 全体最大（保守側）
    L = ["# スキッド底の高さの差 δ と、頭スキッドが受ける割合 f（R-031。2026-10-05）\n",
         "**ANALYTIC（剛体・並列ばね・線形の床）。床の押し込み剛性 k は未測定（HT-012）。実機で未確認。HARDWARE_VERIFIED = 0。5.7 N・0.25 N·m は暫定で使わない。** "
         f"体の前側の重さ = 全体最大 {m['total_hi']:.0f} g の半分 = {wf:.1f} N。f = ks/(ks+kb) × (1 + kb δ / 前側の重さ)（0〜1 に切る）。δ > 0 = スキッドが帯より出ている。\n",
         "## 1. ks = kb（スキッドと帯が同じ剛さ）\n", "| k [N/mm]（マット → フローリング）| δ = −1.0 | −0.5 | 0 | +0.5 |", "|---|---|---|---|---|"]
    for k in (0.3, 1.0, 3.0, 10.0, 30.0):
        L.append(f"| {k:g} | " + " | ".join(f"{skid_share(k, k, d, wf):.2f}" for d in (-1.0, -0.5, 0.0, 0.5)) + " |")
    L += ["\n## 2. ks = 2 kb（スキッドの方が硬い）\n", "| k（kb）[N/mm] | δ = −1.0 | −0.5 | 0 | +0.5 |", "|---|---|---|---|---|"]
    for k in (0.3, 1.0, 3.0, 10.0, 30.0):
        L.append(f"| {k:g} | " + " | ".join(f"{skid_share(2 * k, k, d, wf):.2f}" for d in (-1.0, -0.5, 0.0, 0.5)) + " |")
    L += ["\n## 3. 読み（Engineering の見立て。PROPOSED。決めるのは Design / Integration）\n",
          "- **危険なのは δ の公差が正に振れること**: 硬い床（k ≥ 10 N/mm）では、δ = +0.5 mm でスキッドが体の前側をほぼ全部（f 0.9〜1.0）受け、δ = 0 でも ks/(ks+kb) = 0.5〜0.67。",
          "- **δ < 0（スキッドを上げる）は、硬い床の f を 0 近くまで下げる**（δ = −1 で k ≥ 10 → 0、−0.5 で 0.07〜0.09）。**厳しくする側なので Engineering に異論は無い**。印刷の公差 ±0.5 mm を見込んで**公称 −0.75 mm（−0.25〜−1.25）**にすれば、公差を含めても δ が正にならない。",
          "- **ただし柔らかいマット（k ≲ 1 N/mm）の f はほとんど下がらない**（δ = −1 でも 0.41〜0.55）。**f の上限（E-0015 の仮定 0.5）はマットで決まり、δ では変わらない**。ks が kb の 2 倍なら 0.67 まで。R-027 の f の上限は、k の実測（HT-012）が出るまで 0.5〜0.67 を置く。",
          "- 副作用: 口の前縁が |δ| だけ上がる（c が増える）。取り込み率の表（`j1_range_opt.py` の intake）では c = 1.1 mm でも 0.95〜0.96、c ≤ 0.6 mm は 1.0。J1 で補正できる範囲（−4〜+3°）なら吸収できる。ToF の床との距離は |δ| mm 増える（崖の判定の Δ に対して小さい）。"
          "顎の接地感（Design の見た目）は Design の領分。"]
    return "\n".join(L) + "\n"


if __name__ == "__main__":
    if "--delta" in sys.argv:
        txt = delta_report()
        (ROOT / "simulation" / "results" / "skid_delta_2026-10-05.md").write_text(txt, encoding="utf-8", newline="\n")
        print(txt)
    else:
        main()
