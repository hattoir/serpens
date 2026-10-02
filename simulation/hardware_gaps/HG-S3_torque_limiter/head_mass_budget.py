r"""頭の質量予算 90 g と、Design の殻の見積もり 137〜234 g のずれの影響（LB-E-001 / R-021 / ENTRY-D-0015 (4)）。**ANALYTIC + ACTUATOR_MODEL_SIM。PROVISIONAL。実機で未確認。**

    .\.venv\Scripts\python.exe simulation\hardware_gaps\HG-S3_torque_limiter\head_mass_budget.py

入力（出典）:
  - 殻の質量: Design（ENTRY-D-0015 (4)、`head_e3_integrated_v2_notes_2026-09-30.md` §4）= 殻 1.2 / 1.6 / 2.0 / 2.5 mm で 137 / 169 / 199 / 234 g（Fusion の体積・表面積。殻 ≈ 表面積 × 厚さ。密度は ASSUMED）。**Design の見積もり（Engineering 未検証）**
  - 電子部品（XIAO ESP32S3 Sense・ToF 2・LED・配線）: 殻の見積もりには入っていない。**10 / 15 / 20 g は ASSUMED**（実測なし。部品の質量の一次資料は未確認）
  - 重心: Design の (−200, +0.4, 38)。**J1 軸の約 19 mm 前**（D-0015 (4)）。Engineering の config（`neck_lifted_mass` 200 g・重心 104 mm = 従来の J7 の保守側の基準）は変えない
  - 質量の上限 `safety_limits.mass_total_g_max` = 1700 g、`mass_budget_g`（1000 g）、`mass_additions_g`（PROPOSED。収支には含まれない）
読み方: **この計算は予算を決めない**。config の `head_total: 90` は変更しない（変更は PROPOSED として ENTRY に書く）。5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）で、ここでは使わない。
"""
from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
G = 9.81
SOFT_CAP_NM = 0.45                      # ソフトの上限（レジスタ比 0.167 × ストール 2.687 N·m）
SHELL_G = {1.2: 137.0, 1.6: 169.0, 2.0: 199.0, 2.5: 234.0}      # Design の見積もり（殻の厚さ mm → 質量 g）
ELEC_G = (10.0, 15.0, 20.0)             # 電子部品（ASSUMED）
DESIGN_COG_MM = 19.0                    # 頭の重心の J1 軸からの距離（Design。CAD_CONCEPT）
CFG_COG_MM = 104.0                      # config の保守側の重心の距離（neck_lifted_cog_mm）
# Design の Fusion 実体（HEAD E3 INTEGRATED v3。ENTRY-D-0021、2026-10-02。CAD_CONCEPT）: シェル 77.7 + 取り込み 23.8 + スキッド 10.3 = 113.8 g、重心 (−198.0, +3.1, 36.1)。
# D-0015 の「殻 137〜234 g」は**取り込み・スキッドを含む全体の値だった**（比較の基準の誤り。D-0021 で訂正）。電子部品（XIAO・ToF・LED・配線）は殻に含まれない: Design の見積もり 約 25〜30 g
V3_HEAD_G = 113.8
V3_ELEC_G = (25.0, 30.0)
V3_COG_X_MM = 198.0 - 181.8             # J1 軸（X −181.8。CAD.md）からの重心の前後距離 = 16.2 mm
V3_COG_Y_MM = 3.1


def _load(name: str, rel: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / rel)
    m = importlib.util.module_from_spec(spec)
    sys.modules[name] = m
    spec.loader.exec_module(m)
    return m


def cfg() -> dict:
    return yaml.safe_load((ROOT / "config" / "robot.yaml").read_text(encoding="utf-8"))


def budget() -> dict:
    c = cfg()
    b = c["mass_budget_g"]
    total = (b["servo_each"] * b["servo_count"] + b["segment_frame_each"] * b["segment_frame_count"] + b["passive_wheels_total"]
             + b["head_total"] + b["skin_and_wiring"])
    return {"head_total": float(b["head_total"]), "neck_lifted_mass": float(b["neck_lifted_mass"]), "neck_lifted_cog_mm": float(b["neck_lifted_cog_mm"]),
            "total": float(total), "cap": float(c["safety_limits"]["mass_total_g_max"]), "additions": c["mass_additions_g"]}


def head_total_g(shell_mm: float, elec_g: float) -> float:
    return SHELL_G[shell_mm] + elec_g


def static_torque_nm(delta_g: float, lever_mm: float) -> float:
    """config の J7 静的トルク（200 g × 104 mm）に、頭の増分 delta_g を J1 軸から lever_mm の位置で足したときの [N·m]。
    lever = 19 mm（Design の重心）= 増分が頭の重心にあるとき。lever = 104 mm = 増分が config の重心と同じ距離にあるとき（保守側の上限）。"""
    b = budget()
    base = b["neck_lifted_mass"] / 1000.0 * G * b["neck_lifted_cog_mm"] / 1000.0
    return base + delta_g / 1000.0 * G * lever_mm / 1000.0


def head_only_torque_nm(m_g: float, lever_mm: float = DESIGN_COG_MM) -> float:
    """頭だけ（Design の幾何: 重心 19 mm）の静的トルク [N·m]。Design の design-state の 0.020 N·m（93 g）と同じ種類の値。"""
    return m_g / 1000.0 * G * lever_mm / 1000.0


def total_mass_g(delta_g: float) -> dict:
    """全体質量 = 収支 1000 g + 頭の増分 + 追加（KNUCKLE DRUM・頭の下の後ろの詰め物・取り込みの頭）。追加は PROPOSED（収支に含まれない）。最小 / 最大。"""
    b = budget()
    add = b["additions"]
    lo = b["total"] + delta_g + add["knuckle_drum_hollow_shell_1p2mm_max"]["total"] + add["head_lower_rear_fill_solid"] + add["intake_head_hollow_assumed"][0]
    hi = b["total"] + delta_g + add["knuckle_drum_solid_max"]["total"] + add["head_lower_rear_fill_solid"] + add["intake_head_solid"][1]
    return {"min": lo, "max": hi, "cap": b["cap"]}


def load_inertia_kgm2(m_g: float, lever_mm: float) -> float:
    """頭（点質量）の J1 軸まわりの慣性 M L²（頭自身の慣性は含めない = 下限）。"""
    return m_g / 1000.0 * (lever_mm / 1000.0) ** 2


def y_limit_mm(m_g: float, k_floor_n_mm: float = 1.0) -> float:
    """頭の重心 y の偏りの許容（左右のたわみの差 ≤ 0.05 mm）。質量に反比例（`j1_head_pitch_check.head_cog_y_limit_mm`、k = 1 N/mm は ASSUMED）。"""
    chk = _load("j1_head_pitch_check", "simulation/hardware_gaps/HG-H1_actuator/j1_head_pitch_check.py")
    return chk.head_cog_y_limit_mm(k_floor_n_mm, m_kg=m_g / 1000.0)


def stopper_rows(n: int = 8000) -> list[dict]:
    """ストッパーの衝撃: 頭の慣性 M L² を、ロータの反映慣性 prior（1e-3〜1e-2 kg·m²。UNKNOWN）に足したときの変化（鋼ダウエル、120 °/s、パッド P1 = E 26 MPa・t 3 mm / P2 = E 10 MPa・t 5 mm）。"""
    o = _load("j1_range_opt", "simulation/hardware_gaps/HG-H1_actuator/j1_range_opt.py")
    shell_lo, shell_hi = min(SHELL_G), max(SHELL_G)
    cases = {
        "頭の慣性なし（従来の計算）": 0.0,
        "最小（137 g + 10 g、重心 19 mm）": load_inertia_kgm2(head_total_g(shell_lo, ELEC_G[0]), DESIGN_COG_MM),
        "最大・Design の重心（234 g + 20 g、19 mm）": load_inertia_kgm2(head_total_g(shell_hi, ELEC_G[2]), DESIGN_COG_MM),
        "最大・config の重心（234 g + 20 g、104 mm = 保守側の上限）": load_inertia_kgm2(head_total_g(shell_hi, ELEC_G[2]), CFG_COG_MM),
    }
    rows = []
    for pad, e, t in (("P1（E 26 MPa・t 3 mm）", 26.0, 3.0), ("P2（E 10 MPa・t 5 mm）", 10.0, 5.0)):
        for name, j in cases.items():
            for decel in (0.0, 2.0):
                r = o.stopper_mc(n=n, omega=120.0, decel_deg=decel, tpu=True, material="steel", a_decel=o.A_DECEL_MIN, e_mpa=e, t_mm=t, j_load_kgm2=j)
                rows.append({"pad": pad, "case": name, "j_load": j, "decel_deg": decel, "impact_p50": r["impact_p50"], "load_p95": r["load_p95"],
                             "sf_p05": r["sf_p05"], "p_sf_lt_1p5": r["p_sf_lt_1p5"]})
    return rows


def main() -> None:
    b = budget()
    L: list[str] = []
    A = L.append
    A("# 頭の質量予算 90 g と、Design の殻の見積もり（137〜234 g）のずれの影響（2026-10-01）\n")
    A("**ANALYTIC + ACTUATOR_MODEL_SIM。PROVISIONAL。実機で未確認（HARDWARE_VERIFIED = 0）。Design の殻の質量は Design の見積もり（CAD_CONCEPT、Engineering 未検証）。電子部品の質量は ASSUMED。"
      "5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）で、ここでは使わない。安全・合格の語は使わない。** 再現: `simulation/hardware_gaps/HG-S3_torque_limiter/head_mass_budget.py`。\n")
    A(f"config（変更しない）: `head_total` = {b['head_total']:g} g、`neck_lifted_mass` = {b['neck_lifted_mass']:g} g・重心 {b['neck_lifted_cog_mm']:g} mm（J7 の保守側の基準）、収支の合計 = {b['total']:g} g、`mass_total_g_max` = {b['cap']:g} g。\n")
    A("## 0. 【最新】Design の Fusion 実体（v3、ENTRY-D-0021）での値\n")
    A(f"頭 = シェル 77.7 + 取り込み 23.8 + スキッド 10.3 = **{V3_HEAD_G:g} g**（CAD_CONCEPT）+ 電子部品 {V3_ELEC_G[0]:g}〜{V3_ELEC_G[1]:g} g（Design の見積もり）= **{V3_HEAD_G + V3_ELEC_G[0]:g}〜{V3_HEAD_G + V3_ELEC_G[1]:g} g**（90 g の {(V3_HEAD_G + V3_ELEC_G[0]) / b['head_total']:.2f}〜{(V3_HEAD_G + V3_ELEC_G[1]) / b['head_total']:.2f} 倍）。"
      f"重心は J1 軸の {V3_COG_X_MM:g} mm 前、y {V3_COG_Y_MM:+g} mm。**§1 以降は、D-0015 の旧い見積もり（137〜234 g を『殻』として電子部品を足していた = 二重計上）での表。比較のために残す（最新は上の v3）。**\n")
    A("| 項目 | 頭 [g] | (a) 増分を 16.2 mm に置く [N·m]（割合）| (a) 増分を 104 mm に置く（保守側）[N·m]（割合）| (b) 頭だけ 16.2 mm [N·m]（割合）| 全体質量の最大 [g]（上限 1700 g に対する割合）|\n|---|---|---|---|---|---|")
    for e in V3_ELEC_G:
        m = V3_HEAD_G + e
        d = m - b["head_total"]
        ta, tb, tc = static_torque_nm(d, V3_COG_X_MM), static_torque_nm(d, CFG_COG_MM), head_only_torque_nm(m, V3_COG_X_MM)
        tm = total_mass_g(d)
        A(f"| v3 + 電子 {e:g} g | {m:g} | {ta:.3f}（{100 * ta / SOFT_CAP_NM:.0f}%）| {tb:.3f}（{100 * tb / SOFT_CAP_NM:.0f}%）| {tc:.3f}（{100 * tc / SOFT_CAP_NM:.0f}%）| {tm['max']:.0f}（{100 * tm['max'] / tm['cap']:.0f}%）|")
    m_v3 = V3_HEAD_G + V3_ELEC_G[1]
    lim = y_limit_mm(m_v3, 1.0)
    A(f"\n重心 y の許容（左右のたわみの差 ≤ 0.05 mm。k = 1 N/mm は ASSUMED）: 頭 {m_v3:g} g で **{lim:.2f} mm**（k に比例）。Design の v3 の y = {V3_COG_Y_MM:+g} mm は、**k ≥ {V3_COG_Y_MM / lim:.2f} N/mm なら内側**（Design の「余裕小」と同じ読み。HT-012 で k を実測）。")
    ov = load_inertia_kgm2(m_v3, V3_COG_X_MM)
    oc = load_inertia_kgm2(m_v3, CFG_COG_MM)
    A(f"頭の慣性（点質量。ストッパーの衝撃の J に足す）: Design の重心で {ov:.2e} kg·m²（ロータの反映慣性 prior の下限 1e-3 の {100 * ov / 1e-3:.0f}%）、config の保守側の重心（104 mm）で {oc:.2e}（同 {100 * oc / 1e-3:.0f}%）。")
    A("**読み**: config の `head_total: 90` は変えない（PROPOSED）。v3 は 90 g を約 54〜60% 超える（Design の「シェル + 電子で約 105 g、取り込み 23.8 g は別枠 19 g 超過」と同じ向き）。J7 の静的トルクは保守側の基準でも上限の約 6 割（56〜58%）、Design の幾何では 5%。重心 y は k ≥ 約 1.1 N/mm（ASSUMED の 1 N/mm のすぐ上）が要り、**余裕が小さい**（HT-012 で k を実測）。**実測 = HT-016。**\n")
    A("## 1. （旧）頭の質量（殻 + 電子部品）と、config の 90 g に対する倍率 — D-0015 の見積もり。D-0021 で全体の値と訂正されたので二重計上\n")
    A("| 殻の厚さ [mm] | 殻 [g]（Design）| + 電子部品 10 / 15 / 20 g（ASSUMED）| 90 g に対する倍率 |\n|---|---|---|---|")
    for s, m in SHELL_G.items():
        tot = [head_total_g(s, e) for e in ELEC_G]
        A(f"| {s:g} | {m:g} | {tot[0]:g} / {tot[1]:g} / {tot[2]:g} | {tot[0] / b['head_total']:.2f}〜{tot[2] / b['head_total']:.2f} 倍 |")
    A("")
    A("## 2. J7（Design の J1）の静的トルク（ソフトの上限 0.45 N·m に対する割合）\n")
    A("2 つの置き方で出す。**(a) config の基準（200 g・104 mm = 0.204 N·m）に、頭の増分を足す**（増分を Design の重心 19 mm に置く場合 / config の重心 104 mm に置く保守側の上限の場合）。"
      "**(b) 頭だけを Design の幾何（重心 19 mm）で見る**（Design の design-state の 0.020 N·m と同じ種類。`neck_lifted` の 200 g を使わない）。\n")
    A("| 頭の質量 [g]（殻 + 電子 15 g）| 増分（− 90 g）[g] | (a) 増分を 19 mm に置く [N·m]（割合）| (a) 増分を 104 mm に置く [N·m]（割合）| (b) 頭だけ 19 mm [N·m]（割合）|\n|---|---|---|---|---|")
    base = static_torque_nm(0.0, DESIGN_COG_MM)
    A(f"| 90（config。基準）| 0 | {base:.3f}（{100 * base / SOFT_CAP_NM:.0f}%）| {base:.3f}（{100 * base / SOFT_CAP_NM:.0f}%）| {head_only_torque_nm(90.0):.3f}（{100 * head_only_torque_nm(90.0) / SOFT_CAP_NM:.0f}%）|")
    worst_hi = 0.0
    for s in SHELL_G:
        m = head_total_g(s, 15.0)
        d = m - b["head_total"]
        ta, tb, tc = static_torque_nm(d, DESIGN_COG_MM), static_torque_nm(d, CFG_COG_MM), head_only_torque_nm(m)
        worst_hi = max(worst_hi, tb)
        A(f"| {m:g}（殻 {s:g} mm）| +{d:g} | {ta:.3f}（{100 * ta / SOFT_CAP_NM:.0f}%）| {tb:.3f}（{100 * tb / SOFT_CAP_NM:.0f}%）| {tc:.3f}（{100 * tc / SOFT_CAP_NM:.0f}%）|")
    m_max = head_total_g(max(SHELL_G), ELEC_G[2])
    tb_max = static_torque_nm(m_max - b["head_total"], CFG_COG_MM)
    A(f"\n最大（殻 2.5 mm + 電子 20 g = {m_max:g} g）の保守側の上限 (a) 104 mm: **{tb_max:.3f} N·m（{100 * tb_max / SOFT_CAP_NM:.0f}%）**。"
      "静的トルクだけで、動かすときの加速のトルク・外力・床反力は含まない。**ソフトの上限 0.45 N·m に対する余裕の見積もりであって、合否ではない。**\n")
    A("静的トルクが上限の X % に達する頭の質量（(a) 増分を 104 mm に置く保守側。参考の表。**X は基準でなく、読みやすさのための刻み**）:\n")
    A("| X | その質量 [g] |\n|---|---|")
    for x in (50, 60, 75, 100):
        # 0.45 * x/100 = (200 g * 104 mm + d * 104 mm) g  →  d = (0.45 x /100 / (G * 0.104)) * 1000 - 200
        m_total_lift = (SOFT_CAP_NM * x / 100.0) / (G * CFG_COG_MM / 1000.0) * 1000.0
        A(f"| {x}% | 頭の質量 {m_total_lift - b['neck_lifted_mass'] + b['head_total']:.0f} g（`neck_lifted_mass` {m_total_lift:.0f} g）|")
    A("")
    A("## 3. 全体質量（上限 1700 g）\n")
    A("収支 1000 g + 頭の増分 + 追加（KNUCKLE DRUM 55.8〜61.1 g、頭の下の後ろの詰め物 1.8 g、取り込みの頭 中空 10〜19 g / 中実 55〜65 g。`mass_additions_g` は PROPOSED で、収支に含まれていない）。\n")
    A("| 頭の質量 [g] | 増分 [g] | 全体の最小 [g]（割合）| 全体の最大 [g]（割合）|\n|---|---|---|---|")
    for s in SHELL_G:
        for e in (ELEC_G[0], ELEC_G[2]):
            m = head_total_g(s, e)
            t = total_mass_g(m - b["head_total"])
            A(f"| {m:g}（殻 {s:g} + 電子 {e:g}）| +{m - b['head_total']:g} | {t['min']:.0f}（{100 * t['min'] / t['cap']:.0f}%）| {t['max']:.0f}（{100 * t['max'] / t['cap']:.0f}%）|")
    A("\n9 軸の収支（EX-1）を基準にした値。Floor Watch MVP は 5 サーボ（CAD）で収支が別になる（`mass_budget_g` は 9 サーボのまま = 古い。**5 サーボ / 6 モーターの最終は User の判断**）。\n")
    A("## 4. ストッパーの衝撃への影響（頭の慣性 M L² を足す）\n")
    A("衝撃の式 F = ω√(J k)。J = ロータの反映慣性の prior（1e-3〜1e-2 kg·m²。**UNKNOWN**）に、頭の慣性 M L² を足した（点質量 = 下限）。鋼ダウエル、120 °/s、パッド P1 / P2。MC 8,000 点（seed 固定）。\n")
    A("| パッド | 頭の慣性 | J の追加 [kg·m²] | 端の手前の減速 [°] | 衝撃 p50 [N] | 荷重 p95 [N] | SF p05 | P(SF < 1.5) |\n|---|---|---|---|---|---|---|---|")
    rows = stopper_rows()
    for r in rows:
        A(f"| {r['pad']} | {r['case']} | {r['j_load']:.2e} | {r['decel_deg']:g} | {r['impact_p50']:.1f} | {r['load_p95']:.1f} | {r['sf_p05']:.2f} | {r['p_sf_lt_1p5']:.3f} |")
    A("")
    A("## 5. 頭の重心 y の許容（左右のたわみの差 ≤ 0.05 mm）\n")
    A("許容 y = 4.2 mm × k_floor × (97 g / M)。**質量に反比例**。k = 1 N/mm は ASSUMED（HT-012 で実測）。\n")
    A("| 頭の質量 [g] | 許容 y [mm]（k = 1 N/mm）| 許容 y [mm]（k = 0.5 N/mm）|\n|---|---|---|")
    for m in (97.0, head_total_g(1.2, 15.0), head_total_g(2.5, 15.0), head_total_g(2.5, 20.0)):
        A(f"| {m:g} | {y_limit_mm(m, 1.0):.2f} | {y_limit_mm(m, 0.5):.2f} |")
    k_min = 0.4 / y_limit_mm(m_max, 1.0)
    A(f"\nDesign の頭の重心 y = +0.4 mm（SG90 を含む。D-0015 (4)）は、最大の頭（{m_max:g} g）でも、床の剛性 k が **{k_min:.2f} N/mm 以上**なら上の許容の内側（k は ASSUMED。HT-012 で実測）。\n")
    A("## 6. 読み方と、ENTRY への書き方\n")
    A("- **config の `head_total: 90` は変えない**（ここでは何も緩めない）。殻だけで 137〜234 g の頭は、90 g の予算を満たさない（1.5〜2.6 倍。Design の指摘どおり）。**予算の見直しは PROPOSED**（C に束ねる）。")
    tc_max = head_only_torque_nm(m_max)
    A(f"- J7 の静的トルクは、config の保守側の基準（増分を 104 mm に置く）で、最大の頭（{m_max:g} g）が上限の **{100 * tb_max / SOFT_CAP_NM:.0f}%**（基準の 45% から上がる）。"
      f"Design の幾何（重心 19 mm）の頭だけでは **{100 * tc_max / SOFT_CAP_NM:.0f}%**。**どちらが現実かは、実機の頭の質量・重心の実測（HT-016 案）で決まる**。")

    def pick(pad: str, case_prefix: str, decel: float) -> dict:
        return next(r for r in rows if r["pad"].startswith(pad) and r["case"].startswith(case_prefix) and r["decel_deg"] == decel)
    p1_0, p1_d, p1_c = pick("P1", "頭の慣性なし", 0.0), pick("P1", "最大・Design", 0.0), pick("P1", "最大・config", 0.0)
    p2_0, p2_c = pick("P2", "頭の慣性なし", 0.0), pick("P2", "最大・config", 0.0)
    worst_p = max(r["p_sf_lt_1p5"] for r in rows)
    A(f"- ストッパーの衝撃は、ロータの反映慣性（UNKNOWN。1e-3〜1e-2）に頭の慣性を足して見た。**Design の重心（19 mm）では変化が小さい**（P1 の衝撃 p50 {p1_0['impact_p50']:.1f} → {p1_d['impact_p50']:.1f} N）。"
      f"**config の保守側の重心（104 mm）では増える**: P1 の衝撃 p50 {p1_0['impact_p50']:.1f} → {p1_c['impact_p50']:.1f} N（+{100 * (p1_c['impact_p50'] / p1_0['impact_p50'] - 1):.0f}%）、SF p05 {p1_0['sf_p05']:.2f} → {p1_c['sf_p05']:.2f}。"
      f"P2 は {p2_0['impact_p50']:.1f} → {p2_c['impact_p50']:.1f} N、SF p05 {p2_0['sf_p05']:.2f} → {p2_c['sf_p05']:.2f}。いずれも P(SF < 1.5) は最大 {worst_p:.3f}（この prior の範囲）。**実測（HT-003）で J を決めるまで、この比較は相対値**。")
    A("- 全体質量は、上限 1700 g の範囲内（上の表）。ただし 5 サーボ・追加の確定で再計算が要る。")
    A("- **Design への依頼（急がない）**: 薄肉化（殻 1.2 mm）・目を中空・あごの空洞化（D-0015）で、頭の質量をどこまで下げられるか。目標の質量は **Engineering が決めない**（形は Design の領分）。上の表の「X % に達する質量」は参考。")
    out = ROOT / "simulation" / "results" / "head_mass_budget_2026-10-01.md"
    out.write_text("\n".join(L) + "\n", encoding="utf-8", newline="\n")
    print("\n".join(L))
    print("wrote", out)


if __name__ == "__main__":
    main()
