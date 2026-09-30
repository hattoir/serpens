r"""J3〜J5 の挟み込みの許容トルク、J1（頭–首のピッチ）の下げる側、頭に足す質量の J1 トルクを、Design の r で確認する。**PROVISIONAL / SAFETY_UNVERIFIED。**

    .\.venv\Scripts\python.exe simulation\hardware_gaps\HG-S3_torque_limiter\j1_head.py

入力（Design の ENTRY-D-0004、`docs/design/gap_check_report2_2026-09-30.md`。CAD_CONCEPT の外形、SAFETY_UNVERIFIED）と User の判断（2026-09-30）:
  1. J3〜J5 の挟み込み r = 44.4〜49.8 mm（5〜95%。全体は 42.5〜50）→ 5.7 N の暫定値でも許容トルクは約 0.25 N·m。ソフトの上限 0.45 N·m との不一致 = OPEN
  2. J1 の下げる側は r = 5〜44 mm（上げる側は 43〜52）。力の制限だけでは成立しない。動作範囲の制限か、物理的なカバーが要る
  3. 暫定 J1 範囲 −5°〜+10° を提案（符号は Design の CAD: 負 = 頭を下げる）。取り込み機構の頭下げの要否の結果を見て再判断
  4. 質量の追加は、中実で 55〜65 g、中空で 10 g 台。中空前提で重心と J1 トルクを確認する
  5. **5.7 N・0.25 N·m は暫定値。実測・規格確認なしで、安全の確定として扱わない**
r は関節軸からの動径（Design の値）。法線力に効く腕は、接線点からの距離 s の可能性がある（`flank_v.md` §B の見積もり。Design の確認待ち）。
頭・J1 の数値: Design の design-state（頭 93 g、J1 静的 0.020 N·m = ソフトの上限 0.45 N·m の 4.4%。**ASSUMED**）。
"""
from __future__ import annotations

import math
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
GAPS = HERE.parent
G = 9.81
SOFT_CAP_NM = 0.45              # ソフトの上限（レジスタ比 0.167 × ストール 2.687 N·m）
WINDOW_NM = (0.7, 1.0)          # 機械式リミッターの窓
HEAD_G, J1_STATIC_NM = 93.0, 0.020      # Design: 頭 93 g（ASSUMED）、J1 静的 0.020 N·m
R_RANGES = {                    # Design ENTRY-D-0004: 幅 5〜12.5 mm・深さ 8 mm 以上・外から届く体積の、関節軸からの動径 r [mm]
    "J3〜J5（5〜95%）": (44.4, 49.8),
    "J3〜J5（全体）": (42.5, 50.0),
    "J2（5〜95%）": (30.5, 38.0),
    "J2（全体）": (28.0, 49.0),
    "J1 下げる側": (5.0, 44.0),
    "J1 上げる側": (43.0, 52.0),
}


def thresholds() -> dict[str, float]:
    y = yaml.safe_load((GAPS / "safety_thresholds.yaml").read_text(encoding="utf-8"))
    q = y["quasi_static_contact_force_n"]
    return {k: float(q[k]["baseline"]) for k in ("hand_finger", "neck", "chest", "face")}


def allowed_nm(force_n: float, r_mm: float) -> float:
    return force_n * r_mm / 1000.0


def force_n(torque_nm: float, r_mm: float) -> float:
    return torque_nm / (r_mm / 1000.0)


def com_lever_mm() -> float:
    """現行の頭の重心の J1 からの距離（静的トルクから逆算）。"""
    return J1_STATIC_NM / (HEAD_G / 1000.0 * G) * 1000.0


def with_added_mass(m_add_g: float, d_add_mm: float) -> tuple[float, float, float]:
    """(重心の J1 からの距離 [mm]、J1 の静的トルク [N·m]、ソフトの上限に対する割合)。"""
    m0, d0 = HEAD_G, com_lever_mm()
    m = m0 + m_add_g
    lever = (m0 * d0 + m_add_g * d_add_mm) / m
    tau = m / 1000.0 * G * lever / 1000.0
    return lever, tau, tau / SOFT_CAP_NM


def hood_pitch_tolerance_deg(lever_mm: float, thickness_mm: float, clearance_mm: float = 0.1) -> float:
    """頭を上げたとき、口の前縁が床から thickness を超えないための J1 の上げ角の上限 [°]（前縁が J1 から lever だけ前）。"""
    return math.degrees(math.asin((thickness_mm - clearance_mm) / lever_mm))


def cup_pitch_for_lift_deg(lift_mm: float, lever_mm: float) -> float:
    """カップの昇降 lift を J1 の頭下げで代用するときの角度 [°]。"""
    return math.degrees(math.asin(lift_mm / lever_mm))


def main() -> None:
    th = thresholds()
    L: list[str] = []
    A = L.append
    A("# J3〜J5 の挟み込み・J1 の下げる側・頭に足す質量（Engineering、2026-09-30）\n")
    A("**PROVISIONAL / SAFETY_UNVERIFIED。** 5.7 N も、そこから出る 0.25 N·m も暫定値で、実測も規格本文の確認もしていない（`safety_thresholds.yaml`、OQ-0115）。**安全の確定として扱わない。** "
      "入力の r は Design の ENTRY-D-0004（CAD_CONCEPT の外形、SAFETY_UNVERIFIED）。再現: `simulation/hardware_gaps/HG-S3_torque_limiter/j1_head.py`。\n")
    A("## 1. Design の r での許容トルクと、ソフトの上限との不一致\n")
    A("許容トルク = しきい値 × r（動径）。0.45 N·m は関節のソフトの上限、0.7〜1.0 N·m は機械式リミッターの窓。\n")
    A("| 場所（Design の r） | r [mm] | 手・指 5.7 N の許容 [N·m] | 首 5.0 N の許容 [N·m] | 0.45 N·m の力 [N] | 0.7 N·m の力 [N] | 1.0 N·m の力 [N] |\n|---|---|---|---|---|---|---|")
    for name, (a, b) in R_RANGES.items():
        A(f"| {name} | {a:g}〜{b:g} | {allowed_nm(th['hand_finger'], a):.3f}〜{allowed_nm(th['hand_finger'], b):.3f} | "
          f"{allowed_nm(th['neck'], a):.3f}〜{allowed_nm(th['neck'], b):.3f} | {force_n(SOFT_CAP_NM, b):.1f}〜{force_n(SOFT_CAP_NM, a):.1f} | "
          f"{force_n(0.7, b):.1f}〜{force_n(0.7, a):.1f} | {force_n(1.0, b):.1f}〜{force_n(1.0, a):.1f} |")
    lo, hi = allowed_nm(th["hand_finger"], 44.4), allowed_nm(th["hand_finger"], 49.8)
    A(f"\n**J3〜J5**: r = 44.4〜49.8 mm（5〜95%）で、5.7 N の暫定値でも許容トルクは **{lo:.3f}〜{hi:.3f} N·m（約 0.25 N·m）**。"
      f"ソフトの上限 0.45 N·m では {force_n(SOFT_CAP_NM, 49.8):.1f}〜{force_n(SOFT_CAP_NM, 44.4):.1f} N で、5.7 N を超える。**0.45 N·m の上限との不一致 = OPEN**（DECISIONS.md 2026-09-30）。")
    a1, b1 = allowed_nm(th["hand_finger"], 5.0), allowed_nm(th["hand_finger"], 44.0)
    A(f"**J1 の下げる側**: r = 5〜44 mm で、許容トルクは **{a1:.3f}〜{b1:.3f} N·m**（0.45 N·m の {100 * a1 / SOFT_CAP_NM:.0f}〜{100 * b1 / SOFT_CAP_NM:.0f}%）。"
      "**力の制限だけでは成立しない**（r が小さい所ほど、小さいトルクで 5.7 N を超える）。**動作範囲の制限か、物理的なカバーが要る**。\n")

    A("## 2. J1 の暫定範囲 −5°〜+10° と、取り込み機構の頭下げの要否\n")
    A("**暫定 J1 範囲 −5°〜+10° を提案**（符号は Design の CAD: 負 = 頭を下げる。Design の数: 下げる側 −5° で 601 mm³、−10° で 1,562、+10° で 2,062 mm³ の届くすき間）。**取り込み機構の頭下げの要否の結果を見て、再判断する。**\n")
    A("取り込み機構の探索（`simulation/results/scoop_forms_2026-09-30.md`）からの、J1（頭のピッチ）への要求（**J1 軸から口までの距離は ASSUMED**: Design の頭の寸法を確認）:\n")
    A("| 距離 J1 → 口の前縁 [mm] | 口を上げてよい上限（1 円玉 1.5 mm を超えない）[°] | カップの昇降 14 mm を J1 で代用する頭下げ [°] |\n|---|---|---|")
    for lever in (60.0, 80.0, 100.0):
        A(f"| {lever:g} | +{hood_pitch_tolerance_deg(lever, 1.5):.1f} | −{cup_pitch_for_lift_deg(14.0, lever):.1f} |")
    A("\n- **受け身のフード（推奨 1）は頭下げを要らない。ただし頭のピッチは、作業中はほぼ 0° に保つ必要がある**（口の前縁が 1.5 mm 上がると 1 円玉が壁の下へ逃げる。上の表の +0.8°〜+1.3° 以内）。範囲 +10° まで上げると、フードの取り込みは成り立たない（取り込み中は 0° 付近に固定する運用が前提）。"
      "**下げる側は、前縁が床に押しつけられる（剛体では下げられない）ので、頭下げの範囲は取り込みには使わない**。")
    A("- **カップ（推奨 2、条件つき）の昇降を J1 で代用すると、−8°〜−13.5° の頭下げが要る**（暫定 −5° を超える）。カップは昇降 1 駆動を別に持つほうが、J1 の範囲を広げるより安全（J1 の下げる側は r = 5〜44 mm で、力の制限では守れない）。\n")

    A("## 3. 頭に足す質量: 中空前提での重心と J1 の静的トルク\n")
    A(f"入力: 頭 {HEAD_G:g} g、J1 静的 {J1_STATIC_NM} N·m（Design、ASSUMED）→ 重心は J1 から {com_lever_mm():.1f} mm。足す質量の位置（J1 からの距離）は ASSUMED（口の付近 40〜80 mm）。"
      "Design の数: 質量の追加は**中実で 55〜65 g、中空で 10 g 台**。\n")
    A("| 足す質量 | 足す位置 [mm] | 重心 J1 からの距離 [mm] | J1 の静的トルク [N·m] | ソフトの上限 0.45 N·m に対する割合 |\n|---|---|---|---|---|")
    rows = [("なし（現行）", 0.0, 0.0)]
    for label, m in (("中空 10 g", 10.0), ("中空 15 g", 15.0), ("中空 19 g", 19.0), ("中実 55 g", 55.0), ("中実 65 g", 65.0)):
        for d in (40.0, 60.0, 80.0):
            rows.append((label, m, d))
    for label, m, d in rows:
        lever, tau, frac = with_added_mass(m, d)
        A(f"| {label} | {'—' if m == 0 else f'{d:g}'} | {lever:.1f} | {tau:.3f} | {100 * frac:.0f}% |")
    l1, t1, f1 = with_added_mass(19.0, 80.0)
    l2, t2, f2 = with_added_mass(65.0, 80.0)
    A(f"\n読み: **中空 10〜19 g なら、J1 の静的トルクは最大でも {t1:.3f} N·m（ソフトの上限の {100 * f1:.0f}%）、重心は J1 から {l1:.1f} mm（現行 {com_lever_mm():.1f}）**。"
      f"**中実 55〜65 g だと最大 {t2:.3f} N·m（{100 * f2:.0f}%）、重心は {l2:.1f} mm まで前へ寄る**。どちらもソフトの上限（0.45 N·m）に対して余裕はあるが、"
      "重心が前へ寄るほど、頭を持ち上げる姿勢・人に引かれたとき・頭スキッドで体を支えるときの J1 負荷は増える（動的・引く力は未計算）。**中空前提を推奨**（重心のずれが小さい）。中空の強度・壁の厚みは Design の確認待ち。\n")
    (HERE / "j1_head.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    print("\n".join(L))


if __name__ == "__main__":
    main()
