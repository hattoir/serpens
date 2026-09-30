r"""脇の V（胴の脇の直線と同心ナックルの接線にできるくさび）を、閉じる力から見直す。**PROVISIONAL / SAFETY_UNVERIFIED。**

    .\.venv\Scripts\python.exe simulation\hardware_gaps\HG-S3_torque_limiter\flank_v.py

背景（Design の ENTRY-D-0003 (2)(3)、`docs/design/gap_check_report_2026-09-30.md` §5、OPEN-SERPENS-DESIGN-014）:
  曲げた姿勢では、胴の脇（半径 約 46 mm）に V 字のくさびができ、幅 5〜12.5 mm を通る。形では消せない。
  Design の提案 (b) 「閉じる力を暫定しきい値以下にする」: 5.7 N × 0.046 m = 0.26 N·m。
User の判断（2026-09-30）:
  1. **V は「柔らかいカバーで塞ぐ」を主とし、トルク上限は補助として残す。**
  2. 0.26 N·m は、いちばん外側の点（力が最小）の半径 46 mm で割っている。**幅 5〜12 mm の位置の r（Design が出す）で F = τ / r を再計算する。**
     r が 46 mm より小さければ、許容トルクはさらに下がる。
  3. **5.7 N は暫定値のまま。実測・規格確認なしで、安全の確定として扱わない。**
  4. リミッターの窓 0.7〜1.0 N·m との食い違いを DECISIONS.md に OPEN で記録する。

このスクリプトがやること:
  A. 接触点の半径 r [mm] を引数として、許容トルク τ = F しきい値 × r と、各トルクでの力 F = τ / r を表にする（r は Design が出すまで仮の掃引）
  B. Design の V の幾何（幅 w = 4 mm + s · tan φ、s = 接線点から脇に沿った距離）から、幅 5〜12.5 mm の位置の s と、そこでの力を見積もる。
     見積もりの前提（**Design に確認が要る**）: 脇の直線が、関節軸を中心とする半径 46 mm の円に接線点で接する。
       - 軸から接触点までの距離（動径）r(s) = √(46² + s²)（46 mm 以上）
       - **閉じるトルクが法線力に伝わる腕の長さは、動径 r ではなく s**（接線点では脇の面の法線が軸を通るので、閉じる動きが法線方向に成分を持たない。
         接線点から s だけ離れると、法線力の作用線が軸から s だけずれ、トルク = 法線力 × s）。したがって法線力 = τ / s で、s ≪ r のとき τ / r より大きい。
       - 摩擦なし、剛体、指は V の中で 2 面に挟まれる、と仮定。指の変形・外装の柔らかさは入れていない
  どれも暫定しきい値（5.7 N は子どもの痛み閾値で成人値を縮めた値。実測・規格本文の確認なし）に対する比較で、**安全の確定ではない**。
"""
from __future__ import annotations

import csv
import math
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
GAPS = HERE.parent
R_FLANK_MM = 46.0                  # 胴の脇の半径（Design の報告。接線円の半径 = 脇の直線の軸からの距離）
W0_MM = 4.0                        # 接線点での幅（同心ナックルの一定のすき間。Design）
W_BAND_MM = (5.0, 12.5)            # 人が触れる範囲で作ってはいけない幅（Design の報告 §5。EN 71 の 5〜12 mm 帯）
PHIS_DEG = (9.0, 10.0, 25.0, 43.0, 52.0)          # 開き角 φ（Design: 全範囲 9°〜52°、10° と 25° と 43° は報告の例）
R_SWEEP_MM = (5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 46.0, 50.0)
TAUS_NM = (0.26, 0.45, 0.70, 1.00)   # 0.26 = Design の算術 / 0.45 = ソフトの上限 / 0.7〜1.0 = リミッターの窓


def thresholds() -> dict[str, float]:
    y = yaml.safe_load((GAPS / "safety_thresholds.yaml").read_text(encoding="utf-8"))
    q = y["quasi_static_contact_force_n"]
    return {k: float(q[k]["baseline"]) for k in ("hand_finger", "neck", "chest", "face")}


def allowed_torque_nm(force_n: float, arm_mm: float) -> float:
    """しきい値 F を超えないトルク τ = F × 腕の長さ。"""
    return force_n * arm_mm / 1000.0


def force_n(torque_nm: float, arm_mm: float) -> float:
    """トルク τ が腕の長さ arm の点で出す力 F = τ / arm。"""
    return torque_nm / (arm_mm / 1000.0)


def wedge_s_range_mm(phi_deg: float) -> tuple[float, float]:
    """幅 w = W0 + s tan φ が W_BAND を通る s の範囲 [mm]（接線点からの距離）。"""
    t = math.tan(math.radians(phi_deg))
    return (W_BAND_MM[0] - W0_MM) / t, (W_BAND_MM[1] - W0_MM) / t


def radial_mm(s_mm: float) -> float:
    """軸から接触点までの距離（脇の直線が半径 46 の円に接線点で接するとき）。"""
    return math.hypot(R_FLANK_MM, s_mm)


def pressure_limit_n_cm2() -> float:
    y = yaml.safe_load((GAPS / "safety_thresholds.yaml").read_text(encoding="utf-8"))
    return float(y["pressure_limit_n_per_cm2"]["baseline"])


def edge_pressure_n_cm2(force_n_: float, edge_t_mm: float, width_mm: float = 30.0) -> float:
    """受け身の垂れ布の下端（薄い板の縁）が指に当たるときの圧力 = 力 / (縁の厚み × 幅)。平らな縁が指に線で当たる最悪の見積もり。"""
    area_cm2 = (edge_t_mm / 10.0) * (width_mm / 10.0)
    return force_n_ / area_cm2


def allowed_curtain_force_n(edge_t_mm: float, width_mm: float = 30.0) -> float:
    """暫定の圧力限界（8.2 N/cm²）を超えない、縁が指に掛けてよい力。"""
    return pressure_limit_n_cm2() * (edge_t_mm / 10.0) * (width_mm / 10.0)


def main() -> None:
    th = thresholds()
    L: list[str] = []
    A = L.append
    A("# 脇の V: 閉じる力の見直し（Engineering、2026-09-30）\n")
    A("**PROVISIONAL / SAFETY_UNVERIFIED。** 力のしきい値（手・指 5.7 N 等）は、子どもの痛み閾値で成人の規格値を縮めた暫定値で、"
      "実測も規格本文の確認もしていない（`safety_thresholds.yaml`、OQ-0115）。**この表は安全の確定ではない。**"
      "再現: `simulation/hardware_gaps/HG-S3_torque_limiter/flank_v.py`。\n")
    A("## 判断（User 2026-09-30）\n")
    A("1. **脇の V は「柔らかいカバーで塞ぐ」を主とする**（Design の FLANK SKIRT STUDY）。トルク上限は**補助**として残す。")
    A("2. 0.26 N·m = 5.7 N × 0.046 m は、最も外側の点（力が最小）の半径 46 mm で割った値。**幅 5〜12 mm の位置の r（Design が出す）で F = τ / r を再計算する。r が 46 mm より小さければ、許容トルクはさらに下がる。**")
    A("3. 5.7 N は暫定値のまま。実測・規格確認なしで、安全の確定として扱わない。")
    A("4. リミッターの窓（0.7〜1.0 N·m）との食い違いは、DECISIONS.md に OPEN で記録した。\n")
    A("**Design の r は未着**（ENTRY-E-0005 で依頼）。それまでは、r を仮に掃引した表（A）と、Design の V の幾何の式から見積もった表（B、Design の確認が要る前提つき）で見る。\n")

    A("## A. 接触点の半径 r [mm] を掃引した、許容トルクと、各トルクでの力\n")
    A("許容トルク τ = しきい値 × r [N·m]（そのしきい値を超えない上限）。r が小さいほど下がる。\n")
    A("| r [mm] | 手・指・胸 5.7 N | 首 5.0 N | 顔 2.7 N |\n|---|---|---|---|")
    for r in R_SWEEP_MM:
        mark = "（Design の 0.26 の前提）" if r == 46.0 else ""
        A(f"| {r:g}{mark} | {allowed_torque_nm(th['hand_finger'], r):.3f} | {allowed_torque_nm(th['neck'], r):.3f} | {allowed_torque_nm(th['face'], r):.3f} |")
    A("\n各トルクで出る力 F = τ / r [N]（手・指のしきい値 5.7 N を超える所は ×）:\n")
    A("| r [mm] | " + " | ".join(f"{t:g} N·m" for t in TAUS_NM) + " |\n|---|" + "---|" * len(TAUS_NM))
    for r in R_SWEEP_MM:
        cells = []
        for t in TAUS_NM:
            f = force_n(t, r)
            cells.append(f"{f:.1f}" + ("" if f <= th["hand_finger"] else " ×"))
        A(f"| {r:g} | " + " | ".join(cells) + " |")
    A("\n読み: どの r でも、リミッターの窓 0.7〜1.0 N·m と、ソフトの上限 0.45 N·m は、手・指の 5.7 N を超える（r = 46 mm でも 0.7 N·m で 15 N、1.0 N·m で 22 N）。"
      "0.26 N·m でも r < 46 mm では超える。**トルクの制限だけでは V を守れない**（HG-S3 の結論と同じ）。\n")

    A("## B. Design の V の幾何（w = 4 mm + s · tan φ）から見積もる、幅 5〜12.5 mm の位置\n")
    A("**前提（Design に確認が要る）**: 脇の直線が、関節軸を中心とする半径 46 mm の円に接線点で接する。s = 接線点から脇に沿った距離。"
      "動径 r(s) = √(46² + s²)（46 mm 以上）。**閉じるトルクが法線力に伝わる腕の長さは s**（接線点では脇の面の法線が軸を通り、閉じる動きが法線方向に成分を持たない）。摩擦なし・剛体・指は 2 面に挟まれる。\n")
    A("| φ [°] | 幅 5〜12.5 mm の s [mm] | 動径 r(s) [mm] | 許容トルク: r(s) の最小で（Design の 0.26 と同じ考え方）[N·m] | 許容トルク: 腕 = s の最小で [N·m] | 0.7 N·m の力（腕 = s の最小）[N] | 1.0 N·m の力（同）[N] |\n|---|---|---|---|---|---|---|")
    rows = []
    for phi in PHIS_DEG:
        s0, s1 = wedge_s_range_mm(phi)
        r0, r1 = radial_mm(s0), radial_mm(s1)
        a_r = allowed_torque_nm(th["hand_finger"], r0)
        a_s = allowed_torque_nm(th["hand_finger"], s0)
        f07, f10 = force_n(0.7, s0), force_n(1.0, s0)
        A(f"| {phi:g} | {s0:.1f}〜{s1:.1f} | {r0:.1f}〜{r1:.1f} | {a_r:.3f} | {a_s:.4f} | {f07:.0f} | {f10:.0f} |")
        rows.append((phi, s0, s1, r0, r1, a_r, a_s, f07, f10))
    a_s_min, a_s_max = min(r[6] for r in rows), max(r[6] for r in rows)
    f07_min, f07_max = min(r[7] for r in rows), max(r[7] for r in rows)
    f10_min, f10_max = min(r[8] for r in rows), max(r[8] for r in rows)
    r_lo, r_hi = min(r[3] for r in rows), max(r[4] for r in rows)
    A("\n読み（見積もり。Design の確認が要る）:")
    A(f"- **動径 r で割る考え方（Design の 0.26）**: 脇の直線に沿った V（幅 5〜12.5 mm）の動径は {r_lo:.0f}〜{r_hi:.0f} mm と **46 mm 以上**で、"
      "この幾何では 46 mm は最も外側ではなく最も内側（接線点）。したがって 0.26 N·m は、この V については動径で見た下限になる。"
      "**User の指摘（r が 46 mm より小さければ許容トルクはさらに下がる）が当たるのは、軸に近い接触点**: 曲げの内側の面取りとの V、あごと首の前のくさび、"
      "そのほか Design が出す位置。その r が未着なので、**答えは表 A の r < 46 mm の行**（r = 30 mm で 0.17、20 mm で 0.11、10 mm で 0.057 N·m）。")
    A(f"- **法線力に効く腕は動径ではなく s（数 mm）**（上の前提）。s の最小（幅が 5 mm になる位置）は φ = 25° で 2.1 mm、43° で 1.1 mm、9° で 6.3 mm。"
      f"このとき 5.7 N を超えない許容トルクは **{a_s_min:.4f}〜{a_s_max:.3f} N·m**（0.26 N·m の 1/{0.262 / a_s_max:.0f}〜1/{0.262 / a_s_min:.0f}。歩容が要る 0.13〜0.69 N·m を下回る）。"
      f"0.7 N·m で {f07_min:.0f}〜{f07_max:.0f} N、1.0 N·m で {f10_min:.0f}〜{f10_max:.0f} N（**剛体・摩擦なしの理想化の上限に近い見積もり**。指と外装の変形で下がるが、下がる量は未実測）。")
    A("- したがって、**閉じる力（トルク上限）だけで V を守る案（b）は成り立たない**（許容トルクが歩容の必要トルクを下回る）。V は**カバーで塞ぐ**のを主とし、トルク上限は補助（ソフトの故障時に力を下げる・歯車の保護）、という判断と整合する。")
    A("- 摩擦・指の変形・外装の柔らかさ・2 面の相対運動は入れていない。**実測（挟み込みの力）が要る。5.7 N は暫定値のまま。**\n")

    A("## C. Design への依頼（ENTRY-E-0005）\n")
    A("- 関節 J2〜J5 それぞれ・曲げの角度（9°〜52°）ごとに、**幅 5〜12.5 mm を通る位置の、関節軸からの距離 r と、脇の面の接線点からの距離 s**（動径と腕の両方）。")
    A("- V を作る 2 つの面のうち、どちらが動く面か（脇の直線 / 同心ナックルの円）。")
    A("- 柔らかいカバー（FLANK SKIRT STUDY）の中の挟み込み（蛇腹の内側）の隙間と、カバーの押し込みの力。\n")
    # ---- E. 受け身の垂れ布のゲートの挟み込みの目安 ----
    thr, plim = th["hand_finger"], pressure_limit_n_cm2()
    A("## E. 受け身の TPU 垂れ布のゲートの挟み込みの目安（取り込み機構。`simulation/results/scoop_forms_2026-09-30.md` §8・§12.2）\n")
    A(f"**PROVISIONAL / SAFETY_UNVERIFIED。** 垂れ布は駆動なし。力は**ばねの閉じる力 F**で頭打ちになる（F = 0.01〜1 N。**ASSUMED**。シミュレーションでは 0.1〜1 N では入らず、成立するのは 3 mN 以下）。"
      f"暫定しきい値は手・指 {thr} N、圧力 {plim} N/cm²（PROVISIONAL）。TPU の薄板（Design の試験片 B は 0.4〜0.6 mm）の**縁の厚み**が、指に線で当たる面積（縁の厚み × 幅 30 mm）を決める。\n")
    A("| 縁の厚み [mm] | 圧力限界を超えない力 [N] | F = 0.01 N | 0.03 N | 0.1 N | 0.3 N | 1 N |\n|---|---|---|---|---|---|---|")
    for t in (0.05, 0.2, 0.4, 0.6):
        cells = []
        for F in (0.01, 0.03, 0.1, 0.3, 1.0):
            pr = edge_pressure_n_cm2(F, t)
            cells.append(f"{pr:.1f}" + ("" if pr <= plim else " ×"))
        A(f"| {t:g} | {allowed_curtain_force_n(t):.2f} | " + " | ".join(cells) + " |")
    A(f"\n（表の値は圧力 [N/cm²]。{plim} を超えるものは ×。）\n")
    A(f"- **力**: F ≤ 1 N は、手・指の {thr} N の 1/5.7 以下（余裕 4.7 N 以上）。垂れ布の力だけで暫定しきい値を超えることはない。")
    A(f"- **圧力**: 縁が薄いと、力が小さくても圧力が先に暫定の限界を超える。**縁 0.4 mm（試験片 B）で 1 N が限界（{allowed_curtain_force_n(0.4):.2f} N）**、縁 0.05 mm の極薄では 0.12 N まで。"
      "シミュレーションで成立する F ≤ 3 mN なら、どの厚みでも圧力は限界の 1/40 以下（縁 0.05 mm でも 0.2 N/cm²）。**縁は丸める（半径 0.5 mm 以上）と、面積が増えて圧力が下がる**（未検証）。")
    A("- **隙間の帯（User の目安: 5 mm 未満 または 25 mm 以上）**: 閉じた位置の垂れ布と床のすき間は 0.1〜1 mm、壁との隙間は 0（幅を壁から壁までとる）で、**5 mm 未満を満たす**。"
      "開く途中で、垂れ布と屋根・壁のあいだにできるくさび（垂れ布の長さ 15 mm、開き 0°〜90°）は、垂れ布の先端と屋根のあいだが 15 → 0 mm で、**5〜25 mm の帯を通る（開き 0°〜70° の間）**。これは避けられないので、**力（≤ 1 N）と圧力（縁の厚み）で守る**。短い垂れ布（8 mm）でも、5 mm 以上のくさびは通る（幾何は Design の確認待ち）。")
    A("- **限界**: 垂れ布は TPU で伸び・たわむ。指が垂れ布の裏に入ったときの力・圧力の実測が要る。**5.7 N・8.2 N/cm² は暫定値で、安全の確定として扱わない**。\n")
    (HERE / "flank_v.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    (HERE / "results").mkdir(exist_ok=True)
    with open(HERE / "results" / "flank_v_wedge.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["phi_deg", "s_min_mm", "s_max_mm", "r_min_mm", "r_max_mm", "allowed_nm_by_r", "allowed_nm_by_s", "force_n_at_0.7nm_by_s", "force_n_at_1.0nm_by_s"])
        w.writerows([[f"{x:.6g}" for x in r] for r in rows])
    print("\n".join(L))


if __name__ == "__main__":
    main()
