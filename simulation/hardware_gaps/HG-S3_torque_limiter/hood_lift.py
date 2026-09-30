"""フード昇降（cloche drop）の落とす力・速さ・縁の圧力の確認の枠組み（flank_v.py と同じ形）。**PROVISIONAL / SAFETY_UNVERIFIED。**

暫定しきい値（`safety_thresholds.yaml`）: 手・指 5.7 N（その半分 2.8 N を上限にする）、圧力 8.2 N/cm²（子どもの痛み閾値 mean−1SD × 1 cm²。二次情報）。
**5.7 N・8.2 N/cm² は暫定で、安全の確定として扱わない。** 入力の値（フードの質量・クランクの半径・サーボの慣性・接触の剛性・縁の幅）は **ASSUMED**（Design の概念 `docs/design/hood_lift_2026-09-30.md`、HG-H1 の prior）。

見るもの:
  (1) 静的な力: 下げる側 = フードの自重（Design 案: 下向きにサーボの力を掛けない）/ 上げる側 = サーボ（クランク）の力 = τ / r → 2.8 N 以下に制限できるか
  (2) 衝撃: 落ちる速さ v で、指に当たったときの力の見積もり F = v √(m_eq k)（接触剛性 k）。フードの自重で降りる（m_eq = フード + リンク）と、サーボが降ろす（m_eq = J / r²）で比べる
  (3) 縁の圧力: P = F / (縁の幅 w × 指の接触長 L)。許容 F = min(2.8 N, 8.2 N/cm² × w × L)
  (4) 隙間の帯（User の目安: 5 mm 未満か 25 mm 以上）: 上げたときの下のすき間 s が 5 mm 未満か

    python simulation/hardware_gaps/HG-S3_torque_limiter/hood_lift.py
"""
from __future__ import annotations

import math

G = 9.81
F_LIMIT_N = 2.8                # 暫定しきい値 5.7 N の半分
P_LIMIT_N_CM2 = 8.2            # 暫定（1 cm² の圧力計の値。縁・角の小さな接触面積では 力 = 圧力 × 面積 の方が先に効く）
HOOD_KG = 0.0114 + 0.0116      # フード PLA 11.4 g + リンク・ピン・ばね（Design の約 3 g に、揺れる質量の余裕を足した仮置き）。ASSUMED
K_CONTACT_N_M = (3.0e3, 3.0e4)  # 指 + TPU 外皮の接触剛性 3〜30 N/mm（HG-H1 の prior、ASSUMED）
J_REFLECTED = (1.0e-3, 1.0e-2)  # 1:191 サーボの反射慣性 kg·m²（HG-H1 の prior。UNKNOWN。SG90 級は不明）


def weight_n(mass_kg: float = 0.0114) -> float:
    return mass_kg * G


def impact_gravity_descent(v_m_s: float, k_n_m: float, m_eq_kg: float = HOOD_KG) -> float:
    """フードが自重で降り（サーボは速さだけ抑える）、指に当たったときの力の見積もり。F = v √(m k)。"""
    return v_m_s * math.sqrt(m_eq_kg * k_n_m)


def impact_driven_descent(v_m_s: float, lever_m: float, j_kg_m2: float, k_n_m: float) -> float:
    """サーボが降ろす（反射慣性 J が指に当たる）ときの力の見積もり。F = ω √(J k)、ω = v / r（HG-H1 の式と同じ）。"""
    return (v_m_s / lever_m) * math.sqrt(j_kg_m2 * k_n_m)


def allowed_force_n(width_mm: float, contact_len_mm: float) -> float:
    """縁の幅 w と指の接触長 L での許容力 = min(2.8 N, 8.2 N/cm² × w × L)。"""
    return min(F_LIMIT_N, P_LIMIT_N_CM2 * (width_mm / 10.0) * (contact_len_mm / 10.0))


def pressure_n_cm2(force_n: float, width_mm: float, contact_len_mm: float) -> float:
    return force_n / ((width_mm / 10.0) * (contact_len_mm / 10.0))


def up_stroke_torque_limit_nm(lever_mm: float, f_limit_n: float = F_LIMIT_N) -> float:
    """上げる側の力を f_limit 以下にするための、クランク軸の許容トルク。"""
    return f_limit_n * lever_mm * 1e-3


def gap_band_ok(stroke_mm: float) -> bool:
    """上げたときの下のすき間 s が 5 mm 未満か（降りる途中は s → c で、s < 5 なら全域が 5 mm 未満）。"""
    return stroke_mm < 5.0


def main() -> dict:
    out: dict = {"weight_n": weight_n()}
    rows = []
    for v_mm in (5.0, 10.0, 20.0):
        v = v_mm * 1e-3
        g_lo, g_hi = (impact_gravity_descent(v, k) for k in K_CONTACT_N_M)
        for r_mm in (2.5, 5.0, 10.0):
            d_lo, d_hi = impact_driven_descent(v, r_mm * 1e-3, J_REFLECTED[0], K_CONTACT_N_M[0]), impact_driven_descent(v, r_mm * 1e-3, J_REFLECTED[1], K_CONTACT_N_M[1])
            rows.append({"v_mm_s": v_mm, "lever_mm": r_mm, "gravity_descent_n": (g_lo, g_hi), "driven_descent_n": (d_lo, d_hi)})
    out["impact"] = rows
    out["edge_allowed_n"] = {f"w={w} mm, L={L} mm": allowed_force_n(w, L) for w in (0.4, 0.8, 1.6, 3.6) for L in (10, 30)}
    out["up_torque_limit_nm"] = {r: up_stroke_torque_limit_nm(r) for r in (2.5, 4.0, 5.0)}
    out["gap_ok"] = {s: gap_band_ok(s) for s in (3, 4, 5, 8, 10)}
    return out


if __name__ == "__main__":
    o = main()
    print(f"自重 {o['weight_n']:.3f} N（フード 11.4 g）")
    print("衝撃の見積もり [N]（自重で降りる: k 3〜30 N/mm / サーボが降ろす: J 1e-3〜1e-2 kg·m²）")
    for r in o["impact"]:
        print(f"  v={r['v_mm_s']:>4g} mm/s r={r['lever_mm']:>4g} mm  自重降下 {r['gravity_descent_n'][0]:.2f}〜{r['gravity_descent_n'][1]:.2f}   サーボ降下 {r['driven_descent_n'][0]:.1f}〜{r['driven_descent_n'][1]:.1f}")
    print("縁の許容力 [N]:", {k: round(v, 2) for k, v in o["edge_allowed_n"].items()})
    print("上げる側のクランクの許容トルク [N·m]:", {k: round(v, 4) for k, v in o["up_torque_limit_nm"].items()})
    print("隙間の帯（s < 5 mm）:", o["gap_ok"])
