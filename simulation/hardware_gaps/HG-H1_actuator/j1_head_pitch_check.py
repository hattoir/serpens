"""Design（ENTRY-D-0008 (2)）の J1（頭ピッチ）の見積もりを、HG-H1 の prior で確かめる（ACTUATOR_MODEL_SIM。**prior は実測ではない**）。

確かめる点:
  (a) 受動の柔らかさ: バックドライブ（脱力時に外から回すのに要るトルク）> 頭の重力モーメント なら、頭は自重で降りない
  (b) 作業中の上限 ≲ 0.022 N·m の実効値と、横スキッド前端（腕 44.2 mm）を押す力。バックドライブと比べる
  (c) 既定の上限 0.449 N·m で、押し込むと口の前縁が床に触れる角度（前縁は横スキッド前端の 10 mm 前）
  (d) 【Design が触れていない点】上限がバックドライブ以下なら、**位置指令でも J1 を動かせない可能性**（動き出しのトルクが不明）→ 較正用の上限 τ_move が要る

    python simulation/hardware_gaps/HG-H1_actuator/j1_head_pitch_check.py
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).parent
A = yaml.safe_load(open(HERE / "assumptions.yaml", encoding="utf-8"))
SK_ARM_M = 0.0442             # J1 → 横スキッドの前端（Design、ENTRY-D-0008）
EDGE_ARM_M = 0.0542           # J1 → 口の前縁
STEP_DEG = 360.0 / 4096.0
N = 200_000


def sample(n=N, seed=1):
    r = np.random.default_rng(seed)
    s = A["servo"]
    stall = r.uniform(*s["stall_torque_nm_at_7v4"], n)
    bd = r.uniform(*s["backdrive_torque_nm"], n)
    e = r.choice(A["control"]["cap_model_error"], n)
    return stall, bd, e


def main() -> dict:
    stall, bd, e = sample()
    out: dict = {}
    # (a) 受動の柔らかさ。重力モーメント: 頭だけ（E-0006: 0.024〜0.035）/ 首を持ち上げた姿勢（訂正後 0.216〜0.238、中実 0.269〜0.319）
    grav = {"頭だけ 0.024": 0.024, "頭だけ 0.035": 0.035, "首を上げた姿勢（中空）0.216": 0.216, "首を上げた姿勢（中空）0.238": 0.238, "首を上げた姿勢（中実）0.319": 0.319}
    out["p_backdrive_lt_gravity"] = {k: float(np.mean(bd < g)) for k, g in grav.items()}
    # (b) 作業中の上限
    reg = round(0.022 / 2.687 * 1000)                       # 8（1000 = ストールの 100%）
    cap = reg / 1000 * stall * (1 + e)
    out["register"] = reg
    out["cap_nominal_nm"] = reg / 1000 * 2.687
    out["cap_range_nm"] = (float(cap.min()), float(cap.max()))
    out["cap_p05_p95"] = (float(np.percentile(cap, 5)), float(np.percentile(cap, 95)))
    out["force_range_n"] = tuple(c / SK_ARM_M for c in out["cap_range_nm"])
    out["backdrive_min_force_n"] = A["servo"]["backdrive_torque_nm"][0] / SK_ARM_M
    out["p_cap_gt_backdrive"] = float(np.mean(cap > bd))
    # 既定の上限
    out["default_cap_nm"] = 0.449
    out["default_force_n"] = 0.449 / SK_ARM_M
    out["p_default_cap_gt_backdrive"] = float(np.mean(0.167 * stall * (1 + e) > bd))
    # (c) 前縁が床に触れる角度（横スキッド前端で回る。前縁はその 10 mm 前）
    for c0 in (0.1, 0.3, 1.0):
        out[f"edge_touch_deg_c{c0}"] = math.degrees(c0 * 1e-3 / (EDGE_ARM_M - SK_ARM_M))
        out[f"edge_touch_steps_c{c0}"] = out[f"edge_touch_deg_c{c0}"] / STEP_DEG
    return out


def stopper_loads() -> dict:
    """機械ストッパー（Design: 首の芯の半径方向のピン φ3、r 44〜48.5、フードの窓の端に当たる。`docs/design/j1_cover_stopper_2026-09-30.md`）にかかる荷重の見積もり [N]。
    ACTUATOR_MODEL_SIM。**すべて prior・仮定（ASSUMED）**。5.7 N・0.25 N·m とは無関係の、機械の強度の見積もり。
      静的: 上限トルク ÷ 腕（腕 = ピンの半径 44〜48.5 mm。Design の記述 36〜47 mm も含めて 36〜48.5 mm で見る）
      衝撃: F = ω √(J k)（HG-H1 と同じ式。**トルク上限では抑えられない**）。k = ストッパーの剛性（PLA 同士 100 N/mm / TPU の緩衝 5 N/mm、ASSUMED）
      ピンの曲げ: 片持ち梁、長さ L = 4.5 mm、直径 3 mm。許容応力 PLA 35 MPa（FDM の積層方向でばらつく。20〜50、ASSUMED）/ 鋼 250 MPa（ASSUMED）
    """
    stall = A["servo"]["stall_torque_nm_at_7v4"]
    e = A["control"]["cap_model_error"]
    arm = (0.036, 0.0485)
    cap_lo, cap_hi = 0.167 * stall[0] * (1 + min(e)), 0.167 * stall[1] * (1 + max(e))
    out = {"arm_m": arm}
    out["static_default_cap_nominal_n"] = (0.449 / arm[1], 0.449 / arm[0])
    out["static_default_cap_prior_n"] = (cap_lo / arm[1], cap_hi / arm[0])
    w_lo, w_hi = 8 / 1000 * stall[0] * (1 + min(e)), 8 / 1000 * stall[1] * (1 + max(e))
    out["static_working_cap_prior_n"] = (w_lo / arm[1], w_hi / arm[0])
    out["static_limiter_ineffective_n"] = (stall[0] / arm[1], stall[1] / arm[0])        # レジスタが効かない（解釈違い・設定漏れ）= ストール
    j = A["servo"]["reflected_inertia_kgm2"]
    imp = {}
    for label, dps in (("120 deg/s（robot.yaml の J7 の最高）", 120.0), ("30 deg/s", 30.0)):
        w = math.radians(dps)
        for kname, k in (("PLA 100 N/mm", 1.0e5), ("TPU 緩衝 5 N/mm", 5.0e3)):
            imp[f"{label} / {kname}"] = (w * math.sqrt(j[0] * k), w * math.sqrt(j[1] * k))
    out["impact_n"] = imp
    d, L = 3.0, 4.5
    S = math.pi * d ** 3 / 32.0                          # 断面係数 mm³
    out["pin_capacity_n"] = {"PLA 35 MPa（20〜50）": 35.0 * S / L, "PLA 20 MPa": 20.0 * S / L, "鋼 250 MPa": 250.0 * S / L}
    return out


def head_cog_y_effects(y_mm: float, m_kg: float = 0.097, half_span_mm: float = 40.0, k_floor_n_mm: float = 1.0) -> dict:
    """頭の重心の左右（y）の偏りの影響（Design の頭 E3 v2: SG90 が +y 側のみで約 +2.8 mm）。**すべて仮定（ASSUMED）。実測ではない。**
      - J1（ピッチ軸）: 重心の y の偏りは、ピッチの静的トルク（x–z 面）に効かない。左右の傾き（ロール）のモーメント m g y が軸にかかるだけ（軸受の径方向の荷重）。
      - 横スキッド（|y| 36〜44 の中心 ±40 mm）の荷重の左右差: ΔN = m g y / (2 × 半スパン)（横スキッドだけで支える上限。後ろのスキッドが分担すれば小さい）。
      - 床（スキッドの押し込み剛性 k_floor [N/mm]）のたわみの左右差 = ΔN / k_floor。壁の下端の左右の高さの差になり、c₀ = 0.1 mm の半分（0.05 mm）以下に収める。
    """
    g = 9.81
    weight = m_kg * g
    d_n = weight * (y_mm * 1e-3) / (2.0 * half_span_mm * 1e-3)
    return {"roll_moment_mn_m": weight * y_mm, "skid_asymmetry_ratio": y_mm / half_span_mm, "delta_n": d_n, "delta_deflection_mm": d_n / k_floor_n_mm}


def head_cog_y_limit_mm(k_floor_n_mm: float, m_kg: float = 0.097, half_span_mm: float = 40.0, max_delta_mm: float = 0.05) -> float:
    """左右のたわみの差を max_delta（既定 0.05 mm = c₀ の半分）以下にできる重心 y の偏りの上限 [mm]。"""
    weight = m_kg * 9.81
    return max_delta_mm * k_floor_n_mm * 2.0 * half_span_mm / weight


if __name__ == "__main__":
    for k, v in main().items():
        print(k, v)
