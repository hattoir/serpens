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


if __name__ == "__main__":
    for k, v in main().items():
        print(k, v)
