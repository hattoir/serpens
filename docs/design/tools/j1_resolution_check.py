"""J1 の分解能・トルク上限・接触力の見積もり（ENTRY-D-0008）。HG-H1 の prior（assumptions.yaml）と STS3215 の分解能（docs/sts3215_registers.md）を使う。
HG-H1 は kp・バックラッシ・不感帯を持たない（UNKNOWN）ので、ここでも仮定しない。ACTUATOR_MODEL_SIM の prior に基づく概算で、実測ではない。
"""
import json, math
from pathlib import Path
STEP = 360.0 / 4096
J1 = (-181.8, 32.4)
LEV = {"口の前縁 x−236": 54.2, "カメラ x−232": 50.2, "横スキッドの前端 x−226": 44.2, "後ろのスキッド x−164": -17.8}
out = {"step_deg": STEP}
out["mm_per_step"] = {k: round(abs(v) * math.radians(STEP), 4) for k, v in LEV.items()}
out["window_steps"] = {"すき間の面 −2〜+1°": round(3.0 / STEP, 1), "取り込み（c 基準、c0 によらず）": round(math.degrees(math.asin(1.0 / 54.2)) / STEP, 1)}
out["quantization_mouth_mm"] = round(0.5 * math.radians(STEP) * 54.2, 3)
out["front_rear_height_diff_mm_half_step"] = round(60 * math.tan(math.radians(0.5 * STEP)), 3)
stall = (1.4, 3.0); ratio = 0.167; e = (-0.4, 0.4)
lim = lambda r, s, ee: r * s * (1 + ee)
out["torque_limit_default_range_nm"] = [round(lim(ratio, stall[0], e[0]), 3), round(lim(ratio, stall[1], e[1]), 3)]
out["torque_limit_default_nominal_nm"] = round(lim(ratio, 2.687, 0), 3)
F = lambda t, lever_mm: t / (lever_mm / 1000)
out["skid_front_force_default_n"] = {"nominal": round(F(0.449, 44.2), 1), "min": round(F(out["torque_limit_default_range_nm"][0], 44.2), 1), "max": round(F(out["torque_limit_default_range_nm"][1], 44.2), 1)}
r_work = 0.022 / 2.687
out["work_ratio_register_permille_nominal"] = round(1000 * r_work, 1)
wl = [lim(r_work, s, ee) for s in stall for ee in e]
out["work_limit_range_nm"] = [round(min(wl), 4), round(max(wl), 4)]
out["skid_front_force_work_n"] = [round(F(min(wl), 44.2), 2), round(F(max(wl), 44.2), 2)]
out["backdrive_prior_nm"] = [0.05, 0.50]
out["backdrive_min_contact_force_n"] = round(F(0.05, 44.2), 2)
out["head_gravity_moment_nm"] = [0.024, 0.035]
Path(__file__).resolve().parents[1].joinpath("results", "j1_resolution_check_2026-09-30.json").write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
print(json.dumps(out, ensure_ascii=False, indent=1))
