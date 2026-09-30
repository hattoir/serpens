"""J1（頭ピッチ = Engineering の J7）の動作範囲（ストッパー）と、ストッパーの荷重の最適化。**ACTUATOR_MODEL_SIM + MUJOCO_SIM + CAD_CONCEPT の組み合わせ。実機で未確認。**

符号は Engineering（θ_E: + = 頭を上げる）。Design の CAD の符号は + = 下げ（θ_E = −θ_CAD）。

入力（出典）:
  - すき間（挟み込み）の危険体積 V(θ) [mm³]: **Design の見積もり（CAD_CONCEPT。外形 STL・0.5 mm ボクセル。Engineering 未検証）**。`docs/design/results/gap_pitch_*.json`、`j1_cover_eval_2026-09-30.json`
    （計算済みの点だけ。**θ_E = +4, +6〜+9, −4, −7.5 の間は対数補間 = 計算していない**。`interpolated` と印を付ける）
  - 取り込み割合 I(c): **MUJOCO_SIM**（`agent/engineering-scoop` の `tools/scoop_forms_sweep.py intake_c`。フード + ゲート、頭 10 mm/s、位置ずれ 0・5 mm、床 = 平ら 2 種 + 凹凸 ±0.5 mm（絨毯の代用）、摩擦の prior の幅 0.7〜1.4 倍）
    θ → すき間 c の写像は **Design の幾何（口の前縁 +0.95 mm/°、前縁が床に触れる θ = −0.106°、押し込んだときの奥の壁の下 +1.17 mm/°）**（CAD_CONCEPT、ASSUMED）
  - 使えなくなる姿勢: `config/robot.yaml` の J7 の目標（home / rest_arc / coil = 8°、人を見る 55〜65°、フル鎌首 85〜90°、呼吸 ±5°）
  - ストッパーの荷重: ω√(J k)（HG-H1 と同じ式）+ 静的（上限 ÷ 腕）。J・k・強度は **prior（仮定）**
5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）で、ここでは使わない（機械の強度の見積もり）。**安全の確定ではない。**

    python simulation/hardware_gaps/HG-H1_actuator/j1_range_opt.py
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).parent
ROOT = HERE.parents[2]
RES = ROOT / "simulation" / "results"
A = yaml.safe_load(open(HERE / "assumptions.yaml", encoding="utf-8"))
CFG = yaml.safe_load(open(ROOT / "config" / "robot.yaml", encoding="utf-8"))

STEP_DEG = 360.0 / 4096.0
WINDOW = (-2.0, 1.0)                 # Floor Watch の作業窓（Design の提案。すき間 ≤ 40 mm³）
MM_PER_DEG_FRONT = 0.95              # 口の前縁 [mm/°]（J1 軸から 54.2 mm）
THETA_TOUCH = -0.106                 # c = 0.1 で前縁が床に触れる θ [°]
REAR_MM_PER_DEG = 1.17               # 前縁が床に触れたあと、奥の壁の下が上がる [mm/°]（Design: −1° で 1.15 mm）
C_NOMINAL = 0.1                      # 壁の下端のすき間の設計値 [mm]

# --- 危険体積 V(θ_E)（Design。CAD の符号 θ_CAD = −θ_E から読み替え） ---
V_COMPUTED = {   # θ_E: (覆いなし, 上の覆いあり)。上の覆い = Design の STL（頭を下げる側のすき間を減らす）。上げ側の覆いは無い
    -22.0: (4307, 4307), -10.0: (2062, 234), -7.5: (1350, None), -5.0: (401, 29), -3.0: (105, 6), -2.0: (40, 6), -1.0: (3, 3),
    0.0: (2, 2), 1.0: (10, 10), 2.0: (10, 10), 3.0: (44, 44), 5.0: (601, 601), 10.0: (1562, 1562), 22.0: (1228, 1228), 45.0: (856, 856),
}


def hazard(theta: float, cover: bool) -> tuple[float, bool]:
    """(危険体積 [mm³], 補間か)。計算済みの点の間は log(V + 1) の線形補間（**Design は計算していない**）。"""
    pts = sorted(V_COMPUTED)
    xs, ys = [], []
    for t in pts:
        v = V_COMPUTED[t][1 if cover else 0]
        if v is None:                                # 覆いありで未計算の点（−7.5）は使わない
            continue
        xs.append(t)
        ys.append(math.log(v + 1.0))
    if theta in V_COMPUTED and V_COMPUTED[theta][1 if cover else 0] is not None:
        return float(V_COMPUTED[theta][1 if cover else 0]), False
    return float(math.exp(np.interp(theta, xs, ys)) - 1.0), True


def range_hazard(lo: float, hi: float, cover: bool, step: float = 0.5) -> dict:
    """範囲 [lo, hi] の最悪の危険体積と平均（0.5° 刻み。計算済みの点でなければ補間）。"""
    th = np.arange(lo, hi + 1e-9, step)
    vals = [hazard(float(t), cover) for t in th]
    v = np.array([x[0] for x in vals])
    return {"worst": float(v.max()), "mean": float(v.mean()), "end_lo": hazard(lo, cover)[0], "end_hi": hazard(hi, cover)[0],
            "interpolated_ends": hazard(lo, cover)[1] or hazard(hi, cover)[1]}


# --- 使えなくなる姿勢 ---
def pose_groups() -> dict[str, tuple[float, float]]:
    """姿勢グループ → その J7 の目標の範囲 [deg]（robot.yaml から）。"""
    p = CFG["poses"]
    n = CFG["neck"]
    br = float(CFG["breath"]["amplitude_by_axis"]["J7"])
    return {"home": (p["home"]["J7"],) * 2, "rest_arc": (p["rest_arc"]["J7"],) * 2, "coil": (CFG["legacy_poses"]["coil"]["J7"],) * 2,
            "look(人を見る)": (n["look_min_deg"], n["look_max_deg"]), "full_rear(フル鎌首)": (n["full_rear_min_deg"], n["full_rear_max_deg"]),
            "breath(呼吸 ±%g°)" % br: (p["home"]["J7"] - br, p["home"]["J7"] + br)}


def lost_poses(lo: float, hi: float) -> list[str]:
    return [k for k, (a, b) in pose_groups().items() if a < lo or b > hi]


# --- 取り込み割合 I(c)（MUJOCO_SIM の CSV） ---
def load_intake(path: Path | None = None) -> dict:
    """(floor_class, mu_wall_scale, mu_floor_scale) → {c: (成功, 回数)}。位置ずれ ≤ 5 mm のみ。floor_class = flooring / mat / bump（凹凸 ±0.5 の代用）。"""
    p = path or (HERE / "results" / "scoop_forms_intake_c_cells.csv")
    out: dict = {}
    for r in csv.DictReader(open(p, encoding="utf-8")):
        if float(r["offset"]) > 5.0:
            continue
        q = dict(kv.split("=", 1) for kv in r["tag"].split("|")[1].split(","))
        cls = "bump" if "bump_mm" in q else r["floor"]
        key = (cls, float(q.get("mu_wall_scale", 1.0)), float(q.get("mu_floor_scale", 1.0)))
        d = out.setdefault(key, {})
        k, n = d.get(float(q["clearance_mm"]), (0, 0))
        d[float(q["clearance_mm"])] = (k + int(r["success"]), n + int(r["n"]))
    return out


def gap_from_theta(theta: float) -> float:
    """θ_E → 壁の下端のすき間 c [mm]（一様の値。θ ≥ 前縁が床に触れる角: 口の前縁の高さ = 0.1 + 0.95 θ。それより下げると、前縁は床に押しつけられ（0）、奥の壁の下が
    0.1 + 1.17 × (下げた角) で上がる。**大きい方の側 = 物が逃げやすい側 = 保守的**）。"""
    if theta >= THETA_TOUCH:
        return C_NOMINAL + MM_PER_DEG_FRONT * theta
    return C_NOMINAL + REAR_MM_PER_DEG * (THETA_TOUCH - theta)


def rate_at(curve: dict, c: float) -> float:
    cs = sorted(curve)
    ys = [curve[x][0] / curve[x][1] for x in cs]
    return float(np.interp(c, cs, ys))


def intake_curve(intake: dict, thetas: np.ndarray, classes=("flooring", "mat", "bump")) -> dict:
    """θ ごとの取り込み割合（床 3 種 × 摩擦の prior の 9 通りの平均・最小）。"""
    out = {}
    for cls in classes:
        keys = [k for k in intake if k[0] == cls]
        out[cls] = np.array([[rate_at(intake[k], gap_from_theta(float(t))) for k in keys] for t in thetas])   # (θ, 摩擦の組)
    return out


def intake_band(intake: dict, target: float, step: float = 0.05) -> tuple[float, float]:
    """すべての床・摩擦の組で I ≥ target の θ の範囲（[下端, 上端]）。"""
    th = np.arange(-5.0, 10.0 + 1e-9, step)
    ic = intake_curve(intake, th)
    ok = np.all([ic[c].min(axis=1) >= target for c in ic], axis=0)
    xs = th[ok]
    return (float(xs.min()), float(xs.max())) if len(xs) else (float("nan"), float("nan"))


# --- ストッパーの荷重 ---
ARM_M = (0.036, 0.0485)
A_DECEL_MAX = 2233.0                 # °/s²（加速度レジスタ 254 × 100 step/s² × 0.0879°。ASSUMED。実測 J1 で確認）
V_CREEP = 5.0                        # °/s（窓の端の最終の速さ）


def impact_speed(omega: float, decel_deg: float, a: float = A_DECEL_MAX) -> float:
    """端の手前 decel_deg から一定の減速度 a で減速したときの、ストッパーに当たる速さ [°/s]（下限は V_CREEP）。"""
    return max(V_CREEP, math.sqrt(max(0.0, omega ** 2 - 2.0 * a * decel_deg)))


def k_series(e_mpa: float, t_mm: float, area_mm2: float, k_pin_n_mm: float, k_win_n_mm: float = 300.0) -> float:
    """TPU 緩衝 + ピン + 窓の端の直列ばね [N/m]（k_tpu = E A / t）。"""
    k_tpu = e_mpa * area_mm2 / t_mm
    return 1.0 / (1.0 / k_tpu + 1.0 / k_pin_n_mm + 1.0 / k_win_n_mm) * 1000.0


def pin_capacity(material: str, sigma_mpa: float, d: float = 3.0, L: float = 4.5) -> float:
    return sigma_mpa * math.pi * d ** 3 / 32.0 / L


def stopper_mc(n: int = 20000, seed: int = 1, omega: float = 120.0, decel_deg: float = 0.0, tpu: bool = True, material: str = "steel",
               a_decel: float | None = None, e_mpa: float | None = None, t_mm: float | None = None) -> dict:
    """衝撃 + 静的（上限 ÷ 腕）の合計荷重（保守的に和）と、ピンの許容荷重（曲げ）から、安全率 SF = 許容 ÷ 荷重 を prior で。"""
    r = np.random.default_rng(seed)
    j = np.exp(r.uniform(math.log(A["servo"]["reflected_inertia_kgm2"][0]), math.log(A["servo"]["reflected_inertia_kgm2"][1]), n))
    stall = r.uniform(*A["servo"]["stall_torque_nm_at_7v4"], n)
    e = r.choice(A["control"]["cap_model_error"], n)
    arm = r.uniform(*ARM_M, n)
    static = 0.167 * stall * (1 + e) / arm                     # 作業中でなく、既定の上限（0.167）の場合
    if tpu:
        E = np.exp(r.uniform(math.log(3.0), math.log(30.0), n)) if e_mpa is None else np.full(n, e_mpa)      # MPa（TPU 60A〜95A 級、ASSUMED）
        t = r.uniform(0.5, 2.0, n) if t_mm is None else np.full(n, t_mm)                                   # mm
        k_pin = 459.0 if material == "PLA" else 26000.0              # N/mm（片持ち 3EI/L³）
        k = 1.0 / (1.0 / (E * 10.8 / t) + 1.0 / k_pin + 1.0 / 300.0) * 1000.0                # N/m（TPU パッド 3.6 × 3 mm² + ピン + 窓の端 300 N/mm の直列）
    else:
        k = np.full(n, 1.0e5)                                        # 硬い同士 100 N/mm
    a = a_decel if a_decel is not None else A_DECEL_MAX
    w = math.radians(impact_speed(omega, decel_deg, a))
    imp = w * np.sqrt(j * k)
    load = imp + static
    sig = r.uniform(20.0, 50.0, n) if material == "PLA" else r.uniform(200.0, 350.0, n)
    cap = sig * math.pi * 3.0 ** 3 / 32.0 / 4.5
    sf = cap / load
    return {"impact_p50": float(np.percentile(imp, 50)), "impact_p95": float(np.percentile(imp, 95)), "load_p95": float(np.percentile(load, 95)),
            "sf_p05": float(np.percentile(sf, 5)), "sf_p50": float(np.percentile(sf, 50)), "p_sf_lt_1p5": float(np.mean(sf < 1.5)),
            "p_sf_lt_1": float(np.mean(sf < 1.0)), "omega_impact_dps": impact_speed(omega, decel_deg, a)}


# --- 範囲の候補・パレート・膝 ---
LO_CANDIDATES = (-2.0, -3.0, -4.0, -5.0)
HI_CANDIDATES = (1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0)


def candidates(cover: bool) -> list[dict]:
    rows = []
    for lo in LO_CANDIDATES:
        for hi in HI_CANDIDATES:
            h = range_hazard(lo, hi, cover)
            lp = lost_poses(lo, hi)
            rows.append({"lo": lo, "hi": hi, "cover": cover, **h, "lost_n": len([x for x in lp if not x.startswith("breath")]), "lost_n_with_breath": len(lp), "lost": lp})
    return rows


def pareto(rows: list[dict], keys=("worst", "lost_n")) -> list[dict]:
    front = []
    for r in rows:
        if not any(all(o[k] <= r[k] for k in keys) and any(o[k] < r[k] for k in keys) for o in rows):
            front.append(r)
    return sorted(front, key=lambda x: (x[keys[0]], x[keys[1]]))


def knee(side: str, cover: bool) -> dict:
    """「次の計算済みの点で危険体積が最も大きく増える手前」の点（膝）。**Design が計算した点だけ**で比べる（補間の点は使わない）。
    side = "hi"（θ ≥ 1。窓の外の最初の点から）/ "lo"（θ ≤ −2）。"""
    idx = 1 if cover else 0
    if side == "hi":
        pts = [t for t in sorted(V_COMPUTED) if 1.0 <= t <= 10.0 and V_COMPUTED[t][idx] is not None]
    else:
        pts = [t for t in sorted(V_COMPUTED, reverse=True) if -10.0 <= t <= -1.0 and V_COMPUTED[t][idx] is not None]
    v = {t: float(V_COMPUTED[t][idx]) for t in pts}
    ratios = {pts[i]: v[pts[i + 1]] / max(v[pts[i]], 1.0) for i in range(len(pts) - 1) if abs(pts[i]) >= (2.0 if side == "lo" else 2.0)}
    best = max(ratios, key=ratios.get)
    return {"knee": best, "ratio_next": ratios[best], "ratios": ratios, "next": pts[pts.index(best) + 1]}


V_NEAR_MIN = 30.0                    # 窓の端の手前で許す速さの下限 [°/s]（Floor Watch の J1 は小さな補正だけ。これ以上遅いと使いにくい、という仮定 = ASSUMED）
A_DECEL_MIN = 1000.0                 # 保守的な減速度 [°/s²]（最大 2233。ASSUMED。J1 の実測で確認）


def room_deg(edge: float, side: str) -> float:
    """作業窓の端（+1° / −2°）からストッパー（edge）までの角度 [°]。"""
    return (edge - WINDOW[1]) if side == "hi" else (WINDOW[0] - edge)


def speed_cap_dps(edge: float, side: str, m_ctrl_deg: float, a: float = A_DECEL_MIN) -> float:
    """余裕（room − 位置の誤差）で、V_CREEP まで減速できる ω_near [°/s]。余裕が負なら 0（位置の誤差だけでストッパーに届く）。"""
    r = room_deg(edge, side) - m_ctrl_deg
    return 0.0 if r <= 0 else math.sqrt(V_CREEP ** 2 + 2.0 * a * r)


def feasibility_mc(n: int = 20000, seed: int = 3, v_min: float | None = None) -> dict:
    """prior（D・B）で、各候補の端が「ω_near ≥ 30°/s で減速できる」確率（位置の誤差 δθ は量子化 + D + B、ENC_MOTOR の確率 0.5）。"""
    r = np.random.default_rng(seed)
    d = r.uniform(0.0, 4.0, n)
    bk = np.where(r.random(n) < 0.5, r.uniform(0.0, 8.0, n), 0.0)
    m = (0.5 + d + bk) * STEP_DEG
    a = r.uniform(A_DECEL_MIN, A_DECEL_MAX, n)
    vmin = V_NEAR_MIN if v_min is None else v_min
    out = {"hi": {}, "lo": {}}
    for side, cands in (("hi", HI_CANDIDATES), ("lo", LO_CANDIDATES)):
        for e in cands:
            w = np.array([speed_cap_dps(e, side, m[i], a[i]) for i in range(n)])
            out[side][e] = {"p_feasible": float(np.mean(w >= vmin)), "cap_p05": float(np.percentile(w, 5)), "cap_p50": float(np.percentile(w, 50))}
    return out


def intake_working_point_mc(intake: dict, target: float = 0.90, n: int = 20000, seed: int = 4) -> dict:
    """作業点（θ_cmd = 0°、床接触の較正で c = 0.1 mm）の取り込み割合を prior で。c = 0.1 + 0.95 δθ + ε（δθ は J1 の位置の誤差、ε = 足の帯・スキッドの平らさ ±0.05 mm）。"""
    r = np.random.default_rng(seed)
    classes = ["flooring", "mat", "bump"]
    keys = {c: [k for k in intake if k[0] == c] for c in classes}
    d = r.uniform(0.0, 4.0, n)
    bk = np.where(r.random(n) < 0.5, r.uniform(0.0, 8.0, n), 0.0)
    dth = (r.uniform(-0.5, 0.5, n) + r.uniform(-1, 1, n) * (d + bk)) * STEP_DEG
    eps = r.uniform(-0.05, 0.05, n)
    vals = []
    for i in range(n):
        cls = classes[int(r.integers(3))]
        k = keys[cls][int(r.integers(len(keys[cls])))]
        c = max(0.0, C_NOMINAL + MM_PER_DEG_FRONT * dth[i] + eps[i])
        vals.append(rate_at(intake[k], c))
    v = np.array(vals)
    return {"mean": float(v.mean()), "p05": float(np.percentile(v, 5)), "p50": float(np.percentile(v, 50)), "p_ge_target": float(np.mean(v >= target)), "target": target}


def control_margin_deg(n: int = 20000, seed: int = 2) -> dict:
    """J1 の位置の誤差 δθ（量子化 ±0.5 ステップ + 不感帯 D + バックラッシ B（エンコーダがモーター側の場合））の prior。"""
    r = np.random.default_rng(seed)
    d = r.uniform(0.0, 4.0, n)                                    # ステップ（UNKNOWN。J1-1 で実測）
    b = np.where(r.random(n) < 0.5, r.uniform(0.0, 8.0, n), 0.0)  # ENC_MOTOR の確率 0.5
    err = (0.5 + d + b) * STEP_DEG                                # 最悪側の和
    return {"p50": float(np.percentile(err, 50)), "p95": float(np.percentile(err, 95)), "max": float(err.max())}


def sensitivity_knee() -> dict:
    """危険体積の不確かさ（Design の見積もり。V(+5) が Design の値の 1/k だったら、膝はどう動くか）。"""
    out = {}
    for k in (1.0, 2.0, 4.0, 8.0):
        v = {t: V_COMPUTED[t][0] * (1.0 / k if t == 5.0 else 1.0) for t in (1.0, 2.0, 3.0, 5.0, 10.0)}
        ratios = {2.0: v[3.0] / v[2.0], 3.0: v[5.0] / v[3.0], 5.0: v[10.0] / v[5.0]}
        out[k] = {"knee": max(ratios, key=ratios.get), "v5": v[5.0]}
    return out


def main() -> dict:
    out = {}
    out["margin_control_deg"] = control_margin_deg()
    out["knee"] = {f"{side}_{'cover' if cv else 'nocover'}": knee(side, cv) for side in ("hi", "lo") for cv in (False, True)}
    out["feasibility_mc"] = feasibility_mc()
    out["sensitivity_knee"] = sensitivity_knee()
    return out


def stopper_sweep(n: int = 4000) -> list[dict]:
    """ω（60〜180 °/s）× TPU 緩衝（E 3 / 10 MPa、厚み 1 / 3 / 6 mm。k = E A / t）× 端の手前で減速を始める角度（0〜5°、減速度 1000 °/s²）× ピン（PLA / 鋼）。"""
    rows = []
    for material in ("PLA", "steel"):
        for omega in (60.0, 90.0, 120.0, 150.0, 180.0):
            for tpu, e, t in ((False, None, None), (True, 3.0, 1.0), (True, 3.0, 3.0), (True, 10.0, 3.0), (True, 3.0, 6.0), (True, 10.0, 6.0)):
                for dec in (0.0, 1.0, 2.0, 3.0, 5.0):
                    r = stopper_mc(n=n, omega=omega, decel_deg=dec, tpu=tpu, material=material, a_decel=A_DECEL_MIN, e_mpa=e, t_mm=t)
                    rows.append({"material": material, "omega_dps": omega, "tpu": f"E{e:g}MPa_t{t:g}mm" if tpu else "なし", "decel_deg": dec, **r})
    return rows


def write_outputs() -> dict:
    """結果を simulation/results/ に書く。"""
    res = main()
    out = {"main": res}
    RES.mkdir(exist_ok=True)
    stop = stopper_sweep()
    with open(RES / "j1_range_opt_stopper_sweep.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(stop[0]))
        w.writeheader()
        w.writerows(stop)
    rows = []
    for cover in (False, True):
        for r in candidates(cover):
            hi_f = res["feasibility_mc"]["hi"][r["hi"]]["p_feasible"]
            lo_f = res["feasibility_mc"]["lo"][r["lo"]]["p_feasible"]
            rows.append({**{k: v for k, v in r.items() if k != "lost"}, "lost": ";".join(r["lost"]), "p_feasible_hi": hi_f, "p_feasible_lo": lo_f})
    with open(RES / "j1_range_opt_candidates.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    out["rows"] = rows
    out["stopper"] = stop
    return out


def recommend(res: dict, p_min: float = 0.99, v_min: float | None = None) -> dict:
    """制約（窓 + 余裕 + 減速でき、取り込みの帯を含む）を満たす候補のうち、最悪の危険体積が最小のもの（覆いあり / なし）。"""
    fe = res["feasibility_mc"] if v_min is None else feasibility_mc(v_min=v_min)
    out = {}
    for cover in (False, True):
        best = None
        for lo in LO_CANDIDATES:
            for hi in HI_CANDIDATES:
                if fe["hi"][hi]["p_feasible"] < p_min or fe["lo"][lo]["p_feasible"] < p_min:
                    continue
                h = range_hazard(lo, hi, cover)
                if best is None or (h["worst"], -(lo), hi) < (best["worst"], -(best["lo"]), best["hi"]):
                    best = {"lo": lo, "hi": hi, **h}
        out["cover" if cover else "nocover"] = best
    return out


def make_plot(path: Path, rows: list[dict], rec: dict) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7.5, 4.6))
    for cover, mk in ((False, "o"), (True, "s")):
        xs = [r["worst"] for r in rows if r["cover"] == cover]
        ys = [r["lost_n"] + (0.08 if cover else -0.08) for r in rows if r["cover"] == cover]
        ax.scatter(xs, ys, marker=mk, alpha=0.55, label="cover: yes" if cover else "cover: no")
    for k, r in rec.items():
        if r:
            ax.scatter([r["worst"]], [r.get("lost_n", 5)], s=180, facecolors="none", edgecolors="k")
            ax.annotate(f"[{r['lo']:g}, +{r['hi']:g}] {k}", (r["worst"], 5.1), fontsize=8, rotation=20)
    ax.set_xscale("log")
    ax.set_xlabel("worst-case gap hazard volume [mm^3] (Design CAD_CONCEPT estimate)")
    ax.set_ylabel("poses lost (of 5 groups)")
    ax.set_title("J1 range candidates: hazard vs lost poses (simulation only)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


if __name__ == "__main__":
    print(json.dumps(main(), ensure_ascii=False, indent=1))
