r"""HG-H1 アクチュエータの Hardware Gap: サーボ・電源・熱・安全を prior で掃引し、H1 で何を測れば決まるかを絞る。

    .\.venv\Scripts\python.exe simulation\hardware_gaps\HG-H1_actuator\run.py
    .\.venv\Scripts\python.exe simulation\hardware_gaps\HG-H1_actuator\run.py --measured hardware\prototypes\H1_joint\h1_joint_YYYYMMDD.csv

モデル（ACTUATOR_MODEL_SIM。**実測ではない**）:
  直流モータ ＋ 減速機を出力軸で見た線形モデル。
    τ_avail(V, ω) = τ_stall(V) · (1 − |ω| / ω0(V))、τ_stall ∝ V、ω0 ∝ V
    I = I0 + (I_stall − I0) · |τ| / τ_stall(V)（電流はトルクに比例）
    トルク上限 = ratio · τ_stall(V) · (1 + e)（レジスタが「電圧ごとの最大に対する割合」だという仮定。e はモデル誤差）
  電源: V_bus = V_oc − I_total · (R_pack + R_wiring)
  熱: 巻線の銅損 P = I² R_w（R_w = 7.4 V / I_stall）、一次遅れ T = T_amb + R_th · P · (1 − e^(−t/τ))
  安全: 静的な挟み込み F = 上限 / r、衝撃 F ≈ ω · √(J_reflected · k_contact)（トルク上限では抑えられない過渡）
歩容の負荷（関節トルクと角速度の時系列）は HG-H0 と同じ平面摩擦モデルから取る。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT))

from serpens.config import load_config  # noqa: E402
from simulation.planar_friction import Friction, PlanarSnake, body_from_cfg  # noqa: E402

A = yaml.safe_load((HERE / "assumptions.yaml").read_text(encoding="utf-8"))
H0 = yaml.safe_load((HERE.parent / "HG-H0_friction" / "assumptions.yaml").read_text(encoding="utf-8"))
SOURCE = "ACTUATOR_MODEL_SIM"
V_NOM = 7.4
N_MC = 3000


def demand_scenarios() -> list[dict[str, Any]]:
    """H0 の平面モデルで、歩容 1 周期の関節トルク・角速度の時系列を作る。"""
    d = A["demand"]
    out = []
    for cfg_name in d["configs"]:
        cfg = load_config(overlay=ROOT / H0["configs"][cfg_name]["overlay"])
        body = body_from_cfg(cfg, cfg_name)
        for law in d["laws"]:
            for mu_f in d["mu_forward"]:
                for r in d["ratio"]:
                    for amp, waves in d["gaits"]:
                        rec: dict[str, list] = {}
                        res = PlanarSnake(body, Friction.simple(mu_f, mu_f * r, law=law)).run_cycle(
                            float(amp), float(waves), float(d["freq_hz"]), 0.0, 32, record=rec)
                        out.append({"config": cfg_name, "law": law, "mu_f": mu_f, "ratio": r, "amp": amp, "waves": waves,
                                    "speed_mm_s": res.speed_mm_s, "tau": np.abs(np.array(rec["torque_nm"])),
                                    "omega": np.radians(np.abs(np.array(rec["qdot_dps"]))), "n_servo": len(body.joint_names)})
    return out


def sample_priors(rng: np.random.Generator, n: int) -> dict[str, np.ndarray]:
    s, p = A["servo"], A["power"]
    u = lambda lo_hi: rng.uniform(lo_hi[0], lo_hi[1], n)
    lu = lambda lo_hi: np.exp(rng.uniform(math.log(lo_hi[0]), math.log(lo_hi[1]), n))
    return {"tau_s": u(s["stall_torque_nm_at_7v4"]), "rpm0": u(s["no_load_rpm_at_7v4"]),
            "i_s": u(s["stall_current_a_at_7v4"]), "i0": u(s["no_load_current_a"]),
            "r_th": u(s["thermal_resistance_k_per_w"]), "tau_th": u(s["thermal_time_constant_s"]),
            "j_ref": lu(s["reflected_inertia_kgm2"]), "backdrive": lu(s["backdrive_torque_nm"]),
            "v_oc": u(p["v_open_circuit"]), "r_pack": u(p["pack_resistance_ohm"]), "r_wire": u(p["wiring_resistance_ohm"]),
            "i_elec": u(p["electronics_current_a"]), "cap_mah": u(p["capacity_mah"]),
            "cap_err": rng.choice(np.array(A["control"]["cap_model_error"], dtype=float), n),
            "k_contact": lu(A["safety"]["contact_stiffness_n_per_mm"]) * 1000.0,
            "lever": u(A["safety"]["pinch_lever_m"]), "t_amb": rng.uniform(25.0, 35.0, n)}


def evaluate(sc: dict[str, Any], pr: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    """1 つの歩容負荷 × prior のサンプル群（ベクトル化）。"""
    tau, om = sc["tau"], sc["omega"]                          # (T, J)
    n = len(pr["tau_s"])
    ratio = float(A["control"]["torque_limit_ratio"])
    r_tot = pr["r_pack"] + pr["r_wire"]
    # 電流 → 電圧降下 → 使えるトルク（1 回の固定点反復で十分: 降下は数 %）
    v_bus = pr["v_oc"].copy()
    for _ in range(3):
        tau_s_v = pr["tau_s"] * v_bus / V_NOM
        i_ser = pr["i0"][:, None, None] + (pr["i_s"] - pr["i0"])[:, None, None] * tau[None] / tau_s_v[:, None, None]
        i_tot = i_ser.sum(2) + pr["i_elec"][:, None]          # (n, T)
        v_min = pr["v_oc"] - i_tot.max(1) * r_tot
        v_bus = v_min
    tau_s_v = pr["tau_s"] * v_bus / V_NOM
    w0 = pr["rpm0"] * 2 * math.pi / 60.0 * v_bus / V_NOM
    avail = tau_s_v[:, None, None] * np.clip(1.0 - om[None] / w0[:, None, None], 0.0, None)
    cap = ratio * tau_s_v * (1.0 + pr["cap_err"])
    limit = np.minimum(avail, cap[:, None, None])
    margin = (limit - tau[None]).min(axis=(1, 2))             # < 0 なら歩容を追従できない
    # 熱（最も負荷の大きい関節）
    r_w = V_NOM / pr["i_s"]
    p_cu = (i_ser ** 2).mean(1) * r_w[:, None]                # (n, J) 周期平均
    t_ss = pr["t_amb"][:, None] + pr["r_th"][:, None] * p_cu
    t_worst = t_ss.max(1)
    stop = float(A["control"]["temp_stop_c"])
    with np.errstate(divide="ignore", invalid="ignore"):
        frac = (stop - pr["t_amb"]) / (t_worst - pr["t_amb"])
        t_to_stop = np.where(t_worst > stop, -pr["tau_th"] * np.log(1.0 - np.clip(frac, 0, 0.999999)), np.inf)
    # 電流・ヒューズ・稼働時間
    i_peak = i_tot.max(1)
    i_avg = i_tot.mean(1)
    i_stall_capped = sc["n_servo"] * (pr["i0"] + (pr["i_s"] - pr["i0"]) * np.clip(ratio * (1 + pr["cap_err"]), 0, 1))
    i_stall_uncapped = sc["n_servo"] * pr["i_s"] * pr["v_oc"] / V_NOM
    runtime_min = pr["cap_mah"] / 1000.0 * float(A["power"]["usable_fraction"]) / i_avg * 60.0
    # 安全（危険側）
    omega_peak = om.max()
    f_static = cap / pr["lever"]
    f_static_fail = tau_s_v / pr["lever"]                     # レジスタが効かない場合（ストール全開）
    f_impact = omega_peak * np.sqrt(pr["j_ref"] * pr["k_contact"])
    return {"margin_nm": margin, "t_ss_c": t_worst, "t_to_stop_s": t_to_stop, "i_peak_a": i_peak, "i_avg_a": i_avg,
            "i_stall_capped_a": i_stall_capped, "i_stall_uncapped_a": i_stall_uncapped, "runtime_min": runtime_min,
            "v_min": v_bus, "f_static_n": f_static, "f_static_fail_n": f_static_fail, "f_impact_n": f_impact,
            "cap_nm": cap, "backdrive_nm": pr["backdrive"], "overvolt": pr["v_oc"] > float(A["servo"]["max_input_voltage_register_v"])}


def spearman(x: np.ndarray, y: np.ndarray) -> float:
    ok = np.isfinite(x) & np.isfinite(y)
    if ok.sum() < 10:
        return float("nan")
    rx = np.argsort(np.argsort(x[ok]))
    ry = np.argsort(np.argsort(y[ok]))
    return float(np.corrcoef(rx, ry)[0, 1])


def run(stamp: str) -> None:
    rng = np.random.default_rng(20260929)
    pr = sample_priors(rng, N_MC)
    scs = demand_scenarios()
    rows, sens = [], []
    for sc in scs:
        ev = evaluate(sc, pr)
        rows.append({"config": sc["config"], "law": sc["law"], "mu_f": sc["mu_f"], "ratio": sc["ratio"], "amp": sc["amp"],
                     "speed_mm_s": sc["speed_mm_s"], "peak_demand_nm": float(sc["tau"].max()),
                     "peak_speed_dps": float(np.degrees(sc["omega"].max())),
                     **{f"p_track_{int(e * 100):+d}": float(np.mean(ev["margin_nm"][pr["cap_err"] == e] >= 0))
                        for e in A["control"]["cap_model_error"]},
                     "p_track_all": float(np.mean(ev["margin_nm"] >= 0)),
                     "p_thermal_stop": float(np.mean(np.isfinite(ev["t_to_stop_s"]))),
                     "t_ss_p95_c": float(np.percentile(ev["t_ss_c"], 95)),
                     "i_peak_p95_a": float(np.percentile(ev["i_peak_a"], 95)),
                     "runtime_p05_min": float(np.percentile(ev["runtime_min"], 5)),
                     "v_min_p05": float(np.percentile(ev["v_min"], 5))})
        for out_key in ("margin_nm", "t_ss_c", "i_peak_a", "runtime_min"):
            for k in ("tau_s", "rpm0", "i_s", "i0", "r_th", "cap_err", "v_oc", "r_pack", "r_wire", "i_elec"):
                sens.append({"config": sc["config"], "law": sc["law"], "mu_f": sc["mu_f"], "ratio": sc["ratio"], "amp": sc["amp"],
                             "output": out_key, "param": k, "spearman": spearman(pr[k], ev[out_key])})
    res = HERE / "results"
    res.mkdir(exist_ok=True)
    _csv(res / "scenarios.csv", rows)
    _csv(res / "sensitivity.csv", sens)
    ev0 = evaluate(max(scs, key=lambda s: s["tau"].max()), pr)       # 最も重い負荷で安全・電流を見る
    safety = {k: {"p50": float(np.percentile(ev0[k], 50)), "p95": float(np.percentile(ev0[k], 95)), "max": float(np.max(ev0[k]))}
              for k in ("f_static_n", "f_static_fail_n", "f_impact_n", "i_stall_capped_a", "i_stall_uncapped_a", "cap_nm")}
    safety["p_overvolt_at_full_charge"] = float(np.mean(ev0["overvolt"]))
    safety["p_backdrive_gt_cap"] = float(np.mean(ev0["backdrive_nm"] > ev0["cap_nm"]))
    (res / "safety.json").write_text(json.dumps(safety, indent=2), encoding="utf-8")
    made = _plots(rows, sens, ev0, pr)
    _decision_md(rows, sens, safety, made, stamp)
    print(f"{len(scs)} 負荷 × {N_MC} prior → results/, plots/, decision_boundary.md")


def _csv(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as fp:
        w = csv.DictWriter(fp, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def _plots(rows, sens, ev0, pr) -> list[str]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    out = HERE / "plots"
    out.mkdir(exist_ok=True)
    made = []
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), sharey=True)
    for ax, cfg in zip(axes, A["demand"]["configs"]):
        for e in A["control"]["cap_model_error"]:
            g = [r for r in rows if r["config"] == cfg and r["law"] == "decoupled" and r["amp"] == 30]
            xs = sorted({r["mu_f"] * r["ratio"] for r in g})
            ys = [np.mean([r[f"p_track_{int(e * 100):+d}"] for r in g if abs(r["mu_f"] * r["ratio"] - x) < 1e-9]) for x in xs]
            ax.plot(xs, ys, marker="o", ms=3, label=f"cap error {e:+.0%}")
        ax.set_xscale("log")
        ax.set_xlabel("mu_lateral (= mu_forward x ratio)")
        ax.set_title(f"{cfg}, decoupled, A30")
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("P(gait trackable) over actuator priors")
    axes[1].legend(fontsize=8)
    fig.suptitle(f"{SOURCE} - not hardware data")
    fig.tight_layout()
    fig.savefig(out / "trackable_vs_lateral_mu.png", dpi=120)
    plt.close(fig)
    made.append("trackable_vs_lateral_mu.png")
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.hist(ev0["f_static_n"], bins=50, alpha=0.6, label="static pinch (cap / lever)")
    ax.hist(ev0["f_impact_n"], bins=50, alpha=0.6, label="impact (omega*sqrt(J*k))")
    ax.set_xlabel("force N (worst load case)")
    ax.set_title("safety forces - no threshold decided (OQ-0006)")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "safety_forces.png", dpi=120)
    plt.close(fig)
    made.append("safety_forces.png")
    return made


def _decision_md(rows, sens, safety, made, stamp) -> None:
    L = [f"# HG-H1 アクチュエータ — 判定の境界（自動生成 {stamp}。source = {SOURCE}。**実測ではない**）\n",
         f"prior は `assumptions.yaml`（C044 の資料値が中心。UNKNOWN の幅は広め）。Monte Carlo {N_MC} 点 × 歩容負荷 "
         f"{len(rows)} 通り（負荷は HG-H0 の平面摩擦モデル）。トルク上限 = {A['control']['torque_limit_ratio']} × τ_stall(V) × (1 + e)。\n",
         "## 1. 歩容を追従できる確率（トルク上限の誤差 e ごと）\n",
         "| 構成 | 法則 | μ_前 | 比 | 振幅 | 必要トルク peak N·m | 必要速さ °/s | " +
         " | ".join(f"e={e:+.0%}" for e in A["control"]["cap_model_error"]) + " | 熱停止の確率 | 稼働 p05 分 |",
         "|---|---|---|---|---|---|---|" + "---|" * len(A["control"]["cap_model_error"]) + "---|---|"]
    for r in rows:
        L.append(f"| {r['config']} | {r['law']} | {r['mu_f']} | {r['ratio']} | {r['amp']} | {r['peak_demand_nm']:.3f} | {r['peak_speed_dps']:.0f} | "
                 + " | ".join(f"{r[f'p_track_{int(e * 100):+d}']:.0%}" for e in A["control"]["cap_model_error"])
                 + f" | {r['p_thermal_stop']:.0%} | {r['runtime_p05_min']:.0f} |")
    # 必要トルクの境界: 上限 0.45 × (1+e) に収まる μ_横
    L.append("\n## 2. 読み\n")
    worst = [r for r in rows if r["peak_demand_nm"] > float(A["control"]["software_torque_limit_nm"]) * 0.6]
    L.append(f"- 必要トルクが上限 0.45 N·m の 60% を超える負荷: {len(worst)} / {len(rows)} 通り。"
             "必要トルクは **μ_横（= μ_前 × 比）にほぼ比例**するので、滑りにくい床（カーペット）ほどトルクが律速になる")
    lat_ok = {}
    for cfg in A["demand"]["configs"]:
        for e in A["control"]["cap_model_error"]:
            g = [r for r in rows if r["config"] == cfg and r[f"p_track_{int(e * 100):+d}"] >= 0.9]
            lat_ok[(cfg, e)] = max((r["mu_f"] * r["ratio"] for r in g), default=None)
    L.append("- 追従確率 ≥ 90% を保てる μ_横 の最大（掃引した点のうち）:")
    for (cfg, e), v in lat_ok.items():
        L.append(f"  - {cfg}, e={e:+.0%}: {'—（どの点も 90% 未満）' if v is None else f'{v:.2f}'}")
    # 感度
    L.append("\n## 3. どの実測値が効くか（Spearman の順位相関の絶対値の平均。大きいほど、その値を測ると結果が決まる）\n")
    L.append("| 出力 | 1 位 | 2 位 | 3 位 |\n|---|---|---|---|")
    for out_key in ("margin_nm", "t_ss_c", "i_peak_a", "runtime_min"):
        agg: dict[str, list[float]] = {}
        for s in sens:
            if s["output"] == out_key and math.isfinite(s["spearman"]):
                agg.setdefault(s["param"], []).append(abs(s["spearman"]))
        top = sorted(((np.mean(v), k) for k, v in agg.items()), reverse=True)[:3]
        L.append(f"| {out_key} | " + " | ".join(f"{k} ({m:.2f})" for m, k in top) + " |")
    L.append("\n## 4. 安全（危険側。**しきい値は未決定 = OQ-0006。ここでは合否を出さない**）\n")
    L.append("| 量 | p50 | p95 | 最大 |\n|---|---|---|---|")
    names = {"f_static_n": "静的な挟み込み力 N（上限 / 腕 20〜95mm）", "f_static_fail_n": "上限が効かない場合の挟み込み力 N（ストール全開）",
             "f_impact_n": "衝撃力 N（ω√(J k)。トルク上限では抑えられない）", "cap_nm": "実際のトルク上限 N·m",
             "i_stall_capped_a": "全関節が上限で止まったときの電流 A", "i_stall_uncapped_a": "上限が効かず全関節ストールの電流 A"}
    for k, label in names.items():
        v = safety[k]
        L.append(f"| {label} | {v['p50']:.2f} | {v['p95']:.2f} | {v['max']:.2f} |")
    L.append(f"\n- 満充電（最大 8.4V）がメモリテーブル初期値の最高入力電圧 8.0V を超える確率（残量一様）: "
             f"{safety['p_overvolt_at_full_charge']:.0%} → **H1 の最初に 14 番地を読む**（超えると過電圧保護で動かない可能性）")
    L.append(f"- 脱力しても外から回すのに要るトルクが上限より大きい確率: {safety['p_backdrive_gt_cap']:.0%}"
             "（この場合、脱力しても挟まれた指は自分で抜けにくい。docs/contact_release_requirements.md §2）")
    L.append("\n## 図\n")
    L += [f"- `plots/{m}`" for m in made]
    (HERE / "decision_boundary.md").write_text("\n".join(L) + "\n", encoding="utf-8")


# ---- 実測の取り込み --------------------------------------------------------------------------------
def ingest(csv_path: Path) -> Path:
    """H1 の CSV（hardware/prototypes/H1_joint/h1_joint_template.csv の形）から prior を当てはめ直す。"""
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    mdir = HERE / "results" / "measured"
    (mdir / "raw").mkdir(parents=True, exist_ok=True)
    raw = mdir / "raw" / f"{csv_path.stem}_{stamp}{csv_path.suffix}"
    shutil.copy2(csv_path, raw)
    with csv_path.open(encoding="utf-8-sig", newline="") as fp:
        rows = list(csv.DictReader(fp))
    f = lambda r, k: float(r[k]) if (r.get(k) or "").strip() else None
    problems, fits = [], {}
    tq = [(f(r, "torque_nm_calc"), f(r, "current_a")) for r in rows if r.get("test_id") in ("4", "torque_current")]
    tq = [(t, i) for t, i in tq if t is not None and i is not None]
    if len(tq) >= 3:
        t, i = np.array(tq).T
        slope, icpt = np.polyfit(t, i, 1)
        fits["kt_a_per_nm"], fits["no_load_current_a"] = float(slope), float(icpt)
    else:
        problems.append("トルク-電流（test_id 4）が 3 点未満 → 電流の傾きを当てはめない")
    stall = [(f(r, "torque_nm_calc"), f(r, "current_a")) for r in rows if r.get("test_id") in ("5", "stall")]
    stall = [s for s in stall if s[0] is not None]
    if stall:
        fits["stall_torque_nm"] = float(max(s[0] for s in stall))
        cur = [s[1] for s in stall if s[1] is not None]
        if cur:
            fits["stall_current_a"] = float(max(cur))
    else:
        problems.append("ストール（test_id 5）が無い")
    temp = [(f(r, "duration_s"), f(r, "temp_c")) for r in rows if r.get("test_id") in ("6", "thermal")]
    temp = [x for x in temp if x[0] is not None and x[1] is not None]
    if len(temp) >= 4:
        ts, T = np.array(sorted(temp)).T
        best = None
        for tau in np.linspace(60, 3000, 300):                   # T = T0 + ΔT (1 − e^(−t/τ)) の τ を格子で
            x = 1 - np.exp(-ts / tau)
            dT, T0 = np.polyfit(x, T, 1)
            err = float(((T0 + dT * x - T) ** 2).sum())
            if best is None or err < best[0]:
                best = (err, tau, dT, T0)
        fits["thermal_time_constant_s"], fits["thermal_rise_c"] = float(best[1]), float(best[2])
    else:
        problems.append("温度の時系列（test_id 6）が 4 点未満")
    cap = [f(r, "torque_nm_calc") for r in rows if r.get("test_id") in ("7", "cap")]
    cap = [c for c in cap if c is not None]
    if cap:
        fits["measured_cap_nm"] = float(max(cap))
        fits["cap_error_vs_0p45"] = float(max(cap) / 0.45 - 1.0)
    if "stall_torque_nm" in fits:
        fits["recommended_ratio"] = float(A["control"]["software_torque_limit_nm"] / fits["stall_torque_nm"])
    L = [f"# HG-H1 実測の取り込み（{stamp}）\n", f"- 入力 `{csv_path.as_posix()}`、控え `results/measured/raw/{raw.name}`",
         "- 当てはめた値は **HARDWARE_VERIFIED（この個体・この条件に限る）**。これを prior の代わりに入れた再評価は ACTUATOR_MODEL_SIM\n",
         "## 当てはめ\n"] + [f"- {k}: {v:.4g}" for k, v in fits.items()] + ["\n## 問題\n"] + ([f"- {p}" for p in problems] or ["- なし"])
    if "recommended_ratio" in fits:
        L.append(f"\n## 判定の更新（提案。**config は自動で書き換えない**。安全に関わるので User 承認）\n\n"
                 f"- トルク制限レジスタ比: 現在 {A['control']['torque_limit_ratio']} → 実測ストールからは {fits['recommended_ratio']:.3f}")
        if "measured_cap_nm" in fits:
            L.append(f"- 比 {A['control']['torque_limit_ratio']} で実際に出たトルク上限: {fits['measured_cap_nm']:.3f} N·m"
                     f"（0.45 に対して {fits['cap_error_vs_0p45']:+.0%}）")
    out = mdir / f"{csv_path.stem}_{stamp}_decision.md"
    out.write_text("\n".join(L) + "\n", encoding="utf-8")
    (mdir / f"{csv_path.stem}_{stamp}_fit.json").write_text(json.dumps({"fits": fits, "problems": problems}, indent=2, ensure_ascii=False),
                                                              encoding="utf-8")
    ho = ROOT / "ai-outbox" / "handoffs" / f"{datetime.now():%Y-%m-%d}_HG-H1_measured.md"
    ho.parent.mkdir(parents=True, exist_ok=True)
    with ho.open("a", encoding="utf-8") as fp:
        fp.write(f"\n## {stamp} H1 実測を取り込んだ\n\n- `simulation/hardware_gaps/HG-H1_actuator/results/measured/{out.name}`\n"
                 f"- 問題 {len(problems)} 件。レジスタ比の変更は User 承認が要る\n")
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--measured", type=Path, default=None)
    args = ap.parse_args()
    if args.measured:
        print(ingest(args.measured))
        return 0
    run(datetime.now().strftime("%Y-%m-%d %H:%M"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
