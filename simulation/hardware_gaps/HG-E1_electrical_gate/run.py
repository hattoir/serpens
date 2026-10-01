r"""HG-E1 電気層（ELECTRICAL_SAFETY_GATE 8 項目）の Hardware Gap: パラメータ化 → Monte Carlo → 最悪ケース → 判断境界。**ELECTRICAL_BUDGET_MC。すべて prior（ASSUMPTION）。実測 0 件。**

    .\.venv\Scripts\python.exe simulation\hardware_gaps\HG-E1_electrical_gate\run.py

何を出すか（各 GATE 項目に、「どの値より大きければ / 小さければ何が変わるか」= 判断境界。**合否・安全の確定は出さない**）:
  real_current          : N 個の保持・歩容・最悪（同時ストール）の電流の分布（MVP 5 軸 / EX-1 9 軸）
  power_capacity        : 最悪の突入での電圧降下。**電源の全抵抗 R がいくつ以下なら、最小電圧が 6 V（動作下限の目安）を下回らないか**
  overcurrent_protection: 通常の最大電流 × 余裕 と、最悪の電流のあいだに保護の動作電流の窓があるか（窓が空なら、ヒューズだけでは足りず、遅延つきの電子遮断が要る）
  wiring_heat           : 配線とコネクタの温度上昇（I²R × 熱抵抗）。**コネクタの接触抵抗がいくつ以上で上昇が X K を超えるか**
  servo_temperature     : サーボの温度（lumped）。**保持の負荷（実効電流）がいくつ以上で、10 分以内に 60 ℃ に届くか**
  independent_power_cut : ESP32 に依存しない遮断の時間（リレーの開放 + 出力の低下）
  physical_estop        : 同上（押してから遮断まで）
  real_stop_time        : 通信断・緊急停止・遮断から動きが止まるまで（通信断は config の 400 ms + 周期）
最小の実機試験は `docs/electrical_gate_boundary.md`、取り込みは `tools/ingest_measurements.py`（current / power_sag / trip / thermal / servo_temp / stop_time）。
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT))
A = yaml.safe_load((HERE / "assumptions.yaml").read_text(encoding="utf-8"))
SEED = 20261001
N = 20000


def u(rng: np.random.Generator, lohi, n: int = N) -> np.ndarray:
    return rng.uniform(lohi[0], lohi[1], n)


def currents(rng: np.random.Generator, n_servo: int) -> dict[str, np.ndarray]:
    """軸ごとに整合させる（保持 ≤ 0.6 × ストール、歩容の実効 ≤ 0.8 × ストール。1 軸が自分のストール電流を超える組み合わせは作らない）。合計は N 軸分。"""
    s = A["servo"]
    stall1 = u(rng, s["i_stall_a"])
    hold1 = np.minimum(u(rng, s["i_hold_a"]), 0.6 * stall1)
    gait1 = np.minimum(hold1 * u(rng, s["gait_rms_over_hold"]), 0.8 * stall1)
    sim = u(rng, s["simultaneity"])
    return {"hold": hold1 * n_servo, "gait_rms": gait1 * n_servo, "gait_peak": np.minimum(gait1 * 1.5, stall1) * n_servo,
            "peak_start": n_servo * stall1 * sim, "all_stall": n_servo * stall1, "stall1": stall1}


def pct(a: np.ndarray, ps=(5, 50, 95, 99)) -> dict[str, float]:
    return {f"p{p}": float(np.percentile(a, p)) for p in ps} | {"max": float(a.max())}


def sag(rng: np.random.Generator, i_peak: np.ndarray, v0: float) -> dict:
    r = u(rng, A["supply"]["r_total_ohm"])
    vmin = np.maximum(v0 - i_peak * r, 0.0)            # 0 V が下限（それ以下は「電圧が保てない」）
    vm = A["servo"]["v_min_operating"]
    # 判断境界: p95 の最悪電流で v_min = 6 V になる R
    r_max = (v0 - vm) / float(np.percentile(i_peak, 95))
    return {"v_min": pct(vmin), "p_below_6v": float(np.mean(vmin < vm)), "r_total_max_for_p95_ohm": float(r_max), "r_total_prior": A["supply"]["r_total_ohm"]}


def protection(rng: np.random.Generator, c: dict[str, np.ndarray]) -> dict:
    """同じ標本（同じ個体の電流）ごとに、設定電流 T [A]（合計）での 誤動作 = 通常の歩容のピーク（× 余裕）が T を超える確率 と、不動作 = 全軸ストールが T に届かない確率。
    **判断境界 = 2 つの確率が釣り合う T**（prior の幅の中の話。部品の動作電流のばらつき ±20 % は別に加える）。"""
    d = A["protection"]["derating"]
    normal = d * c["gait_peak"]
    n_servo = float(np.median(c["all_stall"] / c["stall1"]))
    grid = np.linspace(float(np.percentile(c["gait_peak"], 5)), float(np.percentile(c["all_stall"], 95)), 41)
    rows = [{"trip_a": float(T), "false_trip": float(np.mean(normal > T)), "no_trip_at_all_stall": float(np.mean(c["all_stall"] < T))} for T in grid]
    best = min(rows, key=lambda r: max(r["false_trip"], r["no_trip_at_all_stall"]))
    open_frac = float(np.mean(normal < c["all_stall"]))
    return {"table": rows[::5], "balanced": best, "p_window_open_per_unit": open_frac, "n_servo": n_servo, "derating": d,
            "note": "per-unit で窓が開いている割合 = 通常のピーク × 余裕 < 全軸ストール。**窓が閉じている個体では、ヒューズ 1 本では分けられない**（遅延つきの電子遮断・軸ごとの電流制限・歩容の電流を下げる側の設計が要る）"}


def wiring(rng: np.random.Generator, c: dict[str, np.ndarray]) -> dict:
    """配線 = 往復の I²R を、束ねた被覆線の熱抵抗（K·m/W）で温度上昇に（長さに依らない）。コネクタ = 接触抵抗の I²R × 熱抵抗。
    **判断境界は「許容できる実効電流」と「コネクタの接触抵抗」**。ΔT が 100 K を超える値は『許容電流を大きく超える』の意味で、数値そのものに意味は無い。"""
    rth_m = u(rng, A["wiring"]["wire_rth_k_per_w_per_m"])                      # K·m/W
    out = {}
    for gauge, rpm in A["wiring"]["wire_r_ohm_per_m"].items():
        dT = c["gait_rms"] ** 2 * rpm * 2.0 * rth_m
        i20 = float(np.sqrt(20.0 / (2.0 * rpm * float(np.median(rth_m)))))     # ΔT = 20 K になる実効電流（中央の熱抵抗）
        out[gauge] = {"delta_t_wire_k": pct(dT), "i_rms_for_20k_a": i20, "p_over_100k": float(np.mean(dT > 100.0))}
    rc = u(rng, A["wiring"]["connector_r_ohm"])
    rth_c = u(rng, [20.0, 80.0])                                                # コネクタ 1 個の熱抵抗 K/W（ASSUMED）
    dt = c["gait_rms"] ** 2 * rc * rth_c
    i95 = float(np.percentile(c["gait_rms"], 95))
    bound = {k: float(k / (i95 ** 2 * 50.0)) for k in (10, 20, 40)}             # 接触抵抗 [Ω]（熱抵抗 50 K/W、p95 の電流）
    return {"wire": out, "connector_delta_t_k": pct(dt), "connector_r_for_delta_t_ohm_at_p95_current_median_rth": bound, "i_rms_p95_a": i95}


def servo_temp(rng: np.random.Generator) -> dict:
    th = A["thermal"]
    cth, rth, rm = u(rng, th["c_th_j_per_k"]), u(rng, th["r_th_k_per_w"]), u(rng, th["r_motor_ohm"])
    amb = u(rng, th["ambient_c"])
    lim = th["limits_c"]["link_fault"]
    rows = {}
    for label, i in (("保持 0.3 A", 0.3), ("保持 0.6 A", 0.6), ("保持 1.0 A", 1.0), ("歩容 1.5 A", 1.5), ("ストール 2.0 A", 2.0)):
        p = i * i * rm
        t_inf = amb + p * rth
        tau = cth * rth
        # 60 ℃ に届くまでの時間（届かなければ inf）
        with np.errstate(divide="ignore", invalid="ignore"):
            frac = (lim - amb) / (t_inf - amb)
            t_reach = np.where(t_inf > lim, -tau * np.log(1.0 - np.clip(frac, 0, 0.999999)), np.inf)
        rows[label] = {"t_inf_c": pct(t_inf), "p_reaches_60c": float(np.mean(t_inf > lim)), "t_to_60c_min_p50_of_those": (float(np.median(t_reach[np.isfinite(t_reach)]) / 60.0) if np.isfinite(t_reach).any() else None),
                       "p_within_10min": float(np.mean(t_reach <= 600.0))}
    # 判断境界: 10 分以内に 60 ℃ に届く実効電流（中央の熱パラメータ）
    cth_m, rth_m, rm_m, amb_m = float(np.median(cth)), float(np.median(rth)), float(np.median(rm)), float(np.median(amb))
    grid = np.linspace(0.1, 3.0, 291)
    reach = [i for i in grid if amb_m + i * i * rm_m * rth_m > lim and (-cth_m * rth_m * np.log(1 - (lim - amb_m) / (i * i * rm_m * rth_m))) <= 600.0]
    return {"by_load": rows, "i_for_60c_within_10min_a": float(reach[0]) if reach else None, "median_params": {"c_th": cth_m, "r_th": rth_m, "r_motor": rm_m, "ambient": amb_m}}


def stop_times(rng: np.random.Generator, c: dict) -> dict:
    st = A["stop_time"]
    relay, bleed, motion = u(rng, st["relay_open_ms"]), u(rng, st["supply_bleed_ms"]), u(rng, st["servo_motion_stop_ms"])
    estop = relay + bleed + motion
    per = st["control_period_ms"]
    comm = c["link"]["heartbeat_timeout_ms"] + per + motion          # 通信断: heartbeat タイムアウト + 制御周期 + 動きが止まるまで（保持へ入る。電源は切らない）
    ttl = c["link"]["drive_ttl_ms"] + per + motion
    req = st["contact_release_requirement_ms"]
    return {"estop_to_motion_stop_ms": pct(estop), "supply_cut_ms": pct(relay + bleed), "comm_loss_to_hold_ms": pct(comm), "drive_ttl_to_hold_ms": pct(ttl),
            "p_estop_within_requirement_20ms": float(np.mean(estop <= req)), "p_supply_cut_within_20ms": float(np.mean(relay + bleed <= req)),
            "note": "20 ms は要求そのものが未確定（接触 → 脱力。docs/contact_release_requirements.md）。PC 経由では原理的に届かない。電源遮断（ESP32 に依存しない経路）だけが射程"}


def main() -> None:
    from serpens.config import load_config
    cfg = load_config()
    res: dict = {"seed": SEED, "n": N, "label": A["label"], "servo_counts": A["servo"]["count"]}
    for name, n_servo in A["servo"]["count"].items():
        rng = np.random.default_rng(SEED)
        c = currents(rng, n_servo)
        res[name] = {"currents_a": {k: pct(v) for k, v in c.items() if k != "stall1"},
                     "sag_7v4": sag(rng, c["peak_start"], A["supply"]["nominal_v"]),
                     "sag_8v4_full": sag(rng, c["peak_start"], A["supply"]["full_charge_v"]),
                     "protection": protection(rng, c), "wiring": wiring(rng, c)}
    rng = np.random.default_rng(SEED + 1)
    res["servo_temperature"] = servo_temp(rng)
    res["stop_time"] = stop_times(np.random.default_rng(SEED + 2), cfg)
    (HERE / "results").mkdir(exist_ok=True)
    (HERE / "results" / "e1_electrical.json").write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding="utf-8")

    L: list[str] = []
    P = L.append
    P("# HG-E1 電気層の判断境界（2026-10-02）\n")
    P("**ELECTRICAL_BUDGET_MC。すべて prior（ASSUMPTION）。実測 0 件（HARDWARE_VERIFIED = 0）。`ELECTRICAL_SAFETY_GATE` の 8 項目は INCOMPLETE のまま（証拠が無い）。安全・合格の語は使わない。** 再現: `simulation/hardware_gaps/HG-E1_electrical_gate/run.py`（seed 固定、N = 20,000）。入力: `assumptions.yaml`（C044 の電流は UNKNOWN。12V 品の値は使わない）。\n")
    for name in A["servo"]["count"]:
        r = res[name]
        n = A["servo"]["count"][name]
        P(f"## {name}（サーボ {n} 個）\n")
        P("| 電流 [A] | p5 | p50 | p95 | p99 | max |\n|---|---|---|---|---|---|")
        for k, lab in (("hold", "保持（合計）"), ("gait_rms", "歩容の実効（合計）"), ("gait_peak", "歩容のピーク（合計）"), ("peak_start", "同時起動・衝突のピーク"), ("all_stall", "全軸ストール（最悪）")):
            v = r["currents_a"][k]
            P(f"| {lab} | {v['p5']:.2f} | {v['p50']:.2f} | {v['p95']:.2f} | {v['p99']:.2f} | {v['max']:.2f} |")
        P("")
        for tag, key, v0 in (("7.4 V（2S 公称）", "sag_7v4", 7.4), ("8.4 V（満充電。レジスタ 14 の初期値 8.0 V を超える）", "sag_8v4_full", 8.4)):
            s = r[key]
            P(f"- **電圧降下（power_capacity）{tag}**: 最小電圧 p5 {s['v_min']['p5']:.2f} V（p50 {s['v_min']['p50']:.2f}。0 V = 電圧が保てない）、動作下限 6.0 V（ASSUMED）を下回る確率 {s['p_below_6v']:.2f}。**判断境界: 電源の全抵抗 R ≤ {s['r_total_max_for_p95_ohm']:.2f} Ω なら、p95 の最悪電流でも 6 V を保つ**（prior の R は {s['r_total_prior'][0]}〜{s['r_total_prior'][1]} Ω）。")
        p = r["protection"]
        P(f"- **過電流保護（overcurrent_protection）**: 設定電流 T（合計）での『通常で誤動作する確率 / 全軸ストールで動作しない確率』: " + "、".join(f"T = {x['trip_a']:.1f} A: {x['false_trip']:.2f} / {x['no_trip_at_all_stall']:.2f}" for x in p["table"])
          + f"。**釣り合う T ≈ {p['balanced']['trip_a']:.1f} A（誤動作 {p['balanced']['false_trip']:.2f}、不動作 {p['balanced']['no_trip_at_all_stall']:.2f}）**。通常のピーク × {p['derating']:g} が全軸ストールを下回る（= 窓が開いている）個体は {p['p_window_open_per_unit']:.2f}。{p['note']}")
        w = r["wiring"]
        P(f"- **配線・コネクタの発熱（wiring_heat）**: 幹線（全軸の電流が 1 本に流れる）の実効電流 p95 = {w['i_rms_p95_a']:.1f} A。**判断境界（配線）: ΔT ≤ 20 K に収まる実効電流は " + "、".join(f"{g} で約 {v['i_rms_for_20k_a']:.1f} A" for g, v in w["wire"].items())
          + f"**（中央の熱抵抗。束ねた被覆線。ASSUMED）。この実効電流を超える幹線は AWG24 / 22 / 20 では**足りない**（100 K 超になる確率: " + " / ".join(f"{g} {v['p_over_100k']:.2f}" for g, v in w["wire"].items()) + "）= **幹線は太い線（または分岐）が要る**。"
          f" コネクタの温度上昇 p50 {w['connector_delta_t_k']['p50']:.1f} K / p95 {w['connector_delta_t_k']['p95']:.1f} K。**判断境界（コネクタ）: 接触抵抗が " + "、".join(f"{v * 1000:.1f} mΩ 以上で ΔT ≥ {k} K" for k, v in w["connector_r_for_delta_t_ohm_at_p95_current_median_rth"].items()) + "**（熱抵抗 50 K/W、p95 の電流）。\n")
    t = res["servo_temperature"]
    P("## サーボの温度（servo_temperature）\n")
    P("| 負荷（実効電流）| 定常温度 p5 / p50 / p95 [℃] | 60 ℃ に届く確率 | 10 分以内に届く確率 |\n|---|---|---|---|")
    for lab, v in t["by_load"].items():
        P(f"| {lab} | {v['t_inf_c']['p5']:.0f} / {v['t_inf_c']['p50']:.0f} / {v['t_inf_c']['p95']:.0f} | {v['p_reaches_60c']:.2f} | {v['p_within_10min']:.2f} |")
    P(f"\n**判断境界**: 中央の熱パラメータ（C_th {t['median_params']['c_th']:.0f} J/K、R_th {t['median_params']['r_th']:.1f} K/W、巻線 {t['median_params']['r_motor']:.1f} Ω、周囲 {t['median_params']['ambient']:.0f} ℃）で、実効電流が約 "
      f"{t['i_for_60c_within_10min_a'] if t['i_for_60c_within_10min_a'] is None else format(t['i_for_60c_within_10min_a'], '.2f')} A 以上だと 10 分以内に 60 ℃（機体側の停止温度）に届く。")
    P("config の温度: 機体側 60 ℃（`link.faults.temp_limit_c`）・サーボのレジスタ 70 ℃・安全の停止 80 ℃。**サーボが自分で止まる前に機体が止める順序**（60 < 70 < 80）。\n")
    s = res["stop_time"]
    P("## 停止時間（independent_power_cut / physical_estop / real_stop_time）\n")
    P("| 経路 | p5 | p50 | p95 | max [ms] |\n|---|---|---|---|---|")
    for lab, k in (("電源の遮断（リレー + 出力の低下）", "supply_cut_ms"), ("緊急停止 → 動きの停止", "estop_to_motion_stop_ms"), ("通信断 → 保持（電源は切らない）", "comm_loss_to_hold_ms"), ("DRIVE の TTL 切れ → 保持", "drive_ttl_to_hold_ms")):
        v = s[k]
        P(f"| {lab} | {v['p5']:.0f} | {v['p50']:.0f} | {v['p95']:.0f} | {v['max']:.0f} |")
    P(f"\n20 ms 以内（要求は未確定）に届く確率: 電源の遮断 {s['p_supply_cut_within_20ms']:.2f}、緊急停止 → 動きの停止 {s['p_estop_within_requirement_20ms']:.2f}。{s['note']}\n")
    P("## 限界\n")
    P("- 電流・抵抗・熱・停止の値はすべて prior（幅）。C044 の電流は UNKNOWN。**実測（HT 項目ではなく HA-E-xx。`docs/electrical_gate_boundary.md`）で置き換える**。判断境界の位置は prior の幅に従属する。")
    P("- 回路の検証ではない（部品が未選定）。電源の遮断・E-STOP は回路の設計が先（LB-E-018）。サーボの温度は lumped の 1 次モデルで、巻線の局所の発熱・風・外装の中の放熱は入っていない。")
    P("- 結果が悪くても書く。後付けの調整はしない。")
    (HERE / "decision_boundary.md").write_text("\n".join(L) + "\n", encoding="utf-8", newline="\n")
    print("\n".join(L))


if __name__ == "__main__":
    main()
