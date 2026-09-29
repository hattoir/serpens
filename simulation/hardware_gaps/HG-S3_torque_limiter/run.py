r"""HG-S3 機械式トルクリミッター: 滑りトルクを 0.4〜1.5 N·m で掃引し、二重の安全層として評価する。

    .\.venv\Scripts\python.exe simulation\hardware_gaps\HG-S3_torque_limiter\run.py

DEC-USER-0002 §4: ソフトの上限（約 0.45 N·m。レジスタ比 0.167）+ 機械のリミッター（下限の候補 約 0.6 N·m）。
評価: 通常の移動 / 首の保持 / 障害物越え（過渡の倍率で代用）/ 子どもが引く / 指の挟み込み / 巻き付き・接触 /
      歯車の保護 / 不要な滑り / 故障モード。

歩容の負荷は HG-H0 の平面摩擦モデル、サーボの出力上限の分布は HG-H1 の prior を使う（同じ仮定）。
source = TORQUE_LIMITER_SWEEP_SIM。**実測ではない。力のしきい値は調査結果が入るまで PROVISIONAL。**
"""
from __future__ import annotations

import importlib.util
import json
import math
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT))

A = yaml.safe_load((HERE / "assumptions.yaml").read_text(encoding="utf-8"))
SOURCE = str(A["label"])
G = 9.81


def _load_h1():
    name = "hg_h1_for_s3"
    spec = importlib.util.spec_from_file_location(name, HERE.parent / "HG-H1_actuator" / "run.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def loguni(rng: np.random.Generator, lo_hi: list[float], n: int) -> np.ndarray:
    return np.exp(rng.uniform(math.log(lo_hi[0]), math.log(lo_hi[1]), n))


def thresholds() -> dict[str, Any] | None:
    p = HERE.parent / "safety_thresholds.yaml"
    return yaml.safe_load(p.read_text(encoding="utf-8")) if p.exists() else None


def run() -> dict[str, Any]:
    h1 = _load_h1()
    rng = np.random.default_rng(20260929)
    n = 3000
    pr = h1.sample_priors(rng, n)
    scs = h1.demand_scenarios()
    demand_peak = np.array([float(sc["tau"].max()) for sc in scs])                  # 各負荷の準静的なピーク（N·m）
    # 各負荷 × prior で、モーターが実際に出せる上限（電圧降下込み）と、歩容を追従できるか
    caps, track = [], []
    for sc in scs:
        ev = h1.evaluate(sc, pr)
        caps.append(ev["cap_nm"])
        track.append(ev["margin_nm"] >= 0)
    caps = np.array(caps)                                                             # (S, n)
    track = np.array(track)
    tol = np.array(A["sweep"]["slip_tolerance"])

    out: dict[str, Any] = {"nuisance": [], "pinch": [], "gear": [], "neck": [], "scenarios": len(scs)}
    for ts in A["sweep"]["slip_torque_nm"]:
        for dyn in A["sweep"]["dynamic_factor"]:
            for tl in tol:
                t_actual = ts * (1 + tl)
                need = demand_peak[:, None] * dyn                                     # (S, 1)
                trackable = track & (need <= caps + 1e-12)                            # 過渡を入れても追従できる負荷
                slips = trackable & (need > t_actual)                                 # それでもリミッターが滑る = 不要な滑り
                lost = trackable.sum() - (trackable & (need <= t_actual)).sum()
                out["nuisance"].append({"slip": ts, "dyn": dyn, "tol": float(tl),
                                        "p_nuisance": float(slips.sum() / max(trackable.sum(), 1)),
                                        "lost_scenarios": float(lost / max(trackable.sum(), 1)),
                                        "n_trackable": int(trackable.sum())})
    lever = loguni(rng, A["pinch"]["lever_m"], n)
    cap_med = np.median(caps, axis=0)                                                 # モーターの実際の上限（負荷ごとにほぼ同じ）
    stall = pr["tau_s"] * pr["v_oc"] / 7.4
    for ts in A["sweep"]["slip_torque_nm"]:
        for tl in tol:
            t_actual = ts * (1 + tl)
            row = {"slip": ts, "tol": float(tl)}
            for label, torque in (("normal", np.minimum(cap_med, t_actual)),            # ソフト上限が効いている
                                  ("software_fail", np.minimum(stall, t_actual)),      # レジスタが効かない → リミッターだけ
                                  ("both_fail", stall)):                               # 両方失敗 = リミッター無し
                f = torque / lever
                row[f"{label}_p50"], row[f"{label}_p95"], row[f"{label}_max"] = (float(np.percentile(f, 50)), float(np.percentile(f, 95)),
                                                                                  float(f.max()))
            out["pinch"].append(row)
    gear = loguni(rng, A["gear"]["break_torque_nm"], n)
    ext = (rng.uniform(*A["external"]["pull_n"], n) * rng.uniform(*A["external"]["lever_m"], n))
    lift = rng.uniform(*A["neck"]["lifted_by_child_nm"], n)
    hold = (rng.uniform(*A["neck"]["hold_mass_g"], n) / 1000.0 * G * rng.uniform(*A["neck"]["hold_cog_mm"], n) / 1000.0
            * float(A["neck"]["hold_dynamic"]))
    m = float(A["gear"]["protect_margin"])
    for ts in A["sweep"]["slip_torque_nm"]:
        for tl in tol:
            t_actual = ts * (1 + tl)
            out["gear"].append({"slip": ts, "tol": float(tl),
                                "p_protect_design": float(np.mean(t_actual * m <= gear)),          # 設計として歯車を守れるか
                                "p_damage_pull": float(np.mean(np.minimum(ext, t_actual) > gear)),
                                "p_damage_pull_no_clutch": float(np.mean(ext > gear)),
                                "p_damage_lift": float(np.mean(np.minimum(lift, t_actual) > gear)),
                                "p_damage_lift_no_clutch": float(np.mean(lift > gear))})
            out["neck"].append({"slip": ts, "tol": float(tl), "hold_p95_nm": float(np.percentile(hold, 95)),
                                "hold_margin_p05": float(np.percentile(t_actual / hold, 5)),
                                "p_head_slips_under_hold": float(np.mean(hold * float(A["criteria"]["min_hold_margin"]) > t_actual))})
    return out


def write(out: dict[str, Any]) -> None:
    c = A["criteria"]
    th = thresholds()
    L = [f"# HG-S3 機械式トルクリミッター — 滑りトルクの窓（自動生成 {datetime.now():%Y-%m-%d %H:%M}。source = {SOURCE}）\n",
         f"DEC-USER-0002 §4: ソフト上限 約 0.45 N·m（レジスタ比 0.167）+ 機械のリミッター。滑りトルクの下限の候補は約 0.6 N·m。"
         f"歩容の負荷 {out['scenarios']} 通り（HG-H0 の平面摩擦モデル）× サーボの prior 3000 点（HG-H1）。\n",
         "**力のしきい値は " + ("`safety_thresholds.yaml` を読み込んだ" if th else "まだ無い（調査中）。PROVISIONAL / SAFETY_UNVERIFIED。合否は出さず、力の値を並べる") + "。**\n",
         "**配置の前提**: リミッターは**サーボ出力とリンクの間（出力側）**に置く。入力側だと外力が歯車を通り、保護にならない。\n",
         "## 1. 不要な滑り（追従できたはずの負荷で滑る割合。過渡の倍率 × 個体差）\n",
         f"基準（ASSUMPTION）: ≤ {c['max_nuisance_slip']:.0%}。歩容の必要トルクは μ に比例（HG-H0/H1）。\n",
         "| 公称の滑りトルク N·m | " + " | ".join(f"過渡 ×{d:g} / 個体差 {t:+.0%}" for d in A["sweep"]["dynamic_factor"] for t in A["sweep"]["slip_tolerance"]) + " |",
         "|---|" + "---|" * (len(A["sweep"]["dynamic_factor"]) * len(A["sweep"]["slip_tolerance"]))]
    for ts in A["sweep"]["slip_torque_nm"]:
        cells = []
        for d in A["sweep"]["dynamic_factor"]:
            for t in A["sweep"]["slip_tolerance"]:
                r = next(x for x in out["nuisance"] if x["slip"] == ts and x["dyn"] == d and abs(x["tol"] - t) < 1e-9)
                cells.append(f"{r['p_nuisance']:.0%}")
        L.append(f"| {ts:g} | " + " | ".join(cells) + " |")
    L.append("\n## 2. 挟み込み・巻き付きの力（関節から 20〜95 mm で押す点。N）\n")
    L.append("normal = ソフト上限が効いている、software_fail = レジスタが効かずリミッターだけが守る、both_fail = リミッターも効かない（= リミッター無し）。\n")
    L.append("| 滑りトルク | 個体差 | normal p95 | software_fail p50 | software_fail p95 | software_fail 最大 | both_fail p95 | both_fail 最大 |\n|---|---|---|---|---|---|---|---|")
    for r in out["pinch"]:
        if r["tol"] in (0.0, 0.3):
            L.append(f"| {r['slip']:g} | {r['tol']:+.0%} | {r['normal_p95']:.0f} | {r['software_fail_p50']:.0f} | {r['software_fail_p95']:.0f} | "
                     f"{r['software_fail_max']:.0f} | {r['both_fail_p95']:.0f} | {r['both_fail_max']:.0f} |")
    probe = A["threshold_probe_n"]
    L.append("\n### 力のしきい値が X N だとしたら（software_fail の p95 が X 以下になる滑りトルク。個体差 +30% を見込む）\n")
    L.append("| しきい値 N | " + " | ".join(f"{ts:g}" for ts in A["sweep"]["slip_torque_nm"]) + " |\n|---|" + "---|" * len(A["sweep"]["slip_torque_nm"]))
    for x in probe:
        cells = []
        for ts in A["sweep"]["slip_torque_nm"]:
            r = next(y for y in out["pinch"] if y["slip"] == ts and abs(y["tol"] - 0.3) < 1e-9)
            cells.append("○" if r["software_fail_p95"] <= x else "×")
        L.append(f"| {x:g} | " + " | ".join(cells) + " |")
    if th:
        forces = th["quasi_static_contact_force_n"]
        levers = [0.020, 0.0475, 0.095]
        L.append("\n### 暫定しきい値（`safety_thresholds.yaml`、PROVISIONAL / SAFETY_UNVERIFIED）を満たすために許される滑りトルクの上限（N·m = しきい値 × 腕の長さ）\n")
        L.append("| 部位 | しきい値 baseline N | 関節から 20 mm | 47.5 mm（節の中点） | 95 mm（隣の節の先） |\n|---|---|---|---|---|")
        for region, v in forces.items():
            f = float(v["baseline"])
            L.append(f"| {region} | {f:g} | " + " | ".join(f"{f * lv:.2f}" for lv in levers) + " |")
        need = min(0.13, 0.69)
        L.append(f"\n**読み**: 歩容が必要とする関節トルクは 0.13〜0.69 N·m（HG-H1、μ 0.2〜0.8 × 比 2〜8）。"
                 "手・指のしきい値（5.7 N）を関節から 20 mm の点で満たすには **0.11 N·m 以下**、節の中点でも 0.27 N·m 以下 — "
                 "**トルクの制限だけでは、しきい値と移動を同時には満たせない**（0.5〜1.0 N·m のリミッターでは 20 mm の点で 25〜50 N）。"
                 "したがって指の挟み込みは、**トルクではなく幾何で守る**: 関節から 20〜60 mm 以内の人が触れる範囲に 5〜12 mm の隙間を作らない"
                 "（`geometry_rules.accessible_gap_forbidden_mm`）、外装を柔らかく力を逃がす形にする（mechanically forgiving）。"
                 "リミッターの役目は、しきい値の達成ではなく **ソフトの故障時に力を「p95 で 119 N（最大 164 N）」から「0.7 N·m のリミッターで p95 33〜42 N」へ下げる**こと（§2 の software_fail）と、歯車の保護。\n")
    L.append("\n## 3. 歯車の保護と、子どもが引く / 持ち上げる\n")
    L.append("歯車の破断トルクは UNKNOWN（prior 2〜8 N·m の対数一様、ASSUMPTION）。設計として守る条件: 滑りトルク × 1.5 ≤ 破断トルク。\n")
    L.append("| 滑りトルク | 個体差 | 設計で歯車を守れる確率 | 引く: 歯車が壊れる確率 | （リミッター無し） | 持ち上げ: 壊れる確率 | （リミッター無し） |\n|---|---|---|---|---|---|---|")
    for r in out["gear"]:
        if r["tol"] in (0.0, 0.3):
            L.append(f"| {r['slip']:g} | {r['tol']:+.0%} | {r['p_protect_design']:.0%} | {r['p_damage_pull']:.0%} | {r['p_damage_pull_no_clutch']:.0%} | "
                     f"{r['p_damage_lift']:.0%} | {r['p_damage_lift_no_clutch']:.0%} |")
    L.append("\n## 4. 首（J1）の保持\n")
    L.append("| 滑りトルク | 個体差 | 保持トルク p95 N·m（過渡 ×2） | 余裕の p05 | 保持中に頭が滑る確率（余裕 1.5） |\n|---|---|---|---|---|")
    for r in out["neck"]:
        if r["tol"] in (-0.3, 0.0):
            L.append(f"| {r['slip']:g} | {r['tol']:+.0%} | {r['hold_p95_nm']:.2f} | {r['hold_margin_p05']:.1f} | {r['p_head_slips_under_hold']:.0%} |")
    # 窓
    L.append("\n## 5. 滑りトルクの窓（提案。**正式 Decision ではない**）\n")
    ok_nuis = sorted({r["slip"] for r in out["nuisance"] if r["dyn"] == 1.5 and abs(r["tol"] + 0.3) < 1e-9 and r["p_nuisance"] <= c["max_nuisance_slip"]})
    ok_gear = sorted({r["slip"] for r in out["gear"] if abs(r["tol"] - 0.3) < 1e-9 and r["p_protect_design"] >= c["min_gear_protect"]})
    ok_neck = sorted({r["slip"] for r in out["neck"] if abs(r["tol"] + 0.3) < 1e-9 and r["p_head_slips_under_hold"] == 0.0})
    L.append(f"- 不要な滑り ≤ {c['max_nuisance_slip']:.0%}（過渡 ×1.5、個体差 −30%）を満たす: {ok_nuis}")
    L.append(f"- 歯車を守れる確率 ≥ {c['min_gear_protect']:.0%}（個体差 +30%）を満たす: {ok_gear}")
    L.append(f"- 首の保持で滑らない（余裕 1.5、個体差 −30%）: {ok_neck}")
    both = sorted(set(ok_nuis) & set(ok_gear) & set(ok_neck))
    L.append(f"- **すべてを満たす: {both if both else '無い → 歯車の破断トルクを実測・確認するか、不要な滑りの基準を見直す'}**")
    L.append("\n下限は「不要な滑り」と「首の保持」、上限は「歯車の保護」と「力のしきい値」で決まる。"
             "歯車の破断トルクが未知なので、**上限側は H1 と S4（歯車の資料確認 / 予備個体の破壊試験）まで確定しない**。\n")
    L.append("## 6. 故障モード\n")
    L.append("| 故障 | 起きること | 対策・確認 |\n|---|---|---|\n"
             "| レジスタが効かない（設定漏れ・ファームの不具合） | モーターはストール（1.4〜3 N·m）まで出す。**リミッターだけが守る** → §2 の software_fail の力 | ソフトとリミッターを独立にする。H1 で両方の実効トルクを測る |\n"
             "| リミッターが固着（滑らない） | リミッター無しと同じ（both_fail） | 固着を検知できるか（滑りの有無）。定期の確認 |\n"
             "| 滑りトルクの劣化・温度・摩耗 | 個体差 ±30% を見込んだ（§1〜§4） | 耐久試験で変化を測る |\n"
             "| 保持中に滑る | 頭が落ちる（J1） | §4。頭の重りを模した試験 |\n"
             "| スティックスリップ（ガタつき） | 歩容の追従が乱れる。異音 | 実物で確認（**MODEL GAP**: 静的な滑りトルクしか入れていない） |\n"
             "| 滑ったまま位置がずれる | 関節角のエンコーダとホーンがずれる | 復帰の手順（原点合わせ）が要る。サーボ側の角度とリンク側の角度が食い違う |\n")
    (HERE / "decision_boundary.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    (HERE / "results").mkdir(exist_ok=True)
    (HERE / "results" / "s3_sweep.json").write_text(json.dumps(out, indent=1), encoding="utf-8")


def main() -> int:
    out = run()
    write(out)
    print("decision_boundary.md を書いた:", HERE / "decision_boundary.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
