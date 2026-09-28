r"""HG-H0 摩擦の Hardware Gap: 実測を待つあいだの掃引・感度・Monte Carlo と、実測 CSV の取り込み。

    .\.venv\Scripts\python.exe simulation\hardware_gaps\HG-H0_friction\run.py            # 事前分布での解析
    .\.venv\Scripts\python.exe simulation\hardware_gaps\HG-H0_friction\run.py --quick    # 動作確認（小さい掃引）
    .\.venv\Scripts\python.exe simulation\hardware_gaps\HG-H0_friction\run.py --measured hardware\prototypes\H0_friction\h0_friction_YYYYMMDD.csv

モデルは `simulation/planar_friction.py`（PLANAR_FRICTION_SIM）。**結果は HARDWARE_VERIFIED にしない。**
実測した摩擦係数そのものは、クーポン × 床の組み合わせに限って HARDWARE_VERIFIED（記録に日付・条件・道具があること）。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT))

from serpens.config import load_config  # noqa: E402
from simulation.planar_friction import SOURCE, Friction, PlanarSnake, body_from_cfg  # noqa: E402

ASSUME = yaml.safe_load((HERE / "assumptions.yaml").read_text(encoding="utf-8"))
SWEEP = yaml.safe_load((HERE / "sweep_config.yaml").read_text(encoding="utf-8"))
CRIT = ASSUME["criteria"]
FREQ = float(ASSUME["gait"]["freq_hz"])
DECISION_CONFIGS = ("FW6_HEADYAW", "FW6_YAW5")          # 6 本目の用途の比較（User 指示）
WHEEL_KEY = "WHEEL_FW5"

_BODIES: dict[str, Any] = {}


def _body(name: str):
    if name not in _BODIES:
        cfg = load_config(overlay=ROOT / ASSUME["configs"][name]["overlay"])
        _BODIES[name] = body_from_cfg(cfg, name)
    return _BODIES[name]


# ---- 1 条件の評価（ワーカー） --------------------------------------------------------------------
@dataclass(frozen=True)
class Case:
    config: str
    fr: dict[str, Any]
    amplitude_deg: float
    waves: float


def evaluate(case: Case) -> dict[str, Any]:
    """1 つの歩容で 直進 + 左右の最大旋回 を解き、合否の材料を返す。"""
    body = _body(case.config)
    fr = Friction(**case.fr)
    sim = PlanarSnake(body, fr, int(SWEEP["points_per_segment"]))
    steps = int(SWEEP["steps_per_cycle"])
    a = case.amplitude_deg
    gamma = max(float(CRIT["joint_limit_deg"]) - a, 0.0)      # 振幅 + γ ≤ 可動域
    st = sim.run_cycle(a, case.waves, FREQ, 0.0, steps)
    out: dict[str, Any] = {"config": case.config, **case.fr, "amplitude_deg": a, "waves": case.waves,
                           "gamma_avail_deg": gamma, "speed_mm_s": st.speed_mm_s,
                           "drift_deg_per_cycle": st.heading_deg_per_cycle, "slip": st.slip,
                           "energy_j_per_m": st.energy_j_per_m, "converged": st.converged}
    peak = st.peak_torque_nm
    turn_rates, radii = [], []
    for g in ((gamma, -gamma) if gamma > 0 else ()):
        tr = sim.run_cycle(a, case.waves, FREQ, g, steps)
        out["converged"] = out["converged"] and tr.converged
        peak = max(peak, tr.peak_torque_nm)
        dh = tr.heading_deg_per_cycle - st.heading_deg_per_cycle
        turn_rates.append(abs(dh))
        step_m = max(abs(tr.speed_mm_s) / FREQ / 1000.0, 1e-6)
        radii.append(step_m / (2.0 * abs(math.sin(math.radians(dh) / 2.0))) * 1000.0 if abs(dh) > 1e-3 else math.inf)
    out["turn_authority_deg_per_cycle"] = min(turn_rates) if turn_rates else 0.0
    out["turn_radius_mm"] = max(radii) if radii else math.inf          # 悪い側（大きい方）
    out["peak_torque_nm"] = peak
    out["torque_per_mu_f"] = peak / fr.mu_f
    return out


def passes(row: dict[str, Any], mu_forward: float | None = None, torque_scale: float = 1.0) -> dict[str, bool]:
    """合否の内訳。torque_scale はトルク上限の誤差（HG-H1 の ±20 / 40%）。"""
    mu_f = row["mu_f"] if mu_forward is None else mu_forward
    drift_ok = abs(row["drift_deg_per_cycle"]) <= float(CRIT["drift_authority"]) * row["turn_authority_deg_per_cycle"]
    return {
        "speed": row["speed_mm_s"] >= float(CRIT["min_speed_mm_s"]),
        "turn": row["turn_radius_mm"] <= float(CRIT["max_turn_radius_mm"]),
        "drift": bool(drift_ok),
        "torque": row["torque_per_mu_f"] * mu_f <= float(CRIT["torque_margin"]) * float(CRIT["software_torque_limit_nm"]) * torque_scale,
        "converged": bool(row["converged"]),
    }


def capability(rows: list[dict[str, Any]], **kw: Any) -> dict[str, Any]:
    """同じ構成・摩擦での**能力**。直進と旋回は別の歩容を使ってよい（実機でも旋回時は振幅を下げて γ を増やせる）。

      直進: トルク・ずれ（その歩容の旋回余力で直せる）を満たす歩容のうち最速
      旋回: トルクを満たす歩容のうち最小の旋回半径
    2026-09-29 の初回は「1 つの歩容で直進も旋回も」を要求していて、旋回の基準が過度に厳しかった。
    """
    p = [(r, passes(r, **kw)) for r in rows]
    straight = [r for r, q in p if q["torque"] and q["drift"] and q["converged"]]
    turning = [r for r, q in p if q["torque"] and q["converged"]]
    best = max(straight, key=lambda r: r["speed_mm_s"]) if straight else max(rows, key=lambda r: r["speed_mm_s"])
    radius = min((r["turn_radius_mm"] for r in turning), default=math.inf)
    ok_speed = bool(straight) and best["speed_mm_s"] >= float(CRIT["min_speed_mm_s"])
    ok_turn = radius <= float(CRIT["max_turn_radius_mm"])
    fail = []
    if not turning:
        fail.append("torque")                                   # どの歩容もトルク上限を超える
    elif not ok_speed:
        fast = [r for r in turning if r["speed_mm_s"] >= float(CRIT["min_speed_mm_s"])]
        fail.append("drift" if fast else "speed")               # 速い歩容はあるが、ずれを直せない / そもそも遅い
    if turning and not ok_turn:
        fail.append("turn")
    return {"row": best, "speed_mm_s": best["speed_mm_s"], "turn_radius_mm": radius,
            "ok_speed": ok_speed, "ok_turn": ok_turn, "ok": ok_speed and ok_turn, "fail": fail}


def best_passing(rows: list[dict[str, Any]], **kw: Any) -> tuple[dict[str, Any], bool]:
    """（互換）能力の判定を (直進の代表歩容, 合格か) で返す。"""
    c = capability(rows, **kw)
    return c["row"], c["ok"]


def run_cases(cases: list[Case], workers: int) -> list[dict[str, Any]]:
    if workers <= 1:
        return [evaluate(c) for c in cases]
    with ProcessPoolExecutor(max_workers=workers) as ex:
        return list(ex.map(evaluate, cases, chunksize=8))


def group_by(rows: list[dict[str, Any]], keys: tuple[str, ...]) -> dict[tuple, list[dict[str, Any]]]:
    out: dict[tuple, list[dict[str, Any]]] = {}
    for r in rows:
        out.setdefault(tuple(r[k] for k in keys), []).append(r)
    return out


# ---- S1: 比の掃引 ---------------------------------------------------------------------------------
def s1_ratio_sweep(workers: int, quick: bool) -> list[dict[str, Any]]:
    rs = SWEEP["ratio_sweep"]
    rr = rs["ratios"]
    n = 8 if quick else int(rr["n"])
    ratios = np.geomspace(float(rr["log_min"]), float(rr["log_max"]), n)
    amps = rs["amplitude_deg"][-1:] if quick else rs["amplitude_deg"]
    waves = rs["waves"][:2] if quick else rs["waves"]
    cases = [Case(c, {"mu_f": 0.2, "mu_b": 0.2, "mu_left": 0.2 * r, "mu_right": 0.2 * r, "law": law}, float(a), float(w))
             for law in ASSUME["laws"] for c in ASSUME["configs"] for r in ratios for a in amps for w in waves]
    rows = run_cases(cases, workers)
    for row in rows:
        row["ratio"] = row["mu_left"] / row["mu_f"]
    return rows


def s1_boundaries(rows: list[dict[str, Any]]) -> dict[str, dict[str, float | None]]:
    """構成 × 法則ごとに、速さ・旋回・ずれを満たす最小の比（トルクは μ_forward しだいなので別）。"""
    out: dict[str, dict[str, float | None]] = {}
    for (law, cfg), grp in group_by(rows, ("law", "config")).items():
        best = {}
        for ratio, g in group_by(grp, ("ratio",)).items():
            best[ratio[0]] = best_passing(g, mu_forward=0.0)          # μ_forward=0 → トルク条件を外す
        passing = sorted(r for r, (_, ok) in best.items() if ok)
        speed_only = sorted(r for r, (row, _) in best.items() if row["speed_mm_s"] >= float(CRIT["min_speed_mm_s"]))
        out.setdefault(law, {})[cfg] = passing[0] if passing else None
        out[law][cfg + ":speed_only"] = speed_only[0] if speed_only else None
    return out


# ---- S2: 感度 ------------------------------------------------------------------------------------
def s2_sensitivity(workers: int, quick: bool) -> list[dict[str, Any]]:
    sv = SWEEP["sensitivity"]
    rng = np.random.default_rng(int(SWEEP["seed"]))
    cases: list[Case] = []
    tags: list[dict[str, Any]] = []
    n_pts = {c: len(_body(c).seg_len_m) * int(SWEEP["points_per_segment"]) for c in DECISION_CONFIGS}
    ratios = sv["ratio_ref"][:1] if quick else sv["ratio_ref"]
    for law in ASSUME["laws"]:
        for c in DECISION_CONFIGS:
            for r in ratios:
                def add(param: str, value: float, fr: dict[str, Any]) -> None:
                    for a in (30.0, 40.0):
                        for w in (0.75, 1.0):
                            cases.append(Case(c, fr, a, w))
                            tags.append({"param": param, "value": value, "ratio_ref": r})
                base = {"mu_f": 0.2, "mu_b": 0.2, "mu_left": 0.2 * r, "mu_right": 0.2 * r, "law": law}
                for a_lr in sv["lr_asym"]:
                    add("lr_asym", a_lr, {**base, "mu_left": 0.2 * r * (1 + a_lr), "mu_right": 0.2 * r * (1 - a_lr)})
                for fb in sv["fb_ratio"]:
                    add("fb_ratio", fb, {**base, "mu_b": 0.2 * fb})
                for sg in sv["point_sigma"]:
                    for s in range(2 if quick else int(sv["point_sigma_seeds"])):
                        scale = tuple(float(x) for x in np.exp(rng.normal(0.0, sg, n_pts[c])))
                        add("point_sigma", sg, {**base, "point_scale": scale})
    rows = run_cases(cases, workers)
    for row, tag in zip(rows, tags):
        row.update(tag)
        row.pop("point_scale", None)
    return rows


# ---- S3: 床ごとの Monte Carlo ---------------------------------------------------------------------
def _sample_snake(rng: np.random.Generator, floor: dict[str, Any], n_pts: int, law: str) -> dict[str, Any]:
    mu_f = rng.uniform(*floor["mu_forward"])
    ratio = math.exp(rng.uniform(math.log(floor["ratio"][0]), math.log(floor["ratio"][1])))
    ks = 1.0 + rng.uniform(0.0, floor["ks_spread"], 2)        # 静止 → 滑りで方向ごとに違う倍率
    mu_f_dyn, mu_l_dyn = mu_f / ks[0], mu_f * ratio / ks[1]
    a = rng.uniform(-floor["lr_asym"], floor["lr_asym"])
    fb = rng.uniform(*floor["fb_ratio"])
    scale = tuple(float(x) for x in np.exp(rng.normal(0.0, floor["point_sigma"], n_pts)))
    return {"mu_f": mu_f_dyn, "mu_b": mu_f_dyn * fb, "mu_left": mu_l_dyn * (1 + a), "mu_right": mu_l_dyn * (1 - a),
            "law": law, "point_scale": scale}


def s3_monte_carlo(workers: int, quick: bool) -> list[dict[str, Any]]:
    mc = SWEEP["monte_carlo"]
    rng = np.random.default_rng(int(SWEEP["seed"]) + 1)
    n = 6 if quick else int(mc["samples_per_floor"])
    laws, p_law = list(ASSUME["laws"]), list(ASSUME["laws"].values())
    cases: list[Case] = []
    tags: list[dict[str, Any]] = []
    for floor_id, floor in ASSUME["floors"].items():
        wheel = ASSUME["wheel"][floor_id]
        for i in range(n):
            law = str(rng.choice(laws, p=p_law))
            for c in ASSUME["configs"]:
                n_pts = len(_body(c).seg_len_m) * int(SWEEP["points_per_segment"])
                fr = _sample_snake(np.random.default_rng([int(SWEEP["seed"]), i, hash(floor_id) % 997]), floor, n_pts, law)
                for a in mc["amplitude_deg"]:
                    for w in mc["waves"]:
                        cases.append(Case(c, fr, float(a), float(w)))
                        tags.append({"floor": floor_id, "sample": i, "belly": "SNAKE", "sampled_law": law})
            mr = rng.uniform(*wheel["mu_roll"])
            ms = rng.uniform(*wheel["mu_side"])
            geom = ASSUME["wheel"]["geometry"]
            fr = {"mu_f": mr, "mu_b": mr, "mu_left": ms, "mu_right": ms, "law": "decoupled"}
            for a in mc["amplitude_deg"]:
                for w in mc["waves"]:
                    cases.append(Case(geom, fr, float(a), float(w)))
                    tags.append({"floor": floor_id, "sample": i, "belly": "WHEEL", "sampled_law": "decoupled"})
    rows = run_cases(cases, workers)
    for row, tag in zip(rows, tags):
        row.update(tag)
        row.pop("point_scale", None)
    return rows


def s3_decisions(rows: list[dict[str, Any]], torque_scale: float = 1.0) -> list[dict[str, Any]]:
    """サンプルごとに構成の合否を出し、6 本目の読み（HEAD_YAW_OK / BODY_YAW_NEEDED / WHEEL / NONE）を付ける。"""
    out = []
    for (floor, sample), grp in group_by(rows, ("floor", "sample")).items():
        ok: dict[str, bool] = {}
        fails: dict[str, str] = {}
        for (belly, cfg), g in group_by(grp, ("belly", "config")).items():
            cap = capability(g, torque_scale=torque_scale)
            key = WHEEL_KEY if belly == "WHEEL" else cfg
            ok[key] = cap["ok"]
            fails[key] = "+".join(cap["fail"]) or "-"
        if ok.get("FW6_HEADYAW"):
            reading = "HEAD_YAW_OK"
        elif ok.get("FW6_YAW5"):
            reading = "BODY_YAW_NEEDED"
        elif ok.get(WHEEL_KEY):
            reading = "WHEEL_FALLBACK"
        else:
            reading = "NONE"
        law = next(r["sampled_law"] for r in grp if r["belly"] == "SNAKE")
        out.append({"floor": floor, "sample": sample, "law": law, "reading": reading, "torque_scale": torque_scale,
                    **{f"ok_{k}": v for k, v in ok.items()}, **{f"fail_{k}": v for k, v in fails.items()}})
    return out


# ---- 出力 ----------------------------------------------------------------------------------------
def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = sorted({k for r in rows for k in r})
    with path.open("w", newline="", encoding="utf-8") as fp:
        w = csv.DictWriter(fp, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def plots(s1: list[dict[str, Any]], s3d: list[dict[str, Any]], s2: list[dict[str, Any]], outdir: Path) -> list[str]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    outdir.mkdir(parents=True, exist_ok=True)
    made = []
    # 前進の速さ vs 比
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), sharey=True)
    for ax, law in zip(axes, ASSUME["laws"]):
        for cfg in ASSUME["configs"]:
            pts = sorted((k[0], best_passing(g, mu_forward=0.0)[0]["speed_mm_s"])
                         for k, g in group_by([r for r in s1 if r["law"] == law and r["config"] == cfg], ("ratio",)).items())
            ax.plot([p[0] for p in pts], [p[1] for p in pts], marker="o", ms=3, label=cfg)
        ax.axhline(CRIT["min_speed_mm_s"], color="k", ls="--", lw=1)
        ax.set_xscale("log")
        ax.set_title(f"law = {law}")
        ax.set_xlabel("mu_lateral / mu_forward")
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("forward speed mm/s (0.5 Hz, best gait)")
    axes[1].legend(fontsize=8)
    fig.suptitle(f"{SOURCE} - not hardware data")
    fig.tight_layout()
    p = outdir / "speed_vs_ratio.png"
    fig.savefig(p, dpi=120)
    plt.close(fig)
    made.append(p.name)
    # 床ごとの読み
    floors = list(ASSUME["floors"])
    readings = ["HEAD_YAW_OK", "BODY_YAW_NEEDED", "WHEEL_FALLBACK", "NONE"]
    fig, ax = plt.subplots(figsize=(7, 4))
    bottom = np.zeros(len(floors))
    for rd in readings:
        v = np.array([np.mean([d["reading"] == rd for d in s3d if d["floor"] == f]) for f in floors])
        ax.bar(floors, v, bottom=bottom, label=rd)
        bottom += v
    ax.set_ylabel("fraction of Monte Carlo samples")
    ax.set_title(f"reading per floor (prior ranges, {SOURCE})")
    ax.legend(fontsize=8)
    fig.tight_layout()
    p = outdir / "mc_reading_per_floor.png"
    fig.savefig(p, dpi=120)
    plt.close(fig)
    made.append(p.name)
    # 感度（速さの相対変化）
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.8))
    for ax, param in zip(axes, ("lr_asym", "fb_ratio", "point_sigma")):
        for cfg in DECISION_CONFIGS:
            for law in ASSUME["laws"]:
                g = [r for r in s2 if r["param"] == param and r["config"] == cfg and r["law"] == law
                     and r["ratio_ref"] == SWEEP["sensitivity"]["ratio_ref"][0]]
                xs = sorted({r["value"] for r in g})
                ys = [np.mean([max(rr["speed_mm_s"] for rr in g if rr["value"] == x and rr.get("amplitude_deg") == a)
                               for a in (30.0, 40.0)]) for x in xs]
                ax.plot(xs, ys, marker="o", ms=3, label=f"{cfg}/{law}")
        ax.set_title(param)
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("speed mm/s")
    axes[2].legend(fontsize=7)
    fig.tight_layout()
    p = outdir / "sensitivity.png"
    fig.savefig(p, dpi=120)
    plt.close(fig)
    made.append(p.name)
    return made


def drift_table(s2: list[dict[str, Any]]) -> list[tuple]:
    out = []
    for (cfg, law, r), g in group_by([x for x in s2 if x["param"] == "lr_asym"], ("config", "law", "ratio_ref")).items():
        for v in sorted({x["value"] for x in g}):
            gg = [x for x in g if x["value"] == v]
            best, ok = best_passing(gg, mu_forward=0.0)
            out.append((cfg, law, r, v, best["drift_deg_per_cycle"], best["turn_authority_deg_per_cycle"], ok))
    return out


def write_decision_md(s1b: dict, s1: list[dict[str, Any]], s3d: list[dict[str, Any]], s2: list[dict[str, Any]],
                      plots_made: list[str], stamp: str, s3_rows: list[dict[str, Any]] | None = None) -> None:
    L = [f"# HG-H0 摩擦 — 判定の境界（自動生成 {stamp}。source = {SOURCE}。**実測ではない**）\n",
         "`run.py` が書き出す。手で直さない（`hardware_test_plan.md` と `README.md` は手で書く）。\n",
         f"基準（ASSUMPTION）: 前進 ≥ {CRIT['min_speed_mm_s']:.0f} mm/s、旋回半径 ≤ {CRIT['max_turn_radius_mm']:.0f} mm"
         f"（振幅 + γ ≤ {CRIT['joint_limit_deg']:.0f}°）、直進のずれ ≤ 旋回の余力 × {CRIT['drift_authority']}、"
         f"トルク ≤ {CRIT['software_torque_limit_nm']} N·m × {CRIT['torque_margin']}。歩容は構成・摩擦ごとに選び直す。0.5 Hz。\n",
         "## 1. 比 r = μ_横 / μ_前 の境界（トルク以外の基準をすべて満たす最小の r）\n",
         "準静的クーロン摩擦では、動きは比だけで決まる（全方向の μ を同じ倍率にしても同じ。2026-09-29 に数値で確認）。\n",
         "| 構成 | " + " | ".join(f"{law}（全基準）| {law}（速さだけ）" for law in ASSUME["laws"]) + " |",
         "|---|" + "---|---|" * len(ASSUME["laws"])]
    for cfg in ASSUME["configs"]:
        cells = []
        for law in ASSUME["laws"]:
            for key in (cfg, cfg + ":speed_only"):
                v = s1b.get(law, {}).get(key)
                cells.append(f"{v:.2f}" if v else "> 80")
        L.append(f"| {cfg} | " + " | ".join(cells) + " |")
    L.append("")
    for law in ASSUME["laws"]:
        hb, ha = s1b[law].get("FW6_HEADYAW"), s1b[law].get("FW6_YAW5")
        fmt = lambda v: f"{v:.2f}" if v else "> 80"
        L.append(f"- **{law}**: r ≥ {fmt(hb)} なら Head Yaw（FW6_HEADYAW）で足りる。{fmt(ha)} ≤ r < {fmt(hb)} は Body Yaw（FW6_YAW5）が要る。"
                 f"r < {fmt(ha)} は 6 モーターの Pure Snake では不足 → Wheel Belly（受動輪で r を大きくする）")
    # トルクの境界
    L.append("\n## 2. トルクの境界（μ_前 の上限。トルクは μ に比例）\n")
    L.append("| 構成 | 法則 | r=2 | r=4 | r=8 |\n|---|---|---|---|---|")
    for law in ASSUME["laws"]:
        for cfg in DECISION_CONFIGS:
            cells = []
            for target in (2.0, 4.0, 8.0):
                grp = [r for r in s1 if r["law"] == law and r["config"] == cfg]
                rk = min({r["ratio"] for r in grp}, key=lambda x: abs(math.log(x / target)))
                best, _ = best_passing([r for r in grp if r["ratio"] == rk], mu_forward=0.0)
                cells.append(f"μ_前 ≤ {CRIT['torque_margin'] * CRIT['software_torque_limit_nm'] / best['torque_per_mu_f']:.2f}")
            L.append(f"| {cfg} | {law} | " + " | ".join(cells) + " |")
    # Monte Carlo
    L.append("\n## 3. 床ごとの Monte Carlo（事前分布は `assumptions.yaml`。**探索用の幅で、床の実測ではない**）\n")
    L.append("| 床 | 法則 | n | HEAD_YAW_OK | BODY_YAW_NEEDED | WHEEL_FALLBACK | NONE |\n|---|---|---|---|---|---|---|")
    for floor in ASSUME["floors"]:
        for law in list(ASSUME["laws"]) + ["(両方)"]:
            g = [d for d in s3d if d["floor"] == floor and (law == "(両方)" or d["law"] == law)]
            if not g:
                continue
            frac = {rd: np.mean([d["reading"] == rd for d in g]) for rd in ("HEAD_YAW_OK", "BODY_YAW_NEEDED", "WHEEL_FALLBACK", "NONE")}
            L.append(f"| {floor} | {law} | {len(g)} | " + " | ".join(f"{v:.0%}" for v in frac.values()) + " |")
    L.append("\n### 不合格の理由（サンプル数。torque = どの歩容も上限超え / speed = 遅い / drift = 速い歩容はあるがずれを直せない / turn = 旋回半径）\n")
    L.append("| 床 | 構成 | 合格 | torque | speed | drift | turn |\n|---|---|---|---|---|---|---|")
    for floor in ASSUME["floors"]:
        for key in list(ASSUME["configs"]) + [WHEEL_KEY]:
            g = [d for d in s3d if d["floor"] == floor and f"fail_{key}" in d]
            if not g:
                continue
            c = lambda k: sum(k in str(d[f"fail_{key}"]).split("+") for d in g)
            L.append(f"| {floor} | {key} | {sum(bool(d[f'ok_{key}']) for d in g)}/{len(g)} | {c('torque')} | {c('speed')} | {c('drift')} | {c('turn')} |")
    if s3_rows is not None:
        L.append("\n### トルク上限の誤差（HG-H1）への感度: 各床で「6 本目の読み」の割合\n")
        L.append("| 床 | 上限 × | HEAD_YAW_OK | BODY_YAW_NEEDED | WHEEL_FALLBACK | NONE |\n|---|---|---|---|---|---|")
        for ts in (0.6, 0.8, 1.0, 1.2, 1.4):
            dd = s3_decisions(s3_rows, torque_scale=ts)
            for floor in ASSUME["floors"]:
                g = [d for d in dd if d["floor"] == floor]
                L.append(f"| {floor} | {ts:g} | " + " | ".join(f"{np.mean([d['reading'] == rd for d in g]):.0%}"
                         for rd in ("HEAD_YAW_OK", "BODY_YAW_NEEDED", "WHEEL_FALLBACK", "NONE")) + " |")
    # 感度
    L.append("\n## 4. 感度（r = 基準値のまわり。速さの変化）\n")
    L.append("| 構成 | 法則 | r | パラメータ | 値 → 前進 mm/s |\n|---|---|---|---|---|")
    for (cfg, law, r, param), g in sorted(group_by(s2, ("config", "law", "ratio_ref", "param")).items()):
        vals = []
        for v in sorted({x["value"] for x in g}):
            gg = [x for x in g if x["value"] == v]
            vals.append(f"{v:g}→{np.mean([max(x['speed_mm_s'] for x in gg if x['amplitude_deg'] == a) for a in (30.0, 40.0)]):.0f}")
        L.append(f"| {cfg} | {law} | {r:g} | {param} | " + ", ".join(vals) + " |")
    L.append("\n### 左右の差 → 直進のずれ（°/周期）と、使える γ での旋回の余力\n")
    L.append("| 構成 | 法則 | r | 左右差 a | ずれ | 余力 | 全基準 |\n|---|---|---|---|---|---|---|")
    for row in drift_table(s2):
        L.append(f"| {row[0]} | {row[1]} | {row[2]:g} | {row[3]:g} | {row[4]:+.2f} | {row[5]:.1f} | {'OK' if row[6] else 'NG'} |")
    L.append("\n## 図\n")
    L += [f"- `plots/{p}`" for p in plots_made]
    (HERE / "decision_boundary.md").write_text("\n".join(L) + "\n", encoding="utf-8")


# ---- 実測の取り込み --------------------------------------------------------------------------------
REQUIRED = ("date", "method", "floor_id", "coupon_id", "material", "load_g", "direction", "trial")
DIRECTIONS = ("forward", "backward", "lateral", "diag45")


def _mu(row: dict[str, str]) -> float | None:
    m = (row.get("method") or "").strip().upper()
    if m == "T" and (row.get("angle_deg") or "").strip():
        return math.tan(math.radians(float(row["angle_deg"])))
    if m == "P":
        load = float(row["load_g"]) if (row.get("load_g") or "").strip() else 0.0
        f = (row.get("force_kinetic_g") or "").strip() or (row.get("force_static_g") or "").strip()
        return float(f) / load if (f and load > 0) else None
    return None


def validate(rows: list[dict[str, str]]) -> tuple[list[dict[str, Any]], list[str]]:
    """スキーマと値の範囲を確かめ、使える行と問題の一覧を返す（推測で埋めない）。"""
    problems, good = [], []
    for i, r in enumerate(rows, start=2):
        miss = [k for k in REQUIRED if not (r.get(k) or "").strip()]
        if miss:
            problems.append(f"行 {i}: 必須列が空 {miss}")
            continue
        d = r["direction"].strip().lower()
        if d not in DIRECTIONS:
            problems.append(f"行 {i}: direction が不明 {d!r}")
            continue
        mu = _mu(r)
        if mu is None:
            problems.append(f"行 {i}: μ を出せない（angle_deg / force が空）")
            continue
        if (r.get("method") or "").strip().upper() == "T" and not 0.5 <= float(r["angle_deg"]) <= 70.0:
            problems.append(f"行 {i}: 傾斜角 {r['angle_deg']}° が範囲外（0.5〜70）")
            continue
        track = (r.get("slide_track_deg") or "").strip()
        good.append({"floor": r["floor_id"].strip(), "coupon": f"{r['coupon_id'].strip()}-{r['material'].strip()}",
                     "direction": d, "mu": mu, "track_deg": float(track) if track else None,
                     "method": r["method"].strip().upper()})
    return good, problems


def fit(good: list[dict[str, Any]], n_boot: int = 400, seed: int = 0) -> list[dict[str, Any]]:
    """床 × クーポンごとに μ（方向別）と比を、ブートストラップの 95% 区間つきで出す。45° の滑り方向で法則を推定する。"""
    rng = np.random.default_rng(seed)
    out = []
    for (floor, coupon), g in group_by(good, ("floor", "coupon")).items():
        by = {d: np.array([x["mu"] for x in g if x["direction"] == d]) for d in DIRECTIONS}
        rec: dict[str, Any] = {"floor": floor, "coupon": coupon, "n": {d: int(len(v)) for d, v in by.items()}, "issues": []}
        for d, v in by.items():
            if len(v):
                rec[f"mu_{d}"] = float(v.mean())
                if len(v) >= 2 and v.std(ddof=1) / v.mean() > 0.2:
                    rec["issues"].append(f"{d} のばらつきが大きい（CV {v.std(ddof=1) / v.mean():.0%}）")
        if len(by["forward"]) >= 2 and len(by["lateral"]) >= 2:
            boots = [rng.choice(by["lateral"], len(by["lateral"])).mean() / rng.choice(by["forward"], len(by["forward"])).mean()
                     for _ in range(n_boot)]
            rec["ratio"] = rec["mu_lateral"] / rec["mu_forward"]
            rec["ratio_ci95"] = [float(np.percentile(boots, 2.5)), float(np.percentile(boots, 97.5))]
        else:
            rec["issues"].append("forward と lateral がそれぞれ 2 回以上ないので比を出さない")
        if len(by["forward"]) and len(by["backward"]) and rec["mu_backward"] < rec["mu_forward"]:
            rec["issues"].append("後ろ向きの方が滑りやすい（μ_back < μ_forward）。鱗の向きが逆の可能性")
        tracks = [x["track_deg"] for x in g if x["direction"] == "diag45" and x["track_deg"] is not None]
        if tracks and "ratio" in rec:
            k = 1.0 / rec["ratio"]                               # μ_t / μ_n
            pred = {"decoupled": math.degrees(math.atan(k)), "ellipse": math.degrees(math.atan(k * k))}
            m = float(np.mean(tracks))
            rec["law_track_measured_deg"] = m
            rec["law_track_predicted_deg"] = pred
            rec["law_estimate"] = min(pred, key=lambda lw: abs(pred[lw] - m))
        out.append(rec)
    return out


def ingest(csv_path: Path, workers: int) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    mdir = HERE / "results" / "measured"
    (mdir / "raw").mkdir(parents=True, exist_ok=True)
    raw = mdir / "raw" / f"{csv_path.stem}_{stamp}{csv_path.suffix}"
    shutil.copy2(csv_path, raw)                                  # 1. 生データを保存（書き換えない）
    with csv_path.open(encoding="utf-8-sig", newline="") as fp:
        rows = list(csv.DictReader(fp))
    good, problems = validate(rows)                             # 2. 検証
    fits = fit(good)                                            # 3. 当てはめ
    cases, tags = [], []                                        # 4. 再実行（比の区間の両端と中央 × 法則）
    for rec in fits:
        if "ratio" not in rec:
            continue
        laws = [rec["law_estimate"]] if "law_estimate" in rec else list(ASSUME["laws"])
        mu_f = rec["mu_forward"]
        mu_b = rec.get("mu_backward", mu_f)
        for which, r in (("lo", rec["ratio_ci95"][0]), ("mid", rec["ratio"]), ("hi", rec["ratio_ci95"][1])):
            for law in laws:
                for c in ASSUME["configs"]:
                    for a in SWEEP["monte_carlo"]["amplitude_deg"]:
                        for w in SWEEP["monte_carlo"]["waves"]:
                            cases.append(Case(c, {"mu_f": mu_f, "mu_b": mu_b, "mu_left": mu_f * r, "mu_right": mu_f * r, "law": law},
                                              float(a), float(w)))
                            tags.append({"floor": rec["floor"], "coupon": rec["coupon"], "which": which})
    results = run_cases(cases, workers) if cases else []
    for row, tag in zip(results, tags):
        row.update(tag)
    write_csv(mdir / f"{csv_path.stem}_{stamp}_runs.csv", results) if results else None
    # 5. 判定の更新
    L = [f"# HG-H0 実測の取り込み（{stamp}）\n", f"- 入力: `{csv_path.as_posix()}`（生データの控え: `results/measured/raw/{raw.name}`）",
         f"- 使えた行: {len(good)} / {len(rows)}。摩擦係数は **HARDWARE_VERIFIED（このクーポン × 床 × 条件に限る）**、"
         f"前進の予測は **{SOURCE}**\n", "## 検証で見つかった問題\n"]
    L += [f"- {p}" for p in problems] or ["- なし"]
    L.append("\n## 当てはめ\n\n| 床 | クーポン | n(前/後/横/45°) | μ_前 | μ_後 | μ_横 | 比 [95%] | 法則の推定 | 注意 |\n|---|---|---|---|---|---|---|---|---|")
    for rec in fits:
        n = rec["n"]
        ci = rec.get("ratio_ci95")
        law = rec.get("law_estimate", "未測定（45° の試験なし）")
        L.append(f"| {rec['floor']} | {rec['coupon']} | {n['forward']}/{n['backward']}/{n['lateral']}/{n['diag45']} | "
                 f"{rec.get('mu_forward', float('nan')):.3f} | {rec.get('mu_backward', float('nan')):.3f} | {rec.get('mu_lateral', float('nan')):.3f} | "
                 f"{rec.get('ratio', float('nan')):.2f} [{ci[0]:.2f}〜{ci[1]:.2f}] | {law} | {'; '.join(rec['issues']) or '—'} |"
                 if ci else f"| {rec['floor']} | {rec['coupon']} | {n['forward']}/{n['backward']}/{n['lateral']}/{n['diag45']} | — | — | — | — | — | {'; '.join(rec['issues'])} |")
    L.append("\n## 構成ごとの合否（比の 95% 区間の 下端 / 中央 / 上端。すべての法則の候補で）\n")
    L.append("| 床 | クーポン | " + " | ".join(ASSUME["configs"]) + " |\n|---|---|" + "---|" * len(ASSUME["configs"]))
    floor_best: dict[str, dict[str, bool]] = {}
    for (floor, coupon), g in group_by(results, ("floor", "coupon")).items():
        cells = []
        for c in ASSUME["configs"]:
            marks = []
            for which in ("lo", "mid", "hi"):
                gg = [r for r in g if r["config"] == c and r["which"] == which]
                oks = [best_passing([r for r in gg if r["law"] == lw])[1] for lw in {r["law"] for r in gg}]
                marks.append("○" if oks and all(oks) else ("△" if any(oks) else "×"))
                if which == "lo":
                    floor_best.setdefault(floor, {})
                    floor_best[floor][c] = floor_best[floor].get(c, False) or bool(oks and all(oks))
            cells.append("".join(marks))
        L.append(f"| {floor} | {coupon} | " + " | ".join(cells) + " |")
    L.append("\n○ = すべての法則候補で合格、△ = 法則しだい、× = 不合格。比の区間の下端でも ○ なら、その床ではその構成で足りる見込み。\n")
    L.append("## 床ごとの読み（比の 95% 区間の下端・最良のクーポンで。**提案であって正式 Decision ではない**）\n")
    for floor, ok in floor_best.items():
        rd = ("HEAD_YAW_OK" if ok.get("FW6_HEADYAW") else "BODY_YAW_NEEDED" if ok.get("FW6_YAW5") else
              "SNAKE_INSUFFICIENT（Wheel Belly を検討）")
        L.append(f"- {floor}: **{rd}**")
    out_md = mdir / f"{csv_path.stem}_{stamp}_decision.md"
    out_md.write_text("\n".join(L) + "\n", encoding="utf-8")
    (mdir / f"{csv_path.stem}_{stamp}_fit.json").write_text(json.dumps({"fits": fits, "problems": problems}, ensure_ascii=False, indent=2),
                                                              encoding="utf-8")
    # 6. Handoff の更新
    ho = ROOT / "ai-outbox" / "handoffs" / f"{datetime.now():%Y-%m-%d}_HG-H0_measured.md"
    ho.parent.mkdir(parents=True, exist_ok=True)
    with ho.open("a", encoding="utf-8") as fp:
        fp.write(f"\n## {stamp} H0 実測を取り込んだ\n\n- 判定: `simulation/hardware_gaps/HG-H0_friction/results/measured/{out_md.name}`\n"
                 f"- 使えた行 {len(good)} / {len(rows)}、問題 {len(problems)} 件\n"
                 + "".join(f"- {f}: {'HEAD_YAW_OK' if ok.get('FW6_HEADYAW') else 'BODY_YAW_NEEDED' if ok.get('FW6_YAW5') else 'SNAKE_INSUFFICIENT'}\n"
                           for f, ok in floor_best.items())
                 + "- 次: DEC-USER-0001 の順に、locomotion の再評価 → 6 本目の用途の仮決定（User 判断）\n")
    return out_md


def _read_rows(path: Path) -> list[dict[str, Any]]:
    """保存した CSV を読み戻す（数値は float、真偽は bool）。"""
    text = {"config", "law", "floor", "belly", "sampled_law", "reading", "param"}
    out = []
    with path.open(encoding="utf-8", newline="") as fp:
        for r in csv.DictReader(fp):
            row: dict[str, Any] = {}
            for k, v in r.items():
                if v == "":
                    continue
                if v in ("True", "False"):
                    row[k] = v == "True"
                elif k in text or k.startswith("fail_"):
                    row[k] = v
                else:
                    try:
                        row[k] = float(v)
                    except ValueError:
                        row[k] = v
            out.append(row)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="小さい掃引で動作確認")
    ap.add_argument("--measured", type=Path, default=None, help="H0 実測 CSV を取り込む")
    ap.add_argument("--from-results", action="store_true", help="保存済みの results/*.csv から判定書と図だけ作り直す")
    ap.add_argument("--workers", type=int, default=int(SWEEP["workers"]))
    args = ap.parse_args()
    if args.measured:
        print(ingest(args.measured, args.workers))
        return 0
    stamp = datetime.now().strftime("%Y-%m-%d %H:%M")
    res = HERE / "results"
    suffix = "_quick" if args.quick else ""
    if args.from_results:
        s1 = _read_rows(res / f"s1_ratio_sweep{suffix}.csv")
        s2 = _read_rows(res / f"s2_sensitivity{suffix}.csv")
        s3 = _read_rows(res / f"s3_monte_carlo{suffix}.csv")
    else:
        s1 = s1_ratio_sweep(args.workers, args.quick)
        s2 = s2_sensitivity(args.workers, args.quick)
        s3 = s3_monte_carlo(args.workers, args.quick)
        write_csv(res / f"s1_ratio_sweep{suffix}.csv", s1)
        write_csv(res / f"s2_sensitivity{suffix}.csv", s2)
        write_csv(res / f"s3_monte_carlo{suffix}.csv", s3)
    s1b = s1_boundaries(s1)
    s3d = s3_decisions(s3)
    write_csv(res / f"s3_decisions{suffix}.csv", s3d)
    (res / f"s1_boundaries{suffix}.json").write_text(json.dumps(s1b, indent=2), encoding="utf-8")
    if not args.quick:
        made = plots(s1, s3d, s2, HERE / "plots")
        write_decision_md(s1b, s1, s3d, s2, made, stamp, s3_rows=s3)
    print(json.dumps(s1b, indent=1))
    print(f"S1 {len(s1)} / S2 {len(s2)} / S3 {len(s3)} runs, 収束しなかった run: "
          f"{sum(not r['converged'] for r in s1 + s2 + s3)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
