"""H2: 床見の合成画像で、条件を 1 つずつ（OFAT）/ 同時に（Monte Carlo）ずらして検出の崩れ方を測る。
**SYNTHETIC_VISION_SIM（実写・実機の性能ではない）。**

    .\\.venv\\Scripts\\python.exe tools\\h2_vision.py sweep  [--reps 3] [--workers 12]
    .\\.venv\\Scripts\\python.exe tools\\h2_vision.py mc     [--n 300]
    → simulation/results/h2_vision_{sweep,mc}.{csv,md}（1 行 1 試行の *_trials.csv も）

解像度は既定で UXGA の半分（800×600、同じ画角・f も半分。画素のしきい値は f に比例して検出側で直る）。`--scale 1` で UXGA。
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, replace
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulation.h2_vision import CRITICAL, SOURCE, TARGETS, Condition, Trial, summarize  # noqa: E402

NEGATIVES = ("empty", "stain", "seam")

# 1 つずつずらす因子と水準（最初が名目。範囲は家の中で起こりうる幅の見込み = ASSUMPTION）
OFAT: dict[str, list[Any]] = {
    "floor": ["wood", "tile", "rug", "carpet", "pattern"],
    "texture_contrast": [1.0, 0.5, 2.0, 3.0],
    "floor_height_sigma_mm": [0.0, 0.1, 0.2, 0.4, 0.8],
    "blur_sigma_px": [0.0, 1.0, 2.0, 4.0, 8.0],
    "ambient_lux": [8.0, 50.0, 150.0, 400.0],
    "ambient_drift": [0.0, 0.02, 0.05, 0.1, 0.2],
    "shot_noise_k": [0.0, 0.5, 1.0, 2.0],
    "line_width_mm": [3.0, 1.5, 5.0],
    "line_scatter_mm": [0.0, 0.5, 1.0, 2.0],
    "raking_led_height_mm": [6.0, 3.0, 10.0, 15.0],
    "cam_height_err_mm": [0.0, -10.0, -5.0, 5.0],
    "cam_pitch_err_deg": [0.0, -5.0, -2.0, 2.0, 5.0],
    "fov_err_deg": [0.0, -10.0, -5.0, 5.0, 10.0],
    "aim_err_mm": [0.0, 2.0, 4.0, 6.0, 8.0],
    "clutter": [False, True],
    "sides": [True, False],
    "reaim": [False, True],
    "pose_from_line": [True, False],
    "lighting_model": ["uniform", "point", "point+flat"],
}
# 露出を合わせて比べる因子（合わせないと 255 で飽和して比較にならない）。点光源は近い床が明るく、名目の露出では飽和する
AMBIENT_WITH_AE = {"ambient_lux", "lighting_model"}
# 照明の模型: uniform = 一様（従来）/ point = 点光源 cos/r² / point+flat = 点光源 + 照明の較正画像で割る（VIS-0007）
LIGHTING_MODELS = {"uniform": {}, "point": {"physical_falloff": True}, "point+flat": {"physical_falloff": True, "flat_field": True}}

_CTX: dict[str, Any] = {}


def _init(scale: float) -> None:
    from serpens.config import load_config
    from serpens.floorwatch.geometry import Camera, LightPlane
    from serpens.floorwatch.synthetic import default_lighting
    from simulation.h2_vision import scaled_camera
    cfg = load_config()
    _CTX.update(cfg=cfg, cam=scaled_camera(Camera.from_cfg(cfg), scale), plane=LightPlane.design(cfg),
                lt=default_lighting(cfg), scale=scale)


def _job(args: tuple[dict[str, Any], str | None, str, int]) -> dict[str, Any]:
    from simulation.h2_vision import run_trial
    cond_d, target, negative, seed = args
    c = Condition(**cond_d)
    t = run_trial(_CTX["cfg"], _CTX["cam"], _CTX["plane"], _CTX["lt"], c, target, seed, blur_scale=_CTX["scale"],
                  negative=negative)
    return {"cond": cond_d, "target": target, "negative": negative if target is None else "", "seed": seed,
            "patrol": t.stage_hits["patrol"], "inspect": t.stage_hits["inspect"],
            "fa_patrol": t.false_objects["patrol"], "fa_inspect": t.false_objects["inspect"],
            "height_est": t.height_est, "height_reason": t.height_reason, "diameter_est": t.diameter_est,
            "critical_flag": t.critical_flag, "loc_err_mm": t.loc_err_mm}


def _as_trial(r: dict[str, Any]) -> Trial:
    t = Trial(r["target"], {"patrol": r["patrol"], "inspect": r["inspect"]},
              {"patrol": r["fa_patrol"], "inspect": r["fa_inspect"]}, r["height_est"], r["height_reason"],
              r["diameter_est"], r["critical_flag"], r["loc_err_mm"])
    return t


def _jobs_for(cond: Condition, reps: int, seed0: int) -> list[tuple[dict[str, Any], str | None, str, int]]:
    d = asdict(cond)
    out = []
    k = seed0
    for rep in range(reps):
        for tg in TARGETS:
            out.append((d, tg, "", k))
            k += 1
        for neg in NEGATIVES:
            out.append((d, None, neg, k))
            k += 1
    return out


def _run(jobs: list, workers: int, scale: float) -> list[dict[str, Any]]:
    with ProcessPoolExecutor(max_workers=workers, initializer=_init, initargs=(scale,)) as ex:
        return list(ex.map(_job, jobs, chunksize=4))


def _write_trials(path: Path, rows: list[dict[str, Any]]) -> None:
    flat = [{**{f"c_{k}": v for k, v in r["cond"].items()}, **{k: v for k, v in r.items() if k != "cond"}} for r in rows]
    with path.open("w", newline="", encoding="utf-8") as fp:
        w = csv.DictWriter(fp, fieldnames=list(flat[0]))
        w.writeheader()
        w.writerows(flat)


def _per_target(rows: list[dict[str, Any]]) -> dict[str, float]:
    out = {}
    for tg in TARGETS:
        xs = [r for r in rows if r["target"] == tg]
        out[tg] = float(np.mean([r["inspect"] for r in xs])) if xs else float("nan")
    return out


def cmd_sweep(a: argparse.Namespace) -> int:
    base = Condition()
    plan: list[tuple[str, Any, Condition]] = []
    only = set(a.factors.split(",")) if a.factors else None
    for factor, levels in OFAT.items():
        if only is not None and factor not in only:
            continue
        for lv in levels:
            extra = {"auto_exposure": True} if factor in AMBIENT_WITH_AE else {}
            if factor == "lighting_model":
                plan.append((factor, lv, replace(base, **LIGHTING_MODELS[lv], **extra)))
            else:
                plan.append((factor, lv, replace(base, **{factor: lv}, **extra)))
    jobs, index = [], []
    for i, (_f, _lv, c) in enumerate(plan):
        js = _jobs_for(c, a.reps, 1000 * i)
        jobs += js
        index += [i] * len(js)
    t0 = time.time()
    rows = _run(jobs, a.workers, a.scale)
    a.out.mkdir(parents=True, exist_ok=True)
    _write_trials(a.out / f"h2_vision_sweep{a.tag}_trials.csv", rows)
    table = []
    for i, (factor, lv, _c) in enumerate(plan):
        rs = [r for r, j in zip(rows, index) if j == i]
        s = summarize([_as_trial(r) for r in rs])
        table.append({"factor": factor, "level": lv, **s, **{f"insp_{k}": v for k, v in _per_target(rs).items()}})
    keys = list(table[0])
    with (a.out / f"h2_vision_sweep{a.tag}.csv").open("w", newline="", encoding="utf-8") as fp:
        w = csv.DictWriter(fp, fieldnames=keys)
        w.writeheader()
        w.writerows(table)
    _write_sweep_md(a, table, time.time() - t0, len(rows))
    return 0


def _fmt(v: Any) -> str:
    return "—" if isinstance(v, float) and math.isnan(v) else (f"{v:.2f}" if isinstance(v, float) else str(v))


def _write_sweep_md(a: argparse.Namespace, table: list[dict[str, Any]], dt: float, n: int) -> None:
    lines = [f"# H2 視覚: 1 因子ずつ（{SOURCE}。{n} 試行、{dt / 60:.1f} 分、解像度 UXGA×{a.scale}、1 水準 = 対象 {len(TARGETS)} × "
             f"{a.reps} 回 + 陰性 {len(NEGATIVES)} × {a.reps} 回）", "",
             "名目: 木目・模様 1.0・凹凸 0・ぼけ 0・環境光 8・線 3mm・斜め照明 6mm・カメラのずれ 0・狙いの誤差 0・側面あり。",
             "critical = ボタン電池・磁石・錠剤。flagged = 鏡面の危険物が metal_disc（危険物側）に回った割合。", "",
             "| 因子 | 水準 | patrol 検出 | inspect 検出 | critical inspect | 鏡面 critical flagged | 誤報 patrol | 誤報 inspect "
             "| 高さ誤差 中央 mm | 大きさ誤差 中央 | 位置誤差 中央 mm |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in table:
        lines.append(f"| {r['factor']} | {r['level']} | {_fmt(r['patrol_recall'])} | {_fmt(r['inspect_recall'])} | "
                     f"{_fmt(r['critical_inspect_recall'])} | {_fmt(r['specular_critical_flagged'])} | {_fmt(r['false_alarm_patrol'])} | "
                     f"{_fmt(r['false_alarm_inspect'])} | {_fmt(r['height_abs_err_median_mm'])} | {_fmt(r['diameter_rel_err_median'])} | "
                     f"{_fmt(r['loc_err_median_mm'])} |")
    lines += ["", "## 対象ごとの inspect 検出（因子 × 水準）", "",
              "| 因子 | 水準 | " + " | ".join(TARGETS) + " |", "|---|---|" + "---|" * len(TARGETS)]
    for r in table:
        lines.append(f"| {r['factor']} | {r['level']} | " + " | ".join(_fmt(r[f'insp_{t}']) for t in TARGETS) + " |")
    (a.out / f"h2_vision_sweep{a.tag}.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _sample_condition(rng: np.random.Generator) -> Condition:
    """家の中の条件の見込み（ASSUMPTION）から 1 つ引く。"""
    floor = str(rng.choice(["wood", "wood", "tile", "rug", "carpet", "pattern"]))
    pile = {"wood": 0.0, "tile": 0.0, "rug": 0.2, "carpet": 0.4, "pattern": 0.1}[floor]
    return Condition(floor=floor, texture_contrast=float(rng.uniform(0.5, 2.0)),
                     floor_height_sigma_mm=float(rng.uniform(0.0, 2.0) * pile),
                     blur_sigma_px=float(rng.choice([0.0, 0.0, 1.0, 2.0, 4.0])),
                     ambient_lux=float(rng.choice([8.0, 50.0, 150.0])), ambient_drift=float(rng.uniform(0.0, 0.05)),
                     auto_exposure=True, shot_noise_k=float(rng.uniform(0.0, 1.0)),
                     line_scatter_mm=float(rng.uniform(0.0, 1.5)) if floor in ("rug", "carpet") else 0.0,
                     cam_height_err_mm=float(rng.normal(0.0, 3.0) - (4.0 if floor == "carpet" else 0.0)),
                     cam_pitch_err_deg=float(rng.normal(0.0, 2.0)), fov_err_deg=float(rng.normal(0.0, 3.0)),
                     aim_err_mm=float(abs(rng.normal(0.0, 2.0))), clutter=bool(rng.random() < 0.2),
                     reaim=True)                                   # mission は候補へ線を向け直して撮り直す


def cmd_mc(a: argparse.Namespace) -> int:
    rng = np.random.default_rng(a.seed)
    conds = [_sample_condition(rng) for _ in range(a.n)]
    jobs = []
    for i, c in enumerate(conds):
        d = asdict(c)
        tg = list(TARGETS)[i % len(TARGETS)]
        jobs.append((d, tg, "", 10_000 + 3 * i))
        jobs.append((d, None, NEGATIVES[i % len(NEGATIVES)], 10_001 + 3 * i))
        jobs.append((d, list(CRITICAL)[i % len(CRITICAL)], "", 10_002 + 3 * i))
    t0 = time.time()
    rows = _run(jobs, a.workers, a.scale)
    a.out.mkdir(parents=True, exist_ok=True)
    _write_trials(a.out / "h2_vision_mc_trials.csv", rows)
    s = summarize([_as_trial(r) for r in rows])
    # 因子ごとの「崩れ方」: critical の inspect 検出を、因子の値の 3 分位で比べる
    crit = [r for r in rows if r["target"] in CRITICAL]
    lines = [f"# H2 視覚: Monte Carlo（{SOURCE}。条件 {a.n}、試行 {len(rows)}、{(time.time() - t0) / 60:.1f} 分、UXGA×{a.scale}）", "",
             "条件は家の中の見込み（`tools/h2_vision.py` の `_sample_condition`、すべて ASSUMPTION）から同時にずらした。", "",
             "| 指標 | 値 |", "|---|---|"] + [f"| {k} | {_fmt(v)} |" for k, v in s.items()]
    lines += ["", "## critical の inspect 検出 × 因子（数値因子は 3 分位、カテゴリは値ごと）", "",
              "| 因子 | 区分 | n | 検出 |", "|---|---|---|---|"]
    for f in ("floor", "floor_height_sigma_mm", "blur_sigma_px", "ambient_lux", "shot_noise_k", "line_scatter_mm",
              "cam_height_err_mm", "cam_pitch_err_deg", "fov_err_deg", "aim_err_mm", "clutter"):
        vals = [r["cond"][f] for r in crit]
        if isinstance(vals[0], (str, bool)):
            groups = {str(v): [r for r in crit if str(r["cond"][f]) == str(v)] for v in sorted(set(map(str, vals)))}
        else:
            qs = np.quantile(vals, [1 / 3, 2 / 3])
            groups = {f"≤{qs[0]:.2f}": [r for r in crit if r["cond"][f] <= qs[0]],
                      f"{qs[0]:.2f}〜{qs[1]:.2f}": [r for r in crit if qs[0] < r["cond"][f] <= qs[1]],
                      f">{qs[1]:.2f}": [r for r in crit if r["cond"][f] > qs[1]]}
        for g, rs in groups.items():
            lines.append(f"| {f} | {g} | {len(rs)} | {_fmt(float(np.mean([r['inspect'] for r in rs])) if rs else float('nan'))} |")
    (a.out / "h2_vision_mc.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("sweep", "mc"):
        p = sub.add_parser(name)
        p.add_argument("--out", type=Path, default=ROOT / "simulation" / "results")
        p.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 4))
        p.add_argument("--scale", type=float, default=0.5)
        p.add_argument("--seed", type=int, default=0)
        if name == "sweep":
            p.add_argument("--reps", type=int, default=3)
            p.add_argument("--factors", default="", help="カンマ区切りで因子を絞る（空 = 全部）")
            p.add_argument("--tag", default="", help="結果のファイル名に付ける（絞ったときに全体の結果を上書きしない）")
        else:
            p.add_argument("--n", type=int, default=300)
    a = ap.parse_args()
    return cmd_sweep(a) if a.cmd == "sweep" else cmd_mc(a)


if __name__ == "__main__":
    raise SystemExit(main())
