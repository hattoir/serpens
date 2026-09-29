"""スコップ形状 × 対象物 × 床 × 速度 × 位置ずれ の掃引（並列）と集計（成功率・Wilson の信頼区間・失敗分類・逃げ距離）。

`tools/scoop_sweep.py` から使う。**MUJOCO_SIM。実物の試験片の結果が出るまで「候補」。**
乱数の種は (対象物, 位置ずれ, 試行番号) だけで決める（共通乱数）→ 形状どうしを同じ初期条件で比べられる。
"""
from __future__ import annotations

import math
import os
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass
from typing import Any, Iterable

from simulation.scoop.model import Shape, all_shapes, load_config
from simulation.scoop.runner import OUTCOMES, run_episode

Z90 = 1.6448536269514722          # 90% 両側
_CFG: dict[str, Any] | None = None


@dataclass(frozen=True)
class Case:
    shape: Shape
    obj: str
    floor: str
    speed: float
    offset: float
    trial: int
    trigger: str = "ideal"
    clearance_mm: float | None = None
    scoop_mu: float | None = None
    rolling_scale: float = 1.0
    numerics: tuple[tuple[str, Any], ...] = ()
    tag: str = ""


def seed_of(cfg: dict[str, Any], obj: str, offset: float, trial: int) -> int:
    objs = list(cfg["objects"])
    offs = list(cfg["placement"]["lateral_offsets_mm"])
    return int(cfg["eval"]["base_seed"]) + trial * 1009 + offs.index(offset) * 101 + objs.index(obj) * 7


def _init(config_path: str | None) -> None:
    global _CFG
    _CFG = load_config(config_path)


def run_case(case: Case) -> dict[str, Any]:
    cfg = _CFG if _CFG is not None else load_config()
    r = run_episode(cfg, case.shape, case.obj, case.floor, case.speed, case.offset, seed_of(cfg, case.obj, case.offset, case.trial),
                    trigger=case.trigger, clearance_mm=case.clearance_mm, scoop_mu=case.scoop_mu,
                    rolling_scale=case.rolling_scale, numerics=dict(case.numerics) or None)
    row = asdict(r)
    row.pop("numerics", None)
    row.update(shape=case.shape.key, tip_mm=case.shape.tip_mm, alpha_deg=case.shape.alpha_deg, side_wall=case.shape.side_wall,
               width_mm=case.shape.width_mm, obj=case.obj, floor=case.floor, speed=case.speed, offset=case.offset,
               trial=case.trial, trigger=case.trigger, clearance_mm=case.clearance_mm, scoop_mu=case.scoop_mu,
               rolling_scale=case.rolling_scale, tag=case.tag)
    return row


def run_cases(cases: list[Case], workers: int = 12, config_path: str | None = None) -> list[dict[str, Any]]:
    if workers <= 1:
        _init(config_path)
        return [run_case(c) for c in cases]
    with ProcessPoolExecutor(max_workers=workers, initializer=_init, initargs=(config_path,)) as ex:
        return list(ex.map(run_case, cases, chunksize=16))


def make_cases(cfg: dict[str, Any], shapes: Iterable[Shape], n: int, *, objects: list[str] | None = None,
               floors: list[str] | None = None, speeds: list[float] | None = None, offsets: list[float] | None = None,
               trigger: str = "ideal", **kw: Any) -> list[Case]:
    objects = objects or list(cfg["objects"])
    floors = floors or list(cfg["floors"])
    speeds = speeds or list(cfg["motion"]["speeds_mm_s"])
    offsets = offsets or list(cfg["placement"]["lateral_offsets_mm"])
    return [Case(s, o, f, v, off, k, trigger=trigger, **kw)
            for s in shapes for o in objects for f in floors for v in speeds for off in offsets for k in range(n)]


def wilson(k: int, n: int, z: float = Z90) -> tuple[float, float]:
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    den = 1 + z * z / n
    mid = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (max(0.0, mid - half), min(1.0, mid + half))


def summarize(rows: list[dict[str, Any]], keys: tuple[str, ...]) -> list[dict[str, Any]]:
    """`keys` ごとに成功率・信頼区間・失敗分類の内訳・押して逃げた距離。"""
    groups: dict[tuple, list[dict[str, Any]]] = {}
    for r in rows:
        groups.setdefault(tuple(r[k] for k in keys), []).append(r)
    out = []
    for key, g in groups.items():
        n = len(g)
        k = sum(r["success"] for r in g)
        lo, hi = wilson(k, n)
        cnt = Counter(r["outcome"] for r in g)
        pushed = [r["forward_disp_mm"] for r in g if r["outcome"] == "pushed_ahead"]
        out.append({**dict(zip(keys, key)), "n": n, "successes": k, "rate": k / n, "ci_lo": lo, "ci_hi": hi,
                    **{f"n_{o}": cnt.get(o, 0) for o in OUTCOMES},
                    "push_mean_mm": sum(pushed) / len(pushed) if pushed else 0.0,
                    "push_max_mm": max(pushed) if pushed else 0.0,
                    "n_no_trigger": sum(not r["triggered"] for r in g)})
    return out
