"""形と機構（A〜G）の掃引（並列）と集計。共通乱数（物・位置ずれ・試行）、成功率は Wilson 90%。**MUJOCO_SIM。実物ではない。**"""
from __future__ import annotations

import math
import os
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass
from typing import Any

from simulation.scoop.forms.runner import OUTCOMES, run_form_episode
from simulation.scoop.model import load_config

Z90 = 1.6448536269514722
_CFG: dict[str, Any] | None = None


@dataclass(frozen=True)
class FCase:
    form: str
    params: tuple[tuple[str, Any], ...]      # 設計（順序つきの (名前, 値)）
    obj: str
    floor: str
    speed: float
    offset: float
    trial: int
    stop: bool = True
    tag: str = ""


def tag_of(form: str, params: dict[str, Any], speed: float, stop: bool) -> str:
    body = ",".join(f"{k}={v}" for k, v in sorted(params.items(), key=lambda kv: kv[0]))
    return f"{form}|{body}|v{speed:g}|{'stop' if stop else 'adv'}"


def seed_of(cfg: dict[str, Any], obj: str, offset: float, trial: int, offsets: list[float]) -> int:
    objs = list(cfg["objects"])
    return int(cfg["eval"]["base_seed"]) + trial * 1009 + offsets.index(offset) * 101 + objs.index(obj) * 7


def offsets_for(cfg: dict[str, Any], form: str) -> list[float]:
    if form == "cloche":
        return list(cfg["forms"]["cloche"]["offsets_mm"])
    return list(cfg["forms"]["cup"]["offsets_mm"]) if form == "cup" else list(cfg["placement"]["lateral_offsets_mm"])


def cases_for(cfg: dict[str, Any], form: str, params: dict[str, Any], speed: float, stop: bool, n_cell: int) -> list[FCase]:
    tag = tag_of(form, params, speed, stop)
    p = tuple(sorted(params.items(), key=lambda kv: kv[0]))
    return [FCase(form, p, o, f, speed, off, k, stop, tag)
            for o in cfg["objects"] for f in cfg["floors"] for off in offsets_for(cfg, form) for k in range(n_cell)]


def _init(config_path: str | None) -> None:
    global _CFG
    _CFG = load_config(config_path)


def run_fcase(c: FCase) -> dict[str, Any]:
    cfg = _CFG if _CFG is not None else load_config()
    r = run_form_episode(cfg, c.form, dict(c.params), c.obj, c.floor, c.speed, c.offset,
                         seed_of(cfg, c.obj, c.offset, c.trial, offsets_for(cfg, c.form)), stop_on_trigger=c.stop)
    row = asdict(r)
    row.update(tag=c.tag, obj=c.obj, floor=c.floor, speed=c.speed, offset=c.offset, trial=c.trial, stop=c.stop,
               **{f"p_{k}": v for k, v in c.params})
    return row


def run_fcases(cases: list[FCase], workers: int = 14) -> list[dict[str, Any]]:
    if workers <= 1:
        _init(None)
        return [run_fcase(c) for c in cases]
    with ProcessPoolExecutor(max_workers=workers, initializer=_init, initargs=(None,)) as ex:
        return list(ex.map(run_fcase, cases, chunksize=8))


def wilson(k: int, n: int, z: float = Z90) -> tuple[float, float]:
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    den = 1 + z * z / n
    mid = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (max(0.0, mid - half), min(1.0, mid + half))


def summarize(rows: list[dict[str, Any]], keys: tuple[str, ...]) -> list[dict[str, Any]]:
    groups: dict[tuple, list[dict[str, Any]]] = {}
    for r in rows:
        groups.setdefault(tuple(r[k] for k in keys), []).append(r)
    out = []
    for key, g in groups.items():
        n = len(g)
        cnt = Counter(r["outcome"] for r in g)
        k = cnt.get("success", 0)
        lo, hi = wilson(k, n)
        row = {**dict(zip(keys, key)), "n": n, "successes": k, "rate": k / n, "ci_lo": lo, "ci_hi": hi,
               **{f"n_{o}": cnt.get(o, 0) for o in OUTCOMES},
               "n_launched": sum(bool(r["launched"]) for r in g),
               "n_entered": sum(bool(r["entered_ever"]) for r in g),
               "n_rode": sum(bool(r["rode_ever"]) for r in g),
               "n_inside_at_close": sum(bool(r["inside_at_close"]) for r in g),
               "n_closed": sum(bool(r["closed"]) for r in g),
               "escape_mean_mm": sum(r["escape_mm"] for r in g) / n,
               "moved_mean_mm": sum(r["moved_mm"] for r in g) / n}
        out.append(row)
    return out
