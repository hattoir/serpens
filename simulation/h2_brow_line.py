"""H2 VIS-0004: 眉の線（左右に走る）を合成画像で描き、列ごとに追って高さと途切れを測る。頬の線（現行・行ごと）と比べる。
**SYNTHETIC_VISION_SIM。**

VIS-0001（幾何だけ）で「眉でも三角測量は成り立つ」とした。ここでは画像の上で、線の太さ・ぼけ・物の側面・鏡面の途切れを入れて確かめる。
検出（`detect.py`）には入れない（試作）。眉に決まったら `trace_line` を線の向きに依らない形にする。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from serpens.floorwatch.detect import _running_baseline, trace_line
from serpens.floorwatch.geometry import Camera, LightPlane
from serpens.floorwatch.synthetic import Disc, Lighting, Renderer, Scene

SOURCE = "SYNTHETIC_VISION_SIM"


def _fp(cam: Camera, u: float, v: float) -> np.ndarray:
    p = cam.floor_point(u, v)
    return np.array([np.nan, np.nan, np.nan]) if p is None else p


def line_v_on_floor(plane: LightPlane, cam: Camera, u: float) -> float | None:
    """列 u で、床（z=0）上の線が写る行 v（二分法）。"""
    def f(v: float) -> float | None:
        p = cam.floor_point(u, v)
        return None if p is None else float(plane.normal @ p - plane.d)
    lo, hi = 0.0, float(cam.height_px - 1)
    while lo < hi and f(lo) is None:                            # 上の方の行は床に届かないことがある（水平線より上）
        lo += 1.0
    flo, fhi = f(lo), f(hi)
    if flo is None or fhi is None or flo * fhi > 0:
        return None
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        fm = f(mid)
        if fm is None:
            return None
        if flo * fm <= 0:
            hi = mid
        else:
            lo, flo = mid, fm
    return 0.5 * (lo + hi)


@dataclass
class ColumnTrace:
    cols: np.ndarray
    v_line: np.ndarray
    v_floor: np.ndarray
    height_mm: np.ndarray


def trace_columns(line_sub: np.ndarray, cam: Camera, plane: LightPlane, search_px: int, min_intensity: float,
                  step: int = 2) -> ColumnTrace:
    """列ごとに線の位置（半値以上の連続区間の輝度重心）→ 光の面との交点で高さ。`detect.trace_line` の列版。"""
    H, W = line_sub.shape
    cols = np.arange(0, W, step)
    v_floor = np.array([line_v_on_floor(plane, cam, float(u)) or np.nan for u in cols])
    v_line, height = np.full(cols.size, np.nan), np.full(cols.size, np.nan)
    for k, u in enumerate(cols):
        if np.isnan(v_floor[k]):
            continue
        lo, hi = int(max(0, v_floor[k] - search_px)), int(min(H, v_floor[k] + search_px))
        prof = line_sub[lo:hi, u]
        if prof.size == 0 or prof.max() < min_intensity:
            continue
        peak = int(np.argmax(prof))
        w = prof >= prof.max() * 0.5
        a = peak
        while a - 1 >= 0 and w[a - 1]:
            a -= 1
        b = peak
        while b + 1 < w.size and w[b + 1]:
            b += 1
        vs = np.arange(lo + a, lo + b + 1)
        pw = prof[a:b + 1]
        v = float((vs * pw).sum() / pw.sum())
        h = plane.height_at(cam, float(u), v)
        if h is not None:
            v_line[k], height[k] = v, h
    return ColumnTrace(cols, v_line, v_floor, height)


def object_reading(heights: np.ndarray, present: np.ndarray, on_obj: np.ndarray, baseline_half: int,
                   dropout_min: int) -> dict[str, Any]:
    """物の上の行（または列）の、局所の床の基準との差の中央値と、途切れた数。"""
    base = _running_baseline(heights, baseline_half)
    rel = heights - base
    hs = rel[on_obj & present]
    missing = int((on_obj & ~present).sum())
    return {"height_mm": float(np.median(hs)) if hs.size else None, "n_on": int(on_obj.sum()),
            "n_measured": int(hs.size), "missing": missing, "dropout": missing >= dropout_min}


def compare_on_object(cfg: dict[str, Any], cam: Camera, cheek: LightPlane, brow: LightPlane, brow_y0: float, lt: Lighting,
                      disc: Disc, seed: int, det: dict[str, Any]) -> dict[str, Any]:
    """同じ物を、頬の線の上（x=0）と眉の線の上（y=y0）に置いて、線の高さと途切れを比べる。"""
    out: dict[str, Any] = {"kind": disc.kind, "d": disc.diameter_mm, "h": disc.height_mm, "specular": disc.specular}
    r = disc.diameter_mm / 2
    # 頬: 物を (0, y0) に置く。行ごとに追う（既存の trace_line）
    d_c = Disc(0.0, brow_y0, disc.diameter_mm, disc.height_mm, disc.albedo, disc.specular, disc.kind)
    fr = Renderer(cam, cheek, lt, sides=True).render(Scene([d_c], seed=seed))
    sub = np.clip(np.float32(fr["line"]) - np.float32(fr["dark"]), 0, None)
    tr = trace_line(sub, cam, cheek, det)
    fy = np.array([np.nan if np.isnan(u) else _fp(cam, float(u), float(v))[1] for v, u in zip(tr.rows, tr.u_floor)])
    on = ~np.isnan(tr.u_floor) & (np.abs(fy - brow_y0) <= r)
    out["cheek"] = object_reading(tr.height_mm, ~np.isnan(tr.u_line), on, int(det["line_baseline_rows"]),
                                  int(det["dropout_rows_min"]))
    # 眉: 同じ物を (0, y0) に置く。列ごとに追う
    fr = Renderer(cam, brow, lt, sides=True).render(Scene([d_c], seed=seed))
    sub = np.clip(np.float32(fr["line"]) - np.float32(fr["dark"]), 0, None)
    ct = trace_columns(sub, cam, brow, int(det["line_search_px"]), float(det["line_min_intensity"]))
    fx = np.array([np.nan if np.isnan(v) else _fp(cam, float(u), float(v))[0] for u, v in zip(ct.cols, ct.v_floor)])
    on = ~np.isnan(ct.v_floor) & (np.abs(fx) <= r)
    step = int(ct.cols[1] - ct.cols[0]) if ct.cols.size > 1 else 1
    # 眉の線は視野の横幅（約 98mm）しかなく、物が占める割合が大きい → 基準の窓は画素でなく線の長さの割合で取る
    # （行の数で取ると 20mm の硬貨が窓の半分を超え、中央値が硬貨の高さになって 0 と読んだ）
    half = max(1, int(0.45 * int((~np.isnan(ct.v_floor)).sum())))
    out["brow"] = object_reading(ct.height_mm, ~np.isnan(ct.v_line), on, half, max(1, int(det["dropout_rows_min"]) // step))
    return out
