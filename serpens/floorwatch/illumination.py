"""照明の較正画像を、今の姿勢に合わせ直す（H2 VIS-0008）。**照明の模型は点光源（cos / r²）。すべて DESIGN / ASSUMED。**

照明の較正画像（白いカード、名目の姿勢で頭ごとに 1 回）は、LED の明るさ・配光・周辺減光と、床の幾何（cos / r²）の積。
頭が沈む・首が上がると幾何の部分だけが変わる（LED もカメラも頭に固定）。そこで

    今の姿勢で予想される明るさ = 較正画像 × 幾何(今の姿勢) / 幾何(較正の姿勢)

とする。姿勢は、線があれば床の線から（pose.py）、無ければ明るさの傾きから（`pose_from_shading`: 通常画像 ÷ 予想 が最も平らになる姿勢）。
較正を名目のまま使うと、首が 5° 上がっただけで巡回の検出が 21/22 → 0/22 に落ちた（合成、VIS-0008）。
"""
from __future__ import annotations

from typing import Any

import numpy as np

from serpens.floorwatch.geometry import Camera, head_point
from serpens.floorwatch.pose import posed_camera


def led_positions(cfg: dict[str, Any]) -> dict[str, np.ndarray]:
    """名目の姿勢での LED の位置（世界座標 = カメラ直下の床が原点）。"""
    fw = cfg["floor_watch"]
    nl = fw.get("normal_light", {"led_height_mm": fw["camera"]["height_mm"], "led_forward_mm": 0.0})
    return {"normal": np.array([0.0, float(nl["led_forward_mm"]), float(nl["led_height_mm"])]),
            "raking": np.array([0.0, float(fw["raking"]["led_forward_mm"]), float(fw["raking"]["led_height_mm"])])}


def floor_irradiance(cam: Camera, led: np.ndarray, step: int = 1) -> np.ndarray:
    """画素ごとに、その画素に写る床（z=0）の点の照度 cos / r²（床の法線は上）。step で間引いた格子（形 = (H/step, W/step)）。"""
    vs, us = np.mgrid[0:cam.height_px:step, 0:cam.width_px:step].astype(float)
    xa, ya, za = cam._axes
    d = ((us - cam.cx) / cam.f_px)[..., None] * xa + ((vs - cam.cy) / cam.f_px)[..., None] * ya + za
    dz = d[..., 2]
    ok = dz < -1e-9
    s = np.where(ok, -cam.height_mm / np.where(ok, dz, 1.0), np.nan)
    p = cam.center + s[..., None] * d
    v = led - p
    r2 = np.sum(v * v, axis=-1)
    e = np.clip(v[..., 2], 0.0, None) / np.sqrt(np.maximum(r2, 1e-9)) / np.maximum(r2, 1e-9)
    return np.where(ok, e, np.nan)


def repose_ratio(cam_nom: Camera, dh: float, dp: float, led_nom: np.ndarray, step: int = 1) -> np.ndarray:
    """幾何(姿勢 dh, dp) / 幾何(名目)。LED は頭に固定なので姿勢と一緒に動く。"""
    cam = posed_camera(cam_nom, dh, dp)
    g1 = floor_irradiance(cam, head_point(led_nom, cam_nom, cam), step)
    g0 = floor_irradiance(cam_nom, led_nom, step)
    return g1 / np.maximum(g0, 1e-30)


def pose_from_shading(normal_sub: np.ndarray, flat_normal: np.ndarray, cam_nom: Camera, led_nom: np.ndarray,
                      dh_range: float, dp_range: float, step: int = 8) -> tuple[float, float]:
    """通常画像 ÷（較正画像 × 幾何の比）の対数が最も平らになる (dh, dp)。平らさ = 頑健な散らばり（MAD）。
    床の模様・物は外れ値として MAD が無視する。粗い格子 → 細かい格子。"""
    n = np.asarray(normal_sub, float)[::step, ::step]
    f = np.asarray(flat_normal, float)[::step, ::step]
    good = (f > 5.0) & (n > 2.0)
    ln, lf = np.log(np.where(good, n, 1.0)), np.log(np.where(good, f, 1.0))

    def spread(dh: float, dp: float) -> float:
        rr = repose_ratio(cam_nom, dh, dp, led_nom, step)[: n.shape[0], : n.shape[1]]
        m = good & np.isfinite(rr) & (rr > 0)
        if m.sum() < 50:
            return float("inf")
        r = (ln - lf - np.log(np.where(m, rr, 1.0)))[m]
        return float(np.median(np.abs(r - np.median(r))))
    best = (0.0, 0.0)
    for k in range(3):
        sh, sp = dh_range / 3 ** k, dp_range / 3 ** k
        grid = [(best[0] + a, best[1] + b) for a in np.linspace(-sh, sh, 7) for b in np.linspace(-sp, sp, 7)]
        best = min(grid, key=lambda g: spread(*g))
    return best
