"""床の線からカメラの姿勢のずれ（高さ・pitch）を推定する。光の面は頭に固定なので、床の線の写り方が姿勢で決まる。

画像の線は 2 つの量（位置・傾き）を持つので、Δ高さ と Δpitch の 2 つが解ける（H2 VIS-0003、SYNTHETIC_VISION_SIM で
±0.75mm / ±0.23°）。線の向き（頬 = 前後、眉 = 左右）に依らないよう、残差は画像の直線までの垂直距離で測る。
床が平らで、線の大半が床に乗っている前提。roll と画角のずれは扱わない。
"""
from __future__ import annotations

from dataclasses import replace

import numpy as np

from serpens.floorwatch.geometry import Camera, LightPlane


def posed_camera(cam: Camera, dh_mm: float, dp_deg: float) -> Camera:
    return replace(cam, height_mm=cam.height_mm + dh_mm, pitch_deg=cam.pitch_deg + dp_deg)


def line_residual_px(points_uv: np.ndarray, cam_nom: Camera, plane_nom: LightPlane, dh: float, dp: float) -> np.ndarray:
    """姿勢 (名目 + dh, dp) で予測した床の線と、観測した線の点の、画像上の垂直距離 [px]。"""
    cam = posed_camera(cam_nom, dh, dp)
    img = plane_nom.fixed_to_head(cam_nom, cam).floor_line_image(cam)
    if img is None:
        return np.full(len(points_uv), np.inf)
    a, d = img
    rel = points_uv - a
    return np.abs(rel[:, 0] * d[1] - rel[:, 1] * d[0])


def estimate_pose(points_uv: np.ndarray, cam_nom: Camera, plane_nom: LightPlane, dh_range: float = 12.0,
                  dp_range: float = 6.0, n_points: int = 40, trim: float = 1.0) -> tuple[float, float, float]:
    """(dh_mm, dp_deg, rms_px)。点が少なすぎれば nan。物で持ち上がった点は、残差の中央値の 3 倍を超えたら外す。

    trim < 1: 残差の小さい方から trim の割合だけで合わせる（最小トリム二乗）。名目の姿勢で線を探すと、姿勢が大きくずれた側
    （近い半分など）で探す窓から線が外れ、半分近くが外れ値になる → 最初の推定はこれで行い、直した姿勢で線を探し直す。"""
    pts = np.asarray(points_uv, float)
    pts = pts[~np.isnan(pts).any(axis=1)]
    if len(pts) < 8:
        return float("nan"), float("nan"), float("nan")
    pts = pts[np.linspace(0, len(pts) - 1, min(n_points, len(pts))).astype(int)]
    keep = np.ones(len(pts), bool)
    best = (0.0, 0.0)

    def cost(g: tuple[float, float]) -> float:
        r2 = np.sort(line_residual_px(pts[keep], cam_nom, plane_nom, g[0], g[1]) ** 2)
        return float(np.mean(r2[:max(8, int(round(trim * r2.size)))]))
    def refine(start: tuple[float, float]) -> tuple[float, float]:
        g0 = start
        for k in range(5):                                      # 粗い格子から 4 倍ずつ細かく（最後の刻み 約 0.004mm / 0.002°）
            span_h, span_p = dh_range / 4 ** k, dp_range / 4 ** k
            grid = [(g0[0] + a, g0[1] + b) for a in np.linspace(-span_h, span_h, 9) for b in np.linspace(-span_p, span_p, 9)]
            g0 = min(grid, key=cost)
        return g0
    best = refine(best)
    # 外れ値（物で持ち上がった点）は、収束してから 1 回だけ外して、もう一度合わせる（粗い段で外すと正しい点まで捨てる）
    res = line_residual_px(pts, cam_nom, plane_nom, best[0], best[1])
    keep = res <= max(3.0 * float(np.quantile(res, min(trim, 1.0) / 2)), 2.0)
    if keep.sum() >= 8 and not keep.all():
        best = refine(best)
    return best[0], best[1], float(np.sqrt(cost(best)))
