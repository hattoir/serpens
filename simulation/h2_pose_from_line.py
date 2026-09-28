"""H2 VIS-0003: 床の線から、カメラの高さと pitch のずれを推定する（線を姿勢センサとしても使う）。**SYNTHETIC_VISION_SIM。**

光の面は頭に固定なので、床の線が画像のどこに写るかは、床に対するカメラの高さと pitch で決まる。
画像の線は 2 つの量（位置・傾き）を持つので、2 つの未知（Δ高さ・Δpitch）が解けるかもしれない。解ければ、
検出した候補の床の位置（名目のカメラで床へ戻している）を直せる。

やり方: 検出の `trace_line` が返す行ごとの線の列 u（物の無い行だけ）に、候補の姿勢（Δh, Δp）で予測した床の線の列を
最小二乗で合わせる（粗い格子 → 細かい格子）。

限界: 床が平らで、線の大半が床に乗っている前提。横の傾き（roll）と画角のずれは扱わない（未知を増やすと解けない見込み）。
"""
from __future__ import annotations

from dataclasses import replace

import numpy as np

from serpens.floorwatch.detect import LineTrace
from serpens.floorwatch.geometry import Camera, LightPlane
from simulation.h2_vision import plane_for_true_camera

SOURCE = "SYNTHETIC_VISION_SIM"


def predicted_u(cam_nom: Camera, plane_nom: LightPlane, dh: float, dp: float, rows: np.ndarray) -> np.ndarray:
    """姿勢 (名目 + dh, dp) のとき、床の線が行 rows に写る列（光の面は頭に固定）。"""
    cam = replace(cam_nom, height_mm=cam_nom.height_mm + dh, pitch_deg=cam_nom.pitch_deg + dp)
    pl = plane_for_true_camera(plane_nom, cam_nom, cam)
    return np.array([pl.line_u_on_floor(cam, float(v)) or np.nan for v in rows])


def estimate_pose(tr: LineTrace, cam_nom: Camera, plane_nom: LightPlane, n_rows: int = 40,
                  dh_range: float = 12.0, dp_range: float = 6.0) -> tuple[float, float, float]:
    """(dh_mm, dp_deg, rms_px)。線が見えている行から n_rows を等間隔に使う（外れ値は中央値からの残差で除く）。"""
    rows = np.flatnonzero(~np.isnan(tr.u_line))
    if rows.size < 8:
        return float("nan"), float("nan"), float("nan")
    rows = rows[np.linspace(0, rows.size - 1, min(n_rows, rows.size)).astype(int)]
    obs = tr.u_line[rows]

    def cost(dh: float, dp: float, keep: np.ndarray) -> float:
        pred = predicted_u(cam_nom, plane_nom, dh, dp, rows)
        r = (obs - pred)[keep]
        r = r[~np.isnan(r)]
        return float(np.mean(r * r)) if r.size else float("inf")

    keep = np.ones(rows.size, bool)
    best = (0.0, 0.0)
    for span_h, span_p, n in ((dh_range, dp_range, 9), (dh_range / 4, dp_range / 4, 9), (dh_range / 16, dp_range / 16, 9)):
        grid = [(best[0] + a, best[1] + b) for a in np.linspace(-span_h, span_h, n) for b in np.linspace(-span_p, span_p, n)]
        best = min(grid, key=lambda g: cost(g[0], g[1], keep))
        res = np.abs(obs - predicted_u(cam_nom, plane_nom, best[0], best[1], rows))
        med = np.nanmedian(res)
        keep = res <= max(3.0 * med, 2.0)                    # 物で持ち上がった行を外す
    return best[0], best[1], float(np.sqrt(cost(best[0], best[1], keep)))
