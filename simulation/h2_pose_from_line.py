"""H2 VIS-0003: 床の線から、カメラの高さと pitch のずれを推定する（線を姿勢センサとしても使う）。**SYNTHETIC_VISION_SIM。**

本体は `serpens/floorwatch/pose.py`（検出で使う）。ここは検出の `LineTrace`（行ごとの線の列）を点に直して渡すだけ。
"""
from __future__ import annotations

import numpy as np

from serpens.floorwatch.detect import LineTrace
from serpens.floorwatch.geometry import Camera, LightPlane
from serpens.floorwatch.pose import estimate_pose as _estimate

SOURCE = "SYNTHETIC_VISION_SIM"


def estimate_pose(tr: LineTrace, cam_nom: Camera, plane_nom: LightPlane, n_rows: int = 40,
                  dh_range: float = 12.0, dp_range: float = 6.0) -> tuple[float, float, float]:
    """(dh_mm, dp_deg, rms_px)。線が見えている行の (u, v) を使う。"""
    ok = ~np.isnan(tr.u_line)
    pts = np.stack([tr.u_line[ok], tr.rows[ok].astype(float)], axis=1)
    return _estimate(pts, cam_nom, plane_nom, dh_range, dp_range, n_rows)
