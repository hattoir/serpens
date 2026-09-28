"""床の線からの姿勢推定（serpens/floorwatch/pose.py）。幾何の中身だけを見る（実機の性能ではない）。"""
from __future__ import annotations

import numpy as np
import pytest

from serpens.config import load_config
from serpens.floorwatch.geometry import Camera, LightPlane
from serpens.floorwatch.pose import estimate_pose, posed_camera


@pytest.fixture(scope="module")
def rig():
    cfg = load_config()
    return Camera.from_cfg(cfg), LightPlane.design(cfg)          # 床見のモード（config の floor_mode）


def test_floor_line_image_matches_the_per_row_bisection(rig) -> None:
    cam, plane = rig
    a, d = plane.floor_line_image(cam)
    for v in (700.0, 900.0, 1100.0):
        u = plane.line_u_on_floor(cam, v)
        t = (v - a[1]) / d[1]
        assert a[0] + t * d[0] == pytest.approx(u, abs=1e-6)


def test_fixed_to_head_is_identity_for_the_same_pose(rig) -> None:
    cam, plane = rig
    p = plane.fixed_to_head(cam, cam)
    assert np.allclose(p.normal, plane.normal) and p.d == pytest.approx(plane.d)


@pytest.mark.parametrize("dh,dp", [(-6.0, -2.0), (4.0, 1.5), (0.0, 0.0), (7.0, -3.0)])
def test_pose_is_recovered_from_noise_free_floor_line_points(rig, dh: float, dp: float) -> None:
    cam, plane = rig
    true = posed_camera(cam, dh, dp)
    pl = plane.fixed_to_head(cam, true)
    a, d = pl.floor_line_image(true)
    pts = np.array([a + t * d for t in np.linspace(-400, 800, 120)])          # 画像に写る範囲の線の点（線の全長）
    pts = pts[(pts[:, 1] > 0) & (pts[:, 1] < cam.height_px) & (pts[:, 0] > 0) & (pts[:, 0] < cam.width_px)]
    eh, ep, rms = estimate_pose(pts, cam, plane)
    assert eh == pytest.approx(dh, abs=0.3) and ep == pytest.approx(dp, abs=0.1) and rms < 0.5


def test_too_few_points_gives_nan(rig) -> None:
    cam, plane = rig
    assert np.isnan(estimate_pose(np.zeros((3, 2)), cam, plane)[0])
