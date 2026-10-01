"""H2 ライン光の置き方の幾何（GEOMETRY_SIM）。模型の中身が正しいかだけを見る（実機の性能ではない）。"""
from __future__ import annotations

import numpy as np
import pytest

from serpens.config import load_config
from serpens.floorwatch.geometry import Camera, LightPlane
from simulation.h2_line_light import (LinePlacement, aim_travel_mm_per_deg, calibration_error_mm, evaluate,
                                      placements, _line_samples)


@pytest.fixture(scope="module")
def cfg() -> dict:
    return load_config()


@pytest.fixture(scope="module")
def cam(cfg: dict) -> Camera:
    return Camera.from_cfg(cfg)


def _by_name(cfg: dict, name: str) -> LinePlacement:
    return next(p for p in placements(cfg) if p.name == name)


def test_current_cheek_placement_is_the_design_plane(cfg: dict) -> None:
    a, b = _by_name(cfg, "cheek_current").plane(), LightPlane.design(cfg)
    assert np.allclose(np.abs(a.normal), np.abs(b.normal)) and abs(a.d - b.d) < 1e-9


@pytest.mark.parametrize("name", ["cheek_current", "brow_z64_y71"])
def test_a_raised_point_on_the_plane_reads_back_its_height(cfg: dict, cam: Camera, name: str) -> None:
    """一般の面（縦基線の眉も）で、面の上の高さ H の点の視線と面の交点が H に戻る。"""
    lp = _by_name(cfg, name)
    plane = lp.plane()
    s, p0, d = (np.asarray(v, float) for v in (lp.source_mm, lp.floor_point_mm, lp.floor_dir))
    for h in (0.0, 1.5, 5.0):
        q = s + (s[2] - h) / s[2] * (p0 - s) + 3.0 * d
        u, v, _ = cam.project(q)
        assert plane.height_at(cam, u, v) == pytest.approx(h, abs=1e-6)


def test_no_calibration_error_means_no_height_error(cfg: dict, cam: Camera) -> None:
    for name in ("cheek_current", "brow_z64_y71"):
        lp = _by_name(cfg, name)
        pts = _line_samples(cam, lp)
        assert calibration_error_mm(cam, lp, pts, 0.0, "yaw", 5.0) < 1e-6


def test_both_placements_resolve_well_below_the_object_threshold(cfg: dict, cam: Camera) -> None:
    """感度の比較: どちらも出っ張りの閾値（height_object_min_mm）より 1 桁以上細かく測れる（線の重心 0.2px を仮定）。"""
    thr = float(cfg["floor_watch"]["detect"]["height_object_min_mm"])
    for lp in placements(cfg):
        r = evaluate(cam, lp)
        assert r["visible"] and r["sigma_h_mm_worst"] < thr / 10, r


def test_brow_line_is_aimed_by_neck_pitch_and_cheek_line_by_head_yaw(cfg: dict) -> None:
    """頭を回すと線は床の上を掃く: 眉の線は J1 首 pitch で前後、頬の線は頭 yaw で左右に動く（数 mm/°）。"""
    brow, cheek = _by_name(cfg, "brow_z64_y71"), _by_name(cfg, "cheek_current")
    assert brow.aim_by == "neck_pitch" and cheek.aim_by == "head_yaw"
    for lp in (brow, cheek):
        assert 1.0 < aim_travel_mm_per_deg(lp, 60.0) < 10.0
