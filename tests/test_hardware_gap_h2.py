"""HG-H2 センサーヘッドの道具（劣化・カメラ・実測の取り込み）。**画像の値は作り物（FAKE）。**"""
from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope="module")
def h2():
    name = "hg_h2_run"
    spec = importlib.util.spec_from_file_location(name, ROOT / "simulation/hardware_gaps/HG-H2_sensor_head/run.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def _small(h2, **kw) -> dict:
    return {**h2.A["nominal"], "width_px": 320, **kw}


def test_camera_focal_length_follows_the_fov(h2) -> None:
    cam = h2.camera(_small(h2, fov_h_deg=90.0))
    assert cam.f_px == pytest.approx(160.0)                       # W/2 / tan(45°)
    assert cam.height_px == 240


def test_degrader_is_identity_when_everything_is_off(h2) -> None:
    from serpens.config import load_config
    from serpens.floorwatch.geometry import LightPlane
    from serpens.floorwatch.synthetic import Renderer
    p = _small(h2, aperture_mm=0.0, distortion_k1=0.0, motion_blur_px=0.0, exposure_gain=1.0, read_noise=0.0, shot_noise_k=0.0)
    cfg = h2.cfg_for(load_config(), p)
    cam = h2.camera(p)
    deg = h2.Degrader(cam, Renderer(cam, LightPlane.design(cfg), h2.lighting(p)), p)
    img = (np.arange(cam.width_px * cam.height_px) % 200).reshape(cam.height_px, cam.width_px).astype(np.uint8)
    assert np.array_equal(deg(img, np.random.default_rng(0)), img)


def test_defocus_grows_away_from_the_focus_distance(h2) -> None:
    from serpens.config import load_config
    from serpens.floorwatch.geometry import LightPlane
    from serpens.floorwatch.synthetic import Renderer
    sig = []
    for focus in (70.0, 5000.0):
        p = _small(h2, focus_mm=focus)
        cfg = h2.cfg_for(load_config(), p)
        cam = h2.camera(p)
        sig.append(h2.Degrader(cam, Renderer(cam, LightPlane.design(cfg), h2.lighting(p)), p).sigma_center)
    assert sig[1] > sig[0]                                        # 中心（約 71 mm 先）は 70 mm 合焦の方が鮮明


def test_ingest_fov_and_focus_recover_the_parameters(h2, tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr(h2, "HERE", tmp_path)
    monkeypatch.setattr(h2, "ROOT", tmp_path)
    f_true, w = 1256.0, 1600
    fov = tmp_path / "fov.csv"
    fov.write_text("distance_mm,ruler_mm,ruler_px,width_px\n"
                   + "".join(f"{d},50,{f_true * 50 / d:.3f},{w}\n" for d in (100, 300)), encoding="utf-8")
    out = h2.ingest_fov(fov)
    assert out["f_px"] == pytest.approx(f_true, rel=1e-3)
    assert out["fov_h_deg"] == pytest.approx(math.degrees(2 * math.atan(800 / f_true)), rel=1e-3)
    a_true, fo_true = 1.2, 150.0
    ds = (50, 70, 100, 200, 300, 500)
    focus = tmp_path / "focus.csv"
    focus.write_text("distance_mm,blur_px,f_px\n" + "".join(
        f"{d},{a_true * f_true * abs(1 / fo_true - 1 / d) / 2.5 * 2.56:.4f},{f_true}\n" for d in ds), encoding="utf-8")
    fit = h2.ingest_focus(focus)
    assert fit["focus_mm"] == pytest.approx(fo_true, rel=0.05)
    assert fit["aperture_mm"] == pytest.approx(a_true, rel=0.05)
    assert (tmp_path / "results" / "measured" / "raw").exists()
    assert list((tmp_path / "ai-outbox" / "handoffs").glob("*_HG-H2_measured.md"))
