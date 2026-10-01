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


def test_defocused_far_field_line_dropout_is_not_an_object_but_near_specular_still_is(h2) -> None:
    """OQ-0111 / LB-E-009。2026-09-29 の HG-H2 の発見の回帰（farfield-roi `0e9b79a` のシナリオを、新しい設定キー無しで）:
    nominal（UXGA・100mm 合焦・口径 1mm）では遠方で線光がぼけて消え、その途切れが 1m 先・直径 130mm の specular_break の偽物になっていた。
    統合 base の検出（vision-sim の修正）では、汚れだけの床で `reach_mm` より遠い物の候補は出ない。近くの鏡面（ボタン電池）は同じぼけの下でも見つかる。
    **SYNTHETIC_SENSOR_SIM。実カメラでは未確認。**"""
    from serpens.config import load_config
    from serpens.floorwatch.detect import detect
    from serpens.floorwatch.geometry import LightPlane
    from serpens.floorwatch.synthetic import Disc, Renderer, Scene, Stain
    p = dict(h2.A["nominal"])
    cfg = h2.cfg_for(load_config(), p)
    reach = float(cfg["floor_watch"]["mission"]["reach_mm"])
    cam = h2.camera(p)
    plane = LightPlane.design(cfg)
    rend = Renderer(cam, plane, h2.lighting(p))
    deg = h2.Degrader(cam, rend, p)
    yc = h2.view_center_mm(p)
    stain = rend.render(Scene(stains=[Stain(0.0, yc, 18.0, 0.3)], floor_albedo=float(p["floor_albedo"])))
    for seed in (2, 4, 5, 6, 7, 8):                               # 劣化の雑音次第で出たり出なかったりした（farfield は 6 通り中 3）
        frames = {k: deg(v, np.random.default_rng(seed)) for k, v in stain.items()}
        far = [c for c in detect(frames, cam, plane, cfg)[0] if c.is_object and math.hypot(*c.floor_xy_mm) > reach]
        assert not far, [(c.floor_xy_mm, c.diameter_mm, c.height_reason) for c in far]
    cell = rend.render(Scene([Disc(0.0, yc, 20.0, 3.2, 0.75, True, "button_cell")], floor_albedo=float(p["floor_albedo"])))
    frames = {k: deg(v, np.random.default_rng(2)) for k, v in cell.items()}
    objs = [c for c in detect(frames, cam, plane, cfg)[0] if c.is_object]
    hit = [c for c in objs if abs(c.floor_xy_mm[0]) < 12.0 and abs(c.floor_xy_mm[1] - yc) < 25.0]
    assert hit and any(k["kind"] == "metal_disc" for k in hit[0].kinds)
