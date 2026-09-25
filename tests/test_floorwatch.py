"""Floor Watch フェーズ 2: 幾何・較正・合成画像での検出・大きさ・危険度・データセット評価。

**合成画像（SIMULATED）でパイプラインの筋が通っているかだけを確かめる。実写の性能ではない。**
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from serpens.config import load_config
from serpens.floorwatch.dataset import SYNTHETIC_ITEMS, evaluate, make_synthetic, read_samples
from serpens.floorwatch.detect import align, detect, trace_line
from serpens.floorwatch.geometry import Camera, LightPlane
from serpens.floorwatch.risk import assess, size_factor
from serpens.floorwatch.synthetic import Disc, Renderer, Scene, Stain, default_lighting


@pytest.fixture(scope="module")
def cfg() -> dict:
    return load_config()


@pytest.fixture(scope="module")
def rig(cfg: dict):
    cam, plane, lt = Camera.from_cfg(cfg), LightPlane.design(cfg), default_lighting(cfg)
    return cam, plane, Renderer(cam, plane, lt)


# ---- 幾何 ------------------------------------------------------------------------------
def test_geometry_matches_the_design_numbers(cfg: dict) -> None:
    """レビューの目安: f=1000px・高さ 30mm・下向き 25° で視野中心は床 71mm 先、1.5mm の段が約 21px。"""
    cam, plane = Camera.from_cfg(cfg), LightPlane.design(cfg)
    p = cam.floor_point(cam.cx, cam.cy)
    assert np.linalg.norm(p - cam.center) == pytest.approx(71.0, abs=0.5)
    assert plane.line_u_on_floor(cam, cam.cy) == pytest.approx(cam.cx, abs=0.01)     # 床の線は中心線の真下
    u, v, _ = cam.project(np.array([1.5, p[1], 1.5]))                                # 1 円玉の上面に乗った線
    assert u - cam.cx == pytest.approx(21.3, abs=0.5)
    assert plane.height_at(cam, u, v) == pytest.approx(1.5, abs=1e-6)                # 往復
    u2, v2, _ = cam.project(np.array([0.3, p[1], 0.3]))                              # 床の 0.3mm 段差 ≈ 4px
    assert u2 - cam.cx == pytest.approx(4.3, abs=0.4)
    assert cam.mm_per_px_at(cam.cx, cam.cy) == pytest.approx(1 / 14.1, rel=0.05)


def test_light_plane_is_calibrated_from_coin_stacks_not_the_nominal_baseline(cfg: dict) -> None:
    """較正: 1 円玉 1・2・3 枚の段で光の面を当てはめ、名目値と違う実際の面でも高さが戻る。"""
    cam = Camera.from_cfg(cfg)
    truth = LightPlane(np.array([np.cos(np.radians(40.0)), 0.02, -np.sin(np.radians(40.0))]) / np.linalg.norm(
        [np.cos(np.radians(40.0)), 0.02, np.sin(np.radians(40.0))]), 0.6)             # 名目 45° からずれた面
    thick = cfg["floor_watch"]["calibration"]["coin_thickness_mm"]
    samples = []
    for n in cfg["floor_watch"]["calibration"]["stack_counts"] + [0]:
        h = n * thick
        for y in (50.0, 65.0, 85.0):
            x = (truth.d - truth.normal[1] * y - truth.normal[2] * h) / truth.normal[0]   # 面上の点（高さ h）
            u, v, _ = cam.project(np.array([x, y, h]))
            samples.append((u, v, h))
    fit = LightPlane.fit(cam, samples)
    for u, v, h in samples:
        assert fit.height_at(cam, u, v) == pytest.approx(h, abs=0.05)
    with pytest.raises(ValueError):
        LightPlane.fit(cam, samples[:2])


# ---- 合成画像での検出 --------------------------------------------------------------------
def test_coin_on_the_line_gets_height_shadow_and_size(cfg: dict, rig) -> None:
    cam, plane, r = rig
    ref = r.render(Scene(seed=1))["normal"]
    fr = r.render(Scene([Disc(0.0, 64.0, 20.0, 1.5, 0.85, False, "coin")], seed=1))
    cands, tr, _ = detect(fr, np.clip(np.float32(ref) - np.float32(fr["dark"]), 0, None), cam, plane, cfg)
    objs = [c for c in cands if c.is_object]
    assert len(objs) == 1
    c = objs[0]
    assert c.height_measured and c.height_mm == pytest.approx(1.5, abs=0.2)
    assert c.shadow and not c.line_dropout
    assert abs(c.diameter_mm - 20.0) <= 3.0                                            # 大きさ（画素 → mm）
    assert abs(c.floor_xy_mm[0]) < 5.0 and abs(c.floor_xy_mm[1] - 64.0) < 12.0
    assert 20 < tr.width_px < 80                                                       # 線幅は自動推定（初期値 ≈ 40px）


def test_specular_object_drops_the_line_and_still_counts_as_object(cfg: dict, rig) -> None:
    """鏡面（ボタン電池・磁石相当）では線が乗らない。途切れを出っ張りの証拠として扱い、高さは測れないと記録する。"""
    cam, plane, r = rig
    ref = r.render(Scene(seed=2))["normal"]
    fr = r.render(Scene([Disc(0.0, 64.0, 20.0, 1.5, 0.9, True, "washer")], seed=2))
    cands, _, _ = detect(fr, np.clip(np.float32(ref) - np.float32(fr["dark"]), 0, None), cam, plane, cfg)
    c = [c for c in cands if c.is_object][0]
    assert c.line_dropout and not c.height_measured and c.height_mm == 0.0
    assert any("途切れ" in s for s in c.rationale)


def test_stain_and_pattern_are_not_objects(cfg: dict, rig) -> None:
    cam, plane, r = rig
    ref = r.render(Scene(seed=3))["normal"]
    fr = r.render(Scene(stains=[Stain(0.0, 64.0, 18.0, 0.3)], seed=3))
    cands, _, _ = detect(fr, np.clip(np.float32(ref) - np.float32(fr["dark"]), 0, None), cam, plane, cfg)
    assert cands and all(not c.is_object for c in cands)                              # 見えるが出っ張りではない
    assert all(c.height_mm == 0.0 and not c.shadow for c in cands)


def test_align_ignores_unrelated_content_and_fixes_small_shifts(rig) -> None:
    cam, plane, r = rig
    fr = r.render(Scene([Disc(0.0, 64.0, 20.0, 1.5)], seed=4))
    shifted = np.roll(fr["normal"], 3, axis=1)
    back = align(fr["normal"], shifted)
    assert np.mean(np.abs(np.float32(back[:, 10:-10]) - np.float32(fr["normal"][:, 10:-10]))) < 3.0
    assert align(fr["normal"], fr["line"]) is fr["line"]                                # 線光は合わせない


# ---- 危険度 ------------------------------------------------------------------------------
def test_risk_is_composite_and_critical_kinds_are_separate(cfg: dict) -> None:
    lo, hi = cfg["floor_watch"]["risk"]["ingestion_size_mm"]
    assert size_factor(20.0, lo, hi) == 1.0 and size_factor(60.0, lo, hi) < 0.6 and size_factor(1.0, lo, hi) < 0.5
    coin = assess([{"kind": "coin", "confidence": 0.8}], 20.0, 1.5, True, False, None, True, cfg)
    far = assess([{"kind": "coin", "confidence": 0.8}], 20.0, 1.5, True, False, 6.0, False, cfg)
    assert coin["score"] > far["score"] and coin["child_distance_m"] is None and not coin["mandatory_notify"]
    crumb = assess([{"kind": "food_crumb", "confidence": 0.9}], 4.0, 2.0, True, False, None, True, cfg)
    assert crumb["score"] < coin["score"]
    cell = assess([{"kind": "coin", "confidence": 0.7}, {"kind": "button_cell", "confidence": 0.1}], 20.0, 0.0, False, True,
                  None, True, cfg)
    assert cell["mandatory_notify"] and cell["critical_kinds"] == ["button_cell"]
    assert cell["score"] < 1.0 and any("線の途切れ" in s for s in cell["rationale"])
    weak = assess([{"kind": "magnet", "confidence": 0.01}], 10.0, 0.0, False, False, None, True, cfg)
    assert not weak["mandatory_notify"]                                                 # 最低確信度は要る


# ---- データセット ----------------------------------------------------------------------------
def test_synthetic_dataset_round_trips_and_meets_the_synthetic_bar(cfg: dict, rig, tmp_path: Path) -> None:
    """合成データでの検出率・誤報率（合成での目安。実写の性能ではない）。"""
    cam, plane, r = rig
    root = make_synthetic(tmp_path / "ds", cam, plane, default_lighting(cfg), seed=5)
    samples = read_samples(root)
    assert len(samples) == 3 * (len(SYNTHETIC_ITEMS) + 3)
    assert all(s.frames[k] is not None for s in samples for k in ("dark", "normal", "raking", "line"))
    res = evaluate(root, cam, plane, cfg)
    assert res["source"] == "SYNTHETIC"
    assert res["detection_rate"] >= 0.85, res
    assert res["false_alarm_rate"] == 0.0, res
    hits = [row for row in res["rows"] if row["hit"] and row["height_measured"]]
    assert hits and all(abs(row["height_est_mm"] - row["label_h"]) < 1.0 for row in hits)  # 線に乗った物の高さ（小物は縁で偏る）
