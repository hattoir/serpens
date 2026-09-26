"""Floor Watch フェーズ 2: 幾何・較正・合成画像での検出・大きさ・危険度・データセット評価。

**合成画像（SIMULATED）でパイプラインの筋が通っているかだけを確かめる。実写の性能ではない。**
レビュー 2026-09-26: 測れない高さは None + 理由 / 測れない = 出っ張り扱い / metal_disc は危険物側 / 基準床なし /
撮影順と動き検出 / あごの斜め照明（影は奥） / 2 段評価 / 継ぎ目の合成。
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from serpens.config import load_config
from serpens.floorwatch.dataset import SYNTHETIC_ITEMS, evaluate, make_synthetic, read_samples, wilson
from serpens.floorwatch.detect import MotionError, detect, scaled_thresholds
from serpens.floorwatch.geometry import Camera, LightPlane
from serpens.floorwatch.risk import assess, size_factor
from serpens.floorwatch.synthetic import Disc, Renderer, Scene, Seam, Stain, default_lighting


@pytest.fixture(scope="module")
def cfg() -> dict:
    return load_config()


REF_MODE = "synthetic_ref"          # 合成の回帰試験はレビューの設計値（f=1000、640×480）で回す。実機モードは幾何のテストで確認


@pytest.fixture(scope="module")
def rig(cfg: dict):
    cam, plane, lt = Camera.from_cfg(cfg, REF_MODE), LightPlane.design(cfg), default_lighting(cfg)
    return cam, plane, Renderer(cam, plane, lt)


def objects(fr, cam, plane, cfg, with_line=True):
    cands, tr, _ = detect(fr, cam, plane, cfg, with_line=with_line)
    return [c for c in cands if c.is_object], cands, tr


# ---- 幾何 ------------------------------------------------------------------------------
def test_geometry_matches_the_design_numbers(cfg: dict) -> None:
    """レビューの目安: f=1000px・高さ 30mm・下向き 25° で視野中心は床 71mm 先、1.5mm の段が約 21px。"""
    cam, plane = Camera.from_cfg(cfg, REF_MODE), LightPlane.design(cfg)
    p = cam.floor_point(cam.cx, cam.cy)
    assert np.linalg.norm(p - cam.center) == pytest.approx(71.0, abs=0.5)
    # 実機モード（UXGA 静止画、f ≈ 1256 ASSUMED）では 1.5mm → 約 27px、0.3mm → 約 5px。しきい値は f に比例して拡大する
    hw = Camera.from_cfg(cfg)
    assert hw.width_px == 1600 and hw.f_px == pytest.approx(1600 / 2 / np.tan(np.radians(32.5)), rel=0.01)
    uh, vh, _ = hw.project(np.array([1.5, p[1], 1.5]))
    assert uh - hw.cx == pytest.approx(21.3 * hw.f_px / 1000.0, rel=0.03)
    sc = scaled_thresholds(cfg, hw)
    assert sc["line_search_px"] == round(120 * hw.f_px / 1000.0) and sc["min_blob_px"] == round(30 * (hw.f_px / 1000.0) ** 2)
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


# ---- 合成画像での検出（基準床なし） --------------------------------------------------------
def test_coin_on_the_line_gets_height_shadow_behind_and_size(cfg: dict, rig) -> None:
    cam, plane, r = rig
    fr = r.render(Scene([Disc(0.0, 64.0, 20.0, 1.5, 0.85, False, "coin")], seed=1))
    objs, _, tr = objects(fr, cam, plane, cfg)
    assert len(objs) == 1
    c = objs[0]
    assert c.height_reason == "measured" and c.height_mm == pytest.approx(1.5, abs=0.2)
    assert c.shadow and not c.line_dropout and c.on_line
    assert abs(c.diameter_mm - 20.0) <= 3.0                                            # 大きさ（画素 → mm）
    assert abs(c.floor_xy_mm[0]) < 5.0 and abs(c.floor_xy_mm[1] - 64.0) < 12.0
    assert 20 < tr.width_px < 80                                                       # 線幅は自動推定（初期値 ≈ 40px）
    # あごの照明: 影は物の奥 = 画像の上側にある
    _, _, fg = detect(fr, cam, plane, cfg)
    y0 = c.bbox_px[1]
    dark = np.float32(fr["dark"])
    ratio = (np.float32(fr["raking"]) - dark) / np.maximum(np.float32(fr["normal"]) - dark, 1.0)
    x0, x1 = c.bbox_px[0], c.bbox_px[0] + c.bbox_px[2]
    assert ratio[max(0, y0 - 40):y0, x0:x1].mean() < 0.7 < ratio[y0 + c.bbox_px[3] + 5:y0 + c.bbox_px[3] + 40, x0:x1].mean()


def test_specular_object_records_height_none_with_reason_and_becomes_metal_disc(cfg: dict, rig) -> None:
    """1 + 2: 鏡面（ボタン電池相当）では線が乗らない。高さは 0 ではなく None + specular_break。円形 + 5〜25mm → metal_disc。"""
    cam, plane, r = rig
    fr = r.render(Scene([Disc(0.0, 64.0, 20.0, 1.5, 0.9, True, "washer")], seed=2))
    objs, _, _ = objects(fr, cam, plane, cfg)
    c = objs[0]
    assert c.line_dropout and c.height_mm is None and c.height_reason == "specular_break"
    assert any(k["kind"] == "metal_disc" for k in c.kinds)
    risk = assess(c.kinds, c.diameter_mm, c.height_mm, c.height_reason, None, True, cfg)
    assert risk["mandatory_notify"] and "metal_disc" in risk["critical_kinds"]


def test_stain_pattern_and_seams_are_not_objects(cfg: dict, rig) -> None:
    cam, plane, r = rig
    fr = r.render(Scene(stains=[Stain(0.0, 64.0, 18.0, 0.3)], seed=3))
    objs, cands, _ = objects(fr, cam, plane, cfg)
    assert cands and not objs                                                          # 見えるが出っ張りではない
    assert all(c.height_reason != "measured" or abs(c.height_mm) < 0.3 for c in cands) and not any(c.shadow for c in cands)
    for seam in (Seam("y", 0.0, 0.3, "groove"), Seam("y", 0.0, 0.5, "step"), Seam("x", 64.0, 0.5, "step"),
                 Seam("x", 64.0, 0.5, "groove")):
        fr = r.render(Scene(seams=[seam], seed=4))
        for with_line in (False, True):
            objs, cands, _ = objects(fr, cam, plane, cfg, with_line)
            assert not objs, (seam, with_line, [(c.bbox_px, c.rationale) for c in objs])
            assert all(c.shape == "line" for c in cands if c.bbox_px[2] > 100)


def test_capture_motion_check_rejects_a_moved_sequence(cfg: dict, rig) -> None:
    """4: 最初と最後の通常画像が motion_max_px 以上ずれたら撮り直し（照明の違う画像同士は合わせない）。"""
    cam, plane, r = rig
    fr = r.render(Scene([Disc(0.0, 64.0, 20.0, 1.5)], seed=5))
    detect(fr, cam, plane, cfg)                                                        # 静止 → 通る
    moved = {**fr, "normal2": np.roll(fr["normal2"], 3, axis=1)}
    with pytest.raises(MotionError):
        detect(moved, cam, plane, cfg)


def test_patrol_finds_a_floor_coloured_object_from_its_shadow_alone(cfg: dict, rig) -> None:
    """6: 巡回中（線なし）は影だけが手掛かり。床と同じ色の物でも影の手前に候補が立つ。"""
    cam, plane, r = rig
    fr = r.render(Scene([Disc(-8.0, 80.0, 15.0, 3.0, 0.5, True, "unknown")], seed=6))
    objs, _, _ = objects(fr, cam, plane, cfg, with_line=False)
    assert objs and any(abs(c.floor_xy_mm[0] + 8.0) < 12.0 and abs(c.floor_xy_mm[1] - 80.0) < 25.0 for c in objs)
    assert all(c.height_reason == "off_line" and c.height_mm is None for c in objs)


# ---- 危険度 ------------------------------------------------------------------------------
def test_risk_is_composite_unmeasured_height_counts_as_raised(cfg: dict) -> None:
    lo, hi = cfg["floor_watch"]["risk"]["ingestion_size_mm"]
    assert size_factor(20.0, lo, hi) == 1.0 and size_factor(60.0, lo, hi) < 0.6 and size_factor(1.0, lo, hi) < 0.5
    coin = assess([{"kind": "coin", "confidence": 0.8}], 20.0, 1.5, "measured", None, True, cfg)
    far = assess([{"kind": "coin", "confidence": 0.8}], 20.0, 1.5, "measured", 6.0, False, cfg)
    assert coin["score"] > far["score"] and coin["child_distance_m"] is None and not coin["mandatory_notify"]
    unmeasured = assess([{"kind": "coin", "confidence": 0.8}], 20.0, None, "specular_break", None, True, cfg)
    assert unmeasured["score"] == coin["score"]                                        # 2: 測れない = 出っ張り扱い（×1.0）
    assert any("高さ不明" in s for s in unmeasured["rationale"])
    crumb = assess([{"kind": "food_crumb", "confidence": 0.9}], 4.0, 2.0, "measured", None, True, cfg)
    assert crumb["score"] < coin["score"]
    cell = assess([{"kind": "coin", "confidence": 0.7}, {"kind": "button_cell", "confidence": 0.1}], 20.0, None,
                  "specular_break", None, True, cfg)
    assert cell["mandatory_notify"] and cell["critical_kinds"] == ["button_cell"] and cell["score"] < 1.0
    weak = assess([{"kind": "magnet", "confidence": 0.01}], 10.0, None, "off_line", None, True, cfg)
    assert not weak["mandatory_notify"]                                                 # 最低確信度は要る


# ---- データセット（2 段評価） ------------------------------------------------------------------
def test_synthetic_dataset_two_stage_eval_and_confidence_intervals(cfg: dict, rig, tmp_path: Path) -> None:
    """合成データでの検出率・誤報率（合成での目安。実写の性能ではない）。基準床の画像は無い。"""
    cam, plane, r = rig
    root = make_synthetic(tmp_path / "ds", cam, plane, default_lighting(cfg), seed=5)
    assert not list(root.glob("reference_*.png"))
    samples = read_samples(root)
    assert len(samples) == 6 * (len(SYNTHETIC_ITEMS) + 3) + 10
    assert all(set(s.frames) == {"normal", "raking", "line", "dark", "normal2"} for s in samples)
    res = evaluate(root, cam, plane, cfg)
    assert res["source"] == "SYNTHETIC"
    for st in ("patrol", "inspect"):
        assert res["stages"][st]["detection_rate"] >= 0.85, res["stages"][st]
        assert res["stages"][st]["false_alarm_rate"] == 0.0, res["stages"][st]
        assert res["stages"][st]["neg"] >= 25
    # 線に乗った物の高さ。線の面 x = z は高さ H で横へ H ずれるので、上面に線が届く物（H + 線の半幅 ≤ 半径）だけ比べる
    half_w = cfg["floor_watch"]["line_light"]["width_mm_initial"] / 2
    hits = [row for row in res["rows"] if row["inspect_hit"] and row["height_reason"] == "measured" and row["on_line"]
            and row["label_h"] + half_w <= row["label_d"] / 2]
    assert hits and all(abs(row["height_est_mm"] - row["label_h"]) < 1.0 for row in hits), hits
    assert any(row["metal_disc"] for row in res["rows"] if row["kind"] == "washer")
    assert wilson(19, 21)[0] == pytest.approx(0.71, abs=0.02) and wilson(0, 9)[1] == pytest.approx(0.30, abs=0.02)
