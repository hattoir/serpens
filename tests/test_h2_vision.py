"""H2 の合成視覚（SYNTHETIC_VISION_SIM）: 描画の拡張が従来を変えないこと、側面を描いたときの検出の穴を塞いだこと。
実写・実機の性能ではない。"""
from __future__ import annotations

import numpy as np
import pytest

from serpens.config import load_config
from serpens.floorwatch.detect import detect
from serpens.floorwatch.geometry import Camera, LightPlane
from serpens.floorwatch.synthetic import Disc, Lighting, Renderer, Scene, default_lighting
from simulation.h2_vision import Condition, plane_for_true_camera, run_trial, scaled_camera, true_camera


@pytest.fixture(scope="module")
def cfg() -> dict:
    return load_config()


@pytest.fixture(scope="module")
def setup(cfg: dict):
    cam = scaled_camera(Camera.from_cfg(cfg), 0.5)
    return cam, LightPlane.design(cfg), default_lighting(cfg)


def test_new_render_options_default_to_the_old_images(setup) -> None:
    """既定（側面なし・新しい条件はすべて 0）では、雑音を除いて従来と同じ画像（同じ幾何・同じ明るさ）。"""
    cam, plane, lt = setup
    quiet = Lighting(**{**lt.__dict__, "noise_sigma": 0.0})
    scene = Scene([Disc(0.0, 70.0, 20.0, 1.5, 0.85)], seed=4)
    a = Renderer(cam, plane, quiet).render(scene)
    b = Renderer(cam, plane, quiet, sides=False).render(scene)
    for k in a:
        assert np.array_equal(a[k], b[k])
    assert int(a["normal"][cam.height_px // 2, cam.width_px // 2]) == round(8 + 200 * 0.85)   # 中央は硬貨（模様なし）


def test_sides_make_a_tall_object_taller_in_the_image(setup) -> None:
    cam, plane, lt = setup
    quiet = Lighting(**{**lt.__dict__, "noise_sigma": 0.0})
    scene = Scene([Disc(0.0, 75.0, 10.0, 8.0, 0.1)], seed=1)
    rows = []
    for sides in (False, True):
        img = Renderer(cam, plane, quiet, sides=sides).render(scene)["normal"].astype(float)
        dark = img[:, cam.width_px // 2] < 40
        rows.append(int(dark.sum()))
    assert rows[1] > rows[0] * 1.5, rows                          # 側面（前面）が写る分だけ縦に長い


def test_thick_button_cell_is_still_a_metal_disc_when_its_side_is_drawn(cfg: dict, setup) -> None:
    """LR44（11.6 × 5.4mm、鏡面）: 側面を描くと塊が前後に伸びる。円形の判定が床の高さ前提だと metal_disc（危険物側）から
    落ちていた（VIS-0002）。高さ 0〜metal_disc_max_height_mm のどこかの円盤として見れば円形。"""
    cam, plane, lt = setup
    hits = 0
    for seed in range(4):
        fr = Renderer(cam, plane, lt, sides=True).render(Scene([Disc(0.0, 75.0, 11.6, 5.4, 0.9, True, "button_cell")], seed=seed))
        cands, _tr, _fg = detect(fr, cam, plane, cfg)
        near = [c for c in cands if c.is_object and abs(c.floor_xy_mm[0]) < 15 and 55 < c.floor_xy_mm[1] < 100]
        hits += any(any(k["kind"] == "metal_disc" for k in c.kinds) for c in near)
    assert hits == 4


def test_a_seam_across_the_view_is_still_not_round(cfg: dict, setup) -> None:
    """高さを許しても、横に長い塊（段差・継ぎ目）は円形にならない。"""
    from serpens.floorwatch.detect import _roundish
    cam = setup[0]
    foot, top = np.array([0.0, 70.0, 0.0]), np.array([0.0, 74.0, 0.0])
    assert not _roundish(40.0, foot, top, cam, float(cfg["floor_watch"]["detect"]["metal_disc_max_height_mm"]))
    assert _roundish(4.0, foot, top, cam, 11.0)


def test_light_plane_stays_fixed_to_the_camera_when_the_head_sinks(cfg: dict, setup) -> None:
    """頭が 5mm 沈む・2° 垂れる: 光の面はカメラ座標で同じ（名目の画素で見える床の線は、同じ画素に写る）。"""
    cam, plane, _lt = setup
    t = true_camera(cam, Condition(cam_height_err_mm=-5.0, cam_pitch_err_deg=2.0))
    pt = plane_for_true_camera(plane, cam, t)
    for v in (cam.height_px * 0.5, cam.height_px * 0.8):
        u = plane.line_u_on_floor(cam, v)
        d_nom, d_true = cam.ray(u, v), t.ray(u, v)
        # 同じ画素の視線と面の角度（= 面と視線の関係）がカメラ座標で一致する
        assert float(plane.normal @ d_nom) == pytest.approx(float(pt.normal @ d_true), abs=1e-9)


def test_nominal_trial_finds_a_coin_and_flags_nothing_on_an_empty_floor(cfg: dict, setup) -> None:
    cam, plane, lt = setup
    coin = run_trial(cfg, cam, plane, lt, Condition(), "coin_1yen", seed=1, blur_scale=0.5)
    empty = run_trial(cfg, cam, plane, lt, Condition(), None, seed=2, blur_scale=0.5)
    assert coin.stage_hits == {"patrol": True, "inspect": True} and coin.height_est == pytest.approx(1.5, abs=0.3)
    assert empty.false_objects == {"patrol": 0, "inspect": 0}


def test_floor_coloured_thin_coin_is_found_from_its_shadow_even_behind_a_rim_strip(cfg: dict, setup) -> None:
    """10 円玉（23.5 × 1.5mm、反射率 0.45 ≈ 床 0.5）: 上面は床と見分けがつかず、手前の縁だけが細い帯として前景になる。
    帯は線状（継ぎ目扱い）なので物にならない。以前はその帯が奥の影から作る候補を「覆って」消していた（VIS-0002）。"""
    cam, plane, lt = setup
    found = 0
    for y in (58.0, 70.0, 85.0, 98.0):
        fr = Renderer(cam, plane, lt, sides=True).render(Scene([Disc(0.0, y, 23.5, 1.5, 0.45, False, "coin")], seed=int(y)))
        for with_line in (False, True):
            cands, _tr, _fg = detect(fr, cam, plane, cfg, with_line=with_line)
            found += any(c.is_object and abs(c.floor_xy_mm[0]) < 15 for c in cands)
    assert found == 8


@pytest.mark.parametrize("cond", [Condition(cam_pitch_err_deg=-2.0), Condition(cam_height_err_mm=-5.0),
                                  Condition(blur_sigma_px=4.0), Condition(fov_err_deg=5.0)],
                         ids=["pitch-2", "height-5", "blur4", "fov+5"])
def test_empty_floor_under_small_pose_error_or_blur_gives_no_line_only_object(cfg: dict, setup, cond: Condition) -> None:
    """VIS-0002: 何も無い床で、線だけの候補が出ていた。(A) 遠く・近くの端で線が細って消えるのを「途切れ」とした
    (B) 姿勢のずれで床の高さが線に沿って坂になり、全体の中央値との差で「出っ張り」に見えた。"""
    cam, plane, lt = setup
    for seed in (7, 8):
        t = run_trial(cfg, cam, plane, lt, cond, None, seed=seed, blur_scale=0.5)
        assert t.false_objects["inspect"] == 0, (cond, seed)


def test_clear_object_is_still_found_from_the_line_alone(cfg: dict, setup) -> None:
    """線だけの経路を残す: 床と同じ反射率で鏡面（透明な物の代わり）は、線の途切れ（前後に床の線がある）で見つかる。"""
    cam, plane, lt = setup
    found = 0
    for seed in range(3):
        fr = Renderer(cam, plane, lt, sides=True).render(Scene([Disc(0.0, 80.0, 15.0, 3.0, 0.5, True, "unknown")], seed=seed))
        cands, _tr, _fg = detect(fr, cam, plane, cfg, with_line=True)
        found += any(c.is_object and abs(c.floor_xy_mm[0]) < 15 and 60 < c.floor_xy_mm[1] < 100 for c in cands)
    assert found == 3


def test_five_mm_magnet_ball_is_a_metal_disc(cfg: dict, setup) -> None:
    """5mm の磁石球（鏡面）: 大きさが約 10% 小さく出る。下限 5.0mm だと危険物側に回らなかった（VIS-0002）。"""
    cam, plane, lt = setup
    flagged = 0
    for seed in range(4):
        t = run_trial(cfg, cam, plane, lt, Condition(), "magnet_ball_5", seed=seed, blur_scale=0.5)
        flagged += t.critical_flag
    assert flagged >= 3


def test_the_floor_line_recovers_camera_height_and_pitch_errors(cfg: dict, setup) -> None:
    """VIS-0003: 光の面は頭に固定なので、床の線の写り方から Δ高さ・Δpitch が分かる（線を姿勢センサにも使う）。"""
    from dataclasses import replace

    from simulation.h2_pose_from_line import estimate_pose
    from simulation.h2_vision import lighting_for
    cam, plane, lt = setup
    c = Condition(cam_height_err_mm=-6.5, cam_pitch_err_deg=-0.5)
    ct = true_camera(cam, c)
    fr = Renderer(ct, plane_for_true_camera(plane, cam, ct), lighting_for(lt, c, 0.5), sides=True).render(
        Scene([Disc(0.0, 75.0, 20.0, 1.5, 0.85)], seed=2))
    cands, tr, _fg = detect(fr, cam, plane, cfg)
    dh, dp, _rms = estimate_pose(tr, cam, plane)
    assert dh == pytest.approx(-6.5, abs=1.5) and dp == pytest.approx(-0.5, abs=0.5)
    coin = next(x for x in cands if x.is_object and abs(x.floor_xy_mm[0]) < 15)
    bx, by, bw, bh = coin.bbox_px
    fixed = replace(cam, height_mm=cam.height_mm + dh, pitch_deg=cam.pitch_deg + dp).floor_point(bx + bw / 2, by + bh)
    raw = cam.floor_point(bx + bw / 2, by + bh)
    assert abs(fixed[1] - 65.0) < 3.0 < abs(raw[1] - 65.0)      # 手前の縁 65mm: 名目では大きくずれ、補正で戻る


def test_brow_line_traced_by_column_reads_coins_and_tall_parts(cfg: dict, setup) -> None:
    """VIS-0004: 眉の線（左右に走る）を列ごとに追うと、平たい硬貨は頬と同じく読め、背の高い細い物は頬より実際に近い。"""
    from serpens.floorwatch.detect import scaled_thresholds
    from simulation.h2_brow_line import compare_on_object
    from simulation.h2_line_light import placements
    cam, cheek, lt = setup
    det = scaled_thresholds(cfg, cam)
    brow = next(p for p in placements(cfg) if p.name == "brow_z64_y71").plane()
    coin = compare_on_object(cfg, cam, cheek, brow, 71.0, lt, Disc(0.0, 71.0, 20.0, 1.5, 0.85), 1, det)
    assert coin["brow"]["height_mm"] == pytest.approx(1.5, abs=0.2) and coin["cheek"]["height_mm"] == pytest.approx(1.5, abs=0.2)
    pill = compare_on_object(cfg, cam, cheek, brow, 71.0, lt, Disc(0.0, 71.0, 8.0, 3.5, 0.9), 1, det)
    assert abs(pill["brow"]["height_mm"] - 3.5) < abs(pill["cheek"]["height_mm"] - 3.5)
    cell = compare_on_object(cfg, cam, cheek, brow, 71.0, lt, Disc(0.0, 71.0, 20.0, 3.2, 0.9, True), 1, det)
    assert cell["brow"]["dropout"] and cell["cheek"]["dropout"]                 # 鏡面は両方で途切れる（metal_disc の証拠）
