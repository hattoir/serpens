"""口に入れやすい形と機構（`simulation/scoop/forms/`）。**MuJoCo が無ければ丸ごと skip。**

ここで確かめるのは「モデルが物理として筋が通っていること」と「探索で分かった構造的な結果が変わっていないこと」。
**現実と一致することは確かめていない**（MUJOCO_SIM。摩擦・質量・トルクは ASSUMED）。
"""
from __future__ import annotations

import pytest

pytest.importorskip("mujoco", reason="MuJoCo は任意依存（requirements-sim3d.txt）")

from simulation.scoop.forms.runner import run_form_episode  # noqa: E402
from simulation.scoop.model import load_config  # noqa: E402

FORMS = {
    "hood": {"funnel": False, "backstop": None},
    "sweeper": {"L": 20, "S": 90, "T": 0.3, "soft": False},
    "belt": {"mu": 1.0, "vb": 40, "d": 3, "backstop": "wall"},
    "brush": {"D": 8, "rpm": 120, "stiff": "soft", "backstop": None},
    "cup": {"ID": 30, "gap_mm": 0.0},
    "hook": {"tip": "round", "h": 2},
}


@pytest.fixture(scope="module")
def cfg() -> dict:
    return load_config()


@pytest.mark.parametrize("form", list(FORMS))
def test_every_form_builds_and_runs_and_reports_the_metrics(cfg: dict, form: str) -> None:
    r = run_form_episode(cfg, form, FORMS[form], "coin_1yen", "flooring", 0.0 if form == "cup" else 10.0, 0.0, 1)
    assert r.source == "MUJOCO_SIM" and r.form == form
    assert r.outcome in ("success", "knocked_in", "escaped", "pinched", "pushed_ahead", "not_entered")
    assert r.entered_ever and r.t_end_s > 0.0


def test_passive_hood_takes_a_centred_coin_but_not_one_wider_than_the_mouth_allows(cfg: dict) -> None:
    """口の幅 30 に対し、1 円玉（φ20）はずれ 5mm までは入り、10mm ずれると壁の前の縁に当たって押し出される（幾何のとおり）。"""
    ok = run_form_episode(cfg, "hood", {"funnel": False, "backstop": None}, "coin_1yen", "flooring", 10.0, 0.0, 1)
    assert ok.success and ok.inside_at_close and ok.held and not ok.launched
    bad = run_form_episode(cfg, "hood", {"funnel": False, "backstop": None}, "coin_1yen", "flooring", 10.0, 10.0, 1)
    assert bad.outcome == "pushed_ahead" and not bad.success


def test_belt_cannot_lift_a_free_object_but_can_against_a_backstop(cfg: dict) -> None:
    """ベルトが物を引き上げる力は μ × 垂直抗力。自由な物は床の摩擦の分しか押されない（数 mN で、物の重さより小さい）ので持ち上がらない。
    壁に当てて頭が最大 1N まで押すと、垂直抗力が出て乗る。構造的な結果。"""
    p = {"mu": 1.0, "vb": 40, "d": 3}
    free = run_form_episode(cfg, "belt", {**p, "backstop": None}, "coin_1yen", "flooring", 10.0, 0.0, 1)
    assert free.outcome == "pushed_ahead" and not free.rode_ever
    wall = run_form_episode(cfg, "belt", {**p, "backstop": "wall"}, "coin_1yen", "flooring", 10.0, 0.0, 1)
    assert wall.rode_ever and wall.entered_ever and wall.success


def test_cup_rim_lands_on_an_object_that_is_more_than_the_margin_off_centre(cfg: dict) -> None:
    """カップ内径 30 の余裕は（30 − 20）/ 2 = 5mm。6mm ずれると縁が 1 円玉の上に乗って下りきれない（pinched）。内径 40 なら入る。"""
    assert run_form_episode(cfg, "cup", {"ID": 30, "gap_mm": 0.0}, "coin_1yen", "flooring", 0.0, 3.0, 1).success
    assert run_form_episode(cfg, "cup", {"ID": 30, "gap_mm": 0.0}, "coin_1yen", "flooring", 0.0, 6.0, 1).outcome == "pinched"
    assert run_form_episode(cfg, "cup", {"ID": 40, "gap_mm": 0.0}, "coin_1yen", "flooring", 0.0, 6.0, 1).success


def test_knocked_in_objects_are_not_counted_as_held(cfg: dict) -> None:
    """飛ばされて入った物は保持に数えない（success = False、knocked_in = True で別集計）。"""
    r = run_form_episode(cfg, "sweeper", {"L": 20, "S": 90, "T": 0.3, "soft": False}, "coin_1yen", "flooring", 10.0, 0.0, 1)
    assert r.launched and r.outcome == "knocked_in" and r.knocked_in and not r.success and r.held and r.inside_at_close


def test_a_step_at_the_mouth_floor_turns_the_hood_back_into_a_pusher(cfg: dict) -> None:
    """前の押す方式（0%）との違いを生んだ設計変数 = 口の床の段差。フード（段差なし）は物をまたいで入り、0.1mm の床の板を足すだけで押されて逃げる。"""
    p = {"funnel": False, "gate": True, "retreat": 0, "backstop": None}
    ok = run_form_episode(cfg, "hood", p, "coin_1yen", "flooring", 10.0, 0.0, 1)
    assert ok.success and not ok.wall_contact
    for plate in (0.1, 0.6):
        r = run_form_episode(cfg, "hood", {**p, "plate": plate}, "coin_1yen", "flooring", 10.0, 0.0, 1)
        assert r.outcome == "pushed_ahead" and not r.entered_ever


def test_the_gate_matters_only_when_the_head_backs_away(cfg: dict) -> None:
    """ゲートの効果の切り分け: 頭が止まったままなら、ゲートの有無で保持は変わらない。閉じ終わりの後に頭が後退すると、ゲートなしでは物が口から取り残される。"""
    base = {"funnel": False, "backstop": None}
    args = ("coin_1yen", "flooring", 10.0, 0.0, 1)
    assert run_form_episode(cfg, "hood", {**base, "gate": True, "retreat": 0}, *args).success
    assert run_form_episode(cfg, "hood", {**base, "gate": False, "retreat": 0}, *args).success
    assert run_form_episode(cfg, "hood", {**base, "gate": True, "retreat": 30}, *args).success
    left = run_form_episode(cfg, "hood", {**base, "gate": False, "retreat": 30}, *args)
    assert left.outcome == "escaped" and left.inside_at_close and not left.held


def test_straddling_is_told_apart_from_being_guided_by_the_walls(cfg: dict) -> None:
    """ずれ 0 の 1 円玉は壁に触れずにまたいで入る。ずれ 10 mm の立方体は壁の前の縁に当たって（角が触れて）誘導されて入る。物はすき間 0.1mm より厚いので、壁の下へは逃げない。"""
    p = {"funnel": False, "gate": True, "retreat": 0, "backstop": None}
    a = run_form_episode(cfg, "hood", p, "coin_1yen", "flooring", 10.0, 0.0, 1)
    assert a.success and not a.wall_contact
    b = run_form_episode(cfg, "hood", p, "crumb_cube", "flooring", 10.0, 10.0, 1)
    assert b.entered_ever and b.wall_contact


def test_any_rigid_step_at_the_mouth_blocks_even_a_2_micron_one(cfg: dict) -> None:
    """段差 0.002 mm でも押されて逃げる（有る / 無いで決まり、大きさは効かない）。すき間 0〜1 mm は、段差 0 なら効かない。"""
    base = {"funnel": True, "gate": True, "retreat": 0, "backstop": None}
    for c in (0.0, 1.0):
        assert run_form_episode(cfg, "hood", {**base, "clearance_mm": c}, "coin_1yen", "flooring", 10.0, 0.0, 1).success
    r = run_form_episode(cfg, "hood", {**base, "clearance_mm": 0.1, "plate": 0.002}, "coin_1yen", "flooring", 10.0, 0.0, 1)
    assert r.outcome == "pushed_ahead" and not r.entered_ever


def test_gate_force_limit_does_not_matter_for_closing_behind_the_object(cfg: dict) -> None:
    """ゲートが閉じるのは物が口の面より奥に入ってからなので、力の上限を 0.25 N まで下げても保持は変わらない（2.8 N 以下で成立）。"""
    base = {"funnel": True, "gate": True, "retreat": 30, "backstop": None}
    for f in (5.0, 2.8, 0.25):
        assert run_form_episode(cfg, "hood", {**base, "gate_force_n": f}, "crumb_cube", "flooring", 10.0, 0.0, 1).success


def test_passive_curtain_needs_a_tiny_opening_force_and_a_short_flap(cfg: dict) -> None:
    """受け身の垂れ布: 開く力 0.1 N は 1 円玉の床の摩擦（約 3 mN）より大きく、押されて逃げる。全高の垂れ布は物の上に垂れかかり、後退で物が出る。
    8 mm の短い垂れ布 + 3 mN なら、1 円玉は入り、後退しても残る。"""
    base = {"funnel": True, "gate": "curtain", "backstop": None}
    args = ("coin_1yen", "flooring", 10.0, 0.0, 1)
    assert run_form_episode(cfg, "hood", {**base, "curtain_f": 0.1, "retreat": 0}, *args).outcome == "pushed_ahead"
    full = run_form_episode(cfg, "hood", {**base, "curtain_f": 0.003, "retreat": 30}, "battery_cr2032", "flooring", 10.0, 0.0, 1)
    assert full.outcome == "escaped"
    short = run_form_episode(cfg, "hood", {**base, "curtain_f": 0.003, "curtain_len_mm": 8.0, "retreat": 30}, *args)
    assert short.success


def test_tolerance_expectation_is_bounded_by_the_grid_not_interpolated() -> None:
    """許容差の確率版: 格子点の間の保持率は分からないので、下限（小さい方）/ 上限（大きい方）で期待値の範囲を出す。t=0 だけ 100%、0.002 で 0% なら 0〜0.67%。"""
    import importlib.util
    from pathlib import Path
    spec = importlib.util.spec_from_file_location("tol_mc", Path(__file__).resolve().parents[1] / "tools" / "scoop_forms_tolerance_mc.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    import numpy as np
    lo, hi = m.expected_bounds([0.0, 0.002, 0.1], np.array([1.0, 0.0, 0.0]))
    assert lo == pytest.approx(0.0) and hi == pytest.approx(0.002 / 0.3)


def test_chamfer_polygon_is_a_convex_ramp_from_the_floor() -> None:
    """面取りの板: 斜面は床（z = c）から始まり、上面（z = c + t）までを角 θ で上がる。凸で、丸めても凸のまま。"""
    from simulation.scoop.forms.passive import plate_polygon
    for ang, R in ((5, 0.0), (10, 0.1), (45, 0.3), (90, 0.1)):
        poly = plate_polygon(0.0, 0.2e-3, 30e-3, ang, R)
        cr = []
        for i in range(len(poly)):
            a, b, c = poly[i], poly[(i + 1) % len(poly)], poly[(i + 2) % len(poly)]
            cr.append((b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]))
        assert all(x >= -1e-15 for x in cr) or all(x <= 1e-15 for x in cr)
        assert min(z for _, z in poly) == pytest.approx(0.0) and max(z for _, z in poly) == pytest.approx(0.2e-3)


def test_chamfer_lets_a_cube_ride_in_but_does_not_make_the_hood_hold_it(cfg: dict) -> None:
    """面取りで戻るのは「入る」（乗れる物）だけで、保持ではない。段差 0.2 mm・面取り 10° で、立方体は入るが保持にならず、1 円玉は入らない。"""
    p = {"funnel": True, "gate": True, "retreat": 0, "backstop": None, "clearance_mm": 0.0, "plate": 0.2, "plate_chamfer_deg": 10, "plate_round_mm": 0.1}
    cube = run_form_episode(cfg, "hood", p, "crumb_cube", "flooring", 10.0, 0.0, 1)
    coin = run_form_episode(cfg, "hood", p, "coin_1yen", "flooring", 10.0, 0.0, 1)
    assert cube.entered_ever and not cube.success
    assert not coin.entered_ever and not coin.success


def test_zero_percent_at_a_2_micron_step_does_not_depend_on_the_contact_solver_settings(cfg: dict) -> None:
    """接触のやわらかさ（solref 5 倍）・時間刻み（1/4）を変えても、段差 0.002 mm は保持にならず、段差なしは保持になる（設定が壊れていない）。"""
    base = {"funnel": True, "gate": True, "retreat": 0, "backstop": None, "clearance_mm": 0.1}
    for v in ({"solref_s": 0.01}, {"timestep_s": 5.0e-5}):
        assert run_form_episode(cfg, "hood", {**base, **v}, "coin_1yen", "flooring", 10.0, 0.0, 1).success
        assert not run_form_episode(cfg, "hood", {**base, **v, "plate": 0.002}, "coin_1yen", "flooring", 10.0, 0.0, 1).success


def test_floor_roughness_does_not_rescue_a_vertical_step(cfg: dict) -> None:
    """床の粗さ ±0.1 mm でも、垂直の段差 0.002 mm は保持にならない。板なしなら粗さがあっても保持になる。"""
    base = {"funnel": True, "gate": True, "retreat": 0, "backstop": None, "clearance_mm": 0.1, "rough_mm": 0.1}
    assert run_form_episode(cfg, "hood", base, "crumb_cube", "flooring", 10.0, 0.0, 1).success
    assert not run_form_episode(cfg, "hood", {**base, "plate": 0.002}, "crumb_cube", "flooring", 10.0, 0.0, 1).success


def test_thin_film_edge_without_chamfer_still_blocks(cfg: dict) -> None:
    """薄いフィルム（0.02 mm、垂直の縁、粗さ ±0.1）でも、保持にならない（§13.1）。"""
    p = {"funnel": True, "gate": True, "retreat": 0, "backstop": None, "clearance_mm": 0.1, "plate": 0.02, "rough_mm": 0.1}
    assert not run_form_episode(cfg, "hood", p, "coin_1yen", "flooring", 10.0, 0.0, 1).success


def test_cloche_covers_carries_and_needs_the_gate_only_when_retreating(cfg: dict) -> None:
    """フード昇降: 落として被せれば、その場で覆う・30 mm 前進・30 mm 後退（ゲートあり）で物が残る。ゲートなしの後退は物を置いていく（§13.3）。"""
    base = {"lift_mm": 5.0, "drop_speed_mm_s": 20.0, "clearance_mm": 0.1, "backstop": None}
    for rt in (0, 30, -30):
        assert run_form_episode(cfg, "cloche", {**base, "retreat": rt}, "crumb_cube", "flooring", 10.0, 0.0, 1).success
    r = run_form_episode(cfg, "cloche", {**base, "retreat": 30, "gate": False}, "crumb_cube", "flooring", 10.0, 0.0, 1)
    assert not r.success


def test_cloche_lands_on_a_coin_that_is_10_mm_off_centre(cfg: dict) -> None:
    """1 円玉（半径 10）は中心が壁の内側の面（±15）まで 5 mm しか余裕が無い。10 mm ずれると、落としたフードの壁の下端が縁に乗って挟まる。"""
    base = {"lift_mm": 5.0, "drop_speed_mm_s": 20.0, "clearance_mm": 0.1, "retreat": 0, "backstop": None}
    assert run_form_episode(cfg, "cloche", base, "coin_1yen", "flooring", 10.0, 5.0, 1).success
    assert run_form_episode(cfg, "cloche", base, "coin_1yen", "flooring", 10.0, 10.0, 1).outcome == "pinched"
