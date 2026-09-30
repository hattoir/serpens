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
