"""囲い込み・挟まり検査（`simulation/hardware_gaps/HG-S1_contact_safety/entrapment_check.py`）。GEOMETRY_SIM（平面・剛体・CAD_CONCEPT の寸法）。全形の掃引は重い（数分）ので、ここでは検出器の単体だけ。"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope="module")
def ec():
    spec = importlib.util.spec_from_file_location("entrapment_check", ROOT / "simulation/hardware_gaps/HG-S1_contact_safety/entrapment_check.py")
    m = importlib.util.module_from_spec(spec)
    sys.modules["entrapment_check"] = m
    spec.loader.exec_module(m)
    return m


def test_straight_chain_has_the_cad_overall_length(ec) -> None:
    """CAD.md: 全長 573 mm = 145 + 95 × 3 + 86.8 + 56.2。"""
    pts = ec.chain_points((0, 0, 0, 0), 0.0)
    assert np.linalg.norm(pts[-1] - pts[0]) == pytest.approx(573.0, abs=0.01)
    assert sum(ec.LINKS_MM) == pytest.approx(573.0, abs=0.01)


def test_wrap_angle_is_the_largest_contiguous_signed_sum(ec) -> None:
    assert ec.max_contiguous_sum([50, 50, 45, 0]) == 145
    assert ec.max_contiguous_sum([50, -50, 50, -50]) == 50
    assert ec.max_contiguous_sum([-50, -50, 25]) == 100


def test_body_configs_respect_the_145_degree_budget_and_the_soft_limit(ec) -> None:
    cfgs = ec.body_configs()
    assert cfgs and all(ec.max_contiguous_sum(c) <= 145.0 + 1e-9 and max(abs(x) for x in c) <= 50.0 for c in cfgs)
    assert (50.0, 50.0, 50.0, 0.0) not in cfgs                       # 150° は 145° を超える


def test_segment_distance(ec) -> None:
    a0, a1 = np.array([0.0, 0.0]), np.array([10.0, 0.0])
    assert ec.seg_dist(a0, a1, np.array([0.0, 5.0]), np.array([10.0, 5.0])) == pytest.approx(5.0)
    assert ec.seg_dist(a0, a1, np.array([5.0, -3.0]), np.array([5.0, 3.0])) == 0.0         # 交差
    assert ec.seg_dist(a0, a1, np.array([13.0, 4.0]), np.array([20.0, 4.0])) == pytest.approx(5.0)


def test_straight_body_has_no_self_overlap_for_links_three_apart(ec) -> None:
    """1 つおき（j − i = 2）は丸い端が直線でも重なるモデルの人工物なので数えない。3 つ以上離れたリンクは重ならない。"""
    k = ec.clearances(ec.chain_points((0, 0, 0, 0), 0.0))
    assert k["min_nonadjacent"] > 0 and k["min_head_to_body"] > 0


def test_detector_finds_nothing_in_a_straight_body_and_finds_the_closed_loop_of_a_slender_one(ec) -> None:
    """検出器の確認: まっすぐでは抜けなくなる円は無い。細い体（半径 5 mm）を 90° × 4 で 360° 巻いた閉じた輪の内側は、直径 80 mm 台の円を閉じ込める。"""
    assert ec.trap_diameters(ec.chain_points((0, 0, 0, 0), 0.0)) == []
    pk = ec.pocket(ec.chain_points((90, 90, 90, 90), 0.0), (5.0,) * 6)
    assert pk is not None and 78.0 <= pk["d_in_mm"] <= 90.0 and pk["mouth_mm"] <= 2.0


def test_the_largest_body_bend_without_head_yaw_traps_nothing(ec) -> None:
    assert ec.trap_diameters(ec.chain_points((50, 50, 45, 0), 0.0)) == []


def test_parts_trapped_needs_the_part_to_be_thicker_than_the_mouth_and_fit_inside(ec) -> None:
    pk = {"mouth_mm": 70.0, "d_in_mm": 90.0}
    names = ec.parts_trapped(pk)
    assert len(names) == 1 and names[0].startswith("首")              # 首 63.7〜88.5 のうち 70〜88.5 だけ
    assert ec.parts_trapped({"mouth_mm": 134.0, "d_in_mm": 138.0}) == []   # 胸 138.2 は内側の最大より太い
