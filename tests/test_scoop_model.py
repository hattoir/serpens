"""頭のスコップ＋フタ（`simulation/scoop/`）。**MuJoCo が無ければ丸ごと skip。**

ここで確かめるのは「モデルが物理として筋が通っていること」と「探索で分かった構造的な結果が変わっていないこと」。
**現実と一致することは確かめていない**（MUJOCO_SIM。摩擦・質量・押しつけ力は ASSUMED）。
"""
from __future__ import annotations

import math

import pytest

pytest.importorskip("mujoco", reason="MuJoCo は任意依存（requirements-sim3d.txt）")

from simulation.scoop.model import Shape, all_shapes, build_mjcf, load_config  # noqa: E402
from simulation.scoop.runner import place_on_ramp_and_release, run_episode  # noqa: E402
from simulation.scoop.sweep import wilson  # noqa: E402


@pytest.fixture(scope="module")
def cfg() -> dict:
    return load_config()


def test_all_shapes_and_objects_compile(cfg: dict) -> None:
    import mujoco
    assert len(all_shapes(cfg)) == 36
    for shape in all_shapes(cfg)[::7]:
        for obj in cfg["objects"]:
            mujoco.MjModel.from_xml_string(build_mjcf(cfg, shape, obj, "flooring").xml)
    mujoco.MjModel.from_xml_string(build_mjcf(cfg, all_shapes(cfg)[0], "coin_1yen", "mat", rim_fillet_mm=0.3, front_face="vertical").xml)


def test_ramp_holds_an_object_only_if_tan_alpha_is_below_mu(cfg: dict) -> None:
    """解析の照合: 傾斜板の上の物は tan α ≤ μ なら保持され、超えると滑る（scoop μ = 0.3、tan 18° = 0.325）。"""
    for alpha, mu, holds in ((8.0, 0.3, True), (12.0, 0.3, True), (18.0, 0.3, False), (18.0, 0.5, True)):
        assert (math.tan(math.radians(alpha)) <= mu) == holds
        r = place_on_ramp_and_release(cfg, Shape(0.6, alpha, False, 30.0), "coin_1yen", "flooring", seconds=1.0, scoop_mu=mu)
        assert (abs(r["moved_mm"]) < 2.0) == holds, (alpha, mu, r)


def test_default_brief_pushes_flat_objects_ahead_regardless_of_friction_and_speed(cfg: dict) -> None:
    """探索の結論（構造的）: 指示書の範囲（先端厚 0.4〜1.0、丸みなし）では、平らな物は頭と同じ速さで前へ運ばれ、成功しない。
    床の摩擦・スコップの摩擦・先端の浮き・速度を変えても、押して逃げる距離は変わらない。"""
    shape = Shape(0.4, 8.0, False, 30.0)
    dist = []
    for floor, kw, speed in (("flooring", {}, 20.0), ("mat", {}, 20.0), ("flooring", {"scoop_mu": 1.0}, 20.0),
                             ("flooring", {"clearance_mm": 0.0}, 20.0), ("flooring", {}, 40.0)):
        r = run_episode(cfg, shape, "coin_1yen", floor, speed, 0.0, 1, **kw)
        assert r.outcome == "pushed_ahead" and not r.success
        dist.append(r.forward_disp_mm)
    assert max(dist) - min(dist) < 1.0


def test_wilson_interval() -> None:
    lo, hi = wilson(0, 30)
    assert lo == pytest.approx(0.0, abs=1e-9) and 0.0 < hi < 0.1
    lo, hi = wilson(15, 30)
    assert lo < 0.5 < hi
