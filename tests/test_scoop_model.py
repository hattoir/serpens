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


def test_beak_arm_end_follows_the_specified_path(cfg: dict) -> None:
    """くちばし: 腕先が (−腕長, H) → (0, H − 腕長) → (+腕長, H) を通る（q = 0 / 90 / 180°）。隙間 = H − 腕長。"""
    import mujoco
    spec = build_mjcf(cfg, Shape(0.6, 12.0, False, 30.0, 8.0), "coin_1yen", "mat", beak=(12.0, 11.5))
    m = mujoco.MjModel.from_xml_string(spec.xml)
    d = mujoco.MjData(m)
    body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "lid")
    adr = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "hinge")]
    for q, (x, z) in ((0, (-11.5, 12.0)), (90, (0.0, 0.5)), (180, (11.5, 12.0))):
        d.qpos[adr] = math.radians(q)
        mujoco.mj_forward(m, d)
        end = d.xpos[body] + d.xmat[body].reshape(3, 3) @ [-11.5e-3, 0.0, 0.0]
        assert (end[0] * 1000, end[2] * 1000) == pytest.approx((x, z), abs=1e-6), q
    assert spec.lid_open_rad == 0.0 and spec.lid_closed_rad == pytest.approx(math.pi)


def test_ride_is_true_in_the_known_riding_case_and_false_for_the_rear_hinge_baseline(cfg: dict) -> None:
    """既知ケース（先端 0.05mm・丸み 0.3・マット・μ 0.3 の 1 円玉が 9.1mm まで乗る）で乗る = 真。
    先端 0.6mm・μ 0.3 の通常の形では、奥ヒンジのフタ（ベースライン）は乗らない・入らない・保持しない。"""
    r = run_episode(cfg, Shape(0.05, 8.0, False, 30.0), "coin_1yen", "mat", 20.0, 0.0, 1, rim_fillet_mm=0.3, clearance_mm=0.0,
                    front_face="vertical", scoop_mu=0.3)
    assert r.rode_ever and r.head_edge_lift_max_mm >= 1.0 and r.ride_max_rel_mm == pytest.approx(9.1, abs=0.5)
    b = run_episode(cfg, Shape(0.6, 12.0, False, 30.0, 8.0), "coin_1yen", "mat", 10.0, 0.0, 1, trigger="contact:0", close_time_s=0.3,
                    lid_front_ahead_mm=5.0)
    assert not (b.rode_ever or b.entered_ever or b.success)


def test_beak_closes_and_latches_but_jams_on_a_tall_object(cfg: dict) -> None:
    """くちばしは、薄い物（1 円玉）では 180° まで回ってラッチされ、10mm 角の立方体では腕が物に当たって回り切れない（トルク上限 0.05 N·m）。
    腕が届いていないときはラッチを掛けない（掛けると可動域の端が遠くて腕を強く引っ張る）: 腕が物に掛ける力は、トルク上限から出る値（約 4 N）を超えない。"""
    coin = run_episode(cfg, Shape(0.6, 12.0, False, 30.0, 5.0), "coin_1yen", "flooring", 10.0, 0.0, 1, trigger="contact:0",
                       close_time_s=0.6, beak=(15.0, 14.5), stop_on_trigger=True)
    assert coin.triggered and coin.lid_angle_end_deg == pytest.approx(180.0, abs=1.0)
    trace: list = []
    cube = run_episode(cfg, Shape(0.6, 12.0, False, 30.0, 5.0), "crumb_cube", "flooring", 10.0, 0.0, 1, trigger="contact:0",
                       close_time_s=0.6, beak=(15.0, 14.5), stop_on_trigger=True, trace=trace)
    assert cube.triggered and cube.lid_angle_end_deg < 90.0 and not cube.success
    assert max(row[6] for row in trace[-100:]) < 0.05 / 14.5e-3 * 1.5
