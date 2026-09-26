"""フェーズ 3: 自己位置（AprilTag + IMU + 歩容オドメトリ、座標系 home）。**すべて模擬（SIMULATED / KINEMATIC_SIM）。**

  - 地図（tags.yaml）の読み込みと frame / map_version
  - 推定器: タグ 1 枚で初期化、滑りの σ が距離に比例して育つ、補正で縮む、出どころの遷移、偽タグのゲート、IMU の磁北差
  - 開始条件は σ と局所センサーの健全性で決まる（出どころの名前ではない）。safety.autonomy_blockers も同じ
  - 周回の閉ループで σ が正直（|誤差| ≤ 3σ）
  - 合成画像で AprilTag 検出 → 距離・方位・面の向き（実カメラは未測定）
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from serpens.config import load_config
from serpens.localization.apriltag_detector import AprilTagDetector
from serpens.localization.estimator import PoseEstimator, TagObservation, start_blockers, wrap
from serpens.localization.sim_observer import SimTagObserver
from serpens.localization.sim_run import run_loop_sim
from serpens.localization.tag_map import TagMap
from serpens.safety import AutonomyInputs, autonomy_blockers
from serpens.world_state import PoseSource


@pytest.fixture(scope="module")
def cfg() -> dict:
    return load_config()


@pytest.fixture(scope="module")
def tmap(cfg: dict) -> TagMap:
    return TagMap.from_cfg(cfg)


def test_tag_map_defines_frame_home_and_rejects_duplicates(cfg: dict, tmap: TagMap) -> None:
    assert tmap.frame_id == "home" and tmap.map_version.startswith("tags-v") and tmap.family == "tag36h11"
    assert tmap.get(0) is not None and tmap.get(0).x_m == 0.0 and tmap.get(0).y_m == 0.0     # 原点 = ドック
    with pytest.raises(ValueError):
        TagMap.from_dict({"frame_id": "home", "map_version": "x", "family": "tag36h11", "tag_size_m": 0.06,
                          "tags": [{"id": 1, "x_m": 0, "y_m": 0, "z_m": 0, "yaw_rad": 0}] * 2})
    with pytest.raises(ValueError):
        TagMap.from_dict({"frame_id": "room", "map_version": "x", "family": "tag36h11", "tag_size_m": 0.06, "tags": []})


def _exact_obs(tmap: TagMap, tid: int, x: float, y: float, yaw: float, t: float = 0.0) -> TagObservation:
    tag = tmap.get(tid)
    dx, dy = tag.x_m - x, tag.y_m - y
    return TagObservation(tid, math.hypot(dx, dy), wrap(math.atan2(dy, dx) - yaw), wrap(tag.yaw_rad - yaw), t, simulated=True)


def test_one_tag_initializes_the_pose_and_blind_travel_grows_sigma_linearly(cfg: dict, tmap: TagMap) -> None:
    est = PoseEstimator(cfg, tmap)
    assert est.estimate(0.0).source == "UNKNOWN" and start_blockers(cfg, est.estimate(0.0), est.health(0.0))
    assert est.correct([_exact_obs(tmap, 1, 1.0, 0.2, 0.3)]) == 1
    e = est.estimate(0.0)
    assert e.source == "APRILTAG" and abs(e.x_m - 1.0) < 1e-6 and abs(e.y_m - 0.2) < 1e-6 and abs(e.yaw_rad - 0.3) < 1e-9
    frac = cfg["localization"]["odometry"]["sigma_along_frac"]
    est.x[2] = 0.0                                                                      # +x へ進む（x の分散 = 前進方向の分散）
    p0 = float(est.P[0, 0])
    for i in range(1, 101):                                                             # 1.0 m を 100 歩で
        est.predict(i * 0.1, 0.01, 0.0, yaw_from_imu=False)
        est.last_imu_t = i * 0.1                                                        # IMU は新しいことにする
    e = est.estimate(10.0)
    assert e.source == "ODOMETRY_IMU" and e.last_tag_age_s == pytest.approx(10.0)
    assert float(est.P[0, 0]) - p0 == pytest.approx((frac * 1.0) ** 2, rel=0.15)        # (15% × 1 m)²（1 歩ごとの √N ではない）
    assert any("σ_xy" in b for b in start_blockers(cfg, e, est.health(10.0)))           # 限界 0.15 m を超えた
    est2 = PoseEstimator(cfg, tmap)
    est2.correct([_exact_obs(tmap, 1, 1.0, 0.2, 0.0)])
    for i in range(1, 51):
        est2.predict(i * 0.1, 0.01, 0.0, False)
    before = est2.estimate(5.0).sigma_xy_m
    est2.correct([_exact_obs(tmap, 1, 1.5, 0.2, 0.0, 5.0)])                             # 真の位置 1.5（滑りなし）で補正
    after = est2.estimate(5.0)
    assert after.sigma_xy_m < before * 0.7 and after.source == "APRILTAG" and abs(after.x_m - 1.5) < 0.05


def test_spoofed_tag_is_rejected_by_the_gate_and_imu_offset_is_learned(cfg: dict, tmap: TagMap) -> None:
    est = PoseEstimator(cfg, tmap)
    est.imu_yaw(0.0, 0.3 + 0.7)                                                          # 磁北との差 0.7 rad
    est.correct([_exact_obs(tmap, 1, 1.0, 0.2, 0.3)])
    assert est.imu_offset == pytest.approx(-0.7, abs=1e-6)
    fake = TagObservation(2, 0.5, 0.0, 0.0, 0.1, simulated=True)                         # 位置が 1m 以上飛ぶ偽観測
    assert est.correct([fake]) == 0 and est.rejected == 1
    est.predict(0.2, 0.0, 0.0, True)
    est.imu_yaw(0.2, 0.5 + 0.7)                                                          # 実際に 0.2 rad 回った（IMU だけが知る）
    assert abs(est.estimate(0.2).yaw_rad - 0.5) < 0.05
    assert est.correct([TagObservation(99, 0.5, 0.0, 0.0, 0.3)]) == 0                    # 地図に無い ID は無視


def test_loop_sim_sigma_is_honest_and_blind_distance_to_block_is_reported(cfg: dict) -> None:
    res = run_loop_sim(cfg, seed=1, seconds=240.0)
    assert res.corrections > 50 and res.rejected <= 0.1 * res.corrections
    assert res.consistency(3.0) >= 0.9, res.consistency(3.0)
    assert max(res.err_m[len(res.err_m) // 4:]) < 0.6
    d = res.blind_distance_until_blocked()
    limit = cfg["localization"]["start"]["max_sigma_xy_m"] / cfg["localization"]["odometry"]["sigma_along_frac"]
    assert d is not None and 0.5 * limit <= d <= 1.3 * limit                             # ≈ 1 m 見えないと inspect を受けない
    err_max, sigma = res.error_at_blind_distance(1.0)
    assert err_max <= 3 * sigma + 0.05


def test_start_condition_is_sigma_and_sensor_health_not_source_name(cfg: dict, tmap: TagMap) -> None:
    est = PoseEstimator(cfg, tmap)
    est.correct([_exact_obs(tmap, 0, 0.5, 0.0, 0.0)])
    est.predict(0.1, 0.01, 0.0, False)
    e, h = est.estimate(0.1), est.health(0.1)
    assert e.source == "APRILTAG" and any("IMU" in b for b in start_blockers(cfg, e, h))  # IMU 未受信なら止める
    est.imu_yaw(0.1, 0.0)
    assert start_blockers(cfg, e, est.health(0.1)) == []
    assert any("オドメトリ" in b for b in start_blockers(cfg, e, est.health(5.0)))          # 位相が止まったら止める
    pose = e.finding_pose()
    assert set(pose) == {"x_m", "y_m", "yaw_rad", "sigma_xy_m", "sigma_yaw_rad", "pose_source"}
    ready = AutonomyInputs(True, "apriltag", 0.1, True, True, True, 5, 5, 0.2, sigma_xy_m=0.05, sigma_yaw_rad=0.05,
                           imu_ok=True, odometry_ok=True)
    why = [w for w in autonomy_blockers(cfg, ready) if "電気安全ゲート" not in w]
    assert why == []
    no_sigma = AutonomyInputs(True, "apriltag", 0.1, True, True, True, 5, 5, 0.2)
    assert any("σ" in w for w in autonomy_blockers(cfg, no_sigma))
    wide = AutonomyInputs(True, "odometry_imu", 0.1, True, True, True, 5, 5, 0.2, 0.5, 0.05, True, True)
    assert any("不確かさが大きい" in w for w in autonomy_blockers(cfg, wide))
    no_imu = AutonomyInputs(True, "apriltag", 0.1, True, True, True, 5, 5, 0.2, 0.05, 0.05, False, True)
    assert any("IMU" in w for w in autonomy_blockers(cfg, no_imu))
    assert PoseSource.APRILTAG.is_vision and PoseSource.ARUCO_EXTERNAL is PoseSource.ARUCO


def test_synthetic_apriltag_images_give_range_bearing_and_facing(cfg: dict, tmap: TagMap) -> None:
    """合成画像（実カメラではない）。距離 3%・方位 0.02rad・面の向き 0.06rad 以内。"""
    det = AprilTagDetector(cfg, tmap.tag_size_m, tmap.family)
    for tid, pb, yaw_b, pitch in ((1, (0.8, 0.1, 0.05), math.pi, 0.0), (2, (1.5, -0.3, 0.05), 2.5, 0.0),
                                  (3, (0.9, 0.0, 0.05), math.pi, 10.0)):
        img = det.render_synthetic(tid, np.array(pb), yaw_b, pitch)
        obs = det.detect(img, 1.0, pitch)
        assert len(obs) == 1 and obs[0].tag_id == tid and not obs[0].simulated
        o = obs[0]
        assert o.range_m == pytest.approx(math.hypot(pb[0], pb[1]), rel=0.03)
        assert o.bearing_rad == pytest.approx(math.atan2(pb[1], pb[0]), abs=0.02)
        assert abs(wrap(o.rel_yaw_rad - yaw_b)) < 0.06
    assert det.detect(np.full((480, 640), 200, np.uint8), 0.0, 0.0) == []


def test_sim_observer_respects_fov_range_and_facing(cfg: dict, tmap: TagMap) -> None:
    obs = SimTagObserver(cfg, tmap, np.random.default_rng(0))
    ids = {v[0] for v in obs.visible(1.0, 0.0, 0.0)}
    assert 1 in ids and 0 not in ids                                                      # 前方の奥の壁は見え、背後の原点は見えない
    assert obs.visible(2.9, 0.5, 0.0) == []                                                # 近すぎる（min_range）
    assert obs.observe(0.0, 1.0, 0.0, 0.0, blind=True) == []
    assert all(o.simulated for o in obs.observe(0.0, 1.0, 0.0, 0.0))
