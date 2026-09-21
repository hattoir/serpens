"""動きの質（蛇らしさ・愛着）の回帰テスト。**SIMULATED。合格線は config の quality.thresholds。**

274 件のテストは「蛇らしさ」「愛着」を何も保証していなかった（外部レビュー）。ここで数値として押さえる。
"""
from __future__ import annotations

import numpy as np
import pytest

from serpens.config import load_config
from simulation.quality import evaluate, log_dimensionless_jerk, visible_waves

ALONE_S, PERSON_S, SEED = 120.0, 60.0, 1


@pytest.fixture(scope="module")
def cfg() -> dict:
    return load_config()


@pytest.fixture(scope="module")
def report(cfg: dict):
    return evaluate(cfg, ALONE_S, PERSON_S, SEED)


def test_metrics_are_computed_and_marked_as_simulation(report) -> None:
    assert report.source == "KINEMATIC_SIM"
    assert report.seconds_alone == ALONE_S
    assert all(np.isfinite(v) for v in (report.still_ratio, report.head_ldj, report.visible_waves,
                                        report.primary_latency_s, report.gar)), report.as_row()


def test_still_ratio_is_stop_and_go(cfg: dict, report) -> None:
    """静止率 40〜60%（間欠移動する動物の平均約 50%）。旧実装はほぼ 0%。"""
    lo, hi = cfg["quality"]["thresholds"]["still_ratio"]
    assert lo <= report.still_ratio <= hi, report.still_ratio


def test_primary_reaction_is_inside_the_causal_window(cfg: dict, report) -> None:
    """刺激 → 最初の可視変化 ≤ 300ms（模擬。カメラの遅れは別に vision_budget_s として持つ）。旧 1.4〜2.1 秒。"""
    assert report.primary_latency_s <= cfg["quality"]["thresholds"]["primary_latency_s_max"], report.primary_latency_s
    assert report.vision_budget_s > 0.0          # カメラの予算は隠さない


def test_gaze_aversion_ratio(cfg: dict, report) -> None:
    """GAR 0.4〜0.6（見ては外し、見ては外し）。旧 約 0.04。"""
    lo, hi = cfg["quality"]["thresholds"]["gar"]
    assert lo <= report.gar <= hi, report.gar


def test_visible_waves_and_ldj_band(cfg: dict, report) -> None:
    """可視波数は 9 軸案では 1 波が上限（構造的制約）。LDJ は統制した長さでの帯（大きく崩れたら気づく）。"""
    th = cfg["quality"]["thresholds"]
    assert report.visible_waves >= th["visible_waves_min"]
    lo, hi = th["head_ldj"]
    assert lo <= report.head_ldj <= hi, report.head_ldj


def test_yaw8_overlay_puts_two_waves_on_the_body() -> None:
    """8 軸案（config/robot_yaw8.yaml）: Ω=90° で体に 2 波が乗る。9 軸案（Ω=60°）は 1 波。"""
    from serpens.motion.gait import GaitEngine, GaitParams, body_joint_names, gait_period_s
    from serpens.sim.world import BodyPose, World

    def waves(cfg: dict, preset: str) -> float:
        p = GaitParams.from_cfg(cfg["gait"]["presets"][preset])
        w, eng = World(cfg, BodyPose(600.0, 600.0, 0.0)), GaitEngine(cfg)
        eng.start(p)
        dt, T, t, out = float(cfg["sim"]["dt_s"]), gait_period_s(p), 0.0, []
        for _ in range(int(round(2 * T / dt))):
            t += dt
            w.step(eng.update(t), dt)
            if t > T:
                out.append(visible_waves(w.world_points()[:, :2], float(cfg["quality"]["wave_min_turn_deg"])))
        return float(np.mean(out))

    base = load_config()
    yaw8 = load_config(overlay="config/robot_yaw8.yaml")
    assert len(body_joint_names(base)) == 6 and len(body_joint_names(yaw8)) == 8
    assert 1.0 <= waves(base, "forward") < 1.6
    assert waves(yaw8, "forward") >= 2.0
    assert waves(yaw8, "one_wave") < 1.6


def test_ldj_prefers_smooth_over_jerky() -> None:
    """LDJ の向き: 同じ距離・同じ時間なら、滑らかな軌道の方が値が大きい（0 に近い）。"""
    t = np.linspace(0.0, 2.0, 101)
    smooth = np.column_stack([t ** 3 * (10 - 15 * t / 2 + 6 * t ** 2 / 4), np.zeros_like(t)])   # 最小ジャーク
    jerky = np.column_stack([np.where(t < 1.0, 0.0, 1.0) * smooth[-1, 0], np.zeros_like(t)])
    assert log_dimensionless_jerk(smooth, 0.02) > log_dimensionless_jerk(jerky, 0.02)
