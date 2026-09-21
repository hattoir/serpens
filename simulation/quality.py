"""動きの質の指標（蛇らしさ・愛着）。**実機ゼロで今日から回せる。すべて SIMULATED。**

  頭部軌道の LDJ     対数無次元ジャーク（Balasubramanian et al. 2015; 判別感度 z=5.28: Front. Neurol. 9:615）。
                     試行時間を統制して相対比較にだけ使う（値そのものに絶対の意味は無い）
  静止率             巡回中、止まっている時間の割合（間欠移動する動物は約 50%: Integr. Comp. Biol. 41(2):137）
  可視波数           胴体中心線の曲がりの符号が変わる回数 / 2（= 体に乗っている波の数）
  一次反応レイテンシ 刺激（人が現れる）→ 最初の可視変化（目 or J8）。因果の窓は 140ms〜1s（Front. Psychol. 14:1167809）
  GAR                人を見ていない時間 / 人と向き合っている時間（0.5〜0.7 が会話を最も長くする: Front. Robot. AI 10:1062714）

歩容の追従性（trail_error）は蛇らしさではない。車輪で横滑りゼロ拘束をかけたシミュレータでは
機構で決まる値で、動きの質を反映しない。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from serpens.behavior.brain import STILL_STATES
from serpens.motion.poses import HEAD_YAW
from serpens.sim.session import SimSession
from serpens.sim.virtual_camera import SimPerson
from serpens.sim.world import BodyPose

SOURCE = "KINEMATIC_SIM"


# ---- 指標の計算（配列を受け取る。セッションを知らない） ----------------------------------------
def log_dimensionless_jerk(xy: np.ndarray, dt: float) -> float:
    """LDJ = −ln( T³ / v_peak² · ∫|x⃛|² dt )。値が大きい（0 に近い）ほど滑らか。"""
    p = np.asarray(xy, float)
    if len(p) < 8:
        return float("nan")
    v = np.gradient(p, dt, axis=0)
    a = np.gradient(v, dt, axis=0)
    j = np.gradient(a, dt, axis=0)
    v_peak = float(np.max(np.linalg.norm(v, axis=1)))
    if v_peak <= 0.0:
        return float("nan")
    T = dt * (len(p) - 1)
    dlj = T ** 3 / v_peak ** 2 * float(np.sum(np.sum(j ** 2, axis=1)) * dt)
    return -math.log(dlj) if dlj > 0 else float("nan")


def visible_waves(points_xy: np.ndarray, min_turn_deg: float) -> float:
    """中心線（尾→頭の点列）の曲がりの符号が変わる回数 / 2。小さな曲がりは無視する。"""
    p = np.asarray(points_xy, float)
    d = np.diff(p, axis=0)
    ang = np.arctan2(d[:, 1], d[:, 0])
    turn = np.degrees(np.arctan2(np.sin(np.diff(ang)), np.cos(np.diff(ang))))
    sig = np.sign(turn[np.abs(turn) >= min_turn_deg])
    if len(sig) < 2:
        return 0.0
    return float(np.sum(sig[1:] != sig[:-1])) / 2.0 + 0.5      # 山1つ = 符号変化1回 = 0.5波 + 端の半波


# ---- セッションを回して測る -------------------------------------------------------------
@dataclass
class QualityReport:
    """1回の評価。source は必ず付く。"""

    source: str = SOURCE
    seconds_alone: float = 0.0
    still_ratio: float = float("nan")          # 巡回中の静止率
    head_ldj: float = float("nan")             # 頭先端軌道の LDJ（試行時間 seconds_alone で統制）
    visible_waves: float = float("nan")        # 歩容中の可視波数（平均）
    primary_latency_s: float = float("nan")    # 人が現れてから最初の可視変化まで（模擬。カメラの遅れは含まない）
    vision_budget_s: float = 0.0               # config 上のカメラ遅れ（検出周期 + 遅延）。実測ではない
    gar: float = float("nan")                  # 人と向き合っている間に、見ていない時間の割合
    notes: list[str] = field(default_factory=list)

    def as_row(self) -> dict[str, float | str]:
        return {k: (round(v, 3) if isinstance(v, float) else v) for k, v in self.__dict__.items() if k != "notes"}


def start_pose(cfg: dict[str, Any]) -> BodyPose:
    q = cfg["quality"]["start"]
    return BodyPose(float(q["x_mm"]), float(q["y_mm"]), math.radians(float(q["theta_deg"])))


def measure_alone(cfg: dict[str, Any], seconds: float, seed: int, report: QualityReport) -> None:
    """誰もいない巡回: 静止率（歩容の振幅がゼロの割合）・頭先端の LDJ・可視波数。"""
    s = SimSession(cfg, start_pose(cfg), seed=seed)
    dt = s.ctrl_dt
    head, still, waves = [], [], []
    min_turn = float(cfg["quality"]["wave_min_turn_deg"])
    while s.t < seconds:
        s.step()
        tip = s.world.head_tip()[:2].copy()
        head.append(tip)
        if s.brain.fsm.state == "PATROL":
            still.append(not s.anim.gait.active)
            if s.anim.gait.active:
                waves.append(visible_waves(s.world.world_points()[:, :2], min_turn))
    report.seconds_alone = seconds
    report.still_ratio = float(np.mean(still)) if still else float("nan")
    report.head_ldj = log_dimensionless_jerk(np.array(head), dt)
    report.visible_waves = float(np.mean(waves)) if waves else float("nan")


def measure_with_person(cfg: dict[str, Any], seconds: float, seed: int, report: QualityReport,
                        appear_s: float, person: tuple[float, float]) -> None:
    """人が現れる: 一次反応レイテンシと GAR。"""
    s = SimSession(cfg, start_pose(cfg), seed=seed)
    while s.t < appear_s:
        s.step()
    j8_0 = s.anim.last_output[HEAD_YAW]
    br_0 = s.head.eye_rgb_brightness if s.head is not None else None
    t0 = s.t
    s.people = [SimPerson(*person)]
    first: float | None = None
    looking: list[float] = []
    tol = float(cfg["behavior"]["stimuli"]["looking_tolerance_deg"])
    while s.t < seconds:
        s.step()
        b = s.brain
        if first is None:
            eye = s.head is not None and s.head.eye_rgb_brightness != br_0
            if eye or abs(s.anim.last_base[HEAD_YAW] - j8_0) > float(cfg["quality"]["visible_head_change_deg"]):
                first = s.t - t0
        if b.fsm.state in STILL_STATES and b._last_snake is not None and b._last_person_raw is not None:
            need = b._yaw_to(b._last_snake, b._last_person_raw)
            looking.append(1.0 if abs(need - s.anim.last_output[HEAD_YAW]) <= tol else 0.0)
    report.primary_latency_s = float("nan") if first is None else first
    p, v = cfg["person"], cfg["vision_sim"]
    report.vision_budget_s = 1.0 / float(p["detect_hz"]) + float(v["latency_s"])
    report.gar = float("nan") if not looking else 1.0 - float(np.mean(looking))
    if not looking:
        report.notes.append("人と向き合う状態（ALERT/OBSERVE/ENGAGE）に入らなかった")


def evaluate(cfg: dict[str, Any], seconds_alone: float, seconds_person: float, seed: int) -> QualityReport:
    r = QualityReport()
    measure_alone(cfg, seconds_alone, seed, r)
    q = cfg["quality"]
    measure_with_person(cfg, seconds_person, seed, r, float(q["person_appear_s"]), tuple(q["person_xy_mm"]))
    return r
