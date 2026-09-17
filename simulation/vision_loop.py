"""Vision 経由の閉ループ（Phase 4）と、真値との比較。

    仮想世界 → 仮想カメラ画像 → ArUco 検出 / 人物 bbox → 追跡 → World State（ARUCO, simulated）
    → Behavior → SimulatedSnake（仮想 ESP32 + 仮想サーボ）→ 仮想世界

**真値（GROUND_TRUTH_SIM）は採点にだけ使い、制御には入れない。** 結果の source は KINEMATIC_SIM。
画像も摩擦も模擬なので、誤差の数値は「処理の筋が通っているか」の目安であって実カメラの性能ではない。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from serpens.link.snake import SimulatedSnake
from serpens.perception.snake_pose import wrap_pi
from serpens.sim.session import ManualClock, SimSession
from serpens.sim.sim_vision import SimVisionObserver, VisionFaults
from serpens.sim.world import BodyPose
from simulation.bridge import drive_people
from simulation.virtual_person import VirtualPerson

START = BodyPose(600.0, 600.0, -math.pi / 2)      # 台の中央、来場者側（y<0）を向く


@dataclass
class VisionMetrics:
    """真値と比べた Vision の成績。**source = KINEMATIC_SIM（模擬画像）。**"""

    source: str = "KINEMATIC_SIM"
    steps: int = 0
    pos_err_mm: list[float] = field(default_factory=list)
    heading_err_deg: list[float] = field(default_factory=list)
    age_s: list[float] = field(default_factory=list)
    target_err_mm: list[float] = field(default_factory=list)
    off_mat_steps: int = 0
    travelled_mm: float = 0.0             # 首マーカが実際に動いた道のり（真値）
    moved_while_stale_mm: float = 0.0     # 自己位置が古い間に機体が動いた距離（真値）
    stops: list[str] = field(default_factory=list)

    def summary(self) -> dict[str, float]:
        def stat(v: list[float], f: Any) -> float:
            return float(f(v)) if v else float("nan")
        return {
            "pos_err_mean_mm": stat(self.pos_err_mm, np.mean),
            "pos_err_p95_mm": stat(self.pos_err_mm, lambda v: np.percentile(v, 95)),
            "pos_err_max_mm": stat(self.pos_err_mm, np.max),
            "heading_err_p95_deg": stat(self.heading_err_deg, lambda v: np.percentile(v, 95)),
            "age_max_s": stat(self.age_s, np.max),
            "target_err_p95_mm": stat(self.target_err_mm, lambda v: np.percentile(v, 95)),
            "off_mat_steps": float(self.off_mat_steps),
            "travelled_mm": self.travelled_mm,
            "moved_while_stale_mm": self.moved_while_stale_mm,
            "stops": float(len(self.stops)),
        }


class VisionLoop:
    """Vision を通した閉ループ 1 本（時計は 1 つ）。"""

    def __init__(self, cfg: dict[str, Any], faults: VisionFaults | None = None, seed: int = 5,
                 people: list[VirtualPerson] | None = None, start: BodyPose = START) -> None:
        self.cfg = cfg
        self.clock = ManualClock()
        self.snake = SimulatedSnake(cfg, self.clock)
        self.vision = SimVisionObserver(cfg, faults, seed=seed)
        self.session = SimSession(cfg, start, seed=seed, robot=self.snake, clock=self.clock,
                                  observer=self.vision)
        self.people = list(people or [])
        self.metrics = VisionMetrics()
        self.stale_after_s = float(cfg["behavior"]["safety"]["autonomy"]["pose_max_age_s"])
        self.mat = (float(cfg["mat"]["width_mm"]), float(cfg["mat"]["depth_mm"]))
        self._last_truth: np.ndarray | None = None
        self._stop_seen = 0

    def start_when_seen(self, timeout_s: float = 3.0) -> bool:
        """自己位置が入るまで待ってから開始操作をする（Vision の模擬は待機から始まる）。"""
        for _ in range(int(round(timeout_s / self.session.ctrl_dt))):
            self.step()
            if self.session.snake is not None and self.session.acknowledge_pose("VisionLoop")[0]:
                return self.session.request_start("VisionLoop")[0]
        return False

    def run(self, seconds: float) -> VisionMetrics:
        for _ in range(int(round(seconds / self.session.ctrl_dt))):
            self.step()
        return self.metrics

    def step(self) -> None:
        s, m = self.session, self.metrics
        drive_people(s, self.people, self.clock.t)
        s.step()
        m.steps += 1
        t = self.clock.t
        truth = s.world.marker_xy("neck")
        if self._last_truth is not None:
            d = float(np.linalg.norm(truth - self._last_truth))
            m.travelled_mm += d
            if s.snake is not None and t - s.snake.t > self.stale_after_s:
                m.moved_while_stale_mm += d
        self._last_truth = truth
        if s.snake is not None:
            # 推定は撮影時刻の値なので、その時刻の真値と比べるのが筋だが、制御が使うのは「いま」。
            # ここでは**制御から見た誤差**（いまの真値との差 = 検出誤差 + 遅れ）を測る
            m.pos_err_mm.append(float(math.hypot(s.snake.x - truth[0], s.snake.y - truth[1])))
            m.heading_err_deg.append(abs(math.degrees(wrap_pi(s.snake.theta_body_raw - s.world.snake_pose()[2]))))
            m.age_s.append(t - s.snake.t)
        if s.target is not None and s.people:
            m.target_err_mm.append(min(math.hypot(s.target.floor_mm[0] - p.x_mm, s.target.floor_mm[1] - p.y_mm)
                                       for p in s.people))
        pts = s.world.world_points()[:, :2]
        if (pts[:, 0].min() < 0 or pts[:, 1].min() < 0
                or pts[:, 0].max() > self.mat[0] or pts[:, 1].max() > self.mat[1]):
            m.off_mat_steps += 1
        new = s.stop.events[self._stop_seen:]
        self._stop_seen = len(s.stop.events)
        m.stops += [e.reason for e in new if e.source == "知覚"]
