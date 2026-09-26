"""自己位置の閉ループ模擬（**KINEMATIC_SIM 相当。実機ではない**）: 部屋を周回し、家具の下（blind）でタグが見えない。

真の運動: 歩容の前進量 × (1 − slip)、指令の回転 + 雑音。推定器には指令の前進量と IMU（磁北基準 + 雑音）だけを渡す。
結果は「タグを見失ってからの距離 → σ と実誤差」を数字で出すため（フェーズ 3 の完成条件）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from serpens.localization.estimator import PoseEstimator, start_blockers, wrap
from serpens.localization.sim_observer import GaitOdometrySim, SimTagObserver
from serpens.localization.tag_map import TagMap


@dataclass
class LoopSimResult:
    t: list[float] = field(default_factory=list)
    err_m: list[float] = field(default_factory=list)
    sigma_m: list[float] = field(default_factory=list)
    yaw_err_rad: list[float] = field(default_factory=list)
    sigma_yaw_rad: list[float] = field(default_factory=list)
    blind_dist_m: list[float] = field(default_factory=list)   # 最後のタグ補正からの走行距離
    source: list[str] = field(default_factory=list)
    blocked: list[bool] = field(default_factory=list)
    corrections: int = 0
    rejected: int = 0

    def consistency(self, k: float = 3.0) -> float:
        """|誤差| ≤ k·σ だった割合（初期化後）。σ が正直なら 1 に近い。"""
        pairs = [(e, s) for e, s, src in zip(self.err_m, self.sigma_m, self.source) if src != "UNKNOWN"]
        return sum(e <= k * s for e, s in pairs) / max(len(pairs), 1)

    def error_at_blind_distance(self, d_m: float, tol: float = 0.1) -> tuple[float, float]:
        """タグ補正から d_m ± tol 走った時点の (実誤差の最大, σ の中央値)。"""
        rows = [(e, s) for e, s, b in zip(self.err_m, self.sigma_m, self.blind_dist_m) if abs(b - d_m) <= tol]
        if not rows:
            return float("nan"), float("nan")
        return max(e for e, _ in rows), float(np.median([s for _, s in rows]))

    def blind_distance_until_blocked(self) -> float | None:
        """タグ補正のあと、開始条件が σ で塞がるまでの走行距離（最小）。"""
        ds = [b for b, blk, src in zip(self.blind_dist_m, self.blocked, self.source) if blk and src != "UNKNOWN"]
        return min(ds) if ds else None


def run_loop_sim(cfg: dict[str, Any], seed: int = 0, seconds: float = 240.0, slip_frac: float = 0.10,
                 blind_box: tuple[float, float, float, float] | None = (1.2, 1.9, 0.3, 1.3), imu_offset_rad: float = 0.7,
                 dt: float = 0.1) -> LoopSimResult:
    """長方形の経路を周回。blind_box = (x0, x1, y0, y1) の中ではタグが見えない（家具の下）。"""
    tm = TagMap.from_cfg(cfg)
    rng = np.random.default_rng(seed)
    obs, est = SimTagObserver(cfg, tm, rng), PoseEstimator(cfg, tm)
    sim = cfg["localization"]["sim"]
    odo = GaitOdometrySim(float(cfg["behavior"]["controller"]["advance_per_cycle_mm"]) / 1000.0, slip_frac,
                          float(sim["true_yaw_noise_rad"]))
    wps = [tuple(map(float, w)) for w in sim["waypoints"]]
    x, y, yaw, wi = wps[-1][0], wps[-1][1], 0.0, 0
    freq, max_turn = float(sim["temporal_freq_hz"]), float(sim["max_turn_rad_s"])
    res = LoopSimResult()
    for i in range(int(seconds / dt)):
        t = i * dt
        tx, ty = wps[wi]
        if math.hypot(tx - x, ty - y) < float(sim["waypoint_reach_m"]):
            wi = (wi + 1) % len(wps)
            tx, ty = wps[wi]
        want = wrap(math.atan2(ty - y, tx - x) - yaw)
        turn = max(-max_turn, min(max_turn, want * float(sim["turn_gain"]))) * dt
        ds, dyaw, tds, tdyaw = odo.step(2 * math.pi * freq * dt, turn, rng)
        x += tds * math.cos(yaw + tdyaw / 2)
        y += tds * math.sin(yaw + tdyaw / 2)
        yaw = wrap(yaw + tdyaw)
        imu = yaw + imu_offset_rad + rng.normal(0, float(cfg["localization"]["imu"]["yaw_sigma_rad"]))
        d = None if est.last_imu_yaw is None else wrap(imu - est.last_imu_yaw)
        est.predict(t, ds, d if d is not None else dyaw, d is not None)
        est.imu_yaw(t, imu)
        blind = blind_box is not None and blind_box[0] <= x <= blind_box[1] and blind_box[2] <= y <= blind_box[3]
        res.corrections += est.correct(obs.observe(t, x, y, yaw, blind=blind))
        e = est.estimate(t)
        res.t.append(t)
        res.err_m.append(math.hypot(e.x_m - x, e.y_m - y))
        res.sigma_m.append(e.sigma_xy_m)
        res.yaw_err_rad.append(abs(wrap(e.yaw_rad - yaw)))
        res.sigma_yaw_rad.append(e.sigma_yaw_rad)
        res.blind_dist_m.append(est.dist_since_fix)
        res.source.append(e.source)
        res.blocked.append(bool(start_blockers(cfg, e, est.health(t))))
    res.rejected = est.rejected
    return res
