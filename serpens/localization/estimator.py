"""自己位置の推定（座標系 home、m / rad）: 歩容オドメトリ + IMU で予測し、AprilTag の観測で補正する。

状態 (x, y, yaw) と 3×3 の共分散 P を持つ最小の Kalman フィルタ。
  predict(ds, dyaw)  … 前進量 ds（歩容の位相 × 1 周期の前進量）と回転 dyaw（IMU の yaw 差分。無ければ歩容の指令）で進め、
                       誤差を config の割合で積む（**歩容の滑りは未測定 → 割合は ASSUMED**）
  imu_yaw(yaw)       … IMU の絶対 yaw（磁北基準）。home との向きの差（offset）はタグで補正したときに求め、
                       以後は yaw の観測として更新に使う（向きの σ が育たない。位置の σ は前進量で育つ）
  correct(観測)      … タグ 1 枚の (距離, 方位, タグ面の相対向き) から姿勢が一意に決まるので、姿勢の直接観測として更新。
                       Mahalanobis 距離でゲート（偽検出・誤 ID を捨てる）。未初期化ならそのまま受け入れる
  estimate()         … 姿勢 + σ_xy + σ_yaw + 出どころ（APRILTAG: 最近補正した / ODOMETRY_IMU: 推測航法中 / UNKNOWN: 未初期化）
  health()           … IMU の鮮度、オドメトリの有無、最後のタグからの時間。**開始条件は σ と健全性で判定する**（出どころの名前ではない）

ロボット座標: x 前、y 左、yaw 反時計回り（home と同じ向きの定義）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from serpens.localization.tag_map import TagMap


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class TagObservation:
    """頭カメラから見たタグ 1 枚。range = 水平距離、bearing = 機体正面からの方位、rel_yaw = タグの面の向き（機体座標）。"""

    tag_id: int
    range_m: float
    bearing_rad: float
    rel_yaw_rad: float
    t_s: float
    simulated: bool = False


@dataclass(frozen=True)
class PoseEstimate:
    x_m: float
    y_m: float
    yaw_rad: float
    sigma_xy_m: float
    sigma_yaw_rad: float
    source: str                        # APRILTAG / ODOMETRY_IMU / UNKNOWN
    t_s: float
    last_tag_age_s: float | None
    simulated: bool                    # 観測が模擬（合成）なら True

    def finding_pose(self) -> dict[str, Any]:
        """floor_finding.pose の形（schemas/common.json#/$defs/pose）。"""
        return {"x_m": round(self.x_m, 4), "y_m": round(self.y_m, 4), "yaw_rad": round(self.yaw_rad, 4),
                "sigma_xy_m": round(self.sigma_xy_m, 4), "sigma_yaw_rad": round(self.sigma_yaw_rad, 4),
                "pose_source": self.source}                    # 模擬でも出どころは推定器の値（真値ではない）


@dataclass(frozen=True)
class LocalizationHealth:
    initialized: bool
    imu_ok: bool
    imu_age_s: float | None
    odometry_ok: bool
    last_tag_age_s: float | None
    tags_rejected: int


@dataclass
class PoseEstimator:
    cfg: dict[str, Any]
    tag_map: TagMap
    t_s: float = 0.0
    x: np.ndarray = field(default_factory=lambda: np.zeros(3))
    P: np.ndarray = field(default_factory=lambda: np.eye(3) * 1e6)
    initialized: bool = False
    last_tag_t: float | None = None
    last_imu_t: float | None = None
    last_imu_yaw: float | None = None
    imu_offset: float | None = None        # home の yaw − IMU の yaw（タグで補正するたびに更新）
    imu_offset_t: float | None = None
    last_odom_t: float | None = None
    rejected: int = 0
    simulated: bool = False
    dist_since_fix: float = 0.0            # 最後のタグ補正からの走行距離（滑りは系統誤差なので σ は距離に比例して育てる）
    turn_since_fix: float = 0.0

    def __post_init__(self) -> None:
        loc = self.cfg["localization"]
        self.od, self.imu, self.at = loc["odometry"], loc["imu"], loc["apriltag"]

    # ---- 予測 -------------------------------------------------------------------
    def imu_yaw(self, t: float, yaw_rad: float) -> float | None:
        """IMU の絶対 yaw を入れる。前回との差分を返す（predict の dyaw に使う）。最初は None。
        offset が分かっていれば絶対 yaw で向きを更新する（predict の後に呼ぶこと）。"""
        d = None if self.last_imu_yaw is None else wrap(yaw_rad - self.last_imu_yaw)
        self.last_imu_t, self.last_imu_yaw = t, yaw_rad
        if self.initialized and self.imu_offset is not None:
            age = t - (self.imu_offset_t or t)
            r = wrap(yaw_rad + self.imu_offset - self.x[2])
            R = float(self.imu["yaw_sigma_rad"]) ** 2 + (float(self.imu["yaw_drift_rad_per_s"]) * age) ** 2
            K = self.P[:, 2] / (self.P[2, 2] + R)
            self.x = self.x + K * r
            self.x[2] = wrap(self.x[2])
            self.P = self.P - np.outer(K, self.P[2, :])
        return d

    def predict(self, t: float, ds_m: float, dyaw_rad: float, yaw_from_imu: bool) -> None:
        """前進 ds（負なら後退）と回転 dyaw で状態を進め、共分散を積む。"""
        dt = max(t - self.t_s, 0.0)
        th_mid = self.x[2] + 0.5 * dyaw_rad
        c, s = math.cos(th_mid), math.sin(th_mid)
        self.x = np.array([self.x[0] + ds_m * c, self.x[1] + ds_m * s, wrap(self.x[2] + dyaw_rad)])
        F = np.array([[1.0, 0.0, -ds_m * s], [0.0, 1.0, ds_m * c], [0.0, 0.0, 1.0]])
        # 滑りは系統誤差: σ = 割合 × 累積距離 になるよう、分散の増分を (D+ds)² − D² で積む（1 歩ごとの独立雑音にしない）
        D, D2 = self.dist_since_fix, self.dist_since_fix + abs(ds_m)
        grow = D2 * D2 - D * D
        q_along = float(self.od["sigma_along_frac"]) ** 2 * grow
        q_lat = float(self.od["sigma_lateral_frac"]) ** 2 * grow
        if yaw_from_imu:
            q_yaw = (float(self.imu["delta_sigma_rad"]) + float(self.imu["yaw_drift_rad_per_s"]) * dt) ** 2
        else:
            T, T2 = self.turn_since_fix, self.turn_since_fix + abs(dyaw_rad)
            q_yaw = float(self.od["sigma_yaw_frac"]) ** 2 * (T2 * T2 - T * T) + float(self.od["sigma_yaw_floor_rad"]) ** 2
        self.dist_since_fix, self.turn_since_fix = D2, self.turn_since_fix + abs(dyaw_rad)
        R = np.array([[c, -s], [s, c]])
        Q = np.zeros((3, 3))
        Q[:2, :2] = R @ np.diag([q_along, q_lat]) @ R.T
        Q[2, 2] = q_yaw
        self.P = F @ self.P @ F.T + Q
        self.t_s, self.last_odom_t = t, t

    # ---- 補正 -------------------------------------------------------------------
    def _measurement(self, o: TagObservation) -> tuple[np.ndarray, np.ndarray] | None:
        tag = self.tag_map.get(o.tag_id)
        if tag is None or not (float(self.at["min_range_m"]) <= o.range_m <= float(self.at["max_range_m"])):
            return None
        yaw = wrap(tag.yaw_rad - o.rel_yaw_rad)
        a = yaw + o.bearing_rad
        z = np.array([tag.x_m - o.range_m * math.cos(a), tag.y_m - o.range_m * math.sin(a), yaw])
        s_r = float(self.at["sigma_range_frac"]) * o.range_m
        s_b, s_y = float(self.at["sigma_bearing_rad"]), float(self.at["sigma_yaw_rad"])
        s_pos2 = s_r ** 2 + (o.range_m * s_b) ** 2 + (o.range_m * s_y) ** 2
        return z, np.diag([s_pos2, s_pos2, s_y ** 2])

    def correct(self, observations: list[TagObservation]) -> int:
        """タグ観測で更新。受け入れた数を返す（ゲートで捨てた数は rejected に積む）。"""
        n = 0
        for o in observations:
            m = self._measurement(o)
            if m is None:
                continue
            z, R = m
            if not self.initialized:
                self.x, self.P, self.initialized = z.copy(), R.copy(), True
            else:
                r = z - self.x
                r[2] = wrap(r[2])
                S = self.P + R
                d2 = float(r @ np.linalg.solve(S, r))
                if d2 > float(self.at["gate_chi2"]):
                    self.rejected += 1
                    continue
                K = self.P @ np.linalg.inv(S)
                self.x = self.x + K @ r
                self.x[2] = wrap(self.x[2])
                self.P = (np.eye(3) - K) @ self.P
            self.last_tag_t, self.t_s = max(o.t_s, self.last_tag_t or -math.inf), max(self.t_s, o.t_s)
            self.dist_since_fix, self.turn_since_fix = 0.0, 0.0
            if self.last_imu_yaw is not None:                                   # home と IMU の向きの差を取り直す
                self.imu_offset, self.imu_offset_t = wrap(float(self.x[2]) - self.last_imu_yaw), o.t_s
            self.simulated = self.simulated or o.simulated
            n += 1
        return n

    # ---- 出力 -------------------------------------------------------------------
    def estimate(self, t: float | None = None) -> PoseEstimate:
        t = self.t_s if t is None else t
        age = None if self.last_tag_t is None else max(t - self.last_tag_t, 0.0)
        if not self.initialized:
            source = "UNKNOWN"
        elif age is not None and age <= float(self.at["hold_s"]):
            source = "APRILTAG"
        else:
            source = "ODOMETRY_IMU"
        sxy = math.sqrt(max(float(self.P[0, 0] + self.P[1, 1]) / 2.0, 0.0))     # 位置 σ（x, y の平均。円で近似）
        return PoseEstimate(float(self.x[0]), float(self.x[1]), float(self.x[2]), sxy, math.sqrt(max(float(self.P[2, 2]), 0.0)),
                            source, t, age, self.simulated)

    def health(self, t: float | None = None) -> LocalizationHealth:
        t = self.t_s if t is None else t
        imu_age = None if self.last_imu_t is None else max(t - self.last_imu_t, 0.0)
        imu_ok = imu_age is not None and imu_age <= float(self.imu["max_age_s"])
        odo_ok = self.last_odom_t is not None and t - self.last_odom_t <= float(self.od["max_age_s"])
        age = None if self.last_tag_t is None else max(t - self.last_tag_t, 0.0)
        return LocalizationHealth(self.initialized, imu_ok, imu_age, odo_ok, age, self.rejected)


def start_blockers(cfg: dict[str, Any], est: PoseEstimate, health: LocalizationHealth) -> list[str]:
    """自律走行 / inspect_point を始められない理由（σ と局所センサーの健全性。出どころの名前では判定しない）。"""
    st = cfg["localization"]["start"]
    out: list[str] = []
    if not health.initialized or est.source == "UNKNOWN":
        out.append("自己位置が未初期化（AprilTag をまだ 1 枚も見ていない）")
        return out
    if est.sigma_xy_m > float(st["max_sigma_xy_m"]):
        out.append(f"自己位置の不確かさが大きい（σ_xy {est.sigma_xy_m:.2f}m > {st['max_sigma_xy_m']}m）。タグを見るまで待つ")
    if est.sigma_yaw_rad > float(st["max_sigma_yaw_rad"]):
        out.append(f"向きの不確かさが大きい（σ_yaw {est.sigma_yaw_rad:.2f}rad > {st['max_sigma_yaw_rad']}rad）")
    if bool(st["require_imu"]) and not health.imu_ok:
        out.append("IMU が新しくない" + ("（未受信）" if health.imu_age_s is None else f"（{health.imu_age_s:.1f}s）"))
    if not health.odometry_ok:
        out.append("オドメトリ（歩容の位相）が更新されていない")
    return out
