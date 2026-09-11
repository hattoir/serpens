"""ヘビの位置姿勢の推定（ArUco でもシミュレータでも同じ処理を通す）。

  位置     = 首マーカ（J7 より胴体側）
  θ_body   = 尾マーカ → 首マーカ の向き。蛇行で激しく振れるので、
             時定数 = 歩容 1 周期ぶんのローパスをかけた値を移動制御に使う（生値も保持）
  θ_head   = θ_body + J8 の現在角（頭がどちらを向いているか。「人を見ているか」の判定用）

マーカが片方しか見えないとき:
  首だけ  … 位置は更新、向きは前回値を保持
  尾だけ  … 前回の「尾→首」の距離と向きで首の位置を推定、向きは保持（estimated=True）
  両方×  … 前回値を保持（stale 判定は age で）
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np


HEADING_FILTERS = ("moving_average", "lowpass")


@dataclass(frozen=True)
class SnakePose:
    """推定結果。角度は rad。"""

    x: float
    y: float
    theta_body_raw: float
    theta_body: float
    theta_head: float
    t: float                 # 最後にマーカが見えた時刻
    neck_seen: bool
    tail_seen: bool
    estimated: bool = False  # 尾マーカだけから推定した位置


def wrap_pi(a: float) -> float:
    """角度を (-π, π] に折り返す。"""
    return math.atan2(math.sin(a), math.cos(a))


class SnakePoseTracker:
    """マーカ観測から SnakePose を作る。"""

    def __init__(self, cfg: dict[str, Any]) -> None:
        sp = cfg["snake_pose"]
        ref = cfg["gait"]["presets"][sp["reference_gait"]]
        period = 1.0 / abs(float(ref["temporal_freq_hz"]))
        self.tau_s = float(sp["heading_tau_periods"]) * period
        self.stale_after_s = float(sp["stale_after_s"])
        self.method = str(sp["heading_filter"])
        if self.method not in HEADING_FILTERS:
            raise ValueError(f"snake_pose.heading_filter: {self.method}（{HEADING_FILTERS}）")
        self._window: deque[tuple[float, np.ndarray]] = deque()   # 移動平均用 (時刻, 単位ベクトル)
        self._vec: np.ndarray | None = None      # ローパス中の単位ベクトル（cos, sin）
        self._last_t: float | None = None
        self._span_mm: float | None = None       # 尾→首マーカ間距離の最新値
        self.pose: SnakePose | None = None

    def update(self, t: float, neck: np.ndarray | None, tail: np.ndarray | None, j8_deg: float) -> SnakePose | None:
        """観測を1回取り込む。neck / tail は見えなければ None。"""
        prev = self.pose
        raw = prev.theta_body_raw if prev else None
        est = False
        if neck is not None and tail is not None:
            d = np.asarray(neck, float) - np.asarray(tail, float)
            raw = math.atan2(float(d[1]), float(d[0]))
            self._span_mm = float(np.linalg.norm(d))
            pos = np.asarray(neck, float)
        elif neck is not None:
            pos = np.asarray(neck, float)
        elif tail is not None and prev is not None and self._span_mm is not None:
            pos = np.asarray(tail, float) + self._span_mm * np.array([math.cos(prev.theta_body_raw),
                                                                      math.sin(prev.theta_body_raw)])
            est = True
        else:
            if prev is not None:
                self.pose = SnakePose(prev.x, prev.y, prev.theta_body_raw, prev.theta_body,
                                      wrap_pi(prev.theta_body + math.radians(j8_deg)), prev.t, False, False)
            return self.pose
        if raw is None:          # 最初の観測で向きが分からない
            return None
        body = self._lowpass(raw, t)
        self.pose = SnakePose(float(pos[0]), float(pos[1]), raw, body,
                              wrap_pi(body + math.radians(j8_deg)), t,
                              neck is not None, tail is not None, est)
        return self.pose

    def _lowpass(self, raw: float, t: float) -> float:
        """単位ベクトルでフィルタする（±π の折り返しで暴れない）。

        moving_average … 直近 tau_s 秒（= 1周期）の平均。蛇行の振れを周期ごと打ち消し、遅れは τ/2
        lowpass       … 一次遅れ。定常旋回（角速度 ω）では ω·τ だけ遅れる
        """
        v = np.array([math.cos(raw), math.sin(raw)])
        if self.method == "moving_average":
            self._window.append((t, v))
            while self._window and self._window[0][0] < t - self.tau_s:
                self._window.popleft()
            m = np.mean([w for _, w in self._window], axis=0)
            self._last_t = t
            return math.atan2(float(m[1]), float(m[0]))
        if self._vec is None or self._last_t is None:
            self._vec = v
        else:
            dt = max(t - self._last_t, 0.0)
            a = 1.0 - math.exp(-dt / self.tau_s) if self.tau_s > 0 else 1.0
            self._vec = self._vec + a * (v - self._vec)
        self._last_t = t
        return math.atan2(float(self._vec[1]), float(self._vec[0]))

    def age_s(self, now: float) -> float:
        """最後にマーカが見えてからの秒数。"""
        return math.inf if self.pose is None else now - self.pose.t

    def stale(self, now: float) -> bool:
        """stale_after_s 以上マーカが見えていない。"""
        return self.age_s(now) > self.stale_after_s
