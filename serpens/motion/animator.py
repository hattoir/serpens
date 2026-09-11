"""キーフレーム補間・歩容・呼吸を重ね合わせて、9軸の指令角を作る。

出力角 = キーフレーム（ベース姿勢） + 歩容（J1〜J6、加算） + 呼吸（全軸、加算）

- キーフレーム補間: ease-in-out / ease-out / オーバーシュート付き ease-out
- 呼吸: 全軸に ±amplitude・周期 period のサイン波を常時加算。ON/OFF は fade_s でなめらかに
- freeze(): 出力をその場で固定（驚きの全停止）。解除すると、止まっていた時間ぶん
  すべて（補間・歩容・呼吸）が一時停止していたように続きから動く
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Any

from serpens.hw.servo_bus import Goal, ServoBus
from serpens.motion.gait import GaitEngine
from serpens.motion.poses import Pose


class Easing(Enum):
    """補間の種類。"""

    IN_OUT = "ease_in_out"
    OUT = "ease_out"
    OUT_OVERSHOOT = "ease_out_overshoot"


def ease_in_out(p: float) -> float:
    """3次の ease-in-out（0→1）。"""
    p = min(max(p, 0.0), 1.0)
    return 4.0 * p ** 3 if p < 0.5 else 1.0 - (-2.0 * p + 2.0) ** 3 / 2.0


def ease_out(p: float) -> float:
    """3次の ease-out（0→1）。"""
    p = min(max(p, 0.0), 1.0)
    return 1.0 - (1.0 - p) ** 3


# 各補間の「最大の傾き / 平均の傾き」（3次式なので 3）
EASE_PEAK_SLOPE = 3.0


def min_duration_s(start: float, end: float, easing: Easing, overshoot_deg: float,
                   peak_ratio: float, max_speed_dps: float) -> float:
    """最高角速度が max_speed_dps を超えないために必要な最短時間。"""
    if easing is Easing.OUT_OVERSHOOT and end != start:
        return EASE_PEAK_SLOPE * (abs(end - start) + abs(overshoot_deg)) / (peak_ratio * max_speed_dps)
    return EASE_PEAK_SLOPE * abs(end - start) / max_speed_dps


def overshoot_value(start: float, end: float, p: float, overshoot_deg: float, peak_ratio: float) -> float:
    """行き過ぎ付きの補間値。

    0〜peak_ratio: start → end + overshoot（ease-out）
    peak_ratio〜1: そこから end に戻る（ease-in-out）
    """
    if end == start or overshoot_deg == 0.0:
        return start + (end - start) * ease_out(p)
    peak = end + math.copysign(overshoot_deg, end - start)
    if p < peak_ratio:
        return start + (peak - start) * ease_out(p / peak_ratio)
    return peak + (end - peak) * ease_in_out((p - peak_ratio) / (1.0 - peak_ratio))


@dataclass(frozen=True)
class Keyframe:
    """目標姿勢（一部の関節だけでもよい）と到達までの時間。"""

    pose: Pose
    duration_s: float
    easing: Easing = Easing.IN_OUT
    overshoot_deg: float = 0.0


@dataclass
class _Track:
    start: float
    end: float
    t0: float
    duration: float
    easing: Easing
    overshoot_deg: float

    def value(self, t: float, peak_ratio: float) -> float:
        p = 1.0 if self.duration <= 0 else (t - self.t0) / self.duration
        if p >= 1.0:
            return self.end
        if self.easing is Easing.OUT_OVERSHOOT:
            return overshoot_value(self.start, self.end, p, self.overshoot_deg, peak_ratio)
        f = ease_out(p) if self.easing is Easing.OUT else ease_in_out(p)
        return self.start + (self.end - self.start) * f


class Animator:
    """9軸の指令角を毎周期計算する。時刻 t は呼び出し側が与える（テストしやすいように）。"""

    def __init__(self, cfg: dict[str, Any]) -> None:
        self.names: list[str] = [j["name"] for j in cfg["joints"]]
        self._ids = {j["name"]: int(j["servo_id"]) for j in cfg["joints"]}
        self._limits = {j["name"]: (float(j["min_deg"]), float(j["max_deg"])) for j in cfg["joints"]}
        a, b = cfg["animator"], cfg["breath"]
        self._speed = float(a["default_speed_dps"])
        self._accel = float(a["default_accel_dps2"])
        self._peak_ratio = float(a["overshoot_peak_ratio"])
        self._vmax = float(a["max_joint_speed_dps"])
        self._b_amp = float(b["amplitude_deg"])
        self._b_period = float(b["period_s"])
        self._b_phase = math.radians(float(b["phase_step_deg"]))
        self._b_fade = float(b["fade_s"])
        self.gait = GaitEngine(cfg)
        self.base: Pose = {n: 0.0 for n in self.names}
        self._tracks: dict[str, _Track] = {}
        self._breath_env = 1.0
        self._breath_target = 1.0
        self._frozen_at: float | None = None
        self._frozen_out: Pose = {}
        self._paused_total = 0.0
        self._last_anim_t: float | None = None
        self.last_output: Pose = dict(self.base)

    # ---- 時刻 -----------------------------------------------------------------
    def _anim_t(self, t: float) -> float:
        """freeze していた時間を差し引いたアニメーション時刻。"""
        return t - self._paused_total

    # ---- 操作 -----------------------------------------------------------------
    def play(self, kf: Keyframe, t: float) -> float:
        """キーフレームを開始する。始点は各関節の現在のベース角。

        どれかの関節が max_joint_speed_dps を超える場合は、全関節そろえて時間を延ばす。
        戻り値: 実際の所要時間 [s]
        """
        at = self._anim_t(t)
        starts = {name: self._base_value(name, at) for name in kf.pose}
        need = max((min_duration_s(starts[n], d, kf.easing, kf.overshoot_deg, self._peak_ratio, self._vmax)
                    for n, d in kf.pose.items()), default=0.0)
        dur = max(kf.duration_s, need)
        for name, deg in kf.pose.items():
            self._tracks[name] = _Track(starts[name], deg, at, dur, kf.easing, kf.overshoot_deg)
        return dur

    def set_pose_now(self, pose: Pose) -> None:
        """補間なしでベース姿勢を置き換える。"""
        for name, deg in pose.items():
            self._tracks.pop(name, None)
            self.base[name] = deg

    def busy(self, t: float) -> bool:
        """補間中の関節があれば True。"""
        at = self._anim_t(t)
        return any(at < tr.t0 + tr.duration for tr in self._tracks.values())

    def set_breathing(self, on: bool) -> None:
        """呼吸の ON/OFF（fade_s かけて振幅を変える）。"""
        self._breath_target = 1.0 if on else 0.0

    def freeze(self, t: float) -> None:
        """出力をその場で固定する（呼吸も止まる）。"""
        if self._frozen_at is None:
            self._frozen_at = t
            self._frozen_out = dict(self.last_output)

    def unfreeze(self, t: float) -> None:
        """固定を解除し、止まっていた時間ぶん全体を一時停止扱いにする。"""
        if self._frozen_at is not None:
            self._paused_total += t - self._frozen_at
            self._frozen_at = None

    @property
    def frozen(self) -> bool:
        return self._frozen_at is not None

    # ---- 計算 -----------------------------------------------------------------
    def _base_value(self, name: str, at: float) -> float:
        tr = self._tracks.get(name)
        if tr is None:
            return self.base[name]
        v = tr.value(at, self._peak_ratio)
        if at >= tr.t0 + tr.duration:
            self.base[name] = tr.end
            del self._tracks[name]
        return v

    def breath_offset(self, index: int, at: float) -> float:
        """関節 index の呼吸オフセット [deg]。"""
        return self._breath_env * self._b_amp * math.sin(2.0 * math.pi * at / self._b_period + index * self._b_phase)

    def update(self, t: float) -> Pose:
        """時刻 t の9軸指令角（ソフトウェアリミット内）。"""
        if self._frozen_at is not None:
            return dict(self._frozen_out)
        at = self._anim_t(t)
        dt = 0.0 if self._last_anim_t is None else max(at - self._last_anim_t, 0.0)
        self._last_anim_t = at
        step = dt / self._b_fade if self._b_fade > 0 else 1.0
        self._breath_env += max(-step, min(step, self._breath_target - self._breath_env))
        gait = self.gait.update(at)
        out: Pose = {}
        for i, name in enumerate(self.names):
            v = self._base_value(name, at) + gait.get(name, 0.0) + self.breath_offset(i, at)
            lo, hi = self._limits[name]
            out[name] = min(max(v, lo), hi)
        self.last_output = out
        return out

    def send(self, bus: ServoBus, pose: Pose) -> None:
        """指令角をサーボバスへ送る（SYNC WRITE）。"""
        bus.sync_set_goals({self._ids[n]: Goal(d, self._speed, self._accel) for n, d in pose.items()})
