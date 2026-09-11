"""歩容エンジン（serpenoid / gait equation）。

  α(n, t) = A_n · sin(Ω·n + ω·t) + γ
  A_n     = A · (1 + g · (n − n_c) / (N − 1))

n は胴体ヨー関節の番号（0 = J1 = 尾側 … N−1 = J6 = 頭側）、n_c は中央。
Ω > 0, ω > 0 のとき位相一定の点は n が減る向き（頭 → 尾）へ動き、ヘビは前進する。
旋回は γ（オフセット）と g（胴体方向の振幅勾配）の2つで試せるようにしてある。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any

BODY_AXIS = "yaw"
NECK_JOINT = "J7"


@dataclass(frozen=True)
class GaitParams:
    """歩容パラメータ。"""

    amplitude_deg: float        # A
    spatial_freq_deg: float     # Ω [deg/関節]
    temporal_freq_hz: float     # ω/2π（負なら後退）
    amp_gradient: float = 0.0   # g
    turn_bias_deg: float = 0.0  # γ

    @staticmethod
    def from_cfg(d: dict[str, Any]) -> "GaitParams":
        """config の presets 要素から作る。"""
        return GaitParams(
            amplitude_deg=float(d["amplitude_deg"]),
            spatial_freq_deg=float(d["spatial_freq_deg"]),
            temporal_freq_hz=float(d["temporal_freq_hz"]),
            amp_gradient=float(d.get("amp_gradient", 0.0)),
            turn_bias_deg=float(d.get("turn_bias_deg", 0.0)),
        )

    def scaled(self, gain: float) -> "GaitParams":
        """振幅と旋回オフセットを gain 倍したもの（開始・停止のブレンド用）。"""
        return replace(self, amplitude_deg=self.amplitude_deg * gain,
                       turn_bias_deg=self.turn_bias_deg * gain)


def body_joint_names(cfg: dict[str, Any]) -> list[str]:
    """胴体の水平ヨー関節（首より尾側、J1〜J6）の名前。"""
    names: list[str] = []
    for j in cfg["joints"]:
        if j["name"] == NECK_JOINT:
            break
        if j["axis"] == BODY_AXIS:
            names.append(j["name"])
    return names


def angles_at_phase(p: GaitParams, phase_rad: float, joint_names: list[str]) -> dict[str, float]:
    """時間位相 φ = ω·t [rad] における胴体関節角 [deg]。"""
    n_joints = len(joint_names)
    center = (n_joints - 1) / 2.0
    span = max(n_joints - 1, 1)
    big_omega = math.radians(p.spatial_freq_deg)
    out: dict[str, float] = {}
    for n, name in enumerate(joint_names):
        amp = p.amplitude_deg * (1.0 + p.amp_gradient * (n - center) / span)
        out[name] = amp * math.sin(big_omega * n + phase_rad) + p.turn_bias_deg
    return out


def gait_angles(p: GaitParams, t: float, joint_names: list[str]) -> dict[str, float]:
    """時刻 t [s] における胴体関節角 [deg]（周波数一定のとき）。"""
    return angles_at_phase(p, 2.0 * math.pi * p.temporal_freq_hz * t, joint_names)


def gait_period_s(p: GaitParams) -> float:
    """1周期の時間 [s]。"""
    return 1.0 / abs(p.temporal_freq_hz) if p.temporal_freq_hz else math.inf


class GaitEngine:
    """歩容の再生器。プリセット切替と、開始・停止時の振幅ブレンドを行う。"""

    def __init__(self, cfg: dict[str, Any]) -> None:
        g = cfg["gait"]
        self.presets: dict[str, GaitParams] = {k: GaitParams.from_cfg(v) for k, v in g["presets"].items()}
        self.joint_names = body_joint_names(cfg)
        self._blend_s = float(g["blend_s"])
        self.params: GaitParams | None = None
        self._phase = 0.0            # 時間位相 [rad] の積分（周波数を変えても位相が飛ばない）
        self._gain = 0.0
        self._target_gain = 0.0
        self._last_t: float | None = None

    @property
    def active(self) -> bool:
        """振幅が 0 でなければ動作中。"""
        return self._gain > 0.0 or self._target_gain > 0.0

    def start(self, preset_or_params: str | GaitParams) -> None:
        """歩容を開始（または別の歩容へ切替）。"""
        p = self.presets[preset_or_params] if isinstance(preset_or_params, str) else preset_or_params
        self.params = p
        self._target_gain = 1.0

    def stop(self) -> None:
        """振幅を blend_s かけて 0 にする。"""
        self._target_gain = 0.0

    def update(self, t: float) -> dict[str, float]:
        """時刻 t の胴体関節角を返す。停止中は空 dict。"""
        dt = 0.0 if self._last_t is None else max(t - self._last_t, 0.0)
        self._last_t = t
        step = dt / self._blend_s if self._blend_s > 0 else 1.0
        if self._gain < self._target_gain:
            self._gain = min(self._gain + step, self._target_gain)
        elif self._gain > self._target_gain:
            self._gain = max(self._gain - step, self._target_gain)
        if self.params is None or self._gain <= 0.0:
            return {}
        self._phase += 2.0 * math.pi * self.params.temporal_freq_hz * dt
        # 振幅ブレンドは smoothstep でなめらかに
        g = self._gain * self._gain * (3.0 - 2.0 * self._gain)
        return angles_at_phase(self.params.scaled(g), self._phase, self.joint_names)
