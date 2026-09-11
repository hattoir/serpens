"""歩容エンジン（serpenoid / gait equation）。

  α(n, t) = A · sin(Ω·n + ω·t) + γ(n)

n は胴体ヨー関節の番号（0 = J1 = 尾側 … N = J6 = 頭側）。
Ω > 0, ω > 0 のとき位相一定の点は n が減る向き（頭 → 尾）へ動き、ヘビは前進する。

旋回入力は オフセット γ だけ。関節への配り方（gait.turn_profile）は2種類:
  uniform       γ(n) = γ0              … 胴体全体が均一な円弧
  head_weighted γ(n) = γ0 · n / N      … 頭で舵を切って胴がついてくる
γ0 はステップで変えず、変化の速さを制限する: 0 → turn_full_scale_deg の変化にちょうど
gait.turn_ramp_periods 周期かかる速さ（小さな変化はそのぶん短く済む）。毎周期目標を変えても滑らかに追う。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

BODY_AXIS = "yaw"
NECK_JOINT = "J7"
TURN_PROFILES = ("uniform", "head_weighted")


@dataclass(frozen=True)
class GaitParams:
    """歩容パラメータ。"""

    amplitude_deg: float        # A
    spatial_freq_deg: float     # Ω [deg/関節]
    temporal_freq_hz: float     # ω/2π（負なら後退）
    turn_bias_deg: float = 0.0  # γ0（+ で左）

    @staticmethod
    def from_cfg(d: dict[str, Any]) -> "GaitParams":
        """config の presets 要素から作る。"""
        return GaitParams(
            amplitude_deg=float(d["amplitude_deg"]),
            spatial_freq_deg=float(d["spatial_freq_deg"]),
            temporal_freq_hz=float(d["temporal_freq_hz"]),
            turn_bias_deg=float(d.get("turn_bias_deg", 0.0)),
        )


def body_joint_names(cfg: dict[str, Any]) -> list[str]:
    """胴体の水平ヨー関節（首より尾側、J1〜J6）の名前。"""
    names: list[str] = []
    for j in cfg["joints"]:
        if j["name"] == NECK_JOINT:
            break
        if j["axis"] == BODY_AXIS:
            names.append(j["name"])
    return names


def turn_weights(n_joints: int, profile: str) -> list[float]:
    """γ0 を各関節に配る重み。"""
    if profile == "uniform":
        return [1.0] * n_joints
    if profile == "head_weighted":
        top = max(n_joints - 1, 1)
        return [n / top for n in range(n_joints)]
    raise ValueError(f"未知の turn_profile: {profile}（{TURN_PROFILES}）")


def angles_at_phase(p: GaitParams, phase_rad: float, joint_names: list[str],
                    gamma0_deg: float, profile: str, gain: float = 1.0) -> dict[str, float]:
    """時間位相 φ = ω·t [rad] における胴体関節角 [deg]。gain は振幅・γ の倍率。"""
    big_omega = math.radians(p.spatial_freq_deg)
    w = turn_weights(len(joint_names), profile)
    return {name: gain * (p.amplitude_deg * math.sin(big_omega * n + phase_rad) + gamma0_deg * w[n])
            for n, name in enumerate(joint_names)}


def gait_angles(p: GaitParams, t: float, joint_names: list[str], profile: str = "uniform") -> dict[str, float]:
    """時刻 t [s] における胴体関節角 [deg]（周波数・γ 一定のとき）。"""
    return angles_at_phase(p, 2.0 * math.pi * p.temporal_freq_hz * t, joint_names, p.turn_bias_deg, profile)


def gait_period_s(p: GaitParams) -> float:
    """1周期の時間 [s]。"""
    return 1.0 / abs(p.temporal_freq_hz) if p.temporal_freq_hz else math.inf


def _smoothstep(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


class GaitEngine:
    """歩容の再生器。開始・停止時の振幅ブレンドと、γ のランプを行う。"""

    def __init__(self, cfg: dict[str, Any]) -> None:
        g = cfg["gait"]
        self.presets: dict[str, GaitParams] = {k: GaitParams.from_cfg(v) for k, v in g["presets"].items()}
        self.joint_names = body_joint_names(cfg)
        self.profile = str(g["turn_profile"])
        self._blend_s = float(g["blend_s"])
        self._ramp_periods = float(g["turn_ramp_periods"])
        self._full_scale = float(g["turn_full_scale_deg"])
        self.params: GaitParams | None = None
        self._phase = 0.0            # 時間位相 [rad] の積分（周波数を変えても位相が飛ばない）
        self._gain = 0.0
        self._target_gain = 0.0
        self._last_t: float | None = None
        self.gamma0 = 0.0            # 現在の γ0（目標 _g_to へ速さ制限つきで近づく）
        self._g_to = 0.0

    @property
    def active(self) -> bool:
        """振幅が 0 でなければ動作中。"""
        return self._gain > 0.0 or self._target_gain > 0.0

    def start(self, preset_or_params: str | GaitParams, gamma0_deg: float | None = None) -> None:
        """歩容を開始（または切替）。γ0 の目標は gamma0_deg（省略時はプリセットの turn_bias_deg）。"""
        p = self.presets[preset_or_params] if isinstance(preset_or_params, str) else preset_or_params
        self.params = p
        self._target_gain = 1.0
        self.set_turn(p.turn_bias_deg if gamma0_deg is None else gamma0_deg)

    def set_turn(self, gamma0_deg: float) -> None:
        """旋回オフセット γ0 の目標を変える（速さ制限つきで移る）。"""
        self._g_to = gamma0_deg

    def _turn_rate_dps(self) -> float:
        period = gait_period_s(self.params) if self.params else math.inf
        if not math.isfinite(period) or self._ramp_periods <= 0:
            return math.inf
        return self._full_scale / (self._ramp_periods * period)

    def stop(self) -> None:
        """振幅を blend_s かけて 0 にする。"""
        self._target_gain = 0.0

    def update(self, t: float) -> dict[str, float]:
        """時刻 t の胴体関節角を返す。停止中は空 dict。"""
        dt = 0.0 if self._last_t is None else max(t - self._last_t, 0.0)
        self._last_t = t
        step = dt / self._blend_s if self._blend_s > 0 else 1.0
        self._gain += max(-step, min(step, self._target_gain - self._gain))
        rate = self._turn_rate_dps()
        if math.isinf(rate):
            self.gamma0 = self._g_to          # 歩容が無いときは制限しない（inf × 0 = NaN を避ける）
        else:
            step_g = rate * dt
            self.gamma0 += max(-step_g, min(step_g, self._g_to - self.gamma0))
        if self.params is None or self._gain <= 0.0:
            return {}
        self._phase += 2.0 * math.pi * self.params.temporal_freq_hz * dt
        return angles_at_phase(self.params, self._phase, self.joint_names, self.gamma0,
                               self.profile, _smoothstep(self._gain))
