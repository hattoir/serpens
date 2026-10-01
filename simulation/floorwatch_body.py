"""Floor Watch の身体構成（5 / 6 モーター案）と腹面を MuJoCo で比べる。**source は MUJOCO_SIM。**

目的は「実物で何を測れば決められるか」を絞ること。**構成を決めるための数値ではない。**
摩擦・質量・サーボ応答はすべて未実測なので、読むのは

  - 腹面の異方性（横 / 前後の摩擦比）がどこまであれば前進が成立するか（しきい値の位置）
  - トルク上限（子どもの安全側の制限）が推進をどれだけ縛るか
  - 首 J1 の保持トルクが頭の質量・重心でどう変わるか

といった**感度**だけ。絶対値（mm/s）を実機の性能として扱わない。

    .\\.venv\\Scripts\\python.exe tools\\fw_body_study.py
"""
from __future__ import annotations

import copy
import math
from dataclasses import asdict, dataclass
from typing import Any

from serpens.config import load_config
from serpens.motion.gait import GaitParams, body_joint_names
from simulation.mujoco.belly import BellyProfile
from simulation.mujoco.model import build_mjcf
from simulation.mujoco.runner import SOURCE, run_gait

G = 9.81

# 比べる構成（config の overlay）。CAD にあるのは FW5 だけ。6 モーター案 2 つは CONCEPT
CONFIGS: dict[str, str] = {
    "FW5": "config/robot_fw5.yaml",                 # FW03 の現行 CAD（J1 pitch ＋ yaw ×4）
    "FW6_YAW5": "config/robot_fw6_yaw5.yaml",       # 案 A: yaw ×5 ＋ pitch
    "FW6_HEADYAW": "config/robot_fw6_headyaw.yaml",  # 案 B: yaw ×4 ＋ pitch ＋ 頭 yaw
}

# 巡回に要る速さの目安（**ASSUMPTION**）。4m 四方の部屋の外周 16m を 5 分強で回れる程度
MIN_PATROL_SPEED_MM_S = 50.0


@dataclass(frozen=True)
class StudyRow:
    source: str
    config: str
    belly: str
    slide_along: float
    slide_across: float
    anisotropy: float
    torque_limit_nm: float
    amplitude_deg: float
    waves: float
    temporal_freq_hz: float
    gamma_deg: float
    body_axes: int
    length_mm: float
    mass_kg: float
    speed_mm_s: float
    body_lengths_per_s: float
    lateral_slip_ratio: float     # |横ずれ| / |前進|（小さいほど素直に進む）
    turn_deg: float
    turn_radius_mm: float | None  # 旋回 run だけ。弧の近似
    energy_j_per_m: float | None  # 機械仕事 Σ|τω|dt / 前進距離（**電気的な消費ではない**）
    peak_torque_nm: float
    torque_saturation: float
    tracking_error_rms_deg: float
    fell_over: bool

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def fw_config(name: str, torque_limit_nm: float | None = None) -> dict[str, Any]:
    cfg = load_config(overlay=CONFIGS[name])
    if torque_limit_nm is not None:
        cfg = copy.deepcopy(cfg)
        cfg["safety_limits"]["torque"]["software_torque_limit_nm"] = float(torque_limit_nm)
    return cfg


# 構成ごとに歩容を選び直してから比べる（1 つの歩容で比べると、構成の差より歩容の相性の差が出る。2026-09-29 に確認）
GAIT_GRID_AMPLITUDE_DEG = [20.0, 30.0, 40.0]
GAIT_GRID_WAVES = [0.75, 1.0, 1.25, 1.5]      # 胴体に乗る波の数


def gait_for(cfg: dict[str, Any], amplitude_deg: float = 30.0, waves: float = 1.0,
             freq_hz: float = 0.5) -> GaitParams:
    """胴体に `waves` 波（Ω = 360° × waves / 胴体ヨーの本数）。"""
    n = len(body_joint_names(cfg))
    return GaitParams(amplitude_deg, 360.0 * waves / n, freq_hz)


def run_case(name: str, belly: BellyProfile, torque_limit_nm: float | None = None,
             amplitude_deg: float = 30.0, waves: float = 1.0, freq_hz: float = 0.5,
             gamma_deg: float = 0.0, seconds: float = 8.0, seed: int = 0) -> StudyRow:
    cfg = fw_config(name, torque_limit_nm)
    spec = build_mjcf(cfg, belly)                   # 登録簿 PROFILES は書き換えない（他のテストを汚さない）
    r = run_gait(cfg, belly.name, gait_for(cfg, amplitude_deg, waves, freq_hz), gamma_deg=gamma_deg,
                 seconds=seconds, seed=seed, spec=spec)
    length_m = float(cfg["body"]["head_tip_x_mm"]) / 1000.0
    fwd = r.forward_mm
    dist_m = math.hypot(r.forward_mm, r.lateral_mm) / 1000.0
    radius = None
    if gamma_deg and abs(r.turn_deg) > 5.0:
        # 弧長 ≈ 移動距離、半径 = 弧長 / 回転角
        radius = dist_m * 1000.0 / math.radians(abs(r.turn_deg))
    return StudyRow(
        source=SOURCE, config=name, belly=belly.name,
        slide_along=belly.slide_along, slide_across=belly.slide_across, anisotropy=belly.anisotropy,
        torque_limit_nm=float(cfg["safety_limits"]["torque"]["software_torque_limit_nm"]),
        amplitude_deg=amplitude_deg, waves=waves, temporal_freq_hz=freq_hz, gamma_deg=gamma_deg,
        body_axes=len(body_joint_names(cfg)), length_mm=length_m * 1000.0, mass_kg=spec.total_mass_kg,
        speed_mm_s=r.speed_mm_s, body_lengths_per_s=r.speed_mm_s / 1000.0 / length_m,
        lateral_slip_ratio=abs(r.lateral_mm) / max(abs(fwd), 1.0),
        turn_deg=r.turn_deg, turn_radius_mm=radius,
        energy_j_per_m=(r.energy_j / dist_m) if dist_m > 0.02 else None,
        peak_torque_nm=r.peak_torque_nm, torque_saturation=r.torque_saturation,
        tracking_error_rms_deg=r.tracking_error_rms_deg, fell_over=r.fell_over,
    )


def best_gait(name: str, belly: BellyProfile, seconds: float = 8.0) -> StudyRow:
    """歩容の格子（振幅 × 波数）で最も速く進んだ run（トルク上限に張り付く割合 5% 未満に限る）。"""
    rows = [run_case(name, belly, amplitude_deg=a, waves=w, seconds=seconds)
            for a in GAIT_GRID_AMPLITUDE_DEG for w in GAIT_GRID_WAVES]
    ok = [r for r in rows if r.torque_saturation < 0.05] or rows
    return max(ok, key=lambda r: r.speed_mm_s)


def neck_static_torque_nm(mass_g: float, cog_mm: float, pitch_deg: float) -> float:
    """首 J1 が頭を持ち上げて保持するトルク（静的、ANALYTIC）。τ = m g d cosθ。"""
    return mass_g / 1000.0 * G * cog_mm / 1000.0 * math.cos(math.radians(pitch_deg))
