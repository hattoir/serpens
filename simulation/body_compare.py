"""胴体ヨー 6 / 8 / 10 軸の比較（KINEMATIC_SIM）。波数の掃引と、速度を揃えた条件（SPEED_MATCHED）。

  同じ Ω を使い回さない: 可視波数 w を狙って Ω = w × 360° / N を各構成で作る。
  RAW           … 同じ基準パラメータ（A=30°, f=0.5Hz）
  SPEED_MATCHED … 見た目の移動速度（mm/s）を基準構成（yaw6 の 1 波）へ寄せる。f を変えるだけで、
                  サーボの速度上限（A·2πf ≤ max_speed_dps）と link.limits.temporal_freq_hz を超えない
                  （超えるときは clipped=True として正直に残す）

推定負荷の代用値: 関節の最大角速度 A·2πf [deg/s] と最大角加速度 A·(2πf)² [deg/s²]（サーボ応答は含まない）。
**身体構成を決める数値ではない。** 人の評価（VIDEO_HUMAN_EVALUATION）と並べて見るための表。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from serpens.config import load_config
from serpens.motion.gait import GaitEngine, GaitParams, body_joint_names, gait_period_s
from serpens.sim.measure import per_cycle_advance
from serpens.sim.world import BodyPose, World
from simulation.quality import visible_waves

SOURCE = "KINEMATIC_SIM"
CONFIGS = {
    "yaw6": "config/robot_yaw6.yaml",
    "yaw8": "config/robot_yaw8.yaml",
    "yaw10": "config/robot_yaw10.yaml",
    "yaw8_samelen": "config/robot_yaw8_samelen.yaml",
    "yaw10_samelen": "config/robot_yaw10_samelen.yaml",
}
WAVE_TARGETS = (1.0, 1.3, 1.5, 1.8, 2.0, 2.2)
REFERENCE = ("yaw6", 1.0)          # 速度を揃える基準: 6 軸・1 波


@dataclass
class Row:
    source: str
    config: str
    body_axes: int
    link_mm: float
    length_mm: float
    servo_count: int
    mass_g: float
    condition: str                 # RAW / SPEED_MATCHED
    target_waves: float
    spatial_freq_deg: float
    amplitude_deg: float
    temporal_freq_hz: float
    visible_waves: float
    forward_mm_per_cycle: float
    forward_mm_s: float
    turn_deg_per_cycle: float      # γ0 = gait.turn_full_scale_deg の旋回
    peak_joint_speed_dps: float
    peak_joint_accel_dps2: float
    clipped: bool = False
    notes: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        d = dict(self.__dict__)
        d["notes"] = " ".join(self.notes)
        return {k: (round(v, 3) if isinstance(v, float) else v) for k, v in d.items()}


def load(name: str) -> dict[str, Any]:
    return load_config(overlay=CONFIGS[name])


def geometry(cfg: dict[str, Any]) -> tuple[int, float, float]:
    body = body_joint_names(cfg)
    xs = [float(j["x_mm"]) for j in cfg["joints"] if j["name"] in body]
    link = (xs[-1] - xs[0]) / (len(xs) - 1)
    return len(body), link, float(cfg["body"]["length_mm"])


def measured_waves(cfg: dict[str, Any], p: GaitParams) -> float:
    w, eng = World(cfg, BodyPose(600.0, 600.0, 0.0)), GaitEngine(cfg)
    eng.start(p)
    dt, T, t, out = float(cfg["sim"]["dt_s"]), gait_period_s(p), 0.0, []
    for _ in range(int(round(2 * T / dt))):
        t += dt
        w.step(eng.update(t), dt)
        if t > T:
            out.append(visible_waves(w.world_points()[:, :2], float(cfg["quality"]["wave_min_turn_deg"])))
    return float(np.mean(out))


def evaluate(name: str, cfg: dict[str, Any], target_waves: float, amplitude: float, freq: float,
             condition: str) -> Row:
    n, link, length = geometry(cfg)
    omega = target_waves * 360.0 / n
    p = GaitParams(amplitude, omega, freq, 0.0)
    adv = per_cycle_advance(cfg, p, 2, 2)
    turned = per_cycle_advance(cfg, GaitParams(amplitude, omega, freq, float(cfg["gait"]["turn_full_scale_deg"])), 2, 2)
    w = 2.0 * math.pi * freq
    return Row(SOURCE, name, n, link, length, int(cfg["mass_budget_g"]["servo_count"]), float(cfg["body"]["mass_g"]),
               condition, target_waves, omega, amplitude, freq, measured_waves(cfg, p),
               adv.per_cycle_mm, adv.per_cycle_mm * freq, turned.turn_deg,
               amplitude * w, amplitude * w * w)


def speed_matched_freq(cfg: dict[str, Any], row_raw: Row, v_ref_mm_s: float) -> tuple[float, bool]:
    """基準の速さに合わせる f。サーボ速度上限と link の周波数上限で切る。"""
    body = body_joint_names(cfg)
    v_max = min(float(j["max_speed_dps"]) for j in cfg["joints"] if j["name"] in body)
    f_servo = v_max / (2.0 * math.pi * row_raw.amplitude_deg)
    f_link = float(cfg["link"]["limits"]["temporal_freq_hz"])
    f_want = v_ref_mm_s / max(row_raw.forward_mm_per_cycle, 1e-9)
    f = min(f_want, f_servo, f_link)
    return f, f < f_want - 1e-9


def sweep(names: list[str] | None = None, amplitude: float = 30.0, freq: float = 0.5,
          targets: tuple[float, ...] = WAVE_TARGETS) -> list[Row]:
    names = list(names or CONFIGS)
    cfgs = {n: load(n) for n in names}
    ref_cfg = load(REFERENCE[0])
    v_ref = evaluate(REFERENCE[0], ref_cfg, REFERENCE[1], amplitude, freq, "RAW").forward_mm_s
    rows: list[Row] = []
    for name in names:
        for tw in targets:
            raw = evaluate(name, cfgs[name], tw, amplitude, freq, "RAW")
            rows.append(raw)
            f, clipped = speed_matched_freq(cfgs[name], raw, v_ref)
            m = evaluate(name, cfgs[name], tw, amplitude, f, "SPEED_MATCHED")
            m.clipped = clipped
            if clipped:
                m.notes.append(f"上限で {v_ref:.0f}mm/s に届かない")
            rows.append(m)
    return rows
