"""MuJoCo 上で歩容を走らせて測る（**すべて MUJOCO_SIM。実測ではない**）。

与えるのは駆動リンクと同じ歩容パラメータ（A, Ω, ω, γ0）。角度は
`serpens/motion/gait.py` の式で作るので、**簡易シミュレータと同じ指令**を 3D で走らせられる。

測るもの:
  前進 / 横ずれ / 旋回 / 関節速度・加速度 / ピークトルク / 消費エネルギー /
  頭の高さのばらつき（安定性）/ 胴体の曲率

結果には必ず `source`（MUJOCO_SIM）・モデルの指紋・seed を入れる
（簡易シミュレータ KINEMATIC_SIM の結果と混ぜないため）。
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Any

import numpy as np

from serpens.motion.gait import GaitParams, angles_at_phase, body_joint_names
from simulation.mujoco.model import ModelSpec, build_mjcf

SOURCE = "MUJOCO_SIM"
SETTLE_S = 1.0              # 床に落ち着くまで（この間は指令 0）


@dataclass(frozen=True)
class RunResult:
    """1回の run の結果。**source を落とさないこと。**"""

    source: str
    belly: str
    model_digest: str
    model_version: str
    seed: int
    seconds: float
    amplitude_deg: float
    spatial_freq_deg: float
    temporal_freq_hz: float
    gamma_deg: float
    forward_mm: float            # 初期の向きに沿った移動
    lateral_mm: float            # 横へのずれ（小さいほど素直）
    turn_deg: float              # 向きの変化
    speed_mm_s: float
    joint_speed_rms_dps: float
    joint_accel_rms_dps2: float
    peak_torque_nm: float
    energy_j: float              # Σ|τ·ω|dt（**模擬。電気的な消費電力ではない**）
    tracking_error_rms_deg: float  # 指令角と実角のずれ（サーボが追従できているか）
    torque_saturation: float       # トルク上限に張り付いていた割合（0〜1）
    head_height_std_mm: float    # 頭の高さのばらつき（跳ねていないか）
    body_curvature_1_m: float    # 胴体の平均曲率
    fell_over: bool              # 転倒（胴体が浮き上がった／ひっくり返った）
    notes: list[str] = field(default_factory=list)

    def as_row(self) -> dict[str, Any]:
        return asdict(self)


def _target_angles(cfg: dict[str, Any], p: GaitParams, gamma: float, t: float,
                   names: list[str]) -> dict[str, float]:
    """歩容の目標角（駆動リンクが機体へ送るのと同じ式）。"""
    phase = 2.0 * math.pi * p.temporal_freq_hz * t
    return angles_at_phase(p, phase, names, gamma, str(cfg["gait"]["turn_profile"]))


def run_gait(cfg: dict[str, Any], belly: str, params: GaitParams, gamma_deg: float = 0.0,
             seconds: float = 6.0, seed: int = 0, spec: ModelSpec | None = None,
             kp_scale: float = 1.0) -> RunResult:
    """歩容を `seconds` 秒走らせて測る。"""
    import mujoco

    spec = spec or build_mjcf(cfg, belly, kp_scale=kp_scale)
    model = mujoco.MjModel.from_xml_string(spec.xml)
    data = mujoco.MjData(model)
    rng = np.random.default_rng(seed)
    data.qpos[:3] += rng.normal(0.0, 1e-4, 3)        # 同じ seed で同じ run になる微小ばらつき

    body_names = body_joint_names(cfg)
    all_names = spec.joint_names
    ctrl_dt = 1.0 / float(cfg["link"]["control_hz"])
    home = {n: float(cfg["poses"]["home"].get(n, 0.0)) for n in all_names}
    act_id = {n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{n}_pos") for n in all_names}
    jnt_adr = {n: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
               for n in all_names}
    dof_adr = {n: model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
               for n in all_names}
    head_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"seg{len(spec.link_lengths_m) - 1}")

    steps_per_ctrl = max(int(round(ctrl_dt / model.opt.timestep)), 1)
    n_settle = int(round(SETTLE_S / ctrl_dt))
    n_run = int(round(seconds / ctrl_dt))

    qs, vs, torques, heads, errs, energy = [], [], [], [], [], 0.0
    start_xy = None
    start_yaw = 0.0
    prev_yaw, turned = 0.0, 0.0          # 向きの変化は毎ステップ積算する（±180° を越える旋回で折り返さない）
    for k in range(n_settle + n_run):
        t = max(k - n_settle, 0) * ctrl_dt
        target = home if k < n_settle else {**home, **_target_angles(cfg, params, gamma_deg, t, body_names)}
        for n in all_names:
            data.ctrl[act_id[n]] = math.radians(target[n])
        for _ in range(steps_per_ctrl):
            mujoco.mj_step(model, data)
            energy += float(np.abs(data.actuator_force * data.qvel[[dof_adr[n] for n in all_names]]).sum()
                            * model.opt.timestep)
        if k == n_settle:
            start_xy = np.array(data.xpos[1][:2])     # seg0 の位置
            start_yaw = _yaw(data.xmat[1])
            prev_yaw = start_yaw
        if k > n_settle:
            yaw = _yaw(data.xmat[1])
            turned += _wrap(yaw - prev_yaw)
            prev_yaw = yaw
        if k >= n_settle:
            errs.append([math.radians(target[n]) - float(data.qpos[jnt_adr[n]]) for n in all_names])
            qs.append([float(data.qpos[jnt_adr[n]]) for n in all_names])
            vs.append([float(data.qvel[dof_adr[n]]) for n in all_names])
            torques.append([abs(float(f)) for f in data.actuator_force])
            heads.append(float(data.xpos[head_body][2]))

    end_xy = np.array(data.xpos[1][:2])
    disp = (end_xy - start_xy) * 1000.0
    fwd_dir = np.array([math.cos(start_yaw), math.sin(start_yaw)])
    side_dir = np.array([-fwd_dir[1], fwd_dir[0]])
    q = np.degrees(np.array(qs))
    v = np.degrees(np.array(vs))
    acc = np.diff(v, axis=0) / ctrl_dt if len(v) > 1 else np.zeros_like(v)
    curvature = _curvature(q[:, :len(body_names)], spec.link_lengths_m[1:len(body_names) + 1])
    limit_nm = float(cfg["safety_limits"]["torque"]["software_torque_limit_nm"])
    trq = np.array(torques)
    saturation = float((trq >= limit_nm * 0.95).mean()) if trq.size else 0.0
    err_deg = np.degrees(np.array(errs))
    radius_m = float(cfg["body"]["diameter_mm"]) / 2000.0
    fell = bool(np.max(heads) > radius_m * 4.0)

    return RunResult(
        source=SOURCE, belly=spec.belly, model_digest=spec.digest, model_version=spec.version,
        seed=seed, seconds=seconds,
        amplitude_deg=params.amplitude_deg, spatial_freq_deg=params.spatial_freq_deg,
        temporal_freq_hz=params.temporal_freq_hz, gamma_deg=gamma_deg,
        forward_mm=float(disp @ fwd_dir), lateral_mm=float(disp @ side_dir),
        turn_deg=math.degrees(turned),
        speed_mm_s=float(disp @ fwd_dir) / seconds if seconds else 0.0,
        joint_speed_rms_dps=float(np.sqrt((v ** 2).mean())),
        joint_accel_rms_dps2=float(np.sqrt((acc ** 2).mean())) if acc.size else 0.0,
        peak_torque_nm=float(np.max(torques)) if torques else 0.0,
        tracking_error_rms_deg=float(np.sqrt((err_deg ** 2).mean())) if err_deg.size else 0.0,
        torque_saturation=saturation,
        energy_j=energy,
        head_height_std_mm=float(np.std(heads) * 1000.0),
        body_curvature_1_m=curvature,
        fell_over=fell,
        notes=["摩擦・質量・サーボ応答はすべて未実測（SIMULATED）"],
    )


def _yaw(xmat: np.ndarray) -> float:
    m = np.array(xmat).reshape(3, 3)
    return math.atan2(m[1, 0], m[0, 0])


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _curvature(body_deg: np.ndarray, link_m: list[float]) -> float:
    """関節角の平均から曲率 [1/m] を出す（θ/L の平均）。"""
    if body_deg.size == 0 or not link_m:
        return 0.0
    mean_abs = np.abs(np.radians(body_deg)).mean()
    return float(mean_abs / float(np.mean(link_m)))
