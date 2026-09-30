"""A: 縦軸サイドスイーパー。口（ゲート面）の左右の角に縦軸の腕を 2 本。腕は床上 0.5mm から高さ 10mm の板で、床の平面を掃く。
V 字に開いて前へ出し（前方から外側へ open_half_angle）、閉じながら物を口の中央・奥へ寄せる。
パラメータ: 腕長 L 15/20/25、掃引角 S 60/90/120°、時間 T 0.3/0.6 s、先端 柔らかい（先端 5mm がばね）/ 硬い。
駆動数 = 2（腕 1 = 左右をリンクで連動 + ゲート 1）。ASSUMED: 腕の質量・厚み・トルク上限 0.05 N·m・柔らかい先端のばね定数。
関節角 q: 左の腕（y > 0）は q = −β、右の腕（y < 0）は q = +β（β = 前方から外側への腕の角度。β = 0 で前へまっすぐ、β < 0 で内側）。
"""
from __future__ import annotations

import math

from simulation.scoop.forms.common import CLS, Form, Parts, backstop_parts, hood, merge
from simulation.scoop.model import MM, _f
from simulation.scoop.runner import _smoothstep


class Sweeper(Form):
    name = "sweeper"
    drives = 2
    drive_note = "腕 1（左右をリンクで連動）+ ゲート 1"

    def __init__(self, cfg, params, obj_name, floor_name):
        super().__init__(cfg, params, obj_name, floor_name)
        s = self.fc["sweeper"]
        self.L = float(params["L"]) * MM
        self.S = math.radians(float(params["S"]))
        self.T = float(params["T"])
        self.soft = bool(params.get("soft", False))
        self.beta0 = math.radians(s["open_half_angle_deg"])
        self.peak_speed_mm_s = self.L / MM * self.S * 1.5 / self.T          # smoothstep の最大角速度 = 1.5 × 平均
        self.front_extent = self.L * math.cos(self.beta0)

    @property
    def discrete(self) -> bool:
        return True

    @property
    def gate_delay_s(self) -> float:
        return float(self.fc["sweeper"]["gate_delay_s"])

    def fully_inside(self, rel, margin) -> bool:
        return False                      # ゲートは腕が終わってから（腕がゲートの面を通るため）

    def build(self) -> Parts:
        s = self.fc["sweeper"]
        funnel = bool(self.p.get("funnel", False))
        parts = merge(hood(self, funnel=funnel), backstop_parts(self))
        self.front_extent = max(self.front_extent, self.fc["funnel"]["length_mm"] * MM if funnel else 0.0)
        ha, ta, zb0 = s["arm_height_mm"] * MM, s["arm_thickness_mm"] * MM, s["arm_bottom_mm"] * MM
        za = zb0 + ha / 2
        m = s["arm_mass_g"] / 1000.0
        L = self.L
        tl = s["soft_tip"]["length_mm"] * MM if self.soft else 0.0
        Lr = L - tl
        inertia = m * L * L / 3.0
        tq = float(s["torque_limit_nm"])
        kp = tq / math.radians(2.0)
        kv = 2 * math.sqrt(kp * inertia)
        for tag, sgn in (("L", 1.0), ("R", -1.0)):
            tip = ""
            if self.soft:
                st = s["soft_tip"]
                tip = (f'<body name="arm{tag}_tip" pos="{_f(-Lr)} 0 0"><joint name="arm{tag}_tj" type="hinge" axis="0 0 1" range="-70 70" '
                       f'stiffness="{st["stiffness_nm_rad"]:.6g}" damping="{st["damping"]:.6g}" armature="1e-9"/>'
                       f'<inertial pos="{_f(-tl / 2)} 0 0" mass="{m * tl / L:.6g}" diaginertia="1e-10 1e-10 1e-10"/>'
                       f'<geom name="arm{tag}_tipg" type="box" size="{_f(tl / 2)} {_f(ta / 2)} {_f(ha / 2)}" pos="{_f(-tl / 2)} 0 0" {CLS}/></body>')
                parts.obj_pairs.append((f"arm{tag}_tipg", self.mu_wall))
            parts.children.append(
                f'<body name="arm{tag}" pos="0 {_f(sgn * self.half_w)} {_f(za)}">'
                f'<joint name="arm{tag}_j" type="hinge" axis="0 0 1" damping="1e-6" armature="{inertia:.6g}"/>'
                f'<inertial pos="{_f(-L / 2)} 0 0" mass="{m:.6g}" diaginertia="{inertia / 100:.6g} {inertia:.6g} {inertia:.6g}"/>'
                f'<geom name="arm{tag}_g" type="box" size="{_f(Lr / 2)} {_f(ta / 2)} {_f(ha / 2)}" pos="{_f(-Lr / 2)} 0 0" {CLS}/>{tip}</body>')
            parts.obj_pairs.append((f"arm{tag}_g", self.mu_wall))
            parts.actuators.append(f'<position name="arm{tag}_act" joint="arm{tag}_j" kp="{kp:.6g}" kv="{kv:.6g}" forcerange="{-tq:.6g} {tq:.6g}"/>')
        return parts

    def bind(self, model, mujoco) -> None:
        super().bind(model, mujoco)
        n = lambda kind, nm: mujoco.mj_name2id(model, kind, nm)
        self.aL, self.aR = n(mujoco.mjtObj.mjOBJ_ACTUATOR, "armL_act"), n(mujoco.mjtObj.mjOBJ_ACTUATOR, "armR_act")
        self.jL, self.jR = n(mujoco.mjtObj.mjOBJ_JOINT, "armL_j"), n(mujoco.mjtObj.mjOBJ_JOINT, "armR_j")

    def _set(self, data, beta: float) -> None:
        data.ctrl[self.aL] = -beta
        data.ctrl[self.aR] = beta

    def reset(self, model, data) -> None:
        self._moving = False
        data.qpos[model.jnt_qposadr[self.jL]] = -self.beta0
        data.qpos[model.jnt_qposadr[self.jR]] = self.beta0
        self._set(data, self.beta0)

    def always(self, model, data, t) -> None:
        if not getattr(self, "_moving", False):
            self._set(data, self.beta0)

    def mech_fire(self, rel, t) -> bool:
        reach = float(self.fc["sweeper"]["reach_trigger_mm"]) * MM
        self._moving = False
        return -reach <= rel[0] <= self.depth and abs(rel[1]) <= self.half_w + self.L * math.sin(self.beta0)

    def mech_step(self, model, data, t_since, rel) -> bool:
        self._moving = True
        beta = self.beta0 - self.S * _smoothstep(t_since / self.T)
        self._set(data, beta)
        return t_since >= self.T + 0.05
