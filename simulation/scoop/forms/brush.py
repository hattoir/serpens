"""C: 回転ブラシ / ローラー。口（ゲート面）のすぐ前、床すれすれに横軸（y）のブラシ。**床側が奥（+x）へ動く向き**に回す。
径 8 / 12 mm（フラップの先端まで）、フラップ 6 枚、フラップ剛性 2 水準（柔らかい / 硬い）、回転 60 / 120 / 240 rpm。
フラップは薄い板（ハブに蝶番 + ばね）。先端は床にわずかに食い込む向き（config の floor_interference）で、フラップと床の接触も入れる。
ブラシは口の面の前（x < 0）にあり、空間（30 × 30 × 15）を削らない。ゲートはブラシの奥。
駆動数 = 2（ブラシモーター 1 + ゲート 1）。ASSUMED: ハブ半径・フラップの厚み・質量・剛性・フラップと床の μ 0.5・モーターのトルク上限。
注意: ハブの下を通れる高さは（中心の高さ − ハブ半径）= R − 1.6 mm ほど。それより高い物（ビーズ 8 mm、立方体 10 mm）はハブの下を通れず、ブラシが引き込めなければ前へ押される。
"""
from __future__ import annotations

import math

from simulation.scoop.forms.common import CLS, Form, Parts, backstop_parts, hood, merge
from simulation.scoop.model import MM, _f


class Brush(Form):
    name = "brush"
    drives = 2
    drive_note = "ブラシモーター 1 + ゲート 1"

    def __init__(self, cfg, params, obj_name, floor_name):
        super().__init__(cfg, params, obj_name, floor_name)
        b = self.fc["brush"]
        self.R = float(params["D"]) * MM / 2
        self.rpm = float(params["rpm"])
        self.k = float(b["stiffness_nm_rad"][params["stiff"]])
        self.x_b = -(self.R + b["x_offset_mm"] * MM)
        self.z_b = self.R - b["floor_interference_mm"] * MM
        self.front_extent = 2 * self.R + b["x_offset_mm"] * MM
        self.peak_speed_mm_s = self.R / MM * self.rpm * 2 * math.pi / 60.0

    def build(self) -> Parts:
        b = self.fc["brush"]
        funnel = bool(self.p.get("funnel", False))
        fe = self.front_extent
        parts = merge(hood(self, funnel=funnel, x_front=fe), backstop_parts(self))
        self.front_extent = max(fe, self.fc["funnel"]["length_mm"] * MM if funnel else 0.0)
        R, rh = self.R, b["hub_radius_mm"] * MM
        lf = R - rh
        ft = b["flap_thickness_mm"] * MM
        w = self.half_w
        mf = b["flap_mass_g"] / 1000.0
        tq = float(b["torque_limit_nm"])
        n = int(b["flaps"])
        hub_m = math.pi * rh * rh * 2 * w * 1000.0
        flaps = []
        for i in range(n):
            psi = 360.0 * i / n
            a = math.radians(psi)
            flaps.append(
                f'<body name="fl{i}" pos="{_f(rh * math.sin(a))} 0 {_f(rh * math.cos(a))}" euler="0 {psi:.6g} 0">'
                f'<joint name="fj{i}" type="hinge" axis="0 1 0" range="-60 60" stiffness="{self.k:.6g}" damping="1e-7" armature="1e-10"/>'
                f'<inertial pos="0 0 {_f(lf / 2)}" mass="{mf:.6g}" diaginertia="{mf * lf * lf / 3:.6g} {mf * lf * lf / 3:.6g} 1e-12"/>'
                f'<geom name="fg{i}" type="box" size="{_f(ft / 2)} {_f(w)} {_f(lf / 2)}" pos="0 0 {_f(lf / 2)}" {CLS}/></body>')
            parts.obj_pairs.append((f"fg{i}", self.mu_wall))
            mu = b["flap_floor_mu"]
            parts.raw_pairs.append(f'<pair geom1="fg{i}" geom2="floor" condim="3" friction="{mu} {mu} 1e-4 1e-4 1e-4" solref="SOLREF" solimp="SOLIMP"/>')
        ixz = hub_m * (3 * rh * rh + (2 * w) ** 2) / 12
        parts.children.append(
            f'<body name="brush" pos="{_f(self.x_b)} 0 {_f(self.z_b)}"><joint name="brush_rot" type="hinge" axis="0 1 0" damping="1e-7"/>'
            f'<inertial pos="0 0 0" mass="{hub_m:.6g}" diaginertia="{ixz:.6g} {hub_m * rh * rh / 2:.6g} {ixz:.6g}"/>'
            f'<geom name="hub" type="cylinder" size="{_f(rh)} {_f(w)}" euler="90 0 0" {CLS}/>{"".join(flaps)}</body>')
        parts.obj_pairs.append(("hub", self.mu_wall))
        parts.actuators.append(f'<velocity name="brush_act" joint="brush_rot" kv="1e-4" forcerange="{-tq:.6g} {tq:.6g}"/>')
        return parts

    def bind(self, model, mujoco) -> None:
        super().bind(model, mujoco)
        self.act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "brush_act")

    def always(self, model, data, t) -> None:
        data.ctrl[self.act] = -self.rpm * 2 * math.pi / 60.0          # 負: 床側が +x（奥）へ動く
