"""B: ベルト式ランプ。ランプ先端に φ3 / 4 mm のローラー、その奥へ薄いベルトが動く。**ベルトはローラー列（接して並ぶ駆動ローラー）で近似**する
（MuJoCo は面の速度を持てないため。ローラーの回転で、物との接触点に面の速度が出る）。ランプ（12°）を上がった先の空間の床もローラー列で、物を奥の壁まで運ぶ。
先端の厚みはローラーの径で決まる（印刷の薄さの制約が消える）。
パラメータ: ベルトと物の μ（0.3 / 0.6 / 1.0）、ベルト速度 10 / 20 / 40 mm/s、ローラー径 3 / 4。
駆動数 = 2（ベルトモーター 1 + ゲート 1。全ローラーを 1 つのモーターとベルトで動かす想定）。ASSUMED: ローラー質量・トルク上限 0.01 N·m・ベルトの μ（本来はベルト素材で決まる）。
ベルトの面は、上側が奥（+x）へ動く向き。ローラーの前の面は上へ動き、物の縁を持ち上げて引き込む。
"""
from __future__ import annotations

import math

from simulation.scoop.forms.common import CLS, Form, Parts, backstop_parts, hood, merge
from simulation.scoop.model import MM, _f


class Belt(Form):
    name = "belt"
    drives = 2
    drive_note = "ベルトモーター 1（全ローラーを連動）+ ゲート 1"

    def __init__(self, cfg, params, obj_name, floor_name):
        super().__init__(cfg, params, obj_name, floor_name)
        b = self.fc["belt"]
        self.mu_belt = float(params["mu"])
        self.vb = float(params["vb"]) * MM
        self.d = float(params["d"]) * MM
        self.r = self.d / 2
        self.alpha = math.radians(b["ramp_deg"])
        self.n_r = max(2, round(b["ramp_along_mm"] * MM / self.d))
        self.n_f = int(self.D_over_d())
        self.z_c = self.r + self.clear + (self.n_r - 1) * self.d * math.sin(self.alpha)
        self.zb = self.z_c + self.r
        self.front_extent = (self.n_r - 1) * self.d * math.cos(self.alpha) + self.r
        self.peak_speed_mm_s = self.vb / MM

    def D_over_d(self) -> int:
        return int(self.depth // self.d)

    def build(self) -> Parts:
        b = self.fc["belt"]
        funnel = bool(self.p.get("funnel", False))
        fe = self.front_extent
        parts = merge(hood(self, funnel=funnel, zb=self.zb, x_front=fe), backstop_parts(self))
        self.front_extent = max(fe, self.fc["funnel"]["length_mm"] * MM if funnel else 0.0)
        r, d = self.r, self.d
        w = self.half_w
        tq = float(b["roller_torque_nm"])
        m = math.pi * r * r * 2 * w * 1000.0
        iy, ixz = 0.5 * m * r * r, m * (3 * r * r + (2 * w) ** 2) / 12
        centers = []
        for i in range(self.n_r):
            k = self.n_r - 1 - i
            centers.append((-k * d * math.cos(self.alpha), self.z_c - k * d * math.sin(self.alpha)))
        for j in range(1, self.n_f + 1):
            centers.append((j * d, self.z_c))
        self.n_rollers = len(centers)
        for i, (x, z) in enumerate(centers):
            parts.children.append(
                f'<body name="rl{i}" pos="{_f(x)} 0 {_f(z)}"><joint name="rj{i}" type="hinge" axis="0 1 0" damping="1e-8"/>'
                f'<inertial pos="0 0 0" mass="{m:.6g}" diaginertia="{ixz:.6g} {iy:.6g} {ixz:.6g}"/>'
                f'<geom name="rg{i}" type="cylinder" size="{_f(r)} {_f(w)}" euler="90 0 0" {CLS}/></body>')
            parts.obj_pairs.append((f"rg{i}", self.mu_belt))
            parts.actuators.append(f'<velocity name="rv{i}" joint="rj{i}" kv="1e-4" forcerange="{-tq:.6g} {tq:.6g}"/>')
        return parts

    def bind(self, model, mujoco) -> None:
        super().bind(model, mujoco)
        self.acts = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"rv{i}") for i in range(self.n_rollers)]

    def always(self, model, data, t) -> None:
        w = self.vb / self.r                     # +ω: 上の面が +x（奥）へ動く
        for a in self.acts:
            data.ctrl[a] = w
