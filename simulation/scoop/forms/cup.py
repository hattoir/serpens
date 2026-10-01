"""D: 被せる（カップ）。上から頭のカップを物に被せ、縁を床に付けて（隙間 0 / 0.3 / 1 mm）閉じ込める。カップ内径 30 / 40 mm。
物はカップの真下から 0 / 3 / 6 / 10 mm ずれた位置にある（円周方向はランダム。カップを下ろす前の位置合わせの誤差）。
カップは持ち上げた位置から下ろす（下ろし切れば、そのまま閉じ込め = ラッチ）。ゲートは無い。頭は動かない（前進速度は関係しない）。
駆動数 = 1（昇降）。位置合わせのための頭の移動・センサーは含めない（理想。ASSUMED）。
記録: 縁が物の上に乗るか（挟まる）、物が縁の下へ逃げるか、押されるか（最初の位置からの移動）。
"""
from __future__ import annotations

import math

from simulation.scoop.forms.common import CLS, Form, Parts
from simulation.scoop.model import MM, _f
from simulation.scoop.runner import _smoothstep


class Cup(Form):
    name = "cup"
    has_gate = False
    head_moves = False
    static_placement = True
    drives = 1
    drive_note = "昇降 1（ゲートなし）"

    def __init__(self, cfg, params, obj_name, floor_name):
        super().__init__(cfg, params, obj_name, floor_name)
        c = self.fc["cup"]
        self.Rin = float(params["ID"]) * MM / 2
        self.gap = float(params["gap_mm"]) * MM
        self.lift = c["lift_mm"] * MM
        self.speed = c["lower_speed_mm_s"] * MM
        self.t_lower = self.lift / self.speed
        self.wall_t = c["wall_mm"] * MM
        self.peak_speed_mm_s = self.speed / MM
        self.zb = 0.0
        self.half_w = self.Rin                       # 空間 = カップの内側（円）

    @property
    def discrete(self) -> bool:
        return True

    def build(self) -> Parts:
        c = self.fc["cup"]
        p = Parts()
        n, t, Hc = 24, self.wall_t, self.height
        Rm = self.Rin + t / 2
        seg = 2 * math.pi * (self.Rin + t) / n
        geoms = []
        for k in range(n):
            th = 360.0 * k / n
            a = math.radians(th)
            geoms.append(f'<geom name="cw{k}" type="box" size="{_f(t / 2)} {_f(seg / 2 * 1.03)} {_f(Hc / 2)}" '
                         f'pos="{_f(Rm * math.cos(a))} {_f(Rm * math.sin(a))} {_f(Hc / 2)}" euler="0 0 {th:.6g}" {CLS}/>')
            p.obj_pairs.append((f"cw{k}", self.mu_wall))
            p.raw_pairs.append(f'<pair geom1="cw{k}" geom2="floor" condim="3" friction="0.5 0.5 1e-4 1e-4 1e-4" solref="SOLREF" solimp="SOLIMP"/>')
        geoms.append(f'<geom name="croof" type="cylinder" size="{_f(self.Rin + t)} {_f(self.roof / 2)}" pos="0 0 {_f(Hc + self.roof / 2)}" {CLS}/>')
        p.obj_pairs.append(("croof", self.mu_wall))
        m = 0.05
        p.children.append(
            f'<body name="cup" pos="0 0 {_f(self.gap)}"><joint name="lift" type="slide" axis="0 0 1" range="0 {_f(self.lift + 0.002)}" damping="0.5"/>'
            f'<inertial pos="0 0 {_f(Hc / 2)}" mass="{m}" diaginertia="1e-5 1e-5 1e-5"/>{"".join(geoms)}</body>')
        kp = c["force_n"] / 0.0005
        p.actuators.append(f'<position name="lift_act" joint="lift" kp="{kp:.6g}" kv="{2 * math.sqrt(kp * m):.6g}" '
                           f'forcerange="{-c["force_n"]:.6g} {c["force_n"]:.6g}"/>')
        return p

    def bind(self, model, mujoco) -> None:
        super().bind(model, mujoco)
        self.jl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "lift")
        self.al = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "lift_act")

    def reset(self, model, data) -> None:
        self._lowering = False
        data.qpos[model.jnt_qposadr[self.jl]] = self.lift
        data.ctrl[self.al] = self.lift

    def always(self, model, data, t) -> None:
        if not self._lowering:
            data.ctrl[self.al] = self.lift

    def mech_fire(self, rel, t) -> bool:
        return t >= float(self.fc["cup"]["start_delay_s"])

    def mech_step(self, model, data, t_since, rel) -> bool:
        self._lowering = True
        data.ctrl[self.al] = self.lift * (1.0 - _smoothstep(t_since / self.t_lower))
        return t_since >= self.t_lower + 0.1

    def closed_ok(self, model, data) -> bool:
        return float(data.qpos[model.jnt_qposadr[self.jl]]) <= 0.0006          # 縁が下りきった（隙間の分は関節の 0 に含まれる）

    def inside(self, rel) -> bool:
        return math.hypot(rel[0], rel[1]) <= self.Rin

    def fully_inside(self, rel, margin) -> bool:
        return False

    def escape(self, rel) -> float:
        return max(0.0, math.hypot(rel[0], rel[1]) - self.Rin)
