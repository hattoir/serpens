"""E: 頭を越えて引く（フック）。頭を止め、口から出たフックを持ち上げたまま物を越えて前へ出し、物の向こう側へ下ろして、手前（口）へ引いて空間へ引き込む。
引き終わったらフックを持ち上げ、ゲートを閉じる（フックは空間の中の物の上に残る）。
フックの形: 先端 丸い（径 = 高さの棒）/ 四角い（厚さ 1mm の板）、高さ 2 / 4 mm。幅 28 mm（口を通る）。
駆動数 = 3（前後 1 + 上下 1 + ゲート 1）。ASSUMED: フックの動く速さ（前へ 60、引き 30 mm/s）・力 2 N・物の位置を知る理想センサー（合図: 物の中心が口の前 40 mm 以内。物の位置は合図の時点で読む）。
フックの持ち上げ位置 = 空間の高さ − フックの高さ − 0.5 mm（立方体 10 mm を越す。空間の中に納まる）。
ゲートは、フックが引き終わって持ち上がった後にだけ閉じる（フックがゲートの面を通るため）。
"""
from __future__ import annotations

import math

from simulation.scoop.forms.common import CLS, Form, Parts, backstop_parts, hood, merge
from simulation.scoop.model import MM, _f
from simulation.scoop.runner import _smoothstep


class Hook(Form):
    name = "hook"
    drives = 3
    drive_note = "フック前後 1 + 上下 1 + ゲート 1"
    stops_head_on_mech = True

    def __init__(self, cfg, params, obj_name, floor_name):
        super().__init__(cfg, params, obj_name, floor_name)
        h = self.fc["hook"]
        self.hh = float(params["h"]) * MM
        self.round = params["tip"] == "round"
        self.tb = 1.0 * MM
        self.z_up = self.height - self.hh - h["height_up_margin_mm"] * MM
        self.z_low = h["low_mm"] * MM
        self.x_home = 8.0 * MM
        self.peak_speed_mm_s = h["extend_speed_mm_s"]

    @property
    def discrete(self) -> bool:
        return True

    @property
    def gate_delay_s(self) -> float:
        return float(self.fc["hook"]["gate_delay_s"])

    def fully_inside(self, rel, margin) -> bool:
        return False                      # ゲートはフックが終わってから

    def build(self) -> Parts:
        h = self.fc["hook"]
        parts = merge(hood(self), backstop_parts(self))
        w = h["width_mm"] * MM / 2
        F = float(h["force_n"])
        m = 0.002
        kp = F / 0.0005
        if self.round:
            g = f'<geom name="hook_g" type="cylinder" size="{_f(self.hh / 2)} {_f(w)}" euler="90 0 0" pos="0 0 {_f(self.hh / 2)}" {CLS}/>'
        else:
            g = f'<geom name="hook_g" type="box" size="{_f(self.tb / 2)} {_f(w)} {_f(self.hh / 2)}" pos="0 0 {_f(self.hh / 2)}" {CLS}/>'
        parts.children.append(
            f'<body name="hkx" pos="0 0 0"><joint name="hx" type="slide" axis="1 0 0" range="-0.2 0.04" damping="0.5"/>'
            f'<inertial pos="0 0 0" mass="{m}" diaginertia="1e-7 1e-7 1e-7"/>'
            f'<body name="hkz" pos="0 0 0"><joint name="hz" type="slide" axis="0 0 1" range="0 0.02" damping="0.5"/>'
            f'<inertial pos="0 0 0" mass="{m}" diaginertia="1e-7 1e-7 1e-7"/>{g}</body></body>')
        parts.obj_pairs.append(("hook_g", self.mu_wall))
        for j in ("hx", "hz"):
            parts.actuators.append(f'<position name="{j}_act" joint="{j}" kp="{kp:.6g}" kv="{2 * math.sqrt(kp * m):.6g}" forcerange="{-F:.6g} {F:.6g}"/>')
        return parts

    def bind(self, model, mujoco) -> None:
        super().bind(model, mujoco)
        n = lambda kind, nm: mujoco.mj_name2id(model, kind, nm)
        self.jx, self.jz = n(mujoco.mjtObj.mjOBJ_JOINT, "hx"), n(mujoco.mjtObj.mjOBJ_JOINT, "hz")
        self.ax, self.az = n(mujoco.mjtObj.mjOBJ_ACTUATOR, "hx_act"), n(mujoco.mjtObj.mjOBJ_ACTUATOR, "hz_act")

    def reset(self, model, data) -> None:
        self._plan = None
        data.qpos[model.jnt_qposadr[self.jx]] = self.x_home
        data.qpos[model.jnt_qposadr[self.jz]] = self.z_up
        data.ctrl[self.ax], data.ctrl[self.az] = self.x_home, self.z_up

    def always(self, model, data, t) -> None:
        if self._plan is None:
            data.ctrl[self.ax], data.ctrl[self.az] = self.x_home, self.z_up

    def mech_fire(self, rel, t) -> bool:
        reach = float(self.fc["hook"]["reach_trigger_mm"]) * MM
        return -reach <= rel[0] <= 0.0 and abs(rel[1]) <= self.half_w + 0.005

    def mech_step(self, model, data, t_since, rel) -> bool:
        h = self.fc["hook"]
        if self._plan is None:
            x_far = rel[0] - self.r_bound - h["beyond_mm"] * MM
            x_end = self.depth / 2 - self.r_bound - self.tb / 2
            tv = h["vertical_time_s"]
            t1 = max(0.1, abs(x_far - self.x_home) / (h["extend_speed_mm_s"] * MM))
            t3 = max(0.1, abs(x_end - x_far) / (h["pull_speed_mm_s"] * MM))
            self._plan = (x_far, x_end, tv, t1, t3)
        x_far, x_end, tv, t1, t3 = self._plan
        t = t_since
        zu, zl, xh = self.z_up, self.z_low, self.x_home
        if t < t1:                                   # 1 前へ出す（持ち上げたまま）
            x, z = xh + (x_far - xh) * _smoothstep(t / t1), zu
        elif t < t1 + tv:                            # 2 物の向こうへ下ろす
            x, z = x_far, zu + (zl - zu) * _smoothstep((t - t1) / tv)
        elif t < t1 + tv + t3:                       # 3 引く（物を口の中へ）
            x, z = x_far + (x_end - x_far) * _smoothstep((t - t1 - tv) / t3), zl
        elif t < t1 + 2 * tv + t3:                   # 4 持ち上げる
            x, z = x_end, zl + (zu - zl) * _smoothstep((t - t1 - tv - t3) / tv)
        else:
            x, z = x_end, zu
        data.ctrl[self.ax], data.ctrl[self.az] = x, z
        return t >= t1 + 2 * tv + t3 + 0.1
