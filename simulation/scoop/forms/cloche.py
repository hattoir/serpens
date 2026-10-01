"""H: フード昇降（cloche drop）。開放底のフード + ゲート（`passive.py`）の全体を、上下に動く 1 つの台（`lift` 関節）に載せる。
フードを床から `lift_mm` 上げたまま近づき、物の中心が空間の中へ入ったら頭を止め、フードを床へ落として（`drop_speed_mm_s`、力の上限 `drop_force_n`）物に被せ、
そのあとゲートを閉じる（ラッチ）。**口の前縁に床の板・段差を持たない**（`plate` = 0）ので、「段差はほぼゼロが必須」の制約に当たらない見込み。
駆動数 = 2（フードの昇降 + ゲート）。位置合わせの動き・センサーは理想（物の中心が入ったことを知る。ASSUMED）。
評価: 保持（閉じ終わりから 2 秒後まで空間の中）= 「その場で覆う」。`retreat` = +30 で頭が 30 mm 後退、−30 で 30 mm 前進（引きずる）= 「運ぶ」（フードを下ろしたまま）。
**MUJOCO_SIM。実物ではない。摩擦・質量・力・速さは ASSUMED。**
"""
from __future__ import annotations

import math

from simulation.scoop.forms.common import Form, Parts
from simulation.scoop.forms.passive import Passive
from simulation.scoop.model import MM, _f
from simulation.scoop.runner import _smoothstep

HOOD_MASS_KG = 0.011                       # フード PLA 11.4 g（Design、ENTRY-D-0006。ASSUMED）


class Cloche(Passive):
    name = "cloche"
    drives = 2
    drive_note = "昇降 1 + ゲート 1"
    stops_head_on_mech = True

    def __init__(self, cfg, params, obj_name, floor_name):
        params = {"funnel": True, "gate": True, **params}
        super().__init__(cfg, params, obj_name, floor_name)
        self.lift = float(params.get("lift_mm", 5.0)) * MM
        self.drop_v = float(params.get("drop_speed_mm_s", 20.0)) * MM
        self.drop_force = float(params.get("drop_force_n", self.fc["cup"]["force_n"]))
        self.t_lower = max(self.lift / self.drop_v, 0.05)
        self.peak_speed_mm_s = self.drop_v / MM
        self._lowering = False
        self._down = False
        self._t_jam = None

    def build(self) -> Parts:
        p = super().build()
        body = (f'<body name="cloche" pos="0 0 0"><joint name="lift" type="slide" axis="0 0 1" range="0 {_f(self.lift + 0.002)}" damping="0.5"/>'
                f'<inertial pos="{_f(self.depth / 2)} 0 0.01" mass="{HOOD_MASS_KG}" diaginertia="1e-6 1e-6 1e-6"/>'
                + "".join(p.head_geoms) + "".join(p.children) + "</body>")
        p.head_geoms, p.children = [], [body]
        kp = self.drop_force / 0.0005
        p.actuators.append(f'<position name="lift_act" joint="lift" kp="{kp:.6g}" kv="{2 * math.sqrt(kp * HOOD_MASS_KG):.6g}" '
                           f'forcerange="{-self.drop_force:.6g} {self.drop_force:.6g}"/>')
        return p

    def bind(self, model, mujoco) -> None:
        super().bind(model, mujoco)
        self.jl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "lift")
        self.al = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "lift_act")

    def reset(self, model, data) -> None:
        self._lowering = self._down = False
        self._t_jam = None
        data.qpos[model.jnt_qposadr[self.jl]] = self.lift
        data.ctrl[self.al] = self.lift

    def always(self, model, data, t) -> None:
        if not self._lowering:
            data.ctrl[self.al] = self.lift

    def _q(self, model, data) -> float:
        return float(data.qpos[model.jnt_qposadr[self.jl]])

    # 落とす合図: 物の中心が、外接半径 + margin だけ空間の中へ入った（理想センサー）
    def mech_fire(self, rel, t) -> bool:
        return Form.fully_inside(self, rel, float(self.fc["trigger"]["full_inside_margin_mm"]) * MM)

    def mech_step(self, model, data, t_since, rel) -> bool:
        self._lowering = True
        data.ctrl[self.al] = self.lift * (1.0 - _smoothstep(t_since / self.t_lower))
        if t_since >= self.t_lower + 0.1:
            self._down = self._q(model, data) <= 0.0006                 # 落ちきった（すき間 c は壁の下端の高さで、関節の 0）
            if not self._down and self._t_jam is None:
                self._t_jam = t_since
            return self._down or (t_since - self._t_jam) >= 1.0        # 落ちきれない（物の上に乗った）なら 1 秒で打ち切る
        return False

    def jammed(self, model, data) -> bool:
        return self._lowering and not self._down and self._t_jam is not None and self._q(model, data) > 0.0006

    def fully_inside(self, rel, margin) -> bool:
        return False                     # ゲートは、落とし終わってから（gate_delay_s = 0）閉じる

    @property
    def gate_delay_s(self):
        return 0.0

    @property
    def discrete(self) -> bool:
        return True

    def debug(self, model, data) -> float:
        return self._q(model, data) / MM                    # フードの持ち上がり [mm]（診断。0 = 落ちきった）
