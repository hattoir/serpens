"""H / G: 受け身のフード（機構なし）。頭が前進して物に被さり、物の中心が奥へ入ったらゲートを閉じる。
G（漏斗）= 口が 60 → 奥で 30 に狭まる形。他の案（A / B / C）の補助として組み合わせる形でもある。
駆動数 = ゲート 1（ゲートの駆動を含めて 1）。ここが「機構を足す価値があるか」の比較の基準になる。

変種（パラメータ）:
  gate          True = 駆動のゲート / False = ゲートなし / "curtain" = **受け身の TPU 垂れ布**（駆動なし。口の面の屋根の縁に蝶番で吊るした薄い板。
                頭の前進で物に押されて奥へ開き、通したあとは自分のばねで閉じる。外へは開かない）
  curtain_f     垂れ布の「閉じる力」[N]（ASSUMED）: 下端を 45° 開いたときのばねの力。ばね定数 k = F × 垂れ布の長さ / 45°。開く力も同じばね
  gate_force_n  駆動のゲートの力の上限 [N]（既定は config の 5 N）
  clearance_mm  壁・段差の板の下端と床のすき間（既定 0.1）
  plate         口に厚さ plate の床の板（垂直な前面の段差）を足す（前回のスコップの「先端の厚み」の対照）
  skirt_mm      壁の下端に付ける柔らかいスカートの高さ（3 / 6 mm。TPU の薄い帯。ヒンジ + ばねでたわむ）。0 = なし
  skirt_k       スカートのばね定数 [N·m/rad]（ASSUMED）
  bump_mm       床の凹凸の振幅（±bump）。**絨毯の代用**（絨毯は未対応。「毛に埋まる」の代わりに、床の凹凸 ±0.5 mm でスカートと壁の下端のすき間の効きを見る）
"""
from __future__ import annotations

import math

import numpy as np

from simulation.scoop.forms.common import CLS, Form, Parts, backstop_parts, hood, merge
from simulation.scoop.model import MM, _f


def bump_floor(amp_mm: float) -> tuple[str, str]:
    """床の凹凸（高さ場）: 振幅 ±amp_mm、相関長 約 8 mm、決まった乱数（すべての設計で同じ床）。(アセット, geom)。"""
    n = 200
    rng = np.random.RandomState(12345)
    a = rng.rand(n, n)
    for _ in range(3):                                    # 3 点平均を繰り返してなめらかにする
        a = (np.roll(a, 1, 0) + a + np.roll(a, -1, 0)) / 3.0
        a = (np.roll(a, 1, 1) + a + np.roll(a, -1, 1)) / 3.0
    a = (a - a.min()) / (a.max() - a.min())
    elev = " ".join(f"{v:.5f}" for v in a.ravel())
    amp = amp_mm * MM
    asset = f'<hfield name="bumps" nrow="{n}" ncol="{n}" size="0.25 0.25 {_f(2 * amp)} 0.001" elevation="{elev}"/>'
    geom = f'<geom name="floor" type="hfield" hfield="bumps" pos="0 0 {_f(-amp)}" contype="0" conaffinity="0" rgba="0.8 0.75 0.6 1"/>'
    return asset, geom


class Passive(Form):
    name = "hood"
    drives = 1
    drive_note = "ゲート 1（機構なし）"

    def __init__(self, cfg, params, obj_name, floor_name):
        super().__init__(cfg, params, obj_name, floor_name)
        g = params.get("gate", True)
        self.gate_mode = g if g in ("curtain",) else bool(g)
        self.has_gate = self.gate_mode is True                          # False / "curtain" = 駆動のゲートなし
        if self.gate_mode == "curtain":
            self.drives = 0
            self.drive_note = "駆動なし（受け身の垂れ布）"
        elif self.gate_mode is False:
            self.drives = 0
            self.drive_note = "ゲートなし（対照）"
        self.plate = float(params.get("plate", 0.0)) * MM
        self.skirt = float(params.get("skirt_mm", 0.0)) * MM
        self.skirt_k = float(params.get("skirt_k", 5.0e-4))
        self.bump = float(params.get("bump_mm", 0.0))

    def build(self) -> Parts:
        fc = self.fc
        funnel = bool(self.p.get("funnel", False))
        zb = self.clear + self.plate if self.plate > 0 else 0.0
        c, t, D, W2, Hc = self.clear, self.wall, self.depth, self.half_w, self.height
        parts = merge(hood(self, funnel=funnel, zb=zb, gate=self.has_gate, wall_bottom=(self.skirt if self.skirt > 0 else None),
                           gate_force_n=self.p.get("gate_force_n")), backstop_parts(self))
        if self.plate > 0:
            parts.head_geoms.append(f'<geom name="cavplate" type="box" size="{_f(D / 2)} {_f(W2)} {_f(self.plate / 2)}" '
                                    f'pos="{_f(D / 2)} 0 {_f(c + self.plate / 2)}" {CLS}/>')
            parts.obj_pairs.append(("cavplate", self.mu_wall))
        if self.gate_mode == "curtain":
            F = float(self.p.get("curtain_f", 0.1))
            Lf = zb + Hc - (zb + c)                                          # 垂れ布の長さ（屋根の縁から床のすき間まで）
            k = F * Lf / math.radians(45.0)
            m = 1.5e-4                                                        # TPU 0.2 mm × 30 × 15 mm ≈ 0.1〜0.2 g（ASSUMED）
            i = m * Lf * Lf / 3.0
            parts.children.append(
                f'<body name="curtain" pos="{_f(-t)} 0 {_f(zb + Hc)}">'
                f'<joint name="curt_j" type="hinge" axis="0 -1 0" range="0 90" stiffness="{k:.6g}" damping="{2 * 0.5 * math.sqrt(k * i):.6g}" armature="{i:.6g}"/>'
                f'<inertial pos="0 0 {_f(-Lf / 2)}" mass="{m:.6g}" diaginertia="{i:.6g} {i:.6g} 1e-12"/>'
                f'<geom name="curtain_g" type="box" size="0.00015 {_f(W2 + t / 2)} {_f(Lf / 2)}" pos="0 0 {_f(-Lf / 2)}" {CLS}/></body>')
            parts.obj_pairs.append(("curtain_g", self.mu_wall))
        if self.skirt > 0:
            self._skirts(parts, funnel)
        if self.bump > 0:
            asset, geom = bump_floor(self.bump)
            parts.assets.append(asset)
            parts.floor = geom
        return parts

    def _skirts(self, parts: Parts, funnel: bool) -> None:
        """壁の下端に付ける柔らかいスカート（薄い帯。壁の下端にヒンジ、ばねでたわむ。下端が床に届く）。"""
        fc = self.fc
        h, k, t, D, W2 = self.skirt, self.skirt_k, self.wall, self.depth, self.half_w
        th = 0.15e-3                                                          # 帯の半分の厚み（0.3 mm、TPU）
        m = 5.0e-5
        strips = []                                                           # (名前, 中心 x, 中心 y, 長さ, 向き z[°], 蝶番の軸, 帯の長さ方向)
        for tag, s in (("l", 1.0), ("r", -1.0)):
            strips.append((f"sk{tag}", D / 2, s * (W2 + t / 2), D, 0.0, "1 0 0"))
        if funnel:
            lf = fc["funnel"]["length_mm"] * MM
            wf2 = fc["funnel"]["front_width_mm"] * MM / 2.0
            for tag, s in (("fl", 1.0), ("fr", -1.0)):
                x0, y0, x1, y1 = -lf, s * (wf2 + t / 2), 0.0, s * (W2 + t / 2)
                ln = math.hypot(x1 - x0, y1 - y0)
                strips.append((f"sk{tag}", (x0 + x1) / 2, (y0 + y1) / 2, ln, math.degrees(math.atan2(y1 - y0, x1 - x0)), "1 0 0"))
        for nm, cx, cy, ln, ang, axis in strips:
            i = m * (ln * ln + h * h) / 12.0
            parts.children.append(
                f'<body name="{nm}_b" pos="{_f(cx)} {_f(cy)} {_f(h)}" euler="0 0 {ang:.6g}">'
                f'<joint name="{nm}_j" type="hinge" axis="{axis}" range="-60 60" stiffness="{k:.6g}" damping="{2 * 0.3 * math.sqrt(k * i):.6g}" armature="{i:.6g}"/>'
                f'<inertial pos="0 0 {_f(-h / 2)}" mass="{m:.6g}" diaginertia="{i:.6g} {i:.6g} {i:.6g}"/>'
                f'<geom name="{nm}" type="box" size="{_f(ln / 2)} {_f(th)} {_f(h / 2)}" pos="0 0 {_f(-h / 2)}" {CLS}/></body>')
            parts.obj_pairs.append((nm, self.mu_wall))
            parts.raw_pairs.append(f'<pair geom1="{nm}" geom2="floor" condim="3" friction="0.5 0.5 1e-4 1e-4 1e-4" solref="SOLREF" solimp="SOLIMP"/>')
        # 奥の壁の下端（物が奥の壁の下へ抜けないように）
        ib = m * ((2 * (W2 + t)) ** 2 + h * h) / 12.0
        parts.children.append(
            f'<body name="skb_b" pos="{_f(D + t / 2)} 0 {_f(h)}">'
            f'<joint name="skb_j" type="hinge" axis="0 1 0" range="-60 60" stiffness="{k:.6g}" damping="{2 * 0.3 * math.sqrt(k * ib):.6g}" armature="{ib:.6g}"/>'
            f'<inertial pos="0 0 {_f(-h / 2)}" mass="{m:.6g}" diaginertia="{ib:.6g} {ib:.6g} {ib:.6g}"/>'
            f'<geom name="skb" type="box" size="{_f(th)} {_f(W2 + t)} {_f(h / 2)}" pos="0 0 {_f(-h / 2)}" {CLS}/></body>')
        parts.obj_pairs.append(("skb", self.mu_wall))
        parts.raw_pairs.append('<pair geom1="skb" geom2="floor" condim="3" friction="0.5 0.5 1e-4 1e-4 1e-4" solref="SOLREF" solimp="SOLIMP"/>')
