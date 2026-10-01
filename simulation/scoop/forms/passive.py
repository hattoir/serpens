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
  curtain_len_mm 垂れ布の長さ（既定 = 屋根から床まで全高）。短くすると、蝶番を壁の途中に付ける。**物の前の縁が口の面から垂れ布の長さ以上奥にあれば、垂れ布は物の後ろへ落ちて出口を塞げる**
  rim_fillet_mm 1 円玉・CR2032 の縁の丸み [mm]（実物の縁は丸い。既定は直角 = 段差に対して悲観側）
  plate_chamfer_deg  段差の板の前面を面取りにする（床に対する斜面の角度 [°]。90 = 垂直の段差 = 既定）。斜面は床（板の下面の高さ）から板の上面まで
  plate_round_mm     段差の板の上の前の角を丸める半径 [mm]
  rough_mm      床の粗さ（細かい高さ場。振幅 ±rough、1 mm の格子、相関長 約 2 mm）。凹凸 bump_mm（粗い格子、相関長 約 8 mm）とは別
  solref_s / timestep_s / solimp_width_mm / margin_mm  接触のやわらかさ・時間刻み（ソルバの設定への依存の確認）
  bump_mm       床の凹凸の振幅（±bump）。**絨毯の代用**（絨毯は未対応。「毛に埋まる」の代わりに、床の凹凸 ±0.5 mm でスカートと壁の下端のすき間の効きを見る）
"""
from __future__ import annotations

import math

import numpy as np

from simulation.scoop.forms.common import CLS, Form, Parts, backstop_parts, hood, merge
from simulation.scoop.model import MM, _f


def plate_polygon(c: float, t: float, D: float, chamfer_deg: float, round_mm: float) -> list[tuple[float, float]]:
    """段差の板の断面（x = 奥、z = 上）の凸多角形。下面は z = c、上面は z = c + t。前面は床に対して chamfer_deg の斜面（90 = 垂直）。
    前の上の角は半径 round_mm で丸める（辺の長さの範囲に丸めを収める）。"""
    th = math.radians(chamfer_deg)
    x_top = 0.0 if chamfer_deg >= 89.999 else t / math.tan(th)
    poly = [(0.0, c), (D, c), (D, c + t)]
    corner = (x_top, c + t)
    if round_mm <= 0:
        return poly + [corner] if x_top > 0 else poly + [(0.0, c + t)]
    pa, pb = (0.0, c), (D, c + t)
    v1 = (pa[0] - corner[0], pa[1] - corner[1])
    v2 = (pb[0] - corner[0], pb[1] - corner[1])
    l1, l2 = math.hypot(*v1), math.hypot(*v2)
    u1, u2 = (v1[0] / l1, v1[1] / l1), (v2[0] / l2, v2[1] / l2)
    phi = math.acos(max(-1.0, min(1.0, u1[0] * u2[0] + u1[1] * u2[1])))
    r = min(round_mm * MM, 0.95 * min(l1, l2) * math.tan(phi / 2))
    d = r / math.tan(phi / 2)
    t1 = (corner[0] + u1[0] * d, corner[1] + u1[1] * d)
    t2 = (corner[0] + u2[0] * d, corner[1] + u2[1] * d)
    bx, by = u1[0] + u2[0], u1[1] + u2[1]
    bl = math.hypot(bx, by)
    cx, cy = corner[0] + bx / bl * r / math.sin(phi / 2), corner[1] + by / bl * r / math.sin(phi / 2)
    a1, a2 = math.atan2(t1[1] - cy, t1[0] - cx), math.atan2(t2[1] - cy, t2[0] - cx)
    da = (a2 - a1 + math.pi) % (2 * math.pi) - math.pi
    arc = [(cx + r * math.cos(a1 + da * i / 6), cy + r * math.sin(a1 + da * i / 6)) for i in range(7)]      # t1 → t2
    return poly + arc[::-1]                                                                                     # 上面（t2）から斜面（t1）へ


def fine_rough_floor(amp_mm: float) -> tuple[str, str]:
    """床の粗さ（細かい高さ場）: 振幅 ±amp_mm、1 mm の格子、相関長 約 2 mm。作業範囲（x −0.30〜0.10、y ±0.2）だけ。決まった乱数。"""
    n = 400
    rng = np.random.RandomState(4321)
    a = rng.rand(n, n)
    for _ in range(2):
        a = (np.roll(a, 1, 0) + a + np.roll(a, -1, 0)) / 3.0
        a = (np.roll(a, 1, 1) + a + np.roll(a, -1, 1)) / 3.0
    a = (a - a.min()) / (a.max() - a.min())
    elev = " ".join(f"{v:.4f}" for v in a.ravel())
    amp = amp_mm * MM
    asset = f'<hfield name="rough" nrow="{n}" ncol="{n}" size="0.2 0.2 {_f(2 * amp)} 0.001" elevation="{elev}"/>'
    geom = f'<geom name="floor" type="hfield" hfield="rough" pos="-0.1 0 {_f(-amp)}" contype="0" conaffinity="0" rgba="0.8 0.75 0.6 1"/>'
    return asset, geom


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
        self.rough = float(params.get("rough_mm", 0.0))
        self.chamfer = float(params.get("plate_chamfer_deg", 90.0))
        self.round_r = float(params.get("plate_round_mm", 0.0))
        self.curtain_len = None if params.get("curtain_len_mm") is None else float(params["curtain_len_mm"]) * MM

    def fully_inside(self, rel, margin) -> bool:
        """短い垂れ布のときは、物の前の縁が垂れ布の長さ + margin より奥へ入るまで前進する（垂れ布が物の後ろへ落ちられるように）。"""
        if self.gate_mode == "curtain" and self.curtain_len is not None:
            rx, ry, rz = rel
            return self.r_bound + self.curtain_len + margin <= rx <= self.depth and abs(ry) <= self.half_w and self.zb - 1.0e-3 <= rz <= self.zb + self.height
        return super().fully_inside(rel, margin)

    def debug(self, model, data) -> float:
        if self.gate_mode != "curtain":
            return 0.0
        j = self.mj.mj_name2id(model, self.mj.mjtObj.mjOBJ_JOINT, "curt_j")
        return math.degrees(float(data.qpos[model.jnt_qposadr[j]]))     # 垂れ布の開き角 [°]

    def build(self) -> Parts:
        fc = self.fc
        funnel = bool(self.p.get("funnel", False))
        zb = self.clear + self.plate if self.plate > 0 else 0.0
        c, t, D, W2, Hc = self.clear, self.wall, self.depth, self.half_w, self.height
        parts = merge(hood(self, funnel=funnel, zb=zb, gate=self.has_gate, wall_bottom=(self.skirt if self.skirt > 0 else None),
                           gate_force_n=self.p.get("gate_force_n")), backstop_parts(self))
        if self.plate > 0 and self.chamfer >= 89.999 and self.round_r <= 0:
            parts.head_geoms.append(f'<geom name="cavplate" type="box" size="{_f(D / 2)} {_f(W2)} {_f(self.plate / 2)}" '
                                    f'pos="{_f(D / 2)} 0 {_f(c + self.plate / 2)}" {CLS}/>')
            parts.obj_pairs.append(("cavplate", self.mu_wall))
        elif self.plate > 0:                                                  # 面取り・丸めのある段差（凸メッシュ）
            pts = plate_polygon(c, self.plate, D, self.chamfer, self.round_r)
            v = []
            for y in (-W2, W2):
                for x, z in pts:
                    v += [_f(x), _f(y), _f(z)]
            parts.assets.append(f'<mesh name="cavplate_mesh" vertex="{" ".join(v)}"/>')
            parts.head_geoms.append(f'<geom name="cavplate" type="mesh" mesh="cavplate_mesh" {CLS}/>')
            parts.obj_pairs.append(("cavplate", self.mu_wall))
        if self.gate_mode == "curtain":
            F = float(self.p.get("curtain_f", 0.1))
            Lf = zb + Hc - (zb + c) if self.curtain_len is None else self.curtain_len       # 垂れ布の長さ（既定: 屋根の縁から床のすき間まで）
            hz = zb + Hc if self.curtain_len is None else zb + c + Lf                       # 蝶番の高さ
            k = F * Lf / math.radians(45.0)
            m = 1.5e-4                                                        # TPU 0.2 mm × 30 × 15 mm ≈ 0.1〜0.2 g（ASSUMED）
            i = m * Lf * Lf / 3.0
            parts.children.append(
                f'<body name="curtain" pos="{_f(-t)} 0 {_f(hz)}">'
                f'<joint name="curt_j" type="hinge" axis="0 -1 0" range="0 90" stiffness="{k:.6g}" damping="{2 * 0.5 * math.sqrt(k * i):.6g}" armature="{i:.6g}"/>'
                f'<inertial pos="0 0 {_f(-Lf / 2)}" mass="{m:.6g}" diaginertia="{i:.6g} {i:.6g} 1e-12"/>'
                f'<geom name="curtain_g" type="box" size="0.00015 {_f(W2 + t / 2)} {_f(Lf / 2)}" pos="0 0 {_f(-Lf / 2)}" {CLS}/></body>')
            parts.obj_pairs.append(("curtain_g", self.mu_wall))
        if self.skirt > 0:
            self._skirts(parts, funnel)
        if self.rough > 0:
            asset, geom = fine_rough_floor(self.rough)
            parts.assets.append(asset)
            parts.floor = geom
        elif self.bump > 0:
            asset, geom = bump_floor(self.bump)
            parts.assets.append(asset)
            parts.floor = geom
        return parts

    def _skirts(self, parts: Parts, funnel: bool) -> None:
        """壁の下端に付ける柔らかいスカート（薄い帯）。壁の下端にヒンジ、ばねでたわむ。**外向きに 30° 傾けて吊る**: 床の凸に下端が押されると、
        傾きが増えて外へ逃げる（垂直に吊ると、凸に押されても圧縮でしか逃げられず、頭が止まる）。下端は床（公称）に届く長さ = 高さ / cos 30°。"""
        fc = self.fc
        h, k, t, D, W2 = self.skirt, self.skirt_k, self.wall, self.depth, self.half_w
        th = 0.15e-3                                                          # 帯の半分の厚み（0.3 mm、TPU）
        tilt = math.radians(30.0)
        L = h / math.cos(tilt)                                                # 帯の長さ
        m = 5.0e-5
        strips = []                                                           # (名前, 中心 x, 中心 y, 壁に沿う長さ, 向き z[°], 外向きの符号)
        for tag, s_ in (("l", 1.0), ("r", -1.0)):
            strips.append((f"sk{tag}", D / 2, s_ * (W2 + t / 2), D, 0.0, s_))
        if funnel:
            lf = fc["funnel"]["length_mm"] * MM
            wf2 = fc["funnel"]["front_width_mm"] * MM / 2.0
            for tag, s_ in (("fl", 1.0), ("fr", -1.0)):
                x0, y0, x1, y1 = -lf, s_ * (wf2 + t / 2), 0.0, s_ * (W2 + t / 2)
                strips.append((f"sk{tag}", (x0 + x1) / 2, (y0 + y1) / 2, math.hypot(x1 - x0, y1 - y0), math.degrees(math.atan2(y1 - y0, x1 - x0)), s_))
        for nm, cx, cy, ln, ang, s_ in strips:
            i = m * (ln * ln + L * L) / 12.0
            parts.children.append(
                f'<body name="{nm}_o" pos="{_f(cx)} {_f(cy)} {_f(h)}" euler="0 0 {ang:.6g}">'
                f'<body name="{nm}_b" pos="0 0 0" euler="{s_ * math.degrees(tilt):.6g} 0 0">'
                f'<joint name="{nm}_j" type="hinge" axis="1 0 0" range="-40 40" stiffness="{k:.6g}" damping="{2 * 0.3 * math.sqrt(k * i):.6g}" armature="{i:.6g}"/>'
                f'<inertial pos="0 0 {_f(-L / 2)}" mass="{m:.6g}" diaginertia="{i:.6g} {i:.6g} {i:.6g}"/>'
                f'<geom name="{nm}" type="box" size="{_f(ln / 2)} {_f(th)} {_f(L / 2)}" pos="0 0 {_f(-L / 2)}" {CLS}/></body></body>')
            parts.obj_pairs.append((nm, self.mu_wall))
            parts.raw_pairs.append(f'<pair geom1="{nm}" geom2="floor" condim="3" friction="0.5 0.5 1e-4 1e-4 1e-4" solref="SOLREF" solimp="SOLIMP"/>')
        # 奥の壁の下端（物が奥の壁の下へ抜けないように）。外向き = +x
        ib = m * ((2 * (W2 + t)) ** 2 + L * L) / 12.0
        parts.children.append(
            f'<body name="skb_b" pos="{_f(D + t / 2)} 0 {_f(h)}" euler="0 {-math.degrees(tilt):.6g} 0">'
            f'<joint name="skb_j" type="hinge" axis="0 1 0" range="-40 40" stiffness="{k:.6g}" damping="{2 * 0.3 * math.sqrt(k * ib):.6g}" armature="{ib:.6g}"/>'
            f'<inertial pos="0 0 {_f(-L / 2)}" mass="{m:.6g}" diaginertia="{ib:.6g} {ib:.6g} {ib:.6g}"/>'
            f'<geom name="skb" type="box" size="{_f(th)} {_f(W2 + t)} {_f(L / 2)}" pos="0 0 {_f(-L / 2)}" {CLS}/></body>')
        parts.obj_pairs.append(("skb", self.mu_wall))
        parts.raw_pairs.append('<pair geom1="skb" geom2="floor" condim="3" friction="0.5 0.5 1e-4 1e-4 1e-4" solref="SOLREF" solimp="SOLIMP"/>')
