"""口の形と機構の共通部分（**すべて MUJOCO_SIM。実物ではない**。摩擦・質量・トルクは ASSUMED。config/scoop.yaml の `forms`）。

座標: 頭の原点 = ゲート面（口の面）。**頭は −x 方向へ進む**。空間はゲート面から +x 側（奥）へ depth。
  rel_x = 物の x − 頭の x … 負なら口の前、0 ≤ rel_x ≤ depth なら空間の中。y は左右、z は上。床は z = 0 の剛体平面（絨毯は未対応）。
口 = 「開放底のフード」: 屋根・両脇の壁・奥の壁があり、**床がそのまま空間の床になる**（傾斜板と先端の厚みの制約が無い）。壁の下端は床から clearance だけ浮く。
  絨毯では、壁の下端が毛に沈む・毛が壁の下から入るなど、この前提が崩れる可能性が高い（未対応）。
ゲート = 口の面に立つ薄い板（上から降りる）。閉じたらラッチ（可動域を閉位置に絞る）。
物との接触は `<contact><pair>` で明示（物以外の幾何は contype = 0 で互いに当たらない）。
乗る（ride）の定義: 物の**傾斜板に向かう頭側（+x 側）の縁**が床から 1mm 以上持ち上がった瞬間が一度でもあった（前回から変えない）。床が空間の床の案では、乗るは起きない（N/A）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from simulation.scoop.model import MM, OBJECT_GROUP, _f, _object_geom

CLS = 'contype="0" conaffinity="0" rgba="0.55 0.7 0.9 1"'


def prism_xy(name: str, pts: list[tuple[float, float]], z0: float, z1: float) -> tuple[str, str]:
    """x–y 面の凸多角形を z 方向へ押し出した凸メッシュ（名前と頂点列）。"""
    v: list[str] = []
    for z in (z0, z1):
        for x, y in pts:
            v += [_f(x), _f(y), _f(z)]
    return name, " ".join(v)


@dataclass
class Parts:
    head_geoms: list[str] = field(default_factory=list)      # head 直下の geom
    children: list[str] = field(default_factory=list)        # head の子の body（関節つき）
    actuators: list[str] = field(default_factory=list)
    assets: list[str] = field(default_factory=list)
    obj_pairs: list[tuple[str, float]] = field(default_factory=list)   # (geom 名, 物との μ)
    raw_pairs: list[str] = field(default_factory=list)
    world: list[str] = field(default_factory=list)            # 頭に付かない静的な幾何・body
    floor: str | None = None                                  # 床の geom を差し替える（凹凸の高さ場）。None = 平面
    info: dict[str, Any] = field(default_factory=dict)


class Form:
    """案の共通の土台。各案（sweeper / belt / brush / cup / hook / hood）はこれを継承する。"""

    name = "form"
    has_gate = True                 # 口のゲートで閉じる案（D のカップは持たない）
    head_moves = True               # 頭が一定速度で前進する案（D は動かない）
    static_placement = False        # D: 物をカップの真下から ずらした位置に置く
    drives = 2                      # 駆動数（ゲートを含む）。各案で上書き
    drive_note = ""
    peak_speed_mm_s = 0.0           # 機構の面 / 先端の最大速度（飛ばされた の閾値に使う）

    def __init__(self, cfg: dict[str, Any], params: dict[str, Any], obj_name: str, floor_name: str) -> None:
        self.cfg, self.p, self.obj_name, self.floor_name = cfg, dict(params), obj_name, floor_name
        self.fc = cfg["forms"]
        ov: dict[str, Any] = {}                                          # 接触の設定の上書き（solref / 時間刻み / solimp の幅）。既定は config
        if "solref_s" in self.p:
            ov["solref_time_const_s"] = float(self.p["solref_s"])
        if "timestep_s" in self.p:
            ov["timestep_s"] = float(self.p["timestep_s"])
        if "solimp_width_mm" in self.p:
            ov["solimp"] = [*cfg["sim"]["solimp"][:2], float(self.p["solimp_width_mm"]) * MM]
        if ov:
            self.cfg = {**cfg, "sim": {**cfg["sim"], **ov}}
        cv = self.fc["cavity"]
        self.depth = cv["depth_mm"] * MM
        self.half_w = cv["width_mm"] * MM / 2.0
        self.height = cv["height_mm"] * MM
        self.wall = cv["wall_mm"] * MM
        self.roof = cv["roof_mm"] * MM
        self.clear = float(self.p.get("clearance_mm", cv["clearance_mm"])) * MM
        self.zb = 0.0                 # 空間の床の高さ（開放底 = 0。ベルトは持ち上がる）
        self.front_extent = 0.0       # 開いた状態で頭の一番前が口の面よりどれだけ前へ出るか（物の初期位置の基準）
        ob = cfg["objects"][obj_name]
        rf = self.p.get("rim_fillet_mm")                                   # 円柱（1 円玉・CR2032）の縁の丸み [mm]。既定は直角（悲観側）
        self.obj_geom_xml, self.obj_half_h, self.r_bound, self.obj_asset = _object_geom(ob, None if rf is None else float(rf))
        self.mu_wall = float(self.fc["friction_wall"])

    # --- ビルド ---
    def build(self) -> Parts:  # pragma: no cover - 各案で実装
        raise NotImplementedError

    def key(self) -> tuple:
        return (self.name, tuple(sorted(self.p.items(), key=str)), self.obj_name, self.floor_name)

    # --- 実行時のフック（ids は bind で作る） ---
    def bind(self, model: Any, mujoco: Any) -> None:
        self.mj = mujoco
        self.model = model
        self.gate_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "gate_z") if self.has_gate else -1
        # 口の壁・漏斗・段差の geom（物が口の前の縁に当たったかの判定に使う）
        ids = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) for n in ("wall_l", "wall_r", "fwall_l", "fwall_r", "cavplate", "skl", "skr", "skfl", "skfr", "skb", "curtain_g")}
        self.wall_ids = ids - {-1}
        self.gate_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gate_act") if self.has_gate else -1

    def reset(self, model: Any, data: Any) -> None:
        pass

    def debug(self, model: Any, data: Any) -> float:
        """診断（trace 用）の値。垂れ布の角度など。"""
        return 0.0

    def always(self, model: Any, data: Any, t: float) -> None:
        """常時動く機構（ベルト・ブラシ）の指令。"""

    def mech_fire(self, rel: tuple[float, float, float], t: float) -> bool:
        """離散的な機構（腕・フック）の開始の合図。常時型・カップ以外は False。"""
        return False

    def mech_step(self, model: Any, data: Any, t_since: float, rel: tuple[float, float, float]) -> bool:
        """離散的な機構の 1 ステップ。終わったら True。"""
        return True

    def closed_ok(self, model: Any, data: Any) -> bool:
        """ゲートの無い案（カップ）で、実際に閉じ位置（床）まで下りたか。"""
        return True

    stops_head_on_mech = False      # 機構が始まったら頭を止める案（フックなど）

    # --- 空間の判定（D のカップは上書き） ---
    def inside(self, rel: tuple[float, float, float]) -> bool:
        rx, ry, rz = rel
        return 0.0 <= rx <= self.depth and abs(ry) <= self.half_w and self.zb - 1.0e-3 <= rz <= self.zb + self.height

    def fully_inside(self, rel: tuple[float, float, float], margin: float) -> bool:
        """物の中心が、外接半径 + margin だけ空間の奥へ入った（理想センサー）。"""
        rx, ry, rz = rel
        return self.r_bound + margin <= rx <= self.depth and abs(ry) <= self.half_w and self.zb - 1.0e-3 <= rz <= self.zb + self.height

    def escape(self, rel: tuple[float, float, float]) -> float:
        rx, ry, _ = rel
        return math.hypot(max(-rx, 0.0, rx - self.depth), max(abs(ry) - self.half_w, 0.0))

    @property
    def discrete(self) -> bool:
        return False

    @property
    def gate_delay_s(self) -> float | None:
        """離散的な機構が終わってから、物が完全に入っていなくてもゲートを閉じるまでの待ち。None = 待たない。"""
        return None


def hood(f: Form, funnel: bool = False, zb: float = 0.0, x_front: float | None = None, gate: bool = True,
         wall_bottom: float | None = None, gate_force_n: float | None = None) -> Parts:
    """開放底のフード（両脇の壁・屋根・奥の壁・ゲート）。funnel なら口が 60 → 奥で 30 に狭まる（G）。
    x_front を渡すと、壁を口の面から前へ x_front だけ延ばす（ベルトのランプの脇）。"""
    fc = f.fc
    c, t, rt = f.clear, f.wall, f.roof
    D, W2, Hc = f.depth, f.half_w, f.height
    p = Parts()
    z0, z1 = (c if wall_bottom is None else wall_bottom), zb + Hc + rt        # wall_bottom = スカートを付けるとき、壁の下端の高さ
    lf = fc["funnel"]["length_mm"] * MM if funnel else 0.0
    wf2 = fc["funnel"]["front_width_mm"] * MM / 2.0 if funnel else W2
    front = max(lf, x_front or 0.0)
    if funnel:
        f.front_extent = max(f.front_extent, lf)
    # 直線の壁（口の面から奥の壁まで）
    for nm, s in (("wall_l", 1.0), ("wall_r", -1.0)):
        y_c = s * (W2 + t / 2)
        p.head_geoms.append(f'<geom name="{nm}" type="box" size="{_f(D / 2)} {_f(t / 2)} {_f((z1 - z0) / 2)}" '
                            f'pos="{_f(D / 2)} {_f(y_c)} {_f((z0 + z1) / 2)}" {CLS}/>')
        p.obj_pairs.append((nm, f.mu_wall))
    # 前へ延びる壁（漏斗、または直線の延長）
    for nm, s in (("fwall_l", 1.0), ("fwall_r", -1.0)):
        if funnel:
            pts = [(-lf, wf2), (0.0, W2), (0.0, W2 + t), (-lf, wf2 + t)]
            pts = [(x, s * y) for x, y in pts]
            n, vtx = prism_xy(f"{nm}_mesh", pts, z0, z1)
            p.assets.append(f'<mesh name="{n}" vertex="{vtx}"/>')
            p.head_geoms.append(f'<geom name="{nm}" type="mesh" mesh="{n}" {CLS}/>')
            p.obj_pairs.append((nm, f.mu_wall))
        elif front > 0:
            y_c = s * (W2 + t / 2)
            p.head_geoms.append(f'<geom name="{nm}" type="box" size="{_f(front / 2)} {_f(t / 2)} {_f((z1 - z0) / 2)}" '
                                f'pos="{_f(-front / 2)} {_f(y_c)} {_f((z0 + z1) / 2)}" {CLS}/>')
            p.obj_pairs.append((nm, f.mu_wall))
    # 屋根と奥の壁
    p.head_geoms.append(f'<geom name="roof" type="box" size="{_f(D / 2 + t / 2)} {_f(W2 + t)} {_f(rt / 2)}" '
                        f'pos="{_f(D / 2 + t / 2)} 0 {_f(zb + Hc + rt / 2)}" {CLS}/>')
    p.head_geoms.append(f'<geom name="back" type="box" size="{_f(t / 2)} {_f(W2 + t)} {_f((z1 - z0) / 2)}" '
                        f'pos="{_f(D + t / 2)} 0 {_f((z0 + z1) / 2)}" {CLS}/>')
    p.obj_pairs += [("roof", f.mu_wall), ("back", f.mu_wall)]
    f.zb = zb
    if not gate:                                  # ゲートなし（ゲートの効果を切り分ける対照）
        p.info.update(gate_open=Hc + 0.003)
        return p
    # ゲート（口の面のすぐ前に立つ板。上から降りる）
    gm = float(fc["gate"]["mass_g"]) / 1000.0
    gh = zb + Hc - (zb + c)                     # 板の高さ（床 / 空間の床から屋根の下まで）
    p.children.append(
        f'<body name="gate" pos="0 0 0"><joint name="gate_z" type="slide" axis="0 0 1" range="0 {_f(Hc + 0.004)}" damping="0.5"/>'
        f'<inertial pos="0 0 0" mass="{gm:.6g}" diaginertia="1e-7 1e-7 1e-7"/>'
        f'<geom name="gate_plate" type="box" size="{_f(t / 2)} {_f(W2 + t / 2)} {_f(gh / 2)}" pos="{_f(-t / 2)} 0 {_f(zb + c + gh / 2)}" {CLS}/></body>')
    p.obj_pairs.append(("gate_plate", f.mu_wall))
    gf = float(fc["gate"]["force_n"] if gate_force_n is None else gate_force_n)      # ゲートの力の上限 [N]
    kp = gf / 0.0005
    p.actuators.append(f'<position name="gate_act" joint="gate_z" kp="{kp:.6g}" kv="{2 * math.sqrt(kp * gm):.6g}" '
                       f'forcerange="{-gf:.6g} {gf:.6g}"/>')
    f.zb = zb
    p.info.update(gate_open=Hc + 0.003)
    return p


def backstop_parts(f: Form) -> Parts:
    """F: 物の前に置く壁 / 脚（φ30）。mocap で、エピソードごとに位置を決める。"""
    p = Parts()
    kind = f.p.get("backstop")
    if not kind:
        return p
    if kind == "wall":
        g = '<geom name="stop_geom" type="box" size="0.005 0.15 0.03" ' + CLS + '/>'
    else:
        g = f'<geom name="stop_geom" type="cylinder" size="{_f(f.fc["backstop"]["leg_diameter_mm"] * MM / 2)} 0.03" ' + CLS + '/>'
    p.world.append(f'<body name="stopper" mocap="true" pos="-1 0 0.03">{g}</body>')
    p.obj_pairs.append(("stop_geom", f.mu_wall))
    return p


def merge(a: Parts, b: Parts) -> Parts:
    a.head_geoms += b.head_geoms
    a.children += b.children
    a.actuators += b.actuators
    a.assets += b.assets
    a.obj_pairs += b.obj_pairs
    a.raw_pairs += b.raw_pairs
    a.world += b.world
    a.info.update(b.info)
    return a


def assemble(f: Form, parts: Parts) -> str:
    """MJCF を組み立てる。"""
    cfg = f.cfg
    sim = cfg["sim"]
    dt = float(sim["timestep_s"])
    solref = f"{float(sim['solref_time_const_s']):.6g} 1"
    solimp = " ".join(f"{float(x):.6g}" for x in sim["solimp"])
    ob = cfg["objects"][f.obj_name]
    mu_floor = float(ob.get("friction_floor", cfg["floors"][f.floor_name]["friction"]))
    rolling = float(ob.get("rolling_friction_m", 1.0e-4))
    condim = 6 if ob["shape"] == "sphere" else 3
    pairs = []
    mg = float(f.p.get("margin_mm", 0.0)) * 1.0e-3
    marg = f' margin="{mg:.6g}" gap="0"' if mg > 0 else ""
    for gname, mu in [("floor", mu_floor)] + parts.obj_pairs:
        pairs.append(f'<pair geom1="obj_geom" geom2="{gname}" condim="{condim}" friction="{mu:.6g} {mu:.6g} 1e-4 {rolling:.6g} {rolling:.6g}" '
                     f'solref="{solref}" solimp="{solimp}"{marg}/>')
    for r in parts.raw_pairs:
        pairs.append(r.replace("SOLREF", solref).replace("SOLIMP", solimp))
    assets = list(parts.assets)
    if f.obj_asset:
        assets.append(f.obj_asset)
    cone = "elliptic" if sim.get("cone", "elliptic") == "elliptic" else "pyramidal"
    force = float(cfg["motion"]["push_force_limit_n"])
    nl = "\n"
    return f"""<mujoco model="form_{f.name}">
  <compiler angle="degree"/>
  <option timestep="{dt:.6g}" gravity="0 0 -9.81" integrator="implicitfast" cone="{cone}" impratio="1">
    <flag multiccd="{'enable' if sim.get('multiccd', True) else 'disable'}"/>
  </option>
  <asset>
    {nl.join('    ' + s for s in assets).strip()}
  </asset>
  <worldbody>
    {parts.floor or '<geom name="floor" type="plane" size="2 2 0.1" contype="0" conaffinity="0" rgba="0.8 0.75 0.6 1"/>'}
    {nl.join('    ' + s for s in parts.world).strip()}
    <body name="head" pos="0 0 0">
      <joint name="slide" type="slide" axis="1 0 0" damping="0"/>
      <inertial pos="{_f(f.depth / 2)} 0 0.01" mass="{float(cfg['head']['mass_g']) / 1000.0:.6g}" diaginertia="2e-5 2e-5 2e-5"/>
      {nl.join('      ' + s for s in parts.head_geoms).strip()}
      {nl.join('      ' + s for s in parts.children).strip()}
    </body>
    <body name="obj" pos="-0.1 0 {_f(f.obj_half_h + 1.0e-5)}">
      <freejoint name="obj_free"/>
      {f.obj_geom_xml}
    </body>
  </worldbody>
  <contact>
    {nl.join('    ' + s for s in pairs).strip()}
  </contact>
  <actuator>
    <velocity name="vel" joint="slide" kv="50" forcerange="{-force:.6g} {force:.6g}"/>
    {nl.join('    ' + s for s in parts.actuators).strip()}
  </actuator>
</mujoco>
"""
