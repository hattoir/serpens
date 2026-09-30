"""頭のスコップ＋フタの MJCF を `config/scoop.yaml` から生成する。**手で XML を書かない**（config とずれるため）。

座標: 先端（スコップの床側の前の縁）が頭の原点。**頭は −x 方向へ進む**（先端が先頭）。ランプと閉じ込め空間は +x 側（後ろ）。
  rel_x = 物の x − 先端の x … 正なら先端より後ろ（頭の中側）、負なら先端より前（まだ触れていない側）
  y は左右、z は上。床は z = 0 の剛体平面（絨毯は未対応）。
形（すべて config）:
  傾斜板  … 先端（床から clearance の高さ）から角度 α で斜めに上がる薄板（板厚 = 先端厚 t、斜面に沿って L）
  側壁    … カップ型のとき、傾斜板の両脇に床から立つ壁（傾斜板の上面 + wall_height まで）
  空間    … ランプ上端から水平に続く床（長さ × 幅）、両脇と奥の壁、高さ 15mm
  フタ    … 空間の奥の上端にヒンジ。開き 60° → 閉 0°（θ > 0 で前縁が持ち上がる）
接触: 物と他の幾何の摩擦は `<contact><pair>` で明示する（MuJoCo の既定は 2 つの摩擦の大きい方を取るので使わない）。
       物以外の幾何は contype = 0（互いに当たらない）。頭は x 方向のスライダー 1 つだけで、高さ・姿勢は固定。
**摩擦・質量・押しつけ力はすべて ASSUMED**（config の出典コメント）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

MM = 1.0e-3
ROOT = Path(__file__).resolve().parent.parent.parent
CONFIG_PATH = ROOT / "config" / "scoop.yaml"
MODEL_VERSION = "scoop-mjcf-0.1"
OBJECT_GROUP = 2          # 物の geom グループ（ToF のレイキャストが見る）


def load_config(path: str | Path | None = None) -> dict[str, Any]:
    with Path(path or CONFIG_PATH).open(encoding="utf-8") as f:
        return yaml.safe_load(f)


@dataclass(frozen=True)
class Shape:
    """スコップ形状 1 つ（掃引の単位）。"""

    tip_mm: float
    alpha_deg: float
    side_wall: bool
    width_mm: float
    ramp_mm: float | None = None      # 斜面に沿った傾斜板の長さ。None = config の scoop.depth_along_slope_mm（30）

    @property
    def key(self) -> str:
        base = f"t{self.tip_mm:g}_a{self.alpha_deg:g}_{'cup' if self.side_wall else 'open'}_w{self.width_mm:g}"
        return base if self.ramp_mm is None else f"{base}_r{self.ramp_mm:g}"


def short_ramp_shapes(cfg: dict[str, Any]) -> list[Shape]:
    """短ランプの掃引（config の ramp_sweep）: 先端厚 × ランプ長（斜め角は固定）。空間はランプ終端から始まる。"""
    r = cfg["ramp_sweep"]
    return [Shape(t, r["alpha_deg"], False, r["width_mm"], L) for t in r["tip_thickness_mm"] for L in r["ramp_mm"]]


def all_shapes(cfg: dict[str, Any]) -> list[Shape]:
    s = cfg["scoop"]["sweep"]
    return [Shape(t, a, w_, w) for t in s["tip_thickness_mm"] for a in s["alpha_deg"]
            for w_ in s["side_wall"] for w in s["width_mm"]]


@dataclass(frozen=True)
class Geometry:
    """モデルの幾何（m）。分類・トリガーで使う。"""

    alpha: float
    tip: float
    clearance: float
    ramp_len: float
    half_width: float
    end_x: float            # ランプ上端（上面）の rel_x
    end_z: float            # 空間の床の上面の高さ
    cav_len: float
    cav_half_width: float
    cav_height: float
    wall_t: float
    hinge_x: float
    hinge_z: float

    def ramp_top(self, rel_x: float) -> float:
        """傾斜板の上面の高さ（rel_x は先端からの水平距離）。"""
        return self.clearance + self.tip * math.cos(self.alpha) + max(rel_x, 0.0) * math.tan(self.alpha)

    def ramp_bottom(self, rel_x: float) -> float:
        return self.clearance + max(rel_x, 0.0) * math.tan(self.alpha)


@dataclass(frozen=True)
class ModelSpec:
    xml: str
    geom: Geometry
    obj_half_height: float
    obj_radius: float
    lid_length: float = 0.0
    lid_open_rad: float = 0.0
    lid_closed_rad: float = 0.0
    version: str = MODEL_VERSION


def geometry_of(cfg: dict[str, Any], shape: Shape, clearance_mm: float | None = None) -> Geometry:
    a = math.radians(shape.alpha_deg)
    t = shape.tip_mm * MM
    c = (cfg["scoop"]["tip_clearance_mm"] if clearance_mm is None else clearance_mm) * MM
    L = (cfg["scoop"]["depth_along_slope_mm"] if shape.ramp_mm is None else shape.ramp_mm) * MM
    cv = cfg["cavity"]
    end_x = L * math.cos(a) - t * math.sin(a)
    end_z = c + L * math.sin(a) + t * math.cos(a)
    cav_len = cv["length_mm"] * MM
    return Geometry(alpha=a, tip=t, clearance=c, ramp_len=L, half_width=shape.width_mm * MM / 2.0,
                    end_x=end_x, end_z=end_z, cav_len=cav_len, cav_half_width=cv["width_mm"] * MM / 2.0,
                    cav_height=cv["height_mm"] * MM, wall_t=cv["wall_mm"] * MM,
                    hinge_x=end_x + cav_len, hinge_z=end_z + cv["height_mm"] * MM)


def _f(v: float) -> str:
    return f"{v:.7g}"


def _prism(name: str, pts_xz: list[tuple[float, float]], y0: float, y1: float) -> tuple[str, str]:
    """x–z 面の凸多角形を y 方向へ押し出した凸メッシュ（アセットの文言、geom を作る前の頂点列）。"""
    v = []
    for y in (y0, y1):
        for x, z in pts_xz:
            v += [x, y, z]
    return name, " ".join(_f(x) for x in v)


def build_mjcf(cfg: dict[str, Any], shape: Shape, obj_name: str, floor_name: str, *,
               clearance_mm: float | None = None, scoop_mu: float | None = None, lid_mu: float | None = None,
               rolling_scale: float = 1.0, numerics: dict[str, Any] | None = None,
               rim_fillet_mm: float | None = None, front_face: str | None = None,
               lid_front_ahead_mm: float | None = None, beak: tuple[float, float] | None = None) -> ModelSpec:
    """1 つの（形 × 対象物 × 床）の MJCF。物の初期位置は qpos で与える（モデルは使い回せる）。"""
    g = geometry_of(cfg, shape, clearance_mm)
    sim = {**cfg["sim"], **(numerics or {})}
    dt = float(sim["timestep_s"])
    solref = f"{float(sim['solref_time_const_s']):.6g} 1"
    solimp = " ".join(f"{float(x):.6g}" for x in sim["solimp"])
    mu_s = float(cfg["scoop"]["friction_object"] if scoop_mu is None else scoop_mu)
    mu_l = float(cfg["lid"]["friction_object"] if lid_mu is None else lid_mu)
    ob = cfg["objects"][obj_name]
    mu_floor = float(ob.get("friction_floor", cfg["floors"][floor_name]["friction"]))
    hd, lid, cav = cfg["head"], cfg["lid"], cfg["cavity"]
    wt = float(cfg["scoop"]["wall_thickness_mm"]) * MM
    wh = float(cfg["scoop"]["wall_height_mm"]) * MM
    a, t, c, L, hw = g.alpha, g.tip, g.clearance, g.ramp_len, g.half_width
    # 傾斜板（薄い箱）。下前の角を P0 = (0, c) に置き、斜面方向 u = (cos a, sin a)、上向きの法線 n = (−sin a, cos a)
    xc = (L / 2) * math.cos(a) - (t / 2) * math.sin(a)
    zc = c + (L / 2) * math.sin(a) + (t / 2) * math.cos(a)
    cw, cl = g.cav_half_width, g.cav_len
    tf, wallc = cav["floor_plate_mm"] * MM, g.wall_t
    zf = g.end_z - tf / 2
    hh = (g.cav_height + tf) / 2
    zwall = g.end_z - tf + hh
    # ヒンジ = 空間の奥の上端。フタは −x 方向へ伸びる（θ = 0 で水平・空間を覆う、θ > 0 で前縁が上がる）
    tl = lid["thickness_mm"] * MM
    common = 'contype="0" conaffinity="0" rgba="0.55 0.7 0.9 1"'
    assets, geoms = [], []
    face = (front_face or cfg["scoop"].get("front_face", "perpendicular"))
    if face == "vertical":
        # 前面を床に垂直に切った傾斜板（下の前の角が先頭に立つ）。板に垂直に切った場合は上の角が t·sin(a) だけ前に出る
        n_up = (-math.sin(a), math.cos(a))
        p1 = (0.0, c)
        p4 = (L * math.cos(a), c + L * math.sin(a))
        p3 = (p4[0] + t * n_up[0], p4[1] + t * n_up[1])
        p2 = (0.0, c + t / math.cos(a))
        nm, vtx = _prism("plate_mesh", [p1, p2, p3, p4], -hw, hw)
        assets.append(f'<mesh name="{nm}" vertex="{vtx}"/>')
        plate_geom = f'<geom name="plate" type="mesh" mesh="{nm}" {common}/>'
    else:
        plate_geom = (f'<geom name="plate" type="box" size="{_f(L / 2)} {_f(hw)} {_f(t / 2)}" pos="{_f(xc)} 0 {_f(zc)}" '
                      f'euler="0 {-shape.alpha_deg:.6g} 0" {common}/>')
    if shape.side_wall:
        d_x = -t * math.sin(a)
        poly = [(d_x, 0.0), (g.end_x, 0.0), (g.end_x, g.end_z + wh), (d_x, c + t * math.cos(a) + wh)]
        for nm, sgn in (("ramp_wl", 1.0), ("ramp_wr", -1.0)):
            y0, y1 = (sgn * hw, sgn * (hw + wt)) if sgn > 0 else (sgn * (hw + wt), sgn * hw)
            n, vtx = _prism(nm, poly, y0, y1)
            assets.append(f'<mesh name="{n}" vertex="{vtx}"/>')
            geoms.append(f'<geom name="{nm}" type="mesh" mesh="{n}" {common}/>')
    ob_geom, half_h, radius, ob_asset = _object_geom(ob, rim_fillet_mm)
    if ob_asset:
        assets.append(ob_asset)
    rolling = float(ob.get("rolling_friction_m", 1.0e-4)) * rolling_scale
    z0 = half_h + 1.0e-5
    pairs = []
    targets = [("floor", mu_floor), ("plate", mu_s), ("cav_floor", mu_s), ("cav_wl", mu_s), ("cav_wr", mu_s),
               ("cav_back", mu_s), ("lid_plate", mu_l)] + ([("ramp_wl", mu_s), ("ramp_wr", mu_s)] if shape.side_wall else [])
    condim = 6 if ob["shape"] == "sphere" else 3
    for gname, mu in targets:
        pairs.append(f'<pair geom1="obj_geom" geom2="{gname}" condim="{condim}" friction="{mu:.6g} {mu:.6g} 1e-4 {rolling:.6g} {rolling:.6g}" '
                     f'solref="{solref}" solimp="{solimp}"/>')
    force = float(cfg["motion"]["push_force_limit_n"])
    lid_tq = float(lid["torque_limit_nm"])
    lid_open = math.radians(float(lid["open_deg"]))
    lid_mass = float(lid["mass_g"]) / 1000.0
    # フタの長さ ell。既定は空間の長さ。lid_front_ahead_mm を渡すと、開き（open_deg）のときフタの前縁が
    # 傾斜板の先端より その分だけ前（−x）に出る長さ: ell = (ヒンジの x + ahead) / cos(open)
    ell = cl if lid_front_ahead_mm is None else (g.hinge_x + lid_front_ahead_mm * MM) / math.cos(lid_open)
    lid_i = max(lid_mass * ell * ell / 3.0, 1e-9)
    # フタの種類。既定 = 奥ヒンジ（User の最初の指定。幾何的に物を奥へ押せないことを確認済み）。
    # beak = (ヒンジの床からの高さ mm, 腕の長さ mm) を渡すと「巻き込みくちばし」（User 決定 2026-09-29）:
    #   ヒンジ = ランプ先端（x = 0）の真上。関節角 q = 0 で腕が前（−x）へ水平、q = 90° で真下、q = 180° で奥（+x）向き水平（閉）。
    if beak is None:
        lid_pos, lid_axis, lid_rng = (g.hinge_x, g.hinge_z), "0 1 0", f"0 {lid['open_deg']:.6g}"
        lid_hw, lid_gz, q_open, q_closed = cw + wallc, tl / 2, lid_open, 0.0
    else:
        ell = beak[1] * MM
        lid_i = max(lid_mass * ell * ell / 3.0, 1e-9)
        lid_pos, lid_axis, lid_rng = (0.0, beak[0] * MM), "0 -1 0", "0 180"
        lid_hw, lid_gz, q_open, q_closed = cw, 0.0, 0.0, math.pi   # 腕の幅 = 空間の内幅（ASSUMED）。ヒンジは物の通り道の外の側板で支える想定（モデルには入れない）
    head_m = float(hd["mass_g"]) / 1000.0
    cone = "elliptic" if sim.get("cone", "elliptic") == "elliptic" else "pyramidal"
    xml = f"""<mujoco model="scoop_{shape.key}_{obj_name}_{floor_name}">
  <!-- 生成物。手で編集しない（simulation/scoop/model.py が config/scoop.yaml から作る）。摩擦・質量・押しつけ力は ASSUMED -->
  <compiler angle="degree"/>
  <option timestep="{dt:.6g}" gravity="0 0 -9.81" integrator="implicitfast" cone="{cone}" impratio="1">
    <flag multiccd="{'enable' if sim.get('multiccd', True) else 'disable'}"/>
  </option>
  <asset>
    {chr(10).join('    ' + s for s in assets).strip()}
  </asset>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1" contype="0" conaffinity="0" rgba="0.8 0.75 0.6 1"/>
    <body name="head" pos="0 0 0">
      <joint name="slide" type="slide" axis="1 0 0" damping="0"/>
      <inertial pos="{_f(g.end_x + cl / 2)} 0 {_f(g.end_z)}" mass="{head_m:.6g}" diaginertia="2e-5 2e-5 2e-5"/>
      {plate_geom}
      <geom name="cav_floor" type="box" size="{_f(cl / 2)} {_f(cw + wallc)} {_f(tf / 2)}" pos="{_f(g.end_x + cl / 2)} 0 {_f(zf)}" {common}/>
      <geom name="cav_wl" type="box" size="{_f(cl / 2)} {_f(wallc / 2)} {_f(hh)}" pos="{_f(g.end_x + cl / 2)} {_f(cw + wallc / 2)} {_f(zwall)}" {common}/>
      <geom name="cav_wr" type="box" size="{_f(cl / 2)} {_f(wallc / 2)} {_f(hh)}" pos="{_f(g.end_x + cl / 2)} {_f(-cw - wallc / 2)} {_f(zwall)}" {common}/>
      <geom name="cav_back" type="box" size="{_f(wallc / 2)} {_f(cw + wallc)} {_f(hh)}" pos="{_f(g.end_x + cl + wallc / 2)} 0 {_f(zwall)}" {common}/>
      {chr(10).join('      ' + s for s in geoms).strip()}
      <body name="lid" pos="{_f(lid_pos[0])} 0 {_f(lid_pos[1])}">
        <joint name="hinge" type="hinge" axis="{lid_axis}" range="{lid_rng}" damping="0" armature="{lid_i:.6g}"/>
        <inertial pos="{_f(-ell / 2)} 0 0" mass="{lid_mass:.6g}" diaginertia="{lid_i:.6g} {lid_i:.6g} {lid_i:.6g}"/>
        <geom name="lid_plate" type="box" size="{_f(ell / 2)} {_f(lid_hw)} {_f(tl / 2)}" pos="{_f(-ell / 2)} 0 {_f(lid_gz)}" {common}/>
      </body>
    </body>
    <body name="obj" pos="-0.1 0 {_f(z0)}">
      <freejoint name="obj_free"/>
      {ob_geom}
    </body>
  </worldbody>
  <contact>
    {chr(10).join('    ' + p for p in pairs).strip()}
  </contact>
  <actuator>
    <velocity name="vel" joint="slide" kv="50" forcerange="{-force:.6g} {force:.6g}"/>
    <position name="lid_act" joint="hinge" kp="{lid_tq / math.radians(2.0):.6g}" kv="{2 * math.sqrt(lid_tq / math.radians(2.0) * lid_i):.6g}" forcerange="{-lid_tq:.6g} {lid_tq:.6g}"/>
  </actuator>
</mujoco>
"""
    return ModelSpec(xml=xml, geom=g, obj_half_height=half_h, obj_radius=radius, lid_length=ell,
                     lid_open_rad=q_open, lid_closed_rad=q_closed)


def rounded_disc_vertices(radius: float, half_h: float, fillet: float, n_ang: int = 32, n_arc: int = 5) -> list[float]:
    """縁を半径 fillet で丸めた円盤の頂点（凸包になる）。円柱の縁が直角だと、先端が縁の下へ入る向きの力が出ない。"""
    f = min(fillet, half_h * 0.999, radius * 0.5)
    prof = []
    for sgn in (1.0, -1.0):
        for k in range(n_arc + 1):
            phi = (math.pi / 2) * k / n_arc
            prof.append((radius - f + f * math.sin(phi), sgn * (half_h - f + f * math.cos(phi))))
    v: list[float] = []
    for i in range(n_ang):
        th = 2 * math.pi * i / n_ang
        for rho, z in prof:
            v += [rho * math.cos(th), rho * math.sin(th), z]
    return v


def _object_geom(ob: dict[str, Any], rim_fillet_mm: float | None = None) -> tuple[str, float, float, str]:
    """物の geom の文言、半分の高さ・代表半径（m）、必要ならメッシュのアセット。
    円柱の縁の丸み（rim_fillet_mm。config の objects[].rim_fillet_mm、既定 0 = 直角）が指定されたら、丸めた円盤のメッシュにする。"""
    m = float(ob["mass_g"]) / 1000.0
    tag = f'name="obj_geom" group="{OBJECT_GROUP}" contype="0" conaffinity="0" mass="{m:.6g}" rgba="0.9 0.3 0.3 1"'
    if ob["shape"] == "cylinder":
        r, h = ob["diameter_mm"] * MM / 2, ob["height_mm"] * MM / 2
        fillet = float(ob.get("rim_fillet_mm", 0.0) if rim_fillet_mm is None else rim_fillet_mm) * MM
        if fillet > 0.0:
            vtx = " ".join(_f(x) for x in rounded_disc_vertices(r, h, fillet))
            return f'<geom type="mesh" mesh="obj_mesh" {tag}/>', h, r, f'<mesh name="obj_mesh" vertex="{vtx}"/>'
        return f'<geom type="cylinder" size="{_f(r)} {_f(h)}" {tag}/>', h, r, ""
    if ob["shape"] == "sphere":
        r = ob["diameter_mm"] * MM / 2
        return f'<geom type="sphere" size="{_f(r)}" {tag}/>', r, r, ""
    e = ob["edge_mm"] * MM / 2
    return f'<geom type="box" size="{_f(e)} {_f(e)} {_f(e)}" {tag}/>', e, e * math.sqrt(2), ""
