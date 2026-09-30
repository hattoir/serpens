"""スコップ＋フタの 1 エピソードを走らせて、閉じ込め成功と失敗の種類を判定する（**すべて MUJOCO_SIM。実物ではない**）。

動き: 頭（スライダー）が一定速度で −x へ進む → トリガー条件が成り立ったらフタを閉じ始める（遅れを足せる）→ 閉じ終わり + 待ち時間で判定。
成功 = フタが閉じ終わった（開き ≤ 許容）時点で、物の中心が閉じ込め空間（長さ × 幅 × 高さ）の内側にある。
失敗の分類（軌跡から自動）:
  pushed_ahead … 前に押して逃げた（先端より前）
  lateral      … 横に逃げた
  under        … スコップの下に潜った（傾斜板の下面より下。**数値の貫通の疑いもある**）
  pinched      … フタに挟まった（空間の入口にあり、フタが閉じ切らない）
  on_ramp      … ランプ上・空間の入口で止まった / 滑り落ちた（奥まで入らずにフタが閉じた、またはトリガーが無かった）
  other        … 上のどれでもない（頭の後ろへ抜けた、など）
トリガー: ideal_center_depth / latency:MS / contact_advance / tof（config の trigger）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from simulation.scoop.model import OBJECT_GROUP, MM, Geometry, ModelSpec, Shape, build_mjcf, load_config

SOURCE = "MUJOCO_SIM"
OUTCOMES = ("success", "pushed_ahead", "lateral", "under", "pinched", "on_ramp", "other")

try:
    import mujoco
except ImportError:  # pragma: no cover - 任意依存
    mujoco = None


@dataclass
class EpisodeResult:
    source: str
    outcome: str
    success: bool
    triggered: bool
    lid_angle_end_deg: float
    rode_ever: bool                 # 「乗る」（User 定義 2026-09-29）: 物の前縁が床から 1mm 以上上がった瞬間が一度でもあった
    rode_end: bool                  # 判定時にも前縁が 1mm 以上上がっている
    front_lift_max_mm: float        # 前縁の床からの高さの最大値
    obj_speed_max_mm_s: float       # 物の並進速度の最大値。フタの先端速度（≦ 約 175 mm/s）+ 頭の速度の 2 倍を超えたら、はじき飛ばされた疑い
    on_head_ever: bool              # （旧定義）物の中心が先端より後ろで床から離れた。頭の上に完全に載った
    entered_ever: bool              # 「入る」: 途中で一度でも、物の中心が空間（ランプ終端より奥）の内側に載った
    ride_max_rel_mm: float          # 「乗る」が成り立っていた間の、先端から物の中心までの最大距離（負 = 先端より前）
    rel_x_mm: float                 # 判定時の物の先端からの位置（負 = 先端より前）
    y_mm: float
    forward_disp_mm: float          # 物が前（−x）へ動かされた総量（押して逃げた距離の目安）
    ahead_of_tip_mm: float          # 先端より前に出ている量（>0 のとき）
    trigger_travel_mm: float | None
    t_end_s: float
    numerics: dict[str, Any] = field(default_factory=dict)


@dataclass
class _Ids:
    slide_adr: int
    slide_dof: int
    obj_qadr: int
    obj_body: int
    obj_dof: int
    obj_geom: int
    hinge_adr: int
    act_vel: int
    act_lid: int
    plate_geom: int
    ramp_geoms: tuple[int, ...]


_CACHE: dict[tuple, tuple[Any, Any, ModelSpec, _Ids]] = {}


def _prepare(cfg: dict[str, Any], shape: Shape, obj: str, floor: str, clearance_mm: float | None, scoop_mu: float | None,
             lid_mu: float | None, rolling_scale: float, numerics: dict[str, Any] | None, rim_fillet_mm: float | None = None,
             front_face: str | None = None, lid_front_ahead_mm: float | None = None,
             beak: tuple[float, float] | None = None):
    key = (id(cfg), shape, obj, floor, clearance_mm, scoop_mu, lid_mu, rolling_scale, rim_fillet_mm, front_face, lid_front_ahead_mm, beak,
           tuple(sorted((numerics or {}).items(), key=str)))
    if key in _CACHE:
        return _CACHE[key]
    spec = build_mjcf(cfg, shape, obj, floor, clearance_mm=clearance_mm, scoop_mu=scoop_mu, lid_mu=lid_mu,
                      rolling_scale=rolling_scale, numerics=numerics, rim_fillet_mm=rim_fillet_mm, front_face=front_face,
                      lid_front_ahead_mm=lid_front_ahead_mm, beak=beak)
    model = mujoco.MjModel.from_xml_string(spec.xml)
    data = mujoco.MjData(model)
    n = lambda kind, name: mujoco.mj_name2id(model, kind, name)
    j = n(mujoco.mjtObj.mjOBJ_JOINT, "slide")
    o = n(mujoco.mjtObj.mjOBJ_JOINT, "obj_free")
    ids = _Ids(slide_adr=int(model.jnt_qposadr[j]), slide_dof=int(model.jnt_dofadr[j]), obj_qadr=int(model.jnt_qposadr[o]),
               obj_body=n(mujoco.mjtObj.mjOBJ_BODY, "obj"), obj_dof=int(model.jnt_dofadr[o]), obj_geom=n(mujoco.mjtObj.mjOBJ_GEOM, "obj_geom"),
               hinge_adr=int(model.jnt_qposadr[n(mujoco.mjtObj.mjOBJ_JOINT, "hinge")]),
               act_vel=n(mujoco.mjtObj.mjOBJ_ACTUATOR, "vel"), act_lid=n(mujoco.mjtObj.mjOBJ_ACTUATOR, "lid_act"),
               plate_geom=n(mujoco.mjtObj.mjOBJ_GEOM, "plate"),
               ramp_geoms=tuple(g for g in (n(mujoco.mjtObj.mjOBJ_GEOM, "ramp_wl"), n(mujoco.mjtObj.mjOBJ_GEOM, "ramp_wr")) if g >= 0))
    if len(_CACHE) > 64:
        _CACHE.clear()
    _CACHE[key] = (model, data, spec, ids)
    return _CACHE[key]


def _yaw_quat(yaw: float, tilt_about_y_deg: float = 0.0) -> np.ndarray:
    """z 軸まわり yaw、そのあと y 軸まわり tilt。"""
    qz = np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])
    th = math.radians(tilt_about_y_deg)
    qy = np.array([math.cos(th / 2), 0.0, math.sin(th / 2), 0.0])
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, qy, qz)
    return out


def _tof_dirs(fov_half_deg: float) -> np.ndarray:
    """前（−x）向きの視野（縦 7 × 横 3 本）。"""
    dirs = []
    for e in np.linspace(-fov_half_deg, fov_half_deg, 7):
        for az in np.linspace(-fov_half_deg, fov_half_deg, 3):
            ee, aa = math.radians(e), math.radians(az)
            dirs.append([-math.cos(ee) * math.cos(aa), math.cos(ee) * math.sin(aa), math.sin(ee)])
    return np.array(dirs)


RIDE_LIFT_M = 1.0e-3      # 「乗る」= 物の前縁が床から 1mm 以上上がった（User 定義 2026-09-29）


def _object_points(model: Any, geom_id: int) -> np.ndarray | None:
    """物の表面の代表点（物の座標系）。球は None（前縁 = 最下点 = 中心 − 半径）。"""
    gt = int(model.geom_type[geom_id])
    sz = model.geom_size[geom_id]
    if gt == int(mujoco.mjtGeom.mjGEOM_SPHERE):
        return None
    if gt == int(mujoco.mjtGeom.mjGEOM_BOX):
        return np.array([[sx * sz[0], sy * sz[1], sz_ * sz[2]] for sx in (-1, 1) for sy in (-1, 1) for sz_ in (-1, 1)], dtype=float)
    if gt == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
        th = np.linspace(0.0, 2 * math.pi, 48, endpoint=False)
        return np.array([[sz[0] * math.cos(a), sz[0] * math.sin(a), s * sz[1]] for a in th for s in (-1, 1)], dtype=float)
    mid = int(model.geom_dataid[geom_id])
    a0, n = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid])
    return np.array(model.mesh_vert[a0:a0 + n], dtype=float)


def _front_edge_z(model: Any, data: Any, ids: "_Ids", pts: np.ndarray | None) -> float:
    """物の前縁（中心より前 = −x 側の表面）の、床からの最小の高さ。"""
    pos = data.geom_xpos[ids.obj_geom]
    if pts is None:
        return float(pos[2] - model.geom_size[ids.obj_geom][0])
    w = pos + pts @ data.geom_xmat[ids.obj_geom].reshape(3, 3).T
    front = w[w[:, 0] <= pos[0]]
    return float(front[:, 2].min()) if len(front) else float(w[:, 2].min())


def _smoothstep(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


def run_episode(cfg: dict[str, Any], shape: Shape, obj: str, floor: str, speed_mm_s: float, y_offset_mm: float, seed: int, *,
                trigger: str = "ideal", clearance_mm: float | None = None, scoop_mu: float | None = None,
                lid_mu: float | None = None, rolling_scale: float = 1.0, numerics: dict[str, Any] | None = None,
                initial: dict[str, float] | None = None, rim_fillet_mm: float | None = None,
                front_face: str | None = None, lid_front_ahead_mm: float | None = None,
                close_time_s: float | None = None, beak: tuple[float, float] | None = None) -> EpisodeResult:
    """1 回走らせる。`trigger` は "ideal" / "latency:50" / "contact" / "tof"。
    `initial` を渡すと初期位置・向きを固定する（{"d0_mm", "y_mm", "yaw"}。収束確認・単体テスト用）。"""
    if mujoco is None:  # pragma: no cover
        raise RuntimeError("MuJoCo が無い（任意依存。requirements-sim3d.txt）")
    model, data, spec, ids = _prepare(cfg, shape, obj, floor, clearance_mm, scoop_mu, lid_mu, rolling_scale, numerics, rim_fillet_mm, front_face,
                                     lid_front_ahead_mm, beak)
    g = spec.geom
    rng = np.random.default_rng(seed)
    pl = cfg["placement"]
    if initial is None:
        d0 = rng.uniform(*pl["distance_ahead_mm"]) * MM
        y0 = (y_offset_mm * MM) * (1.0 if rng.random() < 0.5 else -1.0)
        yaw = rng.uniform(0.0, 2 * math.pi) if pl["yaw_random"] else 0.0
    else:
        d0, y0, yaw = initial["d0_mm"] * MM, initial["y_mm"] * MM, float(initial.get("yaw", 0.0))
    mujoco.mj_resetData(model, data)
    data.qpos[ids.slide_adr] = 0.0
    data.qpos[ids.hinge_adr] = spec.lid_open_rad
    q = ids.obj_qadr
    data.qpos[q:q + 3] = [-d0, y0, spec.obj_half_height + 1.0e-5]
    data.qpos[q + 3:q + 7] = _yaw_quat(yaw)
    data.ctrl[ids.act_lid] = spec.lid_open_rad
    mujoco.mj_forward(model, data)
    # 物を床に落ち着かせる（頭は止めたまま）
    for _ in range(int(0.05 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    x_obj0 = float(data.xpos[ids.obj_body][0])

    dt = float(model.opt.timestep)
    check = int(cfg["sim"]["check_every_steps"])
    v = speed_mm_s * MM
    ramp_up = float(cfg["motion"]["ramp_up_s"])
    travel_max = float(cfg["motion"]["travel_max_mm"]) * MM
    close_t = float(cfg["lid"]["close_time_s"] if close_time_s is None else close_time_s)
    settle = float(cfg["motion"]["settle_after_close_s"])
    open_rad = spec.lid_open_rad
    tcfg = cfg["trigger"]
    latency = 0.0
    kind = trigger
    contact_adv_mm = float(cfg["trigger"]["contact_advance_mm"])
    if trigger.startswith("latency:"):
        kind, latency = "ideal", float(trigger.split(":")[1]) / 1000.0
    elif trigger.startswith("contact:"):
        kind = "contact"
        contact_adv_mm = float(trigger.split(":")[1])
    elif trigger == "ideal":
        latency = 0.0
    tof_dirs = _tof_dirs(float(tcfg["tof"]["fov_half_deg"])) if kind == "tof" else None
    tof_period = 1.0 / float(tcfg["tof"]["rate_hz"])
    geomgroup = np.zeros(6, dtype=np.uint8)
    geomgroup[OBJECT_GROUP] = 1
    geomid = np.zeros(1, dtype=np.int32)
    depth_trig = float(tcfg["ideal_center_depth_mm"]) * MM
    scoop_geoms = {ids.plate_geom, *ids.ramp_geoms}

    t = 0.0
    step = 0
    trig_t: float | None = None           # トリガーが成り立った時刻
    close_start: float | None = None
    trig_travel: float | None = None
    detect_travel: float | None = None    # contact / tof: 最初に検出した時点の前進距離
    next_tof = 0.0
    t_end = None
    rode_ever = entered_ever = on_head_ever = False
    ride_max = 0.0
    lift_max = 0.0
    speed_max = 0.0
    rode_now = False
    lift = spec.obj_half_height + 4.0e-4      # （旧定義）中心がこれより高ければ床から離れている
    pts_local = _object_points(model, ids.obj_geom)
    while True:
        mujoco.mj_step(model, data)
        t += dt
        step += 1
        if step % check:
            continue
        tip_x = float(data.qpos[ids.slide_adr])
        travel = -tip_x
        _ox, _oy, _oz = data.xpos[ids.obj_body]
        _rel = float(_ox) - tip_x
        _fz = _front_edge_z(model, data, ids, pts_local)
        lift_max = max(lift_max, _fz)
        speed_max = max(speed_max, float(np.linalg.norm(data.qvel[ids.obj_dof:ids.obj_dof + 3])))
        rode_now = _fz >= RIDE_LIFT_M
        if rode_now:
            rode_ever = True
            ride_max = max(ride_max, _rel)
        if _rel >= 0.0 and _oz > lift:
            on_head_ever = True
        if _rel >= g.end_x and _oz > lift and abs(_oy) <= g.cav_half_width and _oz >= g.end_z - 1.0e-3:
            entered_ever = True
        data.ctrl[ids.act_vel] = -v * min(1.0, t / ramp_up)
        if close_start is None:
            ox, oy, oz = data.xpos[ids.obj_body]
            rel = ox - tip_x
            fire = False
            if kind == "ideal":
                fire = (g.end_x + depth_trig <= rel <= g.end_x + g.cav_len and abs(oy) <= g.cav_half_width
                        and oz >= g.end_z - 1.0e-3)
            elif kind == "contact":
                if detect_travel is None:
                    for k in range(data.ncon):
                        c = data.contact[k]
                        if (c.geom1 == ids.obj_geom and c.geom2 in scoop_geoms) or (c.geom2 == ids.obj_geom and c.geom1 in scoop_geoms):
                            detect_travel = travel
                            break
                fire = detect_travel is not None and travel - detect_travel >= contact_adv_mm * MM
            elif kind == "tof":
                if detect_travel is None and t >= next_tof:
                    next_tof = t + tof_period
                    tc = tcfg["tof"]
                    pnt = np.array([tip_x + float(tc["sensor_x_mm"]) * MM, 0.0, float(tc["sensor_z_mm"]) * MM])
                    dmin = math.inf
                    for d in tof_dirs:
                        dist = mujoco.mj_ray(model, data, pnt, d, geomgroup, 1, -1, geomid)
                        if dist >= 0.0 and dist < dmin:
                            dmin = dist
                    if dmin <= float(tc["threshold_mm"]) * MM:
                        detect_travel = travel
                fire = detect_travel is not None and travel - detect_travel >= float(tcfg["tof"]["advance_mm"]) * MM
            if fire:
                trig_t = t
                trig_travel = travel
                close_start = t + latency
            elif travel >= travel_max:
                t_end = t
                break
        if close_start is not None:
            frac = _smoothstep((t - close_start) / close_t) if t >= close_start else 0.0
            data.ctrl[ids.act_lid] = open_rad + (spec.lid_closed_rad - open_rad) * frac
            if not bool(cfg["motion"]["continue_during_close"]) and t >= close_start:
                data.ctrl[ids.act_vel] = 0.0
            if t >= close_start + close_t + settle:
                t_end = t
                break
        if t > 90.0:
            t_end = t
            break
    return _judge(cfg, spec, g, data, ids, x_obj0, trig_t is not None, trig_travel, t_end or t, dict(numerics or {}),
                  rode_ever, entered_ever, ride_max / MM, rode_now, lift_max / MM, on_head_ever, speed_max / MM)


def _judge(cfg: dict[str, Any], spec: ModelSpec, g: Geometry, data: Any, ids: _Ids, x_obj0: float, triggered: bool,
           trig_travel: float | None, t_end: float, numerics: dict[str, Any], rode_ever: bool = False,
           entered_ever: bool = False, ride_max_mm: float = 0.0, rode_now: bool = False, lift_max_mm: float = 0.0,
           on_head_ever: bool = False, speed_max_mm_s: float = 0.0) -> EpisodeResult:
    tip_x = float(data.qpos[ids.slide_adr])
    ox, oy, oz = (float(x) for x in data.xpos[ids.obj_body])
    rel = ox - tip_x
    lid_deg = math.degrees(float(data.qpos[ids.hinge_adr]))
    closed = abs(lid_deg - math.degrees(spec.lid_closed_rad)) <= float(cfg["lid"]["closed_tolerance_deg"])
    x_in = g.end_x <= rel <= g.end_x + g.cav_len
    inside = x_in and abs(oy) <= g.cav_half_width and g.end_z - 1.0e-3 <= oz <= g.end_z + g.cav_height
    wall = float(cfg["scoop"]["wall_thickness_mm"]) * MM
    fwd = max(0.0, x_obj0 - ox)
    ahead = max(0.0, -rel)
    if inside and closed:
        outcome = "success"
    elif rel < 0.0:
        outcome = "lateral" if abs(oy) > g.half_width + wall else "pushed_ahead"
    elif rel > g.end_x + g.cav_len + 0.02:
        outcome = "other"
    elif abs(oy) > max(g.half_width, g.cav_half_width) + 2 * wall + 1.0e-3:
        outcome = "lateral"
    elif 0.0 <= rel < g.end_x and oz < g.ramp_bottom(rel) - 1.0e-4:
        outcome = "under"
    elif x_in and not closed and inside:
        outcome = "pinched"
    elif (g.end_x - 0.005 <= rel <= g.end_x + g.cav_len) and not closed:
        outcome = "pinched"
    else:
        outcome = "on_ramp"
    rode_end = rode_now
    return EpisodeResult(source=SOURCE, outcome=outcome, success=outcome == "success", triggered=triggered, lid_angle_end_deg=lid_deg,
                         rode_ever=rode_ever, rode_end=rode_end, front_lift_max_mm=lift_max_mm, obj_speed_max_mm_s=speed_max_mm_s, on_head_ever=on_head_ever,
                         entered_ever=entered_ever, ride_max_rel_mm=ride_max_mm,
                         rel_x_mm=rel / MM, y_mm=oy / MM, forward_disp_mm=fwd / MM, ahead_of_tip_mm=ahead / MM,
                         trigger_travel_mm=None if trig_travel is None else trig_travel / MM, t_end_s=t_end, numerics=numerics)


def place_on_ramp_and_release(cfg: dict[str, Any], shape: Shape, obj: str, floor: str, rel_x_mm: float = 15.0,
                              seconds: float = 1.0, scoop_mu: float | None = None) -> dict[str, float]:
    """頭を止め、物を傾斜板の上に置いて離し、滑り落ちるかを見る（解析の照合: tan α ≤ μ なら保持）。返り値は rel_x の変化 [mm]。"""
    model, data, spec, ids = _prepare(cfg, shape, obj, floor, None, scoop_mu, None, 1.0, None)
    g = spec.geom
    mujoco.mj_resetData(model, data)
    data.qpos[ids.hinge_adr] = math.radians(cfg["lid"]["open_deg"])
    data.ctrl[ids.act_lid] = math.radians(cfg["lid"]["open_deg"])
    rx = rel_x_mm * MM
    n_up = np.array([-math.sin(g.alpha), 0.0, math.cos(g.alpha)])
    pos = np.array([rx, 0.0, g.ramp_top(rx)]) + n_up * (spec.obj_half_height + 1.0e-5)
    q = ids.obj_qadr
    data.qpos[q:q + 3] = pos
    data.qpos[q + 3:q + 7] = _yaw_quat(0.0, -shape.alpha_deg)
    mujoco.mj_forward(model, data)
    for _ in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)
    end = float(data.xpos[ids.obj_body][0]) - float(data.qpos[ids.slide_adr])
    return {"rel_x_start_mm": rel_x_mm, "rel_x_end_mm": end / MM, "moved_mm": (end - rx) / MM}
