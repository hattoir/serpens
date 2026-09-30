"""形と機構（A〜G）の 1 エピソード。**MUJOCO_SIM。実物ではない。**

動き: 頭（スライダー）が一定速度で −x へ進む（D は動かない）→ 機構が働く → 物の中心が空間の奥へ入ったらゲートを閉じる → 閉じ終わりから hold_s（2 秒）後に判定。
指標（前回と同じ定義）:
  乗る  … 物の頭側（+x 側）の縁が床から 1mm 以上持ち上がった瞬間が一度でもあった（床が空間の床の案では起きない）
  入る  … 物の中心が空間の範囲に入った瞬間が一度でもあった
  保持（success）… ゲートが閉じ終わり、そこから 2 秒後まで空間の中にあり、逃げず、**飛ばされていない**
  飛ばされた … 物の速さが一度でも閾値（350 mm/s、機構が速い案は 1.5 × 機構の最大速度）を超えた。飛ばされて入った分は knocked_in として別集計（保持に数えない）
  逃げた距離 … 判定時の物の中心の、空間の範囲からの距離 [mm]
"""
from __future__ import annotations

import importlib
import math
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Any

import numpy as np

from simulation.scoop.forms.common import Form, assemble
from simulation.scoop.model import MM
from simulation.scoop.runner import RIDE_LIFT_M, _edge_heights, _object_points, _smoothstep, _yaw_quat

try:
    import mujoco
except ImportError:  # pragma: no cover
    mujoco = None

SOURCE = "MUJOCO_SIM"
OUTCOMES = ("success", "knocked_in", "escaped", "pinched", "pushed_ahead", "not_entered")
REGISTRY = {"hood": ("passive", "Passive"), "sweeper": ("sweeper", "Sweeper"), "belt": ("belt", "Belt"),
            "brush": ("brush", "Brush"), "cup": ("cup", "Cup"), "hook": ("hook", "Hook")}


def make_form(cfg: dict[str, Any], name: str, params: dict[str, Any], obj: str, floor: str) -> Form:
    mod, cls = REGISTRY[name]
    return getattr(importlib.import_module(f"simulation.scoop.forms.{mod}"), cls)(cfg, params, obj, floor)


@dataclass
class FormResult:
    source: str
    form: str
    outcome: str
    success: bool                   # 保持（飛ばされていない）
    knocked_in: bool                # 飛ばされて入り、2 秒後も中にいた（保持に数えない）
    launched: bool
    rode_ever: bool
    entered_ever: bool
    inside_at_close: bool
    held: bool
    closed: bool
    mech_fired: bool
    gate_fired: bool
    escape_mm: float
    forward_disp_mm: float
    moved_mm: float                 # 物が最初の位置から動いた距離（平面）
    obj_speed_max_mm_s: float
    head_edge_lift_max_mm: float
    wall_contact: bool              # 物の中心が口の面に届く前に、壁の前の縁・漏斗・段差に触れた（またいで入ったか、当たって誘導 / 押されたかの分類に使う）
    fire_rel_x_mm: float            # ゲートを閉じ始めた（合図の）とき、物の中心の口の面からの位置
    fire_y_mm: float
    rel_x_mm: float
    y_mm: float
    t_end_s: float


_CACHE: dict[tuple, tuple[Any, Any, Form]] = {}


def _prepare(cfg: dict[str, Any], name: str, params: dict[str, Any], obj: str, floor: str, numerics: dict[str, Any] | None):
    key = (id(cfg), name, tuple(sorted(params.items(), key=str)), obj, floor, tuple(sorted((numerics or {}).items(), key=str)))
    if key in _CACHE:
        return _CACHE[key]
    if numerics:
        cfg = {**cfg, "sim": {**cfg["sim"], **numerics}}
    form = make_form(cfg, name, params, obj, floor)
    parts = form.build()
    xml = assemble(form, parts)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    form.bind(model, mujoco)
    if len(_CACHE) > 48:
        _CACHE.clear()
    _CACHE[key] = (model, data, form)
    return _CACHE[key]


def run_form_episode(cfg: dict[str, Any], name: str, params: dict[str, Any], obj: str, floor: str, speed_mm_s: float,
                     offset_mm: float, seed: int, *, stop_on_trigger: bool = True, numerics: dict[str, Any] | None = None,
                     trace: list | None = None) -> FormResult:
    if mujoco is None:  # pragma: no cover
        raise RuntimeError("MuJoCo が無い（任意依存）")
    model, data, form = _prepare(cfg, name, params, obj, floor, numerics)
    fc = cfg["forms"]
    rng = np.random.default_rng(seed)
    mj = mujoco
    n = lambda kind, nm: mj.mj_name2id(model, kind, nm)
    j_slide, j_obj = n(mj.mjtObj.mjOBJ_JOINT, "slide"), n(mj.mjtObj.mjOBJ_JOINT, "obj_free")
    slide_adr, obj_qadr, obj_dof = int(model.jnt_qposadr[j_slide]), int(model.jnt_qposadr[j_obj]), int(model.jnt_dofadr[j_obj])
    act_vel = n(mj.mjtObj.mjOBJ_ACTUATOR, "vel")
    obj_body = n(mj.mjtObj.mjOBJ_BODY, "obj")
    ids = SimpleNamespace(obj_geom=n(mj.mjtObj.mjOBJ_GEOM, "obj_geom"))
    pts_local = _object_points(model, ids.obj_geom)
    D, W2, Hc, zb = form.depth, form.half_w, form.height, form.zb
    rb = form.r_bound

    mj.mj_resetData(model, data)
    if form.has_gate:
        gj = form.gate_joint
        model.jnt_range[gj] = (0.0, Hc + 0.004)
        gate_adr = int(model.jnt_qposadr[gj])
        gate_open = Hc + 0.003
        data.qpos[gate_adr] = gate_open
        data.ctrl[form.gate_act] = gate_open
    form.reset(model, data)
    # 物の初期位置
    if form.static_placement:
        ang = rng.uniform(0.0, 2 * math.pi)
        ox0, oy0 = offset_mm * MM * math.cos(ang), offset_mm * MM * math.sin(ang)
    else:
        d0 = rng.uniform(*cfg["placement"]["distance_ahead_mm"]) * MM
        ox0 = -(form.front_extent + d0)
        oy0 = offset_mm * MM * (1.0 if rng.random() < 0.5 else -1.0)
    yaw = rng.uniform(0.0, 2 * math.pi)
    data.qpos[obj_qadr:obj_qadr + 3] = [ox0, oy0, form.obj_half_h + 1.0e-5]
    data.qpos[obj_qadr + 3:obj_qadr + 7] = _yaw_quat(yaw)
    backstop = params.get("backstop")
    if backstop:
        gap = fc["backstop"]["gap_mm"] * MM
        bid = n(mj.mjtObj.mjOBJ_BODY, "stopper")
        mid = int(model.body_mocapid[bid])
        if backstop == "wall":
            data.mocap_pos[mid] = [ox0 - rb - gap - 0.005, 0.0, 0.03]
        else:
            data.mocap_pos[mid] = [ox0 - rb - gap - fc["backstop"]["leg_diameter_mm"] * MM / 2, oy0, 0.03]
    mj.mj_forward(model, data)
    for _ in range(int(0.05 / model.opt.timestep)):
        mj.mj_step(model, data)
    x_obj0 = float(data.xpos[obj_body][0])
    y_obj0 = float(data.xpos[obj_body][1])

    dt = float(model.opt.timestep)
    check = int(cfg["sim"]["check_every_steps"])
    v = speed_mm_s * MM
    ramp_up = float(cfg["motion"]["ramp_up_s"])
    hold_s = float(fc["hold_s"])
    gate_time = float(fc["gate"]["time_s"])
    margin = float(fc["trigger"]["full_inside_margin_mm"]) * MM
    launch_thr = max(float(fc["launch_speed_mm_s"]), 1.5 * form.peak_speed_mm_s) * MM
    abort_push = float(fc["abort_push_mm"]) * MM
    travel_max = (-x_obj0) + float(fc["travel_extra_mm"]) * MM
    close_tol = 0.0005
    retreat = float(params.get("retreat", 0.0)) * MM      # 閉じ終わりの後、頭がこの距離だけ後退する（ゲートの効果の切り分け。0 = 後退しない）
    retreat_x0 = None
    retreat_t = None
    wall_contact = False
    fire_rel = (float("nan"), float("nan"), float("nan"))

    t, step = 0.0, 0
    stopped = not form.head_moves
    mech_started = mech_done = False
    mech_t0 = mech_done_t = None
    gate_t0 = closed_t = None
    pinched_nogate = False
    latched = False
    inside_at_close = False
    held = True
    rode = entered = False
    lift_max = speed_max = 0.0
    t_end = None
    while True:
        mj.mj_step(model, data)
        t += dt
        step += 1
        if step % check:
            continue
        head_x = float(data.qpos[slide_adr])
        travel = -head_x
        ox, oy, oz = (float(a) for a in data.xpos[obj_body])
        rel = (ox - head_x, oy, oz)
        zf, zh = _edge_heights(model, data, ids, pts_local)
        lift_max = max(lift_max, zh)
        if zh >= RIDE_LIFT_M:
            rode = True
        speed_max = max(speed_max, float(np.linalg.norm(data.qvel[obj_dof:obj_dof + 3])))
        inside = form.inside(rel)
        if inside:
            entered = True
        if not wall_contact and rel[0] < 0.0 and form.wall_ids:
            for k in range(data.ncon):
                c = data.contact[k]
                if (c.geom1 == ids.obj_geom and c.geom2 in form.wall_ids) or (c.geom2 == ids.obj_geom and c.geom1 in form.wall_ids):
                    wall_contact = True
                    break
        if trace is not None:
            trace.append((t, travel / MM, rel[0] / MM, rel[1] / MM, rel[2] / MM, speed_max / MM))
        data.ctrl[act_vel] = 0.0 if stopped else -v * min(1.0, t / ramp_up)
        form.always(model, data, t)
        # 離散的な機構
        if not mech_started and form.mech_fire(rel, t):
            mech_started, mech_t0 = True, t
            if stop_on_trigger or form.stops_head_on_mech:
                stopped = True
        if mech_started and not mech_done:
            if form.mech_step(model, data, t - mech_t0, rel):
                mech_done, mech_done_t = True, t
        # ゲート
        if form.has_gate and gate_t0 is None:
            full = form.fully_inside(rel, margin)
            late = form.gate_delay_s is not None and mech_done and (t - mech_done_t) >= form.gate_delay_s
            if full or late:
                gate_t0 = t
                stopped = True
                fire_rel = rel
        # ゲートなしで頭が動く案: 同じ合図で止まり、ゲートが閉じ終わるのと同じ時間だけ待って「閉じ終わり」とする（時間を揃える）
        if (not form.has_gate) and form.head_moves and not form.discrete and gate_t0 is None and form.fully_inside(rel, margin):
            gate_t0 = t
            stopped = True
            fire_rel = rel
        if (not form.has_gate) and gate_t0 is not None and closed_t is None and form.head_moves and t >= gate_t0 + gate_time:
            closed_t = t
            inside_at_close = inside
        if form.has_gate and gate_t0 is not None:
            frac = _smoothstep((t - gate_t0) / gate_time)
            data.ctrl[form.gate_act] = (Hc + 0.003) * (1.0 - frac)
            q = float(data.qpos[gate_adr])
            if closed_t is None and q <= close_tol:
                closed_t = t
                inside_at_close = inside
                model.jnt_range[form.gate_joint] = (0.0, close_tol + 0.0001)     # ラッチ: 実際に閉じてから掛ける
            elif closed_t is None and t >= gate_t0 + gate_time + 1.5:
                t_end = t                                                         # 物を挟んで閉じ切れない
                break
        if (not form.has_gate) and mech_done and closed_t is None:
            if form.closed_ok(model, data):
                closed_t = t
                inside_at_close = inside
            elif t >= mech_done_t + 1.0:
                t_end = t                                    # 縁が物に乗って下りきれない（挟まった）
                pinched_nogate = True
                break
        if closed_t is not None:
            held = held and inside
            if retreat > 0.0 and retreat_t is None:                  # 閉じ終わりの後、頭を後退させる
                if retreat_x0 is None:
                    retreat_x0 = head_x
                data.ctrl[act_vel] = +v
                if head_x - retreat_x0 >= retreat:
                    retreat_t = t
                    data.ctrl[act_vel] = 0.0
            elif retreat > 0.0:
                data.ctrl[act_vel] = 0.0
            hold_from = closed_t if retreat <= 0.0 else retreat_t
            if hold_from is not None and t >= hold_from + hold_s:
                t_end = t
                break
        # 打ち切り
        if not mech_started and form.head_moves and (x_obj0 - ox) > abort_push:
            t_end = t
            break
        if form.head_moves and not stopped and travel >= travel_max:
            t_end = t
            break
        if t > 120.0:
            t_end = t
            break
    head_x = float(data.qpos[slide_adr])
    ox, oy, oz = (float(a) for a in data.xpos[obj_body])
    rel = (ox - head_x, oy, oz)
    inside_end = form.inside(rel)
    launched = speed_max > launch_thr
    closed = closed_t is not None
    if closed and inside_at_close and held and inside_end:
        outcome = "knocked_in" if launched else "success"
    elif closed and inside_at_close:
        outcome = "escaped"
    elif (gate_t0 is not None or pinched_nogate) and not closed:
        outcome = "pinched"
    elif rel[0] < 0.0:
        outcome = "pushed_ahead"
    else:
        outcome = "not_entered"
    return FormResult(source=SOURCE, form=name, outcome=outcome, success=outcome == "success", knocked_in=outcome == "knocked_in",
                      launched=launched, rode_ever=rode, entered_ever=entered, inside_at_close=inside_at_close, held=held, closed=closed,
                      mech_fired=mech_started, gate_fired=gate_t0 is not None, escape_mm=form.escape(rel) / MM,
                      forward_disp_mm=max(0.0, x_obj0 - ox) / MM, moved_mm=math.hypot(ox - x_obj0, oy - y_obj0) / MM, obj_speed_max_mm_s=speed_max / MM,
                      head_edge_lift_max_mm=lift_max / MM, wall_contact=wall_contact, fire_rel_x_mm=fire_rel[0] / MM, fire_y_mm=fire_rel[1] / MM, rel_x_mm=rel[0] / MM, y_mm=rel[1] / MM, t_end_s=t_end or t)
