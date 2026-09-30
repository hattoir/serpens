"""H2 の Hardware Gap: 床見の合成画像で、条件をずらしたときの検出の崩れ方を測る。**source は SYNTHETIC_VISION_SIM。**

実写でも実機の性能でもない。既存の描画（`serpens.floorwatch.synthetic`）と検出（`serpens.floorwatch.detect`）をそのまま使い、
条件（床の模様・毛足・ぼけ・環境光・露出・線の幅とにじみ・斜め照明の高さ・カメラの高さ/pitch/画角のずれ・隣の物）を
1 つずつ、または同時にずらして、次を数える:

  patrol  … 線なし（通常 + 斜め）で、物の近くに「物」の候補が出たか（巡回中に止まれるか）
  inspect … 線あり、物は線の上（狙いの誤差つき）。見つかったか・高さ・大きさ・metal_disc（危険物側）に回ったか
  誤報    … 物の無い床（模様・汚れ・継ぎ目）で「物」の候補が出たか

**カメラのずれ**: 描画は「本当の」カメラ（高さ・pitch・f）、検出は名目のカメラと較正済みの光の面で行う。光の面は頭に固定
（カメラ座標では同じ）なので、本当のカメラの世界座標へ写して描く。毛足に沈む・首が垂れる、を模す。

限界: 円柱の物だけ（球・錠剤の丸みは円柱で近似）、影は 2 次元の近似、ぼけは全画面で一様、床の反射は拡散のみ、
透明な物・濡れた床・日差しの縞は無い。**数値は相対比較と「どこで崩れるか」の目安にだけ使う。**
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field, replace
from typing import Any

import numpy as np

from serpens.floorwatch.detect import Candidate, MotionError, detect
from serpens.floorwatch.geometry import Camera, LightPlane
from serpens.floorwatch.synthetic import Disc, Lighting, Renderer, Scene, Seam, Stain

SOURCE = "SYNTHETIC_VISION_SIM"

# 対象（直径 mm, 高さ mm, 反射率, 鏡面, 危険物側に回るべきか）。寸法は代表値、反射率は模擬。
TARGETS: dict[str, tuple[float, float, float, bool, bool]] = {
    "coin_1yen": (20.0, 1.5, 0.85, False, False),
    "coin_10yen": (23.5, 1.5, 0.45, False, False),
    "button_cr2032": (20.0, 3.2, 0.9, True, True),
    "button_lr44": (11.6, 5.4, 0.9, True, True),
    "bead_6": (6.0, 6.0, 0.7, False, False),
    "bead_10_dark": (10.0, 10.0, 0.12, False, False),
    "magnet_disc_10x3": (10.0, 3.0, 0.6, True, True),
    "magnet_ball_5": (5.0, 5.0, 0.6, True, True),
    "pill_8": (8.0, 3.5, 0.9, False, True),
    "plastic_part_8": (8.0, 5.0, 0.35, False, False),
    "floor_colored_12": (12.0, 3.0, 0.5, False, False),
}
CRITICAL = tuple(k for k, v in TARGETS.items() if v[4])


@dataclass(frozen=True)
class Condition:
    floor: str = "wood"
    texture_contrast: float = 1.0
    floor_height_sigma_mm: float = 0.0
    blur_sigma_px: float = 0.0              # 名目の解像度（UXGA）の画素で。縮小して回すときは比で直す
    ambient_lux: float = 8.0
    ambient_drift: float = 0.0
    auto_exposure: bool = False
    shot_noise_k: float = 0.0
    line_width_mm: float = 3.0
    line_scatter_mm: float = 0.0
    raking_led_height_mm: float = 6.0
    cam_height_err_mm: float = 0.0          # 本当の高さ − 名目
    cam_pitch_err_deg: float = 0.0
    fov_err_deg: float = 0.0                # 本当の水平画角 − 名目
    aim_err_mm: float = 0.0                 # inspect で物の中心が線から横にずれる量（狙いの誤差）
    clutter: bool = False                   # 隣に高い物（おもちゃ 25mm 径・25mm 高）
    sides: bool = True
    pose_from_line: bool = True             # 検出が床の線から姿勢のずれを推定して位置を直すか（config の detect.pose_from_line を上書き）
    reaim: bool = False                     # inspect の後、候補へ線を向け直してもう 1 回撮る（mission の狙い直しを模す）
    physical_falloff: bool = False          # 照明を点光源（cos / r²）にする（通常 = レンズの脇、斜め = あご）
    flat_field: bool = False                # 照明の較正画像（白いカード、名目の姿勢で 1 回）を検出に渡す
    led_layout: str = "legacy"              # LED の置き方（LED_LAYOUTS）。legacy = 従来の 1 灯・影は 2 次元の近似


def scaled_camera(cam: Camera, scale: float) -> Camera:
    """同じ画角のまま解像度だけ変える（f も同じ比）。"""
    return replace(cam, f_px=cam.f_px * scale, cx=cam.cx * scale, cy=cam.cy * scale,
                   width_px=int(round(cam.width_px * scale)), height_px=int(round(cam.height_px * scale)))


def true_camera(cam: Camera, c: Condition) -> Camera:
    f = cam.f_px
    if c.fov_err_deg:
        half = math.atan(cam.width_px / 2 / cam.f_px)
        f = cam.width_px / 2 / math.tan(half + math.radians(c.fov_err_deg) / 2)
    return replace(cam, f_px=f, height_mm=cam.height_mm + c.cam_height_err_mm, pitch_deg=cam.pitch_deg + c.cam_pitch_err_deg)


def plane_for_true_camera(plane: LightPlane, cam_nom: Camera, cam_true: Camera) -> LightPlane:
    """頭に固定の光の面: 名目の世界座標の面 → カメラ座標 → 本当のカメラの世界座標。"""
    Rn = np.stack(cam_nom._axes)                 # 行 = カメラの x, y, z 軸（世界座標）
    Rt = np.stack(cam_true._axes)
    n_c = Rn @ plane.normal
    d_c = plane.d - float(plane.normal @ cam_nom.center)
    n_t = Rt.T @ n_c
    return LightPlane(n_t, d_c + float(n_t @ cam_true.center))


# LED の置き方（名目の世界座標 mm: x 右 / y 前 / z 上、カメラは (0, 0, 30)）。Design ENTRY-0022 の案は Design の座標
# （X 前が負、レンズ X −232）から: 斜め = 頬の下 X −212・y ±40・Z7 → (±40, −20, 7)、通常 = 口の線 X −220・y ±44.7・Z21 → (±44.7, −12, 21)
LED_LAYOUTS: dict[str, dict[str, tuple[tuple[float, float, float], ...]] | None] = {
    "legacy": None,
    "center3d": {"raking": ((0.0, 0.0, 6.0),), "normal": ((0.0, 0.0, 30.0),)},
    "design_2x2": {"raking": ((40.0, -20.0, 7.0), (-40.0, -20.0, 7.0)), "normal": ((44.7, -12.0, 21.0), (-44.7, -12.0, 21.0))},
    # Design ENTRY-D-0008 の移設案（座標は Design の X → y = −232 − X）。**CAD にはまだ入っていない**（Fusion 切断中）
    "front_corner": {"raking": ((36.0, 4.5, 6.0), (-36.0, 4.5, 6.0)), "normal": ((44.7, -12.0, 21.0), (-44.7, -12.0, 21.0))},        # 口の前の外側の角（X −236.5、|y| 36、Z 6）
    "runner_front": {"raking": ((40.0, -6.0, 6.0), (-40.0, -6.0, 6.0)), "normal": ((44.7, -12.0, 21.0), (-44.7, -12.0, 21.0))},      # 横スキッドの前端（X −226、|y| 40、Z 6）
    "runner_front_z4": {"raking": ((40.0, -6.0, 4.0), (-40.0, -6.0, 4.0)), "normal": ((44.7, -12.0, 21.0), (-44.7, -12.0, 21.0))},
    "runner_front_z9": {"raking": ((40.0, -6.0, 9.0), (-40.0, -6.0, 9.0)), "normal": ((44.7, -12.0, 21.0), (-44.7, -12.0, 21.0))},
}


def rigid_point(p: np.ndarray, cam_nom: Camera, cam_true: Camera) -> np.ndarray:
    """頭に固定の点（LED）: 名目の世界座標 → カメラ座標 → 本当のカメラの世界座標。"""
    Rn, Rt = np.stack(cam_nom._axes), np.stack(cam_true._axes)
    return Rt.T @ (Rn @ (np.asarray(p, float) - cam_nom.center)) + cam_true.center


def lighting_for(base: Lighting, c: Condition, blur_scale: float, cam_nom: Camera | None = None,
                 cam_true: Camera | None = None) -> Lighting:
    """条件の照明。cam_nom / cam_true を渡すと、LED を頭に固定として本当の姿勢へ動かす（頭が沈めばあごの LED も床に近づく）。"""
    layout = LED_LAYOUTS[c.led_layout]
    if layout is not None:
        mv = (lambda q: tuple(float(v) for v in rigid_point(np.array(q), cam_nom, cam_true))) if cam_nom is not None and cam_true is not None             else (lambda q: tuple(float(v) for v in q))
        base = replace(base, raking_leds=tuple(mv(q) for q in layout["raking"]), normal_leds=tuple(mv(q) for q in layout["normal"]))
    if cam_nom is not None and cam_true is not None:
        n = rigid_point(np.array([0.0, base.normal_led_y_mm, base.normal_led_height_mm]), cam_nom, cam_true)
        r = rigid_point(np.array([0.0, base.raking_led_y_mm, c.raking_led_height_mm]), cam_nom, cam_true)
        base = replace(base, normal_led_y_mm=float(n[1]), normal_led_height_mm=float(n[2]), raking_led_y_mm=float(r[1]))
        c = replace(c, raking_led_height_mm=float(r[2]))
    return replace(base, blur_sigma_px=c.blur_sigma_px * blur_scale, ambient_lux=c.ambient_lux, ambient_drift=c.ambient_drift,
                   auto_exposure=c.auto_exposure, shot_noise_k=c.shot_noise_k, line_width_mm=c.line_width_mm,
                   line_scatter_mm=c.line_scatter_mm, raking_led_height_mm=c.raking_led_height_mm,
                   physical_falloff=c.physical_falloff)


@dataclass
class Trial:
    target: str | None                      # None = 物の無い床
    stage_hits: dict[str, bool] = field(default_factory=dict)
    false_objects: dict[str, int] = field(default_factory=dict)
    height_est: float | None = None
    height_reason: str | None = None
    diameter_est: float | None = None
    critical_flag: bool = False              # metal_disc として危険物側に回ったか
    loc_err_mm: float | None = None          # inspect で見つかった候補の床の位置と、物の中心の距離


def _objects(cands: list[Candidate]) -> list[Candidate]:
    return [c for c in cands if c.is_object]


def true_bbox_px(cam_true: Camera, x: float, y: float, dia: float, h: float) -> tuple[float, float, float, float] | None:
    """本当のカメラで物（円柱）が写る画像の範囲 (u0, v0, u1, v1)。上下の円周の点を投影する。"""
    pts = []
    for z in (0.0, h):
        for a in np.linspace(0, 2 * np.pi, 33):
            u, v, zc = cam_true.project(np.array([x + dia / 2 * np.cos(a), y + dia / 2 * np.sin(a), z]))
            if zc > 0:
                pts.append((u, v))
    if not pts:
        return None
    p = np.array(pts)
    return float(p[:, 0].min()), float(p[:, 1].min()), float(p[:, 0].max()), float(p[:, 1].max())


def _hit_in_image(cands: list[Candidate], box: tuple[float, float, float, float] | None, pad_px: float) -> Candidate | None:
    """検出の成否は画像の上で判定する: 候補の枠が、物が本当に写っている範囲（pad_px 広げた）と重なるか。
    床の位置は検出側の名目カメラで戻すので、カメラの姿勢がずれると位置だけがずれる → それは loc_err_mm に別に記録する。"""
    if box is None:
        return None
    u0, v0, u1, v1 = box[0] - pad_px, box[1] - pad_px, box[2] + pad_px, box[3] + pad_px
    best, ba = None, 0.0
    for c in cands:
        bx, by, bw, bh = c.bbox_px
        iw, ih = min(u1, bx + bw) - max(u0, bx), min(v1, by + bh) - max(v0, by)
        if iw > 0 and ih > 0 and iw * ih > ba:
            best, ba = c, iw * ih
    return best


def _near(cands: list[Candidate], x: float, y: float, dia: float, h: float, cam_h: float) -> Candidate | None:
    """物に当たった候補か。検出は床の高さを前提に床へ戻すので、高い物ほど奥へずれて出る（上面の視差）。
    許す範囲: 横 ±(半径 + 10mm)、前後 [手前の縁 − 5mm, 奥の縁 × h_cam/(h_cam − H) + 5mm]。位置の誤差は別に記録する。"""
    r = dia / 2
    far = (y + r) * cam_h / max(cam_h - h, 1.0) + 5.0
    best, bd = None, float("inf")
    for c in cands:
        fx, fy = c.floor_xy_mm
        if abs(fx - x) <= r + 10.0 and y - r - 5.0 <= fy <= far:
            d = float(np.hypot(fx - x, fy - y))
            if d < bd:
                best, bd = c, d
    return best


def _floor_line_x(plane: LightPlane, y_mm: float) -> float:
    """床（z=0）の上で、前 y_mm の所を線が通る横位置（頬の線 = 前後に走る線）。"""
    n = plane.normal
    return float((plane.d - n[1] * y_mm) / n[0]) if abs(n[0]) > 1e-9 else float("nan")


def _with_pose_flag(cfg: dict[str, Any], on: bool) -> dict[str, Any]:
    fw = dict(cfg["floor_watch"])
    fw["detect"] = {**fw["detect"], "pose_from_line": on}
    return {**cfg, "floor_watch": fw}


def run_trial(cfg: dict[str, Any], cam_nom: Camera, plane_nom: LightPlane, base_light: Lighting, c: Condition,
              target: str | None, seed: int, blur_scale: float = 1.0, negative: str = "empty") -> Trial:
    """1 回の撮影を描いて、patrol（線なし）と inspect（線あり）で検出する。reaim なら、inspect の候補へ線を向け直して撮り直す。"""
    cfg = _with_pose_flag(cfg, c.pose_from_line)
    rng = np.random.default_rng(seed)
    cam_t = true_camera(cam_nom, c)
    plane_t = plane_for_true_camera(plane_nom, cam_nom, cam_t)
    r = Renderer(cam_t, plane_t, lighting_for(base_light, c, blur_scale, cam_nom, cam_t), sides=c.sides)
    objs: list[Disc] = []
    stains: list[Stain] = []
    seams: list[Seam] = []
    x = y = 0.0
    if target is not None:
        dia, h, alb, spec, _crit = TARGETS[target]
        y = float(rng.uniform(55.0, 100.0))
        x = float(rng.choice([-1.0, 1.0]) * c.aim_err_mm)
        objs.append(Disc(x, y, dia, h, alb, spec, target))
    elif negative == "stain":
        stains.append(Stain(float(rng.uniform(-10, 10)), float(rng.uniform(55, 100)), 18.0, 0.3))
    elif negative == "seam":
        seams.append(Seam("x", float(rng.uniform(55, 100)), 0.3, "step"))
    toy_box = None
    if c.clutter:
        toy = Disc(float(rng.choice([-1.0, 1.0]) * 28.0), y + float(rng.uniform(-15, 15)) if target else 80.0,
                   25.0, 25.0, 0.6, False, "toy")
        objs.append(toy)
        toy_box = true_bbox_px(cam_t, toy.x_mm, toy.y_mm, toy.diameter_mm, toy.height_mm)
    frames = r.render(Scene(objs, stains, seams, seed=seed, floor=c.floor, texture_contrast=c.texture_contrast,
                            floor_height_sigma_mm=c.floor_height_sigma_mm), noise_seed=seed)   # 試行ごとに再現できる雑音
    if c.flat_field:                                   # 較正は名目の姿勢で 1 回（本当の姿勢がずれていれば較正も少しずれる）
        cal = Renderer(cam_nom, plane_nom, lighting_for(base_light, c, blur_scale))
        frames = {**frames, **cal.flat_field(getattr(r, "last_gain", 1.0))}
    out = Trial(target)
    for stage, with_line in (("patrol", False), ("inspect", True)):
        try:
            cands, _tr, _fg = detect(frames, cam_nom, plane_nom, cfg, with_line=with_line)
        except MotionError:
            cands = []
        found = _objects(cands)
        hit = _hit_in_image(found, true_bbox_px(cam_t, x, y, TARGETS[target][0], TARGETS[target][1]),
                            pad_px=0.01 * cam_nom.f_px) if target else None
        # 陰性の誤報に使う。隣の物（おもちゃ）そのものを見つけた候補は誤報ではない（画像でおもちゃの写る範囲と重なるもの）
        others = [f for f in found if f is not hit and not (toy_box is not None and _hit_in_image([f], toy_box, 0.01 * cam_nom.f_px))]
        out.stage_hits[stage] = hit is not None
        out.false_objects[stage] = len(others)
        if stage == "inspect" and hit is not None:
            out.height_est, out.height_reason = hit.height_mm, hit.height_reason
            out.diameter_est = hit.diameter_mm
            out.critical_flag = any(k["kind"] == "metal_disc" for k in hit.kinds)
            out.loc_err_mm = float(np.hypot(hit.floor_xy_mm[0] - x, hit.floor_xy_mm[1] - y))
            if c.reaim and target is not None:
                out.critical_flag = _reaim_and_recapture(cfg, cam_nom, plane_nom, cam_t, plane_t, r, c, hit, _tr, target, x, y, seed)
    return out


def _reaim_and_recapture(cfg: dict[str, Any], cam_nom: Camera, plane_nom: LightPlane, cam_t: Camera, plane_t: LightPlane,
                         r: Renderer, c: Condition, hit: Candidate, tr: Any, target: str, x: float, y: float, seed: int) -> bool:
    """検出側が知っている姿勢（推定できていればそれ、無ければ名目）で「候補の横位置 − 線の横位置」を出し、その分だけ線を横へ動かして
    （頭 yaw の小さな回転を横移動で近似）撮り直す。本当の線に対する物のずれ = 最初のずれ − 検出側が見積もった動かす量。"""
    from serpens.floorwatch.pose import posed_camera
    if tr is not None and tr.pose is not None:
        cam_e = posed_camera(cam_nom, tr.pose[0], tr.pose[1])
        plane_e = plane_nom.fixed_to_head(cam_nom, cam_e)
    else:
        plane_e = plane_nom
    shift = hit.floor_xy_mm[0] - _floor_line_x(plane_e, hit.floor_xy_mm[1])
    offset = (x - _floor_line_x(plane_t, y)) - shift             # 狙い直した後の、本当の線からの物のずれ
    dia, h, alb, spec, _crit = TARGETS[target]
    x2 = _floor_line_x(plane_t, y) + offset
    frames = r.render(Scene([Disc(x2, y, dia, h, alb, spec, target)], seed=seed + 7, floor=c.floor,
                            texture_contrast=c.texture_contrast, floor_height_sigma_mm=c.floor_height_sigma_mm), noise_seed=seed + 7)
    try:
        cands, _t2, _fg = detect(frames, cam_nom, plane_nom, cfg, with_line=True)
    except MotionError:
        return False
    h2 = _hit_in_image(_objects(cands), true_bbox_px(cam_t, x2, y, dia, h), pad_px=0.01 * cam_nom.f_px)
    return h2 is not None and any(k["kind"] == "metal_disc" for k in h2.kinds)


def summarize(trials: list[Trial]) -> dict[str, Any]:
    pos = [t for t in trials if t.target is not None]
    neg = [t for t in trials if t.target is None]
    crit = [t for t in pos if TARGETS[t.target][4]]
    spec_crit = [t for t in pos if TARGETS[t.target][3] and TARGETS[t.target][4]]
    herr = [abs(t.height_est - TARGETS[t.target][1]) for t in pos if t.height_est is not None]
    derr = [abs(t.diameter_est - TARGETS[t.target][0]) / TARGETS[t.target][0] for t in pos if t.diameter_est is not None]
    rate = lambda xs, k: float(np.mean([x.stage_hits[k] for x in xs])) if xs else float("nan")  # noqa: E731
    return {"n_pos": len(pos), "n_neg": len(neg),
            "patrol_recall": rate(pos, "patrol"), "inspect_recall": rate(pos, "inspect"),
            "critical_inspect_recall": rate(crit, "inspect"),
            "specular_critical_flagged": float(np.mean([t.critical_flag for t in spec_crit])) if spec_crit else float("nan"),
            "false_alarm_patrol": float(np.mean([t.false_objects["patrol"] > 0 for t in neg])) if neg else float("nan"),
            "false_alarm_inspect": float(np.mean([t.false_objects["inspect"] > 0 for t in neg])) if neg else float("nan"),
            "height_abs_err_median_mm": float(np.median(herr)) if herr else float("nan"),
            "diameter_rel_err_median": float(np.median(derr)) if derr else float("nan"),
            "loc_err_median_mm": float(np.median([t.loc_err_mm for t in pos if t.loc_err_mm is not None]))
            if any(t.loc_err_mm is not None for t in pos) else float("nan")}


def condition_dict(c: Condition) -> dict[str, Any]:
    return asdict(c)
