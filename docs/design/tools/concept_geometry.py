"""Design Agent の外観案を、Engineering 制約に照らして数値で比べる（DESIGN_ESTIMATE）。

これは Design の比較用の概算であって、CAD でも Engineering 仕様でもない。
- 座標は CAD と同じ（mm、頭方向が −X、Z は床基準、J1 0° のとき）。
- センサー位置は Fusion `Serpens_FW03_CLEARANCE_STUDY_NOT_FOR_PRINT` の HEAD STUDY 02 参照箱
  （2026-09-29 に読み取り専用で bbox を読んだ値）。Engineering の値で、ここでは動かさない。
- 質量・材料密度・電子部品の質量はすべて ASSUMED。実測ゼロ。
- 「人から見た目の見えやすさ」は視線方向と目の法線の内積による幾何の目安で、人の評価ではない（HUMAN_EVALUATED = 0）。

実行:  .venv/Scripts/python.exe docs/design/tools/concept_geometry.py
出力:  docs/design/assets/*.svg, docs/design/results/concept_metrics_2026-09-29.json
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from matplotlib.path import Path as MplPath

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets"
RESULTS = ROOT / "results"

# ---- Engineering 側の参照値（出典は上の docstring。変更しない）----
J1_AXIS = (-181.8, 32.35)          # (X, Z) 首 pitch 軸。ASSUMED（CAD.md）
J1_RANGE_DEG = (0.0, 45.0)         # CAD 検討範囲。ソフトは 0〜40（robot_fw5.yaml, ASSUMPTION）
TORQUE_LIMIT_NM = 0.45             # software_torque_limit_nm。REFERENCE 由来、実測ではない
LENS = (-232.0, 30.0)              # カメラ前面 X と光学中心 Z（初期仮定）
CAM_TILT_DEG = 25.0                # 下向き
CAM_VFOV_DEG = 42.0                # FOV 65°（対角）を 4:3 に割った垂直。ASSUMED
CAM_HFOV_DEG = 54.0
NECK_WIDTH_FW03 = 88.0             # NECK DORSAL shell の幅（Fusion bbox）
BODY_WIDTH = 92.0
BODY_HEIGHT = 94.0

SENSORS = {  # 名前: (xmin, xmax, ymin, ymax, zmin, zmax)  Fusion bbox
    "camera XIAO Sense": (-232.0, -217.0, -10.5, 10.5, 22.0, 38.0),
    "ToF front": (-232.0, -230.0, -9.0, 9.0, 42.0, 55.0),
    "ToF down": (-200.0, -182.0, -9.0, 9.0, 5.0, 7.0),
    "raking LED L": (-232.0, -227.0, -22.0, -15.0, 5.0, 9.0),
    "raking LED R": (-232.0, -227.0, 15.0, 22.0, 5.0, 9.0),
    "IR LED": (-232.0, -228.0, 11.0, 17.0, 25.0, 31.0),
    "line light": (-232.0, -224.0, -8.0, 8.0, 62.0, 70.0),
    "WS2812B x2": (-200.0, -190.0, -16.0, -8.0, 70.0, 73.0),
    "skid TPU": (-230.0, -200.0, -18.0, 18.0, 0.0, 5.0),
}

# ライン光の光源候補（X, Z）。H1 は「眉」を顔の面からどこまで後ろへ下げられるかを見る
LINE_LIGHT_CANDIDATES = {
    "H0": [(-232.0, 66.0)],
    "H1": [(-235.0, 66.0), (-233.0, 66.0), (-230.0, 67.0), (-226.0, 68.0), (-222.0, 68.0)],
    "H2": [(-240.0, 62.0)],
}

# ---- 材料・部品（ASSUMED）----
PETG_G_CM3 = 1.27
WALL_MM = 2.0
HEAD_PARTS_G = {  # 名前: (質量 g, X, Z)。位置は SENSORS の中心付近
    "XIAO ESP32S3 Sense + camera": (8.0, -224.0, 30.0),
    "VL53L1X front (breakout)": (1.5, -231.0, 48.5),
    "VL53L1X down (breakout)": (1.5, -191.0, 6.0),
    "LEDs + slit + IR + WS2812": (4.0, -222.0, 40.0),
    "wiring + fasteners": (8.0, -200.0, 35.0),
    "skid TPU (5.4 cm3 x 1.2)": (6.5, -215.0, 2.5),
}


@dataclass
class Eye:
    center: tuple[float, float, float]      # X, Y(+側), Z。左右対称に 2 個
    diameter: float
    normal: tuple[float, float, float]      # +Y 側の目の外向き法線


@dataclass
class HeadConcept:
    key: str
    name: str
    xs: list[float]
    half_w: list[float]
    z_bot: list[float]
    z_top: list[float]
    eye: Eye
    n_super: float = 2.5                    # 断面の超楕円の指数（2 = 楕円）
    notes: list[str] = field(default_factory=list)
    lens_x: float = LENS[0]

    def section(self, x: float) -> tuple[float, float, float]:
        return (float(np.interp(x, self.xs, self.half_w)),
                float(np.interp(x, self.xs, self.z_bot)),
                float(np.interp(x, self.xs, self.z_top)))


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def egg_head() -> HeadConcept:
    """H0: 現行 FW03 の卵型（a38 r38 相当、bbox X−238〜−171.4, Y±38, Z6〜80）。"""
    cx, a, b, cz, c = -204.7, 33.3, 38.0, 43.0, 37.0
    xs = list(np.linspace(cx - a, cx + a, 41))
    s = [math.sqrt(max(0.0, 1 - ((x - cx) / a) ** 2)) for x in xs]
    # 目: FW03 の eye insert D14 中心 (−225, ±20, 62)。法線は卵の面法線
    p = np.array([-225.0 - cx, 20.0, 62.0 - cz])
    n = _unit([p[0] / a**2, p[1] / b**2, p[2] / c**2])
    return HeadConcept(
        key="H0", name="Egg（現行 FW03）", xs=xs,
        half_w=[b * v for v in s], z_bot=[cz - c * v for v in s], z_top=[cz + c * v for v in s],
        eye=Eye((-225.0, 20.0, 62.0), 14.0, tuple(n)), n_super=2.0,
        notes=["頭幅 76 < 首 88 < 胴 92。頭が首より細い", "前面が球面なので Z5〜9 の斜め照明が殻の外に出る（FW03 は窓で逃がしている）"],
    )


def bean_head() -> HeadConcept:
    """H1: Bean（平らなあご・丸い鼻先・盛り上がった頭頂、頭幅 100 > 胴 92）。Design 推奨候補。"""
    return HeadConcept(
        key="H1", name="Bean（推奨候補）",
        n_super=3.0,
        xs=[-236, -233, -226, -215, -200, -185, -170, -158],
        half_w=[30, 36, 42, 48, 50, 48, 40, 32],
        z_bot=[4, 3, 3, 3, 3, 4, 6, 10],
        z_top=[56, 60, 67, 74, 77, 76, 72, 66],
        eye=Eye((-212.0, 37.0, 60.0), 24.0, tuple(_unit([-0.62, 0.72, 0.30]))),
        notes=["鼻先（Z≤60。前 ToF Z42〜55 の窓が顔に載る高さ）はカメラ・前 ToF・斜め照明の平らな顔。ライン光は鼻先の上の『眉の段』（X≈−224, Z66〜70）",
               "頭幅 100 で横基線 30 mm のライン光（PRODUCT.md の記載）も入る",
               "首は幅 64 へ絞る提案（首の中の J2 サーボ ±12.4・首シャーシ ±24 が入るかは Engineering 確認）"],
        lens_x=-234.0,
    )


def wedge_head() -> HeadConcept:
    """H2: Wedge（写実寄り。長い平たい鼻先、頭幅 80）。比較用。"""
    return HeadConcept(
        key="H2", name="Wedge（写実寄り）",
        xs=[-262, -255, -240, -220, -200, -180, -165, -158],
        half_w=[22, 28, 34, 38, 40, 38, 34, 32],
        z_bot=[4, 3, 3, 3, 3, 4, 6, 10],
        z_top=[40, 44, 50, 56, 61, 62, 60, 58],
        eye=Eye((-226.0, 37.0, 50.0), 16.0, tuple(_unit([-0.25, 0.95, 0.20]))),
        notes=["鼻先を 26 mm 延ばすのでカメラも前へ（X≈−258）。頭の重心が J1 から遠くなる",
               "頭頂 Z≤62 なので、カメラ上 35 mm の縦基線ライン光（Z62〜70）が鼻先に入らない"],
        lens_x=-258.0,
    )


# ---------------------------------------------------------------- head metrics
def _superellipse_perimeter_area(w: float, h: float, n: float, m: int = 256) -> tuple[float, float]:
    t = np.linspace(0, 2 * np.pi, m, endpoint=False)
    c, s = np.cos(t), np.sin(t)
    y = w * np.sign(c) * np.abs(c) ** (2 / n)
    z = h * np.sign(s) * np.abs(s) ** (2 / n)
    per = float(np.sum(np.hypot(np.diff(np.r_[y, y[0]]), np.diff(np.r_[z, z[0]]))))
    area = float(0.5 * abs(np.dot(y, np.roll(z, -1)) - np.dot(z, np.roll(y, -1))))
    return per, area


def head_metrics(hc: HeadConcept) -> dict:
    xs = np.linspace(min(hc.xs), max(hc.xs), 200)
    dx = xs[1] - xs[0]
    per_l, area_l, cx_l, cz_l = [], [], [], []
    for x in xs:
        w, zb, zt = hc.section(x)
        per, area = _superellipse_perimeter_area(w, (zt - zb) / 2, hc.n_super)
        per_l.append(per); area_l.append(area); cx_l.append(x); cz_l.append((zt + zb) / 2)
    per_a = np.array(per_l)
    lateral_mm2 = float(np.sum(per_a) * dx)
    # 端面（前と後ろ）は半分を殻にする近似
    caps = 0.5 * (area_l[0] + area_l[-1])
    shell_mm3 = (lateral_mm2 + caps) * WALL_MM
    shell_g = shell_mm3 / 1000.0 * PETG_G_CM3
    shell_cx = float(np.sum(per_a * np.array(cx_l)) / np.sum(per_a))
    shell_cz = float(np.sum(per_a * np.array(cz_l)) / np.sum(per_a))
    eye_g = 2 * (2 * math.pi * (hc.eye.diameter / 2) ** 2 * 1.5) / 1000.0 * PETG_G_CM3  # 半球殻 1.5 mm
    masses = [(shell_g, shell_cx, shell_cz), (eye_g, hc.eye.center[0], hc.eye.center[2])]
    dx_parts = hc.lens_x - LENS[0]           # 鼻先を延ばす案は前面の部品も一緒に前へ出る
    for g, x, z in HEAD_PARTS_G.values():
        masses.append((g, x + (dx_parts if x < -215 else 0.0), z))
    m = sum(g for g, _, _ in masses)
    com_x = sum(g * x for g, x, _ in masses) / m
    com_z = sum(g * z for g, _, z in masses) / m
    torques = []
    for phi in np.linspace(*J1_RANGE_DEG, 46):
        # +Y 回転で前方（−X）が上がる。J1 まわりに (dx, dz) を回して水平の腕を取る
        rx, rz = com_x - J1_AXIS[0], com_z - J1_AXIS[1]
        p = math.radians(phi)
        x_rot = rx * math.cos(p) + rz * math.sin(p)
        torques.append(abs(x_rot) / 1000.0 * m / 1000.0 * 9.81)
    t_max = max(torques)

    max_w = 2 * max(hc.half_w)
    height = max(hc.z_top) - min(hc.z_bot)
    return {
        "key": hc.key, "name": hc.name,
        "length_mm": round(max(hc.xs) - min(hc.xs), 1),
        "max_width_mm": round(max_w, 1), "height_mm": round(height, 1),
        "head_to_body_width": round(max_w / BODY_WIDTH, 2),
        "eye_diameter_mm": hc.eye.diameter,
        "eye_to_head_height": round(hc.eye.diameter / height, 2),
        "shell_mass_g_ASSUMED": round(shell_g, 1),
        "head_mass_g_ASSUMED": round(m, 1),
        "com_ahead_of_J1_mm": round(J1_AXIS[0] - com_x, 1),
        "j1_static_torque_max_Nm_ESTIMATE": round(t_max, 3),
        "j1_torque_fraction_of_software_limit": round(t_max / TORQUE_LIMIT_NM, 3),
        "fov_self_occlusion": fov_self_occlusion(hc),
        "line_light_candidates": [line_light_occlusion(hc, x, z) for x, z in LINE_LIGHT_CANDIDATES.get(hc.key, [])],
        "eye_visibility_mm2": eye_visibility(hc),
        "notes": hc.notes,
    }


def _inside_profile(hc: HeadConcept, x: float, z: float) -> bool:
    if not (min(hc.xs) <= x <= max(hc.xs)):
        return False
    _, zb, zt = hc.section(x)
    return zb <= z <= zt


def fov_self_occlusion(hc: HeadConcept) -> dict:
    """側面視で、カメラの垂直視野（下向き 25° ± 21°）の光線が頭の殻から出たあと、また殻に当たるか。
    殻を出る所 = 窓。頭の座標で見るので J1 角に依らない（床・胴は別）。"""
    lens_x, lens_z = hc.lens_x, LENS[1]
    blocked, exits = [], []
    for ang in np.linspace(CAM_TILT_DEG - CAM_VFOV_DEG / 2, CAM_TILT_DEG + CAM_VFOV_DEG / 2, 43):
        a = math.radians(ang)
        d = np.array([-math.cos(a), -math.sin(a)])
        left, exit_pt = False, None
        for t in np.arange(0.0, 300.0, 0.25):
            x, z = lens_x + d[0] * t, lens_z + d[1] * t
            if z <= 0:
                break
            inside = _inside_profile(hc, x, z)
            if not left and not inside:
                left, exit_pt = True, (x, z)
            elif left and inside:
                blocked.append(round(float(ang), 1))
                break
        if exit_pt:
            exits.append(exit_pt)
    zs = [z for _, z in exits]
    xs = [x for x, _ in exits]
    return {"occluded": bool(blocked), "blocked_ray_deg_down": blocked[:6],
            "window_z_range_mm": [round(min(zs), 1), round(max(zs), 1)] if zs else None,
            "window_ahead_of_lens_mm": round(lens_x - min(xs), 1) if xs else None,
            "rule": "レンズより前に出る殻は、前方距離 d で Z < 30 − d·tan46° か Z > 30 − d·tan4° に限る（下向き 25°、垂直 42° ASSUMED）"}



def line_light_occlusion(hc: HeadConcept, src_x: float, src_z: float, near_mm: float = 38.0, far_mm: float = 147.0) -> dict:
    """ライン光の光源 (X, Z) から、レンズ前方 near〜far mm の床へ向かう光線が頭の殻に当たるか（側面視）。
    near/far は floor_watch の位置合わせ目標（床カメラの見える範囲 38〜147 mm、docs/floorwatch_phase5.md）。"""
    blocked = []
    for d in np.linspace(near_mm, far_mm, 23):
        tx = hc.lens_x - d
        n = 400
        for i in range(1, n):
            t = i / n
            x = src_x + (tx - src_x) * t
            z = src_z + (0.0 - src_z) * t
            if x > src_x - 0.5:        # 光源の直前は窓（殻を出る所）
                continue
            if _inside_profile(hc, x, z):
                blocked.append(round(float(d), 1))
                break
    return {"source_xz": [src_x, src_z], "blocked_floor_distances_mm": blocked, "clear": not blocked}

VIEWS = {  # 名前: 機体から見た人の方向（単位ベクトル。−X = 機体の正面）
    "child_front_eye_level (1 m, 目の高さ 0.4 m)": (-math.cos(math.radians(22)), 0.0, math.sin(math.radians(22))),
    "parent_front_standing (2 m, 1.5 m)": (-math.cos(math.radians(37)), 0.0, math.sin(math.radians(37))),
    "parent_side_standing (2 m, 1.5 m)": (0.0, math.cos(math.radians(37)), math.sin(math.radians(37))),
    "child_side_crawling (1 m, 0.2 m)": (0.0, math.cos(math.radians(11)), math.sin(math.radians(11))),
    "top_down": (0.0, 0.0, 1.0),
}


def eye_visibility(hc: HeadConcept) -> dict:
    """目の見かけの面積（mm²、両目の合計）。遮蔽は無視（凸形状の近似）。"""
    r = hc.eye.diameter / 2
    n_r = np.array(hc.eye.normal)
    n_l = n_r * np.array([1, -1, 1])
    out = {}
    for name, v in VIEWS.items():
        v = np.array(v)
        a = math.pi * r * r * (max(0.0, float(n_r @ v)) + max(0.0, float(n_l @ v)))
        out[name] = round(a, 0)
    return out


# ------------------------------------------------------------- joint pinch gaps
def _sample_polygon(poly: np.ndarray, step: float = 0.5) -> np.ndarray:
    pts = []
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        n = max(2, int(np.linalg.norm(b - a) / step))
        pts.append(np.linspace(a, b, n, endpoint=False))
    return np.vstack(pts)


def _rot(pts: np.ndarray, deg: float) -> np.ndarray:
    t = math.radians(deg)
    r = np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
    return pts @ r.T


def _min_dist(a: np.ndarray, b: np.ndarray) -> float:
    d = np.sqrt(((a[:, None, :] - b[None, :, :]) ** 2).sum(-1))
    return float(d.min())


def egg_pair() -> tuple[np.ndarray, np.ndarray]:
    """B0: FW03 の LINK1/LINK2 卵殻の上面視（a40, 半幅 46）。関節 J3 を原点、胴の前方を −x。"""
    t = np.linspace(0, 2 * np.pi, 400, endpoint=False)
    front = np.c_[-45 + 40 * np.cos(t), 46 * np.sin(t)]   # X −85〜−5
    rear = np.c_[50 + 40 * np.cos(t), 46 * np.sin(t)]     # X 10〜90
    return front, rear


def knuckle_pair(r_bead=46.0, clearance=4.0, lip_deg=40.0, half_w=46.0, chamfer_x=60.0):
    """B1: 同心ナックル。前リンクの後端 = 関節軸まわりの円柱（半径 r_bead）、
    後リンクの前端 = 半径 r_bead+clearance の受け（唇は ±lip_deg まで）。上面視の輪郭。"""
    t = np.linspace(math.radians(90), math.radians(-90), 181)
    bead = np.c_[r_bead * np.cos(t), r_bead * np.sin(t)]
    front = np.vstack([[-95.0, half_w], [0.0, half_w], bead[1:-1], [0.0, -half_w], [-95.0, -half_w]])
    rs = r_bead + clearance
    tl = np.linspace(math.radians(-lip_deg), math.radians(lip_deg), 81)
    socket = np.c_[rs * np.cos(tl), rs * np.sin(tl)]
    rear = np.vstack([[chamfer_x, -half_w], socket, [chamfer_x, half_w], [95.0, half_w], [95.0, -half_w]])
    return front, rear


def pinch_table(kind: str) -> dict:
    front, rear = egg_pair() if kind == "B0" else knuckle_pair()
    fs = _sample_polygon(front) if kind == "B1" else front
    rows = []
    collide_any = False
    for deg in range(0, 51, 5):
        r = _rot(rear, deg)
        rs = _sample_polygon(r) if kind == "B1" else r
        d = _min_dist(fs, rs)
        inside = MplPath(front).contains_points(rs).any() or MplPath(r).contains_points(fs).any()
        collide_any |= bool(inside)
        # 内側（曲げの内側 = +y）の、体の側面付近（|y| ≥ 25）に限った最小すき間 = 指が横から入る所
        side_f = fs[fs[:, 1] >= 25]
        side_r = rs[rs[:, 1] >= 25]
        d_side = _min_dist(side_f, side_r) if len(side_f) and len(side_r) else None
        rows.append({"yaw_deg": deg, "min_gap_mm": round(d, 1),
                     "inner_side_gap_mm": None if d_side is None else round(d_side, 1),
                     "collision": bool(inside)})
    danger = [r for r in rows if r["inner_side_gap_mm"] is not None and 8.0 <= r["inner_side_gap_mm"] <= 25.0]
    return {"kind": kind, "rows": rows, "collision_any": collide_any,
            "angles_with_side_gap_in_8_to_25mm": [r["yaw_deg"] for r in danger],
            "rule": "構想設計書 16 章: すき間 8 mm 以下か 25 mm 以上（docs/safety_limits.md §2）。閉じていく 8〜25 mm が指の挟み込み域"}


def knuckle_wedge_angles(r_bead=46.0, clearance=4.0, lip_deg=40.0, half_w=46.0, chamfer_x=60.0) -> dict:
    """B1 の側面の V 溝の角度（唇の先で、前リンクの面と後リンクの面剥がれ方向のなす角）。鈍角なら指は押し出される。"""
    rs = r_bead + clearance
    lip = np.array([rs * math.cos(math.radians(lip_deg)), rs * math.sin(math.radians(lip_deg))])
    cham = _unit(np.array([chamfer_x, half_w]) - lip)
    out = {}
    for yaw in (0, 50, -50):
        c = _rot(cham[None, :], yaw)[0]
        lip_rot = _rot(lip[None, :], yaw)[0]
        ang = math.degrees(math.atan2(lip_rot[1], lip_rot[0]))
        if ang >= 90:       # 唇が胴の直線の側面まで来た: 前リンクの面は −x 方向の直線
            face = np.array([-1.0, 0.0])
        else:               # 円柱の接線（外へ向かう側 = 角度が増える向き）
            a = math.radians(ang)
            face = np.array([-math.sin(a), math.cos(a)])
        out[f"yaw_{yaw:+d}"] = {"lip_angle_deg": round(ang, 1),
                                "wedge_deg": round(math.degrees(math.acos(float(np.clip(face @ c, -1, 1)))), 1),
                                "apex_gap_mm": clearance}
    return {"lip_deg": lip_deg, "requirement": f"lip_deg ≤ 90 − yaw_max（= {90 - 50}°）でないと、曲げの内側で唇が前リンクへ入る",
            "angles": out}


# ------------------------------------------------------------------- body poses
SEGMENTS = [("head+neck", 143.0), ("L1", 95.0), ("L2", 95.0), ("L3", 95.0), ("tail", 145.0)]


def pose_chain(yaws: list[float], j1_deg: float = 0.0):
    """上面視の中心線（頭先端 → J2 → J3 → J4 → J5 → 尾端）。尾の線分を +x に固定して置く
    （体の後ろ半分は発見時の位置に残り、J2 側で頭の向きが変わる）。"""
    head_len = SEGMENTS[0][1]
    # J1 を上げると頭の平面投影は短くなる（頭先端から J1 までの 56 mm 分だけ）
    head_proj = head_len - 56.0 * (1 - math.cos(math.radians(j1_deg)))
    lengths = [head_proj] + [s for _, s in SEGMENTS[1:]]
    pts = [np.array([0.0, 0.0])]
    heading = 0.0
    for i, L in enumerate(lengths):
        if i >= 1:
            heading += yaws[i - 1]
        a = math.radians(heading)
        pts.append(pts[-1] + L * np.array([math.cos(a), math.sin(a)]))
    pts = np.array(pts)
    # 尾の線分を +x 向き、尾端を (573, 0) に合わせる
    d = pts[-1] - pts[-2]
    rot = -math.atan2(d[1], d[0])
    c, s_ = math.cos(rot), math.sin(rot)
    pts = pts @ np.array([[c, -s_], [s_, c]]).T
    return pts + (np.array([573.0, 0.0]) - pts[-1])


OBJECT_WORLD = np.array([-90.0, 0.0])     # 発見の構え（一直線）で頭先端の 90 mm 前


POSES = {
    "patrol": {"label": "Patrol（巡回）", "yaws": [20, 30, -5, -30], "j1": 5,
               "note": "低い頭、ゆるい S。目は柔らかい青緑、呼吸だけ"},
    "discover_point": {"label": "Discover – Point（見つけた）", "yaws": [0, 0, 0, 0], "j1": 0,
                       "note": "撮影のため静止。体の一直線が物を指す矢印になる（ポインター犬の構え）", "object": "world"},
    "look_person": {"label": "Look at person（人を見る）", "yaws": [45, 10, 0, 0], "j1": 30,
                    "note": "J2 で頭ごと人へ、J1 で見上げる。目を明るく、短い音。物は体の線が指したまま", "object": "world",
                    "person": 320},
    "marker_wait": {"label": "Marker（ここにあるよ）", "yaws": [10, 20, 25, 25], "j1": 10,
                    "note": "子どもが近くにいない時だけ。物の横で頭を物へ向けたまま待つ『生きたピン』。囲まない", "object": "ahead"},
    "child_near_quiet": {"label": "Child near（子どもが近い）", "yaws": [-50, -20, 0, 0], "j1": 25,
                         "note": "物を見ない・指さない。頭と目は子どもへ（物から約 70° 外す）。光と音は控えめ、通知は保護者だけ",
                         "object": "world", "child": 260},
    "sleep_crescent": {"label": "Sleep / charge（三日月）", "yaws": [45, 45, 45, 45], "j1": 0,
                       "note": "yaw 合計 180°。合計上限（未決 Q8）と要照合。とぐろ（輪）ではない"},
}


# --------------------------------------------------------------------- SVG out
SVG_STYLE = """<style>
.shell{fill:#dfe9d8;stroke:#3c5a3c;stroke-width:1.2}
.belly{fill:#f4efe2;stroke:#3c5a3c;stroke-width:1}
.sensor{fill:#6b7cff;fill-opacity:.55;stroke:#3440a0;stroke-width:.6}
.fov{fill:#ffb347;fill-opacity:.18;stroke:#d9822b;stroke-width:.8;stroke-dasharray:3 2}
.eye{fill:#1d2330;stroke:#1d2330}
.eyeglint{fill:#ffffff}
.axis{fill:none;stroke:#b3261e;stroke-width:1}
.label{font:10px sans-serif;fill:#333}
.title{font:bold 12px sans-serif;fill:#111}
.floor{stroke:#999;stroke-width:1}
.body{fill:none;stroke:#8fae8a;stroke-linecap:round;stroke-linejoin:round}
.spine{fill:none;stroke:#3c5a3c;stroke-width:1.2}
.obj{fill:#e0b400;stroke:#7a6200}
.person{fill:#b3261e;fill-opacity:.8}
.childm{fill:#e46aa0}
</style>"""


def _svg(w, h, body, vb=None):
    vb = vb or f"0 0 {w} {h}"
    return f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{vb}" width="{w}" height="{h}">{SVG_STYLE}{body}</svg>'


def head_side_svg(hc: HeadConcept) -> str:
    # 画面: X −270..−140 → 横、Z −10..100 → 縦（上が +Z）
    sx = lambda x: (x + 275) * 2.2
    sz = lambda z: (100 - z) * 2.2
    top = " ".join(f"{sx(x):.1f},{sz(z):.1f}" for x, z in zip(hc.xs, hc.z_top))
    bot = " ".join(f"{sx(x):.1f},{sz(z):.1f}" for x, z in zip(reversed(hc.xs), reversed(hc.z_bot)))
    parts = [f'<text class="title" x="8" y="14">{hc.key} {hc.name} — 側面</text>',
             f'<line class="floor" x1="0" y1="{sz(0):.1f}" x2="{sx(-140):.1f}" y2="{sz(0):.1f}"/>',
             f'<polygon class="shell" points="{top} {bot}"/>']
    # 腹色の帯（Z 45 以下）: 側面から見える下半分
    lens_x, lens_z = hc.lens_x, LENS[1]
    for name, (x0, x1, _, _, z0, z1) in SENSORS.items():
        dx = (lens_x - LENS[0]) if x0 < -215 else 0.0
        if name == "line light" and hc.key == "H1":
            x0, x1 = -226.0, -220.0       # 眉の段へ（提案。Engineering 確認）
            z0, z1 = 66.0, 70.0
        parts.append(f'<rect class="sensor" x="{sx(x0 + dx):.1f}" y="{sz(z1):.1f}" '
                     f'width="{(x1 - x0) * 2.2:.1f}" height="{(z1 - z0) * 2.2:.1f}"/>')
    # 視錐台
    L = 140
    lo = math.radians(CAM_TILT_DEG + CAM_VFOV_DEG / 2)
    hi = math.radians(CAM_TILT_DEG - CAM_VFOV_DEG / 2)
    x_lo = lens_x - min(L, lens_z / math.tan(lo))
    parts.append(f'<polygon class="fov" points="{sx(lens_x):.1f},{sz(lens_z):.1f} '
                 f'{sx(x_lo):.1f},{sz(lens_z - (lens_x - x_lo) * math.tan(lo)):.1f} '
                 f'{sx(lens_x - L):.1f},{sz(lens_z - L * math.tan(hi)):.1f}"/>')
    ex, _, ez = hc.eye.center
    parts.append(f'<circle class="eye" cx="{sx(ex):.1f}" cy="{sz(ez):.1f}" r="{hc.eye.diameter / 2 * 2.2 * max(0.35, abs(hc.eye.normal[1])):.1f}"/>')
    parts.append(f'<circle class="eyeglint" cx="{sx(ex) - 3:.1f}" cy="{sz(ez) - 4:.1f}" r="2.5"/>')
    ax, az = J1_AXIS
    parts.append(f'<circle class="axis" cx="{sx(ax):.1f}" cy="{sz(az):.1f}" r="4"/>'
                 f'<text class="label" x="{sx(ax) + 6:.1f}" y="{sz(az) + 4:.1f}">J1</text>')
    parts.append(f'<text class="label" x="8" y="{sz(0) + 16:.1f}">青 = センサー参照箱（Engineering の位置）、橙 = カメラ視錐台（ASSUMED FOV）</text>')
    return _svg(300, 260, "".join(parts))


def head_front_svg(hc: HeadConcept) -> str:
    cx = 150
    sy = lambda y: cx + y * 2.2
    sz = lambda z: (100 - z) * 2.2
    zs = np.linspace(min(hc.z_bot), max(hc.z_top), 80)
    halfw = []
    for z in zs:
        best = 0.0
        for x in np.linspace(min(hc.xs), max(hc.xs), 120):
            w, zb, zt = hc.section(x)
            h = (zt - zb) / 2
            zc = (zt + zb) / 2
            u = abs(z - zc) / h if h > 0 else 2
            if u <= 1:
                best = max(best, w * (1 - u ** hc.n_super) ** (1 / hc.n_super))
        halfw.append(best)
    right = [f"{sy(w):.1f},{sz(z):.1f}" for z, w in zip(zs, halfw)]
    left = [f"{sy(-w):.1f},{sz(z):.1f}" for z, w in zip(reversed(zs), reversed(halfw))]
    parts = [f'<text class="title" x="8" y="14">{hc.key} — 正面（胴の輪郭は点線）</text>',
             f'<rect x="{sy(-BODY_WIDTH / 2):.1f}" y="{sz(BODY_HEIGHT):.1f}" width="{BODY_WIDTH * 2.2:.1f}" '
             f'height="{BODY_HEIGHT * 2.2:.1f}" rx="60" fill="none" stroke="#999" stroke-dasharray="4 3"/>',
             f'<polygon class="shell" points="{" ".join(right + left)}"/>']
    for name, (_, _, y0, y1, z0, z1) in SENSORS.items():
        if name in ("ToF down", "skid TPU", "WS2812B x2"):
            continue
        if name == "line light" and hc.key == "H1":
            z0, z1 = 66.0, 70.0
        parts.append(f'<rect class="sensor" x="{sy(y0):.1f}" y="{sz(z1):.1f}" width="{(y1 - y0) * 2.2:.1f}" height="{(z1 - z0) * 2.2:.1f}"/>')
    ex, ey, ez = hc.eye.center
    rx = hc.eye.diameter / 2 * 2.2 * max(0.3, abs(hc.eye.normal[0]))
    ry = hc.eye.diameter / 2 * 2.2
    for s in (1, -1):
        parts.append(f'<ellipse class="eye" cx="{sy(s * ey):.1f}" cy="{sz(ez):.1f}" rx="{rx:.1f}" ry="{ry:.1f}"/>')
        parts.append(f'<circle class="eyeglint" cx="{sy(s * ey) - 2:.1f}" cy="{sz(ez) - 4:.1f}" r="2.5"/>')
    return _svg(300, 260, "".join(parts))


def pose_svg(key: str, pose: dict) -> str:
    pts = pose_chain(pose["yaws"], pose["j1"])
    head_dir = _unit(pts[0] - pts[1])
    marks = []
    if pose.get("object") == "world":
        marks.append(("obj", OBJECT_WORLD, "物"))
    elif pose.get("object") == "ahead":
        marks.append(("obj", pts[0] + head_dir * 90, "物"))
    if pose.get("person"):
        marks.append(("person", pts[0] + head_dir * pose["person"], "保護者"))
    if pose.get("child"):
        marks.append(("childm", pts[0] + head_dir * pose["child"], "子ども"))
    allp = np.vstack([pts] + [m[1][None, :] for m in marks])
    mn, mx = allp.min(0) - 70, allp.max(0) + 70
    w, h = 300, 230
    s = min((w - 10) / (mx - mn)[0], (h - 50) / (mx - mn)[1], 0.42)
    off = np.array([w / 2, h / 2 + 4]) - (mn + mx) / 2 * s
    P = pts * s + off
    path = " ".join(f"{x:.1f},{y:.1f}" for x, y in P)
    parts = [f'<text class="title" x="8" y="14">{pose["label"]}</text>',
             f'<polyline class="body" points="{path}" stroke-width="{BODY_WIDTH * s:.1f}"/>',
             f'<polyline class="spine" points="{path}"/>']
    hx, hy = P[0] - head_dir * 18 * s
    ang = math.degrees(math.atan2(head_dir[1], head_dir[0]))
    parts.append(f'<ellipse class="shell" cx="{hx:.1f}" cy="{hy:.1f}" rx="{40 * s:.1f}" ry="{50 * s:.1f}" transform="rotate({ang:.1f} {hx:.1f} {hy:.1f})"/>')
    for sgn in (1, -1):
        n = np.array([-head_dir[1], head_dir[0]]) * sgn
        e = np.array([hx, hy]) + n * 40 * s + head_dir * 4 * s
        parts.append(f'<circle class="eye" cx="{e[0]:.1f}" cy="{e[1]:.1f}" r="{6 * s + 1.5:.1f}"/>')
    for cls, p, label in marks:
        q = p * s + off
        r = 5 if cls == "obj" else 8
        parts.append(f'<circle class="{cls}" cx="{q[0]:.1f}" cy="{q[1]:.1f}" r="{r}"/><text class="label" x="{q[0] + r + 3:.1f}" y="{q[1] + 4:.1f}">{label}</text>')
    yaw_sum = sum(pose["yaws"])
    parts.append(f'<text class="label" x="8" y="{h - 8}">J1 {pose["j1"]}° / J2–J5 {pose["yaws"]}（合計 {yaw_sum}°）</text>')
    return _svg(w, h, "".join(parts))


def gap_chart_svg(b0: dict, b1: dict) -> str:
    w, h = 420, 240
    x0, y0, x1, y1 = 50, 20, 400, 200
    sx = lambda a: x0 + (x1 - x0) * a / 50
    sy = lambda g: y1 - (y1 - y0) * min(g, 40) / 40
    parts = [f'<rect x="{x0}" y="{sy(25):.1f}" width="{x1 - x0}" height="{sy(8) - sy(25):.1f}" fill="#b3261e" fill-opacity=".12"/>',
             f'<text class="label" x="{x0 + 4}" y="{sy(25) + 12:.1f}">8〜25 mm = 挟み込み域</text>',
             f'<line class="floor" x1="{x0}" y1="{y1}" x2="{x1}" y2="{y1}"/><line class="floor" x1="{x0}" y1="{y0}" x2="{x0}" y2="{y1}"/>']
    for g in (0, 8, 25, 40):
        parts.append(f'<text class="label" x="{x0 - 26}" y="{sy(g) + 4:.1f}">{g}</text>')
    for a in (0, 25, 50):
        parts.append(f'<text class="label" x="{sx(a) - 6:.1f}" y="{y1 + 14}">{a}°</text>')
    for data, color, name in ((b0, "#b3261e", "B0 卵殻（現行）"), (b1, "#2e7d32", "B1 同心ナックル")):
        pts = " ".join(f"{sx(r['yaw_deg']):.1f},{sy(r['inner_side_gap_mm']):.1f}" for r in data["rows"] if r["inner_side_gap_mm"] is not None)
        parts.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="2"/>')
    parts.append(f'<text class="label" x="{x1 - 150}" y="{y0 + 10}" fill="#b3261e">— B0 卵殻（現行）</text>')
    parts.append(f'<text class="label" x="{x1 - 150}" y="{y0 + 24}" fill="#2e7d32">— B1 同心ナックル（4 mm）</text>')
    parts.append(f'<text class="label" x="{x0}" y="{h - 6}">横軸 = 隣り合う節の yaw、縦軸 = 曲げの内側・胴の側面（|y|≥25）の最小すき間 mm（上面視の 2D 近似）</text>')
    return _svg(w, h, "".join(parts))


def main() -> None:
    ASSETS.mkdir(parents=True, exist_ok=True)
    RESULTS.mkdir(parents=True, exist_ok=True)
    heads = [egg_head(), bean_head(), wedge_head()]
    metrics = {"source": "DESIGN_ESTIMATE（Design Agent の概算。HUMAN_EVALUATED = 0、HARDWARE_UNVERIFIED）",
               "date": "2026-09-29", "heads": [], "joints": {}, "poses": {}}
    for hc in heads:
        metrics["heads"].append(head_metrics(hc))
        (ASSETS / f"head_{hc.key}_side.svg").write_text(head_side_svg(hc), encoding="utf-8")
        (ASSETS / f"head_{hc.key}_front.svg").write_text(head_front_svg(hc), encoding="utf-8")
    b0, b1 = pinch_table("B0"), pinch_table("B1")
    b1["wedge"] = knuckle_wedge_angles()
    metrics["joints"] = {"B0_egg": b0, "B1_knuckle": b1}
    (ASSETS / "joint_gap_chart.svg").write_text(gap_chart_svg(b0, b1), encoding="utf-8")
    for key, pose in POSES.items():
        pts = pose_chain(pose["yaws"], pose["j1"])
        metrics["poses"][key] = {"label": pose["label"], "yaws_J2_J5": pose["yaws"], "j1_deg": pose["j1"],
                                 "yaw_sum_deg": sum(pose["yaws"]), "within_software_yaw_limit": all(abs(a) <= 50 for a in pose["yaws"]),
                                 "within_j1_cad_range": J1_RANGE_DEG[0] <= pose["j1"] <= J1_RANGE_DEG[1],
                                 "head_to_tail_mm": round(float(np.linalg.norm(pts[-1] - pts[0])), 0), "note": pose["note"]}
        (ASSETS / f"pose_{key}.svg").write_text(pose_svg(key, pose), encoding="utf-8")
    out = RESULTS / "concept_metrics_2026-09-29.json"
    out.write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")
    for h in metrics["heads"]:
        print(h["key"], h["name"], {k: h[k] for k in ("max_width_mm", "height_mm", "head_to_body_width", "eye_to_head_height",
                                                      "head_mass_g_ASSUMED", "com_ahead_of_J1_mm", "j1_static_torque_max_Nm_ESTIMATE",
                                                      "j1_torque_fraction_of_software_limit")},
              "occluded" if h["fov_self_occlusion"]["occluded"] else "FOV clear")
        print("   eyes", h["eye_visibility_mm2"])
    for k, j in metrics["joints"].items():
        print(k, "collision" if j["collision_any"] else "no collision", "danger angles", j["angles_with_side_gap_in_8_to_25mm"])
        print("   ", [(r["yaw_deg"], r["min_gap_mm"], r["inner_side_gap_mm"]) for r in j["rows"]])
    print("B1 wedge", json.dumps(b1["wedge"]["angles"], ensure_ascii=False))
    for k, p in metrics["poses"].items():
        print(k, p["yaw_sum_deg"], p["within_software_yaw_limit"], p["head_to_tail_mm"])
    print("wrote", out)


if __name__ == "__main__":
    main()
