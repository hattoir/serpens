"""FLANK SKIRT STUDY: 胴の脇の V を塞ぐ「すべる重ねフラップ」の運動学モデル（KINEMATIC_SIM。実物の柔らかさ・摩擦・つっぱりは入っていない）。

考え方: 各関節の各脇に、椀側リンク B の面取りの外の角 R に根元を固定した薄い帯（TPU 想定、厚さ T mm、長さ L_F mm）を付ける。
帯は玉側リンク A の外形（玉の円 Rc ＋ 脇の直線）に沿って「ぴんと張る」: R から A の玉へ接線（または脇の直線の起点 T_top）に向かい、
その先は A の外形に沿ってすべる。帯の先端は A に触れているので、帯・A・B の間の V は閉じた空洞になる（口が無い）。
A の座標系（玉の中心が原点、+x が B 側）で計算し、姿勢の変換で世界へ戻す。断面は z 方向にまっすぐ立てる（Z0〜Z1）。

寸法: 玉の円は Rc = 46.3（測定: 玉 R 45.5、隙間 0.8）、B の面取りの角 R_local = (59.5, ±45.5)（J3・J4。J5 は尾が細いので別）。
実行の例は gap_skirt_run.py。
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

TF = Path("C:/Users/Public/serpens_gapcheck/transforms.json")
RC = 46.3            # A の玉に沿う帯の内側の半径（玉 R 45.5 + すき間 0.8）
T = 1.2              # 帯の厚さ
TAIL_OVERLAP = 8.0   # 帯の先端が A に触れる点（接点）から先へ A の外形に重ねる長さ（帯は伸縮する想定: 長さは接点までの距離 + これ）
ROOT_EXT = 6.0       # 根元を B の中へ延ばす長さ（固定の表現）
Z0, Z1 = 2.0, 93.0   # 帯の高さ（腹の下端〜背板の頂上まで、層ごとに断面の幅に合わせる）
DZ_LAYER = 3.0
RB = 45.5            # 玉の半径（円柱、断面の幅で切られる）
R_SOCKET = 49.5      # 椀の内側の半径
# 胴（J3〜J5 のリンク）の断面の半幅 w(z)（中立、測定 2026-09-30）
W_Z = [(2.0, 19.5), (4.0, 19.5), (6.0, 23.5), (8.0, 29.0), (10.0, 32.5), (14.0, 37.0), (20.0, 41.0), (30.0, 44.5), (40.0, 45.5), (50.0, 45.5), (60.0, 45.0), (70.0, 44.0), (75.0, 42.5), (77.0, 41.5), (80.0, 39.5), (85.0, 35.5), (88.0, 31.5), (90.0, 27.5), (92.0, 21.5), (93.0, 16.5)]

W_TAIL = [(2.0, 19.5), (6.0, 23.0), (10.0, 32.0), (14.0, 36.5), (20.0, 40.0), (30.0, 42.5), (40.0, 43.0), (60.0, 43.0), (70.0, 41.5), (75.0, 40.5), (77.0, 40.0), (93.0, 16.5)]


def half_width(z: float) -> float:
    zs = [a for a, _ in W_Z]; ws = [b for _, b in W_Z]
    return float(np.interp(z, zs, ws))

JOINTS = {           # 名前: (軸のリンク A, 軸の A のキー, B のキー, 軸 x（中立）, R_local (x, |y|))
    "J3": ("LINK1", "LINK2", 0.0, (59.5, 45.5)),
    "J4": ("LINK2", "LINK3", 95.0, (59.5, 45.5)),
    "J5": ("LINK3", "TAIL", 190.0, (60.0, 43.0)),
    "J2": ("NECKV", "LINK1", -95.0, (59.5, 34.0)),      # 首の玉 R34（実測の目安）。フラップの角は未測定
}


def _rot(deg: float) -> np.ndarray:
    t = math.radians(deg); c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s], [s, c]])


def body_pose(pose: str, key: str) -> tuple[float, np.ndarray]:
    tf = json.loads(TF.read_text())[pose]
    m = np.array(tf[key]).reshape(4, 4) if key in tf else np.eye(4)
    return math.degrees(math.atan2(m[1, 0], m[0, 0])), m[:2, 3] * 10.0


def axis_world(pose: str, joint: str) -> np.ndarray:
    a_key, _b, x, _ = JOINTS[joint]
    ang, t = body_pose(pose, a_key if a_key != "LINK2" else "LINK2")
    if a_key == "LINK2":
        return np.array([x, 0.0])
    return _rot(ang) @ np.array([x, 0.0]) + t


def strip_path(R: np.ndarray, w: float) -> list[np.ndarray]:
    """A の座標系、s = +（y > 0 側）。R = B の角。A の外形 = 半径 RB の円 ∩ |y| ≤ w。帯は 0.8 mm 浮かせて沿わせる。根元から先端まで（長さ L_F で切る）。"""
    rc = RB + 0.8; yb = w + 0.8
    rho = float(np.hypot(*R)); th = math.atan2(R[1], R[0])
    alpha = math.acos(min(1.0, rc / rho)) if rho > rc else 0.0
    ta = th + alpha
    pts = [R.copy()]
    corner = np.array([math.sqrt(max(rc * rc - yb * yb, 0.0)), yb])
    if ta < math.pi / 2 and rc * math.sin(ta) <= yb:
        pts.append(rc * np.array([math.cos(ta), math.sin(ta)]))
        a_end = math.asin(min(1.0, yb / rc))
        n = max(2, int((a_end - ta) / math.radians(3)))
        for a in np.linspace(ta, a_end, n + 1)[1:]:
            pts.append(rc * np.array([math.cos(a), math.sin(a)]))
    else:
        pts.append(corner)
    pts.append(np.array([-95.0, yb]))
    # 接点（pts[1]）までの長さ + TAIL_OVERLAP で切る
    L = float(np.linalg.norm(pts[1] - pts[0])) + TAIL_OVERLAP
    out = [pts[0]]; acc = 0.0
    for p in pts[1:]:
        d = float(np.linalg.norm(p - out[-1]))
        if acc + d >= L:
            out.append(out[-1] + (p - out[-1]) * ((L - acc) / d)); break
        out.append(p); acc += d
    return out


def _prism(poly_xy: np.ndarray, z0: float, z1: float) -> np.ndarray:
    """単純な多角形を z0〜z1 の柱の三角形に（外向きの向き）。poly_xy: (n, 2)。"""
    a = 0.5 * np.sum(poly_xy[:, 0] * np.roll(poly_xy[:, 1], -1) - np.roll(poly_xy[:, 0], -1) * poly_xy[:, 1])
    if a < 0:
        poly_xy = poly_xy[::-1]
    n = len(poly_xy); tris = []
    for i in range(n):
        p, q = poly_xy[i], poly_xy[(i + 1) % n]
        pb, qb, qt, pt = [p[0], p[1], z0], [q[0], q[1], z0], [q[0], q[1], z1], [p[0], p[1], z1]
        tris += [[pb, qb, qt], [pb, qt, pt]]
    return np.array(tris, float), poly_xy


def strip_mesh(path: list[np.ndarray], z0: float, z1: float, root_dir: np.ndarray | None = None, inner_ext: float = 0.0) -> np.ndarray:
    P = np.array(path)
    n = len(P)
    tang = np.zeros_like(P)
    tang[1:-1] = P[2:] - P[:-2]; tang[0] = P[1] - P[0]; tang[-1] = P[-1] - P[-2]
    tang /= np.linalg.norm(tang, axis=1)[:, None]
    nor = np.stack([tang[:, 1], -tang[:, 0]], 1)
    mid = P.mean(0)
    if np.sum(nor * (P - 0.0)) < 0:      # 原点（玉の中心）から外向き
        nor = -nor
    # 根元を B の中へ延ばす
    root = P[0] + (root_dir if root_dir is not None else -tang[0]) * ROOT_EXT
    P = np.vstack([root, P]); nor = np.vstack([nor[0], nor])
    inner = P - nor * inner_ext; outer = P + nor * T
    poly = np.vstack([inner, outer[::-1]])
    tris, poly = _prism(poly, z0, z1)
    # 上下の蓋: 帯を四角形の並びで
    n2 = len(inner)
    caps = []
    for i in range(n2 - 1):
        for (a, b, c) in ((inner[i], inner[i + 1], outer[i + 1]), (inner[i], outer[i + 1], outer[i])):
            cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
            if abs(cross) < 1e-12:
                continue
            up = cross > 0
            caps.append([[a[0], a[1], z1], [b[0], b[1], z1], [c[0], c[1], z1]] if up else [[a[0], a[1], z1], [c[0], c[1], z1], [b[0], b[1], z1]])
            caps.append([[a[0], a[1], z0], [c[0], c[1], z0], [b[0], b[1], z0]] if up else [[a[0], a[1], z0], [b[0], b[1], z0], [c[0], c[1], z0]])
    return np.vstack([tris, np.array(caps, float)])


def skirt_meshes(pose: str, joints=("J3", "J4", "J5"), z0: float = Z0, z1: float = Z1) -> list[np.ndarray]:
    """姿勢 pose の、全関節・両脇のフラップ（高さ方向は層ごとに胴の断面に沿う）の三角形メッシュ（世界座標、mm）の一覧。"""
    out = []
    layers = []
    z = z0
    while z < z1 - 1e-6:
        step = 1.0 if (z < 16.0 or z >= 75.0) else DZ_LAYER
        top = min(z + step, z1)
        layers.append((z, top)); z = top
    wl = [half_width(0.5 * (a + b)) for a, b in layers]
    for j in joints:
        a_key, b_key, x, (rx, ry) = JOINTS[j]
        angA, tA = body_pose(pose, a_key); angB, _ = body_pose(pose, b_key)
        c = axis_world(pose, j)
        phi = angB - angA
        for li, (za, zb) in enumerate(layers):
            w = wl[li]
            nb = [abs(w - wl[k]) for k in (li - 1, li + 1) if 0 <= k < len(wl)]
            ext = (max(nb) if nb else 0.0) + 0.3       # 隣の層との段差だけ内側へ厚くして、板をつなげる
            zm = 0.5 * (za + zb)
            # B の角: 背板の層（z > 77）は縁の弧が脇に直角に当たる (58, w)。胴の層は面取りの直線（縁 (44.5, 32) → (59.5, 45.5)）を w で切る。w < 32 は椀の弧
            wb = float(np.interp(zm, [a for a, _ in W_TAIL], [b for _, b in W_TAIL])) if j == "J5" else w
            wb = min(wb, w)
            if zm > 77:
                rx_z, ry_z = 58.0, wb
            elif wb >= 32.0:
                rx_z, ry_z = 44.5 + (wb - 32.0) * (rx - 44.5) / (ry - 32.0), wb
            else:
                rx_z, ry_z = math.sqrt(R_SOCKET ** 2 - wb * wb), wb
            for s in (1, -1):
                R = _rot(phi) @ np.array([rx_z, s * ry_z])
                Rm = R if s == 1 else np.array([R[0], -R[1]])
                path = strip_path(Rm, w)
                if s == -1:
                    path = [np.array([p[0], -p[1]]) for p in path]
                m = strip_mesh(path, za - (0.25 if za > z0 else 0.0), zb, _rot(phi) @ np.array([1.0, 0.0]), ext)
                xy = m[:, :, :2].reshape(-1, 2) @ _rot(angA).T + c
                wm = m.copy(); wm[:, :, :2] = xy.reshape(-1, 3, 2)
                out.append(wm)
    return out


# ---- あご・首の前（J1 ピッチ）のフラップ -------------------------------------------------
CHIN_ROOT0 = (-158.2, 9.6)      # 頭の下の後ろの角（下の後ろの埋め物の下の角）。頭の座標（中立）の x, z
NECK_TIP = (-154.3, 9.0)        # 首の腹の先端（中立）の x, z。ここから +x へ腹の線 z ≈ 9.0 − 0.05 (x + 154)
CHIN_HALF_Y = 24.0
CHIN_TAIL = 8.0


def _tf4(pose: str, key: str) -> np.ndarray:
    m = np.array(json.loads(TF.read_text())[pose][key]).reshape(4, 4)
    m[:3, 3] *= 10.0
    return m


def chin_flap_mesh(pose: str) -> np.ndarray:
    """頭の下の後ろの角に根元を付け、首の腹の先端の下（0.8 mm 浮かせて）へ張る帯（首の座標で XZ の折れ線、y ±CHIN_HALF_Y に押し出し）。"""
    Tn = _tf4(pose, "NECKV"); Th = _tf4(pose, "HEADV")
    loc = np.linalg.inv(Tn) @ Th                      # 頭 → 首の座標
    r0 = loc @ np.array([CHIN_ROOT0[0], 0.0, CHIN_ROOT0[1], 1.0])
    fwd = loc[:3, :3] @ np.array([-1.0, 0.0, 0.0])    # 頭の前（首の座標）
    root = np.array([r0[0], r0[2]])
    f2 = np.array([fwd[0], fwd[2]]); f2 /= np.linalg.norm(f2)
    tip = np.array([NECK_TIP[0], NECK_TIP[1] - 0.8])
    def zb(x): return NECK_TIP[1] - 0.05 * (x - NECK_TIP[0]) - 0.8
    tail = np.array([NECK_TIP[0] + CHIN_TAIL, zb(NECK_TIP[0] + CHIN_TAIL)])
    P = np.array([root + f2 * ROOT_EXT, root, tip, tail])
    tang = np.zeros_like(P); tang[1:-1] = P[2:] - P[:-2]; tang[0] = P[1] - P[0]; tang[-1] = P[-1] - P[-2]
    tang /= np.linalg.norm(tang, axis=1)[:, None]
    nor = np.stack([tang[:, 1], -tang[:, 0]], 1)
    if np.sum(nor[:, 1]) > 0:                          # 外側 = 下向き
        nor = -nor
    poly = np.vstack([P, (P + nor * T)[::-1]])
    tris, poly = _prism(poly, -CHIN_HALF_Y, CHIN_HALF_Y)
    # 蓋（帯の四角形）
    caps = []
    inner, outer = P, P + nor * T
    for i in range(len(P) - 1):
        for (a, b, c) in ((inner[i], inner[i + 1], outer[i + 1]), (inner[i], outer[i + 1], outer[i])):
            cr = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
            if abs(cr) < 1e-12:
                continue
            up = cr > 0
            caps.append([[a[0], a[1], CHIN_HALF_Y], [b[0], b[1], CHIN_HALF_Y], [c[0], c[1], CHIN_HALF_Y]] if up else [[a[0], a[1], CHIN_HALF_Y], [c[0], c[1], CHIN_HALF_Y], [b[0], b[1], CHIN_HALF_Y]])
            caps.append([[a[0], a[1], -CHIN_HALF_Y], [c[0], c[1], -CHIN_HALF_Y], [b[0], b[1], -CHIN_HALF_Y]] if up else [[a[0], a[1], -CHIN_HALF_Y], [b[0], b[1], -CHIN_HALF_Y], [c[0], c[1], -CHIN_HALF_Y]])
    m = np.vstack([tris, np.array(caps, float)])
    # (a, b, c) = (x, z, y) → (x, y, z)。軸の入れ替えで向きが反転するので頂点の順を逆にする
    w = m[:, ::-1, :][:, :, [0, 2, 1]]
    world = (Tn[:3, :3] @ w.reshape(-1, 3).T).T + Tn[:3, 3]
    return world.reshape(-1, 3, 3)


# ---- 椀の縁の「ワイパー」（唇）: 椀の縁（弧の端）から玉の面へ触れる柔らかい唇 --------------------------------
LIP_ANGLE = 40.0      # 椀の弧の端の角度（椀の座標、中心線から）。実測: 椀の内側 R 49.5 の弧の端が (37.9, ±31.8) = 40°
LIP_R0, LIP_R1 = 45.0, 50.0   # 唇の内側（玉の面 R 45.5 に軽く触れる）と外側（椀の縁 R 49.5 を少し越える）
LIP_T = 1.2
LIP_Z0, LIP_Z1 = 8.0, 77.0


def wiper_meshes(pose: str, joints=("J3", "J4", "J5"), z0: float = LIP_Z0, z1: float = LIP_Z1) -> list[np.ndarray]:
    out = []
    for j in joints:
        a_key, b_key, x, _ = JOINTS[j]
        angB, _t = body_pose(pose, b_key)
        c = axis_world(pose, j)
        for s in (1, -1):
            th = math.radians(LIP_ANGLE) * s
            u = np.array([math.cos(th), math.sin(th)]); n = np.array([-math.sin(th), math.cos(th)])
            poly = np.array([LIP_R0 * u - LIP_T / 2 * n, LIP_R1 * u - LIP_T / 2 * n, LIP_R1 * u + LIP_T / 2 * n, LIP_R0 * u + LIP_T / 2 * n])
            poly = poly @ _rot(angB).T + c
            tris, _ = _prism(poly, z0, z1)
            caps = []
            pp = _prism(poly, z0, z1)[1]
            for k in range(1, len(pp) - 1):
                caps.append([[*pp[0], z1], [*pp[k], z1], [*pp[k + 1], z1]])
                caps.append([[*pp[0], z0], [*pp[k + 1], z0], [*pp[k], z0]])
            out.append(np.vstack([tris, np.array(caps, float)]))
    return out


# ---- 層ごとの唇（椀の縁の角から、玉の外形の最も近い点まで張る）------------------------------------------
LIP2_R = 50.5          # 椀の縁の角の半径（椀の内側 R 49.5 の外）
RA = 46.0              # 玉の半径（測定 2026-09-30: 縁の角の角度での玉の面）
LIP2_DZ = 2.0


def _outline_A(w: float, n: int = 400) -> np.ndarray:
    """A の座標系、玉の外形（半径 RA の円 ∩ |y| ≤ w）と脇の直線（x ≤ 0）の点列。"""
    pts = []
    for a in np.linspace(-math.pi / 2, math.pi / 2, n):
        x, y = RA * math.cos(a), RA * math.sin(a)
        pts.append((x, max(-w, min(w, y))))
    for x in np.linspace(0.0, -30.0, 40):
        pts += [(x, w), (x, -w)]
    return np.array(pts)


def lip_meshes(pose: str, joints=("J3", "J4", "J5"), z0: float = 8.0, z1: float = 76.0, gap: float = 0.5) -> list[np.ndarray]:
    """椀（B）の縁の角 L（角度 LIP_ANGLE、半径 LIP2_R）から玉（A）の外形の最も近い点まで、層（LIP2_DZ mm）ごとに張る薄い唇（厚さ LIP_T）。"""
    out = []
    z = z0
    layers = []
    while z < z1 - 1e-6:
        layers.append((z, min(z + LIP2_DZ, z1))); z += LIP2_DZ
    for j in joints:
        a_key, b_key, x, _ = JOINTS[j]
        angA, tA = body_pose(pose, a_key); angB, _t = body_pose(pose, b_key)
        c = axis_world(pose, j)
        phi = angB - angA
        for (za, zb) in layers:
            w = half_width(0.5 * (za + zb))
            ol = _outline_A(w)
            for s in (1, -1):
                th = math.radians(LIP_ANGLE) * s
                L = _rot(phi) @ (LIP2_R * np.array([math.cos(th), math.sin(th)]))     # A の座標系
                k = int(np.argmin(np.linalg.norm(ol - L, axis=1)))
                P = ol[k]
                d = P - L; dist = float(np.linalg.norm(d)); u = d / dist
                P = P - u * gap                                                        # 玉の面から gap だけ浮かせる
                path = [L, P]
                m = strip_mesh(path, za - (0.2 if za > z0 else 0.0), zb, root_dir=-u, inner_ext=0.0)
                xy = m[:, :, :2].reshape(-1, 2) @ _rot(angA).T + c
                wm = m.copy(); wm[:, :, :2] = xy.reshape(-1, 3, 2)
                out.append(wm)
    return out


# ---- 実験: 玉を高さ全体で完全な円柱 R46 にする（樽形の絞りで玉の脇が痩せる所を埋める）-----------------------------
def full_ball_meshes(pose: str, joints=("J3", "J4", "J5"), z0: float = 8.0, z1: float = 76.0, r: float = 46.0) -> list[np.ndarray]:
    out = []
    for j in joints:
        a_key, b_key, x, _ = JOINTS[j]
        angA, tA = body_pose(pose, a_key); c = axis_world(pose, j)
        ang = np.linspace(-math.pi / 2, math.pi / 2, 61)
        poly = np.vstack([np.stack([r * np.cos(ang), r * np.sin(ang)], 1), np.array([[-10.0, r], [-10.0, -r]])])
        poly = poly @ _rot(angA).T + c
        tris, pp = _prism(poly, z0, z1)
        caps = []
        for k in range(1, len(pp) - 1):
            caps.append([[*pp[0], z1], [*pp[k], z1], [*pp[k + 1], z1]]); caps.append([[*pp[0], z0], [*pp[k + 1], z0], [*pp[k], z0]])
        out.append(np.vstack([tris, np.array(caps, float)]))
    return out
