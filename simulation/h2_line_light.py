"""H2 の Hardware Gap: ライン光をどこに置くか（頬 = 横基線 / 眉 = 縦基線）の三角測量の比較。**source は GEOMETRY_SIM。**

Design の問い（integration-log ENTRY-0006(a) / OPEN-SERPENS-DESIGN-001）: 眉の段と頬のどちらで三角測量が成り立つか。
実機も実写も無いので、頭カメラの幾何（`serpens.floorwatch.geometry.Camera`、値はすべて ASSUMED）と、任意の位置の
投光部から出る光の面で、次を数値で出す:

  - 感度 S [px/mm]   … 物の上面が 1 mm 高いと、画像の線が線と直角の向きに何 px ずれるか
  - 分解能 σ_H [mm] … 線の重心の誤差 σ_c [px] / S
  - 較正誤差の偏り   … 光の面の傾きを δ だけ誤って較正したとき、床の上で出る見かけの高さ
  - 線の長さ         … 床の上の線のうち、画像に入って見える範囲の長さ [mm]
  - 狙いの手段       … 線を候補へ当てるのに何を動かすか（頬 = 頭 yaw で左右 / 眉 = 胴の前後の微動）

限界（**このモデルで言えないこと**）: 線の太さ・ボケ・床での散乱（カーペットで線が太る）、鏡面での途切れ、
物自身が光をさえぎる影（投光部が低いほど長い）、鼻先による光路のさえぎり（Design の側面視の計算に任せる）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from serpens.floorwatch.geometry import Camera, LightPlane

SOURCE = "GEOMETRY_SIM"


@dataclass(frozen=True)
class LinePlacement:
    """投光部の位置 S と、光の面が床と交わる線（点 P0 と向き dvec、水平）。座標は geometry.py と同じ（x 右 / y 前 / z 上 mm）。"""

    name: str
    source_mm: tuple[float, float, float]
    floor_point_mm: tuple[float, float, float]
    floor_dir: tuple[float, float, float]
    aim_by: str                                   # "head_yaw"（線を左右へ）/ "neck_pitch"（線を前後へ）
    note: str = ""

    def plane(self) -> LightPlane:
        s, p0, d = (np.asarray(v, float) for v in (self.source_mm, self.floor_point_mm, self.floor_dir))
        n = np.cross(p0 - s, d)
        n = n / np.linalg.norm(n)
        return LightPlane(n, float(n @ p0))


def _rotate(v: np.ndarray, axis: np.ndarray, deg: float) -> np.ndarray:
    a = axis / np.linalg.norm(axis)
    t = math.radians(deg)
    return v * math.cos(t) + np.cross(a, v) * math.sin(t) + a * (a @ v) * (1 - math.cos(t))


def _in_image(cam: Camera, uv: tuple[float, float, float], margin_px: float = 0.0) -> bool:
    u, v, zc = uv
    return zc > 0 and margin_px <= u <= cam.width_px - margin_px and margin_px <= v <= cam.height_px - margin_px


def _line_samples(cam: Camera, lp: LinePlacement, span_mm: float = 400.0, step_mm: float = 0.5) -> np.ndarray:
    """床の線のうち画像に写る点（世界座標）。"""
    p0, d = np.asarray(lp.floor_point_mm, float), np.asarray(lp.floor_dir, float)
    d = d / np.linalg.norm(d)
    pts = [p0 + t * d for t in np.arange(-span_mm / 2, span_mm / 2 + step_mm, step_mm)]
    return np.array([p for p in pts if _in_image(cam, cam.project(p))])


def sensitivity_px_per_mm(cam: Camera, lp: LinePlacement, floor_pt: np.ndarray, h_mm: float = 1.0) -> float:
    """床の点での線と、高さ h の面の上の線（光の面 ∩ z=h）の、画像上の距離（線と直角の向き）/ h。"""
    s, p0, d = (np.asarray(v, float) for v in (lp.source_mm, lp.floor_point_mm, lp.floor_dir))
    d = d / np.linalg.norm(d)
    t = (s[2] - h_mm) / s[2]                                   # S → 床の線を結ぶ線分で z = h になる割合
    base_h = s + t * (p0 - s)                                  # 高さ h の線の 1 点（光の面の上）
    lam = float((floor_pt - p0) @ d)
    a_f, b_f = cam.project(floor_pt - 2 * d)[:2], cam.project(floor_pt + 2 * d)[:2]
    # 高さ h の線の上で、床の点と同じ画像の位置に近い点を探す（線の向きに沿って走査）
    q = cam.project(floor_pt)[:2]
    cands = [base_h + (lam + k) * d for k in np.linspace(-80, 80, 3201)]
    uvs = np.array([cam.project(c)[:2] for c in cands])
    tang = np.subtract(b_f, a_f)
    tang = tang / np.linalg.norm(tang)
    normal = np.array([-tang[1], tang[0]])
    along = (uvs - q) @ tang                                   # 画像で床の点と同じ「線に沿った位置」の点
    i = int(np.nanargmin(np.abs(along)))
    return float(abs((uvs[i] - q) @ normal)) / h_mm


def calibration_error_mm(cam: Camera, lp: LinePlacement, floor_pts: np.ndarray, err_deg: float, axis: str,
                         h_mm: float) -> float:
    """光の面を err_deg だけ誤って較正したとき、真の高さ h_mm の点の高さの誤差の最大 [mm]。

    axis: "tilt" = 床の線のまわり（投光部の傾き）/ "yaw" = 鉛直軸まわり（投光部の向き）。
    **高さは局所の床の線との差で測る**（detect の line_context と同じ）ので、床の線に出る見かけの高さは引く。"""
    true = lp.plane()
    p0 = np.asarray(lp.floor_point_mm, float)
    d = np.asarray(lp.floor_dir, float)
    d = d / np.linalg.norm(d)
    rot_axis = d if axis == "tilt" else np.array([0.0, 0.0, 1.0])
    wrong_n = _rotate(true.normal, rot_axis, err_deg)
    wrong = LightPlane(wrong_n, float(wrong_n @ p0))
    s = np.asarray(lp.source_mm, float)
    t = (s[2] - h_mm) / s[2]
    base_h = s + t * (p0 - s)
    worst = 0.0
    for p in floor_pts:
        lam = float((p - p0) @ d)
        q = base_h + lam * d                                  # 光の面の上で高さ h の点（床の点と同じ線に沿った位置）
        uq, vq, zq = cam.project(q)
        uf, vf, zf = cam.project(p)
        if zq <= 0 or zf <= 0:
            continue
        hq, hf = wrong.height_at(cam, uq, vq), wrong.height_at(cam, uf, vf)
        if hq is None or hf is None:
            continue
        worst = max(worst, abs((hq - hf) - h_mm))
    return worst


def aim_travel_mm_per_deg(lp: LinePlacement, pivot_back_mm: float, view_y_mm: float = 90.0,
                          delta_deg: float = 1.0) -> float:
    """頭を回して線を動かすとき、床の線が 1° あたり何 mm 動くか（視野中ほど view_y_mm で）。

    カメラと投光部はどちらも頭に固定なので、頭を回すと光の面ごと回る（線は画像の中ではほぼ同じ所に留まり、
    床の上を掃く）。aim_by = head_yaw: 鉛直軸まわり（首の yaw 軸はカメラの pivot_back_mm 後ろ）→ 線の横の移動 /
    neck_pitch: J1 首 pitch（カメラの pivot_back_mm 後ろ・同じ高さ、ASSUMPTION）まわり → 線の前後の移動。"""
    plane = lp.plane()
    s = np.asarray(lp.source_mm, float)
    pivot = np.array([0.0, -pivot_back_mm, s[2] if lp.aim_by == "head_yaw" else lp.source_mm[2]])
    axis = np.array([0.0, 0.0, 1.0]) if lp.aim_by == "head_yaw" else np.array([1.0, 0.0, 0.0])
    p_on = plane.normal * plane.d                            # 光の面の 1 点

    def floor_hit(deg: float) -> float:
        n = _rotate(plane.normal, axis, deg)
        p = pivot + _rotate(p_on - pivot, axis, deg)
        dd = float(n @ p)
        if lp.aim_by == "head_yaw":                          # z = 0, y = view_y の上で n·(x, y, 0) = d を x について解く
            return (dd - n[1] * view_y_mm) / n[0]
        return (dd - n[0] * 0.0) / n[1]                       # z = 0, x = 0 の上で y を解く
    return abs(floor_hit(delta_deg) - floor_hit(0.0)) / delta_deg


def evaluate(cam: Camera, lp: LinePlacement, centroid_sigma_px: float = 0.2, err_deg: float = 0.5,
             view_y_mm: tuple[float, float] = (38.0, 147.0)) -> dict[str, Any]:
    pts = _line_samples(cam, lp)
    pts = np.array([p for p in pts if view_y_mm[0] <= p[1] <= view_y_mm[1]]) if len(pts) else pts
    if len(pts) == 0:
        return {"name": lp.name, "visible": False}
    sens = np.array([sensitivity_px_per_mm(cam, lp, p) for p in pts[:: max(1, len(pts) // 25)]])
    length = float(np.linalg.norm(pts[-1] - pts[0]))
    s_min, s_med = float(sens.min()), float(np.median(sens))
    out: dict[str, Any] = {"name": lp.name, "visible": True, "aim_by": lp.aim_by, "line_length_mm": round(length, 1),
                           "sens_px_per_mm_min": round(s_min, 2), "sens_px_per_mm_median": round(s_med, 2),
                           "sigma_h_mm_worst": round(centroid_sigma_px / s_min, 4)}
    for axis in ("tilt", "yaw"):
        for h in (1.5, 5.0):
            out[f"calib_{axis}{err_deg}deg_err_mm_h{h}"] = round(calibration_error_mm(cam, lp, pts, err_deg, axis, h), 3)
    for back in (40.0, 60.0, 80.0, 104.0):
        out[f"aim_mm_per_deg_pivot{int(back)}"] = round(aim_travel_mm_per_deg(lp, back), 2)
    out["note"] = lp.note
    return out


def placements(cfg: dict[str, Any]) -> list[LinePlacement]:
    """比べる置き方。頬の現行は config の line_light（lateral 30 / height 30 / tilt 45°）。眉は Design の案（Z64〜70）。"""
    ll = cfg["floor_watch"]["line_light"]
    lat, hgt = float(ll["lateral_mm"]), float(ll["height_mm"])
    out = [LinePlacement("cheek_current", (lat, 0.0, hgt), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0), "head_yaw",
                         "現行（config）: 横基線 30mm・45° 内向き")]
    for lat2 in (15.0, 45.0):
        out.append(LinePlacement(f"cheek_lat{int(lat2)}", (lat2, 0.0, hgt), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0), "head_yaw",
                                 f"横基線 {lat2:.0f}mm"))
    for zs in (64.0, 70.0):
        for y0 in (60.0, 71.0, 90.0):
            out.append(LinePlacement(f"brow_z{int(zs)}_y{int(y0)}", (0.0, 0.0, zs), (0.0, y0, 0.0), (1.0, 0.0, 0.0),
                                     "neck_pitch", f"眉の段: 縦基線 {zs - hgt:.0f}mm、床の線は {y0:.0f}mm 先を左右に"))
    return out
