"""合成画像: 4 枚（全消灯 / 通常 / 斜め照明 / 線光）を幾何から描く。**KINEMATIC_SIM 相当の模擬。実写ではない。**

床の模様、物体（円盤: 直径・高さ・反射率・鏡面）、汚れ（高さ 0）。斜め照明は横の LED からの影（長さ = H / tan(照射角)）、
線光は光の面（`LightPlane`）と表面の交線。鏡面の物では線が乗らずに消える（ボタン電池・磁石を模す）。
評価用データセットの「正解」を生成する（`dataset.py`）。実写データが来るまでのパイプライン検証用。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from serpens.floorwatch.geometry import Camera, LightPlane


@dataclass(frozen=True)
class Disc:
    """床の上の物（円盤で近似）。"""

    x_mm: float
    y_mm: float
    diameter_mm: float
    height_mm: float
    albedo: float = 0.8            # 0〜1（床は約 0.5）
    specular: bool = False         # 鏡面: 線光が乗らない
    kind: str = "unknown"


@dataclass(frozen=True)
class Stain:
    x_mm: float
    y_mm: float
    diameter_mm: float
    albedo: float = 0.3            # 床より暗い模様・汚れ（高さ 0）


@dataclass
class Scene:
    objects: list[Disc] = field(default_factory=list)
    stains: list[Stain] = field(default_factory=list)
    floor_albedo: float = 0.5
    seed: int = 0


@dataclass(frozen=True)
class Lighting:
    normal_lux: float = 200.0      # 相対値（画素値のスケール）
    raking_lux: float = 200.0
    raking_angle_deg: float = 15.0 # 床に対する照射角（影の長さ = H / tan）
    raking_from_left: bool = True  # LED はカメラの左（影は右へ伸びる）
    line_lux: float = 400.0
    line_width_mm: float = 3.0
    ambient_lux: float = 8.0       # 全消灯でも残る光
    shadow_factor: float = 0.35
    noise_sigma: float = 2.0


def _floor_texture(cam: Camera, xy: np.ndarray, rng: np.random.Generator, seed: int) -> np.ndarray:
    """木目風の模様（前後方向の縞 + 低周波のむら）。0.8〜1.2 の倍率。"""
    x, y = xy[..., 0], xy[..., 1]
    grain = 0.08 * np.sin(x / 9.0 + 0.7 * np.sin(y / 40.0 + seed)) + 0.05 * np.sin(x / 2.3 + seed)
    blotch = 0.06 * np.sin(x / 55.0 + y / 70.0 + seed * 1.3)
    return 1.0 + grain + blotch


class Renderer:
    """幾何から 4 枚を描く。"""

    def __init__(self, cam: Camera, plane: LightPlane, lighting: Lighting) -> None:
        self.cam, self.plane, self.lt = cam, plane, lighting
        vs, us = np.mgrid[0:cam.height_px, 0:cam.width_px].astype(float)
        self.floor_xy = np.full((cam.height_px, cam.width_px, 2), np.nan)
        self.floor_dist = np.full((cam.height_px, cam.width_px), np.nan)
        for v in range(cam.height_px):
            for u in (0, cam.width_px - 1):
                pass
        # 行ごとに視線を計算（ベクトル化）
        xa, ya, za = cam._axes
        d = ((us - cam.cx) / cam.f_px)[..., None] * xa + ((vs - cam.cy) / cam.f_px)[..., None] * ya + za
        dz = d[..., 2]
        ok = dz < -1e-9
        s = np.where(ok, -cam.height_mm / np.where(ok, dz, 1.0), np.nan)
        p = cam.center + s[..., None] * d
        self.floor_xy[ok] = p[ok][:, :2]
        self.floor_dist[ok] = s[ok]
        self.dir = d

    def _surface(self, scene: Scene) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """各画素の表面: 高さ、反射率、鏡面、物体の index（-1 = 床）。物体の上面を投影して塗る。"""
        H, W = self.cam.height_px, self.cam.width_px
        height = np.zeros((H, W))
        albedo = np.full((H, W), scene.floor_albedo)
        specular = np.zeros((H, W), bool)
        owner = np.full((H, W), -1)
        # 汚れ（床の上、高さ 0）
        for st in scene.stains:
            r = np.hypot(self.floor_xy[..., 0] - st.x_mm, self.floor_xy[..., 1] - st.y_mm)
            m = r <= st.diameter_mm / 2
            albedo[m] = st.albedo
        # 物体: 高さ H の水平面と視線の交点が円盤内なら、その画素は物体の上面
        for i, ob in enumerate(scene.objects):
            dz = self.dir[..., 2]
            ok = dz < -1e-9
            s = np.where(ok, (ob.height_mm - self.cam.height_mm) / np.where(ok, dz, 1.0), np.nan)
            p = self.cam.center + s[..., None] * self.dir
            r = np.hypot(p[..., 0] - ob.x_mm, p[..., 1] - ob.y_mm)
            top = ok & (r <= ob.diameter_mm / 2)
            height[top], albedo[top], specular[top], owner[top] = ob.height_mm, ob.albedo, ob.specular, i
        return height, albedo, specular, owner

    def render(self, scene: Scene) -> dict[str, np.ndarray]:
        cam, lt = self.cam, self.lt
        rng = np.random.default_rng(scene.seed)
        height, albedo, specular, owner = self._surface(scene)
        tex = _floor_texture(cam, np.nan_to_num(self.floor_xy), rng, scene.seed)
        tex = np.where(owner >= 0, 1.0, tex)
        base = albedo * tex
        # 斜め照明の影: 床画素で、LED と反対側に物体の足元から H/tan(角) だけ伸びる範囲
        shadow = np.zeros_like(base, bool)
        sign = 1.0 if lt.raking_from_left else -1.0
        L = lambda h: h / math.tan(math.radians(lt.raking_angle_deg))  # noqa: E731
        fx, fy = self.floor_xy[..., 0], self.floor_xy[..., 1]
        for ob in scene.objects:
            dx = (fx - ob.x_mm) * sign
            dy = np.abs(fy - ob.y_mm)
            half = ob.diameter_mm / 2
            inside = (dx >= -half) & (dx <= half + L(ob.height_mm)) & (dy <= half)
            shadow |= inside & (owner < 0)
        # 線光: 表面と光の面の交線。床は面上の点（|n·p − d| 小）。物体の上面は高さ H の面と光の面の交線
        line = np.zeros_like(base)
        hw = lt.line_width_mm / 2
        # 各画素の表面点（高さ height の水平面との交点）
        dz = self.dir[..., 2]
        ok = dz < -1e-9
        s = np.where(ok, (height - cam.height_mm) / np.where(ok, dz, 1.0), np.nan)
        surf = cam.center + s[..., None] * self.dir
        dist = np.abs(surf @ self.plane.normal - self.plane.d) / max(abs(self.plane.normal[0]), 1e-9)   # x 方向の距離 [mm]
        lit = ok & (dist <= hw)
        lit &= ~specular                                     # 鏡面では線が乗らない（消える）
        line[lit] = 1.0
        def img(lux: float, mult: np.ndarray) -> np.ndarray:
            v = lt.ambient_lux + lux * base * mult
            noise = np.random.default_rng().normal(0, lt.noise_sigma, v.shape)   # 撮影ごとに違う雑音（基準床と同じにしない）
            return np.clip(v + noise, 0, 255).astype(np.uint8)
        one = np.ones_like(base)
        return {
            "dark": img(0.0, one),
            "normal": img(lt.normal_lux, one),
            "raking": img(lt.raking_lux, np.where(shadow, lt.shadow_factor, 1.0)),
            "line": img(lt.line_lux, line + 0.03),
        }


def default_lighting(cfg: dict[str, Any]) -> Lighting:
    r, ll, syn = cfg["floor_watch"]["raking"], cfg["floor_watch"]["line_light"], cfg["floor_watch"]["synthetic"]
    return Lighting(raking_angle_deg=float(r["angle_deg"][0]), line_width_mm=float(ll["width_mm_initial"]),
                    noise_sigma=float(syn["noise_sigma"]), shadow_factor=float(syn["shadow_factor"]))
