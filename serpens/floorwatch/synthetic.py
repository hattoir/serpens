"""合成画像: 4 枚（全消灯 / 通常 / 斜め照明 / 線光）+ 動き確認用の通常 2 枚目を幾何から描く。**模擬。実写ではない。**

照明は実機の位置に合わせる（2026-09-26 レビュー）:
  通常照明 … レンズのすぐ横（影はほぼ出ない）
  斜め照明 … あごの位置（床から約 6mm、カメラの真下付近）から前向き。影は物の**奥**（画像の上側）に出る。
             視野中心の 1 円玉（1.5mm、64mm 先）で 約 2cm = H·d/(h_led − H)。H ≥ h_led の物は影が長く伸びる（上限あり）
  線光     … 光の面（`LightPlane`）と表面の交線。鏡面の物では線が乗らずに消える
床の段差（線状: 溝と段差、0.2〜0.5mm）も描ける（誤報の主犯候補）。
"""
from __future__ import annotations

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


@dataclass(frozen=True)
class Seam:
    """床の継ぎ目。along: "y"（前後方向、線光と平行）/ "x"（左右方向、線光を横切る）。

    kind: "step"（pos より先が height_mm 高い）/ "groove"（幅 width_mm の溝、深さ height_mm）
    """

    along: str
    pos_mm: float
    height_mm: float
    kind: str = "groove"
    width_mm: float = 1.5


@dataclass
class Scene:
    objects: list[Disc] = field(default_factory=list)
    stains: list[Stain] = field(default_factory=list)
    seams: list[Seam] = field(default_factory=list)
    floor_albedo: float = 0.5
    seed: int = 0


@dataclass(frozen=True)
class Lighting:
    normal_lux: float = 200.0
    raking_lux: float = 200.0
    raking_led_height_mm: float = 6.0      # あご
    raking_led_y_mm: float = 0.0           # カメラの真下付近（前後位置）
    shadow_max_mm: float = 150.0
    line_lux: float = 400.0
    line_width_mm: float = 3.0
    ambient_lux: float = 8.0
    shadow_factor: float = 0.35
    noise_sigma: float = 2.0


def _floor_texture(xy: np.ndarray, seed: int) -> np.ndarray:
    """木目風の模様（前後方向の縞 + 低周波のむら）。"""
    x, y = xy[..., 0], xy[..., 1]
    grain = 0.08 * np.sin(x / 9.0 + 0.7 * np.sin(y / 40.0 + seed)) + 0.05 * np.sin(x / 2.3 + seed)
    blotch = 0.06 * np.sin(x / 55.0 + y / 70.0 + seed * 1.3)
    return 1.0 + grain + blotch


def _seam_height(fx: np.ndarray, fy: np.ndarray, seams: list[Seam]) -> np.ndarray:
    h = np.zeros_like(fx)
    for s in seams:
        c = fx if s.along == "y" else fy
        if s.kind == "step":
            h = h + np.where(c > s.pos_mm, s.height_mm, 0.0)
        else:
            h = h - np.where(np.abs(c - s.pos_mm) <= s.width_mm / 2, s.height_mm, 0.0)
    return h


class Renderer:
    """幾何から画像を描く。"""

    def __init__(self, cam: Camera, plane: LightPlane, lighting: Lighting) -> None:
        self.cam, self.plane, self.lt = cam, plane, lighting
        vs, us = np.mgrid[0:cam.height_px, 0:cam.width_px].astype(float)
        xa, ya, za = cam._axes
        d = ((us - cam.cx) / cam.f_px)[..., None] * xa + ((vs - cam.cy) / cam.f_px)[..., None] * ya + za
        self.dir = d
        dz = d[..., 2]
        ok = dz < -1e-9
        s = np.where(ok, -cam.height_mm / np.where(ok, dz, 1.0), np.nan)
        p = cam.center + s[..., None] * d
        self.floor_xy = np.full((cam.height_px, cam.width_px, 2), np.nan)
        self.floor_xy[ok] = p[ok][:, :2]

    def _surface(self, scene: Scene) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """各画素の表面: 高さ、反射率、鏡面、物体 index（-1 = 床）。"""
        H, W = self.cam.height_px, self.cam.width_px
        fx, fy = np.nan_to_num(self.floor_xy[..., 0]), np.nan_to_num(self.floor_xy[..., 1])
        height = _seam_height(fx, fy, scene.seams)                      # 継ぎ目は視差を無視して床の高さ場に
        albedo = np.full((H, W), scene.floor_albedo)
        for s in scene.seams:                                            # 溝の中は暗い、段差の縁は細い線
            c = fx if s.along == "y" else fy
            if s.kind == "groove":
                albedo = np.where(np.abs(c - s.pos_mm) <= s.width_mm / 2, albedo * 0.6, albedo)
            else:
                albedo = np.where(np.abs(c - s.pos_mm) <= 0.4, albedo * 0.7, albedo)
        specular = np.zeros((H, W), bool)
        owner = np.full((H, W), -1)
        for st in scene.stains:
            m = np.hypot(fx - st.x_mm, fy - st.y_mm) <= st.diameter_mm / 2
            albedo[m] = st.albedo
        dz = self.dir[..., 2]
        ok = dz < -1e-9
        for i, ob in enumerate(scene.objects):
            s = np.where(ok, (ob.height_mm - self.cam.height_mm) / np.where(ok, dz, 1.0), np.nan)
            p = self.cam.center + s[..., None] * self.dir
            top = ok & (np.hypot(p[..., 0] - ob.x_mm, p[..., 1] - ob.y_mm) <= ob.diameter_mm / 2)
            height[top], albedo[top], specular[top], owner[top] = ob.height_mm, ob.albedo, ob.specular, i
        return height, albedo, specular, owner

    def _shadow(self, scene: Scene, owner: np.ndarray) -> np.ndarray:
        """あごの LED（高さ h_led、前後 y_led、真下）からの影: 物の奥（+y）へ H·d/(h_led − H) 伸びる。"""
        lt = self.lt
        fx, fy = self.floor_xy[..., 0], self.floor_xy[..., 1]
        shadow = np.zeros(fx.shape, bool)
        for ob in scene.objects:
            d = ob.y_mm - lt.raking_led_y_mm
            denom = lt.raking_led_height_mm - ob.height_mm
            length = lt.shadow_max_mm if denom <= 0 else min(lt.shadow_max_mm, ob.height_mm * d / denom)
            half = ob.diameter_mm / 2
            dx = fx - ob.x_mm
            rear = ob.y_mm + np.sqrt(np.clip(half * half - dx * dx, 0.0, None))     # 円盤の奥の縁から影が始まる
            inside = (np.abs(dx) <= half) & (fy >= rear) & (fy <= rear + length)
            shadow |= inside & (owner < 0)
        for sm in scene.seams:                                   # 線光を横切る段差も影を落とす（実機の誤報の候補）
            if sm.along == "x" and sm.kind == "step":
                denom = lt.raking_led_height_mm - sm.height_mm
                length = lt.shadow_max_mm if denom <= 0 else sm.height_mm * (sm.pos_mm - lt.raking_led_y_mm) / denom
                shadow |= (fy > sm.pos_mm) & (fy <= sm.pos_mm + length) & (owner < 0)
        return shadow

    def render(self, scene: Scene) -> dict[str, np.ndarray]:
        cam, lt = self.cam, self.lt
        height, albedo, specular, owner = self._surface(scene)
        tex = np.where(owner >= 0, 1.0, _floor_texture(np.nan_to_num(self.floor_xy), scene.seed))
        base = albedo * tex
        shadow = self._shadow(scene, owner)
        # 線光: 表面（高さ height の水平面）と光の面の交線。x 方向の距離で幅を切る
        dz = self.dir[..., 2]
        ok = dz < -1e-9
        s = np.where(ok, (height - cam.height_mm) / np.where(ok, dz, 1.0), np.nan)
        surf = cam.center + s[..., None] * self.dir
        dist = np.abs(surf @ self.plane.normal - self.plane.d) / max(abs(self.plane.normal[0]), 1e-9)
        lit = ok & (dist <= lt.line_width_mm / 2) & ~specular
        line = np.where(lit, 1.0, 0.0)

        def img(lux: float, mult: np.ndarray) -> np.ndarray:
            v = lt.ambient_lux + lux * base * mult
            noise = np.random.default_rng().normal(0, lt.noise_sigma, v.shape)   # 撮影ごとに違う雑音
            return np.clip(v + noise, 0, 255).astype(np.uint8)

        one = np.ones_like(base)
        return {"normal": img(lt.normal_lux, one),
                "raking": img(lt.raking_lux, np.where(shadow, lt.shadow_factor, 1.0)),
                "line": img(lt.line_lux, line + 0.03),
                "dark": img(0.0, one),
                "normal2": img(lt.normal_lux, one)}          # 撮影順 通常→斜め→線光→全消灯→通常


def default_lighting(cfg: dict[str, Any]) -> Lighting:
    r, ll, syn = cfg["floor_watch"]["raking"], cfg["floor_watch"]["line_light"], cfg["floor_watch"]["synthetic"]
    return Lighting(raking_led_height_mm=float(r["led_height_mm"]), raking_led_y_mm=float(r["led_forward_mm"]),
                    shadow_max_mm=float(r["shadow_max_mm"]), line_width_mm=float(ll["width_mm_initial"]),
                    noise_sigma=float(syn["noise_sigma"]), shadow_factor=float(syn["shadow_factor"]))
