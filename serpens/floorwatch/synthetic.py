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

import cv2
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


FLOOR_KINDS = ("wood", "tile", "rug", "carpet", "pattern")


@dataclass
class Scene:
    objects: list[Disc] = field(default_factory=list)
    stains: list[Stain] = field(default_factory=list)
    seams: list[Seam] = field(default_factory=list)
    floor_albedo: float = 0.5
    seed: int = 0
    floor: str = "wood"                    # FLOOR_KINDS（H2 の Hardware Gap 用。既定は従来の木目）
    texture_contrast: float = 1.0          # 模様の濃さの倍率
    floor_height_sigma_mm: float = 0.0     # 床の凹凸（カーペットの毛足など。約 2mm 周期のなめらかな乱数場）


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
    # 以下は H2 の Hardware Gap 用（既定は従来どおり = 変化なし）
    blur_sigma_px: float = 0.0             # ピンぼけ・手ぶれ（全画像に同じガウスぼけ）
    ambient_drift: float = 0.0             # 環境光が撮影の間に変わる割合（窓の光・照明のちらつき。1σ）
    line_scatter_mm: float = 0.0           # 床で線がにじむ幅（毛足の散乱。0 なら幅どおりの矩形）
    shot_noise_k: float = 0.0              # 明るさに比例する雑音（σ² = noise_sigma² + k·信号）
    auto_exposure: bool = False            # 5 枚に共通の露出を、通常画像の 99% 点が 230 になるよう合わせる
    physical_falloff: bool = False         # 照明を点光源（cos / r²）にする。通常照明はレンズの脇、斜め照明はあご。視野の中心の床で 1 に正規化
    normal_led_height_mm: float = 30.0     # 通常照明（レンズの脇）の高さ
    normal_led_y_mm: float = 0.0           # 通常照明の前後位置
    irradiance_max: float = 20.0           # 光源のすぐ近くで発散しないよう上限（正規化した値）
    # 複数の LED（世界座標 (x, y, z) mm の組）。指定すると影を 3 次元で描く（LED から床の点への線が物に遮られるか）。
    # None なら従来の 1 灯（カメラ直下、影は物の奥へ）。Design の案: 頬の下に斜め 2 灯・口の線に通常 2 灯（integration-log ENTRY-0022）
    raking_leds: tuple[tuple[float, float, float], ...] | None = None
    normal_leds: tuple[tuple[float, float, float], ...] | None = None


def _random_field(seed: int, scale_mm: float, extent_mm: float = 320.0, res_mm: float = 0.25) -> tuple[np.ndarray, float]:
    """床の座標に固定された乱数場（平均 0・標準偏差 1）。撮影ごとに変わらない（床そのものの模様・凹凸）。"""
    n = int(extent_mm / res_mm)
    g = np.random.default_rng(seed).normal(0.0, 1.0, (n, n)).astype(np.float32)
    k = max(1, int(round(scale_mm / res_mm)))
    g = cv2.GaussianBlur(g, (0, 0), k)
    g = (g - g.mean()) / max(float(g.std()), 1e-6)
    return g, res_mm


def _sample(field_: tuple[np.ndarray, float], xy: np.ndarray) -> np.ndarray:
    """床の座標 (x: −160〜160, y: −10〜310 mm) の乱数場を画素へ写す（範囲外は 0）。"""
    g, res = field_
    mx = ((xy[..., 0] + 160.0) / res).astype(np.float32)
    my = ((xy[..., 1] + 10.0) / res).astype(np.float32)
    return cv2.remap(g, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0.0)


def _floor_texture(xy: np.ndarray, seed: int, floor: str = "wood", contrast: float = 1.0) -> np.ndarray:
    """床の模様（反射率の倍率）。wood は従来の木目（前後方向の縞 + 低周波のむら）。**すべて模擬の模様。**"""
    x, y = xy[..., 0], xy[..., 1]
    if floor == "wood":
        grain = 0.08 * np.sin(x / 9.0 + 0.7 * np.sin(y / 40.0 + seed)) + 0.05 * np.sin(x / 2.3 + seed)
        blotch = 0.06 * np.sin(x / 55.0 + y / 70.0 + seed * 1.3)
        return 1.0 + contrast * (grain + blotch)
    if floor == "tile":                                        # ほぼ無地 + 目地（300mm 角。視野に 1 本入るかどうか）
        speck = 0.03 * _sample(_random_field(seed, 1.0), xy)
        grout = np.where((np.abs((x + 150.0 + 37.0 * seed) % 300.0 - 150.0) > 148.5) |
                         (np.abs((y + 11.0 * seed) % 300.0 - 150.0) > 148.5), -0.35, 0.0)
        return 1.0 + contrast * (speck + grout)
    if floor == "rug":                                         # 繊維（約 0.7mm）+ 織りのむら
        return 1.0 + contrast * (0.18 * _sample(_random_field(seed, 0.7), xy) + 0.06 * _sample(_random_field(seed + 1, 6.0), xy))
    if floor == "carpet":                                      # 毛足の房（約 1.5mm）が強い
        return 1.0 + contrast * (0.28 * _sample(_random_field(seed, 1.5), xy) + 0.08 * _sample(_random_field(seed + 1, 8.0), xy))
    if floor == "pattern":                                     # 濃い柄の敷物（10〜20mm の模様）
        return 1.0 + contrast * (0.35 * np.tanh(2.0 * _sample(_random_field(seed, 6.0), xy)))
    raise ValueError(f"floor は {FLOOR_KINDS} のどれか: {floor}")


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

    def __init__(self, cam: Camera, plane: LightPlane, lighting: Lighting, sides: bool = False) -> None:
        """sides=True: 円柱の側面も描く（高い物ほど側面が大きく写る。H2 の評価用）。既定は従来の上面だけ。"""
        self.cam, self.plane, self.lt, self.sides = cam, plane, lighting, sides
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
        """各画素の表面: 3 次元の点、反射率、鏡面、物体 index（-1 = 床）。視線で最も手前の面が写る。"""
        H, W = self.cam.height_px, self.cam.width_px
        fx, fy = np.nan_to_num(self.floor_xy[..., 0]), np.nan_to_num(self.floor_xy[..., 1])
        height = _seam_height(fx, fy, scene.seams)                      # 継ぎ目は視差を無視して床の高さ場に
        if scene.floor_height_sigma_mm > 0:                              # 毛足の凹凸（床の座標に固定）
            height = height + scene.floor_height_sigma_mm * _sample(_random_field(scene.seed + 101, 2.0), self.floor_xy)
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
        cz = self.cam.height_mm
        s_best = np.where(ok, (height - cz) / np.where(ok, dz, 1.0), np.inf)   # 床（継ぎ目・凹凸の高さの水平面）
        nrm = np.zeros((H, W, 3))
        nrm[..., 2] = 1.0                                               # 床・上面は上向き
        dx, dy = self.dir[..., 0], self.dir[..., 1]
        a = dx * dx + dy * dy
        for i, ob in enumerate(scene.objects):
            r = ob.diameter_mm / 2
            s_top = np.where(ok, (ob.height_mm - cz) / np.where(ok, dz, 1.0), np.inf)
            p = self.cam.center + np.where(np.isfinite(s_top), s_top, 0.0)[..., None] * self.dir
            top = ok & (np.hypot(p[..., 0] - ob.x_mm, p[..., 1] - ob.y_mm) <= r)
            s_obj = np.where(top, s_top, np.inf)
            shade = np.ones((H, W))
            n_side = None
            if self.sides:                                              # 鉛直な円柱の側面（入る側の交点）
                ox, oy = -ob.x_mm, -ob.y_mm                             # カメラ中心 (0, 0) − 物の中心
                b = 2 * (ox * dx + oy * dy)
                c = ox * ox + oy * oy - r * r
                disc = b * b - 4 * a * c
                hit = (disc >= 0) & (a > 1e-12)
                s_side = np.where(hit, (-b - np.sqrt(np.where(hit, disc, 0.0))) / (2 * np.where(hit, a, 1.0)), np.inf)
                z_side = cz + s_side * dz
                side = hit & (s_side > 0) & (z_side >= 0) & (z_side <= ob.height_mm) & (s_side < s_obj)
                s_obj = np.where(side, s_side, s_obj)
                # 側面は上からの照明に対して暗い（法線が水平）。見る向きとの角度で 0.45〜0.75 倍（模擬）
                s_fin = np.where(side, s_side, 0.0)
                px_ = dx * s_fin - ob.x_mm
                py_ = dy * s_fin - ob.y_mm
                nlen = np.maximum(np.hypot(px_, py_), 1e-9)
                facing = np.clip(-(px_ * dx + py_ * dy) / (nlen * np.sqrt(np.maximum(a, 1e-12))), 0.0, 1.0)
                if not self.lt.physical_falloff:              # 点光源にするときは照明の向きで決まる（ここでは暗くしない）
                    shade = np.where(side, 0.45 + 0.3 * facing, shade)
                n_side = (side, px_ / nlen, py_ / nlen)
            win = s_obj < s_best
            s_best = np.where(win, s_obj, s_best)
            albedo = np.where(win, ob.albedo * shade, albedo)
            specular = np.where(win, ob.specular, specular)
            owner = np.where(win, i, owner)
            nrm = np.where(win[..., None], np.array([0.0, 0.0, 1.0]), nrm)
            if n_side is not None:
                sd, nx_, ny_ = n_side
                m = win & sd
                nrm = np.where(m[..., None], np.stack([nx_, ny_, np.zeros_like(nx_)], axis=-1), nrm)
        xyz = self.cam.center + np.where(np.isfinite(s_best), s_best, 0.0)[..., None] * self.dir
        self._normals = nrm                                             # 照明（点光源）の計算に使う
        return xyz, albedo, specular, owner

    def flat_field(self, gain: float) -> dict[str, np.ndarray]:
        """照明の較正画像（白いカード = 反射率 1 の平らな床を各照明で撮り、全消灯を引いたもの）。雑音なし、gain は撮影の露出。
        点光源（physical_falloff）でなければ一様。"""
        lt = self.lt
        ok = self.dir[..., 2] < -1e-9
        s = np.where(ok, -self.cam.height_mm / np.where(ok, self.dir[..., 2], 1.0), 0.0)
        floor = self.cam.center + s[..., None] * self.dir
        self._normals = np.zeros(floor.shape)
        self._normals[..., 2] = 1.0
        if lt.physical_falloff:
            e_n = np.mean([self._irradiance(floor, L) for L in self._leds("normal")], axis=0)
            e_r = np.mean([self._irradiance(floor, L) for L in self._leds("raking")], axis=0)
        else:
            e_n = e_r = np.ones(floor.shape[:2])
        # 白いカードは床より明るいので、較正は飽和しない低い露出で撮り、露出の比を掛けて戻す（255 で切らない）。
        # 切ると近い側が飽和したまま較正され、そこだけ割り算が合わなかった（VIS-0008）
        return {"flat_normal": (lt.normal_lux * e_n * gain).astype(np.float32),
                "flat_raking": (lt.raking_lux * e_r * gain).astype(np.float32)}

    def _leds(self, kind: str) -> list[np.ndarray]:
        lt = self.lt
        many = lt.raking_leds if kind == "raking" else lt.normal_leds
        if many:
            return [np.asarray(p, float) for p in many]
        if kind == "raking":
            return [np.array([0.0, lt.raking_led_y_mm, lt.raking_led_height_mm])]
        return [np.array([0.0, lt.normal_led_y_mm, lt.normal_led_height_mm])]

    def _shadow3d(self, scene: Scene, owner: np.ndarray, surf: np.ndarray, led: np.ndarray) -> np.ndarray:
        """床の点から LED への線分が、物（鉛直な円柱の側面・上面）に遮られるか。床の画素だけ（物の上は影にしない）。"""
        p = surf
        v = led - p                                              # 床の点 → LED
        shadow = np.zeros(p.shape[:2], bool)
        floor = owner < 0
        for ob in scene.objects:
            r = ob.diameter_mm / 2
            ox, oy = p[..., 0] - ob.x_mm, p[..., 1] - ob.y_mm
            a = v[..., 0] ** 2 + v[..., 1] ** 2
            b = 2 * (ox * v[..., 0] + oy * v[..., 1])
            c = ox * ox + oy * oy - r * r
            disc = b * b - 4 * a * c
            ok = (disc >= 0) & (a > 1e-12)
            sq = np.sqrt(np.where(ok, disc, 0.0))
            hit = np.zeros_like(shadow)
            for sgn in (-1.0, 1.0):
                t = np.where(ok, (-b + sgn * sq) / (2 * np.where(ok, a, 1.0)), -1.0)
                z = p[..., 2] + t * v[..., 2]
                hit |= ok & (t > 1e-6) & (t < 1.0) & (z >= 0.0) & (z <= ob.height_mm)
            vz = v[..., 2]
            t_top = np.where(np.abs(vz) > 1e-9, (ob.height_mm - p[..., 2]) / np.where(np.abs(vz) > 1e-9, vz, 1.0), -1.0)
            q = p + t_top[..., None] * v
            hit |= (t_top > 1e-6) & (t_top < 1.0) & (np.hypot(q[..., 0] - ob.x_mm, q[..., 1] - ob.y_mm) <= r)
            shadow |= hit & floor
        return shadow

    def _irradiance(self, surf: np.ndarray, light: np.ndarray) -> np.ndarray:
        """点光源の照度 cos / r²（表面の法線と光源の向き）。視野の中心あたりの床（前 71mm）で 1 に正規化する。"""
        ref = _irradiance_at(np.array([0.0, 71.0, 0.0]), np.array([0.0, 0.0, 1.0]), light)
        e = _irradiance_at(surf, self._normals, light) / max(float(ref), 1e-12)
        return np.minimum(e, self.lt.irradiance_max)

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

    def render(self, scene: Scene, noise_seed: int | None = None) -> dict[str, np.ndarray]:
        """noise_seed: 雑音の乱数の種（試験の再現用）。None なら撮影ごとに違う雑音（従来どおり）。"""
        cam, lt = self.cam, self.lt
        surf, albedo, specular, owner = self._surface(scene)
        tex = np.where(owner >= 0, 1.0, np.clip(_floor_texture(np.nan_to_num(self.floor_xy), scene.seed, scene.floor,
                                                               scene.texture_contrast), 0.15, None))   # 繊維も真っ黒にはならない
        base = albedo * tex
        shadow = self._shadow(scene, owner)
        # 線光: 表面の点と光の面の距離。幅は床の上（水平）で測る → 面の法線の水平成分で割る
        ok = self.dir[..., 2] < -1e-9
        n_h = max(float(np.hypot(self.plane.normal[0], self.plane.normal[1])), 1e-9)
        dist = np.abs(surf @ self.plane.normal - self.plane.d) / n_h
        lit = ok & (dist <= lt.line_width_mm / 2) & ~specular
        line = np.where(lit, 1.0, 0.0)
        if lt.line_scatter_mm > 0:                                       # 毛足で線がにじむ（床の上だけ）
            halo = 0.5 * np.exp(-0.5 * (np.maximum(dist - lt.line_width_mm / 2, 0.0) / lt.line_scatter_mm) ** 2)
            line = np.where(ok & (owner < 0) & ~lit, halo, line)
        rng = np.random.default_rng(noise_seed)
        e_n = e_r = np.ones_like(base)
        raking_mult = None
        if lt.raking_leds or lt.normal_leds:                               # 複数の LED: 影は LED ごと（片方だけ遮られると半影）
            rk = self._leds("raking")
            er = [self._irradiance(surf, L) if lt.physical_falloff else np.ones_like(base) for L in rk]
            raking_mult = sum(e * np.where(self._shadow3d(scene, owner, surf, L), lt.shadow_factor, 1.0) for e, L in zip(er, rk)) / len(rk)
            if lt.physical_falloff:
                e_n = np.mean([self._irradiance(surf, L) for L in self._leds("normal")], axis=0)
        elif lt.physical_falloff:
            e_n = self._irradiance(surf, np.array([0.0, lt.normal_led_y_mm, lt.normal_led_height_mm]))
            e_r = self._irradiance(surf, np.array([0.0, lt.raking_led_y_mm, lt.raking_led_height_mm]))
        signals = {"normal": (lt.normal_lux, e_n),
                   "raking": (lt.raking_lux, raking_mult if raking_mult is not None else e_r * np.where(shadow, lt.shadow_factor, 1.0)),
                   "line": (lt.line_lux, line + 0.03),
                   "dark": (0.0, np.ones_like(base)),
                   "normal2": (lt.normal_lux, e_n)}                        # 撮影順 通常→斜め→線光→全消灯→通常
        raw = {}
        for k, (lux, mult) in signals.items():
            amb = lt.ambient_lux * (1.0 + (rng.normal(0.0, lt.ambient_drift) if lt.ambient_drift > 0 else 0.0))
            v = amb * base + lux * base * mult if lt.ambient_drift > 0 or lt.auto_exposure else lt.ambient_lux + lux * base * mult
            if lt.blur_sigma_px > 0:
                v = cv2.GaussianBlur(v.astype(np.float32), (0, 0), lt.blur_sigma_px)
            raw[k] = v
        gain = 1.0
        if lt.auto_exposure:                                              # 5 枚に共通の露出（別々に合わせると差が壊れる）
            gain = 230.0 / max(float(np.percentile(raw["normal"], 99)), 1e-6)
        self.last_gain = gain                                             # 較正画像（flat_field）を同じ露出で作るため

        def img(v: np.ndarray) -> np.ndarray:
            v = v * gain
            sigma = np.sqrt(lt.noise_sigma ** 2 + lt.shot_noise_k * np.maximum(v, 0.0)) if lt.shot_noise_k > 0 else lt.noise_sigma
            return np.clip(v + rng.normal(0.0, 1.0, v.shape) * sigma, 0, 255).astype(np.uint8)   # 撮影ごとに違う雑音
        return {k: img(v) for k, v in raw.items()}


def _irradiance_at(p: np.ndarray, n: np.ndarray, light: np.ndarray) -> np.ndarray:
    v = light - p
    r2 = np.maximum(np.sum(v * v, axis=-1), 1e-6)
    cos = np.sum(n * v, axis=-1) / np.sqrt(r2)
    return np.clip(cos, 0.0, None) / r2


def default_lighting(cfg: dict[str, Any]) -> Lighting:
    r, ll, syn = cfg["floor_watch"]["raking"], cfg["floor_watch"]["line_light"], cfg["floor_watch"]["synthetic"]
    nl = cfg["floor_watch"].get("normal_light", {"led_height_mm": 30.0, "led_forward_mm": 0.0})
    return Lighting(raking_led_height_mm=float(r["led_height_mm"]), raking_led_y_mm=float(r["led_forward_mm"]),
                    normal_led_height_mm=float(nl["led_height_mm"]), normal_led_y_mm=float(nl["led_forward_mm"]),
                    shadow_max_mm=float(r["shadow_max_mm"]), line_width_mm=float(ll["width_mm_initial"]),
                    noise_sigma=float(syn["noise_sigma"]), shadow_factor=float(syn["shadow_factor"]))
