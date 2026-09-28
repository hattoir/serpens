"""Floor Watch の幾何: 頭カメラ・床・線光の平面（すべて DESIGN 値。実測ではない）。

世界座標（床見守りの局所系。Task の `home` 系とは別）:
    原点 = カメラ直下の床、x = 右（横）、y = 前、z = 上 [mm]
カメラ: 床から height_mm、光軸を pitch_deg 下向き。画像座標 u 右・v 下、焦点距離 f_px、主点 (cx, cy)。
線光: 投光部（カメラの横 lateral_mm・同じ高さ）から内側へ tilt_deg 傾けた光の面。床上では x = 0 の線（前後方向）。
      高さ H の面ではその線が横へずれる（tilt 45° なら 1mm あたり 1mm）。
**名目の基線を式に埋め込まない**: 光の面はカメラ座標で較正（`LightPlane.fit`。1 円玉 1.5mm を 1・2・3 枚重ねた段）し、
高さは画素の視線とその面の交点で求める。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class Camera:
    f_px: float
    cx: float
    cy: float
    width_px: int
    height_px: int
    height_mm: float
    pitch_deg: float

    @staticmethod
    def from_cfg(cfg: dict[str, Any], mode: str | None = None) -> "Camera":
        """floor_watch.camera.modes[mode]（省略時 floor_mode）の解像度と f（ASSUMED）で作る。"""
        c = cfg["floor_watch"]["camera"]
        m = c["modes"][mode or c["floor_mode"]]
        return Camera(float(m["f_px"]), float(m["width_px"]) / 2.0, float(m["height_px"]) / 2.0,
                      int(m["width_px"]), int(m["height_px"]), float(c["height_mm"]), float(c["pitch_deg"]))

    # 世界 → カメラ: xc = 右, yc = 下, zc = 前（光軸）
    @property
    def _axes(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        s, c = math.sin(math.radians(self.pitch_deg)), math.cos(math.radians(self.pitch_deg))
        return np.array([1.0, 0.0, 0.0]), np.array([0.0, -s, -c]), np.array([0.0, c, -s])

    @property
    def center(self) -> np.ndarray:
        return np.array([0.0, 0.0, self.height_mm])

    def project(self, p_world: np.ndarray) -> tuple[float, float, float]:
        """世界点 → (u, v, zc)。zc ≤ 0 は写らない。"""
        v = np.asarray(p_world, float) - self.center
        xa, ya, za = self._axes
        xc, yc, zc = float(v @ xa), float(v @ ya), float(v @ za)
        if zc <= 1e-9:
            return float("nan"), float("nan"), zc
        return self.cx + self.f_px * xc / zc, self.cy + self.f_px * yc / zc, zc

    def ray(self, u: float, v: float) -> np.ndarray:
        """画素の視線方向（世界座標、単位ベクトル）。"""
        xa, ya, za = self._axes
        d = (u - self.cx) / self.f_px * xa + (v - self.cy) / self.f_px * ya + za
        return d / np.linalg.norm(d)

    def floor_point(self, u: float, v: float) -> np.ndarray | None:
        """画素の視線が床（z=0）と交わる世界点。上を向く画素は None。"""
        d = self.ray(u, v)
        if d[2] >= -1e-9:
            return None
        s = -self.center[2] / d[2]
        return self.center + s * d

    def mm_per_px_at(self, u: float, v: float) -> float:
        """床上のその点で 1 画素が何 mm か（横方向）。"""
        p = self.floor_point(u, v)
        if p is None:
            return float("inf")
        q = self.floor_point(u + 1.0, v)
        return float(np.linalg.norm(q - p)) if q is not None else float("inf")


@dataclass(frozen=True)
class LightPlane:
    """光の面 n·p = d（世界座標）。"""

    normal: np.ndarray
    d: float

    @staticmethod
    def design(cfg: dict[str, Any]) -> "LightPlane":
        """DESIGN 値から: 投光部 (lateral, 0, height) を通り、床の線 x=0 を含む面。"""
        ll, cam = cfg["floor_watch"]["line_light"], cfg["floor_watch"]["camera"]
        lateral, height = float(ll["lateral_mm"]), float(ll["height_mm"] if ll.get("height_mm") is not None else cam["height_mm"])
        # 面は点 (lateral, y, height) と (0, y, 0) を含む → 法線は (height, 0, −lateral) に比例
        n = np.array([height, 0.0, -lateral])
        n = n / np.linalg.norm(n)
        return LightPlane(n, 0.0)

    @staticmethod
    def fit(cam: Camera, samples: list[tuple[float, float, float]]) -> "LightPlane":
        """較正: (u, v, 既知の高さ H) の組から面を当てはめる（最小二乗）。3 組以上、2 段以上の高さが要る。"""
        pts = []
        for u, v, h in samples:
            d = cam.ray(u, v)
            if abs(d[2]) < 1e-9:
                continue
            s = (h - cam.center[2]) / d[2]                 # 視線が z = h の水平面と交わる点
            pts.append(cam.center + s * d)
        p = np.array(pts)
        if len(p) < 3 or len({round(h, 3) for _, _, h in samples}) < 2:
            raise ValueError("較正には 3 点以上・2 段以上の高さが要る")
        c = p.mean(axis=0)
        _, _, vt = np.linalg.svd(p - c)
        n = vt[-1]
        if n[0] < 0:
            n = -n
        return LightPlane(n / np.linalg.norm(n), float(n @ c / np.linalg.norm(n)))

    def fixed_to_head(self, cam_from: Camera, cam_to: Camera) -> "LightPlane":
        """頭に固定の光の面を、別の姿勢のカメラの世界座標で表す（カメラ座標では同じ面）。
        頭が沈む・首が垂れる（高さ・pitch が変わる）と、床の上の線の位置が変わる。"""
        Rf, Rt = np.stack(cam_from._axes), np.stack(cam_to._axes)     # 行 = カメラの x, y, z 軸（世界座標）
        n_c = Rf @ self.normal
        d_c = self.d - float(self.normal @ cam_from.center)
        n_t = Rt.T @ n_c
        return LightPlane(n_t, d_c + float(n_t @ cam_to.center))

    def floor_line_image(self, cam: Camera) -> tuple[np.ndarray, np.ndarray] | None:
        """床（z=0）との交線が画像に写る直線（点, 単位方向）。交線は 3 次元の直線なので画像でも直線（歪みなし）。"""
        n = self.normal
        nh2 = float(n[0] ** 2 + n[1] ** 2)
        if nh2 < 1e-12:
            return None
        p0 = np.array([n[0], n[1], 0.0]) * self.d / nh2
        dvec = np.cross(n, [0.0, 0.0, 1.0])
        dvec = dvec / np.linalg.norm(dvec)
        if dvec[1] < 0 or (abs(dvec[1]) < 1e-9 and dvec[0] < 0):
            dvec = -dvec
        ts = (40.0, 140.0) if abs(dvec[1]) > 0.5 else (-40.0, 40.0)   # 前後に走る線は前方の 2 点、左右なら左右の 2 点
        (u1, v1, z1), (u2, v2, z2) = (cam.project(p0 + t * dvec) for t in ts)
        if z1 <= 0 or z2 <= 0:
            return None
        a, b = np.array([u1, v1]), np.array([u2, v2])
        d = b - a
        return a, d / max(float(np.linalg.norm(d)), 1e-12)

    def height_at(self, cam: Camera, u: float, v: float) -> float | None:
        """線が写った画素の視線と光の面の交点の高さ [mm]。面と平行なら None。"""
        d = cam.ray(u, v)
        den = float(self.normal @ d)
        if abs(den) < 1e-9:
            return None
        s = (self.d - float(self.normal @ cam.center)) / den
        if s <= 0:
            return None
        return float((cam.center + s * d)[2])

    def line_u_on_floor(self, cam: Camera, v: float) -> float | None:
        """行 v で、床（z=0）上の線が写る画素列 u（面と床の交線を投影）。"""
        # 交線: n·p = d かつ z = 0。行 v を固定して u を数値で解く（単調なので二分法）
        lo, hi = -cam.width_px, 2 * cam.width_px
        f = lambda u: self._floor_residual(cam, u, v)  # noqa: E731
        flo, fhi = f(lo), f(hi)
        if flo is None or fhi is None or flo * fhi > 0:
            return None
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            fm = f(mid)
            if fm is None:
                return None
            if flo * fm <= 0:
                hi, fhi = mid, fm
            else:
                lo, flo = mid, fm
        return 0.5 * (lo + hi)

    def _floor_residual(self, cam: Camera, u: float, v: float) -> float | None:
        p = cam.floor_point(u, v)
        return None if p is None else float(self.normal @ p - self.d)
