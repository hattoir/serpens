"""画像座標 [px] ↔ 床座標 [mm] の変換（ホモグラフィ）。

マット四隅4点の画像上の位置から作る（tools/calibrate_floor.py でクリックして JSON 保存）。
床（z=0）の点しか正しく変換できないので、高さ h のもの（胴体上面の ArUco など）は
カメラ位置が分かっていれば correct_height() で視差を補正する。

注意: 広角レンズの歪みはホモグラフィでは取れない。マットの端ほど誤差が出る。
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import cv2
import numpy as np

FORMAT_VERSION = 1


def mat_corners_mm(cfg: dict[str, Any]) -> np.ndarray:
    """マット四隅の世界座標（左手前, 右手前, 右奥, 左奥）。"""
    w, d = float(cfg["mat"]["width_mm"]), float(cfg["mat"]["depth_mm"])
    return np.array([[0.0, 0.0], [w, 0.0], [w, d], [0.0, d]])


@dataclass
class FloorHomography:
    """画像 ↔ 床 の射影変換。"""

    image_px: np.ndarray        # shape (4, 2) クリックした四隅
    floor_mm: np.ndarray        # shape (4, 2) 対応する世界座標
    image_size: tuple[int, int] = (0, 0)

    def __post_init__(self) -> None:
        self.image_px = np.asarray(self.image_px, dtype=np.float64).reshape(4, 2)
        self.floor_mm = np.asarray(self.floor_mm, dtype=np.float64).reshape(4, 2)
        self.H = cv2.getPerspectiveTransform(self.image_px.astype(np.float32), self.floor_mm.astype(np.float32))
        self.H_inv = np.linalg.inv(self.H)

    @staticmethod
    def from_clicks(cfg: dict[str, Any], clicks_px: Sequence[Sequence[float]],
                    image_size: tuple[int, int] = (0, 0)) -> "FloorHomography":
        """四隅のクリック位置（左手前, 右手前, 右奥, 左奥）から作る。"""
        return FloorHomography(np.array(clicks_px, float), mat_corners_mm(cfg), image_size)

    # ---- 変換 -----------------------------------------------------------------
    @staticmethod
    def _apply(M: np.ndarray, pts: np.ndarray) -> np.ndarray:
        p = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
        q = np.column_stack([p, np.ones(len(p))]) @ M.T
        return q[:, :2] / q[:, 2:3]

    def image_to_floor(self, pts_px: np.ndarray) -> np.ndarray:
        """画像座標 → 床座標 [mm]。shape (N, 2)。"""
        return self._apply(self.H, pts_px)

    def floor_to_image(self, pts_mm: np.ndarray) -> np.ndarray:
        """床座標 [mm] → 画像座標。shape (N, 2)。"""
        return self._apply(self.H_inv, pts_mm)

    def px_per_mm_at(self, floor_xy: np.ndarray, dir_xy: tuple[float, float] = (1.0, 0.0)) -> float:
        """床の点での、ある向きに 1mm 動いたときの画素数（大きさの目安）。"""
        p = np.asarray(floor_xy, float).reshape(1, 2)
        a, b = self.floor_to_image(p), self.floor_to_image(p + np.array(dir_xy))
        return float(np.linalg.norm(b - a))

    # ---- 保存 -----------------------------------------------------------------
    def save(self, path: str | Path) -> None:
        """JSON に保存する。"""
        data = {"version": FORMAT_VERSION, "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "image_size": list(self.image_size), "image_px": self.image_px.tolist(),
                "floor_mm": self.floor_mm.tolist()}
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")

    @staticmethod
    def load(path: str | Path) -> "FloorHomography":
        """JSON から読み込む。"""
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if data.get("version") != FORMAT_VERSION:
            raise ValueError(f"{path}: 形式のバージョンが違います")
        return FloorHomography(np.array(data["image_px"]), np.array(data["floor_mm"]),
                              tuple(data.get("image_size", (0, 0))))


def correct_height(p_floor: np.ndarray, height_mm: float, camera_mm: Sequence[float] | None) -> np.ndarray:
    """高さ h の点を床ホモグラフィで変換した結果 p を、真上の床の位置に直す。

    カメラ C（xy と高さ Hc）から見て、高さ h の点は床上では C から遠い側にずれて写る:
      真の位置 = C_xy + (p − C_xy) · (Hc − h) / Hc
    camera_mm が None なら補正しない。
    """
    p = np.asarray(p_floor, dtype=np.float64)
    if camera_mm is None:
        return p
    c = np.asarray(camera_mm[:2], dtype=np.float64)
    hc = float(camera_mm[2])
    return c + (p - c) * (hc - height_mm) / hc


def apparent_floor_point(true_xy: np.ndarray, height_mm: float, camera_mm: Sequence[float]) -> np.ndarray:
    """correct_height の逆: 高さ h の点が、床ホモグラフィ上ではどこに写るか（仮想カメラの描画用）。"""
    c = np.asarray(camera_mm[:2], dtype=np.float64)
    hc = float(camera_mm[2])
    return c + (np.asarray(true_xy, float) - c) * hc / (hc - height_mm)
