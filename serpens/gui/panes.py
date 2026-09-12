"""俯瞰マップとカメラ映像のペイン（描画のみ）。"""
from __future__ import annotations

import math
from typing import Any

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import QLabel, QWidget

from serpens.gui.style import BODY, BODY_ARROW, HEAD_ARROW, MAT, PANEL, PERSON, TARGET, TEXT, font
from serpens.runner import Snapshot

VIEW_PAD_MM = 150.0        # マットの外側も少し描く
VISITOR_VIEW_MM = 900.0    # 来場者側（y<0）をここまで描く


class MapPane(QWidget):
    """俯瞰マップ。"""

    def __init__(self, cfg: dict[str, Any]) -> None:
        super().__init__()
        self.cfg = cfg
        self.snap = Snapshot()
        self.setMinimumSize(360, 360)

    def paintEvent(self, _e: object) -> None:  # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), PANEL)
        w_mm, d_mm = float(self.cfg["mat"]["width_mm"]), float(self.cfg["mat"]["depth_mm"])
        # 来場者は y<0 側に立つので、マットの手前も描画範囲に入れる
        x0, x1 = -VIEW_PAD_MM, w_mm + VIEW_PAD_MM
        y0, y1 = -VISITOR_VIEW_MM, d_mm + VIEW_PAD_MM
        margin = 26
        sc = min((self.width() - 2 * margin) / (x1 - x0), (self.height() - 2 * margin) / (y1 - y0))
        ox = margin + (self.width() - 2 * margin - (x1 - x0) * sc) / 2 - x0 * sc
        oy = self.height() - margin - (self.height() - 2 * margin - (y1 - y0) * sc) / 2 + y0 * sc

        def P(x: float, y: float) -> tuple[float, float]:
            return ox + x * sc, oy - y * sc

        p.setPen(QPen(TEXT, 2))
        p.setBrush(MAT)
        p.drawRect(*P(0, d_mm), w_mm * sc, d_mm * sc)
        fence = float(self.cfg["behavior"]["controller"]["mat_margin_mm"])
        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(QColor(255, 140, 120), 2, Qt.DashLine))
        p.drawRect(*P(fence, d_mm - fence), (w_mm - 2 * fence) * sc, (d_mm - 2 * fence) * sc)
        s = self.snap
        if s.points is not None:
            p.setPen(QPen(BODY, max(float(self.cfg["body"]["diameter_mm"]) * sc, 5), Qt.SolidLine, Qt.RoundCap))
            pts = [P(x, y) for x, y in s.points[:, :2]]
            for a, b in zip(pts[:-1], pts[1:]):
                p.drawLine(*a, *b)
            p.setPen(QPen(QColor(255, 80, 80), 3))
            p.setBrush(QColor(255, 80, 80))
            hx, hy = pts[-1]
            p.drawEllipse(hx - 6, hy - 6, 12, 12)
        if s.snake_xy is not None:
            for th, col, length in ((s.theta_body, BODY_ARROW, 260.0), (s.theta_head, HEAD_ARROW, 320.0)):
                p.setPen(QPen(col, 4))
                x0, y0 = P(*s.snake_xy)
                x1, y1 = P(s.snake_xy[0] + length * math.cos(th), s.snake_xy[1] + length * math.sin(th))
                p.drawLine(x0, y0, x1, y1)
                p.drawEllipse(x1 - 5, y1 - 5, 10, 10)
        for px, py in s.people:
            p.setPen(QPen(PERSON, 3))
            p.setBrush(PERSON)
            x, y = P(px, py)
            p.drawEllipse(x - 11, y - 11, 22, 22)
        if s.target_xy is not None:
            p.setPen(QPen(TARGET, 4))
            p.setBrush(Qt.NoBrush)
            x, y = P(*s.target_xy)
            p.drawEllipse(x - 20, y - 20, 40, 40)
        if s.waypoint is not None:
            p.setPen(QPen(QColor(140, 220, 255), 3))
            x, y = P(*s.waypoint)
            p.drawLine(x - 9, y - 9, x + 9, y + 9)
            p.drawLine(x - 9, y + 9, x + 9, y - 9)
        p.setFont(font(13))
        p.setPen(TEXT)
        p.setPen(QColor(150, 170, 190))
        p.drawText(*P(0, -VISITOR_VIEW_MM + 60), "← 来場者側（この辺だけ開放）")
        p.setPen(TEXT)
        p.drawText(10, 22, "俯瞰マップ　赤=θ_body　青=θ_head　紫=人　黄丸=追跡中　×=目標点")
        p.end()


class CameraPane(QLabel):
    """カメラ映像（または仮想カメラ）。"""

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumSize(360, 260)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet(f"background:{PANEL.name()}; color:{TEXT.name()};")
        self.setFont(font(16))
        self.setText("カメラ映像なし（--camera 0 で実写）")

    def show_frame(self, bgr: np.ndarray) -> None:
        rgb = np.ascontiguousarray(bgr[:, :, ::-1])
        h, w, _ = rgb.shape
        img = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888)
        self.setPixmap(QPixmap.fromImage(img).scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))


