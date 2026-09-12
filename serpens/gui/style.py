"""GUI の配色とフォント。展示会場の照明下で 3m 離れて読めるよう、大きく・高コントラストに。"""
from __future__ import annotations

from PySide6.QtGui import QColor, QFont

BG = QColor(18, 20, 26)
PANEL = QColor(28, 32, 40)
TEXT = QColor(245, 245, 245)
ACCENT = QColor(255, 214, 80)
MAT = QColor(196, 178, 140)
BODY = QColor(90, 210, 120)
PERSON = QColor(190, 140, 255)
TARGET = QColor(255, 200, 60)
HEAD_ARROW = QColor(90, 190, 255)
BODY_ARROW = QColor(255, 110, 90)
WARN = QColor(255, 120, 80)
BAR_BG = QColor(60, 66, 78)
SUBTEXT = QColor(170, 190, 210)
FONT_FAMILY = "Yu Gothic UI"


def font(size: int, bold: bool = True) -> QFont:
    """太字のゴシック（細いグレー文字は使わない）。"""
    f = QFont(FONT_FAMILY, size)
    f.setBold(bold)
    return f
