"""関節ペイン: 9軸の**指令角と実測角**、Safety 状態、ESP32 状態を1枚で見る。

展示中に「いま何が起きているか」を人が判断できるようにするためのもの。

  - 各軸のバー: operational limit を枠、指令角を線、実測角を塗り
  - 指令と実測のズレ（追従できていない = 引っかかり・過負荷の兆候）を数値で
  - 機体側（ESP32）の状態と、**その値が模擬か実測か**
"""
from __future__ import annotations

from typing import Any

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QWidget

from serpens.gui.style import ACCENT, BAR_BG, BODY, SUBTEXT, TEXT, WARN, font
from serpens.runner import Snapshot

ROW_H = 26                  # 1軸ぶんの高さ
LABEL_W = 48
VALUE_W = 150
SIM_COLOR = QColor(255, 180, 60)      # 模擬の値は色を変える（実測と取り違えないため）


class JointsPane(QWidget):
    """9軸の角度と、安全・機体の状態。"""

    def __init__(self, cfg: dict[str, Any]) -> None:
        super().__init__()
        self.cfg = cfg
        self.snap = Snapshot()
        self.names = [j["name"] for j in cfg["joints"]]
        self.limits = {j["name"]: (float(j["min_deg"]), float(j["max_deg"])) for j in cfg["joints"]}
        self.setMinimumSize(420, ROW_H * len(self.names) + 64)

    def paintEvent(self, _e: object) -> None:  # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        snap = self.snap
        self._header(p, snap)
        y = 52
        for name in self.names:
            self._row(p, y, name, snap.commanded.get(name), snap.measured.get(name))
            y += ROW_H
        p.end()

    # ---- 見出し（安全 → 機体 の順。**止まっているかどうかが最初に見える**） -----------
    def _header(self, p: QPainter, snap: Snapshot) -> None:
        p.setFont(font(15))
        p.setPen(WARN if snap.latched or "停止" in snap.drive_text else TEXT)
        p.drawText(8, 20, f"安全: {snap.drive_text or snap.drive_state}")
        dev = snap.device
        p.setFont(font(12))
        if dev is None:
            p.setPen(SUBTEXT)
            p.drawText(8, 40, f"機体: 駆動リンクなし（{snap.telemetry_source} へ直接書き込み）")
            return
        age = "—" if dev.age_s is None else f"{dev.age_s * 1000:.0f}ms前"
        p.setPen(SIM_COLOR if dev.simulated else TEXT)
        src = "模擬" if dev.simulated else "実測"
        text = f"機体[{src}]: {dev.state_ja} / {dev.reason_ja} / {age}"
        if dev.missing_axes:
            text += f" / 応答なし {dev.missing_axes}軸"
        if dev.overruns:
            text += f" / 周期超過 {dev.overruns}回"
        p.drawText(8, 40, text)

    # ---- 1軸ぶん -----------------------------------------------------------------------
    def _row(self, p: QPainter, y: int, name: str, cmd: float | None, meas: float | None) -> None:
        lo, hi = self.limits[name]
        w = self.width() - LABEL_W - VALUE_W - 16
        x0 = LABEL_W + 8
        p.setFont(font(12))
        p.setPen(TEXT)
        p.drawText(8, y + 16, name)

        p.setPen(Qt.NoPen)
        p.setBrush(BAR_BG)
        p.drawRect(x0, y + 6, w, 12)
        p.setPen(QPen(SUBTEXT, 1))
        zero = x0 + int(w * (0.0 - lo) / (hi - lo))
        p.drawLine(zero, y + 4, zero, y + 20)

        if meas is not None:                     # 実測（塗り）
            p.setPen(Qt.NoPen)
            p.setBrush(BODY)
            mx = x0 + int(w * (min(max(meas, lo), hi) - lo) / (hi - lo))
            p.drawRect(min(zero, mx), y + 8, abs(mx - zero), 8)
        if cmd is not None:                      # 指令（線）
            cx = x0 + int(w * (min(max(cmd, lo), hi) - lo) / (hi - lo))
            p.setPen(QPen(ACCENT, 2))
            p.drawLine(cx, y + 3, cx, y + 21)

        p.setFont(font(11))
        p.setPen(TEXT)
        c_txt = "—" if cmd is None else f"{cmd:+6.1f}"
        m_txt = "—" if meas is None else f"{meas:+6.1f}"
        p.drawText(x0 + w + 8, y + 16, f"指令{c_txt}  実測{m_txt}")
        if cmd is not None and meas is not None and abs(cmd - meas) > TRACK_WARN_DEG:
            p.setPen(WARN)
            p.drawText(x0 + w + 8 + 112, y + 16, f" Δ{abs(cmd - meas):.0f}")


TRACK_WARN_DEG = 8.0        # 指令と実測がこれ以上ずれたら色を変える（引っかかり・過負荷の兆候）
