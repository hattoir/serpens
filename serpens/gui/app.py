"""展示用 GUI（PySide6）。4分割。

  左上 カメラ映像（人物枠 / ArUco 枠 / 追跡中の1人をハイライト）… gui/panes.py
  右上 俯瞰マップ（マット・仮想フェンス・9軸を反映したヘビの形・θ_body と θ_head・人・目標点）… gui/panes.py
  左下 内部状態（Curiosity / Affection / Energy / Heat(℃) / Stress）
  右下 駆動状態（走行 / 停止 / 緊急停止）＋ いま何を考えているか（日本語1行）
       ＋ 状態 ＋ 次の遷移までの秒数 ＋ 各軸の負荷と温度（古い値は「—」で出す）

描画は 10fps。制御ループは別スレッド（serpens/runner.py）なので、描画が重くても周期は乱れない。
"""
from __future__ import annotations

from typing import Any

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor, QPainter
from PySide6.QtWidgets import QApplication, QFrame, QGridLayout, QWidget

from serpens.gui.joints import JointsPane
from serpens.gui.panes import CameraPane, MapPane
from serpens.gui.style import ACCENT, BAR_BG, BG, BODY, PANEL, SUBTEXT, TEXT, WARN, font
from serpens.gui.wording import head_distance_mm, sentence
from serpens.keys import KEY_HELP
from serpens.runner import ControlLoop, Snapshot

GUI_FPS = 10
BAR_KEYS = [("Curiosity", "好奇心"), ("Affection", "親しみ"), ("Energy", "元気"), ("Stress", "警戒")]


class StatePane(QWidget):
    """内部状態の横バー。"""

    def __init__(self, cfg: dict[str, Any]) -> None:
        super().__init__()
        self.cfg = cfg
        self.snap = Snapshot()
        self.setMinimumSize(360, 220)

    def paintEvent(self, _e: object) -> None:  # noqa: N802
        p = QPainter(self)
        p.fillRect(self.rect(), PANEL)
        st = self.snap.status
        p.setFont(font(15))
        p.setPen(TEXT)
        p.drawText(14, 26, "内部状態")
        if st is None:
            p.end()
            return
        y, h = 44, (self.height() - 60) // 5
        p.setFont(font(17))
        for key, label in BAR_KEYS:
            v = float(st.internal[key.lower()])
            self._bar(p, y, h, f"{label} {key}", v, f"{v:.2f}", WARN if key == "Stress" else BODY)
            y += h
        heat = st.heat_c
        limit = float(self.cfg["behavior"]["safety"]["overheat_c"])
        frac = 0.0 if heat is None else min(max(heat / limit, 0.0), 1.0)
        col = WARN if heat is not None and heat >= limit * 0.9 else ACCENT
        self._bar(p, y, h, f"熱さ Heat 限界{limit:.0f}℃", frac, "—" if heat is None else f"{heat:.1f}℃", col)
        p.end()

    def _bar(self, p: QPainter, y: int, h: int, label: str, value: float, text: str, color: QColor) -> None:
        p.setPen(TEXT)
        p.drawText(14, y + h // 2 + 8, label)
        x0, w = 250, self.width() - 350
        p.fillRect(x0, y + 6, w, h - 18, BAR_BG)
        p.fillRect(x0, y + 6, int(w * min(max(value, 0.0), 1.0)), h - 18, color)
        p.setPen(TEXT)
        p.drawText(x0 + w + 14, y + h // 2 + 8, text)


class ThoughtPane(QWidget):
    """いま何を考えているか＋状態＋各軸の負荷と温度。"""

    def __init__(self, cfg: dict[str, Any]) -> None:
        super().__init__()
        self.cfg = cfg
        self.snap = Snapshot()
        self.names = [j["name"] for j in cfg["joints"]]
        self.ids = [int(j["servo_id"]) for j in cfg["joints"]]
        self.setMinimumSize(360, 220)

    def paintEvent(self, _e: object) -> None:  # noqa: N802
        p = QPainter(self)
        p.fillRect(self.rect(), PANEL)
        s = self.snap
        st = s.status
        if st is None:
            p.end()
            return
        dist = head_distance_mm(s.snake_xy, s.target_xy, self.cfg)
        # 駆動状態（走行 / 停止 / 緊急停止）を最初に、色を変えて出す
        drive_col = {"RUN": BODY, "HOLD": ACCENT, "DISABLED": ACCENT, "EMERGENCY": WARN}.get(s.drive_state, TEXT)
        p.setFont(font(24))
        p.setPen(drive_col)
        p.drawText(16, 34, s.drive_text or s.drive_state)
        p.setFont(font(20))
        p.setPen(ACCENT)
        p.drawText(16, 66, f"{st.state_ja}")
        p.setFont(font(15))
        p.setPen(TEXT)
        p.drawText(150, 66, f"次の切替まで {st.time_to_next_s:4.1f} 秒" + ("" if not st.safety else f"　安全: {st.safety}"))
        p.setFont(font(19))
        p.setPen(TEXT if s.drive_state == "RUN" else SUBTEXT)
        text = sentence(st, self.cfg, dist) if s.drive_state == "RUN" else "停止中（行動は止まっています）"
        self._wrap(p, 16, 98, self.width() - 32, text, 24)
        p.setFont(font(14))
        p.setPen(SUBTEXT)
        p.drawText(16, 150, f"移動: {st.drive}")
        if s.fault:
            p.setFont(font(15))
            p.setPen(WARN)
            self._wrap(p, 16, 170, self.width() - 32, f"異常: {s.fault}", 18)
        elif s.blockers:
            p.setFont(font(14))
            p.setPen(WARN)
            self._wrap(p, 16, 170, self.width() - 32, "開始条件: " + " / ".join(s.blockers), 18)
        y = 206
        p.setFont(font(14))
        p.setPen(TEXT)
        p.drawText(16, y, "軸")
        for i, name in enumerate(self.names):
            p.drawText(60 + i * 58, y, name)
        for row, (label, fmt) in enumerate((("負荷", "{:+.2f}"), ("温度", "{:.0f}℃"))):
            yy = y + 24 + row * 24
            p.setPen(TEXT)
            p.drawText(16, yy, label)
            for i, sid in enumerate(self.ids):
                v = s.servo.get(sid)
                if v is None:
                    p.setPen(QColor(150, 150, 150))
                    p.drawText(60 + i * 58, yy, "—")
                    continue
                val = v.load if row == 0 else v.temp_c
                hot = row == 1 and val >= float(self.cfg["behavior"]["safety"]["overheat_c"]) * 0.9
                p.setPen(WARN if hot else TEXT)
                p.drawText(60 + i * 58, yy, fmt.format(val))
        p.setFont(font(13))
        p.setPen(SUBTEXT)
        src = s.telemetry_source or "—"
        miss = "なし" if not s.missing_axes else " ".join(f"ID{i}" for i in s.missing_axes)
        p.drawText(16, y + 74, f"テレメトリ: {src}　古い/未取得の軸: {miss}　頭部I/O: {s.head_link}")
        if s.stats is not None:
            p.setFont(font(13))
            p.setPen(SUBTEXT)
            p.drawText(16, self.height() - 30,
                       f"制御周期 目標 {1000 / s.stats.target_hz:.0f}ms / 実測 平均 {s.stats.mean_ms:.1f}ms "
                       f"最悪 {s.stats.worst_ms:.1f}ms（{s.stats.count} 回, 遅延 {s.stats.late_count} 回）")
        p.setFont(font(13))
        p.setPen(SUBTEXT)
        p.drawText(16, self.height() - 10, "　".join(f"{k}:{v}" for k, v in KEY_HELP) + f"　{s.message}")
        p.end()

    def _wrap(self, p: QPainter, x: int, y: int, w: int, text: str, line_h: int) -> None:
        line = ""
        for ch in text:
            if p.fontMetrics().horizontalAdvance(line + ch) > w:
                p.drawText(x, y, line)
                y += line_h
                line = ""
            line += ch
        p.drawText(x, y, line)


class MainWindow(QWidget):
    """4分割のメイン画面。"""

    def __init__(self, runner: ControlLoop, cfg: dict[str, Any], frame_source: Any | None = None) -> None:
        super().__init__()
        self.runner, self.cfg, self.frame_source = runner, cfg, frame_source
        self.setWindowTitle("Serpens EX-1")
        self.setStyleSheet(f"background:{BG.name()};")
        self.camera, self.map = CameraPane(), MapPane(cfg)
        self.state, self.thought = StatePane(cfg), ThoughtPane(cfg)
        self.joints = JointsPane(cfg)          # 9軸の指令角・実測角と機体の状態
        grid = QGridLayout(self)
        grid.setSpacing(8)
        grid.setRowStretch(0, 3)
        grid.setRowStretch(1, 3)
        grid.setRowStretch(2, 2)
        for widget, pos in ((self.camera, (0, 0)), (self.map, (0, 1)), (self.state, (1, 0)),
                            (self.thought, (1, 1)), (self.joints, (2, 0, 1, 2))):
            frame = QFrame()
            frame.setStyleSheet(f"background:{PANEL.name()}; border-radius:8px;")
            inner = QGridLayout(frame)
            inner.setContentsMargins(6, 6, 6, 6)
            inner.addWidget(widget)
            grid.addWidget(frame, *pos)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(int(1000 / GUI_FPS))

    def refresh(self) -> None:
        snap = self.runner.latest()          # fault / message は latest() が最新にしている
        for pane in (self.map, self.state, self.thought, self.joints):
            pane.snap = snap
            pane.update()
        if self.frame_source is not None:
            frame = self.frame_source()
            if frame is not None:
                self.camera.show_frame(frame)

    def keyPressEvent(self, e: object) -> None:  # noqa: N802
        from serpens import keys

        key = e.text()
        if key.upper() in ("Q",) or e.key() == Qt.Key_Escape:
            QApplication.quit()
            return
        msg = keys.handle(self.runner, key) if key else ""
        if msg:
            self.runner.message = msg

    def keyReleaseEvent(self, e: object) -> None:  # noqa: N802
        if e.text().upper() == "T":
            self.runner.touch(False)
