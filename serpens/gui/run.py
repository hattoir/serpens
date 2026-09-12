"""GUI の起動と、GIF への記録。"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any

from PySide6.QtCore import QTimer
from PySide6.QtGui import QImage
from PySide6.QtWidgets import QApplication

from serpens.gui.app import MainWindow
from serpens.runner import ControlLoop

WINDOW_SIZE = (1480, 900)
RECORD_FPS = 5


def run_gui(loop: ControlLoop, cfg: dict[str, Any], frame_source: Any,
            seconds: float | None = None, record: str | None = None) -> None:
    """GUI を開く。seconds を指定すると自動終了、record を指定すると GIF に保存する。"""
    if record and not os.environ.get("DISPLAY") and os.name != "nt":
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    app = QApplication.instance() or QApplication([])
    win = MainWindow(loop, cfg, frame_source)
    win.resize(*WINDOW_SIZE)
    win.show()
    frames: list[Any] = []
    if record:
        from PIL import Image

        def grab() -> None:
            img = win.grab().toImage().convertToFormat(QImage.Format.Format_RGB32)
            ptr = img.constBits()
            import numpy as np

            arr = np.array(ptr).reshape(img.height(), img.bytesPerLine() // 4, 4)[:, :img.width(), :3]
            frames.append(Image.fromarray(arr[:, :, ::-1]).quantize(colors=128))

        rec_timer = QTimer(win)
        rec_timer.timeout.connect(grab)
        rec_timer.start(int(1000 / RECORD_FPS))
    if seconds is not None:
        QTimer.singleShot(int(seconds * 1000), app.quit)
    app.exec()
    if record and frames:
        out = Path(record)
        out.parent.mkdir(parents=True, exist_ok=True)
        frames[0].save(out, save_all=True, append_images=frames[1:], duration=int(1000 / RECORD_FPS), loop=0)
        print(f"saved {out} ({len(frames)} frames)")
