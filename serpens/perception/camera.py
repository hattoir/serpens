"""画像の入力元。Webカメラ・録画ファイル・静止画・仮想カメラ（シミュレータ）を同じ形で扱う。

どれも read() → (ok, BGR 画像) を返す。カメラが無くても、録画かダミー画像で後段を動かせる。
"""
from __future__ import annotations

from pathlib import Path
from typing import Any, Callable, Protocol

import cv2
import numpy as np

Frame = np.ndarray


class FrameSource(Protocol):
    """画像の入力元。"""

    def read(self) -> tuple[bool, Frame | None]: ...

    def release(self) -> None: ...


class CameraSource:
    """USB Webカメラ（cv2.VideoCapture）。"""

    def __init__(self, cfg: dict[str, Any], index: int | None = None) -> None:
        c = cfg["camera"]
        idx = int(c["index"]) if index is None else index
        api = cv2.CAP_DSHOW if bool(c["use_dshow"]) else cv2.CAP_ANY
        self.cap = cv2.VideoCapture(idx, api)
        if not self.cap.isOpened():
            raise RuntimeError(f"カメラ {idx} を開けません（他のアプリが使用中？）")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(c["width_px"]))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(c["height_px"]))

    def read(self) -> tuple[bool, Frame | None]:
        ok, frame = self.cap.read()
        return ok, frame if ok else None

    def release(self) -> None:
        self.cap.release()


class VideoFileSource:
    """録画ファイル。loop=True なら最後まで行ったら先頭に戻る。"""

    def __init__(self, path: str | Path, loop: bool = True) -> None:
        self.cap = cv2.VideoCapture(str(path))
        if not self.cap.isOpened():
            raise RuntimeError(f"動画を開けません: {path}")
        self.loop = loop

    def read(self) -> tuple[bool, Frame | None]:
        ok, frame = self.cap.read()
        if not ok and self.loop:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        return ok, frame if ok else None

    def release(self) -> None:
        self.cap.release()


class ImageSource:
    """静止画を毎回同じように返す（ダミー画像での動作確認用）。"""

    def __init__(self, path: str | Path) -> None:
        # 日本語パスでも読めるよう imdecode を使う
        data = np.fromfile(str(path), dtype=np.uint8)
        img = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError(f"画像を読めません: {path}")
        self.img = img

    def read(self) -> tuple[bool, Frame | None]:
        return True, self.img.copy()

    def release(self) -> None:
        pass


class CallableSource:
    """関数が返す画像を使う（仮想カメラなど）。"""

    def __init__(self, fn: Callable[[], Frame]) -> None:
        self.fn = fn

    def read(self) -> tuple[bool, Frame | None]:
        return True, self.fn()

    def release(self) -> None:
        pass


def open_source(cfg: dict[str, Any], spec: str) -> FrameSource:
    """コマンドライン指定から入力元を作る。数字ならカメラ番号、画像拡張子なら静止画、それ以外は動画。"""
    if spec.isdigit():
        return CameraSource(cfg, int(spec))
    if Path(spec).suffix.lower() in (".jpg", ".jpeg", ".png", ".bmp"):
        return ImageSource(spec)
    return VideoFileSource(spec)
