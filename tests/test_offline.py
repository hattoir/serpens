"""展示会場にネットは無い前提。ネットワークを遮断しても動くことを確かめる。

YOLO は重みの自動ダウンロードや更新確認で外へ出ようとするので、ここで縛っておく。
重み（config の person.model_path）が無い環境ではスキップする。
"""
from __future__ import annotations

import socket
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

from serpens.config import load_config
from serpens.perception.homography import FloorHomography

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import offline_check  # noqa: E402

ASSETS = Path(".venv/Lib/site-packages/ultralytics/assets/bus.jpg")


@pytest.fixture()
def cfg() -> dict:
    return load_config()


@pytest.fixture()
def no_network(monkeypatch: pytest.MonkeyPatch) -> None:
    """外向きの通信を遮断する（ローカルは許可）。"""
    real = socket.socket

    class Guarded(real):  # type: ignore[misc, valid-type]
        def connect(self, address):  # type: ignore[no-untyped-def]
            host = address[0] if isinstance(address, tuple) else str(address)
            if host not in offline_check.ALLOW_LOCAL:
                raise OSError("オフライン検証: 外へ出ようとした")
            return super().connect(address)

    def no_dns(*a: object, **k: object) -> None:
        raise OSError("オフライン検証: 名前解決しようとした")

    monkeypatch.setattr(socket, "socket", Guarded)
    monkeypatch.setattr(socket, "getaddrinfo", no_dns)
    monkeypatch.setattr(socket, "create_connection", no_dns)


def test_network_guard_blocks_outside(no_network: None) -> None:
    with pytest.raises(OSError):
        socket.getaddrinfo("github.com", 443)


@pytest.mark.skipif(not Path(load_config()["person"]["model_path"]).exists(),
                    reason="YOLO の重みが無い（README「YOLO の重み」参照）")
def test_yolo_loads_and_detects_offline(cfg: dict, no_network: None) -> None:
    """ネットワーク遮断下でも、重みの読み込みと推論ができる。"""
    from serpens.perception.person_detector import YoloPersonDetector

    h = FloorHomography.from_clicks(cfg, cfg["virtual_camera"]["mat_corners_px"], (1920, 1080))
    det = YoloPersonDetector(cfg, h)
    img = cv2.imdecode(np.fromfile(str(ASSETS), np.uint8), cv2.IMREAD_COLOR) if ASSETS.exists() else None
    if img is None:
        pytest.skip("ultralytics の同梱画像が見つからない")
    found = det.detect(img)
    assert len(found) >= 3          # bus.jpg には4人写っている
    assert all(d.conf >= cfg["person"]["conf_threshold"] for d in found)


def test_app_starts_offline(no_network: None) -> None:
    """--sim が遮断下でも起動して終了する。"""
    from serpens.app import main

    assert main(["--sim", "--no-gui", "--seconds", "1.5"]) == 0
