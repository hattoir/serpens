"""人物検出（YOLO, CPU, person のみ）と、最も近い1人の追跡。

足元 = bbox 下辺の中央 → ホモグラフィで床座標へ。
追跡（PersonTracker）:
  - マット中心から person.max_range_mm の外にいる人は無視
  - 来場者は y < 0 側の 1 辺からしか近づけない配置なので、y > visitor_side_y_max_mm の人は無視
  - 追跡するのは「ヘビ（首）に最も近い1人」だけ
  - 別の人の方が近い状態が person.switch_hysteresis_s 続いたら切り替える
  - 検出0人（または追跡中の人を見失った）でも person.hold_s は前回値を保持
YOLO の重みは自動ダウンロードしない（person.model_path に置く。README 参照）。
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from serpens.perception.homography import FloorHomography


@dataclass(frozen=True)
class PersonDetection:
    """1人分の検出。"""

    bbox_px: tuple[float, float, float, float]   # x1, y1, x2, y2
    conf: float
    floor_mm: np.ndarray                          # 足元の床座標

    @property
    def foot_px(self) -> tuple[float, float]:
        x1, _y1, x2, y2 = self.bbox_px
        return (x1 + x2) / 2.0, y2


def detections_from_boxes(boxes: np.ndarray, confs: np.ndarray, homography: FloorHomography) -> list[PersonDetection]:
    """bbox 群 → 足元を床座標にした検出リスト。"""
    out = []
    for b, c in zip(np.asarray(boxes).reshape(-1, 4), np.asarray(confs).reshape(-1)):
        foot = np.array([[(b[0] + b[2]) / 2.0, b[3]]])
        out.append(PersonDetection(tuple(float(v) for v in b), float(c), homography.image_to_floor(foot)[0]))
    return out


class YoloPersonDetector:
    """ultralytics YOLO で person クラスだけを検出する（CPU）。"""

    def __init__(self, cfg: dict[str, Any], homography: FloorHomography) -> None:
        p = cfg["person"]
        path = Path(p["model_path"])
        if not path.exists():
            raise FileNotFoundError(
                f"YOLO の重み {path} がありません。README「YOLO の重み」の手順で置いてください（自動ダウンロードはしません）")
        # 足りないパッケージの自動 pip install と、重みの自動ダウンロードを止めてから import する
        os.environ["YOLO_AUTOINSTALL"] = "false"
        os.environ["YOLO_OFFLINE"] = "true"
        from ultralytics import YOLO  # 遅延 import（重いので使うときだけ）

        self.model = YOLO(str(path))
        self.homography = homography
        self.imgsz = int(p["imgsz"])
        self.conf = float(p["conf_threshold"])
        self.cls = int(p["person_class_id"])

    def detect(self, frame: np.ndarray) -> list[PersonDetection]:
        """画像から人を検出する。"""
        r = self.model.predict(frame, imgsz=self.imgsz, conf=self.conf, classes=[self.cls],
                               device="cpu", verbose=False)[0]
        return detections_from_boxes(r.boxes.xyxy.cpu().numpy(), r.boxes.conf.cpu().numpy(), self.homography)


@dataclass
class TrackedPerson:
    """追跡中の1人。"""

    floor_mm: np.ndarray
    last_seen_t: float
    since_t: float          # この人を追い始めた時刻
    detection: PersonDetection | None = None


class PersonTracker:
    """最も近い1人だけを、ヒステリシス付きで追跡する。"""

    def __init__(self, cfg: dict[str, Any]) -> None:
        p = cfg["person"]
        self.center = np.array([float(cfg["mat"]["width_mm"]) / 2, float(cfg["mat"]["depth_mm"]) / 2])
        self.max_range = float(p["max_range_mm"])
        self.visitor_y_max = float(p["visitor_side_y_max_mm"])
        self.gate = float(p["association_gate_mm"])
        self.switch_s = float(p["switch_hysteresis_s"])
        self.hold_s = float(p["hold_s"])
        self.target: TrackedPerson | None = None
        self._challenger: TrackedPerson | None = None
        self.switches = 0

    def in_range(self, d: PersonDetection) -> bool:
        """追跡の対象にするか（来場者側 かつ マット中心から max_range 以内）。"""
        return (float(np.linalg.norm(d.floor_mm - self.center)) <= self.max_range
                and float(d.floor_mm[1]) <= self.visitor_y_max)

    def update(self, t: float, dets: list[PersonDetection], ref_mm: np.ndarray | None) -> TrackedPerson | None:
        """検出結果を取り込み、追跡中の1人（いなければ None）を返す。ref_mm はヘビの首の位置。"""
        ref = self.center if ref_mm is None else np.asarray(ref_mm, float)
        dets = [d for d in dets if self.in_range(d)]
        nearest = min(dets, key=lambda d: float(np.linalg.norm(d.floor_mm - ref)), default=None)

        if self.target is None:
            if nearest is not None:
                self.target = TrackedPerson(nearest.floor_mm, t, t, nearest)
            return self.target

        mine = self._match(self.target.floor_mm, dets)
        if mine is not None:
            self.target.floor_mm, self.target.last_seen_t, self.target.detection = mine.floor_mm, t, mine
        elif t - self.target.last_seen_t > self.hold_s:
            self.target, self._challenger = None, None       # 見失って hold_s 経過
            return self.update(t, dets, ref_mm) if dets else None

        # 別の人の方が近いか
        if nearest is not None and nearest is not mine and (
                mine is None or np.linalg.norm(nearest.floor_mm - ref) < np.linalg.norm(mine.floor_mm - ref)):
            ch = self._challenger
            if ch is None or np.linalg.norm(ch.floor_mm - nearest.floor_mm) > self.gate:
                self._challenger = TrackedPerson(nearest.floor_mm, t, t, nearest)
            else:
                ch.floor_mm, ch.last_seen_t, ch.detection = nearest.floor_mm, t, nearest
                if t - ch.since_t >= self.switch_s:
                    self.target, self._challenger = TrackedPerson(ch.floor_mm, t, t, nearest), None
                    self.switches += 1
        else:
            self._challenger = None
        return self.target

    def _match(self, pos: np.ndarray, dets: list[PersonDetection]) -> PersonDetection | None:
        """前回位置から gate 以内で最も近い検出。"""
        best = min(dets, key=lambda d: float(np.linalg.norm(d.floor_mm - pos)), default=None)
        if best is None or np.linalg.norm(best.floor_mm - pos) > self.gate:
            return None
        return best
