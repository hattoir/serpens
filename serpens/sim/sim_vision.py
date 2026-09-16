"""観測器 — セッションが「ヘビと人がどこにいるか」を知る経路（Phase 4）。

    GroundTruthObserver … シミュレータの真値をそのまま渡す（pose_source = "sim"）
    SimVisionObserver   … 仮想カメラで**画像を描き**、本物の ArUco 検出と人物 bbox を通す
                          （pose_source = "aruco_sim"）

`aruco_sim` は **Vision の処理を通った推定だが、画像は模擬**。実機の開始条件（"aruco" を要求）は
これを通さない。World State では `ARUCO` / `PERSON_DETECTOR` に `simulated=True` を付けて出す。

撮影は `vision_sim.frame_hz` ごと。結果は `latency_s` 遅れて届き、**撮影時刻のまま**追跡器へ入る
（自己位置の古さ = 遅れ + 撮影間隔、が正直に出る）。

故障注入（`VisionFaults`）は試験用で、config には置かない（`serpens/link/faults.py` と同じ扱い）。
"""
from __future__ import annotations

import random
from collections import deque
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from serpens.perception.aruco_locator import ArucoLocator
from serpens.perception.person_detector import PersonDetection
from serpens.sim.virtual_camera import SimPersonDetector, VirtualCamera

BBOX_NONE = (0.0, 0.0, 0.0, 0.0)
MARKERS = ("neck", "tail")


@dataclass(frozen=True)
class Observation:
    """1回の観測。neck / tail は見えなければ None。"""

    t_capture: float
    neck_mm: np.ndarray | None
    tail_mm: np.ndarray | None
    people: list[PersonDetection]
    rejected_gap: bool = False


@dataclass(frozen=True)
class Window:
    """[t0, t1) の間だけ有効な故障。"""

    t0: float
    t1: float

    def on(self, t: float) -> bool:
        return self.t0 <= t < self.t1


@dataclass
class VisionFaults:
    """**試験用の故障注入。** 実環境の値ではない。"""

    dropout_ratio: float = 0.0                         # フレームごと失う確率
    blackout: list[Window] = field(default_factory=list)            # カメラが丸ごと止まる
    occlude: list[tuple[Window, str]] = field(default_factory=list)  # (期間, "neck"/"tail") を手で隠す
    ghost_marker: list[tuple[Window, str, tuple[float, float]]] = field(default_factory=list)  # 偽マーカ
    ghost_person: list[tuple[Window, tuple[float, float]]] = field(default_factory=list)       # 偽の人
    pixel_noise_std: float = 0.0                       # 画素ノイズ（標準偏差, 0-255）
    occlude_radius_mm: float = 45.0                    # 隠す円の半径（マーカ 50mm + 余白を覆う）

    def any_on(self, items: list, t: float) -> list:
        return [it for it in items if (it[0] if isinstance(it, tuple) else it).on(t)]


class GroundTruthObserver:
    """真値をそのまま返す（**Vision の成果ではない**）。"""

    pose_source = "sim"

    def observe(self, session: Any, t: float) -> Observation:
        w = session.world
        people = [PersonDetection(BBOX_NONE, 1.0, np.array([p.x_mm, p.y_mm])) for p in session.people]
        return Observation(t, w.marker_xy("neck"), w.marker_xy("tail"), people)


class SimVisionObserver:
    """仮想カメラ → ArUco 検出 → 床座標。人は描いた bbox → 足元 → 床座標。"""

    pose_source = "aruco_sim"

    def __init__(self, cfg: dict[str, Any], faults: VisionFaults | None = None, seed: int = 0) -> None:
        v = cfg["vision_sim"]
        self.period_s = 1.0 / float(v["frame_hz"])
        self.latency_s = float(v["latency_s"])
        self.cam = VirtualCamera(cfg)
        self.locator = ArucoLocator(cfg, self.cam.homography)
        self.people_det = SimPersonDetector(self.cam, self.cam.homography)
        self.faults = faults or VisionFaults()
        self.rng = random.Random(seed)
        self.np_rng = np.random.default_rng(seed)
        self._next_capture = 0.0
        self._pending: deque[tuple[float, Observation]] = deque()   # (届く時刻, 観測)
        self.frames = 0
        self.dropped = 0
        self.last_image: np.ndarray | None = None

    def observe(self, session: Any, t: float) -> Observation | None:
        """t までに届いた最新の観測（無ければ None）。撮影もここで行う。"""
        if t + 1e-9 >= self._next_capture:
            self._next_capture = t + self.period_s
            obs = self._capture(session, t)
            if obs is not None:
                self._pending.append((t + self.latency_s, obs))
        ready = None
        while self._pending and self._pending[0][0] <= t + 1e-9:
            ready = self._pending.popleft()[1]
        return ready

    def _capture(self, session: Any, t: float) -> Observation | None:
        f = self.faults
        self.frames += 1
        if f.any_on(f.blackout, t) or self.rng.random() < f.dropout_ratio:
            self.dropped += 1
            return None
        world = session.world
        img = self.cam.render(world, session.people)
        for _w, which in f.any_on(f.occlude, t):
            self.cam.occlude(img, world.marker_xy(which), f.occlude_radius_mm)
        for _w, which, xy in f.any_on(f.ghost_marker, t):
            self.cam.draw_extra_marker(img, which, np.array(xy), world.marker_tangent(which))
        if f.pixel_noise_std > 0:
            noise = self.np_rng.normal(0.0, f.pixel_noise_std, img.shape)
            img = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        self.last_image = img
        m = self.locator.observe(img)
        people = self.people_det.detect(img)
        people += [PersonDetection(BBOX_NONE, 1.0, np.array(xy, float))
                   for _w, xy in f.any_on(f.ghost_person, t)]
        return Observation(t, m.neck_mm, m.tail_mm, people, m.rejected_gap)
