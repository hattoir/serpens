"""実カメラ / 録画 → 観測（Phase 4 の実画像経路）。

`SimSession(observer=CameraObserver(...))` で差し込む。画像の入力元は
`serpens.perception.camera.open_source`（カメラ番号 / 動画 / 静止画）。

    pose_source = "aruco"       … 生きたカメラ（番号指定）。実機の開始条件が要求する出どころ
    pose_source = "aruco_file"  … 動画・静止画。**実画像だが生放送ではない**ので開始条件を満たさない
                                 （録画を再生しながら実機を走らせると、機体の実位置と無関係な位置で動く）

観測には**撮影した時刻**（読み出し時の clock）を付け、追跡器へはその時刻で入れる。
検出に時間がかかっても古さがそのまま出る（隠さない）。

床の校正（`homography.file`）が無ければ自己位置は出せない。画像は表示だけして、
セッションは自己位置なしのまま待機を続ける（`pose_blockers`）。
"""
from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Any, Callable

import cv2
import numpy as np

from serpens.perception.camera import FrameSource, VideoFileSource, open_source
from serpens.perception.person_detector import PersonDetection
from serpens.sim.sim_vision import Observation

MARK_BGR = (0, 0, 255)
PERSON_BGR = (200, 200, 200)


def source_kind(spec: str) -> str:
    """指定文字列から pose_source を決める（カメラ番号だけが "aruco"）。"""
    return "aruco" if spec.isdigit() else "aruco_file"


class CameraObserver(threading.Thread):
    """画像を読み、ArUco と人を検出して `Observation` にする。

    スレッドで `detect_hz` 回だけ回す。`observe()` は制御スレッドから呼ばれ、
    未消費の最新観測を1つ返す（無ければ None）。試験では `step()` を直接呼ぶ。
    """

    def __init__(self, cfg: dict[str, Any], spec: str, clock: Callable[[], float],
                 homography_path: str | Path | None = None, want_people: bool = True) -> None:
        super().__init__(name="camera-observer", daemon=True)
        self.cfg, self.clock = cfg, clock
        self.pose_source = source_kind(spec)
        self.hz = float(cfg["person"]["detect_hz"])
        self.hold_s = float(cfg["person"]["hold_s"])
        self.src: FrameSource = open_source(cfg, spec)
        if isinstance(self.src, VideoFileSource):
            self.src.loop = False        # 録画が先頭へ戻ると位置が飛ぶ。終わったら「観測なし」にする
        self.notes: list[str] = []
        self.frame: np.ndarray | None = None
        self.frames = 0
        self.locator = self.people = None
        hpath = Path(homography_path if homography_path is not None else cfg["homography"]["file"])
        if hpath.exists():
            from serpens.perception.aruco_locator import ArucoLocator
            from serpens.perception.homography import FloorHomography

            h = FloorHomography.load(hpath)
            self.locator = ArucoLocator(cfg, h)
            if want_people:
                try:
                    from serpens.perception.person_detector import YoloPersonDetector

                    self.people = YoloPersonDetector(cfg, h)
                except (FileNotFoundError, ImportError) as e:
                    self.notes.append(f"人物検出なし: {e}")
        else:
            self.notes.append(f"床の校正がありません（{hpath}）。自己位置は出せません。映像のみ表示します")
        self._lock = threading.Lock()
        self._latest: Observation | None = None
        self._stop_evt = threading.Event()

    # ---- 観測器としての顔（SimSession が呼ぶ） --------------------------------------
    def observe(self, _session: Any, _t: float) -> Observation | None:
        with self._lock:
            obs, self._latest = self._latest, None
        return obs

    # ---- 1フレーム -------------------------------------------------------------------
    def step(self) -> Observation | None:
        """1枚読んで検出する。読めなければ None（観測なし）。"""
        ok, frame = self.src.read()
        if not ok or frame is None:
            return None
        t_capture = float(self.clock())
        self.frames += 1
        if self.locator is None:
            self.frame = frame
            return None
        m = self.locator.observe(frame)
        dets: list[PersonDetection] = [] if self.people is None else self.people.detect(frame)
        obs = Observation(t_capture, m.neck_mm, m.tail_mm, dets, m.rejected_gap)
        self.frame = self._annotate(frame, m, dets)
        with self._lock:
            self._latest = obs
        return obs

    @staticmethod
    def _annotate(frame: np.ndarray, m: Any, dets: list[PersonDetection]) -> np.ndarray:
        out = frame.copy()
        for c in m.corners_px.values():
            cv2.polylines(out, [c.astype(np.int32)], True, MARK_BGR, 3)
        for d in dets:
            x1, y1, x2, y2 = (int(v) for v in d.bbox_px)
            cv2.rectangle(out, (x1, y1), (x2, y2), PERSON_BGR, 2)
        return out

    # ---- スレッド ----------------------------------------------------------------------
    def stop(self) -> None:
        self._stop_evt.set()

    def run(self) -> None:
        period = 1.0 / self.hz
        last_ok = time.monotonic()
        while not self._stop_evt.is_set():
            t0 = time.perf_counter()
            if self.step() is not None:
                last_ok = time.monotonic()
            elif time.monotonic() - last_ok > self.hold_s:
                self.frame = None                       # 画像が止まったことを画面に出す
            time.sleep(max(period - (time.perf_counter() - t0), 0.0))

    def latest(self) -> np.ndarray | None:
        return self.frame

    def release(self) -> None:
        self.stop()
        if self.is_alive():
            self.join(timeout=1.0)
        try:
            self.src.release()
        except Exception as e:                    # noqa: BLE001 - 解放失敗も伝える
            print(f"[注意] カメラの解放に失敗: {e}")
