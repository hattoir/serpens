"""制御ループ（別スレッド）と、GUI へ渡すスナップショット。

GUI の描画が重くても制御周期が乱れないよう、制御は専用スレッドで一定周期（behavior.tick_hz）で回し、
GUI は最新のスナップショットを 10fps で読むだけにする。実周期の平均・最悪値を測って表示する。

  --sim        … シミュレータ（SimSession）。人はキーやシナリオで動かす
  --camera N   … 実カメラ + ArUco + YOLO を別スレッド（perception_hz）で回し、ヘビはシミュレータ
  --bus feetech … 実機のサーボへ送る（実機テストは未実施）
"""
from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from serpens.behavior.brain import BrainStatus
from serpens.hw.servo_bus import ServoState
from serpens.sim.session import SimSession
from serpens.sim.virtual_camera import SimPerson

THREAD_PRIORITY_ABOVE_NORMAL = 1


@dataclass
class LoopStats:
    """制御周期の実測。"""

    target_hz: float
    count: int = 0
    mean_ms: float = 0.0
    worst_ms: float = 0.0
    late_count: int = 0            # 目標周期を 20% 以上超えた回数

    def add(self, dt: float) -> None:
        ms = dt * 1000.0
        self.count += 1
        self.mean_ms += (ms - self.mean_ms) / self.count
        self.worst_ms = max(self.worst_ms, ms)
        if ms > 1200.0 / self.target_hz:
            self.late_count += 1


@dataclass
class Snapshot:
    """GUI が描くための、その時点の状態一式。"""

    t: float = 0.0
    status: BrainStatus | None = None
    points: np.ndarray | None = None          # ヘビの世界座標（尾端…頭先端）
    snake_xy: tuple[float, float] | None = None
    theta_body: float = 0.0
    theta_head: float = 0.0
    people: list[tuple[float, float]] = field(default_factory=list)
    target_xy: tuple[float, float] | None = None
    waypoint: tuple[float, float] | None = None
    servo: dict[int, ServoState] = field(default_factory=dict)
    frame: np.ndarray | None = None           # カメラ画像（BGR）
    detections: list[tuple[float, float, float, float]] = field(default_factory=list)
    tracked_bbox: tuple[float, float, float, float] | None = None
    markers: dict[int, np.ndarray] = field(default_factory=dict)
    stats: LoopStats | None = None
    message: str = ""


class ControlLoop(threading.Thread):
    """一定周期で session.step() を回すスレッド。"""

    def __init__(self, session: SimSession, realtime: bool = True) -> None:
        super().__init__(name="control", daemon=True)
        self.session = session
        self.realtime = realtime
        self.dt = session.ctrl_dt
        self.stats = LoopStats(1.0 / self.dt)
        self.snapshot = Snapshot(stats=self.stats)
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()   # Thread._stop と名前が衝突しないように
        self.paused = False
        self.message = ""

    def stop(self) -> None:
        self._stop_evt.set()

    def run(self) -> None:
        self._boost_timer()
        next_t = time.perf_counter()
        last = next_t
        while not self._stop_evt.is_set():
            now = time.perf_counter()
            if self.realtime and now < next_t:
                time.sleep(min(next_t - now, self.dt))
                continue
            if not self.paused:
                self.session.step()
            self.stats.add(now - last)
            last = now
            next_t += self.dt
            if self.realtime and next_t < now - self.dt:   # 大きく遅れたら追いつくのをあきらめる
                next_t = now + self.dt
            self._publish()

    @staticmethod
    def _boost_timer() -> None:
        """Windows: タイマ分解能を 1ms にし、制御スレッドの優先度を上げる。

        既定のタイマ分解能は 15.6ms で、GUI と同時に動かすと制御周期が数十 ms 跳ねることがある。
        """
        if os.name != "nt":
            return
        try:
            import ctypes

            ctypes.windll.winmm.timeBeginPeriod(1)
            handle = ctypes.windll.kernel32.GetCurrentThread()
            ctypes.windll.kernel32.SetThreadPriority(handle, THREAD_PRIORITY_ABOVE_NORMAL)
        except Exception as e:                                   # noqa: BLE001 - 効かなくても動作は続ける
            print(f"[注意] タイマ分解能・優先度の設定に失敗: {e}")

    def _publish(self) -> None:
        s = self.session
        snap = Snapshot(
            t=s.t, status=s.status, points=s.world.world_points().copy(),
            snake_xy=None if s.snake is None else (s.snake.x, s.snake.y),
            theta_body=0.0 if s.snake is None else s.snake.theta_body,
            theta_head=0.0 if s.snake is None else s.snake.theta_head,
            people=[(p.x_mm, p.y_mm) for p in s.people],
            target_xy=None if s.target is None else (float(s.target.floor_mm[0]), float(s.target.floor_mm[1])),
            waypoint=None if s.brain._waypoint is None else (float(s.brain._waypoint[0]), float(s.brain._waypoint[1])),
            servo=dict(s.poller.states), stats=self.stats, message=self.message)
        with self._lock:
            self.snapshot = snap

    def latest(self) -> Snapshot:
        with self._lock:
            return self.snapshot

    # ---- 操作（キーボードから呼ぶ） ------------------------------------------------
    def add_person(self, x: float, y: float) -> None:
        self.session.people = [SimPerson(x, y)]
        self.message = f"人を ({x:.0f}, {y:.0f}) に置いた"

    def clear_people(self) -> None:
        self.session.people = []
        self.message = "人を消した"

    def touch(self, on: bool) -> None:
        self.session.touch(on)
        self.message = "タッチ ON" if on else "タッチ OFF"
