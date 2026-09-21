"""制御ループ（別スレッド）と、GUI へ渡すスナップショット。

制御スレッドで例外が起きたら、黙って死なせない。緊急停止をラッチして出力へ送り、
`Snapshot.fault` に理由を載せて GUI とコンソールへ伝える。終了時は
「停止を出力へ送る → スレッド join → 接続を閉じる」を必ず通り、失敗も報告する。

GUI の描画が重くても制御周期が乱れないよう、制御は専用スレッドで一定周期（behavior.tick_hz）で回し、
GUI は最新のスナップショットを 10fps で読むだけにする。実周期の平均・最悪値を測って表示する。

  --sim        … シミュレータ（SimSession）。人はキーやシナリオで動かす
  --camera N   … 実カメラ + ArUco + YOLO を別スレッド（perception_hz）で回し、ヘビはシミュレータ
  --bus feetech … 実機のサーボへ送る（実機テストは未実施）
"""
from __future__ import annotations

import logging
import os
import threading
import time
from dataclasses import dataclass, field, replace
from typing import Any

import numpy as np

from serpens.behavior.brain import BrainStatus
from serpens.hw.servo_bus import ServoState
from serpens.safety import DriveState
from serpens.sim.session import SimSession
from serpens.sim.virtual_camera import SimPerson

log = logging.getLogger(__name__)
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
    servo_age_s: dict[int, float | None] = field(default_factory=dict)   # 軸ごとの古さ（None = 未取得）
    telemetry_source: str = ""          # 値の出どころ（MockServoBus / FeetechServoBus）
    missing_axes: list[int] = field(default_factory=list)
    drive_state: str = DriveState.HOLD.value
    drive_text: str = ""                # 「停止（姿勢保持）: 理由」
    latched: bool = False
    blockers: list[str] = field(default_factory=list)      # 自律走行を許可できない理由
    fault: str = ""                     # 制御スレッドの例外・停止失敗
    head_link: str = "なし"
    commanded: dict[str, float] = field(default_factory=dict)   # PC が出した角度
    measured: dict[str, float] = field(default_factory=dict)    # 読み戻した角度
    device: Any = None                  # 機体（ESP32）の状態。直接経路では None
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
        self.paused = False                  # **シミュレーションの一時停止だけ**（実機停止ではない）
        self.message = ""
        self.fault = ""
        self.shutdown_errors: list[str] = []

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
            try:
                if self.paused:
                    self.session.enforce_stop_output()   # 一時停止中も保持指令は出す
                else:
                    self.session.step()
            except Exception as e:                       # noqa: BLE001 - 何が起きても機体を止める
                self._on_fault(e)
                break
            self.stats.add(now - last)
            last = now
            next_t += self.dt
            if self.realtime and next_t < now - self.dt:   # 大きく遅れたら追いつくのをあきらめる
                next_t = now + self.dt
            self._publish()

    def _on_fault(self, e: Exception) -> None:
        """制御スレッドの例外: 先に機体を止めてから、理由を公開する。

        self.fault は GUI と監視が「異常を検知した」として読む唯一の合図なので、
        緊急停止をラッチし切る前に代入してはいけない。先に代入すると、
        fault が見えているのにまだ停止していない瞬間が外から観測できてしまう。
        """
        fault = f"制御ループ例外: {type(e).__name__}: {e}"
        log.exception("制御ループが例外で停止しました")
        try:
            self.session.request_emergency(fault, "system")
            self.session.enforce_stop_output()
        except Exception as e2:                          # noqa: BLE001 - 停止失敗も握りつぶさない
            fault += f" / 停止要求も失敗: {e2}"
            log.error("停止要求に失敗: %s", e2)
        self.fault = fault                               # 停止が済んでから公開する
        self._publish()

    # ---- 停止・終了 --------------------------------------------------------------
    def request_stop(self, reason: str, source: str = "操作") -> bool:
        """通常停止（出力へ届く）。"""
        ok = self.session.request_stop(reason, source)
        self.session.enforce_stop_output()
        return ok

    def request_emergency(self, reason: str, source: str = "操作") -> bool:
        """緊急停止（ラッチ・出力へ届く）。"""
        ok = self.session.request_emergency(reason, source)
        self.session.enforce_stop_output()
        return ok

    def shutdown(self, timeout_s: float = 2.0) -> list[str]:
        """停止を出力へ送り、スレッドを止め、接続を閉じる。失敗の一覧を返す。"""
        errors: list[str] = []
        try:
            self.session.request_stop("終了", "system")
            self.session.enforce_stop_output()
        except Exception as e:                           # noqa: BLE001
            errors.append(f"終了時の停止要求に失敗: {e}")
        self._stop_evt.set()
        if self.is_alive():
            self.join(timeout=timeout_s)
            if self.is_alive():
                errors.append(f"制御スレッドが {timeout_s}s で終了しませんでした")
        try:
            errors += self.session.close()
        except Exception as e:                           # noqa: BLE001
            errors.append(f"接続の切断に失敗: {e}")
        if self.fault:
            errors.append(self.fault)
        self.shutdown_errors = errors
        for msg in errors:
            log.error("%s", msg)
        try:
            self._publish()                              # 終了処理は例外を外へ出さない
        except Exception as e:                           # noqa: BLE001
            errors.append(f"終了時の状態公開に失敗: {e}")
            log.exception("終了時の状態公開に失敗")
        return errors

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
            waypoint=None if s.brain.loco.waypoint is None else (float(s.brain.loco.waypoint[0]), float(s.brain.loco.waypoint[1])),
            servo=dict(s.poller.fresh_states(s.t)),
            servo_age_s={sid: s.poller.age_s(sid, s.t) for sid in s.servo_ids},
            telemetry_source=s.poller.source, missing_axes=s.poller.missing_axes(s.t),
            commanded=dict(s.anim.last_output), measured=s.robot.positions_by_name(),
            device=s.robot.device_status(),
            drive_state=s.stop.state.value, drive_text=s.stop.status_text(), latched=s.stop.latched,
            blockers=s.blockers(), fault=self.fault,
            head_link=("なし" if s.head is None else ("接続" if s.head.link_ok() else "断")),
            stats=self.stats, message=self.message)
        with self._lock:
            self.snapshot = snap

    def latest(self) -> Snapshot:
        """いまの状態。**`fault` と `message` は必ず最新の値を載せる。**

        `self.fault = ...` と `_publish()` の間には僅かな隙間がある。そこで読まれると
        「異常を検知したのに、画面には異常なしと出ている」瞬間が生まれる（実際に
        テストが 60 回に 1〜2 回の頻度で捕まえた）。読み出し側で上書きして隙間を消す。
        """
        with self._lock:
            snap = self.snapshot
        return replace(snap, fault=self.fault, message=self.message)

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
