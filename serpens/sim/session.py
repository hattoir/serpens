"""実行セッション: 世界・サーボ・頭部・知覚・行動を1つにまとめる。

サーボバスと頭部 I/O は**外から注入**する（`bus` / `head`）。注入しなければモックを作る。
これで状態監視（poller）・位置指令（animator）・トルク操作（expression）が
**必ず同じ接続先**を参照する。実機のとき `session.bus` だけ差し替える、ということをしない。

停止の扱い（`serpens/safety.py`）:
  - 停止中も**監視は続ける**（温度・負荷・電圧を読み続ける）
  - 停止中は行動を回さず、**保持指令だけを出力へ送り続ける**（最後の目標まで動き続けるのを止める）
  - 停止で勝手にホーム姿勢へ動かさない。脱力は明示操作（DISABLED）のときだけ
  - 実機では実観測が揃うまで自律走行を開始できない（`autonomy_blockers`）

**PC 側だけでは PC 停止・USB 断のときに機体を止められない。** ESP32 の watchdog が必要（Phase 2）。
"""
from __future__ import annotations

import copy
import random
from pathlib import Path
from typing import Any

import numpy as np

from serpens.behavior.brain import Brain, BrainStatus, Percept
from serpens.hw.head_io import HeadIO, MockHeadIO
from serpens.hw.mock_bus import MockServoBus
from serpens.hw.servo_bus import Goal, ServoBus, ServoCommError
from serpens.hw.state_poller import ServoStatePoller
from serpens.motion.animator import Animator
from serpens.perception.person_detector import PersonDetection, PersonTracker, TrackedPerson
from serpens.perception.snake_pose import SnakePose, SnakePoseTracker
from serpens.safety import AutonomyInputs, DriveState, StopSupervisor, autonomy_blockers
from serpens.sim.virtual_camera import SimPerson
from serpens.sim.world import BodyPose, World

BBOX_NONE = (0.0, 0.0, 0.0, 0.0)


class ManualClock:
    """シミュレーション時刻。"""

    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


class SimSession:
    """観測 → 判断 → 出力 を1周期ずつ回す。実機でもシミュレータでも同じ手順。"""

    def __init__(self, cfg: dict[str, Any], start: BodyPose | None = None, seed: int | None = None,
                 overrides: dict[str, dict[str, Any]] | None = None, bus: ServoBus | None = None,
                 head: HeadIO | None = None, robot_is_real: bool = False, pose_source: str = "sim") -> None:
        cfg = copy.deepcopy(cfg)
        for section, values in (overrides or {}).items():
            cfg[section].update(values)
        self.cfg = cfg
        self.errors: list[str] = []                # 出力・切断の失敗をためる（握りつぶさない）
        self.clock = ManualClock()
        self.ctrl_dt = 1.0 / float(cfg["behavior"]["tick_hz"])
        self.sub = max(int(round(self.ctrl_dt / float(cfg["sim"]["dt_s"]))), 1)
        self.robot_is_real = robot_is_real
        self.pose_source = pose_source            # "sim"（仮想世界）/ "aruco"（実観測）
        self.world = World(cfg, start)
        # --- 出力先（注入されたものをそのまま全員で使う） ---
        self.owns_bus = bus is None
        self.bus: ServoBus = bus if bus is not None else MockServoBus(cfg, clock=self.clock)
        if self.owns_bus:
            self.bus.connect()
            for sid in self.bus.ids:
                self.bus.set_torque(sid, True)
        # 実機経路でモックの頭部を実センサーとして扱わない（未接続なら None のまま）
        self.head: HeadIO | None = head if head is not None else (None if robot_is_real else MockHeadIO(cfg, clock=self.clock))
        self.owns_head = head is None and self.head is not None
        failed = self.bus.apply_torque_ceiling()     # 安全上限（構想設計書 16章）を必ず通す
        if failed:
            self.errors.append(f"トルク上限を設定できなかった軸: {failed}")
        self.poller = ServoStatePoller(self.bus, cfg, self.clock)
        self.anim = Animator(cfg)
        self.brain = Brain(cfg, self.anim, self.bus, self.head, random.Random(seed))
        self.snake_tracker = SnakePoseTracker(cfg)
        self.person_tracker = PersonTracker(cfg)
        # 実機は必ず待機から始める。シミュレーションのデモは従来どおりすぐ動く
        self.stop = StopSupervisor(cfg, self.clock,
                                   initial=DriveState.HOLD if robot_is_real else DriveState.RUN)
        self.drive_link_ok = not robot_is_real     # 実機の駆動リンク（ESP32）は Phase 2 まで無い
        self.people: list[SimPerson] = []
        self.snake: SnakePose | None = None
        self.target: TrackedPerson | None = None
        self._serial = 0
        self._switches = 0
        self._ids = {j["name"]: int(j["servo_id"]) for j in cfg["joints"]}
        self._hold_pose: dict[str, float] | None = None
        self._next_hold_send = 0.0
        self._torque_off = False
        self._last_drive: DriveState | None = None
        self.status: BrainStatus | None = None

    @property
    def t(self) -> float:
        return self.clock.t

    def touch(self, on: bool, where: str = "head") -> None:
        """頭部のタッチセンサを押す / 離す（モックのときだけ）。"""
        if isinstance(self.head, MockHeadIO):
            setattr(self.head, "touch_head" if where == "head" else "touch_back", on)

    # ---- 停止・開始（外からの要求はここを通す） ---------------------------------------
    def autonomy_inputs(self) -> AutonomyInputs:
        """自律走行の開始条件を判定するための事実。"""
        now = self.t
        return AutonomyInputs(
            robot_is_real=self.robot_is_real,
            pose_source=self.pose_source if self.snake is not None else "none",
            pose_age_s=None if self.snake is None else self.snake_tracker.age_s(now),
            calibration_present=Path(self.cfg["homography"]["file"]).exists(),
            drive_link_ok=self.drive_link_ok,
            torque_ceiling_ok=self.bus.torque_ceiling_applied,
            telemetry_axes=len(self.poller.fresh_states(now)),
            expected_axes=len(self.bus.ids),
            telemetry_age_s=self.poller.newest_age_s(now),
        )

    def blockers(self) -> list[str]:
        """いま自律走行を許可できない理由（空なら開始できる）。"""
        return autonomy_blockers(self.cfg, self.autonomy_inputs())

    def request_start(self, source: str = "操作") -> tuple[bool, list[str]]:
        """走行を開始する（実機は条件を満たさないと開始しない）。"""
        ok, why = self.stop.start(self.blockers(), source)
        if ok:
            self._resume_output()
        return ok, why

    def request_stop(self, reason: str, source: str = "操作") -> bool:
        """通常停止（姿勢保持）。"""
        return self.stop.stop(reason, source)

    def request_emergency(self, reason: str, source: str = "操作") -> bool:
        """緊急停止（ラッチ）。"""
        return self.stop.emergency(reason, source)

    def clear_emergency(self, source: str = "操作") -> bool:
        """緊急停止の解除 → 待機。走行は再開しない。"""
        return self.stop.clear_emergency(source)

    def request_disable_torque(self, reason: str, source: str = "操作") -> bool:
        """駆動無効化（脱力）。停止中の明示操作のみ。"""
        return self.stop.disable_torque(reason, source)

    # ---- 1周期 -----------------------------------------------------------------------
    def step(self) -> BrainStatus | None:
        """制御周期1回ぶん進める。停止中は監視と保持だけ行う。"""
        self.poller.poll()                                  # 停止中も監視は続ける
        for _ in range(self.sub):
            self.clock.t += self.ctrl_dt / self.sub
            self.world.step(self._world_angles(), self.ctrl_dt / self.sub)
        t = self.clock.t
        self._observe(t)
        if not self.stop.moving_allowed:
            self.enforce_stop_output()
            return self.status
        self._resume_output()
        touch = False
        if self.head is not None:
            self.head.poll()
            frame = self.head.latest
            touch = bool(frame and (frame.touch_head or frame.touch_back))
        person_xy = None if self.target is None else self.target.floor_mm
        self.status = self.brain.tick(t, Percept(self.snake, person_xy, self._serial, touch,
                                                 self.poller.max_temperature_c(t),
                                                 self.poller.max_abs_load(t) or 0.0))
        self.anim.send(self.bus, self.anim.update(t))
        return self.status

    def _world_angles(self) -> dict[str, float]:
        """世界を進めるための関節角。実機では参考値（実位置は Phase 4 で ArUco から入れる）。"""
        pos = self.poller.positions
        return {n: pos[i] for n, i in self._ids.items() if i in pos}

    def _observe(self, t: float) -> None:
        """ヘビと人の位置を更新する（停止中も更新して、画面と復旧判断に使う）。"""
        j8 = self.poller.positions.get(self._ids["J8"], 0.0)
        self.snake = self.snake_tracker.update(t, self.world.marker_xy("neck"), self.world.marker_xy("tail"), j8)
        dets = [PersonDetection(BBOX_NONE, 1.0, np.array([p.x_mm, p.y_mm])) for p in self.people]
        before = self.target
        self.target = self.person_tracker.update(t, dets, None if self.snake is None else np.array([self.snake.x, self.snake.y]))
        if (before is None and self.target is not None) or self.person_tracker.switches != self._switches:
            self._serial += 1
            self._switches = self.person_tracker.switches

    # ---- 停止の出力（ここが「停止が出力先まで届く」部分） ------------------------------
    def enforce_stop_output(self) -> None:
        """停止状態を出力へ反映する。停止中は毎周期呼ばれる。終了時にも呼ぶ。"""
        state = self.stop.state
        entering = state is not self._last_drive
        self._last_drive = state
        if entering:
            self.anim.gait.stop(immediate=True)             # 歩容を即時に止める（惰性で進ませない）
            dropped = self.brain.expr.clear()               # 予約済みの演出を捨てる（解除後に動き出さない）
            self.anim.freeze(self.t)                        # 呼吸も含めて出力を固定
            self._hold_pose = dict(self.anim.last_output)
            self._next_hold_send = 0.0
            if dropped:
                self.brain._events.append(f"停止: 予約済みの演出 {len(dropped)} 件を破棄")
        if self.stop.torque_should_be_off:
            self._set_torque(False)
            return
        self._set_torque(True)
        self._send_hold()

    def _send_hold(self) -> None:
        """保持指令を送る（現在位置がわかればそこ、わからなければ最後の指令角）。"""
        t = self.t
        if t < self._next_hold_send:
            return
        self._next_hold_send = t + self.stop.hold_resend_period_s
        pose = dict(self._hold_pose or self.anim.last_output)
        for name, sid in self._ids.items():                 # 実測位置が新しければそこで止める
            if self.poller.is_fresh(sid, t) and sid in self.poller.positions:
                pose[name] = self.poller.positions[sid]
        goals = {self._ids[n]: Goal(v, self.bus.joints[self._ids[n]].max_speed_dps,
                                    float(self.cfg["animator"]["default_accel_dps2"]))
                 for n, v in pose.items() if n in self._ids}
        self._guard(lambda: self.bus.sync_set_goals(goals), "保持指令の送信")

    def _set_torque(self, on: bool) -> None:
        if self._torque_off == (not on):
            return
        for sid in self.bus.ids:
            self._guard(lambda sid=sid: self.bus.set_torque(sid, on), f"トルク{'ON' if on else 'OFF'}（ID{sid}）")
        self._torque_off = not on

    def _resume_output(self) -> None:
        """走行に戻すとき、固定を解除してトルクを戻す。"""
        if self._last_drive is None or self._last_drive is DriveState.RUN:
            self._last_drive = DriveState.RUN
            return
        self._set_torque(True)
        self.anim.unfreeze(self.t)
        self._hold_pose = None
        self._last_drive = DriveState.RUN

    def _guard(self, fn: Any, what: str) -> bool:
        """出力の失敗を握りつぶさず記録する。"""
        try:
            fn()
            return True
        except (ServoCommError, OSError) as e:
            msg = f"{what}に失敗: {e}"
            if msg not in self.errors:
                self.errors.append(msg)
            return False

    # ---- 終了 -------------------------------------------------------------------------
    def close(self) -> list[str]:
        """停止を出力へ送ってから、接続を閉じる。失敗は返り値で伝える。"""
        self.request_stop("終了処理", "system")
        self.enforce_stop_output()
        torque_off = self.stop.disable_torque_on_exit
        self._guard(lambda: self.bus.disconnect(torque_off=torque_off), "サーボバスの切断")
        if self.head is not None:
            self._guard(self.head.close, "頭部 I/O の切断")
        return list(self.errors)
