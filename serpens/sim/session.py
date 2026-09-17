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
import math
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
from serpens.perception.person_detector import PersonTracker, TrackedPerson
from serpens.robot import DirectRobot, MotionCommand, RobotInterface
from serpens.perception.snake_pose import SnakePose, SnakePoseTracker
from serpens.safety import AutonomyInputs, DriveState, StopSupervisor, autonomy_blockers
from serpens.sim.sim_vision import GroundTruthObserver, Observation
from serpens.sim.virtual_camera import SimPerson
from serpens.sim.world import BodyPose, World



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
                 head: HeadIO | None = None, robot_is_real: bool = False, pose_source: str = "sim",
                 robot: RobotInterface | None = None, clock: "ManualClock | None" = None,
                 observer: Any | None = None) -> None:
        cfg = copy.deepcopy(cfg)
        for section, values in (overrides or {}).items():
            cfg[section].update(values)
        self.cfg = cfg
        self.errors: list[str] = []                # 出力・切断の失敗をためる（握りつぶさない）
        self.clock = clock if clock is not None else ManualClock()
        self.ctrl_dt = 1.0 / float(cfg["behavior"]["tick_hz"])
        self.sub = max(int(round(self.ctrl_dt / float(cfg["sim"]["dt_s"]))), 1)
        self.robot_is_real = robot_is_real
        # 観測の経路。既定はシミュレータの真値。SimVisionObserver で「画像 → ArUco」を通す（Phase 4）
        self.observer = observer if observer is not None else GroundTruthObserver()
        # "sim"（真値）/ "aruco_sim"（模擬画像の Vision）/ "aruco"（実観測）
        self.pose_source = pose_source if observer is None else str(self.observer.pose_source)
        self.pose_max_age_s = float(cfg["behavior"]["safety"]["autonomy"]["pose_max_age_s"])
        self.world = World(cfg, start)
        # --- 出力先（注入されたものをそのまま全員で使う） ---
        # 駆動リンク経路（robot を注入）ではローカルのサーボバスを作らない。
        # **使わないモックを実機の代わりに置かない**（値の出どころを偽らないため）
        self.owns_bus = bus is None and robot is None
        self.bus: ServoBus | None = bus if bus is not None else (
            None if robot is not None else MockServoBus(cfg, clock=self.clock))
        if self.owns_bus and self.bus is not None:
            self.bus.connect()
            for sid in self.bus.ids:
                self.bus.set_torque(sid, True)
        # 実機経路でモックの頭部を実センサーとして扱わない（未接続なら None のまま）
        self.head: HeadIO | None = head if head is not None else (None if robot_is_real else MockHeadIO(cfg, clock=self.clock))
        self.owns_head = head is None and self.head is not None
        # 出力先。ここを差し替えると駆動リンク経路になる（serpens/link/robot.py）
        self.robot: RobotInterface = robot if robot is not None else DirectRobot(cfg, self.bus, self.clock)
        self.errors += list(getattr(self.robot, "errors", []))
        self.poller = self.robot.telemetry           # 状態の読み出し（GUI・開始条件が見る）
        self.anim = Animator(cfg)
        self.brain = Brain(cfg, self.anim, self.robot.torque_sink, self.head, random.Random(seed))
        self.snake_tracker = SnakePoseTracker(cfg)
        self.person_tracker = PersonTracker(cfg)
        # 実機と、Vision を通す模擬は待機から始める（自己位置が入ってから開始操作で動く）。
        # 真値のシミュレーションのデモは従来どおりすぐ動く
        self.observes_truth = isinstance(self.observer, GroundTruthObserver)
        waits = robot_is_real or not self.observes_truth
        self.stop = StopSupervisor(cfg, self.clock, initial=DriveState.HOLD if waits else DriveState.RUN)
        self.drive_link_ok = self.robot.link_ok or not robot_is_real
        self.people: list[SimPerson] = []
        self.snake: SnakePose | None = None
        self.target: TrackedPerson | None = None
        self.last_observation: Observation | None = None
        # Vision の位置は、最初と見失った後に**人が画面で確かめる**まで走行に使わない
        # （偽マーカで再取得した位置のまま走らないため。docs/phase4_vision_bridge.md §4）
        self.pose_needs_ack = not self.observes_truth
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

    @property
    def servo_ids(self) -> list[int]:
        """関節順のサーボ ID（出力先がバスでもリンクでも同じ）。"""
        return list(self._ids.values())

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
            torque_ceiling_ok=self.robot.torque_ceiling_ok,
            telemetry_axes=len(self.poller.fresh_states(now)),
            expected_axes=len(self._ids),
            telemetry_age_s=self.poller.newest_age_s(now),
        )

    def blockers(self) -> list[str]:
        """いま自律走行を許可できない理由（空なら開始できる）。"""
        return autonomy_blockers(self.cfg, self.autonomy_inputs()) + self.pose_blockers()

    def pose_blockers(self) -> list[str]:
        """自己位置が無い・古い（**模擬でも**。見えていない位置で走らせない）。真値は常に最新。"""
        if self.observes_truth:
            return []
        age = self.snake_tracker.age_s(self.t)
        if self.snake is None:
            return ["自己位置が未取得"]
        if age > self.pose_max_age_s:
            return [f"自己位置が古い（{age:.2f}s > {self.pose_max_age_s}s）"]
        if self.pose_needs_ack:
            return [f"自己位置の確認待ち（{self.pose_text()}）。画面の位置が合っていれば K で確認"]
        return []

    def pose_text(self) -> str:
        """推定した自己位置を人が読める形に。"""
        if self.snake is None:
            return "未取得"
        return f"x={self.snake.x:.0f} y={self.snake.y:.0f} θ={math.degrees(self.snake.theta_body):.0f}°"

    def acknowledge_pose(self, source: str = "操作") -> tuple[bool, str]:
        """人が画面で自己位置を確かめた（新しい位置が入っていて古くないときだけ受け付ける）。"""
        if self.observes_truth:
            return True, "真値の模擬では確認は不要"
        was = self.pose_needs_ack
        self.pose_needs_ack = False
        lost = self.pose_blockers()
        if lost:
            self.pose_needs_ack = was
            return False, "確認できません: " + lost[0]
        return True, f"自己位置を確認（{self.pose_text()}, {source}）"

    def request_start(self, source: str = "操作") -> tuple[bool, list[str]]:
        """走行を開始する（実機は条件を満たさないと開始しない）。"""
        ok, why = self.stop.start(self.blockers(), source)
        if ok:
            self.robot.on_operator_start()      # 機体の再起動後はこの操作だけが再開を許す
            self._resume_output()
        return ok, why

    def request_stop(self, reason: str, source: str = "操作") -> bool:
        """通常停止（姿勢保持）。"""
        return self.stop.stop(reason, source)

    def request_emergency(self, reason: str, source: str = "操作") -> bool:
        """緊急停止（ラッチ）。"""
        return self.stop.emergency(reason, source)

    def clear_emergency(self, source: str = "操作") -> bool:
        """緊急停止の解除 → 待機。走行は再開しない（機体側のラッチも解く）。"""
        ok = self.stop.clear_emergency(source)
        if ok:
            self.robot.on_clear_emergency()
        return ok

    def request_disable_torque(self, reason: str, source: str = "操作") -> bool:
        """駆動無効化（脱力）。停止中の明示操作のみ。"""
        return self.stop.disable_torque(reason, source)

    # ---- 1周期 -----------------------------------------------------------------------
    def step(self) -> BrainStatus | None:
        """制御周期1回ぶん進める。停止中は監視と保持だけ行う。"""
        self.robot.poll()                                   # 停止中も監視は続ける
        for _ in range(self.sub):
            self.clock.t += self.ctrl_dt / self.sub
            self.world.step(self._world_angles(), self.ctrl_dt / self.sub)
        t = self.clock.t
        self._observe(t)
        lost = self.pose_blockers() if self.stop.moving_allowed else []
        if lost:
            self.pose_needs_ack = True                                     # 戻った位置は人が確かめる
            self.request_stop(f"自己位置を見失った: {lost[0]}", source="知覚")   # 復帰は開始操作で
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
        self.robot.send(t, MotionCommand.from_animator(self.anim, self.robot.body, t))
        return self.status

    def _world_angles(self) -> dict[str, float]:
        """世界を進めるための関節角。実機では参考値（実位置は Phase 4 で ArUco から入れる）。"""
        return self.robot.positions_by_name()

    def _observe(self, t: float) -> None:
        """ヘビと人の位置を更新する（停止中も更新して、画面と復旧判断に使う）。"""
        j8 = self.poller.positions.get(self._ids["J8"], 0.0)
        obs = self.observer.observe(self, t)
        if obs is None:                                   # 新しい観測なし: 前回値を保持（古さは増える）
            self.snake = self.snake_tracker.update(t, None, None, j8)
            return
        self.last_observation = obs
        self.snake = self.snake_tracker.update(obs.t_capture, obs.neck_mm, obs.tail_mm, j8)
        before = self.target
        ref = None if self.snake is None else np.array([self.snake.x, self.snake.y])
        self.target = self.person_tracker.update(obs.t_capture, obs.people, ref)
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
        self.robot.apply_stop_state(state)               # 経路ごとの停止の出し方
        if self.stop.torque_should_be_off:
            return
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
        self.robot.hold(pose)

    def _resume_output(self) -> None:
        """走行に戻すとき、固定を解除してトルクを戻す。"""
        self.robot.apply_stop_state(DriveState.RUN)     # 走行中であることを出力先へ伝え続ける
        if self._last_drive is None or self._last_drive is DriveState.RUN:
            self._last_drive = DriveState.RUN
            return
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
        self.errors += [e for e in self.robot.close(self.stop.disable_torque_on_exit)
                        if e not in self.errors]
        if self.head is not None:
            self._guard(self.head.close, "頭部 I/O の切断")
        return list(self.errors)
