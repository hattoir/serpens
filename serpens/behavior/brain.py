"""行動の中枢: 知覚 → 刺激 → 内部状態 → 効用 → 状態機械 → 各状態の動作。毎制御周期 tick() を呼ぶ。"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from serpens.behavior.controller import Controller, DriveCommand
from serpens.behavior.expression import Expression
from serpens.behavior.fsm import StateMachine
from serpens.behavior.internal_state import InternalState, Stimuli
from serpens.behavior.utility import STATE_LABELS_JA, Context, UtilityModel, thought_line
from serpens.hw.head_io import HeadIO
from serpens.hw.servo_bus import ServoBus
from serpens.motion.animator import Animator, Keyframe, coil_keyframe
from serpens.motion.kinematics import Chain, forward, point_at_body_x
from serpens.motion.poses import HEAD_YAW, NECK, Poses
from serpens.perception.snake_pose import SnakePose, wrap_pi

STILL_STATES = ("ALERT", "OBSERVE", "ENGAGE")          # 止まって人を見る状態
QUIET_STATES = ("SLEEP", "PETTED", "COIL_REST")        # よそ見をしない状態


@dataclass
class Percept:
    """1周期ぶんの知覚。target_serial は追跡対象が変わる（新しく見つける・切り替える）たびに増える。"""

    snake: SnakePose | None
    person_xy: np.ndarray | None
    target_serial: int
    touch: bool
    max_temp_c: float | None


@dataclass
class BrainStatus:
    """GUI・ログに出す情報。"""

    t: float
    state: str
    state_ja: str
    thought: str
    time_to_next_s: float
    utilities: dict[str, float]
    internal: dict[str, float]
    heat_c: float | None
    drive: str
    noticed: bool
    events: list[str] = field(default_factory=list)


class Brain:
    """行動の中枢。"""

    def __init__(self, cfg: dict[str, Any], animator: Animator, bus: ServoBus | None = None,
                 head: HeadIO | None = None, rng: random.Random | None = None) -> None:
        b = cfg["behavior"]
        self.cfg, self.b, self.x, self.st = cfg, b, b["expression"], b["stimuli"]
        seed = b["seed"]
        self.rng = rng or random.Random(seed)
        self.anim, self.poses = animator, Poses(cfg)
        self.chain = Chain.from_cfg(cfg)
        self.m = cfg["markers"]
        self.internal = InternalState(cfg)
        self.utility = UtilityModel(cfg, self.rng)
        self.fsm = StateMachine(cfg)
        self.ctrl = Controller(cfg)
        self.expr = Expression(cfg, animator, self.poses, self.rng, bus, head)
        self.max_range = float(cfg["person"]["max_range_mm"])
        self._serial = 0
        self.noticed = False
        self._novelty = 0.0
        self._touch_until = -1.0
        self._last_person: np.ndarray | None = None
        self._approach_speed = 0.0
        self._waypoint: np.ndarray | None = None
        self._approach_stage = 0
        self._stage_goal_dist = 0.0
        self._approach_start = np.zeros(2)
        self._approach_dir = np.array([1.0, 0.0])
        self._pause_until = -1.0
        self._at_limit = False
        self._limit_person: np.ndarray | None = None
        self._last_snake: SnakePose | None = None
        self._last_person_raw: np.ndarray | None = None
        self._next_eyes_t = 0.0
        self._last_t: float | None = None
        self.drive = DriveCommand(False, reason="待機")
        self._events: list[str] = []
        self.status: BrainStatus | None = None

    # ---- 1周期 ------------------------------------------------------------------
    def tick(self, t: float, p: Percept) -> BrainStatus:
        dt = 0.0 if self._last_t is None else t - self._last_t
        self._last_t = t
        self._events = []
        self._perceive(t, dt, p)
        self._last_snake, self._last_person_raw = p.snake, p.person_xy
        person = p.person_xy if self.noticed else None
        if self._at_limit and (person is None or self._limit_person is None or
                               np.linalg.norm(np.asarray(person) - self._limit_person) > float(self.cfg["person"]["association_gate_mm"])):
            self._at_limit = False
        head_dist = None if (person is None or p.snake is None) else self.ctrl.head_distance(p.snake, person)
        stim = self._stimuli(p, person, head_dist, dt)
        self.internal.update(dt, stim, p.max_temp_c)
        ev = self.utility.evaluate(self.internal, Context(person is not None, head_dist, stim.touch, self._novelty,
                                                          self._at_limit), t)
        tr = self.fsm.step(t, ev.noisy)
        if tr is not None:
            self._events.append(f"{tr.src}→{tr.dst}（{tr.reason}）")
            self._on_enter(t, tr.dst, p.snake, person)
        self._on_tick(t, p.snake, person, head_dist)
        self.expr.update(t)
        if t >= self._next_eyes_t:
            self.expr.set_eyes(self.fsm.state)
            self._next_eyes_t = t + float(self.x["eyes_refresh_s"])
        s = self.fsm.state
        self.status = BrainStatus(t, s, STATE_LABELS_JA[s], thought_line(ev, s, self.fsm.time_to_next(t)), self.fsm.time_to_next(t), ev.noisy,
                                  self.internal.snapshot(), self.internal.heat_c, self.drive.reason, self.noticed,
                                  self._events)
        return self.status

    def _perceive(self, t: float, dt: float, p: Percept) -> None:
        """b. 新しい人を見つけたら、ランダムな遅延のあとで反応する（それまでは気づいていない）。"""
        if p.person_xy is None:
            self.noticed = False
            self.expr.cancel("react")
        elif p.target_serial != self._serial:
            self.noticed = False
            self.expr.cancel("react")
            self.expr.schedule(t + self.expr._u("reaction_delay_s"), "react", self._react)
        self._serial = p.target_serial
        if p.touch:
            self._touch_until = t + float(self.st["touch_hold_s"])
        self._novelty *= math.exp(-dt / float(self.st["novelty_decay_s"])) if dt > 0 else 1.0

    def _react(self, t: float) -> None:
        """c. 反応の始まり: 全停止（驚き）→ 警戒。PETTED 中は割り込まない。"""
        self.noticed = True
        self._novelty = 1.0
        self.expr.surprise(t)
        self._events.append("反応開始（驚き）")
        if self.fsm.state != "PETTED":
            tr = self.fsm.force(t, "ALERT", "新しい人に気づいた")
            if tr is not None:
                self._events.append(f"{tr.src}→{tr.dst}（{tr.reason}）")
                self._on_enter(t, "ALERT", self._last_snake, self._last_person_raw)

    def _stimuli(self, p: Percept, person: np.ndarray | None, head_dist: float | None, dt: float) -> Stimuli:
        s = Stimuli(touch=1.0 if self._last_t is not None and self._last_t <= self._touch_until else 0.0,
                    novelty=self._novelty)
        if person is None or head_dist is None or p.snake is None:
            self._last_person = None
            self._approach_speed = 0.0
            s.alone = 1.0
            return s
        s.presence = 1.0
        s.proximity = min(max(1.0 - head_dist / self.max_range, 0.0), 1.0)
        # 人自身の速度のうち、ヘビに向かう成分（ヘビが近づいて距離が縮むぶんは含めない）
        if self._last_person is not None and dt > 0:
            v_person = (np.asarray(person, float) - self._last_person) / dt
            to_snake = np.array([p.snake.x, p.snake.y]) - np.asarray(person, float)
            n = float(np.linalg.norm(to_snake))
            v = float(v_person @ to_snake) / n if n > 0 else 0.0
            a = 1.0 - math.exp(-dt / float(self.st["approach_lpf_s"]))
            self._approach_speed += a * (v - self._approach_speed)
        self._last_person = np.asarray(person, float).copy()
        s.approach = min(max(self._approach_speed / float(self.st["approach_ref_mm_s"]), 0.0), 1.0)
        yaw_needed = self._yaw_to(p.snake, person)
        s.looking = 1.0 if abs(yaw_needed - self.anim.last_output.get(HEAD_YAW, 0.0)) <= float(self.st["looking_tolerance_deg"]) else 0.0
        return s

    # ---- 頭の向き ---------------------------------------------------------------------
    def _yaw_to(self, snake: SnakePose, target: np.ndarray) -> float:
        """target を見るための J8 [deg]。首リンクの向き = θ_body(生) + 胴体形状による差 で求める。"""
        pts, _ = forward(self.chain, self.anim.last_output)
        chord = point_at_body_x(self.chain, pts, float(self.m["neck_x_mm"])) - point_at_body_x(self.chain, pts, float(self.m["tail_x_mm"]))
        link = pts[self.chain.names.index(NECK) + 1] - pts[self.chain.names.index(NECK)]
        offset = math.atan2(link[1], link[0]) - math.atan2(chord[1], chord[0])
        bearing = math.atan2(target[1] - snake.y, target[0] - snake.x)
        return math.degrees(wrap_pi(bearing - snake.theta_body_raw - offset))

    # ---- 状態ごとの動作 ------------------------------------------------------------
    def _set_drive(self, cmd: DriveCommand) -> None:
        self.drive = cmd
        if cmd.moving and cmd.params is not None:
            self.anim.gait.start(cmd.params, cmd.gamma0_deg)
        else:
            self.anim.gait.stop()

    def _on_enter(self, t: float, s: str, snake: SnakePose | None, person: np.ndarray | None) -> None:
        x = self.x
        self.expr.stop_tilting()
        self._set_drive(DriveCommand(False, reason=STATE_LABELS_JA[s]))
        if s != "COIL_REST" and self.anim.base.get("J1", 0.0) != self.poses.home()["J1"]:
            self.anim.play(Keyframe({k: v for k, v in self.poses.home().items() if k not in (HEAD_YAW, "J9")}, 1.5), t)
        if s == "SLEEP":
            self.anim.play(Keyframe({NECK: float(x["sleep_neck_deg"]), HEAD_YAW: 0.0}, 2.0), t)
        elif s == "PATROL":
            self._waypoint = None
            self.anim.play(Keyframe({NECK: self.poses.home()[NECK]}, 1.0), t)
        elif s in STILL_STATES and snake is not None and person is not None:
            neck = float(x["alert_neck_deg"] if s == "ALERT" else x["engage_neck_deg"])
            self.expr.look_at(t, self._yaw_to(snake, person), neck, force=True)
            if s != "ALERT":
                self.expr.start_tilting(t)
        elif s == "APPROACH" and snake is not None and person is not None:
            # 進める距離 = 「人の 400mm 手前まで」と「マット端まで」の短い方。その 60% で一度止まる
            stop = float(self.b["controller"]["stop_distance_mm"])
            travel = min(max(self.ctrl.head_distance(snake, person) - stop, 0.0), self.ctrl.room_toward(snake, person))
            self._approach_stage = 1
            self._approach_start = np.array([snake.x, snake.y])
            d = np.asarray(person, float) - self._approach_start
            self._approach_dir = d / max(float(np.linalg.norm(d)), 1e-9)
            self._stage_goal_dist = float(x["approach_first_ratio"]) * travel
        elif s == "PETTED":
            self.expr.petted(t)
        elif s == "COIL_REST":
            self.anim.play(coil_keyframe(self.poses), t)
        self.expr.set_eyes(s)

    def _on_tick(self, t: float, snake: SnakePose | None, person: np.ndarray | None, head_dist: float | None) -> None:
        s = self.fsm.state
        c = self.b["controller"]
        if s not in QUIET_STATES:
            self.expr.maybe_distract(t)
        if snake is None:
            self._set_drive(DriveCommand(False, reason="位置不明: 停止"))
            return
        if s == "PATROL":
            if self._waypoint is None or self.ctrl.arrived(snake, self._waypoint):
                m = float(c["mat_margin_mm"]) * 2
                self._waypoint = np.array([self.rng.uniform(m, self.ctrl.w - m), self.rng.uniform(m, self.ctrl.d - m)])
            # 安全: 1m 以内の速度制限は「気づいたか」に関係なく、追跡中の人に対して必ずかける
            self._set_drive(self.ctrl.drive_to(snake, self._waypoint, float(c["speed_patrol_mm_s"]), self._last_person_raw))
        elif s == "APPROACH" and person is not None:
            self._approach(t, snake, person, head_dist)
        elif s == "RETREAT" and person is not None:
            goal = self.ctrl.retreat_point(snake, person)
            self._set_drive(self.ctrl.drive_to(snake, goal, float(c["speed_retreat_mm_s"]), person))
            self.expr.look_at(t, self._yaw_to(snake, person))
        elif s in STILL_STATES and person is not None:
            self.expr.look_at(t, self._yaw_to(snake, person))
            self.expr.maybe_tilt(t)

    def _approach(self, t: float, snake: SnakePose, person: np.ndarray, head_dist: float | None) -> None:
        """f. 2段階の接近: 距離の 60% 進む → 0.8〜1.5 秒止まる → 残り。"""
        c = self.b["controller"]
        self.expr.look_at(t, self._yaw_to(snake, person))
        if t < self._pause_until:
            self._set_drive(DriveCommand(False, reason="接近: 一度止まって様子を見る"))
            return
        moved = float((np.array([snake.x, snake.y]) - self._approach_start) @ self._approach_dir)   # 人の方向への前進量
        if self._approach_stage == 1 and moved >= self._stage_goal_dist:
            self._approach_stage = 2
            self._pause_until = t + self.expr._u("approach_pause_s")
            self._events.append("接近: 60% で一時停止")
            self._set_drive(DriveCommand(False, reason="接近: 一度止まって様子を見る"))
            return
        cmd = self.ctrl.drive_to(snake, person, float(c["speed_approach_mm_s"]), person,
                                 stop_at_person=True, edge_is_goal=True)
        if cmd.blocked:
            self._at_limit, self._limit_person = True, np.asarray(person, float).copy()
        self._set_drive(cmd)
