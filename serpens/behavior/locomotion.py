"""移動の段取り（巡回の stop-and-go・2段階の接近・退避）と、移動指令の安全な適用。

brain.py から切り出したもの。状態の選択は知らない。「どの状態で何をするか」は brain が決め、
ここは「巡回するとはどう動くか」「接近するとはどう動くか」だけを持つ。
数値は config の behavior.controller / behavior.expression。
"""
from __future__ import annotations

import math
import random
from typing import Any

import numpy as np

from serpens.behavior.controller import Controller, DriveCommand
from serpens.motion.animator import Animator
from serpens.perception.snake_pose import SnakePose


class Locomotion:
    """移動の段取り。drive が現在の移動指令。"""

    def __init__(self, cfg: dict[str, Any], animator: Animator, rng: random.Random) -> None:
        self.c = cfg["behavior"]["controller"]
        self.x = cfg["behavior"]["expression"]
        self.ctrl = Controller(cfg)
        self.anim, self.rng = animator, rng
        self.drive = DriveCommand(False, reason="待機")
        self.events: list[str] = []
        # 巡回
        self.waypoint: np.ndarray | None = None
        self._patrol_dir = 1.0
        self._patrol_flip_t = 0.0
        self.patrol_paused = False           # 巡回の stop-and-go: いま立ち止まっているか
        self._patrol_phase_until = -1.0
        # 接近
        self._approach_stage = 0
        self._stage_goal_dist = 0.0
        self._approach_start = np.zeros(2)
        self._approach_dir = np.array([1.0, 0.0])
        self._pause_until = -1.0
        self.at_limit = False                # マット端まで来て、これ以上人に近づけない
        self.limit_person: np.ndarray | None = None

    def _u(self, key: str) -> float:
        lo, hi = self.c[key]
        return self.rng.uniform(float(lo), float(hi))

    # ---- 移動指令の適用（安全はここで常時かける） --------------------------------------
    def set_drive(self, cmd: DriveCommand, snake: SnakePose | None, person: np.ndarray | None) -> None:
        """移動指令を適用する。人まで stop_distance_mm ならどの状態でも前進しない。

        歩容は止めても振幅が消えるまで進む（実測で 50mm/s・1秒のブレンド中に約80mm）ので、
        安全の停止だけは即座に振幅を 0 にする。
        """
        emergency = False
        if cmd.moving and person is not None and snake is not None:
            stop = float(self.c["stop_distance_mm"])
            if self.ctrl.head_distance(snake, person) <= stop:
                cmd = DriveCommand(False, reason=f"安全: 人まで {stop:.0f}mm（どの状態でも前進しない）", blocked=True)
                emergency = True
        self.drive = cmd
        if cmd.moving and cmd.params is not None:
            self.anim.gait.start(cmd.params, cmd.gamma0_deg)
        else:
            self.anim.gait.stop(immediate=emergency)

    def stop(self, reason: str) -> None:
        self.set_drive(DriveCommand(False, reason=reason), None, None)

    # ---- 巡回 ---------------------------------------------------------------------
    def enter_patrol(self, t: float) -> None:
        self.waypoint = None
        self._patrol_flip_t = t + self._u("patrol_flip_s")
        self.patrol_paused = False
        self._patrol_phase_until = t + self._u("patrol_move_s")

    def patrol(self, t: float, snake: SnakePose, person: np.ndarray | None) -> None:
        """stop-and-go の巡回。静止中は歩容を止める（呼吸は animator 側で続く）。"""
        if self._pausing(t):
            self.set_drive(DriveCommand(False, reason="巡回: 立ち止まって様子をうかがう"), snake, person)
            return
        self.waypoint = self._patrol_target(t, snake)
        # 安全: 1m 以内の速度制限は「気づいたか」に関係なく、追跡中の人に対して必ずかける
        self.set_drive(self.ctrl.drive_to(t, snake, self.waypoint, float(self.c["speed_patrol_mm_s"]), person),
                       snake, person)

    def _pausing(self, t: float) -> bool:
        """動く → 止まる → 動く を乱数の長さで繰り返す。マット端から後退で戻っている最中は止まらない。"""
        if self._patrol_phase_until < 0.0:               # 起動直後（enter_patrol を通らない）は動くところから
            self._patrol_phase_until = t + self._u("patrol_move_s")
        if t >= self._patrol_phase_until and not (not self.patrol_paused and self.ctrl.phase == "back"):
            pause_s = 0.0 if self.patrol_paused else self._u("patrol_pause_s")
            self.patrol_paused = pause_s > 0.0           # 静止 0 秒の設定なら歩き続ける（試験・比較用）
            self._patrol_phase_until = t + (pause_s if self.patrol_paused else self._u("patrol_move_s"))
        return self.patrol_paused

    def _patrol_target(self, t: float, snake: SnakePose) -> np.ndarray:
        """巡回の目標点: 中央の円の上を、いまの位置より patrol_lead_deg 先に置く（キャロット追従）。

        端に寄らないので、フェンスからの復帰がほとんど起きない。向きは patrol_flip_s ごとに反転させる。
        """
        center = self.ctrl.center()
        if t >= self._patrol_flip_t:
            self._patrol_dir *= -1.0
            self._patrol_flip_t = t + self._u("patrol_flip_s")
        d = np.array([snake.x, snake.y]) - center
        phi = math.atan2(d[1], d[0]) + self._patrol_dir * math.radians(float(self.c["patrol_lead_deg"]))
        return center + float(self.c["patrol_radius_mm"]) * np.array([math.cos(phi), math.sin(phi)])

    # ---- 接近・退避 ------------------------------------------------------------------
    def enter_approach(self, snake: SnakePose, person: np.ndarray) -> None:
        """進める距離 = 「人の stop_distance 手前まで」と「マット端まで」の短い方。その一部で一度止まる。"""
        stop = float(self.c["stop_distance_mm"])
        travel = min(max(self.ctrl.head_distance(snake, person) - stop, 0.0), self.ctrl.room_toward(snake, person))
        self._approach_stage = 1
        self._approach_start = np.array([snake.x, snake.y])
        d = np.asarray(person, float) - self._approach_start
        self._approach_dir = d / max(float(np.linalg.norm(d)), 1e-9)
        self._stage_goal_dist = float(self.x["approach_first_ratio"]) * travel

    def approach(self, t: float, snake: SnakePose, person: np.ndarray) -> None:
        """2段階の接近（忍び寄りの波形）: 距離の一部を進む → 少し止まる → 残り。"""
        if t < self._pause_until:
            self.set_drive(DriveCommand(False, reason="接近: 一度止まって様子を見る"), snake, person)
            return
        moved = float((np.array([snake.x, snake.y]) - self._approach_start) @ self._approach_dir)
        if self._approach_stage == 1 and moved >= self._stage_goal_dist:
            self._approach_stage = 2
            lo, hi = self.x["approach_pause_s"]
            self._pause_until = t + self.rng.uniform(float(lo), float(hi))
            self.events.append("接近: 途中で一時停止")
            self.set_drive(DriveCommand(False, reason="接近: 一度止まって様子を見る"), snake, person)
            return
        cmd = self.ctrl.drive_to(t, snake, person, float(self.c["speed_approach_mm_s"]), person,
                                 stop_at_person=True, edge_is_goal=True, gait="stalk")
        if cmd.blocked:
            self.at_limit, self.limit_person = True, np.asarray(person, float).copy()
        self.set_drive(cmd, snake, person)

    def retreat(self, t: float, snake: SnakePose, person: np.ndarray) -> None:
        goal = self.ctrl.retreat_point(snake, person)
        self.set_drive(self.ctrl.drive_to(t, snake, goal, float(self.c["speed_retreat_mm_s"]), person), snake, person)

    def release_limit(self, person: np.ndarray | None, gate_mm: float) -> None:
        """追跡していた人が離れたら「マット端で近づけない」を解く。"""
        if self.at_limit and (person is None or self.limit_person is None
                              or np.linalg.norm(np.asarray(person) - self.limit_person) > gate_mm):
            self.at_limit = False
