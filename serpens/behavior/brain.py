"""行動の中枢: 知覚 → 安全 → 刺激 → 内部状態 → 効用 → 状態機械 → 各状態の動作。毎制御周期 tick() を呼ぶ。

割り込みの優先度: safety > PETTED > ALERT > 効用で選択
  safety の3つ（behavior.safety）:
    過熱     … サーボ最高温度 > overheat_c で COIL_REST_HEAT（最小60秒・人が来ても抜けない）。
                overheat_resume_c 以下に下がるまで続ける
    掴まれた … どれかの軸の負荷率 > grab_load_ratio が grab_hold_s 続いたら PETTED（脱力）
    マット端 … 仮想フェンス（controller）に入ったら、状態に関係なく前進をやめて中央へ向き直る
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from serpens.behavior.controller import DriveCommand
from serpens.behavior.grammar import Grammar, GrammarCtx
from serpens.behavior.primitives import Primitives
from serpens.behavior.fsm import StateMachine, Transition
from serpens.behavior.internal_state import InternalState, Stimuli
from serpens.behavior.locomotion import Locomotion
from serpens.behavior.utility import COIL_STATES, STATE_LABELS_JA, Context, UtilityModel, thought_line
from serpens.hw.head_io import HeadIO
from serpens.hw.servo_bus import ServoBus
from serpens.motion.animator import Animator, Keyframe, rest_keyframe
from serpens.motion.kinematics import Chain, forward, point_at_body_x
from serpens.motion.poses import HEAD_YAW, NECK, Poses
from serpens.perception.snake_pose import SnakePose, wrap_pi

STILL_STATES = ("ALERT", "OBSERVE", "ENGAGE")            # 止まって人を見る状態


@dataclass
class Percept:
    """1周期ぶんの知覚。target_serial は追跡対象が変わる（新しく見つける・切り替える）たびに増える。"""

    snake: SnakePose | None
    person_xy: np.ndarray | None
    target_serial: int
    touch: bool
    max_temp_c: float | None
    max_load: float = 0.0          # 全軸の負荷率の最大（掴まれ検出に使う）


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
    safety: str = ""               # 効いている安全割り込み（空なら無し）
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
        self.loco = Locomotion(cfg, animator, self.rng)     # 移動の段取り（巡回・接近・退避）
        self.ctrl = self.loco.ctrl
        self.expr = Primitives(cfg, animator, self.poses, self.rng, bus, head)     # 語彙
        self.grammar = Grammar(cfg, self.expr, self.rng, self._grammar_ctx)       # 文法（config）
        self.max_range = float(cfg["person"]["max_range_mm"])
        self.safety_cfg = b["safety"]
        self._grab_since: float | None = None
        self.safety = ""
        self._serial = 0
        self.noticed = False
        self._novelty = 0.0
        self._touch_until = -1.0
        self._last_person: np.ndarray | None = None
        self._approach_speed = 0.0
        self._rush_rearm_t = -1.0
        self._last_snake: SnakePose | None = None
        self._last_person_raw: np.ndarray | None = None
        self._next_eyes_t = 0.0
        self._last_t: float | None = None
        self._events: list[str] = []
        self.status: BrainStatus | None = None

    # ---- 1周期 ------------------------------------------------------------------
    def tick(self, t: float, p: Percept) -> BrainStatus:
        dt = 0.0 if self._last_t is None else t - self._last_t
        self._last_t = t
        self._events = []
        self._perceive(t, dt, p)
        self.expr.person_tracked = p.person_xy is not None      # 追跡中は威嚇に見える姿勢を出さない（guard）
        self._last_snake, self._last_person_raw = p.snake, p.person_xy
        person = p.person_xy if self.noticed else None
        self.loco.release_limit(person, float(self.cfg["person"]["association_gate_mm"]))
        head_dist = None if (person is None or p.snake is None) else self.ctrl.head_distance(p.snake, person)
        stim = self._stimuli(p, person, head_dist, dt)
        self.internal.update(dt, stim, p.max_temp_c, moving=self.anim.gait.active)
        forced = self._safety(t, p)
        ev = self.utility.evaluate(self.internal, Context(person is not None, head_dist, stim.touch, self._novelty,
                                                          self.loco.at_limit), t)
        tr = forced or (None if self._hold_for_heat(p) else self.fsm.step(t, ev.noisy))
        if tr is not None:
            self._events.append(f"{tr.src}→{tr.dst}（{tr.reason}）")
            self._on_enter(t, tr.dst, p.snake, person)
        self._on_tick(t, p.snake, person, head_dist)
        self._events += self.loco.events
        self.loco.events = []
        self.expr.update(t)
        if t >= self._next_eyes_t:
            self.expr.set_eyes(self.fsm.state)
            self._next_eyes_t = t + float(self.x["eyes_refresh_s"])
        s = self.fsm.state
        self.status = BrainStatus(t, s, STATE_LABELS_JA[s], thought_line(ev, s, self.fsm.time_to_next(t)),
                                  self.fsm.time_to_next(t), ev.noisy, self.internal.snapshot(), self.internal.heat_c,
                                  self.loco.drive.reason, self.noticed, self.safety, self._events)
        return self.status

    def _perceive(self, t: float, dt: float, p: Percept) -> None:
        """b. 新しい人を見つけたら、ランダムな遅延のあとで反応する（それまでは気づいていない）。"""
        if p.person_xy is None:
            self.noticed = False
            self.expr.cancel("react")
            self.expr.cancel("grammar:on_notice")
        elif p.target_serial != self._serial:
            self.noticed = False
            self.expr.cancel("react")
            self.grammar.event(t, "notice")                          # 一次反応（目・ピクッ）は文法が撃つ
            self.expr.schedule(t + self.expr._u("reaction_delay_s"), "react", self._react)
        self._serial = p.target_serial
        if p.touch:
            self._touch_until = t + float(self.st["touch_hold_s"])
        self._novelty *= math.exp(-dt / float(self.st["novelty_decay_s"])) if dt > 0 else 1.0

    def _safety(self, t: float, p: Percept) -> Transition | None:
        """安全割り込み。必要なら状態を強制的に切り替え、その Transition を返す。"""
        sc = self.safety_cfg
        self.safety = ""
        if p.max_temp_c is not None and p.max_temp_c > float(sc["overheat_c"]) and self.fsm.state != "COIL_REST_HEAT":
            self.safety = f"過熱 {p.max_temp_c:.1f}℃"
            return self._forced(t, "COIL_REST_HEAT", f"安全: {self.safety}")
        if abs(p.max_load) >= float(sc["grab_load_ratio"]):
            self._grab_since = t if self._grab_since is None else self._grab_since
            if t - self._grab_since >= float(sc["grab_hold_s"]) and self.fsm.state != "PETTED":
                self.safety = f"掴まれた（負荷 {abs(p.max_load):.2f}）"
                return self._forced(t, "PETTED", f"安全: {self.safety}")
        else:
            self._grab_since = None
        if self.fsm.state == "COIL_REST_HEAT":
            self.safety = "過熱から回復中" + ("" if p.max_temp_c is None else f"（{p.max_temp_c:.1f}℃）")
        return None

    def _forced(self, t: float, state: str, reason: str) -> Transition | None:
        """安全割り込みによる強制遷移（割り込まれない状態からも抜ける）。"""
        tr = self.fsm.force(t, state, reason, safety=True)
        if tr is not None:
            self._events.append(f"{tr.src}→{tr.dst}（{tr.reason}）")
            self._on_enter(t, tr.dst, self._last_snake, self._last_person_raw)
        return tr

    def _hold_for_heat(self, p: Percept) -> bool:
        """過熱の強制休憩中は、温度が下がるまで効用では抜けない。"""
        if self.fsm.state != "COIL_REST_HEAT":
            return False
        return p.max_temp_c is None or p.max_temp_c > float(self.safety_cfg["overheat_resume_c"])

    def _grammar_ctx(self) -> GrammarCtx:
        """文法が語彙を撃つ瞬間に見る外界（人の方向は撃つ時点の値）。"""
        person, snake = self._last_person_raw, self._last_snake
        direction = 0.0 if person is None or snake is None else self._yaw_to(snake, person)
        return GrammarCtx(self.fsm.state, self._novelty, direction, person is not None, self.loco.patrol_paused)

    def _react(self, t: float) -> None:
        """c. 反応の始まり: 全停止（驚き。呼吸は続く）→ 警戒。PETTED 中は割り込まない。"""
        self.noticed = True
        self._novelty = 1.0
        self.expr.freeze(t, float(self.x["surprise_freeze_s"]), keep_breath=bool(self.x["surprise_keep_breath"]))
        self._events.append("反応開始（驚き）")
        if self.fsm.state != "PETTED":
            # safety=False: 過熱の強制休憩（COIL_REST_HEAT）からは抜けない
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
        if self._approach_speed > float(self.st["rush_mm_s"]) and self._last_t is not None                 and self._last_t >= self._rush_rearm_t:
            self.internal.kick("stress", float(self.st["rush_stress_kick"]))     # 駆け込まれた（驚き）
            self._rush_rearm_t = self._last_t + float(self.st["rush_rearm_s"])
            self._events.append(f"駆け込み {self._approach_speed:.0f}mm/s → Stress 上昇")
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
    def _on_enter(self, t: float, s: str, snake: SnakePose | None, person: np.ndarray | None) -> None:
        x = self.x
        self.grammar.enter(t, s)
        self.loco.stop(STATE_LABELS_JA[s])
        if s not in COIL_STATES and self.anim.base.get("J1", 0.0) != self.poses.home()["J1"]:
            home = {k: v for k, v in self.poses.home().items() if k not in (HEAD_YAW, "J9")}
            self.expr.play(self.expr.with_settle(Keyframe(home, 1.5)), t)
        if s == "SLEEP":
            self.expr.play(Keyframe({NECK: float(x["sleep_neck_deg"]), HEAD_YAW: 0.0}, 2.0), t)
        elif s == "PATROL":
            self.loco.enter_patrol(t)
            self.expr.play(Keyframe({NECK: self.poses.home()[NECK]}, 1.0), t)
        elif s in STILL_STATES and snake is not None and person is not None:
            neck = float(x["alert_neck_deg"] if s == "ALERT" else x["engage_neck_deg"])
            self.expr.look_at(t, self._yaw_to(snake, person), neck, force=True)
        elif s == "APPROACH" and snake is not None and person is not None:
            self.loco.enter_approach(snake, person)
        elif s in COIL_STATES:
            self.expr.play(rest_keyframe(self.poses), t)
        self.expr.set_eyes(s)

    def _on_tick(self, t: float, snake: SnakePose | None, person: np.ndarray | None, head_dist: float | None) -> None:
        s = self.fsm.state
        self.grammar.tick(t)                                # 状態の間ずっと撃つ語彙（config の while）
        if snake is None:
            self.loco.stop("位置不明: 停止")
            return
        if s == "PATROL":
            self.loco.patrol(t, snake, self._last_person_raw)
        elif s == "APPROACH" and person is not None:
            self.expr.look_at(t, self._yaw_to(snake, person))
            self.loco.approach(t, snake, person)
        elif s == "RETREAT" and person is not None:
            self.loco.retreat(t, snake, person)
            self.expr.look_at(t, self._yaw_to(snake, person))
        elif s in STILL_STATES and person is not None:
            self.expr.look_at(t, self._yaw_to(snake, person))
