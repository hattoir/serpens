"""シミュレータ一式: 世界・モックサーボ・モック頭部・仮想の人・知覚・行動を1つにまとめる。

STEP 6 のデモと STEP 7 の GUI（--sim）の両方がこれを使う。
制御周期ごとに step() を呼ぶと:
  サーボ実角度で世界を進める → マーカ位置から SnakePose → 人の位置から追跡 → 頭部のタッチ
  → サーボ温度 → Brain → アニメーター → サーボへ送信
"""
from __future__ import annotations

import copy
import random
from typing import Any

import numpy as np

from serpens.behavior.brain import Brain, BrainStatus, Percept
from serpens.hw.head_io import MockHeadIO
from serpens.hw.mock_bus import MockServoBus
from serpens.hw.state_poller import ServoStatePoller
from serpens.motion.animator import Animator
from serpens.perception.person_detector import PersonDetection, PersonTracker, TrackedPerson
from serpens.perception.snake_pose import SnakePose, SnakePoseTracker
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
    """実機なしで全部を動かす。"""

    def __init__(self, cfg: dict[str, Any], start: BodyPose | None = None, seed: int | None = None,
                 overrides: dict[str, dict[str, Any]] | None = None) -> None:
        cfg = copy.deepcopy(cfg)
        for section, values in (overrides or {}).items():
            cfg[section].update(values)
        self.cfg = cfg
        self.clock = ManualClock()
        self.ctrl_dt = 1.0 / float(cfg["behavior"]["tick_hz"])
        self.sub = max(int(round(self.ctrl_dt / float(cfg["sim"]["dt_s"]))), 1)
        self.world = World(cfg, start)
        self.bus = MockServoBus(cfg, clock=self.clock)
        self.bus.connect()
        for sid in self.bus.ids:
            self.bus.set_torque(sid, True)
        self.head = MockHeadIO(cfg, clock=self.clock)
        self.poller = ServoStatePoller(self.bus, cfg, self.clock)
        self.anim = Animator(cfg)
        self.brain = Brain(cfg, self.anim, self.bus, self.head, random.Random(seed))
        self.snake_tracker = SnakePoseTracker(cfg)
        self.person_tracker = PersonTracker(cfg)
        self.people: list[SimPerson] = []
        self.snake: SnakePose | None = None
        self.target: TrackedPerson | None = None
        self._serial = 0
        self._switches = 0
        self._ids = {j["name"]: int(j["servo_id"]) for j in cfg["joints"]}
        self.status: BrainStatus | None = None

    @property
    def t(self) -> float:
        return self.clock.t

    def touch(self, on: bool, where: str = "head") -> None:
        """頭部のタッチセンサを押す / 離す（シミュレーション）。"""
        setattr(self.head, "touch_head" if where == "head" else "touch_back", on)

    def step(self) -> BrainStatus:
        """制御周期1回ぶん進める。"""
        for _ in range(self.sub):
            self.clock.t += self.ctrl_dt / self.sub
            pos = self.bus.read_positions()
            self.world.step({n: pos[i] for n, i in self._ids.items()}, self.ctrl_dt / self.sub)
        t = self.clock.t
        self.poller.poll()
        j8 = self.poller.positions.get(self._ids["J8"], 0.0)
        self.snake = self.snake_tracker.update(t, self.world.marker_xy("neck"), self.world.marker_xy("tail"), j8)
        dets = [PersonDetection(BBOX_NONE, 1.0, np.array([p.x_mm, p.y_mm])) for p in self.people]
        before = self.target
        self.target = self.person_tracker.update(t, dets, None if self.snake is None else np.array([self.snake.x, self.snake.y]))
        if (before is None and self.target is not None) or self.person_tracker.switches != self._switches:
            self._serial += 1
            self._switches = self.person_tracker.switches
        self.head.poll()
        frame = self.head.latest
        touch = bool(frame and (frame.touch_head or frame.touch_back))
        person_xy = None if self.target is None else self.target.floor_mm
        loads = [abs(v.load) for v in self.poller.states.values()]
        self.status = self.brain.tick(t, Percept(self.snake, person_xy, self._serial, touch,
                                                 self.poller.max_temperature_c(), max(loads, default=0.0)))
        self.anim.send(self.bus, self.anim.update(t))
        return self.status
