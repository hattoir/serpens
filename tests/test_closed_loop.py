"""閉ループ（Stage K〜N）: World State → Behavior → RobotInterface → SimulatedSnake → 世界。

  - 仮想の来場者を置くと、既存の行動エンジンが気づいて向きを変え、近づく
  - World State の姿勢には**出どころ**が付いていて、真値を Vision と数えない
  - 追従中に異常を注入すると、**行動より安全が勝つ**

MuJoCo は要らない（この閉ループは KINEMATIC_SIM + 仮想 ESP32 + 仮想サーボ）。
"""
from __future__ import annotations

import pytest

from serpens.config import load_config
from serpens.link.protocol import State, StopReason
from serpens.link.snake import SimulatedSnake
from serpens.sim.session import ManualClock, SimSession
from serpens.sim.world import BodyPose
from serpens.world_state import PoseSource
from simulation.bridge import drive_people, world_state_from_session
from simulation.record import RunMeta, Recorder, config_hash, load, same_setup
from simulation.virtual_person import TRAJECTORIES, visitor

START = BodyPose(600.0, 600.0, -1.5708)        # 台の中央、来場者側（y<0）を向いて立つ


@pytest.fixture()
def cfg() -> dict:
    return load_config()


class Loop:
    """閉ループ1本。時計は1つ（セッション・機体・記録が同じ時刻を見る）。"""

    def __init__(self, cfg: dict, seed: int = 5) -> None:
        self.cfg = cfg
        self.clock = ManualClock()
        self.snake = SimulatedSnake(cfg, self.clock)
        self.session = SimSession(cfg, START, seed=seed, robot=self.snake, clock=self.clock)
        self.people: list = []
        self.rec = Recorder(RunMeta(source="KINEMATIC_SIM", config_hash=config_hash(cfg), seed=seed,
                                    gait={"preset": "forward"},
                                    notes=["仮想ESP32 + 仮想サーボ。Vision は未接続（真値）"]))
        self.states: list[str] = []
        self.session.request_start("テスト")

    def run_until_driving(self, timeout_s: float = 12.0) -> bool:
        """機体が実際に走り出すまで進める（観察中は止まっているのが正しいので待つ）。"""
        for _ in range(int(round(timeout_s / self.session.ctrl_dt))):
            self.run(self.session.ctrl_dt)
            if self.snake.device.driving:
                return True
        return False

    def run(self, seconds: float) -> None:
        for _ in range(int(round(seconds / self.session.ctrl_dt))):
            drive_people(self.session, self.people, self.clock.t)
            self.session.step()
            ws = world_state_from_session(self.session)
            st = self.session.status.state if self.session.status else ""
            if st and (not self.states or self.states[-1] != st):
                self.states.append(st)
                self.rec.event(self.clock.t, "state", st)
            self.rec.add(self.clock.t, robot=None if ws.robot is None else
                         [round(ws.robot.x_mm, 1), round(ws.robot.y_mm, 1)],
                         people=len(ws.people), safety=self.session.stop.state.value,
                         device=int(self.snake.device.state))

    @property
    def world(self):
        return world_state_from_session(self.session)


# ---- World State ----------------------------------------------------------------------
def test_world_state_marks_ground_truth_as_not_vision(cfg: dict) -> None:
    """**シミュレーションの真値を Vision の成果として数えない。**"""
    loop = Loop(cfg)
    loop.people = [visitor(cfg, "STATIONARY")]
    loop.run(1.0)
    ws = loop.world
    assert ws.robot is not None and ws.robot.source is PoseSource.GROUND_TRUTH_SIM
    assert ws.people and all(p.source is PoseSource.GROUND_TRUTH_SIM for p in ws.people)
    assert not ws.any_vision, "真値なのに Vision 由来と見なしている"
    assert PoseSource.GROUND_TRUTH_SIM.is_vision is False
    assert PoseSource.ARUCO.is_vision and PoseSource.PERSON_DETECTOR.is_vision


def test_world_state_reports_stale_observations(cfg: dict) -> None:
    loop = Loop(cfg)
    loop.run(0.5)
    ws = loop.world
    assert ws.stale(ws.t, 0.5) == []
    assert "robot" in ws.stale(ws.t + 10.0, 0.5)


# ---- 仮想の来場者 ----------------------------------------------------------------------
@pytest.mark.parametrize("kind", TRAJECTORIES)
def test_virtual_person_trajectories(cfg: dict, kind: str) -> None:
    """4種類の軌跡が作れて、位置が時間で動く（立ち止まりを除く）。"""
    p = visitor(cfg, kind)
    a, b = p.at(0.0), p.at(2.0)
    moved = abs(a[0] - b[0]) + abs(a[1] - b[1])
    assert (moved == 0.0) if kind == "STATIONARY" else (moved > 100.0)
    assert p.pose(1.0).source is PoseSource.GROUND_TRUTH_SIM


# ---- 展示シナリオ ----------------------------------------------------------------------
def test_exhibition_scenario_reacts_to_a_visitor(cfg: dict) -> None:
    """誰もいない → 来場者が近づく → 行動が変わる → 去ると巡回へ戻る。

    状態名は既存の行動エンジンのもの（PATROL / ALERT / OBSERVE / APPROACH / ENGAGE …）。
    **Behavior を作り直さずに、閉ループが成立することだけを確かめる。**
    """
    loop = Loop(cfg)
    loop.run(4.0)
    assert loop.states and loop.states[0] in ("PATROL", "SLEEP")
    loop.people = [visitor(cfg, "WALKING", speed_mm_s=400.0)]
    loop.run(14.0)
    assert len(loop.states) >= 2, f"来場者に反応しなかった: {loop.states}"
    assert loop.session.target is not None, "人を追いかける対象として掴んでいない"
    reacted = set(loop.states[1:])
    assert reacted & {"ALERT", "OBSERVE", "APPROACH", "ENGAGE", "RETREAT"}, loop.states
    loop.people = []                                    # 去る
    loop.run(12.0)
    assert loop.session.target is None or loop.session.status.state in ("PATROL", "SLEEP")


def test_safety_stops_reach_the_device_during_the_scenario(cfg: dict) -> None:
    """シナリオの途中で止めたら、機体まで届いて止まる。"""
    loop = Loop(cfg)
    loop.people = [visitor(cfg, "WALKING")]
    loop.run(6.0)
    loop.session.request_stop("テスト停止")
    loop.run(0.6)
    assert not loop.snake.device.driving
    assert loop.snake.device.state is State.DISARMED


# ---- 閉ループでの故障注入（Stage N） -------------------------------------------------------
def test_packet_loss_during_following_stops_the_machine(cfg: dict) -> None:
    """走っている最中に通信が全部落ちたら、**行動が走らせ続けようとしても機体が止まる。**"""
    loop = Loop(cfg)
    loop.people = [visitor(cfg, "WALKING")]
    assert loop.run_until_driving(), "機体が一度も走らなかった（シナリオを見直す）"
    loop.snake.faults.drop_ratio = 1.0
    loop.run(2.0)
    assert not loop.snake.device.driving
    assert loop.snake.device.state is State.FAULT_HOLD


def test_servo_fault_during_following_beats_behavior(cfg: dict) -> None:
    """追従中のサーボ異常は、行動より優先して止まる。"""
    loop = Loop(cfg)
    loop.people = [visitor(cfg, "WALKING")]
    assert loop.run_until_driving(), "機体が一度も走らなかった"
    loop.snake.device.inject_axis("J3", offline=True)
    loop.run(1.0)
    assert loop.snake.device.state is State.FAULT_HOLD
    assert loop.snake.device.stop_reason is StopReason.SERVO_FAULT
    loop.run(3.0)                                        # 行動は動き続けようとする
    assert not loop.snake.device.driving, "行動が安全を上書きした"


def test_reboot_during_following_does_not_auto_resume(cfg: dict) -> None:
    """追従中に機体が再起動しても、人の操作なしには走り出さない。"""
    loop = Loop(cfg)
    loop.people = [visitor(cfg, "WALKING")]
    assert loop.run_until_driving(), "機体が一度も走らなかった"
    loop.snake.reboot()
    loop.run(3.0)
    assert not loop.snake.device.driving and loop.snake.client.rebooted


def test_overheat_during_following_latches(cfg: dict) -> None:
    loop = Loop(cfg)
    loop.people = [visitor(cfg, "WALKING")]
    assert loop.run_until_driving(), "機体が一度も走らなかった"
    loop.snake.device.inject_axis("J5", temp_c=cfg["link"]["faults"]["temp_limit_c"] + 5)
    loop.run(1.0)
    assert loop.snake.device.state is State.EMERGENCY_LATCHED
    loop.run(3.0)
    assert loop.snake.device.state is State.EMERGENCY_LATCHED, "ラッチが外れた"


# ---- 記録 ------------------------------------------------------------------------------
def test_run_is_recorded_with_enough_to_reproduce(cfg: dict, tmp_path) -> None:
    """**記録だけで同じ run を作れる**（source / config hash / seed / 出来事）。"""
    loop = Loop(cfg, seed=11)
    loop.people = [visitor(cfg, "WALKING")]
    loop.run(5.0)
    path = loop.rec.save(tmp_path / "run.json")
    got = load(path)
    assert got.source == "KINEMATIC_SIM"
    assert got.meta["seed"] == 11 and got.meta["config_hash"] == config_hash(cfg)
    assert len(got.rows) > 100 and got.rows[0]["t"] < got.rows[-1]["t"]
    assert any(e["kind"] == "state" for e in got.events)
    again = Loop(cfg, seed=11)
    again.people = [visitor(cfg, "WALKING")]
    again.run(5.0)
    assert same_setup(got, load(again.rec.save(tmp_path / "run2.json")))
    assert loop.states == again.states, "同じ seed で結果が変わった"
