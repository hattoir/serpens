"""Phase 3: 行動 → 駆動リンク → 機体 → 世界 を1本で確かめる。

ここが繋がると、**アプリ経路でも「PC が死んだら機体が自分で止まる」**が本当になる。
直接経路（`DirectRobot`）ではサーボが最後の指令角を保持し続けるだけで、誰も止めない。
"""
from __future__ import annotations

import pytest

from serpens.config import load_config
from serpens.link.client import LinkClient
from serpens.link.device import SimulatedDevice
from serpens.link.protocol import State, StopReason
from serpens.link.robot import LinkRobot
from serpens.link.transport import LoopbackTransport
from serpens.sim.session import ManualClock, SimSession
from serpens.sim.world import BodyPose
from tests.helpers import steady_patrol

START = BodyPose(150.0, 600.0, 0.0)


@pytest.fixture()
def cfg() -> dict:
    return steady_patrol(load_config())     # 歩容が出ている最中の性質を見る（stop-and-go は別の試験）


class Rig:
    """セッション + 偽経路 + 模擬 ESP32。時計は1つだけ（全員が同じ時刻を見る）。"""

    def __init__(self, cfg: dict) -> None:
        self.clock = ManualClock()
        self.device = SimulatedDevice(cfg, now=0.0)
        self.tr = LoopbackTransport(self.device)
        self.client = LinkClient(self.tr, cfg, now=0.0)
        self.robot = LinkRobot(cfg, self.client, self.clock, pump=self.tr.pump)
        self.session = SimSession(cfg, START, seed=1, robot=self.robot, clock=self.clock)

    def run(self, seconds: float) -> None:
        for _ in range(int(round(seconds / self.session.ctrl_dt))):
            self.session.step()

    def run_device_only(self, seconds: float) -> None:
        """**PC が止まった状態**で機体だけ進める（セッションを呼ばない）。"""
        dt = self.session.ctrl_dt
        for _ in range(int(round(seconds / dt))):
            self.clock.t += dt
            self.tr.pump(self.clock.t)

    def body_spread(self) -> float:
        g = self.device.goals
        return max(g[n] for n in self.device.mo.body) - min(g[n] for n in self.device.mo.body)


@pytest.fixture()
def rig(cfg: dict) -> Rig:
    r = Rig(cfg)
    r.session.request_start("テスト")
    return r


def test_behavior_drives_the_device_and_the_world_moves(rig: Rig) -> None:
    """PC は歩容パラメータだけ送り、角度は機体が作り、世界が動く。"""
    x0, y0 = rig.session.world.pose.x, rig.session.world.pose.y
    rig.run(6.0)
    assert rig.device.state is State.DRIVING and rig.device.driving
    assert rig.body_spread() > 10.0, "機体側で進行波が立っていない"
    moved = ((rig.session.world.pose.x - x0) ** 2 + (rig.session.world.pose.y - y0) ** 2) ** 0.5
    assert moved > 50.0, f"世界が動いていない（{moved:.0f}mm）"
    assert rig.session.poller.source == "ESP32 link"
    assert len(rig.session.poller.fresh_states(rig.session.t)) == len(rig.session._ids)


def test_no_angle_streaming_for_the_body(rig: Rig) -> None:
    """胴体へ角度列を送っていない（送っているのはパラメータ）。"""
    rig.run(4.0)
    assert rig.client.drive is not None
    assert len(rig.client.drive.pack()) == 10, "DRIVE は 10 バイトのパラメータ"
    assert rig.device._drive is not None and rig.device.mo.phase != 0.0


def test_pc_death_stops_the_machine(rig: Rig) -> None:
    """**セッションが止まっても機体が自分で止まる。** 直接経路には無い性質。"""
    rig.run(4.0)
    assert rig.device.driving
    rig.run_device_only(1.5)                      # PC のプロセスが消えた
    assert not rig.device.driving
    assert rig.device.stop_reason in (StopReason.DRIVE_TTL, StopReason.HEARTBEAT_LOST)
    before = dict(rig.device.output)
    rig.run_device_only(1.0)
    assert max(abs(rig.device.output[k] - before[k]) for k in before) < 1e-9


def test_usb_unplug_stops_the_machine(rig: Rig) -> None:
    rig.run(4.0)
    rig.tr.unplug()
    rig.run(1.5)                                  # セッションは回り続けている
    assert not rig.device.driving
    assert not rig.robot.link_ok, "PC がリンク断に気付いていない"


def test_operator_stop_reaches_the_machine(rig: Rig) -> None:
    """通常停止が機体まで届き、機体が保持へ入る。"""
    rig.run(4.0)
    rig.session.request_stop("テスト停止")
    rig.run(0.5)
    assert rig.device.state is State.DISARMED and not rig.device.driving
    assert rig.device.stop_reason is StopReason.OPERATOR_STOP
    before = dict(rig.device.output)
    rig.run(1.0)
    assert max(abs(rig.device.output[k] - before[k]) for k in before) < 1e-9, "停止中に動いた"
    assert rig.device.torque_on, "通常停止で脱力してはいけない"


def test_emergency_latches_on_the_machine(rig: Rig) -> None:
    """緊急停止が機体でラッチし、PC 側の解除だけでは走行が戻らない。"""
    rig.run(4.0)
    rig.session.request_emergency("テスト緊急")
    rig.run(0.5)
    assert rig.device.state is State.EMERGENCY_LATCHED
    rig.session.clear_emergency("テスト")         # PC 側の解除 → 待機へ戻るだけ
    rig.run(0.5)
    assert not rig.device.driving
    rig.session.request_start("テスト")           # 人が開始し直して初めて走る
    rig.run(1.0)
    assert rig.device.driving


def test_device_reboot_does_not_auto_resume(rig: Rig) -> None:
    """機体が再起動しても、PC は勝手に ARM しない（完了条件 6 をアプリ経路で確かめる）。"""
    rig.run(4.0)
    assert rig.device.driving
    rig.tr.reboot_device(rig.clock.t)
    rig.run(3.0)
    assert not rig.device.driving, "再起動後に勝手に走り出した"
    assert rig.client.rebooted
    rig.session.request_start("テスト")           # 人の操作で初めて再開
    rig.run(1.0)
    assert rig.device.driving and not rig.client.rebooted


def test_relax_goes_through_the_link(rig: Rig) -> None:
    """脱力の演出がリンク越しに届く（トルク比は機体側の上限に対する割合）。"""
    rig.run(2.0)
    rig.robot.torque_sink.set_torque_limit(1, 0.6)
    rig.run(0.3)
    assert rig.device.torque_ratio == pytest.approx(0.6)
    rig.session.request_stop("テスト停止")
    rig.run(0.3)
    assert rig.device.torque_ratio == 1.0, "停止で脱力が解除されていない"


def test_session_has_no_fake_servo_bus(rig: Rig) -> None:
    """リンク経路では使わないモックのバスを置かない（値の出どころを偽らない）。"""
    assert rig.session.bus is None
    assert rig.session.robot is rig.robot


# ---- SimulatedSnake / RealSnake（上位から見た2つのヘビ） ---------------------------------
def test_simulated_snake_is_a_drop_in_robot(cfg: dict) -> None:
    """`SimulatedSnake` をそのままセッションへ渡せる（行動側は何も変わらない）。"""
    from serpens.link.snake import SimulatedSnake

    clock = ManualClock()
    snake = SimulatedSnake(cfg, clock)
    s = SimSession(cfg, START, seed=1, robot=snake, clock=clock)
    s.request_start("テスト")
    for _ in range(int(5.0 / s.ctrl_dt)):
        s.step()
    assert snake.device.state is State.DRIVING
    assert s.poller.source == "ESP32 link"
    assert snake.device_status() is not None and snake.device_status().simulated


def test_simulated_snake_exposes_fault_injection(cfg: dict) -> None:
    """故障注入が上位からも届く（GUI やテストから異常を起こせる）。"""
    from serpens.link.snake import SimulatedSnake

    clock = ManualClock()
    snake = SimulatedSnake(cfg, clock)
    s = SimSession(cfg, START, seed=1, robot=snake, clock=clock)
    s.request_start("テスト")
    for _ in range(int(4.0 / s.ctrl_dt)):
        s.step()
    assert snake.device.driving
    snake.faults.drop_ratio = 1.0                 # 経路を全部落とす
    for _ in range(int(2.0 / s.ctrl_dt)):
        s.step()
    assert not snake.device.driving, "通信が全部落ちても走り続けた"


def test_real_snake_needs_a_port(cfg: dict) -> None:
    """`RealSnake` は実ポートが要る。**実機が無いので接続は試せない**（未検証のまま）。"""
    from serpens.link.snake import RealSnake

    with pytest.raises(Exception):
        RealSnake(cfg, ManualClock(), "COM_DOES_NOT_EXIST")
