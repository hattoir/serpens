"""MockServoBus のテスト。"""
from __future__ import annotations

import math

import pytest

from serpens.config import load_config
from serpens.hw.mock_bus import MockServoBus
from serpens.hw.servo_bus import Goal, ServoCommError, make_bus
from tests.helpers import FakeClock

FAST_DPS = 200.0
FAST_ACC = 2000.0


@pytest.fixture()
def cfg() -> dict:
    return load_config()


@pytest.fixture()
def rig(cfg: dict) -> tuple[MockServoBus, FakeClock]:
    clock = FakeClock()
    bus = MockServoBus(cfg, clock=clock)
    bus.connect()
    for sid in bus.ids:
        bus.set_torque(sid, True)
    return bus, clock


def test_converges_to_goal_all_axes(rig: tuple[MockServoBus, FakeClock]) -> None:
    """9軸それぞれ異なる目標角に収束する。"""
    bus, clock = rig
    targets = {sid: 5.0 * i for i, sid in enumerate(bus.ids, start=1)}
    targets = {sid: bus.clamp_deg(sid, d) for sid, d in targets.items()}
    bus.sync_set_goals({sid: Goal(d, FAST_DPS, FAST_ACC) for sid, d in targets.items()})
    clock.advance(0.05)
    mid = bus.sync_read_states()
    assert any(abs(mid[s].pos_deg - targets[s]) > 1.0 for s in bus.ids), "まだ途中のはず"
    clock.advance(2.0)
    final = bus.sync_read_states()
    for sid, d in targets.items():
        assert final[sid].pos_deg == pytest.approx(d, abs=0.05)


def test_speed_limit_is_respected(rig: tuple[MockServoBus, FakeClock]) -> None:
    """指令速度より速く動かない。"""
    bus, clock = rig
    bus.set_goal(1, 80.0, 20.0, math.inf)
    clock.advance(1.0)
    assert bus.read_state(1).pos_deg == pytest.approx(20.0, abs=0.5)


def test_software_limits_clamp(rig: tuple[MockServoBus, FakeClock], cfg: dict) -> None:
    """config の min/max を超える指令はクランプされる（J7 は下限 0°、J9 は ±30°）。"""
    bus, clock = rig
    j = {d["name"]: d for d in cfg["joints"]}
    bus.set_goal(7, -45.0, FAST_DPS, FAST_ACC)
    bus.set_goal(9, 80.0, FAST_DPS, FAST_ACC)
    clock.advance(2.0)
    assert bus.read_state(7).pos_deg == pytest.approx(j["J7"]["min_deg"], abs=0.05)
    assert bus.read_state(9).pos_deg == pytest.approx(j["J9"]["max_deg"], abs=0.05)


def _fast_heat_rig(cfg: dict, tau: float) -> tuple[MockServoBus, FakeClock]:
    """温度時定数だけ短くしたモック（テストを速くするため）。"""
    c = {**cfg, "mock_servo": {**cfg["mock_servo"], "heat_tau_s": tau}}
    clock = FakeClock()
    bus = MockServoBus(c, clock=clock)
    bus.connect()
    for sid in bus.ids:
        bus.set_torque(sid, True)
    return bus, clock


def test_temperature_rises_when_moving_and_recovers(cfg: dict) -> None:
    """動かし続けると温度が上がり、止めると環境温度へ戻る。"""
    tau = 20.0
    bus, clock = _fast_heat_rig(cfg, tau)
    ambient = cfg["mock_servo"]["ambient_c"]
    t0 = bus.read_state(1).temp_c
    # 往復運動を tau の 2 倍の時間続ける
    swing, half_period = 40.0, 0.5
    steps = int(2 * tau / half_period)
    for k in range(steps):
        bus.set_goal(1, swing if k % 2 == 0 else -swing, FAST_DPS, FAST_ACC)
        clock.advance(half_period)
    hot = bus.read_state(1).temp_c
    assert hot > t0 + 3.0
    # 他の軸（静止保持）より熱い
    assert hot > bus.read_state(2).temp_c
    # 止めてトルクを切ると冷える
    bus.set_torque(1, False)
    clock.advance(5 * tau)
    assert bus.read_state(1).temp_c == pytest.approx(ambient, abs=0.5)


def test_heat_time_constant_is_configurable(cfg: dict) -> None:
    """heat_tau_s を短くすると同じ時間でより熱くなる（Energy 演出テスト用）。"""
    temps = []
    for tau in (300.0, 10.0):
        bus, clock = _fast_heat_rig(cfg, tau)
        bus.set_external_load(1, 0.8)
        clock.advance(20.0)
        temps.append(bus.read_state(1).temp_c)
    assert temps[1] > temps[0] + 5.0


def test_load_while_moving_and_external(rig: tuple[MockServoBus, FakeClock]) -> None:
    """動いている間は負荷が出て、止まれば 0 付近。外力は負荷に乗る。"""
    bus, clock = rig
    bus.set_goal(1, 60.0, FAST_DPS, FAST_ACC)
    clock.advance(0.1)
    assert abs(bus.read_state(1).load) > 0.05
    clock.advance(3.0)
    assert abs(bus.read_state(1).load) < 0.01
    bus.set_external_load(1, 0.4)
    clock.advance(0.01)
    assert bus.read_state(1).load == pytest.approx(0.4, abs=0.02)


def test_torque_limit_is_relative_to_safety_ceiling(rig: tuple[MockServoBus, FakeClock],
                                                    cfg: dict) -> None:
    """トルク比は**安全上限（safety_limits.torque.software_torque_limit_ratio）に対する割合**。

    脱力演出でも、ここを通して全力（ストールトルク）へは戻せない。
    """
    bus, clock = rig
    ceiling = float(cfg["safety_limits"]["torque"]["software_torque_limit_ratio"])
    bus.set_torque_limit(1, 0.6)
    bus.set_external_load(1, 0.9)
    clock.advance(0.01)
    assert bus.read_state(1).load == pytest.approx(0.6 * ceiling)
    bus.set_torque_limit(1, 1.0)                       # 「全開」でも上限まで
    clock.advance(0.01)
    assert bus.read_state(1).load == pytest.approx(ceiling)
    bus.set_torque_limit(1, 5.0)                       # 上限外の要求も上限で頭打ち
    clock.advance(0.01)
    assert bus.read_state(1).load == pytest.approx(ceiling)


def test_torque_off_does_not_move(rig: tuple[MockServoBus, FakeClock]) -> None:
    bus, clock = rig
    bus.set_torque(1, False)
    bus.set_goal(1, 45.0, FAST_DPS, FAST_ACC)
    clock.advance(1.0)
    assert bus.read_state(1).pos_deg == pytest.approx(0.0)


def test_state_tuple_and_voltage(rig: tuple[MockServoBus, FakeClock], cfg: dict) -> None:
    """read_state は (pos, load, volt, temp) として展開でき、電圧は公称値付近。"""
    bus, _ = rig
    pos, load, volt, temp = bus.read_state(1)
    assert volt == pytest.approx(cfg["mock_servo"]["voltage_nominal_v"], abs=0.1)
    assert temp == pytest.approx(cfg["mock_servo"]["ambient_c"], abs=0.1)


def test_errors_before_connect_and_unknown_id(cfg: dict) -> None:
    bus = MockServoBus(cfg, clock=FakeClock())
    with pytest.raises(ServoCommError):
        bus.read_state(1)
    bus.connect()
    assert bus.ping(1) and not bus.ping(42)
    with pytest.raises(ServoCommError):
        bus.set_torque(42, True)


def test_make_bus(cfg: dict) -> None:
    assert isinstance(make_bus("mock", cfg), MockServoBus)
    with pytest.raises(ValueError):
        make_bus("feetech", cfg)          # --port が無い
    with pytest.raises(ValueError):
        make_bus("dynamixel", cfg)
