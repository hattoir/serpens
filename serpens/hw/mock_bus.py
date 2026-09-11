"""実機なしで動くモックサーボバス。

- 位置: 目標角への一次遅れ。ただし指令速度・指令加速度で頭打ち
- 負荷: 角速度に比例する成分 + 目標との偏差（保持トルク）+ 外力
- 温度: 負荷に応じた平衡温度へ一次遅れで近づく（動かすと上がり、止まると環境温度へ戻る）
- 電圧: 負荷に比例して少し下がる

時刻は clock（既定は time.monotonic）から取り、呼ばれるたびに経過時間ぶん積分する。
テストでは偽の時計を渡して決定的に動かせる。
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Callable

from serpens.hw.servo_bus import Goal, ServoBus, ServoCommError, ServoState

Clock = Callable[[], float]


@dataclass
class _AxisSim:
    """1軸分の模擬状態。"""

    pos_deg: float
    vel_dps: float
    goal: Goal
    temp_c: float
    torque_on: bool = False
    torque_ratio: float = 1.0
    external_load: float = 0.0
    load: float = 0.0


class MockServoBus(ServoBus):
    """一次遅れで追従する模擬サーボ群。"""

    def __init__(self, cfg: dict[str, Any], clock: Clock | None = None) -> None:
        super().__init__(cfg)
        m = cfg["mock_servo"]
        s = cfg["servo"]
        self._m = m
        self._clock: Clock = clock or time.monotonic
        self._step_s = float(m["integration_step_s"])
        self._deg_per_step = 360.0 / float(s["steps_per_rev"])
        self._vmax_dps = float(s["speed_max_step_s"]) * self._deg_per_step
        init = float(m["initial_deg"])
        hold = Goal(init, self._vmax_dps, math.inf)
        self._axes: dict[int, _AxisSim] = {
            sid: _AxisSim(init, 0.0, hold, float(m["ambient_c"])) for sid in self.ids
        }
        self._connected = False
        self._last_t = self._clock()

    # ---- 接続 -----------------------------------------------------------------
    def connect(self) -> None:
        """接続（モックなので状態を立てるだけ）。"""
        self._connected = True
        self._last_t = self._clock()

    def disconnect(self) -> None:
        """全軸トルク OFF にして切断。"""
        self._update()
        for ax in self._axes.values():
            ax.torque_on = False
        self._connected = False

    def ping(self, servo_id: int) -> bool:
        """設定にある ID なら応答する。"""
        return self._connected and servo_id in self._axes

    # ---- 指令 -----------------------------------------------------------------
    def set_torque(self, servo_id: int, on: bool) -> None:
        """トルク ON/OFF。ON にした瞬間の目標は現在位置（跳ねないように）。"""
        ax = self._axis(servo_id)
        self._update()
        if on and not ax.torque_on:
            ax.goal = Goal(ax.pos_deg, ax.goal.speed_dps, ax.goal.accel_dps2)
        ax.torque_on = on

    def set_torque_limit(self, servo_id: int, ratio: float) -> None:
        """トルク上限（0〜1）。"""
        self._update()
        self._axis(servo_id).torque_ratio = min(max(ratio, 0.0), 1.0)

    def set_external_load(self, servo_id: int, load: float) -> None:
        """外力による負荷率を与える（シミュレータ・テスト用）。"""
        self._update()
        self._axis(servo_id).external_load = load

    def _set_goals(self, goals: dict[int, Goal]) -> None:
        self._update()
        for sid, g in goals.items():
            self._axis(sid).goal = g

    # ---- 読み出し -------------------------------------------------------------
    def _read_states(self, ids: list[int]) -> dict[int, ServoState]:
        self._update()
        out: dict[int, ServoState] = {}
        for sid in ids:
            ax = self._axis(sid)
            volt = float(self._m["voltage_nominal_v"]) - float(self._m["voltage_sag_v"]) * abs(ax.load)
            out[sid] = ServoState(ax.pos_deg, ax.load, volt, ax.temp_c)
        return out

    # ---- 内部 -----------------------------------------------------------------
    def _axis(self, servo_id: int) -> _AxisSim:
        if not self._connected:
            raise ServoCommError("MockServoBus: connect() 前に操作しました")
        if servo_id not in self._axes:
            raise ServoCommError(f"MockServoBus: ID {servo_id} は存在しません")
        return self._axes[servo_id]

    def _update(self) -> None:
        """前回からの経過時間ぶん、全軸を積分する。"""
        now = self._clock()
        remaining = now - self._last_t
        self._last_t = now
        while remaining > 0.0:
            dt = min(self._step_s, remaining)
            for ax in self._axes.values():
                self._integrate(ax, dt)
            remaining -= dt

    def _integrate(self, ax: _AxisSim, dt: float) -> None:
        """1軸を dt 秒進める。"""
        m = self._m
        tau = float(m["position_tau_s"])
        if ax.torque_on:
            err = ax.goal.deg - ax.pos_deg
            speed = min(max(ax.goal.speed_dps, 0.0), self._vmax_dps)
            v_des = max(-speed, min(speed, err / tau))
            dv_max = ax.goal.accel_dps2 * dt
            ax.vel_dps += max(-dv_max, min(dv_max, v_des - ax.vel_dps))
            ax.pos_deg += ax.vel_dps * dt
            raw = (float(m["load_per_speed"]) * ax.vel_dps / self._vmax_dps
                   + float(m["load_per_error_deg"]) * err + ax.external_load)
            ax.load = max(-ax.torque_ratio, min(ax.torque_ratio, raw))
            holding = float(m["holding_heat_c"])
        else:
            ax.vel_dps = 0.0
            ax.load = 0.0
            holding = 0.0
        t_eq = float(m["ambient_c"]) + float(m["heat_gain_c"]) * abs(ax.load) + holding
        ax.temp_c += (t_eq - ax.temp_c) * dt / float(m["heat_tau_s"])
