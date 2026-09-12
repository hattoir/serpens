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
from dataclasses import dataclass, field
from typing import Any, Callable

from serpens.hw import registers as reg
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
    registers: dict[int, int] = field(default_factory=dict)
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
        self._center = int(s["center_step"])
        self._load_sign_bit = s.get("load_sign_bit")
        self._load_unit = s["load_unit"]
        self._vmax_dps = float(s["speed_max_step_s"]) * self._deg_per_step
        init = float(m["initial_deg"])
        hold = Goal(init, self._vmax_dps, math.inf)
        self._axes: dict[int, _AxisSim] = {
            sid: _AxisSim(init, 0.0, hold, float(m["ambient_c"]), registers=self._default_registers(sid))
            for sid in self.ids
        }
        self._connected = False
        self._last_t = self._clock()

    # ---- 接続 -----------------------------------------------------------------
    def connect(self) -> None:
        """接続（モックなので状態を立てるだけ）。"""
        self._connected = True
        self._last_t = self._clock()

    def disconnect(self, torque_off: bool = True) -> None:
        """切断する（torque_off=False ならトルクは入れたまま）。"""
        self._update()
        if torque_off:
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

    # ---- レジスタ（実機セットアップのツールを、実機なしで試せるように） ----------------
    @staticmethod
    def _default_registers(servo_id: int) -> dict[int, int]:
        """資料どおりの初期値。**最高入力電圧は 8.0V のまま**なので、12V の罠も再現できる。"""
        d = {r.addr: int(r.default) for r in reg.STARTUP_CHECK if r.default is not None}
        d[reg.ID.addr] = servo_id
        d[reg.BAUD_RATE.addr] = int(reg.BAUD_RATE.default or 0)
        d[reg.LOCK_ADDR] = 1
        d[reg.STATUS.addr] = 0
        return d

    def read_register(self, servo_id: int, addr: int, size: int) -> int | None:
        ax = self._axis(servo_id)
        self._update()
        live = {reg.PRESENT_VOLTAGE.addr: int(round((float(self._m["voltage_nominal_v"])
                                                     - float(self._m["voltage_sag_v"]) * abs(ax.load)) * 10)),
                reg.PRESENT_TEMPERATURE.addr: int(round(ax.temp_c)),
                reg.PRESENT_LOAD.addr: self._encode_load(ax.load),
                reg.PRESENT_POSITION.addr: int(round(ax.pos_deg / self._deg_per_step)) + self._center,
                reg.TORQUE_ENABLE.addr: int(ax.torque_on)}
        if addr in live:
            return live[addr]
        return ax.registers.get(addr, 0)

    def _encode_load(self, load: float) -> int:
        """負荷を「下位10bit = 大きさ、bit10 = 方向」の仮説どおりに符号化する（符号ビットの検証用）。"""
        bit = self._load_sign_bit
        mag = min(int(round(abs(load) / float(self._load_unit))), (1 << bit) - 1) if bit else 0
        if bit is None:
            return min(int(round(abs(load) / float(self._load_unit))), 0xFFFF)
        return mag | ((1 << bit) if load < 0 else 0)

    def write_register(self, servo_id: int, addr: int, size: int, value: int, eeprom: bool = False) -> bool:
        """レジスタに書く。

        eeprom=True は実機と同じ手順（トルクOFF → ロック解除 → 書く → ロック）を模す。
        EEPROM 領域へ eeprom=False で書こうとすると、ロックされていて保存されない（実機と同じ失敗）。
        """
        ax = self._axis(servo_id)
        if eeprom:
            self.set_torque(servo_id, False)
            ax.registers[reg.LOCK_ADDR] = 0
        elif addr in reg.EEPROM_ADDRS and ax.registers.get(reg.LOCK_ADDR, 1) == 1:
            return False
        ax.registers[addr] = int(value)
        if eeprom:
            ax.registers[reg.LOCK_ADDR] = 1
        if addr == reg.TORQUE_ENABLE.addr:
            self.set_torque(servo_id, bool(value))
        return True

    def set_servo_id(self, old_id: int, new_id: int) -> bool:
        if new_id in self._axes:
            return False
        ax = self._axes.pop(old_id)
        ax.registers[reg.ID.addr] = new_id
        self._axes[new_id] = ax
        self.joints[new_id] = self.joints.pop(old_id)
        return True

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
