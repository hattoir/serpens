"""機体の中のサーボ群。**実機では STS3215 の TTL バス、いまは模擬。**

`MockServoBus` をそのまま使う（一次遅れ応答・速度上限・負荷・温度・電圧降下・トルク制限を持つ）。
そのおかげで、テレメトリが返すのは**指令角ではなく模擬の実測角**になり、
「指令したのに追従していない」「軸が応答しない」といった状態を機体が自分で見つけられる。

**この層の値はすべて SIMULATION 由来。** 実測と混ぜないよう、テレメトリに印を付けて返す。
"""
from __future__ import annotations

from typing import Any, Callable

from serpens.hw.mock_bus import MockServoBus
from serpens.hw.servo_bus import Goal
from serpens.link.messages import AxisTelemetry

UNKNOWN_CURRENT = None      # 電流センサが無い機体では測れない（0 を返して偽らない）


class VirtualServoBus:
    """機体から見たサーボ群。書いて、読んで、故障を注入できる。"""

    def __init__(self, cfg: dict[str, Any], clock: Callable[[], float]) -> None:
        self.cfg = cfg
        self.names = [j["name"] for j in cfg["joints"]]
        self.ids = {j["name"]: int(j["servo_id"]) for j in cfg["joints"]}
        self.speed = {j["name"]: float(j["max_speed_dps"]) for j in cfg["joints"]}
        self._accel = float(cfg["animator"]["default_accel_dps2"])
        self.bus = MockServoBus(cfg, clock=clock)
        self.bus.connect()
        self.bus.apply_torque_ceiling()             # 安全上限は機体側でも通す
        for sid in self.bus.ids:
            self.bus.set_torque(sid, True)          # 起動時は現在姿勢を保持
        self.read_once = False

    # ---- 出力 -----------------------------------------------------------------------
    def write(self, pose: dict[str, float], speed: dict[str, float] | None = None) -> None:
        """目標角を書く（9軸同期書き込みに相当）。"""
        sp = speed or self.speed
        self.bus.sync_set_goals({self.ids[n]: Goal(v, sp.get(n, self.speed[n]), self._accel)
                                 for n, v in pose.items() if n in self.ids})

    def set_torque(self, on: bool) -> None:
        for sid in self.bus.ids:
            self.bus.set_torque(sid, on)

    def set_torque_ratio(self, ratio: float) -> None:
        """安全上限に対する割合（脱力の演出）。上限そのものは超えられない。"""
        for sid in self.bus.ids:
            self.bus.set_torque_limit(sid, ratio)

    # ---- 読み出し -------------------------------------------------------------------
    def read(self) -> dict[str, AxisTelemetry]:
        """応答した軸だけ返す。**応答しない軸は欠けたまま返す**（0 で埋めない）。"""
        states = self.bus.sync_read_states()
        out: dict[str, AxisTelemetry] = {}
        for name, sid in self.ids.items():
            st = states.get(sid)
            if st is None:
                continue
            out[name] = AxisTelemetry(st.pos_deg, self.bus.velocity_of(sid), st.load,
                                      int(round(st.temp_c)), st.volt, self.bus.fault_of(sid),
                                      UNKNOWN_CURRENT)
        self.read_once = self.read_once or bool(out)
        return out

    def missing(self, seen: dict[str, AxisTelemetry]) -> list[str]:
        """応答しなかった軸。"""
        return [n for n in self.names if n not in seen]

    # ---- 故障注入（シミュレーション専用） ------------------------------------------------
    def inject(self, name: str, *, offline: bool | None = None, temp_c: float | None = None,
               load: float | None = None, fault: int | None = None) -> None:
        """1軸に異常を起こす。実機ではサーボ自身が返す値。"""
        sid = self.ids[name]
        if offline is not None:
            self.bus.set_offline(sid, offline)
        if temp_c is not None:
            self.bus._axes[sid].temp_c = float(temp_c)
        if load is not None:
            self.bus.set_external_load(sid, float(load))
        if fault is not None:
            self.bus.set_fault(sid, fault)
