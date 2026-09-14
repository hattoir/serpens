"""駆動リンク越しの出力先。**歩容は機体（ESP32）が作る。**

`DirectRobot` との違いは、送るものが「9軸の角度」ではなく「歩容のパラメータ」であること。
そのおかげで PC が落ちても USB が抜けても、機体が TTL と heartbeat で自分から止まる。

  行動 → `MotionCommand` → DRIVE / BODY / HEAD → 機体が角度を作る → サーボ

テレメトリ（位置・温度・負荷）も機体から返ってくるので、`ServoStatePoller` と同じ形にして
セッションと GUI がそのまま読めるようにしてある（`LinkTelemetry`）。
"""
from __future__ import annotations

from typing import Any, Callable

from serpens.hw.servo_bus import ServoState
from serpens.link.client import LinkClient
from serpens.link.messages import BODY_MAX, Telemetry
from serpens.link.protocol import State
from serpens.motion.gait import body_joint_names
from serpens.robot import DeviceStatus, MotionCommand, Pose
from serpens.safety import DriveState

HEAD_SLOTS = 3            # HEAD 指令が運べる軸数（protocol の固定長）


class LinkTelemetry:
    """機体から返る状態を `ServoStatePoller` と同じ形で見せる。"""

    def __init__(self, cfg: dict[str, Any], client: LinkClient, clock: Callable[[], float]) -> None:
        self._client = client
        self._clock = clock
        self._ids = [int(j["servo_id"]) for j in cfg["joints"]]
        self.stale_after_s = float(cfg["behavior"]["safety"]["telemetry"]["stale_after_s"])
        self.positions: dict[int, float] = {}
        self.states: dict[int, ServoState] = {}
        self.seen_at: dict[int, float] = {}
        self.errors = 0
        self.last_error = ""
        self.source = "ESP32 link"          # 値の出どころ（モックと混ぜない）

    def poll(self) -> None:
        """`LinkRobot.poll()` が受信した最新テレメトリを取り込む。"""
        tel: Telemetry | None = self._client.telemetry
        at = self._client.telemetry_at
        if tel is None or at is None:
            return
        for sid, ax in zip(self._ids, tel.axes):
            self.positions[sid] = ax.pos_deg
            self.states[sid] = ServoState(ax.pos_deg, ax.load, ax.volt_v, float(ax.temp_c))
            self.seen_at[sid] = at

    # ---- 鮮度（ServoStatePoller と同じ約束） ---------------------------------------
    def age_s(self, servo_id: int, now: float | None = None) -> float | None:
        t = self._clock() if now is None else now
        seen = self.seen_at.get(servo_id)
        return None if seen is None else max(t - seen, 0.0)

    def is_fresh(self, servo_id: int, now: float | None = None) -> bool:
        age = self.age_s(servo_id, now)
        return age is not None and age <= self.stale_after_s

    def fresh_states(self, now: float | None = None) -> dict[int, ServoState]:
        return {sid: st for sid, st in self.states.items() if self.is_fresh(sid, now)}

    def missing_axes(self, now: float | None = None) -> list[int]:
        return [sid for sid in self._ids if not self.is_fresh(sid, now)]

    def newest_age_s(self, now: float | None = None) -> float | None:
        ages = [a for a in (self.age_s(sid, now) for sid in self._ids) if a is not None]
        return min(ages) if ages else None

    def max_temperature_c(self, now: float | None = None) -> float | None:
        return max((st.temp_c for st in self.fresh_states(now).values()), default=None)

    def max_abs_load(self, now: float | None = None) -> float | None:
        loads = [abs(st.load) for st in self.fresh_states(now).values()]
        return max(loads) if loads else None


class _TorqueSink:
    """脱力演出の宛先。リンクでは軸ごとではなく1指令で全軸に効く。"""

    def __init__(self, client: LinkClient, ids: list[int], clock: Callable[[], float]) -> None:
        self._client, self._ids, self._clock = client, ids, clock
        self._last: float | None = None

    @property
    def ids(self) -> list[int]:
        return self._ids

    def set_torque_limit(self, servo_id: int, ratio: float) -> None:
        """同じ比率が全軸へ来るので、変化したときだけ送る。"""
        if self._last is not None and abs(self._last - ratio) < 1e-6:
            return
        self._last = ratio
        self._client.torque(self._clock(), ratio)


class LinkRobot:
    """ESP32 へ歩容パラメータを送る出力先。"""

    def __init__(self, cfg: dict[str, Any], client: LinkClient, clock: Callable[[], float],
                 pump: Callable[[float], None] | None = None) -> None:
        self.cfg = cfg
        self.client = client
        self._clock = clock
        self._pump = pump                       # 偽経路で機体を進める（実シリアルでは None）
        self.body = body_joint_names(cfg)
        names = [j["name"] for j in cfg["joints"]]
        self.head_names = [n for n in names if n not in self.body][:HEAD_SLOTS]
        self._ids = [int(j["servo_id"]) for j in cfg["joints"]]
        self._speed = {j["name"]: float(j["max_speed_dps"]) for j in cfg["joints"]}
        self.telemetry = LinkTelemetry(cfg, client, clock)
        self.torque_sink = _TorqueSink(client, self._ids, clock)
        self.errors: list[str] = []
        self._next_pose_send = 0.0
        self._breathing: bool | None = None
        self._armed_at: float | None = None

    # ---- 出力 -----------------------------------------------------------------------
    def send(self, t: float, cmd: MotionCommand) -> None:
        """胴体は歩容パラメータ（または姿勢）、首・頭は角度で送る。"""
        now = self._clock()
        if not self.client.armed:
            return                      # 通らない指令を送らない（ARM は apply_stop_state が行う）
        if cmd.gait is not None:
            self.client.set_drive(cmd.gait.amplitude_deg, cmd.gait.spatial_freq_deg,
                                  cmd.gait.temporal_freq_hz, cmd.gamma0_deg)
        else:
            self.client.clear_drive()
        if now >= self._next_pose_send:
            self._next_pose_send = now + self.client.drive_dt
            self._send_head(now, cmd.head)
            if cmd.gait is None:
                self._send_body(now, cmd.body_base)
        if self._breathing != cmd.breathing:
            self._breathing = cmd.breathing
            self.client.breath(now, cmd.breathing)

    def _send_head(self, now: float, head: Pose) -> None:
        if not head:
            return
        deg = [head.get(n, 0.0) for n in self.head_names] + [0.0] * (HEAD_SLOTS - len(self.head_names))
        speed = min(self._speed[n] for n in self.head_names)
        self.client.head(now, deg[0], deg[1], deg[2], speed)

    def _send_body(self, now: float, body: Pose) -> None:
        if not body:
            return
        angles = tuple(body.get(n, 0.0) for n in self.body[:BODY_MAX])
        speed = min(self._speed[n] for n in self.body)
        self.client.body(now, angles, speed)

    def hold(self, pose: Pose) -> None:
        """何もしない。**保持は機体が自分で行う**（PC が死んでも続く保持であることが要点）。"""

    def apply_stop_state(self, state: Any) -> None:
        """停止状態を機体側の指令へ写す。"""
        now = self._clock()
        if state is DriveState.RUN:
            self._arm_if_needed(now)
        elif state is DriveState.EMERGENCY:
            self.client.emergency(now)
        elif state is DriveState.DISABLED:
            self.client.stop(now, disable_torque=True)
        else:                                            # HOLD（通常停止）
            self.client.stop(now)

    def on_operator_start(self) -> None:
        """人が開始操作をした。ここでだけ再起動の記録を解除する。"""
        self.client.rebooted = False
        self._arm_if_needed(self._clock())

    def on_clear_emergency(self) -> None:
        """機体のラッチを解く。**待機へ戻るだけで、走行は再開しない**（完了条件 10）。"""
        self.client.clear_fault(self._clock())

    def _arm_if_needed(self, now: float) -> None:
        """待機中なら ARM する。**再起動後・ラッチ中は自動で ARM しない**（完了条件 6/8）。"""
        st = self.client.state
        if st is None or st.armed or st is State.FAULT_HOLD or self.client.latched or self.client.rebooted:
            return
        if self._armed_at is not None and now - self._armed_at < self.client.drive_dt:
            return
        self._armed_at = now
        self.client.arm(now)

    def poll(self) -> None:
        """heartbeat と DRIVE を送り、テレメトリを受け取る（停止中も続く）。"""
        now = self._clock()
        self.client.update(now)
        if self._pump is not None:
            self._pump(now)
        self.telemetry.poll()

    def close(self, torque_off: bool) -> list[str]:
        """停止を送ってから経路を閉じる。"""
        now = self._clock()
        self.client.stop(now, disable_torque=torque_off)
        self.poll()                                      # 停止指令を実際に送り出す
        try:
            self.client.tr.close()
        except Exception as e:                           # noqa: BLE001 - 切断失敗も伝える
            self.errors.append(f"駆動リンクの切断に失敗: {e}")
        return list(self.errors)

    # ---- 状態 -----------------------------------------------------------------------
    def positions_by_name(self) -> Pose:
        pos = self.telemetry.positions
        names = [j["name"] for j in self.cfg["joints"]]
        return {n: pos[sid] for n, sid in zip(names, self._ids) if sid in pos}

    def device_status(self) -> DeviceStatus | None:
        """機体の状態を GUI 用にまとめる。**模擬か実測かも一緒に返す。**"""
        tel = self.client.telemetry
        if tel is None:
            return None
        return DeviceStatus(state_ja=tel.state_ja, reason_ja=tel.reason_ja, simulated=tel.simulated,
                            age_s=self.client.age_s(self._clock()),
                            loop_period_us=tel.loop_period_us, overruns=tel.overruns,
                            missing_axes=len(self._ids) - len(tel.axes))

    @property
    def link_ok(self) -> bool:
        """機体からの状態が新しいか（USB 断・機体停止はここで分かる）。"""
        return self.client.link_ok(self._clock())

    @property
    def torque_ceiling_ok(self) -> bool:
        """リンク経路では**トルク上限は機体側（firmware/config.h）が持つ。**

        PC からは確認できないので、ここでは機体の生存だけを条件にする。上限が効いていることは
        ファームの検証（`docs/phase2_acceptance.md` 条件 11）で担保する。
        """
        return self.link_ok
