"""Virtual ESP32 — 機体側の参照実装。docs/link_protocol.md をそのまま写したもの。

**ファームウェア（firmware/serpens_esp32/）はこの実装と同じ判断順・同じ時定数で書く。**
通信路もタイマも持たない（`feed` に受信バイトと時刻、`tick` に時刻を渡す）ので、
偽シリアルと偽時計だけで完了条件を検証できる。

  PC ──フレーム──▶ Virtual ESP32 ──指令角──▶ VirtualServoBus ──▶ Simulated Serpens
                        │                         │
                        └──── テレメトリ ◀────── 模擬の実測値（SIMULATION 由来）

状態は7つ: BOOT / DISARMED / ARMED_HOLD / DRIVING / FAULT_HOLD / EMERGENCY_LATCHED / TORQUE_DISABLED

安全の要点（PC が死んでも機体が自分で止まる）:
  - heartbeat の期限は **HEARTBEAT だけ**が延ばす（DRIVE では延びない）
  - DRIVE の期限は **DRIVE だけ**が延ばす（heartbeat では延びない）
  - 停止は「現在の出力を保持」。ホーム姿勢へ動かさない。脱力は STOP(mode=disable) のときだけ
  - EMERGENCY はラッチ。再接続・ARM・STOP では解除されない
  - FAULT_HOLD からの復帰は **DISARMED まで**。走行には ARM + DRIVE が要る
"""
from __future__ import annotations

from typing import Any

from serpens.link import messages as m
from serpens.link.device_motion import DeviceMotion
from serpens.link.device_servos import VirtualServoBus
from serpens.link.protocol import (AGE_MAX_MS, VERSION, Cmd, Flag, Frame, FrameReader, Nack, Rep,
                                   Source, State, StopMode, StopReason, encode, seq_is_forward)

TICK_EPS = 1e-9                         # 時刻の丸め誤差で1周期飛ばさないための許容
OVERRUN_RATIO = 1.5                     # 制御周期がこの倍を超えたら overrun として数える
POSE_IDS = {0: "home", 1: "relax"}      # Phase 2 の機体側はこの2つだけ（とぐろは列なので PC 側）


class SimulatedDevice:
    """PC から見て実機と同じ振る舞いをする機体（**値は SIMULATION 由来**）。"""

    def __init__(self, cfg: dict[str, Any], now: float = 0.0, boot_id: int = 1) -> None:
        self.cfg = cfg
        lk = cfg["link"]
        self.hb_timeout_s = float(lk["heartbeat_timeout_ms"]) / 1000.0
        self.disarm_s = float(lk["drive_disarm_ms"]) / 1000.0
        self.telem_dt = 1.0 / float(lk["telemetry_hz"])
        self.ctrl_dt = 1.0 / float(lk["control_hz"])
        self.temp_limit_c = float(lk["faults"]["temp_limit_c"])
        self._nonce_keep = int(lk["clear_nonce_history"])
        self.boot_id = boot_id
        self._reader = FrameReader()
        self._init_runtime(now)

    def _init_runtime(self, now: float) -> None:
        """起動時の状態（再起動でもここへ戻る）。**必ず BOOT から始まる。**"""
        self._now = now
        self.mo = DeviceMotion(self.cfg)
        self.servos = VirtualServoBus(self.cfg, lambda: self._now)
        self.axes: dict[str, m.AxisTelemetry] = {}     # 模擬の実測値
        self.state = State.BOOT
        self.stop_reason = StopReason.BOOT
        self.breathing = False
        self.torque_ratio = 1.0
        self.last_seq: int | None = None
        self.last_drive_seq = 0
        self.t0 = now
        self._hb_at: float | None = None
        self._drive: m.Drive | None = None
        self._drive_at: float | None = None
        self._drive_until = 0.0
        self._head_until = 0.0
        self._body_until = 0.0
        self._nonces: list[int] = []
        self._last_ctrl = now
        self._last_telem = now
        self._pending_stop_at: float | None = None
        self.stop_latency_s: float | None = None
        self.loop_period_us = int(self.ctrl_dt * 1e6)
        self.overruns = 0
        self.events: list[tuple[float, StopReason, State]] = []
        self.write_count = 0

    def reboot(self, now: float) -> None:
        """電源断・リセット。boot_id が変わり、走行は再開しない（完了条件 6）。"""
        self.boot_id = (self.boot_id + 1) & 0xFFFF
        self._reader = FrameReader()
        self._init_runtime(now)

    # ---- 状態 -----------------------------------------------------------------------
    @property
    def driving(self) -> bool:
        return self.state is State.DRIVING

    @property
    def torque_on(self) -> bool:
        return self.state is not State.TORQUE_DISABLED

    @property
    def goals(self) -> dict[str, float]:
        """指令角（補間後・呼吸を足す前）。"""
        return self.mo.goals

    @property
    def output(self) -> dict[str, float]:
        """サーボへ書いている角度（指令値）。模擬の実測値は `axes`。"""
        return self.mo.output(self._last_ctrl - self.t0, self.breathing and not self.driving)

    def _set_state(self, state: State, reason: StopReason, now: float) -> None:
        self.state, self.stop_reason = state, reason
        self._log(now)

    # ---- 受信 -----------------------------------------------------------------------
    def feed(self, data: bytes, now: float) -> bytes:
        """受信バイトを処理して応答フレームを返す。"""
        self._now = now
        out = b""
        for fr in self._reader.feed(data):
            out += self._handle(fr, now)
        return out

    def _nack(self, fr: Frame, reason: Nack) -> bytes:
        return encode(Rep.NACK, 0, m.pack_nack(fr.seq, fr.type, reason))

    def _ack(self, fr: Frame) -> bytes:
        return encode(Rep.ACK, 0, m.pack_ack(fr.seq, fr.type))

    def _handle(self, fr: Frame, now: float) -> bytes:
        """1フレームを検査して実行する（判断順は docs/link_protocol.md §1 の表）。"""
        if not fr.crc_ok:
            return self._nack(fr, Nack.BAD_CRC)
        if fr.version != VERSION:
            return self._nack(fr, Nack.BAD_VERSION)
        if fr.type not in set(Cmd):
            return self._nack(fr, Nack.UNKNOWN_CMD)
        if not fr.length_ok:
            return self._nack(fr, Nack.BAD_LENGTH)
        if not seq_is_forward(fr.seq, self.last_seq):
            return self._nack(fr, Nack.STALE_SEQ)      # 古い・重複は実行しない（完了条件 7）
        self.last_seq = fr.seq
        return self._dispatch(Cmd(fr.type), fr, now)

    def _dispatch(self, cmd: Cmd, fr: Frame, now: float) -> bytes:
        if cmd is Cmd.HEARTBEAT:
            self._hb_at = now                          # DRIVE の期限はここでは延ばさない
            if self.state is State.BOOT:
                self._set_state(State.DISARMED, StopReason.BOOT, now)
            return self._ack(fr)
        if cmd is Cmd.PING:
            return self._ack(fr)
        if cmd is Cmd.CLEAR_FAULT:
            return self._clear_fault(fr, now)
        if cmd is Cmd.EMERGENCY:
            self._stop(StopReason.EMERGENCY_CMD, now, State.EMERGENCY_LATCHED)
            return self._ack(fr)
        if cmd is Cmd.STOP:
            mode = m.unpack_stop(fr.payload)[0]
            to = State.TORQUE_DISABLED if mode == StopMode.DISABLE_TORQUE else State.DISARMED
            self._stop(StopReason.OPERATOR_STOP, now, to)
            return self._ack(fr)
        if self.state is State.EMERGENCY_LATCHED:
            return self._nack(fr, Nack.LATCHED)        # ラッチ中は以降すべて拒否（完了条件 8/9）
        if cmd is Cmd.DISARM:
            self._stop(StopReason.OPERATOR_STOP, now, State.DISARMED)
            return self._ack(fr)
        if cmd is Cmd.ARM:
            return self._arm(fr, now)
        if cmd is Cmd.LIMITS:
            return self._nack(fr, Nack.UNKNOWN_CMD)    # Phase 9。機体の絶対上限は config 固定
        return self._dispatch_motion(cmd, fr, now)

    def _arm(self, fr: Frame, now: float) -> bytes:
        """走行可へ。**異常で止まっている間は ARM させない**（先に原因を解く）。"""
        if self.state is State.FAULT_HOLD:
            return self._nack(fr, Nack.BUSY)
        if not self._hb_fresh(now):
            return self._nack(fr, Nack.NO_HEARTBEAT)
        self.servos.set_torque(True)
        self._set_state(State.ARMED_HOLD, StopReason.NONE, now)
        return self._ack(fr)

    def _dispatch_motion(self, cmd: Cmd, fr: Frame, now: float) -> bytes:
        """動きを伴う指令（ARMED_HOLD / DRIVING かつ heartbeat 有効のときだけ通す）。"""
        if not self.state.armed:
            return self._nack(fr, Nack.DISARMED)
        if not self._hb_fresh(now):
            return self._nack(fr, Nack.NO_HEARTBEAT)
        if cmd is Cmd.DRIVE:
            d = m.Drive.unpack(fr.payload)
            if not self.mo.drive_ok(d):
                return self._nack(fr, Nack.OUT_OF_RANGE)  # 状態も出力も変えない（完了条件 11）
            self._drive, self._drive_at = d, now
            self._drive_until = now + d.ttl_ms / 1000.0
            self.last_drive_seq = fr.seq
            if self.state is not State.DRIVING:
                self._set_state(State.DRIVING, StopReason.NONE, now)
            return self._ack(fr)
        if cmd is Cmd.HEAD:
            h = m.Head.unpack(fr.payload)
            if not self.mo.head_ok(h):
                return self._nack(fr, Nack.OUT_OF_RANGE)
            self.mo.set_head(h)
            self._head_until = now + h.ttl_ms / 1000.0
            return self._ack(fr)
        if cmd is Cmd.BODY:
            b = m.Body.unpack(fr.payload)
            if self.driving:
                return self._nack(fr, Nack.BUSY)       # 胴体は歩容が使っている
            if not self.mo.body_ok(b):
                return self._nack(fr, Nack.OUT_OF_RANGE)
            self.mo.set_body(b)
            self._body_until = now + b.ttl_ms / 1000.0
            return self._ack(fr)
        if cmd is Cmd.TORQUE:
            ratio = m.unpack_torque(fr.payload)
            if not 0.0 < ratio <= 1.0:
                return self._nack(fr, Nack.OUT_OF_RANGE)
            self.torque_ratio = ratio
            self.servos.set_torque_ratio(ratio)        # 安全上限に対する割合
            return self._ack(fr)
        if cmd is Cmd.POSE:
            pose = POSE_IDS.get(fr.payload[0])
            if pose is None or self.driving or not isinstance(self.cfg["poses"].get(pose), dict):
                return self._nack(fr, Nack.OUT_OF_RANGE)
            self.mo.set_pose(self.cfg["poses"][pose])
            return self._ack(fr)
        if cmd is Cmd.BREATH:
            self.breathing = bool(fr.payload[0])
            return self._ack(fr)
        return self._nack(fr, Nack.UNKNOWN_CMD)        # 取りこぼしを別の指令として実行しない

    def _clear_fault(self, fr: Frame, now: float) -> bytes:
        """ラッチ解除。**待機（DISARMED）へ戻すだけ**（完了条件 10）。"""
        nonce = m.unpack_nonce(fr.payload)
        if nonce in self._nonces:
            return self._nack(fr, Nack.NONCE_REUSED)
        self._nonces = (self._nonces + [nonce])[-self._nonce_keep:]
        self._drive = None
        self.servos.set_torque(True)
        self._set_state(State.DISARMED, self.stop_reason, now)   # 停止理由は残す
        return self._ack(fr)

    def _hb_fresh(self, now: float) -> bool:
        return self._hb_at is not None and now - self._hb_at <= self.hb_timeout_s

    # ---- 周期処理 --------------------------------------------------------------------
    def tick(self, now: float) -> bytes:
        """watchdog → 制御 → テレメトリ。PC からの受信が無くても回り続ける。"""
        self._now = now
        out = self._watchdogs(now)
        if now - self._last_ctrl >= self.ctrl_dt - TICK_EPS:
            self._control(now)
        if now - self._last_telem >= self.telem_dt - TICK_EPS:
            self._last_telem = now
            out += encode(Rep.TELEMETRY, 0, self.telemetry(now).pack())
        return out

    def _watchdogs(self, now: float) -> bytes:
        """PC が死んでも自分で止まる部分。"""
        before = (self.stop_reason, self.state)
        hot = max((a.temp_c for a in self.axes.values()), default=0.0)
        if hot > self.temp_limit_c:
            self._stop(StopReason.OVERHEAT, now, State.EMERGENCY_LATCHED)
        elif any(a.fault for a in self.axes.values()):
            self._stop(StopReason.SERVO_FAULT, now, State.EMERGENCY_LATCHED)
        elif self.state.armed and self.servos.read_once and self.servos.missing(self.axes):
            self._stop(StopReason.SERVO_FAULT, now, State.FAULT_HOLD)   # 応答しない軸がある
        elif self._hb_at is not None and not self._hb_fresh(now) and self.state.armed:
            self._stop(StopReason.HEARTBEAT_LOST, now, State.FAULT_HOLD)  # 条件 2/3/5
        elif self.state is State.FAULT_HOLD and self._hb_fresh(now) and not self.servos.missing(self.axes):
            self._set_state(State.DISARMED, self.stop_reason, now)      # 復帰は待機まで
        elif self.driving and now > self._drive_until:
            self._stop(StopReason.DRIVE_TTL, now, State.ARMED_HOLD)     # 保持（条件 4）
        elif (self.state is State.ARMED_HOLD and self._drive_at is not None
              and now - self._drive_at > self.disarm_s):
            self._stop(StopReason.DRIVE_TTL, now, State.DISARMED)
        if now > self._head_until:
            self.mo.stop_head()                                     # 頭も期限切れで止める
        if now > self._body_until and not self.driving:
            self.mo.stop_body()                                     # 胴体の姿勢も期限切れで止める
        if (self.stop_reason, self.state) != before:
            return encode(Rep.EVENT, 0, m.pack_event(int(self.stop_reason), int(self.state)))
        return b""

    def _stop(self, reason: StopReason, now: float, to: State) -> None:
        """停止。現在の出力を保持する（ホーム姿勢へ動かさない）。"""
        if self.state is State.EMERGENCY_LATCHED and to is not State.EMERGENCY_LATCHED:
            return                                        # ラッチ中の理由は上書きしない
        if self.driving or self._pending_stop_at is None:
            self._pending_stop_at = now                   # 停止が出力へ届くまでの時間を測る
        self._drive = None
        self.breathing = False
        self.torque_ratio = 1.0                           # 演出の脱力は停止で解除する
        self.servos.set_torque_ratio(1.0)
        self.mo.hold()                                    # いまの角度で保持
        if to is State.TORQUE_DISABLED:
            self.servos.set_torque(False)
        self._set_state(to, reason, now)

    def _control(self, now: float) -> None:
        """制御周期。歩容生成 → 指令角をサーボへ → 模擬の実測値を読む。"""
        dt = now - self._last_ctrl
        self._last_ctrl = now
        self.loop_period_us = min(int(dt * 1e6), AGE_MAX_MS)
        if dt > self.ctrl_dt * OVERRUN_RATIO:
            self.overruns += 1
        self.mo.step(dt, self._drive if self.driving else None)
        if self.torque_on:
            self.servos.write(self.output)                # 9軸同期書き込み（保持中も送る）
            self.write_count += 1
            if self._pending_stop_at is not None:
                self.stop_latency_s = now - self._pending_stop_at
                self._pending_stop_at = None
        self.axes = self.servos.read()                    # 指令ではなく模擬の実測値

    def _log(self, now: float) -> None:
        self.events.append((now, self.stop_reason, self.state))

    # ---- テレメトリ -------------------------------------------------------------------
    def telemetry(self, now: float) -> m.Telemetry:
        """PC へ返す状態一式（停止理由はここから読める。完了条件 12）。"""
        flags = (Flag.DRIVING if self.driving else 0) | (Flag.BREATHING if self.breathing else 0)
        flags |= Flag.HEARTBEAT_OK if self._hb_fresh(now) else 0
        flags |= Flag.DRIVE_VALID if self.driving and now <= self._drive_until else 0
        flags |= Flag.TORQUE_ON if self.torque_on else 0
        flags |= Flag.ARMED if self.state.armed else 0
        flags |= Flag.EMERGENCY_LATCHED if self.state is State.EMERGENCY_LATCHED else 0
        flags |= Flag.SIMULATED                            # **実測ではない**
        flags |= Flag.SERVO_MISSING if self.servos.missing(self.axes) else 0
        flags |= Flag.OVERRUN if self.overruns else 0
        return m.Telemetry(
            boot_id=self.boot_id,
            uptime_ms=int((now - self.t0) * 1000) & 0xFFFFFFFF,
            state=self.state, stop_reason=self.stop_reason,
            last_seq=self.last_seq or 0, last_drive_seq=self.last_drive_seq, flags=int(flags),
            heartbeat_age_ms=self._age_ms(self._hb_at, now),
            drive_age_ms=self._age_ms(self._drive_at, now),
            drive_ttl_remaining_ms=max(int((self._drive_until - now) * 1000), 0) if self.driving else 0,
            loop_period_us=self.loop_period_us, overruns=self.overruns,
            source=Source.SIMULATION,
            axes=[self.axes[n] for n in self.servos.names if n in self.axes])

    @staticmethod
    def _age_ms(at: float | None, now: float) -> int:
        return AGE_MAX_MS if at is None else min(int((now - at) * 1000), AGE_MAX_MS)

    def inject_axis(self, name: str, *, temp_c: float | None = None, fault: int | None = None,
                    offline: bool | None = None, load: float | None = None) -> None:
        """試験用: 機体側の異常を起こす（実機ではサーボの読み値）。"""
        self.servos.inject(name, temp_c=temp_c, fault=fault, offline=offline, load=load)
