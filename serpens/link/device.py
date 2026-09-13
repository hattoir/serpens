"""機体側（ESP32-S3）の参照実装。docs/link_protocol.md v1 をそのまま写したもの。

**ファームウェア（firmware/serpens_esp32/）はこの実装と同じ判断順・同じ時定数で書く。**
ここには通信路もタイマも入れない（`feed` に受信バイトと時刻、`tick` に時刻を渡す）ので、
偽シリアルと偽時計だけで完了条件 1/4/5/6/7/8/9/10/11/12 を検証できる。
出力の作り方（歩容・補間・上限）は device_motion.py。

安全の要点（PC が死んでも機体が自分で止まる）:
  - heartbeat の期限は **HEARTBEAT だけ**が延ばす（DRIVE では延びない）
  - DRIVE の期限は **DRIVE だけ**が延ばす（heartbeat では延びない）
  - 停止は「現在の出力を保持」。ホーム姿勢へ動かさない。脱力は STOP(mode=disable) のときだけ
  - EMERGENCY はラッチ。再接続・ARM・STOP では解除されない
"""
from __future__ import annotations

from typing import Any

from serpens.link import messages as m
from serpens.link.device_motion import DeviceMotion
from serpens.link.protocol import (VERSION, Cmd, Flag, Frame, FrameReader, Nack, Rep, State,
                                   StopMode, StopReason, encode, seq_is_forward)

TICK_EPS = 1e-9                         # 時刻の丸め誤差で1周期飛ばさないための許容
POSE_IDS = {0: "home", 1: "relax"}      # Phase 2 の機体側はこの2つだけ（とぐろは列なので PC 側）


class SimulatedDevice:
    """PC から見て実機と同じ振る舞いをする機体。"""

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
        """起動時の状態（再起動でもここへ戻る）。**必ず DISARMED から始まる。**"""
        self.mo = DeviceMotion(self.cfg)
        self.axis_temp_c = {n: float(self.cfg["mock_servo"]["ambient_c"]) for n in self.mo.joints}
        self.axis_fault = {n: 0 for n in self.mo.joints}
        self.state = State.DISARMED
        self.stop_reason = StopReason.BOOT
        self.torque_on = True                 # 起動時は現在姿勢を保持（勝手に脱力しない）
        self.driving = False
        self.breathing = False
        self.last_seq: int | None = None
        self.t0 = now
        self._hb_at: float | None = None
        self._drive: m.Drive | None = None
        self._drive_at: float | None = None
        self._drive_until = 0.0
        self._head_until = 0.0
        self._body_until = 0.0
        self.torque_ratio = 1.0               # TORQUE 指令（脱力の演出）。1.0 = 100%
        self._nonces: list[int] = []
        self._last_ctrl = now
        self._last_telem = now
        self._pending_stop_at: float | None = None
        self.stop_latency_s: float | None = None
        self.events: list[tuple[float, StopReason, State]] = []
        self.write_count = 0

    def reboot(self, now: float) -> None:
        """電源断・リセット。boot_id が変わり、走行は再開しない（完了条件 6）。"""
        self.boot_id = (self.boot_id + 1) & 0xFFFF
        self._reader = FrameReader()
        self._init_runtime(now)

    @property
    def goals(self) -> dict[str, float]:
        """補間後の関節角（呼吸を足す前）。"""
        return self.mo.goals

    @property
    def output(self) -> dict[str, float]:
        """実際にサーボへ書く角度。"""
        return self.mo.output(self._last_ctrl - self.t0, self.breathing and not self.driving)

    # ---- 受信 -----------------------------------------------------------------------
    def feed(self, data: bytes, now: float) -> bytes:
        """受信バイトを処理して応答フレームを返す。"""
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
            return self._ack(fr)
        if cmd is Cmd.PING:
            return self._ack(fr)
        if cmd is Cmd.CLEAR_FAULT:
            return self._clear_fault(fr, now)
        if cmd is Cmd.EMERGENCY:
            self._stop(StopReason.EMERGENCY_CMD, now, latch=True)
            return self._ack(fr)
        if cmd is Cmd.STOP:
            mode = m.unpack_stop(fr.payload)[0]
            self._stop(StopReason.OPERATOR_STOP, now, torque_off=mode == StopMode.DISABLE_TORQUE)
            return self._ack(fr)
        if self.state is State.EMERGENCY:
            return self._nack(fr, Nack.LATCHED)        # ラッチ中は以降すべて拒否（完了条件 8/9）
        if cmd is Cmd.DISARM:
            self._stop(StopReason.OPERATOR_STOP, now)
            return self._ack(fr)
        if cmd is Cmd.ARM:
            if not self._hb_fresh(now):
                return self._nack(fr, Nack.NO_HEARTBEAT)
            self.state, self.stop_reason = State.ARMED, StopReason.NONE
            self.torque_on = True
            return self._ack(fr)
        if cmd is Cmd.LIMITS:
            return self._nack(fr, Nack.UNKNOWN_CMD)    # Phase 9。機体の絶対上限は config 固定
        return self._dispatch_motion(cmd, fr, now)

    def _dispatch_motion(self, cmd: Cmd, fr: Frame, now: float) -> bytes:
        """動きを伴う指令（ARMED かつ heartbeat 有効のときだけ通す）。"""
        if self.state is not State.ARMED:
            return self._nack(fr, Nack.DISARMED)
        if not self._hb_fresh(now):
            return self._nack(fr, Nack.NO_HEARTBEAT)
        if cmd is Cmd.DRIVE:
            d = m.Drive.unpack(fr.payload)
            if not self.mo.drive_ok(d):
                return self._nack(fr, Nack.OUT_OF_RANGE)  # 状態も出力も変えない（完了条件 11）
            self._drive, self._drive_at = d, now
            self._drive_until = now + d.ttl_ms / 1000.0
            self.driving = True
            self.stop_reason = StopReason.NONE
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
                return self._nack(fr, Nack.BUSY)       # 胴体は歩容が使っている（先に DRIVE を止める）
            if not self.mo.body_ok(b):
                return self._nack(fr, Nack.OUT_OF_RANGE)
            self.mo.set_body(b)
            self._body_until = now + b.ttl_ms / 1000.0
            return self._ack(fr)
        if cmd is Cmd.TORQUE:
            ratio = m.unpack_torque(fr.payload)
            if not 0.0 < ratio <= 1.0:
                return self._nack(fr, Nack.OUT_OF_RANGE)
            self.torque_ratio = ratio                  # 実機ではサーボのトルク制限レジスタへ
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
        self.state = State.DISARMED
        self.driving = False
        self._drive = None
        self._log(now)                                 # 停止理由は残す（PC が経緯を読めるように）
        return self._ack(fr)

    def _hb_fresh(self, now: float) -> bool:
        return self._hb_at is not None and now - self._hb_at <= self.hb_timeout_s

    # ---- 周期処理 --------------------------------------------------------------------
    def tick(self, now: float) -> bytes:
        """watchdog → 制御 → テレメトリ。PC からの受信が無くても回り続ける。"""
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
        if max(self.axis_temp_c.values()) > self.temp_limit_c:
            self._stop(StopReason.OVERHEAT, now, latch=True)
        elif any(self.axis_fault.values()):
            self._stop(StopReason.SERVO_FAULT, now, latch=True)
        elif self._hb_at is not None and not self._hb_fresh(now):
            self._stop(StopReason.HEARTBEAT_LOST, now)   # USB 断・PC 強制終了もここ（条件 2/3/5）
        elif self.driving and now > self._drive_until:
            self._stop(StopReason.DRIVE_TTL, now, disarm=False)     # 保持のまま ARMED（条件 4）
        elif (self.state is State.ARMED and self._drive_at is not None
              and now - self._drive_at > self.disarm_s):
            self._stop(StopReason.DRIVE_TTL, now)
        if now > self._head_until:
            self.mo.stop_head()                                     # 頭も期限切れで止める
        if now > self._body_until:
            self.mo.stop_body()                                     # 胴体の姿勢も期限切れで止める
        if (self.stop_reason, self.state) != before:
            self._log(now)
            return encode(Rep.EVENT, 0, m.pack_event(int(self.stop_reason), int(self.state)))
        return b""

    def _stop(self, reason: StopReason, now: float, *, latch: bool = False,
              disarm: bool = True, torque_off: bool = False) -> None:
        """停止。現在の出力を保持する（ホーム姿勢へ動かさない）。"""
        if self.state is State.EMERGENCY and not latch:
            return                                        # ラッチ中の理由は上書きしない
        if self.driving or self._pending_stop_at is None:
            self._pending_stop_at = now                   # 停止が出力へ届くまでの時間を測る
        self.driving = False
        self._drive = None
        self.breathing = False
        self.torque_ratio = 1.0                           # 演出の脱力は停止で解除する
        self.mo.hold()                                    # いまの角度で保持
        self.stop_reason = reason
        if latch:
            self.state = State.EMERGENCY
        elif disarm:
            self.state = State.DISARMED
        if torque_off:
            self.torque_on = False

    def _control(self, now: float) -> None:
        """制御周期。歩容生成と、頭・姿勢の速度制限つき補間。"""
        dt = now - self._last_ctrl
        self._last_ctrl = now
        self.mo.step(dt, self._drive if self.driving else None)
        if self.torque_on:
            self.write_count += 1                         # ここで 9軸同期書き込み（保持中も送る）
            if self._pending_stop_at is not None:
                self.stop_latency_s = now - self._pending_stop_at
                self._pending_stop_at = None

    def _log(self, now: float) -> None:
        self.events.append((now, self.stop_reason, self.state))

    # ---- テレメトリ -------------------------------------------------------------------
    def telemetry(self, now: float) -> m.Telemetry:
        """PC へ返す状態一式（停止理由はここから読める。完了条件 12）。"""
        flags = (Flag.DRIVING if self.driving else 0) | (Flag.BREATHING if self.breathing else 0)
        flags |= Flag.HEARTBEAT_OK if self._hb_fresh(now) else 0
        flags |= Flag.DRIVE_VALID if self.driving and now <= self._drive_until else 0
        flags |= Flag.TORQUE_ON if self.torque_on else 0
        out = self.output
        axes = [m.AxisTelemetry(out[n], 0.0, int(self.axis_temp_c[n]),
                                float(self.cfg["mock_servo"]["voltage_nominal_v"]),
                                self.axis_fault[n], None) for n in self.mo.joints]
        return m.Telemetry(self.boot_id, int((now - self.t0) * 1000) & 0xFFFFFFFF, self.state,
                           self.stop_reason, self.last_seq or 0, int(flags), axes)

    def inject_axis(self, name: str, *, temp_c: float | None = None, fault: int | None = None) -> None:
        """試験用: 機体側の異常を起こす（実機ではサーボの読み値）。"""
        if temp_c is not None:
            self.axis_temp_c[name] = temp_c
        if fault is not None:
            self.axis_fault[name] = fault
