"""PC 側の駆動リンク。heartbeat を送り、DRIVE に期限を付け、機体の状態を読む。

**PC は機体を「止める」ことはできても、機体に代わって安全を保証しない。**
止めるのは機体側（device.py / ファーム）の仕事で、ここはその引き金と表示を持つだけ。

時間は呼び出し側が渡す（`update(now)`）。試験では偽時計、実行時は `time.monotonic()`。
"""
from __future__ import annotations

import random
import struct
from typing import Any

from serpens.link import messages as m
from serpens.link.faults import finite
from serpens.motion.pitch_guard import PitchGuard
from serpens.link.protocol import (Cmd, FrameReader, Nack, Rep, State, StopMode, StopReason,
                                   encode)

RTT_KEEP = 32              # 往復時間を測るために覚えておく送信の数
HISTORY_KEEP = 64          # 停止理由の履歴（完了条件 12）


class LinkClient:
    """機体への指令と、機体からの状態。"""

    def __init__(self, transport: Any, cfg: dict[str, Any], now: float = 0.0) -> None:
        lk = cfg["link"]
        self.tr = transport
        self.hb_dt = 1.0 / float(lk["heartbeat_hz"])
        self.drive_dt = self.hb_dt                     # DRIVE も heartbeat と同じ頻度で更新する
        self.drive_ttl_ms = int(lk["drive_ttl_ms"])
        self.stale_s = float(lk["heartbeat_timeout_ms"]) / 1000.0
        self._reader = FrameReader()
        self._seq = 0
        self._sent_at: dict[int, tuple[float, int]] = {}
        self._last_hb = now - self.hb_dt
        self._last_drive = now - self.drive_dt
        self.drive: m.Drive | None = None              # None = DRIVE を送らない（機体は TTL で止まる）
        self.telemetry: m.Telemetry | None = None
        self.telemetry_at: float | None = None
        self.boot_id: int | None = None
        self.rebooted = False                          # 機体が再起動した（**自動で ARM しない**）
        self.history: list[tuple[float, StopReason, State]] = []
        self.nacks: list[tuple[float, Cmd, Nack]] = []
        self.rtt_ms: float | None = None
        self.events = 0
        self.rejected: list[Any] = []              # 送らずに捨てた指令（NaN / Inf / 桁あふれ）
        self.pitch = PitchGuard.from_cfg(cfg)      # Floor Watch の頭（J7）の範囲。既定オフ。PC 側は先に同じ検査（機体側も独立に行う）
        self.last_j7: float | None = None

    # ---- 送信 -----------------------------------------------------------------------
    def _pack(self, build: Any) -> bytes | None:
        """payload を組み立てる。**表現できない値なら送らない。**

        速度 9000°/s のような値は u16 に入らず `struct.pack` が例外を投げる。
        そのまま投げると送信側（制御ループ）が落ちるので、ここで捨てて記録する。
        """
        try:
            return build()
        except (ValueError, OverflowError, struct.error) as e:
            self.rejected.append(str(e))
            return None

    def _send(self, cmd: Cmd, payload: bytes = b"", now: float = 0.0) -> int:
        self._seq = (self._seq + 1) & 0xFFFF or 1
        self.tr.write(encode(cmd, self._seq, payload))
        self._sent_at[self._seq] = (now, int(cmd))
        if len(self._sent_at) > RTT_KEEP:
            for k in sorted(self._sent_at)[:-RTT_KEEP]:
                self._sent_at.pop(k, None)
        return self._seq

    def arm(self, now: float) -> int:
        """走行可能にする。**人が操作したときだけ呼ぶ**（再接続や再起動で自動で呼ばない）。

        いま見えている `boot_id` を添える。テレメトリを一度も受けていなければ送らない
        （どの機体を ARM するのか分かっていない状態で走らせない）。
        """
        if self.boot_id is None:
            self.rejected.append("boot_id 未取得のまま ARM しようとした")
            return -1
        return self._send(Cmd.ARM, m.pack_arm(self.boot_id), now)

    def disarm(self, now: float) -> int:
        self.drive = None
        return self._send(Cmd.DISARM, b"", now)

    def stop(self, now: float, disable_torque: bool = False) -> int:
        """通常停止（保持）。脱力は明示したときだけ。"""
        self.drive = None
        mode = StopMode.DISABLE_TORQUE if disable_torque else StopMode.HOLD
        return self._send(Cmd.STOP, m.pack_stop(mode, StopReason.OPERATOR_STOP), now)

    def emergency(self, now: float) -> int:
        """緊急停止。**機体側でラッチされ、PC からの再接続では解除されない。**"""
        self.drive = None
        return self._send(Cmd.EMERGENCY, bytes([int(StopReason.EMERGENCY_CMD)]), now)

    def clear_fault(self, now: float, nonce: int | None = None) -> int:
        """ラッチ解除 → 待機。走行は再開しない（`arm` + `set_drive` が要る）。"""
        self.drive = None
        return self._send(Cmd.CLEAR_FAULT, m.pack_nonce(nonce if nonce is not None
                                                        else random.getrandbits(32)), now)

    def set_drive(self, amplitude_deg: float, spatial_freq_deg: float, temporal_freq_hz: float,
                  gamma_deg: float = 0.0, ttl_ms: int | None = None) -> None:
        """歩容を指示する。以後 `update` が期限付きで送り続ける（送信を止めれば機体は止まる）。

        **NaN / Inf は通信へ出す前に捨てる。** 上位の計算が壊れたときに壊れた値を機体へ渡さない
        （`struct.pack` が例外を投げて通信スレッドごと落ちるのも防ぐ）。
        """
        if not finite(amplitude_deg, spatial_freq_deg, temporal_freq_hz, gamma_deg):
            self.rejected.append((amplitude_deg, spatial_freq_deg, temporal_freq_hz, gamma_deg))
            self.drive = None                      # 走らせない（前の指令も引き継がない）
            return
        self.drive = m.Drive(ttl_ms if ttl_ms is not None else self.drive_ttl_ms, amplitude_deg,
                             spatial_freq_deg, temporal_freq_hz, gamma_deg)

    def clear_drive(self) -> None:
        """DRIVE の送信をやめる（TTL 切れで機体が保持へ入る。完了条件 4）。"""
        self.drive = None

    def head(self, now: float, j7: float, j8: float, j9: float, speed_dps: float,
             ttl_ms: int | None = None) -> int:
        """頭部の目標角。NaN / Inf は送らない（-1 を返す）。"""
        if not finite(j7, j8, j9, speed_dps):
            self.rejected.append((j7, j8, j9, speed_dps))
            return -1
        ttl = ttl_ms if ttl_ms is not None else self.drive_ttl_ms
        if self.pitch.enabled:
            v = self.pitch.prevet_head(j7, speed_dps, self.last_j7)      # 範囲外はクランプ + 記録、速さは上限へ
            j7, speed_dps = v.deg, v.speed_dps
        payload = self._pack(lambda: m.Head(ttl, j7, j8, j9, speed_dps).pack())
        if payload is not None:
            self.last_j7 = j7
        return -1 if payload is None else self._send(Cmd.HEAD, payload, now)

    def body(self, now: float, angles_deg: tuple[float, ...], speed_dps: float,
             ttl_ms: int | None = None) -> int:
        """胴体の姿勢（とぐろ・鎌首）。**歩容中は機体が拒否する**ので、先に `clear_drive()`。"""
        if not finite(*angles_deg, speed_dps):
            self.rejected.append(tuple(angles_deg) + (speed_dps,))
            return -1
        ttl = ttl_ms if ttl_ms is not None else self.drive_ttl_ms
        payload = self._pack(lambda: m.Body(ttl, angles_deg, speed_dps).pack())
        return -1 if payload is None else self._send(Cmd.BODY, payload, now)

    def torque(self, now: float, ratio: float) -> int:
        """トルク比（脱力の演出）。停止すると機体側で 100% に戻る。"""
        return self._send(Cmd.TORQUE, m.pack_torque(ratio), now)

    def pose(self, now: float, pose_id: int) -> int:
        return self._send(Cmd.POSE, bytes([pose_id]), now)

    def breath(self, now: float, on: bool) -> int:
        return self._send(Cmd.BREATH, bytes([1 if on else 0]), now)

    def ping(self, now: float) -> int:
        return self._send(Cmd.PING, b"", now)

    # ---- 周期処理 --------------------------------------------------------------------
    def update(self, now: float) -> None:
        """受信を捌き、heartbeat と DRIVE を周期送信する。"""
        for fr in self._reader.feed(self.tr.read()):
            self._on_frame(fr, now)
        if now - self._last_hb >= self.hb_dt:
            self._last_hb = now
            self._send(Cmd.HEARTBEAT, b"", now)
        if self.drive is not None and now - self._last_drive >= self.drive_dt:
            self._last_drive = now
            payload = self._pack(self.drive.pack)
            if payload is None:
                self.drive = None                  # 送れない歩容は捨てる（機体は TTL で止まる）
            else:
                self._send(Cmd.DRIVE, payload, now)

    def _on_frame(self, fr: Any, now: float) -> None:
        if not fr.crc_ok:
            return
        if fr.type == Rep.TELEMETRY:
            self._on_telemetry(m.Telemetry.unpack(fr.payload), now)
        elif fr.type == Rep.ACK:
            self._rtt(m.unpack_ack(fr.payload)[0], now)
        elif fr.type == Rep.NACK:
            seq, cmd, reason = m.unpack_nack(fr.payload)
            self._rtt(seq, now)
            self.nacks.append((now, Cmd(cmd) if cmd in set(Cmd) else cmd, Nack(reason)))
            if reason in (Nack.LATCHED, Nack.DISARMED, Nack.OUT_OF_RANGE):
                self.drive = None                  # 拒否された指令を送り続けない
        elif fr.type == Rep.EVENT:
            self.events += 1

    def _rtt(self, seq: int, now: float) -> None:
        sent = self._sent_at.pop(seq, None)
        if sent is not None:
            self.rtt_ms = (now - sent[0]) * 1000.0

    def _on_telemetry(self, tel: m.Telemetry, now: float) -> None:
        if self.boot_id is not None and tel.boot_id != self.boot_id:
            self.rebooted = True                   # 走行は自動再開しない（完了条件 6）
            self.drive = None
        self.boot_id = tel.boot_id
        prev = self.telemetry
        self.telemetry, self.telemetry_at = tel, now
        if prev is None or (prev.state, prev.stop_reason) != (tel.state, tel.stop_reason):
            self.history.append((now, tel.stop_reason, tel.state))
            del self.history[:-HISTORY_KEEP]
        if not tel.state.armed:
            self.drive = None                      # 機体が待機・異常・ラッチなら送り続けない

    # ---- 表示 -----------------------------------------------------------------------
    def age_s(self, now: float) -> float | None:
        return None if self.telemetry_at is None else now - self.telemetry_at

    def link_ok(self, now: float) -> bool:
        """機体からの状態が新しいか（USB 断・機体停止はここで分かる）。"""
        age = self.age_s(now)
        return age is not None and age <= self.stale_s

    @property
    def state(self) -> State | None:
        return self.telemetry.state if self.telemetry else None

    @property
    def latched(self) -> bool:
        return self.state is State.EMERGENCY_LATCHED

    @property
    def armed(self) -> bool:
        return self.state is not None and self.state.armed

    @property
    def driving(self) -> bool:
        return bool(self.telemetry and self.telemetry.driving)

    def status_text(self, now: float) -> str:
        """人が読む1行（GUI と link_check 用）。"""
        if self.telemetry is None:
            return "機体からの応答なし"
        age = self.age_s(now) or 0.0
        stale = "" if self.link_ok(now) else f"（{age:.1f}s 前の情報）"
        tel = self.telemetry
        drive = "駆動中" if tel.driving else "停止"
        return f"{tel.state_ja} / {drive} / 停止理由: {tel.reason_ja}{stale}"

    def reason_history_text(self) -> list[str]:
        """停止理由の履歴（新しいものが後ろ）。"""
        return [f"{t:7.2f}s  {m.REASON_JA.get(r, str(r))}  →  {m.STATE_JA.get(s, str(s))}"
                for t, r, s in self.history]
