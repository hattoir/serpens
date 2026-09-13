"""停止の種類と、自律走行の開始条件（Phase 1）。

停止を3つに分ける。混ぜると「止めたつもりで動く」「復旧のつもりで暴れる」が起きる。

| 種類 | 意味 | 出力 | 解除 |
|---|---|---|---|
| RUN      | 走行中 | 行動が決めた指令を送る | — |
| HOLD     | **通常停止**。歩容を即時に止め、**現在の指令角を保持**（トルクは入れたまま） | 保持指令を送り続ける | 開始操作（start） |
| DISABLED | **駆動無効化**。トルクを切って脱力させる（明示操作のみ。実機未検証） | トルクOFF | 開始操作（start） |
| EMERGENCY| **緊急停止**。ラッチする。通常操作・予約済みの演出では解除できない | HOLD と同じ（config で DISABLED も選べる。実機未検証） | clear_emergency（明示解除）→ HOLD |

ホーム姿勢へ動かす・全軸を脱力する、を停止の既定動作にはしない（停止中に動くのが最も危険）。
解除しても走行は再開しない。必ず待機（HOLD）へ戻し、別の開始操作を待つ。

**PC 側だけでは、PC 停止・USB 断のときに機体を止められない。**
`serpens/link` と ESP32 側の watchdog（heartbeat 途絶で機体単独停止）が Phase 2 の前提。
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable

Clock = Callable[[], float]


class DriveState(Enum):
    """駆動の状態。"""

    RUN = "RUN"
    HOLD = "HOLD"
    DISABLED = "DISABLED"
    EMERGENCY = "EMERGENCY"

    @property
    def moving_allowed(self) -> bool:
        return self is DriveState.RUN

    @property
    def label_ja(self) -> str:
        return {"RUN": "走行", "HOLD": "停止（姿勢保持）", "DISABLED": "駆動無効（脱力）",
                "EMERGENCY": "緊急停止（ラッチ）"}[self.value]


@dataclass
class StopEvent:
    """停止・解除の記録（Phase 8 のログへそのまま渡せる形）。"""

    t: float
    state: str
    reason: str
    source: str


class StopSupervisor:
    """停止要求を持ち、出力の可否を決める。ラッチの解除はここだけが行う。"""

    def __init__(self, cfg: dict[str, Any], clock: Clock | None = None, initial: DriveState = DriveState.HOLD) -> None:
        s = cfg["behavior"]["safety"]["stop"]
        self._clock: Clock = clock or time.monotonic
        self.emergency_action = DriveState(str(s["emergency_action"]).upper())
        if self.emergency_action not in (DriveState.HOLD, DriveState.DISABLED):
            raise ValueError("safety.stop.emergency_action は hold か disable_torque")
        self.hold_resend_period_s = 1.0 / float(s["hold_resend_hz"])
        self.disable_torque_on_exit = bool(s["disable_torque_on_exit"])
        self.state = initial
        self.reason = "起動直後は待機" if initial is not DriveState.RUN else ""
        self.source = "起動"
        self.latched = False
        self.events: list[StopEvent] = [StopEvent(self._clock(), self.state.value, self.reason, self.source)]

    # ---- 問い合わせ -------------------------------------------------------------
    @property
    def moving_allowed(self) -> bool:
        """行動が決めた移動指令を出してよいか。"""
        return self.state.moving_allowed and not self.latched

    @property
    def torque_should_be_off(self) -> bool:
        """トルクを切るべき状態か。"""
        return self.state is DriveState.DISABLED

    def status_text(self) -> str:
        latch = "（ラッチ中）" if self.latched else ""
        return f"{self.state.label_ja}{latch}" + (f": {self.reason}" if self.reason else "")

    # ---- 要求 -------------------------------------------------------------------
    def stop(self, reason: str, source: str = "操作") -> bool:
        """通常停止（姿勢保持）。緊急停止中は何もしない。"""
        if self.latched:
            return False
        return self._set(DriveState.HOLD, reason, source)

    def disable_torque(self, reason: str, source: str = "操作") -> bool:
        """駆動無効化（脱力）。走行中からは直接行わず、停止してから明示操作で行う。"""
        if self.latched:
            return False
        if self.state is DriveState.RUN:
            return False
        return self._set(DriveState.DISABLED, reason, source)

    def emergency(self, reason: str, source: str = "操作") -> bool:
        """緊急停止。ラッチする。"""
        self.latched = True
        return self._set(DriveState.EMERGENCY, reason, source)

    def clear_emergency(self, source: str = "操作") -> bool:
        """緊急停止の解除。走行は再開せず、待機（HOLD）へ戻す。"""
        if not self.latched:
            return False
        self.latched = False
        return self._set(DriveState.HOLD, "緊急停止を解除（待機）", source)

    def start(self, blockers: list[str] | None = None, source: str = "操作") -> tuple[bool, list[str]]:
        """走行を開始する。ラッチ中、または blockers があれば開始しない。"""
        if self.latched:
            return False, ["緊急停止がラッチ中（解除操作が必要）"]
        if blockers:
            return False, list(blockers)
        return self._set(DriveState.RUN, "", source), []

    def _set(self, state: DriveState, reason: str, source: str) -> bool:
        changed = state is not self.state or reason != self.reason
        self.state, self.reason, self.source = state, reason, source
        if changed:
            self.events.append(StopEvent(self._clock(), state.value, reason, source))
        return True


# =============================================================================
# 自律走行の開始条件（実機では実観測が揃うまで走らせない）
# =============================================================================
@dataclass(frozen=True)
class AutonomyInputs:
    """開始条件の判定に使う事実だけを集めたもの。"""

    robot_is_real: bool
    pose_source: str                  # "aruco" / "sim" / "none"
    pose_age_s: float | None
    calibration_present: bool
    drive_link_ok: bool
    torque_ceiling_ok: bool           # 安全上限（トルク制限）を全軸へ書けたか
    telemetry_axes: int
    expected_axes: int
    telemetry_age_s: float | None


def autonomy_blockers(cfg: dict[str, Any], inputs: AutonomyInputs) -> list[str]:
    """自律走行を許可できない理由を並べる（空なら開始してよい）。

    シミュレーションのデモは従来どおり動かせる。実機のときだけ厳しくする。
    """
    a = cfg["behavior"]["safety"]["autonomy"]
    t = cfg["behavior"]["safety"]["telemetry"]
    out: list[str] = []
    if not inputs.robot_is_real:
        return out
    if bool(a["require_real_pose"]):
        if inputs.pose_source != "aruco":
            out.append(f"自己位置が実観測ではない（現在: {inputs.pose_source}）。"
                       "ArUco からの位置が制御へ入るまで実機の自律走行は禁止（Phase 4）")
        elif inputs.pose_age_s is None:
            out.append("自己位置が未取得")
        elif inputs.pose_age_s > float(a["pose_max_age_s"]):
            out.append(f"自己位置が古い（{inputs.pose_age_s:.1f}s > {a['pose_max_age_s']}s）")
    if bool(a["require_calibration"]) and not inputs.calibration_present:
        out.append(f"床の校正が無い（{cfg['homography']['file']}。tools/calibrate_floor.py で作成）")
    if bool(a["require_drive_link"]) and not inputs.drive_link_ok:
        out.append("駆動リンク（ESP32 heartbeat）が無い。Phase 2 まで実機の自律走行は禁止")
    if not inputs.torque_ceiling_ok:
        out.append("トルク上限を全軸へ設定できていない"
                   f"（safety_limits.torque_ratio_max = {cfg['safety_limits']['torque_ratio_max']}）")
    if inputs.telemetry_axes < inputs.expected_axes:
        out.append(f"テレメトリ不足: {inputs.telemetry_axes}/{inputs.expected_axes} 軸しか読めていない")
    elif inputs.telemetry_age_s is not None and inputs.telemetry_age_s > float(t["stale_after_s"]):
        out.append(f"テレメトリが古い（{inputs.telemetry_age_s:.1f}s > {t['stale_after_s']}s）")
    return out
