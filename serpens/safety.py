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

from serpens.electrical_gate import electrical_gate_blockers

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
    pose_source: str                  # "apriltag" / "odometry_imu" / "aruco"（検証用）/ "sim" / "none"
    pose_age_s: float | None
    calibration_present: bool
    drive_link_ok: bool
    torque_ceiling_ok: bool           # 安全上限（トルク制限）を全軸へ書けたか
    telemetry_axes: int
    expected_axes: int
    telemetry_age_s: float | None
    # フェーズ 3: σ と局所センサーの健全性（require_sigma の出どころでは必須。None = 報告なし）
    sigma_xy_m: float | None = None
    sigma_yaw_rad: float | None = None
    imu_ok: bool | None = None
    odometry_ok: bool | None = None


def _pose_blockers(cfg: dict[str, Any], a: dict[str, Any], inputs: AutonomyInputs) -> list[str]:
    """自己位置の開始条件。出どころが実観測で、新しく、（要る出どころでは）σ と IMU / オドメトリが健全。"""
    sources = a["pose_sources"]
    if inputs.pose_source not in sources:
        return [f"自己位置が実観測ではない（現在: {inputs.pose_source}）。"
                f"認める出どころ: {', '.join(sources)}。実観測の位置が制御へ入るまで実機の自律走行は禁止"]
    if inputs.pose_age_s is None:
        return ["自己位置が未取得"]
    out = []
    if inputs.pose_age_s > float(a["pose_max_age_s"]):
        out.append(f"自己位置が古い（{inputs.pose_age_s:.1f}s > {a['pose_max_age_s']}s）")
    if not bool(sources[inputs.pose_source]["require_sigma"]):
        return out
    st = cfg["localization"]["start"]
    if inputs.sigma_xy_m is None or inputs.sigma_yaw_rad is None:
        out.append("自己位置の不確かさ（σ）が報告されていない")
    else:
        if inputs.sigma_xy_m > float(st["max_sigma_xy_m"]):
            out.append(f"自己位置の不確かさが大きい（σ_xy {inputs.sigma_xy_m:.2f}m > {st['max_sigma_xy_m']}m）")
        if inputs.sigma_yaw_rad > float(st["max_sigma_yaw_rad"]):
            out.append(f"向きの不確かさが大きい（σ_yaw {inputs.sigma_yaw_rad:.2f}rad > {st['max_sigma_yaw_rad']}rad）")
    if bool(st["require_imu"]) and not inputs.imu_ok:
        out.append("IMU が健全ではない（未受信か古い）")
    if not inputs.odometry_ok:
        out.append("オドメトリ（歩容の位相）が更新されていない")
    return out


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
        out.extend(_pose_blockers(cfg, a, inputs))
    if bool(a["require_calibration"]) and not inputs.calibration_present:
        out.append(f"床の校正が無い（{cfg['homography']['file']}。tools/calibrate_floor.py で作成）")
    if bool(a["require_drive_link"]) and not inputs.drive_link_ok:
        out.append("駆動リンク（ESP32 heartbeat）が無い。Phase 2 まで実機の自律走行は禁止")
    if not inputs.torque_ceiling_ok:
        ratio = cfg["safety_limits"]["torque"]["software_torque_limit_ratio"]
        out.append(f"トルク上限を全軸へ設定できていない（ストール比 {ratio}）")
    out.extend(electrical_gate_blockers(cfg))            # 実測が無い間は必ず止まる
    if inputs.telemetry_axes < inputs.expected_axes:
        out.append(f"テレメトリ不足: {inputs.telemetry_axes}/{inputs.expected_axes} 軸しか読めていない")
    elif inputs.telemetry_age_s is not None and inputs.telemetry_age_s > float(t["stale_after_s"]):
        out.append(f"テレメトリが古い（{inputs.telemetry_age_s:.1f}s > {t['stale_after_s']}s）")
    return out
