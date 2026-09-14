"""ロボットへの出力先（実機・モック・駆動リンク）を1つの形にまとめる継ぎ目。

セッションは「どこへ出しているか」を知らずに、毎周期 `MotionCommand` を渡す。

  - `DirectRobot` … サーボバスへ9軸の角度を直接書く（モック / 実機の TTL バス）
  - `LinkRobot`（`serpens/link/robot.py`）… ESP32 へ**歩容パラメータ**を送り、角度は機体が作る

**この差は安全性の差でもある。** 直接経路は PC が死ねばサーボが最後の指令を保持し続けるが、
リンク経路は機体が自分で止まる（`docs/link_protocol.md`）。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol, runtime_checkable

from serpens.hw.servo_bus import Goal, ServoBus, ServoCommError
from serpens.hw.state_poller import ServoStatePoller
from serpens.motion.animator import Animator
from serpens.motion.gait import GaitParams, body_joint_names
from serpens.safety import DriveState

Pose = dict[str, float]


@dataclass(frozen=True)
class MotionCommand:
    """1周期ぶんの出力。**角度とパラメータの両方**を持つ。

    直接経路は `angles` をそのまま書き、リンク経路は `gait`（胴体）と `head`（首・頭）を送る。
    `angles` は世界の更新と画面表示にも使うので、どちらの経路でも埋める。
    """

    angles: Pose                     # 9軸の合成角（ベース + 歩容 + 呼吸）
    gait: GaitParams | None          # 胴体の歩容。None = 歩容していない
    gamma0_deg: float                # 旋回オフセット
    body_base: Pose                  # 歩容を除いた胴体のベース角（とぐろ・鎌首）
    head: Pose                       # 首・頭（胴体ヨーより先）の角
    breathing: bool

    @staticmethod
    def from_animator(anim: Animator, body: list[str], t: float) -> "MotionCommand":
        """アニメータを1周期進めて指令を作る（`anim.update` はここでだけ呼ぶ）。"""
        angles = anim.update(t)
        g = anim.gait
        active = g.active and g.params is not None
        return MotionCommand(
            angles=angles,
            gait=g.params if active else None,
            gamma0_deg=g.gamma0,
            body_base={n: v for n, v in anim.last_base.items() if n in body},
            head={n: v for n, v in anim.last_base.items() if n not in body},
            breathing=anim.breathing,
        )


@dataclass(frozen=True)
class DeviceStatus:
    """機体（ESP32）の状態。**直接経路には無い**ので None になる。"""

    state_ja: str
    reason_ja: str
    simulated: bool             # True = 値がシミュレーション由来
    age_s: float | None         # テレメトリの古さ
    loop_period_us: int
    overruns: int
    missing_axes: int


@runtime_checkable
class TorqueSink(Protocol):
    """トルク比を受け取れるもの（脱力の演出が使う）。ServoBus も LinkRobot も満たす。"""

    @property
    def ids(self) -> list[int]: ...

    def set_torque_limit(self, servo_id: int, ratio: float) -> None: ...


class RobotInterface(Protocol):
    """セッションから見たロボット。"""

    def send(self, t: float, cmd: MotionCommand) -> None:
        """走行中の出力。"""

    def hold(self, pose: Pose) -> None:
        """停止中の保持指令（現在姿勢を保つ）。"""

    def apply_stop_state(self, state: Any) -> None:
        """停止状態（`serpens.safety.DriveState`）を出力へ反映する。経路ごとに出し方が違う。"""

    def on_operator_start(self) -> None:
        """**人が開始操作をした**ときだけ呼ばれる（周期処理からは呼ばない）。

        機体が再起動したあとの走行再開は、この操作を通してのみ許す。
        """

    def on_clear_emergency(self) -> None:
        """人が緊急停止を解除した。機体側のラッチも解く（**待機へ戻すだけ**）。"""

    def poll(self) -> None:
        """状態の読み出し（停止中も続ける）。"""

    def device_status(self) -> "DeviceStatus | None":
        """機体側の状態（駆動リンクがあるときだけ）。"""

    def close(self, torque_off: bool) -> list[str]:
        """出力を切る。失敗は文字列で返す（握りつぶさない）。"""


class DirectRobot:
    """サーボバスへ角度を直接書く経路（従来どおり）。

    **PC が落ちるとサーボは最後の指令角を保持し続ける。** 機体側の watchdog は無い。
    """

    def __init__(self, cfg: dict[str, Any], bus: ServoBus, clock: Any = None) -> None:
        self.cfg = cfg
        self.bus = bus
        self.body = body_joint_names(cfg)
        self.telemetry = ServoStatePoller(bus, cfg, clock)
        self._ids = {j["name"]: int(j["servo_id"]) for j in cfg["joints"]}
        self._vmax = {j["name"]: float(j["max_speed_dps"]) for j in cfg["joints"]}
        self._accel = float(cfg["animator"]["default_accel_dps2"])
        self.errors: list[str] = []
        self._torque_off = False
        failed = bus.apply_torque_ceiling()          # 安全上限を必ず通す（構想設計書 16章）
        if failed:
            self.errors.append(f"トルク上限を設定できなかった軸: {failed}")

    # ---- 出力 -----------------------------------------------------------------------
    def send(self, t: float, cmd: MotionCommand) -> None:
        """9軸の角度をそのまま書く。"""
        self._write(cmd.angles)

    def hold(self, pose: Pose) -> None:
        self._write(pose)

    def _write(self, pose: Pose) -> None:
        goals = {self._ids[n]: Goal(v, self._vmax[n], self._accel)
                 for n, v in pose.items() if n in self._ids}
        self._guard(lambda: self.bus.sync_set_goals(goals), "指令の送信")

    def apply_stop_state(self, state: Any) -> None:
        """直接経路では「脱力するかどうか」だけ。保持は `hold()` が毎周期送る。"""
        self.set_torque(state is not DriveState.DISABLED)

    def on_operator_start(self) -> None:
        """直接経路では何もしない（機体側に状態が無い）。"""

    def on_clear_emergency(self) -> None:
        """直接経路では何もしない（ラッチは PC 側の StopSupervisor だけが持つ）。"""

    def set_torque(self, on: bool) -> None:
        if self._torque_off == (not on):
            return
        for sid in self.bus.ids:
            self._guard(lambda sid=sid: self.bus.set_torque(sid, on), f"トルク{'ON' if on else 'OFF'}（ID{sid}）")
        self._torque_off = not on

    def poll(self) -> None:
        self.telemetry.poll()

    def close(self, torque_off: bool) -> list[str]:
        self._guard(lambda: self.bus.disconnect(torque_off=torque_off), "サーボバスの切断")
        return list(self.errors)

    # ---- 状態 -----------------------------------------------------------------------
    @property
    def torque_sink(self) -> TorqueSink:
        """脱力演出の宛先（直接経路ではバスそのもの）。"""
        return self.bus

    @property
    def torque_ceiling_ok(self) -> bool:
        """安全上限を全軸へ書けたか（書けていなければ実機の自律走行を止める）。"""
        return self.bus.torque_ceiling_applied

    @property
    def link_ok(self) -> bool:
        """駆動リンク（機体側 watchdog）の有無。**直接経路には無い。**"""
        return False

    def positions_by_name(self) -> Pose:
        """関節名 → 角度（読めているものだけ）。"""
        pos = self.telemetry.positions
        return {n: pos[i] for n, i in self._ids.items() if i in pos}

    def device_status(self) -> DeviceStatus | None:
        """直接経路に機体側の状態は無い（サーボへ直接書いているだけ）。"""
        return None

    def _guard(self, fn: Any, what: str) -> bool:
        try:
            fn()
            return True
        except (ServoCommError, OSError) as e:
            msg = f"{what}に失敗: {e}"
            if msg not in self.errors:
                self.errors.append(msg)
            return False
