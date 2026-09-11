"""サーボバスの抽象インターフェース。

上位層（モーション・行動）は必ずこのクラス経由でサーボを操作する。
実装は MockServoBus（シミュレーション）と FeetechServoBus（実機）の2つで、
コマンドライン引数 `--bus mock|feetech` で差し替える（make_bus を参照）。

角度は「関節角」[deg] で扱う。0° = まっすぐ（ホーム姿勢）。
取り付け向き・組立オフセット・ステップ変換は各実装の内部で処理する。
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, NamedTuple


class ServoState(NamedTuple):
    """read_state() の戻り値。タプルとしても (pos, load, volt, temp) で展開できる。"""

    pos_deg: float   # 関節角 [deg]
    load: float      # 負荷率（1.0 = 100%）
    volt: float      # 電源電圧 [V]
    temp_c: float    # 内部温度 [℃]


@dataclass(frozen=True)
class Goal:
    """1軸分の目標指令。"""

    deg: float          # 目標関節角 [deg]
    speed_dps: float    # 最大速度 [deg/s]
    accel_dps2: float   # 加減速度 [deg/s²]


@dataclass(frozen=True)
class JointSpec:
    """設定ファイルから読んだ1関節分の仕様。"""

    name: str
    servo_id: int
    axis: str
    x_mm: float
    direction: int
    horn_offset_deg: float   # サーボホーン取付角のずれ（サーボ角 = direction×関節角 + これ）
    min_deg: float           # ソフトリミット
    max_deg: float
    mech_min_deg: float      # 機械リミット
    mech_max_deg: float
    max_speed_dps: float

    @staticmethod
    def from_cfg(d: dict[str, Any]) -> "JointSpec":
        """config の joints 要素から生成する。ソフトリミットが機械リミットの内側か確認する。"""
        j = JointSpec(
            name=str(d["name"]), servo_id=int(d["servo_id"]), axis=str(d["axis"]),
            x_mm=float(d["x_mm"]), direction=int(d["direction"]),
            horn_offset_deg=float(d["horn_offset_deg"]),
            min_deg=float(d["min_deg"]), max_deg=float(d["max_deg"]),
            mech_min_deg=float(d["mech_min_deg"]), mech_max_deg=float(d["mech_max_deg"]),
            max_speed_dps=float(d["max_speed_dps"]),
        )
        if not j.mech_min_deg <= j.min_deg < j.max_deg <= j.mech_max_deg:
            raise ValueError(f"{j.name}: ソフトリミットが機械リミットの外にあります")
        return j


class ServoCommError(RuntimeError):
    """サーボとの通信に失敗した。"""


class ServoBus(ABC):
    """サーボバスの抽象クラス。

    公開メソッド（set_goal など）はソフトウェアリミットでのクランプを共通で行い、
    実際の送信はサブクラスの `_xxx` メソッドが担当する（テンプレートメソッド）。
    """

    def __init__(self, cfg: dict[str, Any]) -> None:
        self.cfg = cfg
        self.joints: dict[int, JointSpec] = {
            j.servo_id: j for j in (JointSpec.from_cfg(d) for d in cfg["joints"])
        }

    # ---- 共通処理 ------------------------------------------------------------
    @property
    def ids(self) -> list[int]:
        """設定されているサーボ ID（関節順）。"""
        return list(self.joints.keys())

    def clamp_deg(self, servo_id: int, deg: float) -> float:
        """関節のソフトウェアリミットに収める。"""
        j = self.joints[servo_id]
        return min(max(deg, j.min_deg), j.max_deg)

    def set_goal(self, servo_id: int, deg: float, speed_dps: float, accel: float) -> None:
        """目標角を指令する。accel は加減速度 [deg/s²]。"""
        self._set_goals({servo_id: Goal(self.clamp_deg(servo_id, deg), speed_dps, accel)})

    def sync_set_goals(self, goals: dict[int, Goal]) -> None:
        """複数軸の目標を同時に指令する。"""
        clamped = {
            sid: Goal(self.clamp_deg(sid, g.deg), g.speed_dps, g.accel_dps2)
            for sid, g in goals.items()
        }
        self._set_goals(clamped)

    def read_state(self, servo_id: int) -> ServoState:
        """1軸の状態を読む。失敗したら ServoCommError。"""
        states = self._read_states([servo_id])
        if servo_id not in states:
            raise ServoCommError(f"サーボ ID {servo_id} の状態を読めませんでした")
        return states[servo_id]

    def sync_read_states(self, ids: list[int] | None = None) -> dict[int, ServoState]:
        """複数軸の状態を読む。読めなかった軸は結果に含まれない。"""
        return self._read_states(list(ids) if ids is not None else self.ids)

    def read_positions(self, ids: list[int] | None = None) -> dict[int, float]:
        """位置だけを読む [deg]。実装によってはこちらの方が速い。"""
        return {sid: st.pos_deg for sid, st in self.sync_read_states(ids).items()}

    @property
    def fast_reads(self) -> bool:
        """一括読み出しが使えるなら True（読み出し頻度の選択に使う）。"""
        return True

    # ---- 実装が必要なもの ----------------------------------------------------
    @abstractmethod
    def connect(self) -> None:
        """バスに接続する。"""

    @abstractmethod
    def disconnect(self) -> None:
        """バスから切断する（可能ならトルクを切ってから）。"""

    @abstractmethod
    def ping(self, servo_id: int) -> bool:
        """サーボが応答すれば True。"""

    @abstractmethod
    def set_torque(self, servo_id: int, on: bool) -> None:
        """トルクの ON/OFF。"""

    @abstractmethod
    def set_torque_limit(self, servo_id: int, ratio: float) -> None:
        """出力トルクの上限を 0.0〜1.0 で設定する（脱力演出用）。"""

    @abstractmethod
    def _set_goals(self, goals: dict[int, Goal]) -> None:
        """クランプ済みの目標を送信する。"""

    @abstractmethod
    def _read_states(self, ids: list[int]) -> dict[int, ServoState]:
        """状態を読み出す。"""


def make_bus(kind: str, cfg: dict[str, Any], port: str | None = None) -> ServoBus:
    """コマンドライン引数からバスを生成する。

    kind: "mock" または "feetech"。feetech のときは port（例: "COM5"）が必須。
    """
    if kind == "mock":
        from serpens.hw.mock_bus import MockServoBus

        return MockServoBus(cfg)
    if kind == "feetech":
        if not port:
            raise ValueError("--bus feetech には --port（例: COM5）が必要です")
        from serpens.hw.feetech_bus import FeetechServoBus

        return FeetechServoBus(cfg, port)
    raise ValueError(f"未知のバス種別: {kind}（mock / feetech）")
