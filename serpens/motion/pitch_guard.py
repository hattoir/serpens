"""Floor Watch の頭（J7 = Design の J1 ピッチ）の動作範囲の強制（R-009）。

**既定はオフ**（`neck.floor_watch_enforce: false`）。HT-001（J1-0: 実機の符号の確認）が済むまで有効にしない。
範囲・速さは **PROVISIONAL**（`simulation/results/j1_range_opt_2026-09-30.md`。シミュレーション・prior・Design の見積もり。実機で未確認）。
`config/robot.yaml` の J7 の `min_deg` / `max_deg`（−8 / 90）は変えない。この範囲は **Floor Watch の頭（機械ストッパーが効く頭）に限る**追加の制限。

3 つを強制する（符号は + = 頭を上げる）:
  - **ストッパーの範囲** `floor_watch_stop_deg`（[−4°, +3°]）: 範囲外の**明示の角度指令（HEAD）は拒否**（NACK OUT_OF_RANGE。状態は変えない）。
    姿勢プリセット（POSE）・呼吸・内部の目標は**範囲へクランプ**する。どちらも `events` に記録し、ログへ出す。
  - **窓の端の手前の速さ** `floor_watch_near_limit_speed_dps`（40 °/s）: 作業窓 `floor_watch_soft_deg`（[−2°, +1°]）の外（= 窓の端からストッパーまでの区間）では、
    速さを上限で頭打ちにする。窓の内側から区間へ入るときは、窓の端で上限に一致する減速の傾き（`floor_watch_decel_dps2`）で速さを落とす。
  - 上の 2 つは**機体側（`serpens/link/device_motion.py`、ファーム `serpens_esp32.ino`）でも行う**（PC の設定では緩められない）。PC 側（`LinkClient.head`）は先に同じ検査をして、
    範囲外はクランプして記録する（機体の NACK を減らす）。

検証: このモジュールは SOFTWARE_VERIFIED（`tests/test_pitch_guard.py`）。ファームの C++ は写しで、`tools/build_firmware.py` でコンパイル成功（SOFTWARE_VERIFIED。ガード無効・有効の両方）。**書き込みはしていない。実機の動作は HARDWARE_UNVERIFIED**。
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Any

log = logging.getLogger("serpens.pitch_guard")


@dataclass(frozen=True)
class PitchGuardConfig:
    enabled: bool
    soft: tuple[float, float]
    stop: tuple[float, float]
    near_speed_dps: float
    decel_dps2: float

    @classmethod
    def from_cfg(cls, cfg: dict[str, Any], enabled: bool | None = None) -> "PitchGuardConfig | None":
        """`neck` に `floor_watch_*` が無ければ None（= ガードなし）。`enabled` を渡すと設定の値を上書きする（試験用）。"""
        n = cfg.get("neck") or {}
        if "floor_watch_stop_deg" not in n or "floor_watch_soft_deg" not in n:
            return None
        soft, stop = tuple(float(x) for x in n["floor_watch_soft_deg"]), tuple(float(x) for x in n["floor_watch_stop_deg"])
        if not (stop[0] <= soft[0] < soft[1] <= stop[1]):
            raise ValueError(f"floor_watch の範囲が矛盾: soft={soft} stop={stop}（stop ⊇ soft が要る）")
        return cls(bool(n.get("floor_watch_enforce", False)) if enabled is None else bool(enabled), soft, stop,
                   float(n.get("floor_watch_near_limit_speed_dps", 40.0)), float(n.get("floor_watch_decel_dps2", 1000.0)))


@dataclass
class Verdict:
    accepted: bool
    deg: float
    speed_dps: float
    reasons: list[str] = field(default_factory=list)


class PitchGuard:
    """J7 の範囲・速さの検査。純粋な計算（通信・時計を知らない）。無効のときは何も変えない。"""

    def __init__(self, gc: PitchGuardConfig | None) -> None:
        self.gc = gc
        self.events: list[dict[str, Any]] = []
        self._clamped_out = False           # 出力の連続クランプを 1 回だけ記録する

    @classmethod
    def from_cfg(cls, cfg: dict[str, Any], enabled: bool | None = None) -> "PitchGuard":
        return cls(PitchGuardConfig.from_cfg(cfg, enabled))

    @property
    def enabled(self) -> bool:
        return self.gc is not None and self.gc.enabled

    # ---- 記録 -----------------------------------------------------------------------
    def _log(self, kind: str, requested: float, applied: float, reason: str) -> None:
        ev = {"kind": kind, "requested": requested, "applied": applied, "reason": reason}
        self.events.append(ev)
        if len(self.events) > 512:
            del self.events[:256]
        log.warning("pitch guard %s: 要求 %.2f → 適用 %.2f（%s）", kind, requested, applied, reason)

    # ---- 範囲 -----------------------------------------------------------------------
    def in_stop(self, deg: float) -> bool:
        return not self.enabled or self.gc.stop[0] <= deg <= self.gc.stop[1]

    def in_band(self, deg: float) -> bool:
        """窓の端からストッパーまでの区間（速さを制限する区間）。"""
        if not self.enabled:
            return False
        return deg > self.gc.soft[1] or deg < self.gc.soft[0]

    def clamp_target(self, deg: float, why: str = "target") -> float:
        """姿勢プリセット・内部の目標: 範囲へクランプして記録する。"""
        if not self.enabled or self.in_stop(deg):
            return deg
        out = min(max(deg, self.gc.stop[0]), self.gc.stop[1])
        self._log("CLAMP", deg, out, f"{why}: ストッパーの範囲 {self.gc.stop} の外")
        return out

    def clamp_silent(self, deg: float) -> float:
        """記録しないクランプ（制御周期ごとの目標の最終確認。指令の段階で記録済み）。"""
        if not self.enabled:
            return deg
        return min(max(deg, self.gc.stop[0]), self.gc.stop[1])

    def clamp_output(self, deg: float) -> float:
        """サーボへ書く直前（呼吸を足した後）。連続して外れる間は 1 回だけ記録する。"""
        if not self.enabled or self.in_stop(deg):
            self._clamped_out = False
            return deg
        out = min(max(deg, self.gc.stop[0]), self.gc.stop[1])
        if not self._clamped_out:
            self._log("CLAMP", deg, out, "output: ストッパーの範囲の外（呼吸など）")
            self._clamped_out = True
        return out

    # ---- 速さ -----------------------------------------------------------------------
    def speed_limit(self, pos: float, target: float, speed: float) -> float:
        """いまの位置 `pos` から `target` へ向かうときの速さの上限（1 制御周期ごとに呼ぶ）。無効なら speed のまま。"""
        if not self.enabled or target == pos:
            return speed
        gc = self.gc
        if self.in_band(pos):
            return min(speed, gc.near_speed_dps)
        d = 1.0 if target > pos else -1.0
        edge = gc.soft[1] if d > 0 else gc.soft[0]
        if (d > 0 and target > edge) or (d < 0 and target < edge):         # 目標が区間の中 → 窓の端へ近づくほど減速
            dist = abs(edge - pos)
            return min(speed, math.sqrt(gc.near_speed_dps ** 2 + 2.0 * gc.decel_dps2 * dist))
        return speed

    # ---- HEAD 指令（明示の角度）---------------------------------------------------------
    def check_head_range(self, deg: float) -> bool:
        """機体側: 明示の角度指令が範囲内か。範囲外なら False（**拒否。状態を変えない**）で、記録する。"""
        if not self.enabled or self.in_stop(deg):
            return True
        self._log("REJECT", deg, deg, f"HEAD: ストッパーの範囲 {self.gc.stop} の外")
        return False

    def head_speed(self, deg: float, speed: float, current: float | None = None) -> float:
        """機体側: HEAD の速さを、窓の端の手前の上限で頭打ちにする（下げたら記録する）。`current` が分からないときは、目標が区間の中なら上限を掛ける。"""
        if not self.enabled:
            return speed
        cap = self.speed_limit(current, deg, speed) if current is not None else (self.gc.near_speed_dps if self.in_band(deg) else speed)
        if cap < speed:
            self._log("SPEED", speed, cap, f"HEAD: 窓の端の手前の速さの上限（現在 {current}、目標 {deg}）")
            return cap
        return speed

    def vet_head(self, deg: float, speed: float, current: float | None = None) -> Verdict:
        """機体側: 範囲外は拒否（accepted = False）。範囲内で速さが上限を超えるなら、上限へ下げて受け付ける（記録する）。"""
        if not self.check_head_range(deg):
            return Verdict(False, deg, speed, ["out_of_range"])
        v = self.head_speed(deg, speed, current)
        return Verdict(True, deg, v, ["speed_capped"] if v < speed else [])

    def prevet_head(self, deg: float, speed: float, current: float | None = None) -> Verdict:
        """PC 側: 範囲外はクランプして記録（機体の NACK を減らす）。速さは vet_head と同じ。機体側の検査は別に行われる（PC を信頼しない）。"""
        if not self.enabled:
            return Verdict(True, deg, speed)
        d = self.clamp_target(deg, "PC HEAD")
        v = self.vet_head(d, speed, current)
        return Verdict(v.accepted, d, v.speed_dps, v.reasons + (["clamped"] if d != deg else []))
