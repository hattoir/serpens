"""ELECTRICAL_SAFETY_GATE — 電気の安全が実測で確かめられるまで、実機の自律走行を許さない。

項目は `config/robot.yaml` の `safety_limits.electrical_safety_gate`。
各項目は COMPLETE と**証拠**（日付・条件・測定器を書いた文字列）の両方がそろって初めて通る。
シミュレーションの結果では埋めない（docs/verification_status.md）。

このゲートは**自律走行**を止める。人が見ながら 1 軸ずつ試す初回の計測（電流を測るための通電）までは
止めない — 計測できなければゲートを埋められないため。ただし実サーボの操作そのものは
このリポジトリの作業範囲ではまだ禁止されている（agent/STATE.md）。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

COMPLETE = "COMPLETE"
INCOMPLETE = "INCOMPLETE"
STATUSES = (COMPLETE, INCOMPLETE)

# 構想設計書 16章の「層2（電気的制限）」と、実機の前に確かめる項目。減らすときは理由を DECISIONS に書く
REQUIRED_ITEMS = (
    "real_current", "power_capacity", "overcurrent_protection", "wiring_heat",
    "independent_power_cut", "physical_estop", "servo_temperature", "real_stop_time",
)


@dataclass(frozen=True)
class GateItem:
    """ゲートの 1 項目。"""

    key: str
    status: str
    evidence: str | None
    what: str

    @property
    def passed(self) -> bool:
        return self.status == COMPLETE and bool(self.evidence and self.evidence.strip())


def gate_items(cfg: dict[str, Any]) -> list[GateItem]:
    """config から項目を読む。欠けた項目・未知の状態は例外（黙って通さない）。"""
    raw = cfg["safety_limits"]["electrical_safety_gate"]
    missing = [k for k in REQUIRED_ITEMS if k not in raw]
    if missing:
        raise ValueError(f"electrical_safety_gate に項目が無い: {missing}")
    out = []
    for key in REQUIRED_ITEMS:
        v = raw[key]
        if v["status"] not in STATUSES:
            raise ValueError(f"electrical_safety_gate.{key}: 未知の状態 {v['status']!r}")
        out.append(GateItem(key, str(v["status"]), v.get("evidence"), str(v.get("what", ""))))
    return out


def electrical_gate_blockers(cfg: dict[str, Any]) -> list[str]:
    """通っていない項目を言葉で返す（空ならゲートは開いている）。"""
    out = []
    for item in gate_items(cfg):
        if item.passed:
            continue
        why = "証拠が無い" if item.status == COMPLETE else "未実測"
        out.append(f"電気安全ゲート未完了: {item.key}（{item.what}）— {why}")
    return out


def gate_open(cfg: dict[str, Any]) -> bool:
    """全項目が実測の証拠つきで COMPLETE か。"""
    return not electrical_gate_blockers(cfg)
