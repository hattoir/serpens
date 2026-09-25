"""Serpens 側の Task / Event 端点。

Task を受けて検証し、受理 / 拒否を `task_status` で返す。実行そのものは `executor`（後のフェーズで
locomotion / behavior へ繋ぐ）に渡す。ここでは**安全の設定は一切受け取らない**（スキーマにフィールドが無い）。

規則:
  - stop は常に最優先。他の Task の実行中でも即座に受理し、executor.stop() を呼ぶ
  - 同じ id の再送は冪等（前回と同じ task_status を返す）
  - 版違い・スキーマ違反・frame / map_version 違い・実行できない理由（blockers）は rejected
  - Event には必ず data_source（HARDWARE / SIMULATION）を付ける
"""
from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Callable

from serpens.api.bridge import QOS, RETAINED_EVENTS, TOPIC_TASK, Broker, event_topic
from serpens.api.validate import validate_event, validate_task

API_VERSION = 1


class Executor:
    """Task を実際に行う側の最小契約（フェーズ 5 で locomotion / behavior に繋ぐ）。"""

    def blockers(self) -> list[str]:
        """いま Task を始められない理由（空なら開始できる）。安全ゲートや自己位置の欠如など。"""
        return []

    def start(self, task: dict[str, Any]) -> None:
        raise NotImplementedError

    def stop(self, reason: str) -> None:
        raise NotImplementedError


@dataclass
class Endpoint:
    broker: Broker
    executor: Executor
    map_version: str
    data_source: str = "SIMULATION"              # 実機のときだけ HARDWARE
    clock_ms: Callable[[], int] = lambda: int(time.time() * 1000)
    statuses: dict[str, dict[str, Any]] = field(default_factory=dict)   # task id → 最後の task_status
    active_task_id: str | None = None

    def __post_init__(self) -> None:
        self.broker.subscribe(TOPIC_TASK, self._on_task)

    # ---- 送信 -------------------------------------------------------------------
    def emit(self, event: str, **fields: Any) -> dict[str, Any]:
        msg = {"v": API_VERSION, "id": str(uuid.uuid4()), "t_ms": self.clock_ms(), "source": "serpens",
               "event": event, "data_source": self.data_source, **fields}
        errs = validate_event(msg)
        if errs:
            raise ValueError(f"送ろうとした Event がスキーマに合わない: {errs}")
        self.broker.publish(event_topic(event), msg, qos=QOS, retain=event in RETAINED_EVENTS)
        return msg

    def task_status(self, task_id: str, status: str, reason: str | None = None, progress: float | None = None) -> None:
        fields: dict[str, Any] = {"task_id": task_id, "status": status}
        if reason:
            fields["reason"] = reason
        if progress is not None:
            fields["progress"] = progress
        self.statuses[task_id] = self.emit("task_status", **fields)

    def safety_state(self, mode: str, stop_reason: str, latched: bool, telemetry_age_ms: int | None = None) -> None:
        fields: dict[str, Any] = {"mode": mode, "stop_reason": stop_reason, "latched": latched, "resume_requires": "operator"}
        if telemetry_age_ms is not None:
            fields["telemetry_age_ms"] = telemetry_age_ms
        self.emit("safety_state", **fields)

    def floor_finding(self, finding: dict[str, Any]) -> dict[str, Any]:
        return self.emit("floor_finding", finding=finding)

    def battery(self, voltage_v: float, low: bool, percent_est: float | None = None) -> None:
        fields: dict[str, Any] = {"voltage_v": voltage_v, "low": low}
        if percent_est is not None:
            fields["percent_est"] = percent_est
        self.emit("battery", **fields)

    # ---- 受信 -------------------------------------------------------------------
    def _on_task(self, _topic: str, msg: dict[str, Any]) -> None:
        errs = validate_task(msg)
        task_id = str(msg.get("id", "")) if isinstance(msg, dict) else ""
        if errs:
            if task_id:
                self.task_status(task_id, "rejected", f"schema: {errs[0]}")
            return
        if task_id in self.statuses:                       # 冪等: 同じ id には前回と同じ返事
            self.broker.publish(event_topic("task_status"), self.statuses[task_id], qos=QOS)
            return
        kind = msg["task"]
        if kind == "stop":
            self.executor.stop(msg.get("reason", "stop task"))
            self.active_task_id = None
            self.task_status(task_id, "accepted")
            return
        if "map_version" in msg and msg["map_version"] != self.map_version:
            self.task_status(task_id, "rejected", f"map_version: 受け側は {self.map_version}")
            return
        why = self.executor.blockers()
        if why:
            self.task_status(task_id, "rejected", "; ".join(why))
            return
        if self.active_task_id is not None:
            self.task_status(self.active_task_id, "aborted", f"新しい Task {task_id} に置き換え")
        self.active_task_id = task_id
        self.executor.start(msg)
        self.task_status(task_id, "accepted")
