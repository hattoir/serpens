"""Serpens 側の Task / Event 端点。

Task を受けて検証し、受理 / 拒否を `task_status` で返す。実行そのものは `executor`（後のフェーズで
locomotion / behavior へ繋ぐ）に渡す。ここでは**安全の設定は一切受け取らない**（スキーマにフィールドが無い）。

規則（docs/task_event_api.md）:
  - **stop は何より先に判定し、版違い・スキーマ違反・map_version 違いでも受理する**（止める方向は常に通す）
  - stop / FAULT / EMERGENCY で待ち行列を破棄し、人の操作（operator_resume）まで新しい Task を受けない
  - stop より前に発行された Task（t_ms が stop の t_ms 以前。QoS1 再送・再接続で後から届く）は rejected
  - Task トピックの retain は禁止: retain 付きで届いた Task は rejected
  - 同じ id の再送は冪等（前回と同じ task_status を返す）
  - Event には必ず data_source（HARDWARE / SIMULATION）。safety_state は retain、LWT で OFFLINE を retain
  - finding の resolved / dismissed は人の操作だけ（state_by=operator）。ロボットの再訪は reobservations に留める
"""
from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Callable

from serpens.api.bridge import QOS, RETAINED_EVENTS, TOPIC_TASK, Broker, event_topic
from serpens.api.validate import validate_event, validate_task

API_VERSION = 1
LOCKING_MODES = ("FAULT_HOLD", "EMERGENCY_LATCHED", "TORQUE_DISABLED", "OFFLINE")


class Executor:
    """Task を実際に行う側の最小契約（フェーズ 5 で locomotion / behavior に繋ぐ）。"""

    def blockers(self) -> list[str]:
        """いま Task を始められない理由（空なら開始できる）。安全ゲートや自己位置の欠如など。"""
        return []

    def start(self, task: dict[str, Any]) -> None:
        raise NotImplementedError

    def stop(self, reason: str) -> None:
        raise NotImplementedError


def finding_rule_errors(finding: dict[str, Any]) -> list[str]:
    """スキーマでは書けない finding の規則。resolved / dismissed は人だけ。"""
    out = []
    if finding.get("state") in ("resolved", "dismissed") and finding.get("state_by") != "operator":
        out.append("resolved / dismissed にできるのは人の操作だけ（state_by=operator）")
    if finding.get("state_by") == "robot" and finding.get("state") not in ("candidate", "confirmed"):
        out.append("ロボットが付けられる状態は candidate / confirmed だけ")
    return out


@dataclass
class Endpoint:
    broker: Broker
    executor: Executor
    map_version: str
    data_source: str = "SIMULATION"              # 実機のときだけ HARDWARE
    clock_ms: Callable[[], int] = lambda: int(time.time() * 1000)
    statuses: dict[str, dict[str, Any]] = field(default_factory=dict)   # task id → 最後の task_status
    active_task_id: str | None = None
    queue: list[dict[str, Any]] = field(default_factory=list)           # 受理済みで未開始の Task
    locked_reason: str | None = None            # stop / FAULT / EMERGENCY 後。人の操作で解く
    last_stop_t_ms: int | None = None           # これ以前に発行された Task は捨てる
    mode: str = "DISARMED"

    def __post_init__(self) -> None:
        self.broker.set_will(event_topic("safety_state"), self._offline_payload(), retain=True)
        self.broker.subscribe(TOPIC_TASK, self._on_task)

    # ---- 送信 -------------------------------------------------------------------
    def _base(self, event: str) -> dict[str, Any]:
        return {"v": API_VERSION, "id": str(uuid.uuid4()), "t_ms": self.clock_ms(), "source": "serpens",
                "event": event, "data_source": self.data_source}

    def _offline_payload(self) -> dict[str, Any]:
        """LWT: 接続が切れたらブローカーがこれを retain で出す（t_ms は接続時の値。受け側は古さで無効にする）。"""
        return {**self._base("safety_state"), "mode": "OFFLINE", "stop_reason": "UNKNOWN", "latched": True,
                "resume_requires": "operator"}

    def emit(self, event: str, **fields: Any) -> dict[str, Any]:
        msg = {**self._base(event), **fields}
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
        """機体の安全状態を外へ出す。FAULT / EMERGENCY なら待ち行列を破棄して人の操作までロック。"""
        self.mode = mode
        if mode in LOCKING_MODES:
            self._discard_queue(f"safety: {mode} ({stop_reason})")
            self.locked_reason = f"{mode}: {stop_reason}"
        fields: dict[str, Any] = {"mode": mode, "stop_reason": stop_reason, "latched": latched, "resume_requires": "operator"}
        if telemetry_age_ms is not None:
            fields["telemetry_age_ms"] = telemetry_age_ms
        self.emit("safety_state", **fields)

    def floor_finding(self, finding: dict[str, Any]) -> dict[str, Any]:
        errs = finding_rule_errors(finding)
        if errs:
            raise ValueError("; ".join(errs))
        return self.emit("floor_finding", finding=finding)

    def battery(self, voltage_v: float, low: bool, percent_est: float | None = None) -> None:
        fields: dict[str, Any] = {"voltage_v": voltage_v, "low": low}
        if percent_est is not None:
            fields["percent_est"] = percent_est
        self.emit("battery", **fields)

    def operator_resume(self) -> None:
        """人の操作。stop / FAULT 後のロックを解く（自動では解かない）。"""
        self.locked_reason = None

    # ---- 受信 -------------------------------------------------------------------
    def _discard_queue(self, reason: str) -> None:
        for t in self.queue:
            self.task_status(t["id"], "aborted", reason)
        self.queue.clear()
        if self.active_task_id is not None:
            self.task_status(self.active_task_id, "aborted", reason)
            self.active_task_id = None

    def _on_task(self, _topic: str, msg: Any, retained: bool) -> None:
        if not isinstance(msg, dict):
            return
        task_id = str(msg.get("id") or f"anon-{uuid.uuid4()}")
        if msg.get("task") == "stop":                      # 止める方向は何より先・何があっても受理
            self.executor.stop(str(msg.get("reason", "stop task")))
            self.last_stop_t_ms = max(self.last_stop_t_ms or 0, int(msg.get("t_ms", self.clock_ms()) or 0))
            self._discard_queue(f"stop {task_id}")
            self.locked_reason = f"stop {task_id}"
            self.task_status(task_id, "accepted")
            return
        if retained:                                       # Task トピックの retain は禁止
            self.task_status(task_id, "rejected", "retain 付きの Task は受け付けない")
            return
        errs = validate_task(msg)
        if errs:
            self.task_status(task_id, "rejected", f"schema: {errs[0]}")
            return
        if task_id in self.statuses:                       # 冪等: 同じ id には前回と同じ返事
            self.broker.publish(event_topic("task_status"), self.statuses[task_id], qos=QOS)
            return
        if self.last_stop_t_ms is not None and int(msg["t_ms"]) <= self.last_stop_t_ms:
            self.task_status(task_id, "rejected", "stop より前に発行された Task")
            return
        if self.locked_reason is not None:
            self.task_status(task_id, "rejected", f"人の操作で再開するまで受け付けない（{self.locked_reason}）")
            return
        if "map_version" in msg and msg["map_version"] != self.map_version:
            self.task_status(task_id, "rejected", f"map_version: 受け側は {self.map_version}")
            return
        why = self.executor.blockers()
        if why:
            self.task_status(task_id, "rejected", "; ".join(why))
            return
        if self.active_task_id is None:
            self.active_task_id = task_id
            self.executor.start(msg)
            self.task_status(task_id, "accepted")
        else:
            self.queue.append(msg)
            self.task_status(task_id, "accepted", "待ち行列")

    def task_finished(self, status: str = "done", reason: str | None = None) -> None:
        """executor から: 実行中の Task が終わった。待ち行列の次を始める。"""
        if self.active_task_id is not None:
            self.task_status(self.active_task_id, status, reason)
            self.active_task_id = None
        if self.queue and self.locked_reason is None:
            nxt = self.queue.pop(0)
            self.active_task_id = nxt["id"]
            self.executor.start(nxt)
            self.task_status(nxt["id"], "running")
