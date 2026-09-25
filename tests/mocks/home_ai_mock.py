"""Home AI 側のモック（**Home AI 本体ではない**）。Task を発行し、Event を受けて記録するだけ。

MVP の通知は「受け取ってログに出す Webhook」相当。ここでは `notified` に溜める。
受け側の規則:
  - safety_state は t_ms が古ければ無効（`stale`）。retain で残った古い状態を今の状態と取り違えない
  - floor_finding は risk.score ≥ 0.5 **または** risk.mandatory_notify で通知（ボタン電池・磁石・薬は確信度が低くても）
"""
from __future__ import annotations

import uuid
from dataclasses import dataclass, field
from typing import Any, Callable

from serpens.api.bridge import QOS, TOPIC_EVENT, TOPIC_TASK, Broker
from serpens.api.validate import validate_event

NOTIFY_SCORE = 0.5
SAFETY_MAX_AGE_MS = 5000


@dataclass
class HomeAiMock:
    broker: Broker
    map_version: str = "tags-v0"
    clock_ms: Callable[[], int] = lambda: 0
    events: list[dict[str, Any]] = field(default_factory=list)
    notified: list[str] = field(default_factory=list)      # Webhook の代わり（ログ）
    schema_errors: list[str] = field(default_factory=list)
    stale: list[dict[str, Any]] = field(default_factory=list)   # 古い safety_state（無効扱い）
    safety: dict[str, Any] | None = None                    # 有効な最新の safety_state

    def __post_init__(self) -> None:
        self.broker.subscribe(f"{TOPIC_EVENT}/#", self._on_event)

    def _on_event(self, _topic: str, msg: dict[str, Any], retained: bool) -> None:
        errs = validate_event(msg)
        if errs:
            self.schema_errors += errs
            return
        if msg["event"] == "safety_state":
            if self.clock_ms() - int(msg["t_ms"]) > SAFETY_MAX_AGE_MS:
                self.stale.append(msg)                       # retain で残った古い状態
                return
            self.safety = msg
        self.events.append(msg)
        if msg["event"] == "floor_finding":
            f = msg["finding"]
            r = f["risk"]
            if r["score"] >= NOTIFY_SCORE or r["mandatory_notify"]:
                why = "mandatory:" + ",".join(r["critical_kinds"]) if r["mandatory_notify"] else f"risk={r['score']:.2f}"
                self.notified.append(f"floor_finding {f['finding_id']} {why} at "
                                     f"({f['pose']['x_m']:.2f},{f['pose']['y_m']:.2f}) ±{f['pose']['sigma_xy_m']:.2f}m")

    def task(self, kind: str, task_id: str | None = None, t_ms: int | None = None, retain: bool = False,
             **fields: Any) -> str:
        msg: dict[str, Any] = {"v": 1, "id": task_id or str(uuid.uuid4()),
                               "t_ms": self.clock_ms() if t_ms is None else t_ms, "source": "home_ai", "task": kind}
        if kind in ("inspect_point", "patrol_route", "highlight_point"):
            msg["frame_id"], msg["map_version"] = "home", self.map_version
        msg.update(fields)
        self.broker.publish(TOPIC_TASK, msg, qos=QOS, retain=retain)
        return msg["id"]

    def statuses_of(self, task_id: str) -> list[str]:
        return [e["status"] for e in self.events if e["event"] == "task_status" and e["task_id"] == task_id]

    def last_reason(self, task_id: str) -> str:
        return [e for e in self.events if e["event"] == "task_status" and e["task_id"] == task_id][-1].get("reason", "")
