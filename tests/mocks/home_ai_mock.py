"""Home AI 側のモック（**Home AI 本体ではない**）。Task を発行し、Event を受けて記録するだけ。

MVP の通知は「受け取ってログに出す Webhook」相当。ここでは `notified` に溜める。
"""
from __future__ import annotations

import uuid
from dataclasses import dataclass, field
from typing import Any

from serpens.api.bridge import QOS, TOPIC_EVENT, TOPIC_TASK, Broker
from serpens.api.validate import validate_event


@dataclass
class HomeAiMock:
    broker: Broker
    map_version: str = "tags-v0"
    events: list[dict[str, Any]] = field(default_factory=list)
    notified: list[str] = field(default_factory=list)      # Webhook の代わり（ログ）
    schema_errors: list[str] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.broker.subscribe(f"{TOPIC_EVENT}/#", self._on_event)

    def _on_event(self, _topic: str, msg: dict[str, Any]) -> None:
        errs = validate_event(msg)
        if errs:
            self.schema_errors += errs
            return
        self.events.append(msg)
        if msg["event"] == "floor_finding" and msg["finding"]["risk"]["score"] >= 0.5:
            f = msg["finding"]
            self.notified.append(f"floor_finding {f['finding_id']} risk={f['risk']['score']:.2f} at "
                                 f"({f['pose']['x_m']:.2f},{f['pose']['y_m']:.2f}) ±{f['pose']['sigma_xy_m']:.2f}m")

    def task(self, kind: str, task_id: str | None = None, **fields: Any) -> str:
        msg: dict[str, Any] = {"v": 1, "id": task_id or str(uuid.uuid4()), "t_ms": 0, "source": "home_ai", "task": kind}
        if kind in ("inspect_point", "patrol_route", "highlight_point"):
            msg["frame_id"], msg["map_version"] = "home", self.map_version
        msg.update(fields)
        self.broker.publish(TOPIC_TASK, msg, qos=QOS)
        return msg["id"]

    def statuses_of(self, task_id: str) -> list[str]:
        return [e["status"] for e in self.events if e["event"] == "task_status" and e["task_id"] == task_id]
