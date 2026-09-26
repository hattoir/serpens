"""Home AI 側のモック（**Home AI 本体ではない**）。Task を発行し、Event を受けて記録するだけ。

MVP の通知は「受け取ってログに出す Webhook」相当。ここでは `notified` に溜める。
受け側の規則:
  - safety_state の鮮度は**受け側の時計**で「最後に届いてからの時間」で判定する（送り側の t_ms は時計ずれがあるので
    使わない）。api.safety_state_max_age_ms を超えたら `safety_fresh()` が False（= 状態不明。停止扱い）。
    Serpens は変化が無くても周期的に出すので、届かない = 通信か機体の異常
  - 購読時に retain で届いた値は「最後に知られた状態」。`retained` に印を付け、周期送信が届くまで鮮度は保証しない
  - floor_finding は risk.score ≥ 0.5 **または** risk.mandatory_notify で通知（ボタン電池・磁石・薬・metal_disc は
    確信度が低くても）
"""
from __future__ import annotations

import uuid
from dataclasses import dataclass, field
from typing import Any, Callable

from serpens.api.bridge import QOS, TOPIC_EVENT, TOPIC_TASK, Broker
from serpens.api.validate import validate_event
from serpens.config import load_config

NOTIFY_SCORE = 0.5
SAFETY_MAX_AGE_MS = int(load_config()["api"]["safety_state_max_age_ms"])


@dataclass
class HomeAiMock:
    broker: Broker
    map_version: str = "tags-v0"
    clock_ms: Callable[[], int] = lambda: 0
    events: list[dict[str, Any]] = field(default_factory=list)
    notified: list[str] = field(default_factory=list)      # Webhook の代わり（ログ）
    schema_errors: list[str] = field(default_factory=list)
    safety: dict[str, Any] | None = None                    # 最後に届いた safety_state
    safety_received_ms: int | None = None                   # 受け側の時計で届いた時刻
    safety_retained: bool = False                           # 購読時の retain 分か（鮮度は保証しない）
    safety_count: int = 0

    def __post_init__(self) -> None:
        self.broker.subscribe(f"{TOPIC_EVENT}/#", self._on_event)

    def _on_event(self, _topic: str, msg: dict[str, Any], retained: bool) -> None:
        errs = validate_event(msg)
        if errs:
            self.schema_errors += errs
            return
        if msg["event"] == "safety_state":
            self.safety, self.safety_retained, self.safety_count = msg, retained, self.safety_count + 1
            self.safety_received_ms = None if retained else self.clock_ms()
        self.events.append(msg)
        if msg["event"] == "floor_finding":
            f = msg["finding"]
            r = f["risk"]
            if r["score"] >= NOTIFY_SCORE or r["mandatory_notify"]:
                why = "mandatory:" + ",".join(r["critical_kinds"]) if r["mandatory_notify"] else f"risk={r['score']:.2f}"
                self.notified.append(f"floor_finding {f['finding_id']} {why} at "
                                     f"({f['pose']['x_m']:.2f},{f['pose']['y_m']:.2f}) ±{f['pose']['sigma_xy_m']:.2f}m")

    def safety_fresh(self) -> bool:
        """受け側の時計で max_age 以内に（retain ではなく）生で届いているか。False なら状態不明 = 停止扱い。"""
        return self.safety_received_ms is not None and self.clock_ms() - self.safety_received_ms <= SAFETY_MAX_AGE_MS

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
