"""Home AI ↔ Serpens の境界（MQTT）。**リアルタイムの関節制御には使わない**（決定事項 3）。

トピック（docs/task_event_api.md）:
    home/serpens/task              Home AI → Serpens（QoS 1）
    home/serpens/event/<event>     Serpens → Home AI（QoS 1。safety_state は retain）

`Broker` は publish / subscribe だけの薄い抽象。`LoopbackBroker` はプロセス内で配る（テスト・モック用）。
`PahoBroker` は paho-mqtt が入っていれば使える（依存には入れていない。無ければ ImportError を出して止まる）。
"""
from __future__ import annotations

import json
from collections import defaultdict
from dataclasses import dataclass
from typing import Any, Callable

TOPIC_TASK = "home/serpens/task"
TOPIC_EVENT = "home/serpens/event"
RETAINED_EVENTS = ("safety_state",)
QOS = 1

Handler = Callable[[str, dict[str, Any]], None]


def event_topic(event: str) -> str:
    return f"{TOPIC_EVENT}/{event}"


@dataclass(frozen=True)
class Published:
    topic: str
    payload: dict[str, Any]
    qos: int
    retain: bool


class Broker:
    """publish / subscribe の抽象。"""

    def publish(self, topic: str, payload: dict[str, Any], qos: int = QOS, retain: bool = False) -> None:
        raise NotImplementedError

    def subscribe(self, topic: str, handler: Handler) -> None:
        raise NotImplementedError


class LoopbackBroker(Broker):
    """プロセス内のブローカー。配信は同期（テストで順序が読める）。retain も模す。"""

    def __init__(self) -> None:
        self._subs: dict[str, list[Handler]] = defaultdict(list)
        self._retained: dict[str, dict[str, Any]] = {}
        self.log: list[Published] = []

    @staticmethod
    def _match(pattern: str, topic: str) -> bool:
        p, t = pattern.split("/"), topic.split("/")
        for i, seg in enumerate(p):
            if seg == "#":
                return True
            if i >= len(t) or (seg != "+" and seg != t[i]):
                return False
        return len(p) == len(t)

    def publish(self, topic: str, payload: dict[str, Any], qos: int = QOS, retain: bool = False) -> None:
        json.dumps(payload)                                  # JSON にできない物は送らない
        self.log.append(Published(topic, payload, qos, retain))
        if retain:
            self._retained[topic] = payload
        for pattern, handlers in list(self._subs.items()):
            if self._match(pattern, topic):
                for h in list(handlers):
                    h(topic, payload)

    def subscribe(self, topic: str, handler: Handler) -> None:
        self._subs[topic].append(handler)
        for t, payload in self._retained.items():           # retain された最後の値を渡す
            if self._match(topic, t):
                handler(t, payload)


class PahoBroker(Broker):
    """paho-mqtt の薄い包み（任意依存）。"""

    def __init__(self, host: str, port: int = 1883, client_id: str = "serpens") -> None:
        import paho.mqtt.client as mqtt  # 遅延 import（依存に無い）

        self._c = mqtt.Client(client_id=client_id)
        self._handlers: dict[str, list[Handler]] = defaultdict(list)
        self._c.on_message = self._on_message
        self._c.connect(host, port)
        self._c.loop_start()

    def _on_message(self, _client: Any, _userdata: Any, msg: Any) -> None:
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return                                            # 壊れた JSON は捨てる（安全側: 何もしない）
        for pattern, hs in self._handlers.items():
            if LoopbackBroker._match(pattern, msg.topic):
                for h in hs:
                    h(msg.topic, payload)

    def publish(self, topic: str, payload: dict[str, Any], qos: int = QOS, retain: bool = False) -> None:
        self._c.publish(topic, json.dumps(payload, ensure_ascii=False), qos=qos, retain=retain)

    def subscribe(self, topic: str, handler: Handler) -> None:
        self._handlers[topic].append(handler)
        self._c.subscribe(topic, qos=QOS)
