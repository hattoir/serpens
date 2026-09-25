"""Home AI ↔ Serpens の境界（MQTT）。**リアルタイムの関節制御には使わない**（決定事項 3）。

トピック（docs/task_event_api.md）:
    home/serpens/task              Home AI → Serpens（QoS 1、**retain 禁止**: retain 付きは受け側が拒否）
    home/serpens/event/<event>     Serpens → Home AI（QoS 1。safety_state は retain、LWT で offline を retain）

handler(topic, payload, retained): retained は「ブローカーに残っていた値」か。
`Broker` は publish / subscribe / set_will の薄い抽象。`LoopbackBroker` はプロセス内（テスト・モック用。retain と
LWT を模す）。`PahoBroker` は paho-mqtt が入っていれば使える（依存には入れていない）。
ループバックで確かめられないもの: 本物の retain・LWT・QoS1 の再送 → tests/test_mqtt_live.py（Mosquitto が要る）。
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

Handler = Callable[[str, dict[str, Any], bool], None]


def event_topic(event: str) -> str:
    return f"{TOPIC_EVENT}/{event}"


def topic_matches(pattern: str, topic: str) -> bool:
    p, t = pattern.split("/"), topic.split("/")
    for i, seg in enumerate(p):
        if seg == "#":
            return True
        if i >= len(t) or (seg != "+" and seg != t[i]):
            return False
    return len(p) == len(t)


@dataclass(frozen=True)
class Published:
    topic: str
    payload: dict[str, Any]
    qos: int
    retain: bool


class Broker:
    """publish / subscribe / will の抽象。"""

    def publish(self, topic: str, payload: dict[str, Any], qos: int = QOS, retain: bool = False) -> None:
        raise NotImplementedError

    def subscribe(self, topic: str, handler: Handler) -> None:
        raise NotImplementedError

    def set_will(self, topic: str, payload: dict[str, Any], retain: bool = True) -> None:
        """接続が切れたときブローカーが代わりに出す最後の言葉（LWT）。"""
        raise NotImplementedError


class LoopbackBroker(Broker):
    """プロセス内のブローカー。配信は同期（テストで順序が読める）。retain / LWT も模す。"""

    def __init__(self) -> None:
        self._subs: dict[str, list[Handler]] = defaultdict(list)
        self._retained: dict[str, dict[str, Any]] = {}
        self._will: Published | None = None
        self.log: list[Published] = []

    def publish(self, topic: str, payload: dict[str, Any], qos: int = QOS, retain: bool = False) -> None:
        json.dumps(payload)                                  # JSON にできない物は送らない
        self.log.append(Published(topic, payload, qos, retain))
        if retain:
            self._retained[topic] = payload
        for pattern, handlers in list(self._subs.items()):
            if topic_matches(pattern, topic):
                for h in list(handlers):
                    h(topic, payload, False)

    def subscribe(self, topic: str, handler: Handler) -> None:
        self._subs[topic].append(handler)
        for t, payload in self._retained.items():           # retain された最後の値を渡す
            if topic_matches(topic, t):
                handler(t, payload, True)

    def set_will(self, topic: str, payload: dict[str, Any], retain: bool = True) -> None:
        self._will = Published(topic, payload, QOS, retain)

    def simulate_disconnect(self) -> None:
        """クライアントが黙って消えた: ブローカーが LWT を出す。"""
        if self._will is not None:
            self.publish(self._will.topic, self._will.payload, self._will.qos, self._will.retain)

    def redeliver(self, published: Published) -> None:
        """QoS1 の再送・再接続で後から届く、を模す（retain フラグは元のまま）。"""
        for pattern, handlers in list(self._subs.items()):
            if topic_matches(pattern, published.topic):
                for h in list(handlers):
                    h(published.topic, published.payload, published.retain)


class PahoBroker(Broker):
    """paho-mqtt の薄い包み（任意依存）。will は connect の前に set_will しておく。"""

    def __init__(self, host: str, port: int = 1883, client_id: str = "serpens",
                 will: tuple[str, dict[str, Any], bool] | None = None) -> None:
        import paho.mqtt.client as mqtt  # 遅延 import（依存に無い）

        self._c = mqtt.Client(client_id=client_id)
        self._handlers: dict[str, list[Handler]] = defaultdict(list)
        self._c.on_message = self._on_message
        if will is not None:
            t, p, r = will
            self._c.will_set(t, json.dumps(p, ensure_ascii=False), qos=QOS, retain=r)
        self._c.connect(host, port)
        self._c.loop_start()

    def _on_message(self, _client: Any, _userdata: Any, msg: Any) -> None:
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return                                            # 壊れた JSON は捨てる（安全側: 何もしない）
        for pattern, hs in self._handlers.items():
            if topic_matches(pattern, msg.topic):
                for h in hs:
                    h(msg.topic, payload, bool(msg.retain))

    def publish(self, topic: str, payload: dict[str, Any], qos: int = QOS, retain: bool = False) -> None:
        self._c.publish(topic, json.dumps(payload, ensure_ascii=False), qos=qos, retain=retain)

    def subscribe(self, topic: str, handler: Handler) -> None:
        self._handlers[topic].append(handler)
        self._c.subscribe(topic, qos=QOS)

    def set_will(self, topic: str, payload: dict[str, Any], retain: bool = True) -> None:
        raise RuntimeError("PahoBroker の will は生成時に渡す（接続前にしか設定できない）")

    def close(self) -> None:
        self._c.loop_stop()
        self._c.disconnect()
