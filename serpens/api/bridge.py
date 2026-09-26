"""Home AI ↔ Serpens の境界（MQTT）。**リアルタイムの関節制御には使わない**（決定事項 3）。

トピック（docs/task_event_api.md）:
    home/serpens/task              Home AI → Serpens（QoS 1、**retain 禁止**: retain 付きは受け側が拒否）
    home/serpens/event/<event>     Serpens → Home AI（QoS 1。safety_state は retain、LWT で offline を retain）

handler(topic, payload, retained): retained は「ブローカーに残っていた値」か。
`Broker` は publish / subscribe / set_will / on_connect の薄い抽象。`LoopbackBroker` はプロセス内（テスト・モック用。
retain と LWT と再接続を模す）。`PahoBroker` は paho-mqtt（開発用の任意依存）が入っていれば使える。
ループバックで確かめられないもの: 本物の retain・LWT（keepalive の 1.5 倍で出る）・QoS1 の再送
→ tests/test_mqtt_live.py（Mosquitto を 127.0.0.1 の一時ポートでサブプロセス起動する。常駐させない）。
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
    """publish / subscribe / will / on_connect の抽象。"""

    def publish(self, topic: str, payload: dict[str, Any], qos: int = QOS, retain: bool = False) -> None:
        raise NotImplementedError

    def subscribe(self, topic: str, handler: Handler) -> None:
        raise NotImplementedError

    def set_will(self, topic: str, payload: dict[str, Any], retain: bool = True) -> None:
        """接続が切れたときブローカーが代わりに出す最後の言葉（LWT）。"""
        raise NotImplementedError

    def on_connect(self, callback: Callable[[], None]) -> None:
        """（再）接続のたびに呼ぶ（retain の古い状態を今の状態で上書きするため）。既定は何もしない。"""


class LoopbackBroker(Broker):
    """プロセス内のブローカー。配信は同期（テストで順序が読める）。retain / LWT / 再接続も模す。"""

    def __init__(self) -> None:
        self._subs: dict[str, list[Handler]] = defaultdict(list)
        self._retained: dict[str, dict[str, Any]] = {}
        self._will: Published | None = None
        self._on_connect: list[Callable[[], None]] = []
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

    def on_connect(self, callback: Callable[[], None]) -> None:
        self._on_connect.append(callback)

    def simulate_disconnect(self) -> None:
        """クライアントが黙って消えた: ブローカーが LWT を出す。"""
        if self._will is not None:
            self.publish(self._will.topic, self._will.payload, self._will.qos, self._will.retain)

    def simulate_reconnect(self) -> None:
        """再接続: クライアント側の on_connect が走る。"""
        for cb in list(self._on_connect):
            cb()

    def redeliver(self, published: Published) -> None:
        """QoS1 の再送・再接続で後から届く、を模す（retain フラグは元のまま）。"""
        for pattern, handlers in list(self._subs.items()):
            if topic_matches(pattern, published.topic):
                for h in list(handlers):
                    h(published.topic, published.payload, published.retain)


class PahoBroker(Broker):
    """paho-mqtt（2.x）の薄い包み。will は connect の前に渡す。keepalive の 1.5 倍で LWT が出る。"""

    def __init__(self, host: str, port: int = 1883, client_id: str = "serpens",
                 will: tuple[str, dict[str, Any], bool] | None = None, keepalive_s: int | None = None) -> None:
        import paho.mqtt.client as mqtt  # 遅延 import（開発用の任意依存）
        from serpens.config import load_config

        if keepalive_s is None:
            keepalive_s = int(load_config()["api"]["mqtt_keepalive_s"])       # LWT は 1.5 倍後

        self._c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
        self._handlers: dict[str, list[Handler]] = defaultdict(list)
        self._on_connect: list[Callable[[], None]] = []
        self.connections = 0
        self._c.on_message = self._on_message
        self._c.on_connect = self._connected
        if will is not None:
            t, p, r = will
            self._c.will_set(t, json.dumps(p, ensure_ascii=False), qos=QOS, retain=r)
        self._c.connect(host, port, keepalive=keepalive_s)
        self._c.loop_start()

    def _connected(self, *_args: Any) -> None:
        self.connections += 1
        for pattern in list(self._handlers):
            self._c.subscribe(pattern, qos=QOS)                 # 再接続でも購読を張り直す
        for cb in list(self._on_connect):
            cb()

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
        self._c.publish(topic, json.dumps(payload, ensure_ascii=False), qos=qos, retain=retain).wait_for_publish(2.0)

    def subscribe(self, topic: str, handler: Handler) -> None:
        self._handlers[topic].append(handler)
        self._c.subscribe(topic, qos=QOS)

    def set_will(self, topic: str, payload: dict[str, Any], retain: bool = True) -> None:
        raise RuntimeError("PahoBroker の will は生成時に渡す（接続前にしか設定できない）")

    def on_connect(self, callback: Callable[[], None]) -> None:
        self._on_connect.append(callback)

    def freeze(self) -> None:
        """テスト用: ネットワークループを止めて黙る（PC のハング・ケーブル抜け。ソケットは閉じない = FIN も RST も出ない）。
        ブローカーは PINGREQ が来ないので keepalive の 1.5 倍で LWT を出す。ソケットを閉じると即座に LWT が出る（別経路）。"""
        self._c.loop_stop()

    def drop_socket(self) -> None:
        """テスト用: DISCONNECT を送らずにソケットを閉じる（プロセス落ち）。ブローカーは即座に LWT を出す。"""
        self._c.loop_stop()
        sock = self._c.socket()
        if sock is not None:
            sock.close()

    def reconnect(self) -> None:
        self._c.reconnect()
        self._c.loop_start()

    def close(self) -> None:
        """正常終了（DISCONNECT を送る → LWT は出ない。OFFLINE は呼び出し側が先に出す）。"""
        self._c.disconnect()
        self._c.loop_stop()
