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
import threading
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

    def enqueue(self, topic: str, payload: dict[str, Any], qos: int = QOS,
                retain: bool = False) -> Callable[[], None]:
        """送信順を決める所（送信待ち行列に積む）だけを行い、受理を待つ関数を返す。
        呼び出し側は順序を守るロックの中で積み、**待つのはロックの外**でする（待つ間ほかのスレッドを止めない）。
        既定は publish してから何もしない関数を返す（同期配信のブローカー用）。"""
        self.publish(topic, payload, qos, retain)
        return lambda: None

    def subscribe(self, topic: str, handler: Handler) -> None:
        raise NotImplementedError

    def set_will(self, topic: str, payload: dict[str, Any], retain: bool = True) -> None:
        """接続が切れたときブローカーが代わりに出す最後の言葉（LWT）。"""
        raise NotImplementedError

    def on_connect(self, callback: Callable[[], None]) -> None:
        """（再）接続のたびに呼ぶ（retain の古い状態を今の状態で上書きするため）。既定は何もしない。"""

    def start(self) -> None:
        """will・購読・on_connect を登録し終えてから接続を始める。既定は何もしない（最初から繋がっている扱い）。"""


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


def _no_wait() -> None:
    """待つものが無い（ネットワークスレッド上・未接続で積まなかった・paho に積んだだけ）。"""


class PahoBroker(Broker):
    """paho-mqtt（2.x）の薄い包み。will は connect の前に渡す。keepalive の 1.5 倍で LWT が出る。

    Endpoint に渡すときは `autoconnect=False` で作る: Endpoint が will・購読・on_connect を登録してから `start()` する。
    接続は非同期（ブローカーが落ちていても生成・start で例外にしない。paho が再接続を続ける）。"""

    def __init__(self, host: str, port: int = 1883, client_id: str = "serpens",
                 will: tuple[str, dict[str, Any], bool] | None = None, keepalive_s: int | None = None,
                 autoconnect: bool = True) -> None:
        import paho.mqtt.client as mqtt  # 遅延 import（開発用の任意依存）
        from serpens.config import load_config

        if keepalive_s is None:
            keepalive_s = int(load_config()["api"]["mqtt_keepalive_s"])       # LWT は 1.5 倍後

        self._c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
        self._handlers: dict[str, list[Handler]] = defaultdict(list)
        self._on_connect: list[Callable[[], None]] = []
        self._lock = threading.Lock()                          # connections と _on_connect の対応を崩さない
        self._loop_thread: threading.Thread | None = None      # paho のネットワークスレッド（コールバックが走る所）
        self.connections = 0
        self._started = False
        self._addr = (host, port, keepalive_s)
        self._c.on_message = self._on_message
        self._c.on_connect = self._connected
        if will is not None:
            self.set_will(will[0], will[1], retain=will[2])
        if autoconnect:
            self.start()

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        host, port, keepalive_s = self._addr
        self._c.connect_async(host, port, keepalive=keepalive_s)
        self._c.loop_start()

    def _connected(self, *_args: Any) -> None:
        self._loop_thread = threading.current_thread()
        with self._lock:
            self.connections += 1
            cbs = list(self._on_connect)
        for pattern in list(self._handlers):
            self._c.subscribe(pattern, qos=QOS)                 # 再接続でも購読を張り直す
        for cb in cbs:
            cb()

    def _on_message(self, _client: Any, _userdata: Any, msg: Any) -> None:
        self._loop_thread = threading.current_thread()
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return                                            # 壊れた JSON は捨てる（安全側: 何もしない）
        for pattern, hs in self._handlers.items():
            if topic_matches(pattern, msg.topic):
                for h in hs:
                    h(msg.topic, payload, bool(msg.retain))

    def publish(self, topic: str, payload: dict[str, Any], qos: int = QOS, retain: bool = False) -> None:
        self.enqueue(topic, payload, qos, retain)()

    def enqueue(self, topic: str, payload: dict[str, Any], qos: int = QOS,
                retain: bool = False) -> Callable[[], None]:
        """送信順は paho の送信待ち行列の順（= 積んだ順）。返す関数が PUBACK を待つ。

        - **ネットワークスレッド上（on_connect の announce、on_message の task_status）では待たない。**
          PUBACK を読むのはそのスレッド自身なので、待つと必ずタイムアウトまで固まり、その間は何も送れず PINGREQ も
          出ない → keepalive の 1.5 倍で生きているのに LWT の OFFLINE が出る・stop Task の処理が遅れる。
        - **切れている間は例外にしない**（paho は RuntimeError を投げ、主ループの tick まで上がっていた）。
          retain 付き（safety_state）は捨てる: 最後の値だけが意味を持ち、再接続時の announce が今の値で出し直す。
          溜めると announce の後に古い値が届いて retain を巻き戻す。retain 無し（task_status 等）は paho の QoS1
          待ち行列に残り、再接続後に届く。"""
        data = json.dumps(payload, ensure_ascii=False)
        if retain and not self._c.is_connected():
            return _no_wait
        info = self._c.publish(topic, data, qos=qos, retain=retain)
        if info.rc != 0 or threading.current_thread() is self._loop_thread:       # 0 = MQTT_ERR_SUCCESS
            return _no_wait
        return lambda: info.wait_for_publish(2.0)

    def subscribe(self, topic: str, handler: Handler) -> None:
        self._handlers[topic].append(handler)
        if self._c.is_connected():                              # 未接続なら _connected が張る
            self._c.subscribe(topic, qos=QOS)

    def set_will(self, topic: str, payload: dict[str, Any], retain: bool = True) -> None:
        if self._started:
            raise RuntimeError("PahoBroker の will は接続前にしか設定できない（autoconnect=False で作り、start の前に渡す）")
        self._c.will_set(topic, json.dumps(payload, ensure_ascii=False), qos=QOS, retain=retain)

    def on_connect(self, callback: Callable[[], None]) -> None:
        """接続は生成時に始まるので、登録より先に CONNACK を処理し終えていることがある。
        そのときは今の接続の分をここで 1 回呼ぶ（呼ばないと再接続時の retain 上書きが初回だけ抜ける）。"""
        with self._lock:
            self._on_connect.append(callback)
            missed = self.connections > 0 and self._c.is_connected()   # 切れている間なら次の再接続で呼ばれる
        if missed:
            callback()

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
