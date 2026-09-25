"""本物の MQTT ブローカーでしか確かめられないもの: retain・LWT・QoS1 の再送。**ハード不要。**

前提（無ければ skip）:
  - paho-mqtt（`pip install paho-mqtt`。requirements には入れていない）
  - Mosquitto が localhost:1883 で動いている（`mosquitto -v`）。環境変数 SERPENS_MQTT_HOST で変えられる
"""
from __future__ import annotations

import json
import os
import socket
import time

import pytest

paho = pytest.importorskip("paho.mqtt.client")

HOST = os.environ.get("SERPENS_MQTT_HOST", "127.0.0.1")
PORT = int(os.environ.get("SERPENS_MQTT_PORT", "1883"))


def _broker_up() -> bool:
    try:
        with socket.create_connection((HOST, PORT), timeout=0.5):
            return True
    except OSError:
        return False


pytestmark = pytest.mark.skipif(not _broker_up(), reason=f"MQTT ブローカーが {HOST}:{PORT} に無い")


def _wait(pred, timeout_s: float = 3.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if pred():
            return True
        time.sleep(0.02)
    return False


def test_retained_safety_state_and_lwt_offline() -> None:
    from serpens.api.bridge import PahoBroker, event_topic
    from serpens.api.endpoint import Endpoint, Executor

    topic = event_topic("safety_state")

    class Ex(Executor):
        def start(self, task): ...
        def stop(self, reason): ...

    # Serpens 側: will を持って接続し、safety_state を retain で出す
    will = (topic, {"v": 1, "id": "will-000001", "t_ms": int(time.time() * 1000), "source": "serpens",
                    "event": "safety_state", "data_source": "SIMULATION", "mode": "OFFLINE", "stop_reason": "UNKNOWN",
                    "latched": True, "resume_requires": "operator"}, True)
    serpens = PahoBroker(HOST, PORT, client_id="serpens-test", will=will)
    ep = Endpoint.__new__(Endpoint)                            # set_will を通さずに組む（Paho は接続前に will）
    ep.broker, ep.executor, ep.map_version, ep.data_source = serpens, Ex(), "tags-v0", "SIMULATION"
    ep.clock_ms, ep.statuses, ep.active_task_id, ep.queue = lambda: int(time.time() * 1000), {}, None, []
    ep.locked_reason, ep.last_stop_t_ms, ep.mode = None, None, "DISARMED"
    ep.safety_state("DRIVING", "NONE", latched=False)
    time.sleep(0.3)

    # 後から購読した Home AI に retain が届く
    got: list[tuple[dict, bool]] = []
    home = PahoBroker(HOST, PORT, client_id="home-test")
    home.subscribe(topic, lambda _t, m, r: got.append((m, r)))
    assert _wait(lambda: any(m["mode"] == "DRIVING" for m, _ in got)), got
    assert any(r for _m, r in got)                               # retain フラグが立っている

    # Serpens が黙って消える → ブローカーが LWT（OFFLINE）を retain で出す
    serpens._c.loop_stop()
    serpens._c.socket().close()                                  # disconnect を送らずに切る
    assert _wait(lambda: any(m["mode"] == "OFFLINE" for m, _ in got), timeout_s=10.0), got
    home.close()


def test_qos1_redelivery_reaches_a_reconnecting_subscriber() -> None:
    """QoS1 + clean_session=False: 切断中に出た Task が再接続後に届く。"""
    import paho.mqtt.client as mqtt

    seen: list[dict] = []
    sub = mqtt.Client(client_id="serpens-durable", clean_session=False)
    sub.on_message = lambda _c, _u, m: seen.append(json.loads(m.payload))
    sub.connect(HOST, PORT)
    sub.subscribe("home/serpens/task", qos=1)
    sub.loop_start()
    time.sleep(0.2)
    sub.loop_stop()
    sub.disconnect()                                             # 購読を残したまま離脱

    pub = mqtt.Client(client_id="home-pub")
    pub.connect(HOST, PORT)
    pub.publish("home/serpens/task", json.dumps({"v": 1, "id": "t-durable-1", "t_ms": 1, "source": "home_ai",
                                                  "task": "return_dock"}), qos=1).wait_for_publish()
    pub.disconnect()

    sub.connect(HOST, PORT)                                      # 再接続 → 溜まっていた QoS1 が届く
    sub.loop_start()
    assert _wait(lambda: any(m["id"] == "t-durable-1" for m in seen)), seen
    sub.loop_stop()
    sub.disconnect()
