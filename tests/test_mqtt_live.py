"""本物の MQTT ブローカーでしか確かめられないもの: retain・LWT（keepalive の 1.5 倍）・正常終了の OFFLINE・
再接続の上書き・QoS1 の再送。**ハード不要。**

Mosquitto は常駐サービスにしない: このテストが **127.0.0.1 限定・一時ポート・一時設定** でサブプロセス起動し、終わったら止める。
前提（無ければ skip）: paho-mqtt（開発用の任意依存、`pip install paho-mqtt`）と mosquitto 実行ファイル
（PATH、SERPENS_MOSQUITTO 環境変数、または C:\\Program Files\\mosquitto\\mosquitto.exe）。
"""
from __future__ import annotations

import json
import os
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

paho = pytest.importorskip("paho.mqtt.client")

KEEPALIVE_S = 2                       # テストを速くするため短く（本番は config api.mqtt_keepalive_s）
HOST = "127.0.0.1"


def _mosquitto_exe() -> str | None:
    for cand in (os.environ.get("SERPENS_MOSQUITTO"), shutil.which("mosquitto"),
                 r"C:\Program Files\mosquitto\mosquitto.exe", "/usr/sbin/mosquitto", "/opt/homebrew/sbin/mosquitto"):
        if cand and Path(cand).exists():
            return cand
    return None


pytestmark = pytest.mark.skipif(_mosquitto_exe() is None, reason="mosquitto 実行ファイルが無い")


def _free_port() -> int:
    with socket.socket() as s:
        s.bind((HOST, 0))
        return int(s.getsockname()[1])


def _wait(pred, timeout_s: float = 3.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if pred():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture(scope="module")
def broker_port(tmp_path_factory) -> int:
    """一時設定で Mosquitto をサブプロセス起動（127.0.0.1 のみ・匿名・永続化なし）。"""
    port = _free_port()
    conf = tmp_path_factory.mktemp("mosq") / "mosquitto.conf"
    conf.write_text(f"listener {port} {HOST}\nallow_anonymous true\npersistence false\nlog_dest stderr\n", encoding="utf-8")
    proc = subprocess.Popen([_mosquitto_exe(), "-c", str(conf)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0) if sys.platform == "win32" else 0)

    def up() -> bool:
        try:
            with socket.create_connection((HOST, port), timeout=0.2):
                return True
        except OSError:
            return False
    if not _wait(up, 5.0):
        proc.kill()
        pytest.skip("Mosquitto が起動しなかった")
    yield port
    proc.terminate()
    try:
        proc.wait(3)
    except subprocess.TimeoutExpired:
        proc.kill()


def _endpoint(port: int, client_id: str):
    from serpens.api.bridge import PahoBroker, event_topic
    from serpens.api.endpoint import Endpoint, Executor

    class Ex(Executor):
        def start(self, task): ...
        def stop(self, reason): ...
    topic = event_topic("safety_state")
    will = (topic, {"v": 1, "id": "will-000001", "t_ms": int(time.time() * 1000), "source": "serpens",
                    "event": "safety_state", "data_source": "SIMULATION", "mode": "OFFLINE", "stop_reason": "UNKNOWN",
                    "latched": True, "resume_requires": "operator"}, True)
    serpens = PahoBroker(HOST, port, client_id=client_id, will=will, keepalive_s=KEEPALIVE_S)
    ep = Endpoint.__new__(Endpoint)                            # set_will を通さずに組む（Paho は接続前に will）
    ep.broker, ep.executor, ep.map_version, ep.data_source = serpens, Ex(), "tags-v0", "SIMULATION"
    ep.clock_ms, ep.statuses, ep.active_task_id, ep.queue = lambda: int(time.time() * 1000), {}, None, []
    ep.locked_reason, ep.last_stop_t_ms, ep.mode, ep.safety_period_ms = None, None, "DISARMED", 2000
    ep.last_safety, ep.last_safety_pub_ms, ep.closed = None, None, False
    serpens.subscribe("home/serpens/task", ep._on_task)
    serpens.on_connect(ep.announce)
    return serpens, ep, topic


def _home(port: int, topic: str, client_id: str):
    """client_id は購読者ごとに変える。同じ id が 2 つ繋がるとブローカーが古い方を蹴り、paho の自動再接続で
    互いを蹴り合う（再接続のたびに retain が届き直し、「購読した瞬間の retain」を確かめたことにならない）。"""
    from serpens.api.bridge import PahoBroker
    got: list[tuple[dict, bool, float]] = []
    home = PahoBroker(HOST, port, client_id=client_id)
    home.subscribe(topic, lambda _t, m, r: got.append((m, r, time.time())))
    return home, got


def test_retained_safety_state_and_lwt_fires_at_1_5x_keepalive(broker_port: int) -> None:
    """retain が後からの購読に届く。黙った（ハング・ケーブル抜け）クライアントの LWT は keepalive の 1.5 倍で出る。"""
    serpens, ep, topic = _endpoint(broker_port, "serpens-lwt")
    ep.safety_state("DRIVING", "NONE", latched=False)
    home, got = _home(broker_port, topic, "home-lwt")
    assert _wait(lambda: any(m["mode"] == "DRIVING" for m, _r, _t in got)), got      # 後から購読した側に retain が届く
    assert any(r for _m, r, _t in got)
    t_drop = time.time()
    serpens.freeze()                                                                 # 黙る（PINGREQ が止まる。ソケットは開いたまま）
    assert _wait(lambda: any(m["mode"] == "OFFLINE" for m, _r, _t in got), timeout_s=KEEPALIVE_S * 3), got
    dt = [t for m, _r, t in got if m["mode"] == "OFFLINE"][0] - t_drop
    assert KEEPALIVE_S * 1.0 <= dt <= KEEPALIVE_S * 2.5, f"LWT まで {dt:.2f}s（期待 ≈ 1.5 × {KEEPALIVE_S}s）"
    serpens.drop_socket()
    home.close()


def test_lwt_fires_immediately_when_the_socket_dies(broker_port: int) -> None:
    """プロセス落ち（FIN/RST が出る）は keepalive を待たずに即座に OFFLINE。"""
    serpens, ep, topic = _endpoint(broker_port, "serpens-crash")
    ep.safety_state("DRIVING", "NONE", latched=False)
    home, got = _home(broker_port, topic, "home-crash")
    assert _wait(lambda: any(m["mode"] == "DRIVING" for m, _r, _t in got))
    t_drop = time.time()
    serpens.drop_socket()
    assert _wait(lambda: any(m["mode"] == "OFFLINE" for m, _r, _t in got), timeout_s=KEEPALIVE_S), got
    assert [t for m, _r, t in got if m["mode"] == "OFFLINE"][0] - t_drop < KEEPALIVE_S
    home.close()


def test_graceful_close_publishes_offline_immediately_and_reconnect_overwrites(broker_port: int) -> None:
    serpens, ep, topic = _endpoint(broker_port, "serpens-close")
    ep.safety_state("DRIVING", "NONE", latched=False)
    home, got = _home(broker_port, topic, "home-close")
    assert _wait(lambda: any(m["mode"] == "DRIVING" for m, _r, _t in got))
    t0 = time.time()
    ep.close()                                                                       # 正常終了: 自分で OFFLINE、LWT は出ない
    assert _wait(lambda: any(m["mode"] == "OFFLINE" for m, _r, _t in got)), got
    assert [t for m, _r, t in got if m["mode"] == "OFFLINE"][0] - t0 < KEEPALIVE_S * 0.9
    # 立ち上げ直し → 接続時に今の状態で retain を上書き
    serpens2, ep2, _ = _endpoint(broker_port, "serpens-close-2")
    ep2.mode = "DISARMED"
    assert _wait(lambda: serpens2.connections >= 1)
    ep2.safety_state("DISARMED", "BOOT", latched=False)
    late, got_late = _home(broker_port, topic, "home-close-late")
    assert _wait(lambda: any(m["mode"] == "DISARMED" and r for m, r, _t in got_late)), got_late
    assert not any(m["mode"] == "OFFLINE" for m, _r, _t in got_late)                 # retain は DISARMED に置き換わっている
    ep2.close()
    home.close()
    late.close()


def test_reconnect_overwrites_lwt_from_the_network_thread_without_stalling_it(broker_port: int) -> None:
    """再接続時の announce は paho のネットワークスレッドで走る。そこで PUBACK を待つと、PUBACK を読むのがそのスレッド
    自身なのでタイムアウトまで必ず固まる → その間は何も送れず PINGREQ も出ない → 生きているのに keepalive 切れで
    LWT の OFFLINE、再接続してまた固まる（2026-09-29 の全体実行での 1 回落ちの原因）。"""
    serpens, ep, topic = _endpoint(broker_port, "serpens-reconnect")
    ep.safety_state("DISARMED", "BOOT", latched=False)
    home, got = _home(broker_port, topic, "home-reconnect")
    assert _wait(lambda: any(m["mode"] == "DISARMED" for m, _r, _t in got)), got
    took: list[float] = []
    announce = ep.announce

    def timed() -> None:
        t = time.time()
        announce()
        took.append(time.time() - t)
    serpens._on_connect[:] = [timed]
    serpens.drop_socket()                                                            # 落ちる → LWT の OFFLINE
    assert _wait(lambda: any(m["mode"] == "OFFLINE" for m, _r, _t in got), timeout_s=KEEPALIVE_S), got
    n, t0 = serpens.connections, time.time()
    serpens.reconnect()                                                              # CONNACK → announce（ネットワークスレッド）
    assert _wait(lambda: len(took) >= 1), "announce が呼ばれない"
    assert took[0] < 1.0, f"announce が {took[0]:.2f}s 固まった（ネットワークスレッドで PUBACK を待っている）"
    time.sleep(KEEPALIVE_S * 2)                                                      # keepalive 1.5 倍を越えて見張る
    assert serpens.connections == n + 1, "keepalive 切れで切断・再接続された"
    after = [(m["mode"], r) for m, r, t in got if t >= t0]
    assert after and after[0][0] == "DISARMED" and ("OFFLINE", False) not in after, after   # 生きている間に LWT が出ない
    late, got_late = _home(broker_port, topic, "home-reconnect-late")
    assert _wait(lambda: any(r for _m, r, _t in got_late)), got_late
    assert [m["mode"] for m, r, _t in got_late if r] == ["DISARMED"] * len([1 for _m, r, _t in got_late if r])
    ep.close()
    home.close()
    late.close()


def test_qos1_redelivery_reaches_a_reconnecting_subscriber(broker_port: int) -> None:
    """QoS1 + clean_session=False: 切断中に出た Task が再接続後に届く。"""
    import paho.mqtt.client as mqtt

    seen: list[dict] = []
    sub = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="serpens-durable", clean_session=False)
    sub.on_message = lambda _c, _u, m: seen.append(json.loads(m.payload))
    sub.connect(HOST, broker_port)
    sub.subscribe("home/serpens/task", qos=1)
    sub.loop_start()
    time.sleep(0.2)
    sub.loop_stop()
    sub.disconnect()                                             # 購読を残したまま離脱

    pub = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="home-pub")
    pub.connect(HOST, broker_port)
    pub.loop_start()                                             # QoS1 の PUBACK を受けるにはループが要る
    pub.publish("home/serpens/task", json.dumps({"v": 1, "id": "t-durable-1", "t_ms": 1, "source": "home_ai",
                                                  "task": "return_dock"}), qos=1).wait_for_publish(3.0)
    pub.loop_stop()
    pub.disconnect()

    sub.connect(HOST, broker_port)                               # 再接続 → 溜まっていた QoS1 が届く
    sub.loop_start()
    assert _wait(lambda: any(m["id"] == "t-durable-1" for m in seen)), seen
    sub.loop_stop()
    sub.disconnect()
