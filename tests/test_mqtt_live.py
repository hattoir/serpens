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


def _start_broker(conf_dir: Path, port: int) -> subprocess.Popen:
    """一時設定で Mosquitto をサブプロセス起動（127.0.0.1 のみ・匿名・永続化なし）。"""
    conf = conf_dir / f"mosquitto_{port}.conf"
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
    return proc


def _stop_broker(proc: subprocess.Popen) -> None:
    proc.terminate()
    try:
        proc.wait(3)
    except subprocess.TimeoutExpired:
        proc.kill()


@pytest.fixture(scope="module")
def broker_port(tmp_path_factory) -> int:
    port = _free_port()
    proc = _start_broker(tmp_path_factory.mktemp("mosq"), port)
    yield port
    _stop_broker(proc)


def _endpoint(port: int, client_id: str):
    from serpens.api.bridge import PahoBroker, event_topic
    from serpens.api.endpoint import Endpoint, Executor

    class Ex(Executor):
        def start(self, task): ...
        def stop(self, reason): ...
    serpens = PahoBroker(HOST, port, client_id=client_id, keepalive_s=KEEPALIVE_S, autoconnect=False)
    ep = Endpoint(serpens, Ex(), map_version="tags-v0", data_source="SIMULATION", safety_period_ms=2000)  # 本番と同じ組み方
    return serpens, ep, event_topic("safety_state")


def _live_offline_since(got: list, t_from: float) -> list[float]:
    """t_from 以後に**生で**（retain でなく）届いた OFFLINE の受信時刻。モジュール共通のブローカーには前の試験の retain の
    OFFLINE が残っていて、負荷で購読が接続より先になると、それが「今起きた OFFLINE」に見えていた（2026-09-29、負荷下 1/8）。"""
    return [t for m, r, t in got if m["mode"] == "OFFLINE" and not r and t >= t_from]


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
    assert _wait(lambda: bool(_live_offline_since(got, t_drop)), timeout_s=KEEPALIVE_S * 3), got
    dt = _live_offline_since(got, t_drop)[0] - t_drop
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
    assert _wait(lambda: bool(_live_offline_since(got, t_drop)), timeout_s=KEEPALIVE_S), got
    assert _live_offline_since(got, t_drop)[0] - t_drop < KEEPALIVE_S
    home.close()


def test_graceful_close_publishes_offline_immediately_and_reconnect_overwrites(broker_port: int) -> None:
    serpens, ep, topic = _endpoint(broker_port, "serpens-close")
    ep.safety_state("DRIVING", "NONE", latched=False)
    home, got = _home(broker_port, topic, "home-close")
    assert _wait(lambda: any(m["mode"] == "DRIVING" for m, _r, _t in got))
    t0 = time.time()
    ep.close()                                                                       # 正常終了: 自分で OFFLINE、LWT は出ない
    assert _wait(lambda: bool(_live_offline_since(got, t0))), got
    assert _live_offline_since(got, t0)[0] - t0 < KEEPALIVE_S * 0.9
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
    t_drop = time.time()
    serpens.drop_socket()                                                            # 落ちる → LWT の OFFLINE
    assert _wait(lambda: bool(_live_offline_since(got, t_drop)), timeout_s=KEEPALIVE_S), got
    # 「LWT より後」は時刻でなく受信順で切る。Windows の time.time() は刻みが粗く（約 15ms）、LWT と t0 が同じ刻みに入ると
    # 「t >= t0」が LWT 自身を拾っていた（負荷下 1/6 で再現、2026-09-29）
    k = next(i for i, (m, r, t) in enumerate(got) if m["mode"] == "OFFLINE" and not r and t >= t_drop)
    n = serpens.connections
    serpens.reconnect()                                                              # CONNACK → announce（ネットワークスレッド）
    assert _wait(lambda: len(took) >= 1), "announce が呼ばれない"
    assert took[0] < 1.0, f"announce が {took[0]:.2f}s 固まった（ネットワークスレッドで PUBACK を待っている）"
    time.sleep(KEEPALIVE_S * 2)                                                      # keepalive 1.5 倍を越えて見張る
    assert serpens.connections == n + 1, "keepalive 切れで切断・再接続された"
    after = [(m["mode"], r) for m, r, _t in got[k + 1:]]
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


def test_endpoint_builds_on_paho_normally_and_announces_on_first_connect(broker_port: int) -> None:
    """Endpoint(PahoBroker) を普通に組める: will（LWT）は Endpoint のもの、購読と announce は接続前に揃う。
    safety_state を一度も呼ばなくても、繋がった時点で今の状態（DISARMED / BOOT）が retain に載る。"""
    from serpens.api.bridge import event_topic

    serpens, ep, topic = _endpoint(broker_port, "serpens-build")
    assert _wait(lambda: serpens.connections >= 1)
    home, got = _home(broker_port, topic, "home-build")
    # 前の試験の retain（OFFLINE）が先に届くことがある（購読が announce より先）。announce の DISARMED / BOOT が retain で届くのを待つ
    assert _wait(lambda: any(r and (m["mode"], m["stop_reason"]) == ("DISARMED", "BOOT") for m, r, _t in got)), got
    with pytest.raises(RuntimeError):                                                # 接続後の will は黙って無視しない
        serpens.set_will(event_topic("safety_state"), ep._offline_payload("UNKNOWN"))
    t_drop = time.time()
    serpens.drop_socket()
    assert _wait(lambda: any(m["mode"] == "OFFLINE" and m["stop_reason"] == "UNKNOWN" and not r and t >= t_drop
                             for m, r, t in got), timeout_s=KEEPALIVE_S), got       # Endpoint が渡した will が生で出る
    home.close()


def test_publishing_while_the_broker_is_down_does_not_raise_and_reconnect_restores_now(tmp_path: Path) -> None:
    """ブローカーが落ちている間も主ループ（safety_state / tick / task_status）は例外で止まらない。
    切れている間の safety_state は溜めない（溜めると再接続後に古い値が announce の後から届き、retain を巻き戻す）。
    戻ったら announce が今の状態を retain に載せる。"""
    port = _free_port()
    proc = _start_broker(tmp_path, port)
    serpens, ep, topic = _endpoint(port, "serpens-outage")
    ep.safety_state("DRIVING", "NONE", latched=False)
    assert _wait(lambda: serpens.connections >= 1)
    _stop_broker(proc)
    assert _wait(lambda: not serpens._c.is_connected(), timeout_s=5.0), "切断に気づかない"
    ep.safety_state("DRIVING", "NONE", latched=False)
    ep.safety_state("EMERGENCY_LATCHED", "EMERGENCY", latched=True)                   # 以前はここで RuntimeError
    ep.task_status("t-outage-1", "aborted", "broker down")
    time.sleep(ep.safety_period_ms / 1000)
    assert ep.tick()
    serpens.freeze()                                                                 # 購読者が揃うまで再接続させない
    proc = _start_broker(tmp_path, port)
    try:
        watch, seen = _home(port, topic, "home-outage-watch")                        # 再接続の瞬間から全部見る
        assert _wait(lambda: watch.connections >= 1)
        time.sleep(0.3)                                                              # SUBSCRIBE をブローカーに通す
        serpens.reconnect()
        assert _wait(lambda: serpens.connections >= 2), "再接続しない"
        assert _wait(lambda: len(seen) >= 1), "再接続しても今の状態が出ない"
        time.sleep(1.0)                                                              # 遅れて届く古い値が無いか見張る
        assert all(m["mode"] == "EMERGENCY_LATCHED" for m, _r, _t in seen), [m["mode"] for m, _r, _t in seen]
        late, got = _home(port, topic, "home-outage-late")
        assert _wait(lambda: any(r for _m, r, _t in got)), got
        assert [m["mode"] for m, r, _t in got if r] == ["EMERGENCY_LATCHED"] * len([1 for _m, r, _t in got if r])
        ep.close()
        watch.close()
        late.close()
    finally:
        _stop_broker(proc)
