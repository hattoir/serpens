"""Serpens 側の Task / Event 端点。

Task を受けて検証し、受理 / 拒否を `task_status` で返す。実行そのものは `executor`（後のフェーズで
locomotion / behavior へ繋ぐ）に渡す。ここでは**安全の設定は一切受け取らない**（スキーマにフィールドが無い）。

規則（docs/task_event_api.md）:
  - **stop は何より先に判定し、版違い・スキーマ違反・map_version 違いでも受理する**（止める方向は常に通す）
  - stop / FAULT / EMERGENCY で待ち行列を破棄し、人の操作（operator_resume）まで新しい Task を受けない。
    **operator_resume は MQTT / Home AI からは呼べない**（トピックに無い。Task に見せかけても拒否）
  - stop より前に発行された Task（t_ms が stop の t_ms 以前。QoS1 再送・再接続で後から届く）は rejected
  - Task トピックの retain は禁止: retain 付きで届いた Task は rejected
  - 同じ id の再送は冪等（前回と同じ task_status を返す）
  - Event には必ず data_source（HARDWARE / SIMULATION）。safety_state は retain、LWT で OFFLINE を retain
  - safety_state は変化時だけでなく **周期的（api.safety_state_period_ms < 受け側の失効 5s）** に出す（`tick`）。
    受け側は自分の時計で「最後に届いてからの時間」で失効を判定する（送り側の t_ms は使わない）
  - LWT は異常切断でしか出ない → 正常終了では自分で OFFLINE を出す（`close`）。再接続では今の状態で上書き（`announce`）
  - finding の resolved / dismissed は人の操作だけ（state_by=operator）。ロボットの再訪は reobservations に留める
"""
from __future__ import annotations

import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Callable

from serpens.api.bridge import QOS, RETAINED_EVENTS, TOPIC_TASK, Broker, event_topic
from serpens.api.validate import validate_event, validate_task
from serpens.config import load_config

API_VERSION = 1
LOCKING_MODES = ("FAULT_HOLD", "EMERGENCY_LATCHED", "TORQUE_DISABLED", "OFFLINE")
RESUME_WORDS = ("operator_resume", "resume", "clear_fault", "arm")   # MQTT から来たらそれだけで拒否


class Executor:
    """Task を実際に行う側の最小契約（フェーズ 5 で locomotion / behavior に繋ぐ）。"""

    def blockers(self) -> list[str]:
        """いま Task を始められない理由（空なら開始できる）。安全ゲートや自己位置の欠如など。"""
        return []

    def start(self, task: dict[str, Any]) -> None:
        raise NotImplementedError

    def stop(self, reason: str) -> None:
        raise NotImplementedError


def finding_rule_errors(finding: dict[str, Any]) -> list[str]:
    """スキーマでは書けない finding の規則。resolved / dismissed は人だけ。"""
    out = []
    if finding.get("state") in ("resolved", "dismissed") and finding.get("state_by") != "operator":
        out.append("resolved / dismissed にできるのは人の操作だけ（state_by=operator）")
    if finding.get("state_by") == "robot" and finding.get("state") not in ("candidate", "confirmed"):
        out.append("ロボットが付けられる状態は candidate / confirmed だけ")
    return out


@dataclass
class Endpoint:
    broker: Broker
    executor: Executor
    map_version: str
    data_source: str = "SIMULATION"              # 実機のときだけ HARDWARE
    clock_ms: Callable[[], int] = lambda: int(time.time() * 1000)
    safety_period_ms: int | None = None          # None → config api.safety_state_period_ms
    statuses: dict[str, dict[str, Any]] = field(default_factory=dict)   # task id → 最後の task_status
    active_task_id: str | None = None
    queue: list[dict[str, Any]] = field(default_factory=list)           # 受理済みで未開始の Task
    locked_reason: str | None = None            # stop / FAULT / EMERGENCY 後。人の操作で解く
    last_stop_t_ms: int | None = None           # これ以前に発行された Task は捨てる
    mode: str = "DISARMED"
    last_safety: dict[str, Any] | None = None   # 最後に出した safety_state のフィールド（周期送信・再接続で再送）
    last_safety_pub_ms: int | None = None
    closed: bool = False                        # close() 後。OFFLINE の後に古い状態を出し直さない
    # safety_state の「出すか決める → 送信待ち行列に積む」を 1 つにする（主ループの safety_state / tick / close と、
    # ネットワークスレッドの announce が交差しても、OFFLINE の後ろに古い状態が並ばない）。
    # **PUBACK を待つのはこのロックの外**: 中で待つとネットワークスレッドの announce がロック待ちで固まる。
    # RLock: 同期配信のブローカー（Loopback）では積んだ中から受け手経由で同じスレッドが戻ってくることがある。
    _safety_lock: threading.RLock = field(default_factory=threading.RLock, repr=False, compare=False)

    def __post_init__(self) -> None:
        if self.safety_period_ms is None:
            self.safety_period_ms = int(load_config()["api"]["safety_state_period_ms"])
        self.broker.set_will(event_topic("safety_state"), self._offline_payload("UNKNOWN"), retain=True)
        self.broker.subscribe(TOPIC_TASK, self._on_task)
        self.broker.on_connect(self.announce)
        self.broker.start()                     # will・購読・announce を揃えてから繋ぐ（実ブローカーは接続前にしか will を持てない）

    # ---- 送信 -------------------------------------------------------------------
    def _base(self, event: str) -> dict[str, Any]:
        return {"v": API_VERSION, "id": str(uuid.uuid4()), "t_ms": self.clock_ms(), "source": "serpens",
                "event": event, "data_source": self.data_source}

    def _offline_payload(self, stop_reason: str) -> dict[str, Any]:
        """LWT / 正常終了の OFFLINE。LWT の t_ms は接続時の値なので受け側は t_ms で鮮度を判断しない。"""
        return {**self._base("safety_state"), "mode": "OFFLINE", "stop_reason": stop_reason, "latched": True,
                "resume_requires": "operator"}

    def _event(self, event: str, **fields: Any) -> dict[str, Any]:
        msg = {**self._base(event), **fields}
        errs = validate_event(msg)
        if errs:
            raise ValueError(f"送ろうとした Event がスキーマに合わない: {errs}")
        return msg

    def emit(self, event: str, **fields: Any) -> dict[str, Any]:
        msg = self._event(event, **fields)
        self.broker.publish(event_topic(event), msg, qos=QOS, retain=event in RETAINED_EVENTS)
        return msg

    def task_status(self, task_id: str, status: str, reason: str | None = None, progress: float | None = None) -> None:
        fields: dict[str, Any] = {"task_id": task_id, "status": status}
        if reason:
            fields["reason"] = reason
        if progress is not None:
            fields["progress"] = progress
        self.statuses[task_id] = self.emit("task_status", **fields)

    def safety_state(self, mode: str, stop_reason: str, latched: bool, telemetry_age_ms: int | None = None) -> None:
        """機体の安全状態を外へ出す。FAULT / EMERGENCY なら待ち行列を破棄して人の操作までロック。"""
        self.mode = mode
        if mode in LOCKING_MODES:
            self._discard_queue(f"safety: {mode} ({stop_reason})")
            self.locked_reason = f"{mode}: {stop_reason}"
        fields: dict[str, Any] = {"mode": mode, "stop_reason": stop_reason, "latched": latched, "resume_requires": "operator"}
        if telemetry_age_ms is not None:
            fields["telemetry_age_ms"] = telemetry_age_ms
        with self._safety_lock:
            if self.closed:                                # 正常終了の後は出さない（OFFLINE を上書きしない）
                return
            self.last_safety = fields
            wait = self._enqueue_safety()
        wait()

    def _enqueue_safety(self) -> Callable[[], None]:
        """_safety_lock を持って呼ぶ。last_safety を積み、PUBACK を待つ関数を返す（待つのはロックの外）。"""
        msg = self._event("safety_state", **(self.last_safety or {}))
        wait = self.broker.enqueue(event_topic("safety_state"), msg, qos=QOS, retain=True)
        self.last_safety_pub_ms = self.clock_ms()
        return wait

    def tick(self) -> bool:
        """周期送信: 前回から safety_period_ms 以上経っていれば同じ safety_state を出し直す（変化が無くても）。"""
        with self._safety_lock:
            if self.closed or self.last_safety is None:
                return False
            since_ms = None if self.last_safety_pub_ms is None else self.clock_ms() - self.last_safety_pub_ms
            if since_ms is not None and since_ms < int(self.safety_period_ms or 0):
                return False
            wait = self._enqueue_safety()
        wait()
        return True

    def announce(self) -> None:
        """（再）接続直後: retain に残っている古い状態（LWT の OFFLINE など）を今の状態で上書きする。"""
        with self._safety_lock:
            if self.closed:                                # 正常終了の途中で再接続しても OFFLINE を上書きしない
                return
            if self.last_safety is None:
                self.last_safety = {"mode": self.mode, "stop_reason": "BOOT", "latched": self.mode in LOCKING_MODES,
                                    "resume_requires": "operator"}
            wait = self._enqueue_safety()
        wait()

    def close(self) -> None:
        """正常終了。LWT は異常切断でしか出ないので、自分で OFFLINE を retain で出してから切る。
        closed を立てるのと OFFLINE を積むのを同じロックの中で行う: 以後の safety_state / tick / announce は何も積まず、
        すでに判定を終えた送信は OFFLINE より前に並ぶ。"""
        with self._safety_lock:
            self.closed = True
            self.last_safety = None
            wait = self.broker.enqueue(event_topic("safety_state"), self._offline_payload("OPERATOR"), qos=QOS, retain=True)
        wait()
        closer = getattr(self.broker, "close", None)
        if callable(closer):
            closer()

    def floor_finding(self, finding: dict[str, Any]) -> dict[str, Any]:
        errs = finding_rule_errors(finding)
        if errs:
            raise ValueError("; ".join(errs))
        return self.emit("floor_finding", finding=finding)

    def battery(self, voltage_v: float, low: bool, percent_est: float | None = None) -> None:
        fields: dict[str, Any] = {"voltage_v": voltage_v, "low": low}
        if percent_est is not None:
            fields["percent_est"] = percent_est
        self.emit("battery", **fields)

    def operator_resume(self) -> None:
        """人の操作（機体のボタン / 運用者の端末）。stop / FAULT 後のロックを解く。**MQTT からは到達しない。**"""
        self.locked_reason = None

    # ---- 受信 -------------------------------------------------------------------
    def _discard_queue(self, reason: str) -> None:
        for t in self.queue:
            self.task_status(t["id"], "aborted", reason)
        self.queue.clear()
        if self.active_task_id is not None:
            self.task_status(self.active_task_id, "aborted", reason)
            self.active_task_id = None

    def _on_task(self, _topic: str, msg: Any, retained: bool) -> None:
        if not isinstance(msg, dict):
            return
        task_id = str(msg.get("id") or f"anon-{uuid.uuid4()}")
        if msg.get("task") == "stop":                      # 止める方向は何より先・何があっても受理
            self.executor.stop(str(msg.get("reason", "stop task")))
            self.last_stop_t_ms = max(self.last_stop_t_ms or 0, int(msg.get("t_ms", self.clock_ms()) or 0))
            self._discard_queue(f"stop {task_id}")
            self.locked_reason = f"stop {task_id}"
            self.task_status(task_id, "accepted")
            return
        if str(msg.get("task", "")).lower() in RESUME_WORDS or any(k in RESUME_WORDS for k in msg):
            self.task_status(task_id, "rejected", "再開・解除は MQTT / Home AI からはできない（人の操作だけ）")
            return
        if retained:                                       # Task トピックの retain は禁止
            self.task_status(task_id, "rejected", "retain 付きの Task は受け付けない")
            return
        errs = validate_task(msg)
        if errs:
            self.task_status(task_id, "rejected", f"schema: {errs[0]}")
            return
        if task_id in self.statuses:                       # 冪等: 同じ id には前回と同じ返事
            self.broker.publish(event_topic("task_status"), self.statuses[task_id], qos=QOS)
            return
        if self.last_stop_t_ms is not None and int(msg["t_ms"]) <= self.last_stop_t_ms:
            self.task_status(task_id, "rejected", "stop より前に発行された Task")
            return
        if self.locked_reason is not None:
            self.task_status(task_id, "rejected", f"人の操作で再開するまで受け付けない（{self.locked_reason}）")
            return
        if "map_version" in msg and msg["map_version"] != self.map_version:
            self.task_status(task_id, "rejected", f"map_version: 受け側は {self.map_version}")
            return
        why = self.executor.blockers()
        if why:
            self.task_status(task_id, "rejected", "; ".join(why))
            return
        if self.active_task_id is None:
            self.active_task_id = task_id
            self.executor.start(msg)
            self.task_status(task_id, "accepted")
        else:
            self.queue.append(msg)
            self.task_status(task_id, "accepted", "待ち行列")

    def task_finished(self, status: str = "done", reason: str | None = None) -> None:
        """executor から: 実行中の Task が終わった。待ち行列の次を始める。"""
        if self.active_task_id is not None:
            self.task_status(self.active_task_id, status, reason)
            self.active_task_id = None
        if self.queue and self.locked_reason is None:
            nxt = self.queue.pop(0)
            self.active_task_id = nxt["id"]
            self.executor.start(nxt)
            self.task_status(nxt["id"], "running")
