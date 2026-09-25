"""フェーズ 1（Floor Watch）: Home AI ↔ Serpens の Task / Event API。ハード不要。

  - スキーマ（schemas/*.json）に合う / 合わない例
  - 版違い・map_version 違い・未知フィールド・安全設定を変えるフィールドの拒否
  - Loopback で Task → 受理/拒否 → Event の往復。Home AI 側モックが受け取る
  - レビュー 1〜5: stop は何があっても受理 / stop・FAULT で待ち行列を破棄しロック / stop 前の Task は rejected /
    retain 付き Task は拒否 / LWT の OFFLINE と古い safety_state の無効化 / 危険物の別枠通知 / resolved は人だけ
"""
from __future__ import annotations

import json
from typing import Any

import pytest

from serpens.api.bridge import LoopbackBroker, event_topic
from serpens.api.endpoint import Endpoint, Executor, finding_rule_errors
from serpens.api.validate import SCHEMA_DIR, validate_event, validate_task
from tests.mocks.home_ai_mock import SAFETY_MAX_AGE_MS, HomeAiMock

MAP = "tags-v0"


class Clock:
    def __init__(self) -> None:
        self.t = 1_000

    def __call__(self) -> int:
        return self.t


def finding(**over: Any) -> dict[str, Any]:
    f = {
        "finding_id": "f-00000001", "t_ms": 1000, "frame_id": "home", "map_version": MAP,
        "pose": {"x_m": 1.20, "y_m": -0.35, "yaw_rad": 0.1, "sigma_xy_m": 0.08, "sigma_yaw_rad": 0.05,
                 "pose_source": "ODOMETRY_IMU"},
        "photos": [{"kind": "normal", "crop_path": "data/findings/f-00000001/normal.png", "w_px": 160, "h_px": 120, "t_ms": 1000},
                   {"kind": "raking", "crop_path": "data/findings/f-00000001/raking.png", "w_px": 160, "h_px": 120, "t_ms": 1100},
                   {"kind": "line", "crop_path": "data/findings/f-00000001/line.png", "w_px": 160, "h_px": 120, "t_ms": 1200}],
        "candidates": [{"kind": "washer", "confidence": 0.7}, {"kind": "button_cell", "confidence": 0.2}],
        "size": {"diameter_mm": 19.0, "sigma_mm": 3.0, "height_mm": 1.5, "height_sigma_mm": 0.8, "method": "line_light"},
        "risk": {"ingestion": 0.8, "sharp": 0.1, "child_reachable": True, "child_distance_m": None, "score": 0.75,
                 "mandatory_notify": True, "critical_kinds": ["button_cell"],
                 "rationale": ["washer/button_cell 19mm", "高さあり（模様ではない）", "子どもの距離 不明 → 近いとみなす"]},
        "state": "candidate", "state_by": "robot",
    }
    f.update(over)
    return f


class RecordingExecutor(Executor):
    def __init__(self, blockers: list[str] | None = None) -> None:
        self.started: list[dict[str, Any]] = []
        self.stopped: list[str] = []
        self._blockers = blockers or []

    def blockers(self) -> list[str]:
        return list(self._blockers)

    def start(self, task: dict[str, Any]) -> None:
        self.started.append(task)

    def stop(self, reason: str) -> None:
        self.stopped.append(reason)


@pytest.fixture()
def rig():
    clock = Clock()
    broker = LoopbackBroker()
    ex = RecordingExecutor()
    ep = Endpoint(broker, ex, map_version=MAP, data_source="SIMULATION", clock_ms=clock)
    home = HomeAiMock(broker, map_version=MAP, clock_ms=clock)
    return broker, ep, ex, home, clock


TARGET = {"x_m": 1.0, "y_m": 0.2, "yaw_rad": 0.0}


# ---- スキーマ ------------------------------------------------------------------------
def test_schema_files_are_valid_json() -> None:
    for name in ("common.json", "task.json", "event.json"):
        json.loads((SCHEMA_DIR / name).read_text(encoding="utf-8"))


def test_valid_tasks_pass_and_kind_rules_apply() -> None:
    base = {"v": 1, "id": "t-00000001", "t_ms": 0, "source": "home_ai"}
    assert validate_task({**base, "task": "inspect_point", "frame_id": "home", "map_version": MAP, "target": TARGET}) == []
    assert validate_task({**base, "task": "stop", "reason": "test"}) == []
    assert validate_task({**base, "task": "return_dock"}) == []
    assert validate_task({**base, "task": "inspect_point"})                       # target が無い
    assert validate_task({**base, "task": "patrol_route", "frame_id": "home", "map_version": MAP, "waypoints": []})
    assert validate_task({**base, "v": 2, "task": "return_dock"})                  # 版違い
    assert validate_task({**base, "task": "return_dock", "max_speed_mm_s": 999})  # 安全設定は受け付けない
    assert validate_task({**base, "task": "inspect_point", "frame_id": "room", "map_version": MAP, "target": TARGET})
    assert validate_task({**base, "task": "highlight_point", "frame_id": "home", "map_version": MAP,
                          "target": {**TARGET, "yaw_rad": 9.0}})


def test_floor_finding_schema_covers_the_required_fields() -> None:
    base = {"v": 1, "id": "e-00000001", "t_ms": 0, "source": "serpens", "event": "floor_finding", "data_source": "SIMULATION"}
    assert validate_event({**base, "finding": finding()}) == []
    bad = finding()
    del bad["risk"]["mandatory_notify"]
    assert validate_event({**base, "finding": bad})
    assert validate_event({**base, "finding": finding(photos=[])})
    assert validate_event({**base, "finding": finding(state="found")})
    assert validate_event({**base})                                                # finding が無い
    assert validate_event({**base, "finding": finding(), "data_source": "REAL"})   # 出どころの語彙
    assert validate_event({**base, "finding": finding(reobservations=[{"t_ms": 5, "seen": False, "pose_sigma_xy_m": 0.1}])}) == []


def test_task_schema_has_no_safety_settings() -> None:
    props = json.loads((SCHEMA_DIR / "task.json").read_text(encoding="utf-8"))["properties"]
    for word in ("speed", "torque", "ttl", "limit", "angle_deg", "heartbeat"):
        assert not any(word in k.lower() for k in props), word


# ---- 往復 ------------------------------------------------------------------------------
def test_task_roundtrip_accept_queue_reject_and_idempotent(rig) -> None:
    broker, ep, ex, home, clock = rig
    t1 = home.task("inspect_point", target=TARGET)
    assert home.statuses_of(t1) == ["accepted"] and ex.started[0]["id"] == t1
    home.task("inspect_point", task_id=t1, target=TARGET)                         # 再送 → 冪等
    assert home.statuses_of(t1) == ["accepted", "accepted"] and len(ex.started) == 1
    t2 = home.task("highlight_point", target=TARGET, duration_s=3)                 # 実行中 → 待ち行列
    assert home.statuses_of(t2) == ["accepted"] and home.last_reason(t2) == "待ち行列" and len(ex.started) == 1
    ep.task_finished("done")
    assert home.statuses_of(t1)[-1] == "done" and home.statuses_of(t2) == ["accepted", "running"]
    assert ex.started[-1]["id"] == t2
    bad = home.task("inspect_point", target=TARGET, map_version="tags-v9")
    assert home.statuses_of(bad) == ["rejected"] and "map_version" in home.last_reason(bad)
    broken = home.task("inspect_point")                                            # target 無し
    assert home.statuses_of(broken) == ["rejected"] and home.last_reason(broken).startswith("schema")
    assert not home.schema_errors


def test_blockers_reject_without_starting() -> None:
    b = LoopbackBroker()
    ex = RecordingExecutor(blockers=["電気安全ゲート未完了: physical_estop", "自己位置が未取得"])
    Endpoint(b, ex, map_version=MAP)
    home = HomeAiMock(b, map_version=MAP)
    tid = home.task("patrol_route", waypoints=[{"x_m": 0.5, "y_m": 0.0, "yaw_rad": 0.0}])
    assert home.statuses_of(tid) == ["rejected"] and not ex.started
    assert "physical_estop" in home.last_reason(tid)


# ---- レビュー 1〜5 ----------------------------------------------------------------------
def test_stop_is_accepted_even_when_malformed(rig) -> None:
    """1: 止める方向は常に受理。版違い・未知フィールド・map_version 違い・id 無しでも stop は通す。"""
    broker, ep, ex, home, clock = rig
    home.task("stop", v=7, map_version="tags-v9", bogus=1, reason="malformed")
    assert ex.stopped == ["malformed"]
    broker.publish("home/serpens/task", {"task": "stop"})                         # ヘッダすら無い
    assert len(ex.stopped) == 2
    assert all(s == "accepted" for e in home.events if e["event"] == "task_status" for s in [e["status"]])


def test_stop_discards_queue_and_locks_until_operator(rig) -> None:
    """2: stop で待ち行列を破棄し、人の操作なしでは新しい Task を受けない。"""
    broker, ep, ex, home, clock = rig
    t1 = home.task("inspect_point", target=TARGET)
    t2 = home.task("highlight_point", target=TARGET)
    clock.t += 10
    s = home.task("stop", reason="operator")
    assert home.statuses_of(t1)[-1] == "aborted" and home.statuses_of(t2)[-1] == "aborted"
    assert ep.queue == [] and ep.active_task_id is None and ex.stopped == ["operator"]
    clock.t += 10
    t3 = home.task("return_dock")
    assert home.statuses_of(t3) == ["rejected"] and "人の操作" in home.last_reason(t3)
    ep.operator_resume()
    t4 = home.task("return_dock")
    assert home.statuses_of(t4) == ["accepted"]


def test_fault_and_emergency_discard_queue_and_lock(rig) -> None:
    broker, ep, ex, home, clock = rig
    t1 = home.task("inspect_point", target=TARGET)
    t2 = home.task("return_dock")
    ep.safety_state("FAULT_HOLD", "HEARTBEAT_LOST", latched=False)
    assert home.statuses_of(t1)[-1] == "aborted" and home.statuses_of(t2)[-1] == "aborted"
    clock.t += 10
    assert home.statuses_of(home.task("return_dock")) == ["rejected"]
    ep.safety_state("DISARMED", "NONE", latched=False)                          # 自動では解けない
    assert home.statuses_of(home.task("return_dock")) == ["rejected"]
    ep.operator_resume()
    assert home.statuses_of(home.task("return_dock")) == ["accepted"]


def test_tasks_issued_before_stop_are_rejected_when_they_arrive_late(rig) -> None:
    """2: QoS1 の再送・再接続で stop より前に発行された Task が後から届いたら rejected。"""
    broker, ep, ex, home, clock = rig
    clock.t = 5_000
    home.task("stop", reason="operator")
    ep.operator_resume()
    late = home.task("inspect_point", target=TARGET, t_ms=4_990)                   # stop より前の発行
    assert home.statuses_of(late) == ["rejected"] and "stop より前" in home.last_reason(late)
    fresh = home.task("inspect_point", target=TARGET, t_ms=5_010)
    assert home.statuses_of(fresh) == ["accepted"]
    # 再配送（同じメッセージがもう一度届く）も同じ返事
    earlier = [p for p in broker.log if p.topic == "home/serpens/task" and p.payload["id"] == late][0]
    broker.redeliver(earlier)
    assert home.statuses_of(late) == ["rejected", "rejected"] and len(ex.started) == 1


def test_retained_task_is_rejected(rig) -> None:
    """2: Task トピックの retain は禁止。retain 付きは拒否（後から購読した Serpens が古い Task を実行しない）。"""
    broker, ep, ex, home, clock = rig
    # 生配信では MQTT 3.1.1 の retain フラグは 0 で届く（区別できない）ので通る。危険なのは「後から購読したとき
    # ブローカーの保存分が届く」方で、そちらは retain=1 で来るので拒否する
    tid = home.task("inspect_point", target=TARGET, retain=True)
    assert home.statuses_of(tid) == ["accepted"]
    b2 = LoopbackBroker()
    h2 = HomeAiMock(b2, map_version=MAP, clock_ms=clock)
    old = h2.task("inspect_point", target=TARGET, retain=True)                      # 端点が来る前に retain
    ex2 = RecordingExecutor()
    Endpoint(b2, ex2, map_version=MAP, clock_ms=clock)                              # 購読時に retain が届く
    assert ex2.started == [] and "retain" in h2.last_reason(old)


def test_lwt_offline_is_retained_and_stale_safety_state_is_ignored(rig) -> None:
    """3: 接続断で OFFLINE が retain で出る。受け側は古い safety_state を無効にする。"""
    broker, ep, ex, home, clock = rig
    ep.safety_state("DRIVING", "NONE", latched=False)
    assert home.safety["mode"] == "DRIVING"
    broker.simulate_disconnect()
    assert home.safety["mode"] == "OFFLINE" and home.safety["latched"] and home.safety["resume_requires"] == "operator"
    assert [p for p in broker.log if p.retain][-1].payload["mode"] == "OFFLINE"
    clock.t += SAFETY_MAX_AGE_MS + 1                                               # 時間が経ってから購読
    late = HomeAiMock(broker, map_version=MAP, clock_ms=clock)
    assert late.safety is None and late.stale and late.stale[0]["mode"] == "OFFLINE"


def test_critical_kinds_notify_even_with_low_confidence(rig) -> None:
    """4: ボタン電池・磁石・薬は score の掛け算とは別枠で必ず通知できる。"""
    broker, ep, ex, home, clock = rig
    low = finding(candidates=[{"kind": "coin", "confidence": 0.6}, {"kind": "magnet", "confidence": 0.08}],
                  risk={**finding()["risk"], "score": 0.12, "mandatory_notify": True, "critical_kinds": ["magnet"]})
    ep.floor_finding(low)
    assert home.notified and "mandatory:magnet" in home.notified[-1]
    plain = finding(candidates=[{"kind": "bead", "confidence": 0.9}],
                    risk={**finding()["risk"], "score": 0.12, "mandatory_notify": False, "critical_kinds": []})
    ep.floor_finding(plain)
    assert len(home.notified) == 1                                                 # 低スコアで危険物でなければ通知しない
    bad = finding(risk={**finding()["risk"], "critical_kinds": ["coin"]})
    assert validate_event({**ep._base("floor_finding"), "finding": bad})           # critical_kinds の語彙


def test_resolved_or_dismissed_only_by_operator(rig) -> None:
    """5: resolved / dismissed は人の操作だけ。ロボットの再訪は reobservations に留める。"""
    broker, ep, ex, home, clock = rig
    assert finding_rule_errors(finding(state="resolved", state_by="robot"))
    assert finding_rule_errors(finding(state="dismissed", state_by="robot"))
    assert finding_rule_errors(finding(state="resolved", state_by="operator")) == []
    with pytest.raises(ValueError):
        ep.floor_finding(finding(state="resolved", state_by="robot"))
    msg = ep.floor_finding(finding(state="candidate", state_by="robot",
                                   reobservations=[{"t_ms": 9000, "seen": False, "pose_sigma_xy_m": 0.12}]))
    assert msg["finding"]["state"] == "candidate" and not msg["finding"]["reobservations"][0]["seen"]
    ep.floor_finding(finding(state="resolved", state_by="operator"))


def test_events_reach_home_ai_and_endpoint_refuses_invalid(rig) -> None:
    broker, ep, ex, home, clock = rig
    ep.safety_state("FAULT_HOLD", "HEARTBEAT_LOST", latched=False, telemetry_age_ms=120)
    ep.battery(7.6, low=False, percent_est=60.0)
    msg = ep.floor_finding(finding())
    assert msg["data_source"] == "SIMULATION" and msg["source"] == "serpens"
    assert [e["event"] for e in home.events] == ["safety_state", "battery", "floor_finding"]
    assert [p.topic for p in broker.log if p.retain] == [event_topic("safety_state")]
    with pytest.raises(ValueError):
        ep.emit("safety_state", mode="RUNNING", stop_reason="NONE", latched=False, resume_requires="operator")
    with pytest.raises(ValueError):
        ep.floor_finding(finding(frame_id="room"))


def test_loopback_topic_matching_and_json_only() -> None:
    b = LoopbackBroker()
    got: list[str] = []
    b.subscribe("home/serpens/event/+", lambda t, _m, _r: got.append(t))
    b.subscribe("home/serpens/#", lambda t, _m, _r: got.append("all:" + t))
    b.publish("home/serpens/event/battery", {"x": 1})
    b.publish("home/serpens/task", {"x": 1})
    assert got == ["home/serpens/event/battery", "all:home/serpens/event/battery", "all:home/serpens/task"]
    with pytest.raises(TypeError):
        b.publish("home/serpens/task", {"x": object()})
