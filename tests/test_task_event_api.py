"""フェーズ 1（Floor Watch）: Home AI ↔ Serpens の Task / Event API。ハード不要。

  - スキーマ（schemas/*.json）に合う / 合わない例
  - 版違い・map_version 違い・未知フィールド・安全設定を変えるフィールドの拒否
  - Loopback で Task → 受理/拒否 → Event の往復。Home AI 側モックが受け取る
  - stop は常に最優先、同じ id は冪等、安全状態は retain で後から購読しても届く
"""
from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any

import pytest

from serpens.api.bridge import LoopbackBroker, event_topic
from serpens.api.endpoint import Endpoint, Executor
from serpens.api.validate import SCHEMA_DIR, validate_event, validate_task
from tests.mocks.home_ai_mock import HomeAiMock

MAP = "tags-v0"


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
                 "rationale": ["washer/button_cell 19mm", "高さあり（模様ではない）", "子どもの距離 不明 → 近いとみなす"]},
        "state": "candidate",
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
def rig() -> tuple[LoopbackBroker, Endpoint, RecordingExecutor, HomeAiMock]:
    broker = LoopbackBroker()
    ex = RecordingExecutor()
    ep = Endpoint(broker, ex, map_version=MAP, data_source="SIMULATION", clock_ms=lambda: 42)
    home = HomeAiMock(broker, map_version=MAP)
    return broker, ep, ex, home


# ---- スキーマ ------------------------------------------------------------------------
def test_schema_files_are_valid_json_and_reference_each_other() -> None:
    for name in ("common.json", "task.json", "event.json"):
        json.loads((SCHEMA_DIR / name).read_text(encoding="utf-8"))
    assert "safety" not in json.dumps(json.loads((SCHEMA_DIR / "task.json").read_text(encoding="utf-8"))["properties"]).lower()


def test_valid_tasks_pass_and_kind_rules_apply() -> None:
    base = {"v": 1, "id": "t-00000001", "t_ms": 0, "source": "home_ai"}
    assert validate_task({**base, "task": "inspect_point", "frame_id": "home", "map_version": MAP,
                          "target": {"x_m": 1.0, "y_m": 0.0, "yaw_rad": 0.0}}) == []
    assert validate_task({**base, "task": "stop", "reason": "test"}) == []
    assert validate_task({**base, "task": "return_dock"}) == []
    assert validate_task({**base, "task": "inspect_point"})                       # target が無い
    assert validate_task({**base, "task": "patrol_route", "frame_id": "home", "map_version": MAP, "waypoints": []})
    assert validate_task({**base, "v": 2, "task": "stop"})                         # 版違い
    assert validate_task({**base, "task": "stop", "max_speed_mm_s": 999})         # 安全設定は受け付けない
    assert validate_task({**base, "task": "inspect_point", "frame_id": "room", "map_version": MAP,
                          "target": {"x_m": 1.0, "y_m": 0.0, "yaw_rad": 0.0}})     # frame は home だけ
    assert validate_task({**base, "task": "highlight_point", "frame_id": "home", "map_version": MAP,
                          "target": {"x_m": 1.0, "y_m": 0.0, "yaw_rad": 9.0}})     # yaw の範囲


def test_floor_finding_schema_covers_the_required_fields() -> None:
    base = {"v": 1, "id": "e-00000001", "t_ms": 0, "source": "serpens", "event": "floor_finding", "data_source": "SIMULATION"}
    assert validate_event({**base, "finding": finding()}) == []
    bad = finding()
    del bad["risk"]["rationale"]
    assert validate_event({**base, "finding": bad})
    assert validate_event({**base, "finding": finding(photos=[])})
    assert validate_event({**base, "finding": finding(state="found")})
    assert validate_event({**base})                                                # finding が無い
    assert validate_event({**base, "finding": finding(), "data_source": "REAL"})   # 出どころの語彙


# ---- 往復 ------------------------------------------------------------------------------
def test_task_roundtrip_accept_reject_and_idempotent(rig) -> None:
    broker, ep, ex, home = rig
    tid = home.task("inspect_point", target={"x_m": 1.0, "y_m": 0.2, "yaw_rad": 0.0})
    assert home.statuses_of(tid) == ["accepted"] and ex.started[0]["id"] == tid
    home.task("inspect_point", task_id=tid, target={"x_m": 1.0, "y_m": 0.2, "yaw_rad": 0.0})   # 再送
    assert home.statuses_of(tid) == ["accepted", "accepted"] and len(ex.started) == 1          # 2 回目は実行しない
    bad = home.task("inspect_point", target={"x_m": 1.0, "y_m": 0.2, "yaw_rad": 0.0}, map_version="tags-v9")
    assert home.statuses_of(bad) == ["rejected"]
    assert any("map_version" in e.get("reason", "") for e in home.events if e["event"] == "task_status")
    broken = home.task("inspect_point")                                            # target 無し
    assert home.statuses_of(broken) == ["rejected"]
    assert not home.schema_errors


def test_blockers_reject_without_starting(rig) -> None:
    broker, _ep, _ex, home = rig
    ex2 = RecordingExecutor(blockers=["電気安全ゲート未完了: physical_estop", "自己位置が未取得"])
    Endpoint(LoopbackBroker(), ex2, map_version=MAP)     # 別ブローカーで独立に
    b2 = LoopbackBroker()
    home2 = HomeAiMock(b2, map_version=MAP)
    Endpoint(b2, ex2, map_version=MAP)
    tid = home2.task("patrol_route", waypoints=[{"x_m": 0.5, "y_m": 0.0, "yaw_rad": 0.0}])
    assert home2.statuses_of(tid) == ["rejected"] and not ex2.started
    assert "physical_estop" in [e for e in home2.events if e["event"] == "task_status"][-1]["reason"]


def test_stop_is_always_accepted_and_aborts_the_active_task(rig) -> None:
    broker, ep, ex, home = rig
    ex._blockers = ["自己位置が未取得"]
    t1 = home.task("inspect_point", target={"x_m": 1.0, "y_m": 0.0, "yaw_rad": 0.0})
    assert home.statuses_of(t1) == ["rejected"]
    ex._blockers = []
    t2 = home.task("inspect_point", target={"x_m": 1.0, "y_m": 0.0, "yaw_rad": 0.0})
    assert ep.active_task_id == t2
    s = home.task("stop", reason="operator")
    assert home.statuses_of(s) == ["accepted"] and ex.stopped == ["operator"] and ep.active_task_id is None
    t3 = home.task("highlight_point", target={"x_m": 1.0, "y_m": 0.0, "yaw_rad": 0.0}, duration_s=5)
    t4 = home.task("return_dock")
    assert home.statuses_of(t3) == ["accepted", "aborted"] and home.statuses_of(t4) == ["accepted"]


def test_events_reach_home_ai_and_safety_state_is_retained(rig) -> None:
    broker, ep, _ex, home = rig
    ep.safety_state("FAULT_HOLD", "HEARTBEAT_LOST", latched=False, telemetry_age_ms=120)
    ep.battery(7.6, low=False, percent_est=60.0)
    msg = ep.floor_finding(finding())
    assert home.notified and "f-00000001" in home.notified[0]
    assert [e["event"] for e in home.events] == ["safety_state", "battery", "floor_finding"]
    assert msg["data_source"] == "SIMULATION" and msg["source"] == "serpens"
    late = HomeAiMock(broker, map_version=MAP)                                     # 後から購読
    assert [e["event"] for e in late.events] == ["safety_state"]                   # retain で最後の安全状態が届く
    assert late.events[0]["resume_requires"] == "operator"
    retained = [p for p in broker.log if p.retain]
    assert [p.topic for p in retained] == [event_topic("safety_state")]


def test_endpoint_refuses_to_emit_invalid_events(rig) -> None:
    _broker, ep, _ex, _home = rig
    with pytest.raises(ValueError):
        ep.emit("safety_state", mode="RUNNING", stop_reason="NONE", latched=False, resume_requires="operator")
    with pytest.raises(ValueError):
        ep.floor_finding(finding(frame_id="room"))


def test_loopback_topic_matching_and_json_only() -> None:
    b = LoopbackBroker()
    got: list[str] = []
    b.subscribe("home/serpens/event/+", lambda t, _m: got.append(t))
    b.subscribe("home/serpens/#", lambda t, _m: got.append("all:" + t))
    b.publish("home/serpens/event/battery", {"x": 1})
    b.publish("home/serpens/task", {"x": 1})
    assert got == ["home/serpens/event/battery", "all:home/serpens/event/battery", "all:home/serpens/task"]
    with pytest.raises(TypeError):
        b.publish("home/serpens/task", {"x": object()})


def test_task_schema_has_no_safety_settings() -> None:
    """安全は機体側で完結し外から変えられない: Task に速度・角度・TTL・トルクの語が無い。"""
    text = (SCHEMA_DIR / "task.json").read_text(encoding="utf-8").lower()
    props = json.loads(text)["properties"]
    for word in ("speed", "torque", "ttl", "limit", "angle_deg", "heartbeat"):
        assert not any(word in k for k in props), word
