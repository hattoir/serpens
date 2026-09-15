"""安全の絶対値が守られていることを、設定と実装から機械的に確かめる。

出典は `../蛇ロボット_HomeAI_構想設計書.pdf` 16章「安全性 — 4重の防御」。
導出と、達成できていない項目は `docs/safety_limits.md`。

**ここは「そのうち直す」テストではない。** 落ちたら、設定を戻すか設計を見直す。
"""
from __future__ import annotations

import copy
import math

import pytest

from serpens.config import load_config
from serpens.hw.mock_bus import MockServoBus
from serpens.motion.gait import BODY_AXIS, body_joint_names
from serpens.safety import AutonomyInputs, autonomy_blockers
from serpens.sim.session import SimSession
from serpens.sim.world import BodyPose
from tests.helpers import FakeClock

KGFCM_TO_NM = 0.0980665


@pytest.fixture()
def cfg() -> dict:
    return load_config()


@pytest.fixture()
def lim(cfg: dict) -> dict:
    return cfg["safety_limits"]


# ---- 層1: 本質安全（機構） -------------------------------------------------------------
def test_joint_range_keeps_minimum_bend_radius(cfg: dict, lim: dict) -> None:
    """同じ軸が連なる部分の最小曲げ半径が 40mm 以上（人に巻き付けない）。

    等しい関節角 θ、リンク長 L の鎖は半径 R = L / (2·tan(θ/2)) の円に沿う。
    可動域を広げるとここが落ちる。
    """
    joints = cfg["joints"]
    worst = math.inf
    for a, b in zip(joints, joints[1:]):
        if a["axis"] != b["axis"]:
            continue                      # 軸が違う関節は輪を閉じられない（頭部の J7/J8/J9）
        link_mm = float(b["x_mm"] - a["x_mm"])
        theta = math.radians(max(abs(float(a["min_deg"])), abs(float(a["max_deg"]))))
        worst = min(worst, link_mm / (2.0 * math.tan(theta / 2.0)))
    assert worst >= float(lim["min_bend_radius_mm"]), (
        f"最小曲げ半径 {worst:.1f}mm < {lim['min_bend_radius_mm']}mm。"
        "可動域を狭めるか、リンクを長くすること（機構担当と相談）")


def test_free_loop_diameter_meets_design_document(cfg: dict, lim: dict) -> None:
    """ヘビが作れる**自由な穴の直径**が 80mm 以上（指も手首も通せない輪を作れない）。

    構想設計書 16章は「最小の輪の内径 80mm 以上」。中心線の曲率半径 R から胴体半径を引いて出す。
    """
    joints = cfg["joints"]
    r_body = float(cfg["body"]["diameter_mm"]) / 2.0
    worst = math.inf
    for a, b in zip(joints, joints[1:]):
        if a["axis"] != b["axis"]:
            continue
        link_mm = float(b["x_mm"] - a["x_mm"])
        theta = math.radians(max(abs(float(a["min_deg"])), abs(float(a["max_deg"]))))
        worst = min(worst, link_mm / (2.0 * math.tan(theta / 2.0)))
    hole_mm = 2.0 * (worst - r_body)
    assert hole_mm >= 80.0, f"輪の内径 {hole_mm:.0f}mm < 80mm（可動域を狭めること）"


def test_operational_limits_stay_inside_cad_verified_range(cfg: dict) -> None:
    """operational ⊆ mechanical ⊆ geometry。**CAD の干渉検査を超えた角度を出せない。**"""
    from serpens.config import check_joint_limits

    check_joint_limits(cfg)                       # 現物は通る
    bad = copy.deepcopy(cfg)
    bad["joints"][0]["max_deg"] = bad["joints"][0]["mechanical_max_deg"] + 1.0
    with pytest.raises(ValueError, match="software_operational_limit"):
        check_joint_limits(bad)
    worse = copy.deepcopy(cfg)
    worse["joints"][0]["mechanical_max_deg"] = float(worse["joints"][0]["onset_deg"]) + 1.0
    with pytest.raises(ValueError, match="干渉"):
        check_joint_limits(worse)
    clamp = copy.deepcopy(cfg)
    clamp["joint_limit_policy"]["clamp_source"] = "geometry_collision_onset"
    with pytest.raises(ValueError, match="clamp_source"):
        check_joint_limits(clamp)


def test_body_limits_match_the_cad_proposal(cfg: dict) -> None:
    """胴体ヨーの3つの可動域が CAD R03 の最新提案と一致する。

    geometry_collision_onset は 64.8° < onset <= 64.9°（**clamp には使わない**）、
    mechanical_design_limit ±55°（PROVISIONAL）、software_operational_limit ±50°（CONDITIONAL）。
    """
    policy = cfg["joint_limit_policy"]
    assert policy["onset_bracket_deg"] == [64.8, 64.9]
    assert policy["status"]["mechanical_design_limit"] == "PROVISIONAL"
    assert policy["status"]["software_operational_limit"] == "CONDITIONAL"
    assert policy["verified_in_cad"] is True
    assert policy["verified_with_cable"] is False and policy["verified_with_hardware"] is False
    body = [j for j in cfg["joints"] if j["name"] in {f"J{k+1}" for k in range(6)}]
    assert body, "胴体ヨーが見つからない"
    for j in body:
        assert float(j["onset_deg"]) == 64.8 and float(j["onset_collides_deg"]) == 64.9
        assert abs(float(j["mechanical_max_deg"])) == 55.0
        assert abs(float(j["max_deg"])) == 50.0, "clamp 値が ±50°（CONDITIONAL）でない"
        assert abs(float(j["max_deg"])) < abs(float(j["onset_deg"])), "干渉境界を clamp に使っている"


def test_legacy_poses_are_not_reachable_through_normal_paths(cfg: dict) -> None:
    """**旧とぐろを clamp して「同じとぐろ」として扱わない。**

    R03 の可動域に入らない旧モーションは `legacy_poses`（SIMULATION_LEGACY_ONLY）にあり、
    通常の姿勢（`poses`）からは消えている。
    """
    from serpens.motion.poses import Poses

    assert "coil" not in cfg["poses"], "旧とぐろが通常の姿勢に残っている"
    assert cfg["legacy_poses"]["status"] == "SIMULATION_LEGACY_ONLY"
    legacy = Poses(cfg).legacy("coil")
    body_max = max(abs(float(j["max_deg"])) for j in cfg["joints"][:6])
    assert max(legacy[f"J{k+1}"] for k in range(6)) > body_max, "legacy が可動域内に収まっている"
    rest = Poses(cfg).rest()
    assert all(abs(rest[f"J{k+1}"]) <= body_max for k in range(6))


def test_mass_budget_within_limit(cfg: dict, lim: dict) -> None:
    """総質量 1.7kg 上限（落下・衝突時の運動エネルギーの上限）。"""
    b = cfg["mass_budget_g"]
    total = (b["servo_each"] * b["servo_count"] + b["segment_frame_each"] * b["segment_frame_count"]
             + b["passive_wheels_total"] + b["head_total"] + b["skin_and_wiring"])
    assert total <= float(lim["mass_total_g_max"]), f"質量収支 {total}g が上限を超えた"


def test_gait_cannot_wrap_around_anything(cfg: dict) -> None:
    """歩容が動かすのは**水平ヨーだけ**。螺旋（巻き付き）歩容を作れない。

    構想設計書 16章「螺旋歩容のコードを存在させない」。上下に波を伝えれば巻き付けるので、
    歩容の対象にピッチ・ロール軸が入った時点でこの保証が壊れる。
    """
    axes = {j["name"]: j["axis"] for j in cfg["joints"]}
    body = body_joint_names(cfg)
    assert body, "胴体の歩容関節が無い"
    assert all(axes[n] == BODY_AXIS for n in body), f"歩容に {BODY_AXIS} 以外の軸が入った: {body}"


# ---- 層2/3: 電気・ファームの上限 --------------------------------------------------------
def test_torque_model_keeps_reference_and_measured_apart(cfg: dict, lim: dict) -> None:
    """**REFERENCE 値と実測値を混ぜない。** ストールトルクを連続安全トルクとして使わない。"""
    t = lim["torque"]
    assert t["source"] == "SIMULATED_FROM_REFERENCE"
    assert t["hardware_verified"] is False
    assert t["measured_safe_torque_nm"] is None, "実測していないのに値が入っている"
    rated = float(t["rated_torque_reference_nm"])
    stall = float(t["stall_torque_reference_nm"])
    soft = float(t["software_torque_limit_nm"])
    assert soft <= rated, "ソフト上限が定格を超えている（ストールを常用値にしている疑い）"
    assert soft <= float(lim["torque_nm_max"]), "構想設計書16章の 1.2N·m を超えている"
    assert float(t["software_torque_limit_ratio"]) == pytest.approx(soft / stall, abs=0.005)


def test_c044_profile_is_not_polluted_by_the_12v_servo(cfg: dict) -> None:
    """**旧 12V 1:345 の値が C044 プロファイルへ混ざっていない。**"""
    s = cfg["servo"]
    assert s["model"] == "STS3215-C044"
    assert s["nominal_voltage_v"] == pytest.approx(7.4) and s["gear_ratio"] == "1:191"
    assert s["hardware_verified"] is False
    assert s["torque_reference_source"] == "CAD_REFERENCE"
    assert "stall_torque_kgfcm" not in s, "旧 12V のストールトルクが残っている"
    assert float(s["rated_torque_reference_kgfcm"]) == pytest.approx(5.2)
    assert float(s["stall_torque_reference_kgfcm"]) == pytest.approx(16.0)
    assert float(s["supply_voltage_v"]) == pytest.approx(7.4), "電源電圧が 12V のまま"


def test_torque_limit_still_lifts_the_head(cfg: dict, lim: dict) -> None:
    """上限を掛けても、首を持ち上げる分のトルクは残る（**REFERENCE 値での見積り**）。"""
    b = cfg["mass_budget_g"]
    need_nm = b["neck_lifted_mass"] / 1000.0 * 9.81 * b["neck_lifted_cog_mm"] / 1000.0
    available_nm = float(lim["torque"]["software_torque_limit_nm"])
    assert available_nm > need_nm * 2.0, (
        f"首の必要トルク {need_nm:.2f}N·m に対し上限が {available_nm:.2f}N·m しかない")


def test_torque_limit_cannot_exceed_ceiling(cfg: dict, lim: dict) -> None:
    """脱力演出の経路から全力へ戻せない（`set_torque_limit` は上限に対する割合）。"""
    bus = MockServoBus(cfg, clock=FakeClock())
    bus.connect()
    bus.apply_torque_ceiling()
    ceiling = float(lim["torque"]["software_torque_limit_ratio"])
    for asked in (1.0, 2.0, 10.0):
        bus.set_torque_limit(1, asked)
        assert bus._axes[1].torque_ratio == pytest.approx(ceiling)


def test_session_applies_ceiling_and_blocks_without_it(cfg: dict, lim: dict) -> None:
    """セッションが接続先へ必ず上限を書く。書けていない機体は自律走行させない。"""
    s = SimSession(cfg, BodyPose(150.0, 600.0, 0.0), seed=1)
    assert s.bus.torque_ceiling_applied and not s.errors
    assert s.bus._axes[1].torque_ratio == pytest.approx(
        float(lim["torque"]["software_torque_limit_ratio"]))
    no_ceiling = AutonomyInputs(True, "aruco", 0.1, True, True, False, 9, 9, 0.2)
    assert any("トルク上限" in w for w in autonomy_blockers(cfg, no_ceiling))


def test_stop_thresholds_are_stricter_than_the_document(cfg: dict, lim: dict) -> None:
    """温度・通信断は、構想設計書の目標より手前で止める。"""
    assert float(cfg["link"]["faults"]["temp_limit_c"]) < float(lim["servo_temp_stop_c"])
    assert float(cfg["behavior"]["safety"]["overheat_c"]) < float(cfg["link"]["faults"]["temp_limit_c"])
    hold_ms = float(cfg["link"]["heartbeat_timeout_ms"])
    assert hold_ms < float(lim["comms_loss_safe_ms"]), "通信断からの保持が遅すぎる"
    assert float(cfg["link"]["drive_ttl_ms"]) < hold_ms, "TTL は heartbeat より先に効くこと"


# ---- 層4: ソフト（人の近く） ------------------------------------------------------------
def test_speed_limit_near_person(cfg: dict, lim: dict) -> None:
    """人の半径1m以内で 8cm/s を超えない。行動側の設定が絶対値を超えていないこと。"""
    c = cfg["behavior"]["controller"]
    assert float(c["near_person_mm"]) >= float(lim["near_person_mm"])
    assert float(c["near_speed_limit_mm_s"]) <= float(lim["near_speed_limit_mm_s"])
    for key in ("speed_patrol_mm_s", "speed_approach_mm_s", "speed_retreat_mm_s"):
        assert float(c[key]) > 0.0


def test_contact_release_gap_is_recorded(cfg: dict, lim: dict) -> None:
    """接触 → 脱力の目標 20ms に対し、現状は PC 側の検知で 800ms。**未達を明示して残す。**

    実機では機体側（ESP32）が負荷を見て切る必要がある。docs/safety_limits.md の未達項目。
    """
    now_ms = float(cfg["behavior"]["safety"]["grab_hold_s"]) * 1000.0
    assert now_ms > float(lim["contact_release_ms"]), (
        "20ms を達成したなら docs/safety_limits.md とこのテストを更新すること")
