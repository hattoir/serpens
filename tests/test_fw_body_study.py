"""Floor Watch 身体構成の比較（`simulation/floorwatch_body.py`）。**MuJoCo が無ければ skip。**

確かめるのは「overlay がモデルになり、結果に source が付く」ことだけ。
**数値が現実と一致するかは確かめていない**（摩擦・質量・サーボ応答はすべて未実測）。
"""
from __future__ import annotations

import math

import pytest

pytest.importorskip("mujoco", reason="MuJoCo は任意依存（requirements-sim3d.txt）")

from serpens.motion.gait import body_joint_names  # noqa: E402
from simulation.floorwatch_body import CONFIGS, fw_config, neck_static_torque_nm, run_case  # noqa: E402
from simulation.mujoco.belly import profile  # noqa: E402
from simulation.mujoco.model import build_mjcf  # noqa: E402


@pytest.mark.parametrize("name,body_axes,motors", [("FW5", 4, 5), ("FW6_YAW5", 5, 6), ("FW6_HEADYAW", 4, 6)])
def test_overlays_build_with_the_intended_topology(name: str, body_axes: int, motors: int) -> None:
    cfg = fw_config(name)
    assert len(cfg["joints"]) == motors == cfg["mass_budget_g"]["servo_count"]
    assert len(body_joint_names(cfg)) == body_axes          # 首 pitch より尾側の yaw だけが歩容に使われる
    spec = build_mjcf(cfg, "SNAKE_ANISOTROPIC")
    b = cfg["mass_budget_g"]
    want = (b["servo_each"] * b["servo_count"] + b["segment_frame_each"] * b["segment_frame_count"]
            + b["passive_wheels_total"] + b["head_total"] + b["skin_and_wiring"] + b["tail_payload"]) / 1000.0
    assert spec.total_mass_kg == pytest.approx(want, abs=1e-6)
    assert 'contype="0"' in spec.xml                           # 太いカプセルの自己接触は切ってある


def test_study_rows_carry_their_source() -> None:
    row = run_case("FW5", profile("SNAKE_ANISOTROPIC"), seconds=2.0)
    assert row.source == "MUJOCO_SIM"
    assert row.peak_torque_nm <= row.torque_limit_nm + 1e-6


def test_neck_static_torque_is_m_g_d_cos() -> None:
    assert neck_static_torque_nm(100.0, 50.0, 0.0) == pytest.approx(0.1 * 9.81 * 0.05)
    assert neck_static_torque_nm(100.0, 50.0, 60.0) == pytest.approx(0.1 * 9.81 * 0.05 * math.cos(math.radians(60)))


def test_all_configs_are_listed() -> None:
    assert set(CONFIGS) == {"FW5", "FW6_YAW5", "FW6_HEADYAW"}
