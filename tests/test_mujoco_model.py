"""MuJoCo モデル（Stage G/H）。**MuJoCo が入っていなければ丸ごと skip。**

`serpens/` 本体は MuJoCo に依存しない（モックファーストを崩さない）。
入れ方: `.venv/Scripts/python.exe -m pip install -r requirements-sim3d.txt`

ここで確かめるのは「モデルが config と一致していること」と
「物理が物理として振る舞うこと」。**現実と一致することは確かめていない**（MUJOCO_SIM）。
"""
from __future__ import annotations

import math

import pytest

from serpens.config import load_config
from serpens.motion.gait import GaitParams, body_joint_names

mujoco = pytest.importorskip("mujoco", reason="MuJoCo は任意依存（requirements-sim3d.txt）")

from simulation.mujoco.belly import PROFILES, profile  # noqa: E402
from simulation.mujoco.model import VALUE_SOURCES, build_mjcf  # noqa: E402
from simulation.mujoco.runner import SOURCE, run_gait  # noqa: E402

FORWARD = GaitParams(30.0, 60.0, 0.5)


@pytest.fixture()
def cfg() -> dict:
    return load_config()


@pytest.fixture(scope="module")
def spec_and_model():
    cfg = load_config()
    spec = build_mjcf(cfg, "WHEEL")
    return spec, mujoco.MjModel.from_xml_string(spec.xml)


# ---- モデルが config と一致しているか ----------------------------------------------------
def test_model_has_the_real_nine_axes(cfg: dict, spec_and_model) -> None:
    """**9軸の構成はコードから取る。** 胴体ヨー6 + 首ピッチ + 頭ヨー + 頭ロール。"""
    spec, model = spec_and_model
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]
    assert names[0] == "root", "土台の自由関節が無い"
    assert names[1:] == [j["name"] for j in cfg["joints"]]
    axes = {j["name"]: j["axis"] for j in cfg["joints"]}
    assert [axes[n] for n in body_joint_names(cfg)] == ["yaw"] * 6
    assert [axes[n] for n in ("J7", "J8", "J9")] == ["pitch", "yaw", "roll"]


def test_joint_ranges_match_the_software_operational_limit(cfg: dict, spec_and_model) -> None:
    """可動域は **software_operational_limit（±50° CONDITIONAL）** から作る。"""
    spec, model = spec_and_model
    for j in cfg["joints"]:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j["name"])
        lo, hi = model.jnt_range[jid]
        assert math.degrees(lo) == pytest.approx(float(j["min_deg"]), abs=0.05)
        assert math.degrees(hi) == pytest.approx(float(j["max_deg"]), abs=0.05)


def test_mass_matches_the_budget(cfg: dict, spec_and_model) -> None:
    """総質量は `mass_budget_g` と一致（**収支であって計量値ではない**）。"""
    spec, model = spec_and_model
    b = cfg["mass_budget_g"]
    want_kg = (b["servo_each"] * b["servo_count"] + b["segment_frame_each"] * b["segment_frame_count"]
               + b["passive_wheels_total"] + b["head_total"] + b["skin_and_wiring"]) / 1000.0
    assert spec.total_mass_kg == pytest.approx(want_kg, abs=1e-6)
    assert float(sum(model.body_mass)) == pytest.approx(want_kg, abs=1e-3)
    assert want_kg <= float(cfg["safety_limits"]["mass_total_g_max"]) / 1000.0


def test_actuator_force_is_capped_by_the_software_torque_limit(cfg: dict, spec_and_model) -> None:
    """**アクチュエータのトルク上限 = software_torque_limit。** REFERENCE 値由来。"""
    spec, model = spec_and_model
    limit = float(cfg["safety_limits"]["torque"]["software_torque_limit_nm"])
    for i in range(model.nu):
        lo, hi = model.actuator_forcerange[i]
        assert hi == pytest.approx(limit, abs=1e-3) and lo == pytest.approx(-limit, abs=1e-3)


def test_value_sources_are_declared(cfg: dict) -> None:
    """**モデルの値がどこから来たかを明示している。** CONFIRMED は現在 0 件。"""
    assert set(VALUE_SOURCES.values()) <= {"CONFIRMED", "REFERENCE_ONLY", "UNKNOWN"}
    assert "CONFIRMED" not in VALUE_SOURCES.values(), "実測していないのに CONFIRMED がある"
    assert VALUE_SOURCES["friction"] == "UNKNOWN"
    assert VALUE_SOURCES["servo_dynamics"] == "UNKNOWN"


def test_model_is_reproducible(cfg: dict) -> None:
    """同じ config からは同じモデル（指紋が一致）。記録の再現に要る。"""
    a, b = build_mjcf(cfg, "WHEEL"), build_mjcf(cfg, "WHEEL")
    assert a.digest == b.digest
    assert build_mjcf(cfg, "SNAKE_ANISOTROPIC").digest != a.digest


# ---- 物理として妥当か（**現実と一致するかではない**） --------------------------------------
def test_results_carry_their_source(cfg: dict) -> None:
    """結果に source / モデル指紋 / seed が付く（簡易シミュレータの値と混ぜないため）。"""
    r = run_gait(cfg, "WHEEL", FORWARD, seconds=2.0, seed=7)
    assert r.source == SOURCE == "MUJOCO_SIM"
    assert r.seed == 7 and len(r.model_digest) == 16 and r.model_version.startswith("serpens-mjcf")
    assert any("未実測" in n for n in r.notes)


def test_same_seed_gives_the_same_result(cfg: dict) -> None:
    a = run_gait(cfg, "WHEEL", FORWARD, seconds=2.0, seed=3)
    b = run_gait(cfg, "WHEEL", FORWARD, seconds=2.0, seed=3)
    assert a.forward_mm == pytest.approx(b.forward_mm, abs=1e-6)


def test_it_does_not_fall_over_while_walking(cfg: dict) -> None:
    r = run_gait(cfg, "WHEEL", FORWARD, seconds=4.0, seed=1)
    assert not r.fell_over
    assert r.head_height_std_mm < 20.0, "頭が跳ねている"


def test_torque_never_exceeds_the_limit(cfg: dict) -> None:
    """**上限を超えるトルクは出ない**（アクチュエータの forcerange で物理的に抑える）。"""
    limit = float(cfg["safety_limits"]["torque"]["software_torque_limit_nm"])
    r = run_gait(cfg, "WHEEL", FORWARD, seconds=3.0, seed=1)
    assert r.peak_torque_nm <= limit + 1e-6
