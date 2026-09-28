"""平面・準静的の摩擦モデル（`simulation/planar_friction.py`）と Hardware Gap の道具。

確かめるのは「モデルが物理として筋が通っていること」と「道具が CSV を正しく読むこと」。
**現実の床やサーボと一致することは確かめていない**（PLANAR_FRICTION_SIM / ACTUATOR_MODEL_SIM）。
CSV の値は試験用の作り物（FAKE）。
"""
from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from simulation.floorwatch_body import fw_config
from simulation.planar_friction import Friction, PlanarSnake, body_from_cfg

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope="module")
def body_a():
    return body_from_cfg(fw_config("FW6_YAW5"), "FW6_YAW5")


def _run(body, fr: Friction, amp: float = 40.0, waves: float = 0.75, gamma: float = 0.0, **kw):
    return PlanarSnake(body, fr).run_cycle(amp, waves, 0.5, gamma, steps=24, **kw)


def test_motion_depends_only_on_the_ratio(body_a) -> None:
    """全方向の μ を同じ倍率にしても動きは同じ。トルクと仕事は倍率に比例する。"""
    for law in ("decoupled", "ellipse"):
        a = _run(body_a, Friction.simple(0.1, 0.5, law=law))
        b = _run(body_a, Friction.simple(0.3, 1.5, law=law))
        assert a.converged and b.converged
        assert b.speed_mm_s == pytest.approx(a.speed_mm_s, rel=1e-6)
        assert b.peak_torque_nm == pytest.approx(3.0 * a.peak_torque_nm, rel=1e-6)


def test_isotropic_friction_gives_no_propulsion(body_a) -> None:
    """等方のクーロン摩擦では、平面の蛇行はほとんど進まない（Hu et al. 2009 と同じ結論）。"""
    r = _run(body_a, Friction.simple(0.35, 0.35))
    assert abs(r.speed_mm_s) < 5.0


def test_anisotropy_propels_and_wheels_go_fastest(body_a) -> None:
    snake = _run(body_a, Friction.simple(0.12, 0.8))
    wheel = _run(body_a, Friction.simple(0.02, 1.0))
    assert snake.speed_mm_s > 50.0
    assert wheel.speed_mm_s > snake.speed_mm_s


def test_ellipse_law_is_faster_than_decoupled(body_a) -> None:
    """同じ係数でも、法則の形で前進が大きく変わる（MODEL GAP）。"""
    d = _run(body_a, Friction.simple(0.2, 0.6))
    e = _run(body_a, Friction.simple(0.2, 0.6, law="ellipse"))
    assert e.speed_mm_s > 1.5 * d.speed_mm_s


def test_left_right_asymmetry_makes_the_body_drift(body_a) -> None:
    sym = _run(body_a, Friction(0.12, 0.12, 0.8, 0.8))
    left = _run(body_a, Friction(0.12, 0.12, 0.9, 0.7))
    right = _run(body_a, Friction(0.12, 0.12, 0.7, 0.9))
    assert abs(sym.heading_deg_per_cycle) < 0.05
    assert left.heading_deg_per_cycle * right.heading_deg_per_cycle < 0          # 逆向きにずれる
    assert abs(left.heading_deg_per_cycle) > 0.3


def test_positive_gamma_turns_left(body_a) -> None:
    st = _run(body_a, Friction.simple(0.12, 0.8), amp=30.0)
    lf = _run(body_a, Friction.simple(0.12, 0.8), amp=30.0, gamma=15.0)
    assert lf.heading_deg_per_cycle - st.heading_deg_per_cycle > 2.0


def test_record_gives_joint_series(body_a) -> None:
    rec: dict = {}
    r = _run(body_a, Friction.simple(0.12, 0.8), record=rec)
    tau = np.array(rec["torque_nm"])
    assert tau.shape == (24, 5)
    assert float(np.abs(tau).max()) == pytest.approx(r.peak_torque_nm)


def test_point_scale_must_match_the_number_of_points(body_a) -> None:
    with pytest.raises(ValueError):
        PlanarSnake(body_a, Friction(0.1, 0.1, 0.5, 0.5, point_scale=(1.0, 1.0)))


# ---- HG-H0 の道具 ---------------------------------------------------------------------------------
def _load(path: str, name: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod                       # dataclass がモジュールを引けるように
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def h0():
    return _load("simulation/hardware_gaps/HG-H0_friction/run.py", "hg_h0_run")


def _row(direction: str, angle: str, track: str = "", floor: str = "WOOD") -> dict:
    return {"date": "2026-10-01", "method": "T", "floor_id": floor, "coupon_id": "FLAT", "material": "PETG",
            "load_g": "150", "direction": direction, "trial": "1", "angle_deg": angle, "slide_track_deg": track}


def test_h0_validation_rejects_bad_rows(h0) -> None:
    good, problems = h0.validate([_row("forward", "10"), _row("sideways", "10"), _row("lateral", ""),
                                  _row("lateral", "85"), {**_row("forward", "10"), "load_g": ""}])
    assert len(good) == 1 and len(problems) == 4


def test_h0_fit_ratio_and_law_estimate(h0) -> None:
    mu_f, mu_l = math.tan(math.radians(8)), math.tan(math.radians(31))
    k = mu_f / mu_l
    track_ellipse = math.degrees(math.atan(k * k))                     # 楕円形の法則なら滑る向きは軸にほぼ沿う
    rows = ([_row("forward", a) for a in ("7.9", "8.0", "8.1")] + [_row("lateral", a) for a in ("30.8", "31.0", "31.2")]
            + [_row("backward", "9.0")] + [_row("diag45", "20", f"{track_ellipse:.2f}") for _ in range(3)])
    good, problems = h0.validate(rows)
    assert not problems
    (rec,) = h0.fit(good)
    assert rec["ratio"] == pytest.approx(mu_l / mu_f, rel=0.02)
    assert rec["ratio_ci95"][0] <= rec["ratio"] <= rec["ratio_ci95"][1]
    assert rec["law_estimate"] == "ellipse"


def test_h0_fit_flags_reversed_scales(h0) -> None:
    rows = [_row("forward", "12"), _row("forward", "12"), _row("lateral", "25"), _row("lateral", "25"), _row("backward", "8")]
    good, _ = h0.validate(rows)
    (rec,) = h0.fit(good)
    assert any("μ_back < μ_forward" in s for s in rec["issues"])


def test_h0_evaluate_respects_the_joint_limit(h0) -> None:
    """旋回に使える γ は 可動域 − 振幅。振幅 50° なら旋回できない。"""
    row = h0.evaluate(h0.Case("FW5", {"mu_f": 0.2, "mu_b": 0.2, "mu_left": 0.8, "mu_right": 0.8, "law": "decoupled"}, 50.0, 0.75))
    assert row["gamma_avail_deg"] == 0.0 and row["turn_radius_mm"] == math.inf
    assert not h0.passes(row)["turn"]


# ---- HG-H1 の道具 ---------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def h1():
    return _load("simulation/hardware_gaps/HG-H1_actuator/run.py", "hg_h1_run")


def test_h1_torque_cap_error_moves_trackability(h1) -> None:
    """同じ負荷なら、トルク上限が小さい（e = −40%）ほど追従できない確率が上がる。"""
    rng = np.random.default_rng(0)
    pr = h1.sample_priors(rng, 400)
    sc = {"tau": np.full((8, 5), 0.30), "omega": np.full((8, 5), 1.0), "n_servo": 5}
    ev = h1.evaluate(sc, pr)
    lo = np.mean(ev["margin_nm"][pr["cap_err"] == -0.4] >= 0)
    hi = np.mean(ev["margin_nm"][pr["cap_err"] == 0.4] >= 0)
    assert lo < hi
    assert np.all(ev["f_static_fail_n"] >= ev["f_static_n"])          # 上限が効かない場合の方が必ず大きい


def test_h1_ingest_recommends_a_ratio(h1, tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr(h1, "HERE", tmp_path)
    monkeypatch.setattr(h1, "ROOT", tmp_path)
    csv_path = tmp_path / "fake_h1.csv"
    head = "date,operator,servo_serial,supply_v,current_limit_a,test_id,condition,arm_mm,load_g,torque_nm_calc,current_a,load_reg,position_deg,speed_dps,temp_c,voltage_reg_v,duration_s,instrument,note\n"
    body = "".join(f"2026-10-02,FAKE,1,7.4,3,4,,100,,{t},{0.2 + 1.0 * t},,,,,,,,FAKE\n" for t in (0.1, 0.2, 0.3))
    body += "2026-10-02,FAKE,1,7.4,3,5,,100,,2.5,2.9,,,,,,,,FAKE\n"
    csv_path.write_text(head + body, encoding="utf-8")
    out = h1.ingest(csv_path)
    text = out.read_text(encoding="utf-8")
    assert "0.180" in text                                              # 0.45 / 2.5
    assert (tmp_path / "results" / "measured" / "raw").exists()
