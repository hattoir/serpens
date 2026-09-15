"""Belly（腹側）の物理と、サーボゲインへの感度（Stage H）。

**ここで確かめるのは「仮の摩擦係数のもとで何が起きるか」であって、現実ではない。**
既存の簡易シミュレータ（KINEMATIC_SIM）の数値に合わせ込むこともしない。
"""
from __future__ import annotations

import pytest

from serpens.config import load_config
from serpens.motion.gait import GaitParams

pytest.importorskip("mujoco", reason="MuJoCo は任意依存（requirements-sim3d.txt）")

from simulation.mujoco.belly import PROFILES, profile  # noqa: E402
from simulation.mujoco.runner import run_gait  # noqa: E402

FORWARD = GaitParams(30.0, 60.0, 0.5)
SECONDS = 4.0


@pytest.fixture()
def cfg() -> dict:
    return load_config()


def test_three_belly_profiles_exist() -> None:
    assert set(PROFILES) == {"WHEEL", "SNAKE_ISOTROPIC", "SNAKE_ANISOTROPIC"}
    assert profile("SNAKE_ANISOTROPIC").anisotropy > 3.0
    assert profile("SNAKE_ISOTROPIC").anisotropy == pytest.approx(1.0)


def test_isotropic_friction_cannot_propel(cfg: dict) -> None:
    """**等方摩擦では蛇行で進めない。** 横に滑らないことが推進の条件。

    簡易シミュレータ（KINEMATIC_SIM）でも同じ結論だったが、**別のモデルで独立に出た**。
    """
    iso = run_gait(cfg, "SNAKE_ISOTROPIC", FORWARD, seconds=SECONDS, seed=1)
    assert abs(iso.forward_mm) < 100.0, f"等方なのに {iso.forward_mm:.0f}mm 進んだ"


def test_anisotropy_creates_propulsion(cfg: dict) -> None:
    """鱗の異方性を入れると前へ進む（前後に滑り、横に食いつく）。"""
    iso = run_gait(cfg, "SNAKE_ISOTROPIC", FORWARD, seconds=SECONDS, seed=1)
    aniso = run_gait(cfg, "SNAKE_ANISOTROPIC", FORWARD, seconds=SECONDS, seed=1)
    assert aniso.forward_mm > iso.forward_mm + 100.0


def test_wheels_beat_scales_under_these_assumptions(cfg: dict) -> None:
    """仮の係数のもとでは受動輪が最も進む。**実機で確かめるべき仮説。**"""
    wheel = run_gait(cfg, "WHEEL", FORWARD, seconds=SECONDS, seed=1)
    aniso = run_gait(cfg, "SNAKE_ANISOTROPIC", FORWARD, seconds=SECONDS, seed=1)
    assert wheel.forward_mm > aniso.forward_mm


def test_servo_gain_changes_the_answer(cfg: dict) -> None:
    """**サーボの応答（未同定）で前進量が大きく変わる。**

    kp を 1/2 にすると追従誤差は増えるのに前進量は増える（柔らかいほうが進む）。
    つまり **MuJoCo の数値は、実機のサーボ応答を同定するまで予測として使えない**。
    実機が来たら最初に同定する項目（`docs/verification_status.md`）。
    """
    soft = run_gait(cfg, "WHEEL", FORWARD, seconds=SECONDS, seed=1, kp_scale=0.5)
    stiff = run_gait(cfg, "WHEEL", FORWARD, seconds=SECONDS, seed=1, kp_scale=2.0)
    assert soft.tracking_error_rms_deg > stiff.tracking_error_rms_deg
    assert soft.forward_mm > stiff.forward_mm * 1.2, "ゲインの影響が消えている"
    assert stiff.torque_saturation > soft.torque_saturation


def test_turning_uses_gamma_only(cfg: dict) -> None:
    """旋回はオフセット γ だけで起きる（歩容の式は簡易シミュレータと同じ）。"""
    straight = run_gait(cfg, "WHEEL", FORWARD, seconds=SECONDS, seed=1)
    turned = run_gait(cfg, "WHEEL", FORWARD, gamma_deg=20.0, seconds=SECONDS, seed=1)
    assert abs(turned.turn_deg) > abs(straight.turn_deg) + 20.0
