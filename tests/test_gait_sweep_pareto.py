"""歩容の候補選び（Stage I）。**単一スコアで1つに決めない**ことを固定する。

Pareto の判定は純関数なので MuJoCo 無しで試せる。
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

from gait_sweep_mujoco import pareto, snake_like  # noqa: E402


def row(forward: float, energy: float, accel: float, lateral: float = 0.0,
        sat: float = 0.0, head: float = 1.0) -> dict:
    return {"forward_mm": forward, "energy_j": energy, "joint_accel_rms_dps2": accel,
            "abs_lateral_mm": lateral, "torque_saturation": sat, "head_height_std_mm": head}


def test_dominated_candidates_are_dropped() -> None:
    """全部の指標で負けている候補は残さない。"""
    good = row(1000.0, 2.0, 100.0)
    bad = row(500.0, 5.0, 300.0)                  # どの指標でも good に負けている
    assert pareto([good, bad]) == [good]


def test_trade_offs_are_kept() -> None:
    """**速いが重い**と**遅いが軽い**は、どちらも残す（単一スコアで潰さない）。"""
    fast_heavy = row(1000.0, 8.0, 400.0)
    slow_light = row(400.0, 1.0, 80.0)
    front = pareto([fast_heavy, slow_light])
    assert len(front) == 2


def test_snake_likeness_prefers_straight_and_steady() -> None:
    """横ずれが小さく、頭が上下しないほど「蛇らしい」（**代用値**）。"""
    straight = row(1000.0, 3.0, 100.0, lateral=10.0, head=0.5)
    wobbly = row(1000.0, 3.0, 100.0, lateral=300.0, head=8.0)
    assert snake_like(straight) < snake_like(wobbly)
