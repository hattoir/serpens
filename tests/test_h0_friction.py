"""H0 摩擦クーポン: STL の形・CSV の読み方・判定の論理。

**ここで使う CSV の値は試験用の作り物（FAKE）で、実測ではない。**
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from tools.h0_coupons import PATTERNS, THICK_MM, coupon_mesh, signed_volume_mm3

HEADER = ("date,operator,method,floor_id,floor_note,coupon_id,material,material_brand,printer,"
          "top_surface_pattern,load_g,direction,trial,angle_deg,force_static_g,force_kinetic_g,"
          "scale_resolution_g,note\n")


def _row(method: str, floor: str, coupon: str, material: str, direction: str,
         angle: str = "", fs: str = "", fk: str = "", load: str = "150") -> str:
    return (f"2026-10-01,FAKE,{method},{floor},,{coupon},{material},,A1,concentric,{load},{direction},1,"
            f"{angle},{fs},{fk},,FAKE\n")


# ---- STL -------------------------------------------------------------------------------------
@pytest.mark.parametrize("name", list(PATTERNS))
def test_coupon_mesh_is_closed_and_outward(name: str) -> None:
    tris = coupon_mesh(PATTERNS[name], res_mm=1.0)
    edges: dict[tuple, int] = {}
    for t in np.round(tris, 4):
        for i in range(3):
            e = (tuple(t[i]), tuple(t[(i + 1) % 3]))
            edges[e] = edges.get(e, 0) + 1
    assert all(c == 1 and edges.get((b, a)) == 1 for (a, b), c in edges.items()), "閉じていない面がある"
    assert signed_volume_mm3(tris) > 0, "法線が内向き"
    z = tris[:, :, 2]
    assert z.min() == pytest.approx(0.0) and z.max() == pytest.approx(THICK_MM)   # 上面は平ら（矢印は彫り）


# ---- CSV → μ ---------------------------------------------------------------------------------
def test_mu_from_tilt_and_pull() -> None:
    from simulation.h0_friction import mu_of
    assert mu_of({"method": "T", "angle_deg": "20"}) == pytest.approx(math.tan(math.radians(20)))
    assert mu_of({"method": "P", "load_g": "150", "force_kinetic_g": "45", "force_static_g": "60"}) == pytest.approx(0.3)
    assert mu_of({"method": "P", "load_g": "150", "force_static_g": "60"}) == pytest.approx(0.4)
    assert mu_of({"method": "T", "angle_deg": ""}) is None          # 空欄は推測で埋めない


def test_load_groups_and_ratios(tmp_path: Path) -> None:
    from simulation.h0_friction import load_csv
    p = tmp_path / "fake.csv"
    p.write_text(HEADER
                 + _row("T", "WOOD", "FLAT", "PETG", "forward", angle="10")
                 + _row("T", "WOOD", "FLAT", "PETG", "forward", angle="12")
                 + _row("T", "WOOD", "FLAT", "PETG", "backward", angle="14")
                 + _row("T", "WOOD", "FLAT", "PETG", "lateral", angle="25")
                 + _row("T", "RUG", "KEEL", "TPU", "forward", angle="20")
                 + _row("T", "RUG", "KEEL", "TPU", "lateral", angle=""), encoding="utf-8")
    groups = {(g.floor_id, g.coupon): g for g in load_csv(p)}
    wood = groups[("WOOD", "FLAT-PETG")]
    assert wood.complete() and wood.mu["forward"].n == 2
    fwd = (math.tan(math.radians(10)) + math.tan(math.radians(12))) / 2
    assert wood.ratio_optimistic == pytest.approx(math.tan(math.radians(25)) / fwd)
    assert wood.ratio_conservative < wood.ratio_optimistic           # 後ろ向きの方が滑りにくいので保守側は小さい
    assert not groups[("RUG", "KEEL-TPU")].complete()                # 横が空欄 → 予測に使わない


# ---- 判定 ------------------------------------------------------------------------------------
def test_verdict_rules() -> None:
    from simulation.h0_friction import Prediction, verdict

    def pred(floor: str, fw5: float, a: float, b: float) -> Prediction:
        return Prediction(floor, "X", "conservative", 0.1, 0.3, {"FW5": fw5, "FW6_YAW5": a, "FW6_HEADYAW": b})

    v = verdict([pred("WOOD", 80, 120, 90), pred("RUG", 30, 70, 35), pred("CARPET", 10, 20, 12)], min_speed=50)
    assert v == {"WOOD": "PROPULSION_OK", "RUG": "NEEDS_BODY_YAW", "CARPET": "SNAKE_INSUFFICIENT"}


def test_prediction_runs_end_to_end(tmp_path: Path) -> None:
    """MuJoCo まで通す（短い秒数。数値の妥当性は見ない）。"""
    pytest.importorskip("mujoco")
    from simulation.h0_friction import load_csv, predict
    p = tmp_path / "fake.csv"
    p.write_text(HEADER + "".join(_row("T", "WOOD", "FLAT", "PETG", d, angle=a)
                                  for d, a in (("forward", "8"), ("backward", "9"), ("lateral", "30"))),
                 encoding="utf-8")
    (g,) = load_csv(p)
    preds = predict(g, seconds=1.0)
    assert {p.variant for p in preds} == {"optimistic", "conservative"}
    assert all(set(p.speed_mm_s) == {"FW5", "FW6_YAW5", "FW6_HEADYAW"} for p in preds)
