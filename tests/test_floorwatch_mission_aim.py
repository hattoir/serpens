"""mission の「線を候補へ向け直す」（_aim_delta_deg）: 線の横位置は姿勢で動く・当たっていても中心からずれていれば向け直す（H2 VIS-0006）。"""
from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from serpens.config import load_config
from serpens.floorwatch.mission import InspectMission


@pytest.fixture(scope="module")
def mission() -> InspectMission:
    cfg = load_config()
    return InspectMission(cfg, SimpleNamespace(), np.zeros(2), lambda: {}, lambda f: [], 0.0, head_link_mm=50.0)


def _cand(x: float, y: float = 75.0, line_x: float | None = 0.0, on_line: bool = True, height: float | None = None,
          kinds: tuple[str, ...] = ("unknown",), d: float = 10.0) -> SimpleNamespace:
    return SimpleNamespace(floor_xy_mm=(x, y), line_x_mm=line_x, on_line=on_line, height_mm=height,
                           kinds=[{"kind": k} for k in kinds], diameter_mm=d)


def test_aim_is_measured_from_the_line_not_from_the_camera_centre(mission: InspectMission) -> None:
    """頭が沈むと床の線は横へ動く（例 +5mm）。候補が x=+5 にあれば、線はもう候補の上 → 向け直さない。"""
    mission.result = [_cand(5.0, line_x=5.0, on_line=True)]
    assert mission._aim_delta_deg() is None
    mission.result = [_cand(0.0, line_x=5.0, on_line=False)]          # 以前は x=0 を「線の上」とみなして向け直さなかった
    delta = mission._aim_delta_deg()
    assert delta is not None and delta > 0                            # 候補は線の左 → 左（正）へ


def test_on_line_but_off_centre_without_decisive_evidence_is_reaimed(mission: InspectMission) -> None:
    tol = float(mission.m["line_aim_tol_mm"])
    mission.result = [_cand(tol + 1.5, on_line=True)]                  # 当たっているが中心から外れ、決め手が無い
    assert mission._aim_delta_deg() is not None
    mission.result = [_cand(tol + 1.5, on_line=True, kinds=("metal_disc", "unknown"))]   # 決め手あり → そのまま
    assert mission._aim_delta_deg() is None
    mission.result = [_cand(tol + 1.5, on_line=True, height=1.4)]
    assert mission._aim_delta_deg() is None
    mission.result = [_cand(tol * 0.5, on_line=True)]                  # 許容の中
    assert mission._aim_delta_deg() is None


def test_missing_line_position_falls_back_to_the_camera_centre(mission: InspectMission) -> None:
    mission.result = [_cand(-8.0, line_x=None, on_line=False)]           # カメラ x は右が正 → −8 は左 → 頭ヨーは左（正）
    delta = mission._aim_delta_deg()
    assert delta is not None and delta > 0
