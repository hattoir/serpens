"""人の評価（Human Perception Pilot）用の比較クリップ。**すべて KINEMATIC_SIM の描画。**

同じカメラ・同じ背景・同じ時間・同じ開始位置・同じターゲット位置・同じ表示スケールで、
身体構成（6 / 8 / 10 軸）と歩容だけを変えて描く。**条件名は画面に出さない。**
条件と匿名 ID（Clip A / B / C …）の対応は key JSON にだけ書く（tools/pilot_clips.py）。

  scene = "gait" … 固定の開始位置から、固定のターゲット（来場者の人型）へ向かって歩容だけで進む
  scene = "arc"  … EXHIBITION profile の体験弧（来場者が横へ動く → 気づく → 見る → 近づく）を行動エンジンで
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator

import cv2
import numpy as np

from serpens.config import load_config
from serpens.motion.gait import GaitEngine, GaitParams
from serpens.sim.session import SimSession
from serpens.sim.virtual_camera import SimPerson, VirtualCamera
from serpens.sim.world import BodyPose, World
from simulation.bridge import drive_people
from simulation.virtual_person import VirtualPerson

SOURCE = "KINEMATIC_SIM"


@dataclass(frozen=True)
class ClipSpec:
    """1 本ぶんの条件。"""

    condition: str                     # 人には見せない名前（例: yaw8_w2.0_RAW）
    overlay: str | None                # config の上書き（None = robot.yaml）
    scene: str = "gait"                # gait / arc
    amplitude_deg: float = 30.0
    spatial_freq_deg: float = 60.0
    temporal_freq_hz: float = 0.5
    profile: str | None = None         # arc のときの profile overlay


@dataclass(frozen=True)
class RenderSettings:
    """全クリップで共通にする描画条件（違いは身体と歩容だけ）。"""

    duration_s: float = 12.0
    fps: float = 10.0
    width_px: int = 960
    start: tuple[float, float, float] = (600.0, 1100.0, -90.0)   # 尾端 x, y, θ[deg]（来場者側を向く）
    target_xy: tuple[float, float] = (600.0, -450.0)             # 来場者の人型（固定）
    seed: int = 1


def _cfg(spec: ClipSpec) -> dict[str, Any]:
    cfg = load_config(overlay=spec.overlay)
    if spec.profile:
        from serpens.config import _read_yaml, apply_overlay

        cfg = apply_overlay(cfg, _read_yaml(Path(spec.profile)))
    return cfg


def frames(spec: ClipSpec, rs: RenderSettings) -> Iterator[np.ndarray]:
    """描いたフレームを順に返す（BGR、rs.width_px に縮小）。"""
    cfg = _cfg(spec)
    cam = VirtualCamera(cfg)
    scale = rs.width_px / cam.w
    start = BodyPose(rs.start[0], rs.start[1], math.radians(rs.start[2]))
    n = int(round(rs.duration_s * rs.fps))
    if spec.scene == "gait":
        world, eng = World(cfg, start), GaitEngine(cfg)
        eng.start(GaitParams(spec.amplitude_deg, spec.spatial_freq_deg, spec.temporal_freq_hz, 0.0))
        dt, t = float(cfg["sim"]["dt_s"]), 0.0
        people = [SimPerson(*rs.target_xy)]
        for _ in range(n):
            for _ in range(int(round(1.0 / rs.fps / dt))):
                t += dt
                world.step(eng.update(t), dt)
            yield cv2.resize(cam.render(world, people), None, fx=scale, fy=scale)
    elif spec.scene == "arc":
        s = SimSession(cfg, start, seed=rs.seed)
        walker = VirtualPerson("CROSSING", (-200.0, rs.target_xy[1]), 350.0, appear_s=3.0,
                               stop_at_x_mm=rs.target_xy[0])
        steps = int(round(1.0 / rs.fps / s.ctrl_dt))
        for _ in range(n):
            for _ in range(steps):
                drive_people(s, [walker], s.t)
                s.step()
            yield cv2.resize(cam.render(s.world, s.people), None, fx=scale, fy=scale)
    else:
        raise ValueError(f"未知の scene: {spec.scene}")


def write_clip(spec: ClipSpec, rs: RenderSettings, path: Path) -> int:
    """AVI（MJPG）に書く。戻り値はフレーム数。画面に条件名は描かない。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    writer: cv2.VideoWriter | None = None
    count = 0
    for img in frames(spec, rs):
        if writer is None:
            writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), rs.fps, (img.shape[1], img.shape[0]))
            if not writer.isOpened():
                raise RuntimeError(f"動画を開けません: {path}")
        writer.write(img)
        count += 1
    if writer is not None:
        writer.release()
    return count
