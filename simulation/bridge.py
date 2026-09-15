"""World State への橋渡し（Phase 4 の入口）。

    Simulated Vision（いまは Ground Truth）→ **World State** → Behavior → Motion Planner
    → RobotInterface → SimulatedSnake → Virtual ESP32 → 仮想サーボ → 世界

**いまは真値をそのまま流している。** Phase 4 でカメラを繋いだら、`PoseSource` が
`GROUND_TRUTH_SIM` から `ARUCO` / `PERSON_DETECTOR` へ変わるだけで、上位は変わらない。
真値を「Vision が成功した」として数えないために、出どころを必ず付けて回す。
"""
from __future__ import annotations

from typing import Any

from serpens.world_state import Pose2D, PoseSource, WorldState
from simulation.virtual_person import VirtualPerson

# セッションの pose_source（"sim" / "aruco"）→ World State の出どころ
SESSION_SOURCE = {"sim": PoseSource.GROUND_TRUTH_SIM, "aruco": PoseSource.ARUCO}


def world_state_from_session(session: Any) -> WorldState:
    """`SimSession` の見ているものを World State にする。"""
    t = session.t
    src = SESSION_SOURCE.get(getattr(session, "pose_source", "sim"), PoseSource.UNKNOWN)
    ws = WorldState(t=t)
    snake = session.snake
    if snake is not None:
        ws.robot = Pose2D(snake.x, snake.y, snake.theta_body, src, at_s=t)
        ws.head = Pose2D(snake.x, snake.y, snake.theta_head, src, at_s=t)
    # 人は、シミュレーションでは真値。実機では人物検出（PERSON_DETECTOR）へ差し替わる
    person_src = PoseSource.GROUND_TRUTH_SIM if src is PoseSource.GROUND_TRUTH_SIM \
        else PoseSource.PERSON_DETECTOR
    ws.people = [Pose2D(p.x_mm, p.y_mm, 0.0, person_src, at_s=t) for p in session.people]
    if session.target is not None:
        xy = session.target.floor_mm
        ws.target = Pose2D(float(xy[0]), float(xy[1]), 0.0, person_src, at_s=t)
    return ws


def drive_people(session: Any, people: list[VirtualPerson], t: float) -> None:
    """仮想の来場者を、その時刻の位置でセッションへ置く（既存の `session.people` を使う）。"""
    from serpens.sim.virtual_camera import SimPerson

    session.people = [SimPerson(*p.at(t)) for p in people if p.visible(t)]


def mujoco_world_state(model: Any, data: Any, t: float, head_body: str = "") -> WorldState:
    """MuJoCo の状態から World State を作る（**Ground Truth**）。

    3D 側を閉ループに入れるときの入口。ここも `GROUND_TRUTH_SIM` を明示する。
    """
    import math

    import mujoco

    def pose_of(body_id: int) -> Pose2D:
        x, y = float(data.xpos[body_id][0]) * 1000.0, float(data.xpos[body_id][1]) * 1000.0
        m = data.xmat[body_id].reshape(3, 3)
        return Pose2D(x, y, math.atan2(m[1, 0], m[0, 0]), PoseSource.GROUND_TRUTH_SIM, at_s=t)

    ws = WorldState(t=t)
    ws.robot = pose_of(1)                       # seg0（土台）
    head_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, head_body) if head_body else -1
    ws.head = pose_of(head_id if head_id > 0 else model.nbody - 1)
    ws.notes.append("MUJOCO_SIM の真値（Vision ではない）")
    return ws
