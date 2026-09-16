"""World State への橋渡し（Phase 4 の入口）。

    Simulated Vision（真値 or 模擬画像の ArUco）→ **World State** → Behavior → Motion Planner
    → RobotInterface → SimulatedSnake → Virtual ESP32 → 仮想サーボ → 世界

観測器が真値なら `GROUND_TRUTH_SIM`、模擬画像の Vision（`SimVisionObserver`）なら
`ARUCO` / `PERSON_DETECTOR` に **simulated=True** を付ける。実カメラなら simulated=False。
真値を「Vision が成功した」として数えないために、出どころを必ず付けて回す。
"""
from __future__ import annotations

from typing import Any

from serpens.world_state import Pose2D, PoseSource, WorldState
from simulation.virtual_person import VirtualPerson

# セッションの pose_source（"sim" / "aruco"）→ World State の出どころ
SESSION_SOURCE = {"sim": PoseSource.GROUND_TRUTH_SIM, "aruco": PoseSource.ARUCO,
                  "aruco_sim": PoseSource.ARUCO}
SIMULATED_SOURCES = {"sim", "aruco_sim"}


def world_state_from_session(session: Any) -> WorldState:
    """`SimSession` の見ているものを World State にする。"""
    t = session.t
    name = getattr(session, "pose_source", "sim")
    src = SESSION_SOURCE.get(name, PoseSource.UNKNOWN)
    sim = name in SIMULATED_SOURCES
    ws = WorldState(t=t)
    snake = session.snake
    if snake is not None:
        # 観測の時刻は「マーカが最後に見えた時刻」（古さを隠さない）
        at = snake.t if src.is_vision else t
        ws.robot = Pose2D(snake.x, snake.y, snake.theta_body, src, at_s=at, simulated=sim)
        ws.head = Pose2D(snake.x, snake.y, snake.theta_head, src, at_s=at, simulated=sim)
    # 人は、シミュレーションでは真値。実機では人物検出（PERSON_DETECTOR）へ差し替わる
    person_src = PoseSource.GROUND_TRUTH_SIM if src is PoseSource.GROUND_TRUTH_SIM \
        else PoseSource.PERSON_DETECTOR
    if src.is_vision and getattr(session, "last_observation", None) is not None:
        obs = session.last_observation              # 検出器が見た人（真値ではない）
        ws.people = [Pose2D(float(d.floor_mm[0]), float(d.floor_mm[1]), 0.0, person_src,
                            at_s=obs.t_capture, confidence=d.conf, simulated=sim) for d in obs.people]
    else:
        ws.people = [Pose2D(p.x_mm, p.y_mm, 0.0, person_src, at_s=t, simulated=sim) for p in session.people]
    if session.target is not None:
        xy = session.target.floor_mm
        at = session.target.last_seen_t if src.is_vision else t
        ws.target = Pose2D(float(xy[0]), float(xy[1]), 0.0, person_src, at_s=at, simulated=sim)
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
