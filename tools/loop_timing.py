r"""制御ループ（50Hz）の周期を、知覚スレッドを同時に回した状態で実測する（このPC上のソフトの測定）。

実機・実カメラは使わない。測るのは「PC の処理が 50Hz を守れるか」だけ。
  A. 仮想カメラの描画スレッド（GUI の --sim と同じ構成）
  B. 録画ファイル → ArUco + YOLO（--camera と同じ構成。重みが無ければ ArUco だけ）

    .\.venv\Scripts\python.exe tools\loop_timing.py --seconds 10
"""
from __future__ import annotations

import argparse
import math
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2  # noqa: E402

from serpens.app import SimCamera  # noqa: E402
from serpens.config import load_config  # noqa: E402
from serpens.motion.gait import GaitEngine  # noqa: E402
from serpens.runner import ControlLoop  # noqa: E402
from serpens.sim.session import ManualClock, SimSession  # noqa: E402
from serpens.sim.virtual_camera import VirtualCamera  # noqa: E402
from serpens.sim.world import BodyPose, World  # noqa: E402

START = BodyPose(600.0, 600.0, -math.pi / 2)
REC_FPS = 10.0


def record(cfg: dict, path: Path, seconds: float) -> Path:
    """仮想カメラの動画を作る（実カメラの代わり）。"""
    cam = VirtualCamera(cfg)
    cam.homography.save(path.with_suffix(".json"))
    w = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), REC_FPS, (cam.w, cam.h))
    world, gait, dt, t = World(cfg, START), GaitEngine(cfg), float(cfg["sim"]["dt_s"]), 0.0
    gait.start("forward")
    for _ in range(int(seconds * REC_FPS)):
        for _ in range(int(round(1.0 / REC_FPS / dt))):
            t += dt
            world.step(gait.update(t), dt)
        w.write(cam.render(world, []))
    w.release()
    return path


def report(label: str, loop: ControlLoop, extra: str) -> None:
    st = loop.stats
    print(f"{label:32s} 周期 平均 {st.mean_ms:5.2f}ms 最悪 {st.worst_ms:6.1f}ms "
          f"遅れ(>20%) {st.late_count}/{st.count}  {extra}")


def run_sim_camera(cfg: dict, seconds: float) -> None:
    s = SimSession(cfg, START, seed=1)
    loop = ControlLoop(s, realtime=True)
    cam = SimCamera(cfg, s)
    loop.start()
    cam.start()
    time.sleep(seconds)
    cam.release()
    loop.shutdown()
    report("A. 仮想カメラ描画スレッド", loop, "")


def run_recording(cfg: dict, seconds: float, video: Path) -> None:
    from serpens.perception.camera_observer import CameraObserver

    clock = ManualClock()
    obs = CameraObserver(cfg, str(video), clock, homography_path=video.with_suffix(".json"))
    s = SimSession(cfg, START, seed=1, observer=obs, clock=clock)
    loop = ControlLoop(s, realtime=True)
    t0 = time.perf_counter()
    obs.step()                                   # 1枚ぶんの検出時間（YOLO の初回は重い）
    first = time.perf_counter() - t0
    t0 = time.perf_counter()
    obs.step()
    per_frame = time.perf_counter() - t0
    loop.start()
    obs.start()
    time.sleep(seconds)
    obs.release()
    loop.shutdown()
    who = "ArUco+YOLO" if obs.people is not None else "ArUco のみ（YOLO 重みなし）"
    report(f"B. 録画 → {who}", loop,
           f"検出 1枚 {per_frame * 1000:.0f}ms（初回 {first * 1000:.0f}ms）, {obs.frames} 枚")
    for n in obs.notes:
        print("   注意:", n)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()
    cfg = load_config()
    print(f"目標 {cfg['behavior']['tick_hz']}Hz。**このPCでのソフトの測定。実機・実カメラではない**")
    run_sim_camera(cfg, args.seconds)
    with tempfile.TemporaryDirectory() as d:
        video = record(cfg, Path(d) / "rec.avi", args.seconds + 2.0)
        run_recording(cfg, args.seconds, video)
    return 0


if __name__ == "__main__":
    sys.exit(main())
