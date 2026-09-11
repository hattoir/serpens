"""STEP 5 の実写確認: Webカメラ / 録画 / 静止画で、マーカと人を床座標に落とす。

前提:
  1. tools/calibrate_floor.py で config/floor_homography.json を作ってある
  2. 人の検出には YOLO の重み（config の person.model_path）が必要。無ければ ArUco だけ動く
表示: カメラ画像（マーカ枠・人物枠・追跡中の1人を黄色）と、俯瞰図（世界座標）。q / Esc で終了。

使い方:
  python tools/perception_live.py --source 0
  python tools/perception_live.py --source recording.mp4
  python tools/perception_live.py --source photo.jpg --frames 1 --no-window   # 結果をコンソールに出すだけ
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from serpens.config import load_config  # noqa: E402
from serpens.perception.aruco_locator import ArucoLocator  # noqa: E402
from serpens.perception.camera import open_source  # noqa: E402
from serpens.perception.homography import FloorHomography  # noqa: E402
from serpens.perception.person_detector import PersonTracker, YoloPersonDetector  # noqa: E402

WINDOW = "perception_live"
KEY_Q, KEY_ESC = ord("q"), 27
MAX_WIN_W = 1280


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="0")
    ap.add_argument("--homography", default=None, help="既定は config の homography.file")
    ap.add_argument("--frames", type=int, default=0, help="この枚数で終了（0 = 無限）")
    ap.add_argument("--no-window", action="store_true")
    args = ap.parse_args()
    cfg = load_config()
    hpath = Path(args.homography or cfg["homography"]["file"])
    if not hpath.exists():
        raise SystemExit(f"{hpath} がありません。先に tools/calibrate_floor.py を実行してください")
    h = FloorHomography.load(hpath)
    locator = ArucoLocator(cfg, h)
    try:
        people = YoloPersonDetector(cfg, h)
    except FileNotFoundError as e:
        print(f"[人物検出なし] {e}")
        people = None
    tracker = PersonTracker(cfg)
    src = open_source(cfg, args.source)
    t0, n = time.monotonic(), 0
    try:
        while args.frames == 0 or n < args.frames:
            ok, frame = src.read()
            if not ok or frame is None:
                break
            t = time.monotonic() - t0
            pose, obs = locator.update(frame, t, 0.0)
            dets = people.detect(frame) if people else []
            target = tracker.update(t, dets, None if pose is None else np.array([pose.x, pose.y]))
            n += 1
            line = f"[{t:6.2f}s] markers={sorted(obs.corners_px)}"
            if pose is not None:
                line += f"  首=({pose.x:6.0f},{pose.y:6.0f})mm  θ_body={math.degrees(pose.theta_body):6.1f}°"
            line += f"  人={len(dets)}"
            if target is not None:
                line += f"  追跡=({target.floor_mm[0]:6.0f},{target.floor_mm[1]:6.0f})mm"
            print(line)
            if args.no_window:
                continue
            view = frame.copy()
            for mid, c in obs.corners_px.items():
                cv2.polylines(view, [c.astype(np.int32)], True, (0, 0, 255), 3)
            for d in dets:
                x1, y1, x2, y2 = (int(v) for v in d.bbox_px)
                hit = target is not None and target.detection is d
                cv2.rectangle(view, (x1, y1), (x2, y2), (0, 220, 255) if hit else (200, 200, 200), 4 if hit else 2)
            if view.shape[1] > MAX_WIN_W:
                view = cv2.resize(view, None, fx=MAX_WIN_W / view.shape[1], fy=MAX_WIN_W / view.shape[1])
            cv2.imshow(WINDOW, view)
            if cv2.waitKey(1) & 0xFF in (KEY_Q, KEY_ESC):
                break
    finally:
        src.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
