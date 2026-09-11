"""STEP 5 の確認用: 仮想カメラの画像から、ヘビ（ArUco）と人を床座標に落として追跡する。

  シミュレータ → 仮想カメラ画像 → 本物の ArUco 検出 → ホモグラフィ（+視差補正）→ SnakePoseTracker
                                → 人の bbox（SimPersonDetector）→ 足元を床座標 → PersonTracker
結果を STEP 4 の俯瞰図と同じ世界座標でプロットし、GIF と時系列グラフを保存する。

シナリオ: ヘビはマット上で左旋回を続ける。人 A はマットの左側をゆっくり歩く。
          人 B が 6 秒に奥から近づく。「ヘビ（首）に最も近い人」が 2 秒以上入れ替わったら追跡を切り替える
          （ヘビが旋回するので、近い人は A → B → A と変わる）。B は 14 秒から範囲外へ去る。
使い方: python tools/perception_demo.py [--out-dir output] [--no-gif]
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2  # noqa: E402
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

from serpens.config import load_config  # noqa: E402
from serpens.motion.gait import GaitEngine  # noqa: E402
from serpens.perception.aruco_locator import ArucoLocator  # noqa: E402
from serpens.perception.person_detector import PersonTracker  # noqa: E402
from serpens.sim.virtual_camera import SimPerson, SimPersonDetector, VirtualCamera  # noqa: E402
from serpens.sim.world import BodyPose, World  # noqa: E402

FRAME_DT = 0.1            # カメラ 10fps
DURATION_S = 24.0
START = BodyPose(380.0, 380.0, 0.0)
# 人の経路: (時刻 s, x mm, y mm) の折れ線
PATH_A = [(0.0, -450.0, 1500.0), (24.0, -450.0, 300.0)]   # マットの左側（カメラとマットの間に立つとマーカが隠れる）
PATH_B = [(0.0, 2200.0, 2200.0), (6.0, 1500.0, 1500.0), (9.0, 1150.0, 1250.0), (14.0, 1150.0, 1250.0), (18.0, 2300.0, 2300.0), (24.0, 2400.0, 2400.0)]
VIEW_MM = (-900.0, 2300.0)          # 俯瞰図の表示範囲（x, y 共通）
VIEW_PX = 540
GIF_SCALE = 0.5                     # カメラ画像を GIF 用に縮小


def along(path: list[tuple[float, float, float]], t: float) -> SimPerson:
    ts = [p[0] for p in path]
    return SimPerson(float(np.interp(t, ts, [p[1] for p in path])), float(np.interp(t, ts, [p[2] for p in path])))


def top_view(cfg: dict, world: World, people: list[SimPerson], pose, dets, target) -> np.ndarray:
    """俯瞰図（世界座標）を cv2 で描く。"""
    lo, hi = VIEW_MM
    s = VIEW_PX / (hi - lo)
    P = lambda xy: (int((xy[0] - lo) * s), int(VIEW_PX - (xy[1] - lo) * s))  # noqa: E731
    img = np.full((VIEW_PX, VIEW_PX, 3), 245, np.uint8)
    W, D = cfg["mat"]["width_mm"], cfg["mat"]["depth_mm"]
    cv2.rectangle(img, P((0, D)), P((W, 0)), (170, 205, 225), -1)
    cv2.circle(img, P((W / 2, D / 2)), int(cfg["person"]["max_range_mm"] * s), (180, 180, 180), 1)
    pts = world.world_points()
    cv2.polylines(img, [np.array([P(p) for p in pts[:, :2]])], False, (40, 110, 40), 4)
    for p in people:
        cv2.drawMarker(img, P((p.x_mm, p.y_mm)), (90, 60, 160), cv2.MARKER_TILTED_CROSS, 14, 2)
    for d in dets:
        cv2.circle(img, P(d.floor_mm), 5, (0, 0, 0), 1)
    if target is not None:
        cv2.circle(img, P(target.floor_mm), 14, (0, 200, 255), 3)
    if pose is not None:
        c = np.array([pose.x, pose.y])
        cv2.circle(img, P(c), 6, (0, 0, 255), -1)
        for th, col, w in ((pose.theta_body_raw, (150, 150, 255), 1), (pose.theta_body, (0, 0, 255), 3),
                           (pose.theta_head, (255, 0, 0), 2)):
            cv2.arrowedLine(img, P(c), P(c + 250 * np.array([math.cos(th), math.sin(th)])), col, w, tipLength=0.2)
    cv2.putText(img, "red: theta_body (thin=raw)  blue: theta_head", (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
    cv2.putText(img, "o: detection  yellow: tracked  x: true person", (8, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
    return img


def overlay(img: np.ndarray, obs, dets, target) -> np.ndarray:
    out = img.copy()
    for mid, c in obs.corners_px.items():
        cv2.polylines(out, [c.astype(np.int32)], True, (0, 0, 255), 3)
        cv2.putText(out, f"ID{mid}", tuple(c[0].astype(int) + np.array([0, -8])), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    for d in dets:
        x1, y1, x2, y2 = (int(v) for v in d.bbox_px)
        hit = target is not None and target.detection is d
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 200, 255) if hit else (200, 200, 200), 5 if hit else 2)
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", default="output")
    ap.add_argument("--no-gif", action="store_true")
    args = ap.parse_args()
    plt.rcParams["font.family"] = ["Yu Gothic", "Meiryo", "MS Gothic", "sans-serif"]
    cfg = load_config()
    cam = VirtualCamera(cfg)
    cfg["camera"]["position_mm"] = cfg["virtual_camera"]["position_mm"]   # 仮想カメラの位置は既知
    locator = ArucoLocator(cfg, cam.homography)
    people_det, tracker = SimPersonDetector(cam, cam.homography), PersonTracker(cfg)
    world, gait = World(cfg, START), GaitEngine(cfg)
    gait.start("turn_left")
    dt = float(cfg["sim"]["dt_s"])
    t, log, frames = 0.0, [], []
    while t < DURATION_S:
        for _ in range(int(round(FRAME_DT / dt))):
            t += dt
            world.step(gait.update(t), dt)
        people = [along(PATH_A, t), along(PATH_B, t)]
        img = cam.render(world, people)
        pose, obs = locator.update(img, t, world.angles.get("J8", 0.0))
        dets = people_det.detect(img)
        target = tracker.update(t, dets, None if pose is None else np.array([pose.x, pose.y]))
        true_neck = world.marker_xy("neck")
        who = None if target is None else int(np.argmin([math.hypot(p.x_mm - target.floor_mm[0], p.y_mm - target.floor_mm[1]) for p in people]))
        log.append(dict(t=t, seen=(obs.neck_mm is not None, obs.tail_mm is not None),
                        err=None if obs.neck_mm is None else float(np.linalg.norm(obs.neck_mm - true_neck)),
                        raw=None if pose is None else pose.theta_body_raw, filt=None if pose is None else pose.theta_body,
                        true=world.snake_pose()[2], who=who))
        if not args.no_gif:
            cam_img = cv2.resize(overlay(img, obs, dets, target), None, fx=GIF_SCALE, fy=GIF_SCALE)
            tv = top_view(cfg, world, people, pose, dets, target)
            both = np.hstack([cam_img, cv2.resize(tv, (cam_img.shape[0], cam_img.shape[0]))])
            cv2.putText(both, f"t={t:4.1f}s  tracked: {'-' if who is None else 'AB'[who]}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            frames.append(Image.fromarray(cv2.cvtColor(both, cv2.COLOR_BGR2RGB)))
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    if frames:
        frames[0].save(out / "step5_perception.gif", save_all=True, append_images=frames[1:],
                       duration=int(FRAME_DT * 1000), loop=0)
        print("saved", out / "step5_perception.gif")

    ts = np.array([r["t"] for r in log])
    both_seen = np.mean([all(r["seen"]) for r in log])
    errs = np.array([r["err"] for r in log if r["err"] is not None])
    print(f"マーカ2枚とも検出: {both_seen:.0%}   首位置の誤差: 平均 {errs.mean():.1f}mm / 最大 {errs.max():.1f}mm")
    fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    unwrap = lambda k: np.degrees(np.unwrap([r[k] if r[k] is not None else np.nan for r in log]))  # noqa: E731
    ax[0].plot(ts, unwrap("true"), "k-", lw=1, label="真値（尾→首, 生）")
    ax[0].plot(ts, unwrap("raw"), ".", ms=3, label="推定 θ_body 生値")
    ax[0].plot(ts, unwrap("filt"), "-", lw=2.5, label="推定 θ_body フィルタ後（1周期の移動平均, 移動制御用）")
    ax[0].set_ylabel("θ_body [deg]")
    ax[0].legend()
    ax[1].plot(ts, [r["err"] if r["err"] is not None else np.nan for r in log], "-")
    ax[1].set_ylabel("首位置の誤差 [mm]")
    ax[2].step(ts, [-1 if r["who"] is None else r["who"] for r in log], where="post")
    ax[2].set_yticks([-1, 0, 1], ["なし", "A", "B"])
    ax[2].set_ylabel("追跡中の人")
    ax[2].set_xlabel("時刻 [s]")
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "step5_perception.png", dpi=90)
    print("saved", out / "step5_perception.png")


if __name__ == "__main__":
    main()
