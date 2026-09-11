"""STEP 3 の確認用: 9軸の角度列と、姿勢の形を図にする。

使い方: python tools/plot_motion.py [--out output/step3_motion.png]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from serpens.config import load_config  # noqa: E402
from serpens.motion.animator import Animator, Easing, Keyframe, coil_keyframe  # noqa: E402
from serpens.motion.kinematics import min_self_clearance  # noqa: E402
from serpens.motion.poses import Poses  # noqa: E402

DT = 0.02
# 図の体裁だけに使う値
Y_RANGE_DEG = 95
LABEL_Y_DEG = 92


def scenario(cfg: dict) -> tuple[np.ndarray, np.ndarray, list[tuple[float, str]], list[str]]:
    """home → 前進 → 左旋回 → 停止 → とぐろ → 鎌首 → 見る → 脱力。"""
    poses, anim = Poses(cfg), Animator(cfg)
    steps = [
        ("home", lambda t: anim.play(Keyframe(poses.home(), 0.5), t), 1.0),
        ("前進", lambda t: anim.gait.start("forward"), 5.0),
        ("左旋回", lambda t: anim.gait.start("turn_left"), 4.0),
        ("停止", lambda t: anim.gait.stop(), 1.5),
        ("とぐろ", lambda t: anim.play(coil_keyframe(poses), t), 6.0),
        ("鎌首60°", lambda t: anim.play(Keyframe(poses.rear_up(60), 1.5), t), 2.5),
        ("左を見る+かしげ", lambda t: anim.play(Keyframe(poses.head_look(40, 15), 0.8, Easing.OUT_OVERSHOOT, 4.0), t), 2.5),
        ("脱力", lambda t: anim.play(Keyframe(poses.relax().angles, 0.5), t), 2.0),
    ]
    t, ts, rows, marks = 0.0, [], [], []
    for label, act, dur in steps:
        marks.append((t, label))
        act(t)
        end = t + dur
        while t < end - 1e-9:
            rows.append([anim.update(t)[n] for n in anim.names])
            ts.append(t)
            t += DT
    return np.array(ts), np.array(rows), marks, anim.names


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="output/step3_motion.png")
    args = ap.parse_args()
    plt.rcParams["font.family"] = ["Yu Gothic", "Meiryo", "MS Gothic", "sans-serif"]
    cfg = load_config()
    poses = Poses(cfg)
    ts, arr, marks, names = scenario(cfg)

    fig = plt.figure(figsize=(15, 9))
    ax = fig.add_subplot(2, 1, 1)
    for i, n in enumerate(names):
        ax.plot(ts, arr[:, i], label=n, lw=1.6 if i < 6 else 2.4)
    for t, label in marks:
        ax.axvline(t, color="k", lw=0.8)
        ax.text(t + 0.05, LABEL_Y_DEG, label, fontsize=11)
    ax.set_ylim(-Y_RANGE_DEG, Y_RANGE_DEG + 5)
    ax.set_xlabel("時刻 [s]")
    ax.set_ylabel("関節角 [deg]")
    ax.set_title("9軸の指令角（呼吸 ±2° 込み, 50Hz）")
    ax.legend(ncol=9, loc="lower center", fontsize=9)
    ax.grid(alpha=0.3)

    sc = cfg["self_collision"]
    shapes = [("home", poses.home()), ("とぐろ", poses.coil()), ("鎌首60° (胴体S字)", poses.rear_up(60))]
    for k, (label, pose) in enumerate(shapes):
        a2 = fig.add_subplot(2, 4, 5 + k)
        pts = poses.points(pose)
        a2.plot(pts[:, 0], pts[:, 1], "-", color="tab:green", lw=cfg["body"]["diameter_mm"] / 6, alpha=0.35)
        a2.plot(pts[:, 0], pts[:, 1], "o-", color="k", ms=3)
        a2.plot(*pts[-1, :2], "r^", ms=9)
        clr = min_self_clearance(pts[:, :2], sc["arc_skip_mm"], sc["sample_mm"])
        a2.set_title(f"{label}（上から） 最小間隔 {clr:.0f}mm", fontsize=10)
        a2.set_aspect("equal", adjustable="datalim")
        a2.grid(alpha=0.3)
    a3 = fig.add_subplot(2, 4, 8)
    for ang in (55, 65, 85):
        pts = poses.points(poses.rear_up(ang))
        a3.plot(pts[:, 0], pts[:, 2], "o-", ms=3, label=f"J7={ang}° 高さ{pts[-1, 2]:.0f}mm")
    a3.set_title("鎌首（横から）", fontsize=10)
    a3.set_aspect("equal")
    a3.legend(fontsize=8)
    a3.grid(alpha=0.3)
    fig.tight_layout()
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=90)
    print("saved", args.out)


if __name__ == "__main__":
    main()
