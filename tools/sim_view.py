"""STEP 4 の確認用: シミュレータのヘビを上から見た図・アニメーションにする。

アニメーター → MockServoBus（サーボの遅れ込み）→ 実角度 → World、の実際の流れで動かす。
あわせて「1周期あたり何 mm 進むか」を歩容ごとに表示する（実機との比較用）。

使い方: python tools/sim_view.py [--gif] [--out-dir output]
"""
from __future__ import annotations

import argparse
import copy
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402

from serpens.config import load_config  # noqa: E402
from serpens.hw.mock_bus import MockServoBus  # noqa: E402
from serpens.motion.animator import Animator, Keyframe, coil_keyframe  # noqa: E402
from serpens.motion.gait import GaitEngine, gait_period_s  # noqa: E402
from serpens.motion.poses import Poses  # noqa: E402
from serpens.sim.world import BodyPose, World  # noqa: E402

CTRL_DT = 0.02          # 制御周期 50Hz（アニメーター → サーボ）
FAR = 1.0e5             # 推進量測定用の広いマット
START_TAIL_X, START_TAIL_Y = 60.0, 200.0   # デモの初期位置（尾端）
COIL_S = 6.0            # とぐろ（尾から順）を見せる時間


class SimClock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


def measure_per_cycle(cfg: dict, gait: str, via_servo: bool, cycles: int = 3) -> tuple[float, float]:
    """広いマットで歩かせて、1周期あたりの (前進量 mm, 回転 deg) を返す。"""
    c = copy.deepcopy(cfg)
    c["mat"]["width_mm"] = c["mat"]["depth_mm"] = FAR
    frames = run_frames(c, [("walk", lambda a, p, t: a.gait.start(gait), 0.0)],
                        BodyPose(FAR / 2, FAR / 2, 0.0), via_servo, record=False,
                        extra_s=(3 + cycles + 1) * gait_period_s(GaitEngine(c).presets[gait]))
    T = gait_period_s(GaitEngine(c).presets[gait])
    k0 = int(3 * T / CTRL_DT)
    k1 = k0 + int(round(cycles * T / CTRL_DT))
    (c0, th0), (c1, th1) = frames[k0], frames[k1]
    return float(np.linalg.norm(c1 - c0)) / cycles, math.degrees(th1 - th0) / cycles


def run_frames(cfg: dict, script: list, start: BodyPose, via_servo: bool,
               record: bool = True, extra_s: float = 0.0) -> list:
    """script = [(ラベル, 動作関数(anim, poses, t), 継続秒)] を実行し、各周期の記録を返す。"""
    clock = SimClock()
    anim, poses, world = Animator(cfg), Poses(cfg), World(cfg, start)
    bus = MockServoBus(cfg, clock=clock)
    bus.connect()
    for sid in bus.ids:
        bus.set_torque(sid, True)
    ids = {j["name"]: int(j["servo_id"]) for j in cfg["joints"]}
    sub = max(int(round(CTRL_DT / cfg["sim"]["dt_s"])), 1)
    frames, label = [], ""
    for lab, act, dur in script + [("", lambda a, p, t: None, extra_s)]:
        label = lab or label
        act(anim, poses, clock.t)
        for _ in range(int(round(dur / CTRL_DT))):
            cmd = anim.update(clock.t)
            anim.send(bus, cmd)
            for _ in range(sub):
                clock.t += CTRL_DT / sub
                if via_servo:
                    st = bus.sync_read_states()
                    angles = {n: st[i].pos_deg for n, i in ids.items()}
                else:
                    angles = cmd
                world.step(angles, CTRL_DT / sub)
            if record:
                frames.append((clock.t, world.world_points().copy(), world.centroid().copy(), label))
            else:
                frames.append((world.centroid().copy(), world.pose.theta))
    return frames


def demo_script(cfg: dict) -> list:
    T = gait_period_s(GaitEngine(cfg).presets["forward"])
    return [
        ("前進", lambda a, p, t: a.gait.start("forward"), 1.5 * T),
        ("左旋回", lambda a, p, t: a.gait.start("turn_left"), 3.0 * T),
        ("停止", lambda a, p, t: a.gait.stop(), 1.5),
        ("とぐろ", lambda a, p, t: a.play(coil_keyframe(p), t), COIL_S),
    ]


def draw_mat(ax: plt.Axes, cfg: dict) -> None:
    w, d = cfg["mat"]["width_mm"], cfg["mat"]["depth_mm"]
    ax.add_patch(plt.Rectangle((0, 0), w, d, fill=True, fc="#f3efe2", ec="k", lw=2))
    ax.set_xlim(-50, w + 50)
    ax.set_ylim(-50, d + 50)
    ax.set_aspect("equal")
    ax.set_xlabel("X [mm]（原点 = マット左手前）")
    ax.set_ylabel("Y [mm]")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", default="output")
    ap.add_argument("--gif", action="store_true", help="アニメーション GIF も作る")
    args = ap.parse_args()
    plt.rcParams["font.family"] = ["Yu Gothic", "Meiryo", "MS Gothic", "sans-serif"]
    cfg = load_config()
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    print(f"=== 1周期あたりの推進量（belly={cfg['belly']['type']}/{cfg['belly']['friction_profile']} のシミュレータ値）")
    print(f"{'歩容':<11}{'指令角のまま':>16}{'モックサーボ経由':>20}")
    for g in cfg["gait"]["presets"]:
        a = measure_per_cycle(cfg, g, via_servo=False)
        b = measure_per_cycle(cfg, g, via_servo=True)
        print(f"{g:<11}{a[0]:8.1f}mm {a[1]:+6.1f}°   {b[0]:8.1f}mm {b[1]:+6.1f}°")

    T = gait_period_s(GaitEngine(cfg).presets["forward"])
    panels = [
        ("前進 1周期", [("前進", lambda a, p, t: a.gait.start("forward"), T + float(cfg["gait"]["blend_s"]))],
         BodyPose(40.0, 600.0, 0.0), 0.5),
        ("左旋回 4周期", [("左旋回", lambda a, p, t: a.gait.start("turn_left"), 4 * T)],
         BodyPose(250.0, 250.0, 0.0), 1.0),
        ("停止姿勢 → とぐろ", [("とぐろ", lambda a, p, t: a.play(coil_keyframe(p), t), COIL_S)],
         BodyPose(200.0, 400.0, 0.0), 0.5),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(18, 6.6))
    cmap = plt.get_cmap("viridis")
    for ax, (title, script, start, snap_s) in zip(axes, panels):
        frames = run_frames(cfg, script, start, via_servo=True)
        draw_mat(ax, cfg)
        traj = np.array([f[2] for f in frames])
        ax.plot(traj[:, 0], traj[:, 1], "--", color="gray", lw=1.5, label="重心の軌跡")
        snap_every = int(snap_s / CTRL_DT)
        for k in range(0, len(frames), snap_every):
            _, pts, _, _ = frames[k]
            col = cmap(k / len(frames))
            ax.plot(pts[:, 0], pts[:, 1], "-", color=col, lw=4, alpha=0.55)
            ax.plot(pts[-1, 0], pts[-1, 1], "^", color=col, ms=8)
        _, pts, _, _ = frames[-1]
        ax.plot(pts[:, 0], pts[:, 1], "-", color="tab:red", lw=cfg["body"]["diameter_mm"] / 6, alpha=0.5, label="最終")
        ax.set_title(f"{title}（{snap_s}秒ごと, ▲=頭, 紫→黄）")
        ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out / "step4_sim_topview.png", dpi=80)
    print("saved", out / "step4_sim_topview.png")
    frames = run_frames(cfg, demo_script(cfg), BodyPose(START_TAIL_X, START_TAIL_Y, 0.0), via_servo=True)
    traj = np.array([f[2] for f in frames])

    if args.gif:
        fig2, ax2 = plt.subplots(figsize=(6, 6))
        draw_mat(ax2, cfg)
        body, = ax2.plot([], [], "-", color="tab:green", lw=cfg["body"]["diameter_mm"] / 8, solid_capstyle="round")
        head, = ax2.plot([], [], "r^", ms=9)
        path, = ax2.plot([], [], "--", color="gray")
        txt = ax2.text(20, cfg["mat"]["depth_mm"] - 60, "", fontsize=12)
        step = 2

        def upd(i: int) -> tuple:
            t, pts, _, lab = frames[i * step]
            body.set_data(pts[:, 0], pts[:, 1])
            head.set_data([pts[-1, 0]], [pts[-1, 1]])
            path.set_data(traj[: i * step, 0], traj[: i * step, 1])
            txt.set_text(f"t={t:4.1f}s  {lab}")
            return body, head, path, txt

        anim = FuncAnimation(fig2, upd, frames=len(frames) // step, blit=True)
        anim.save(out / "step4_sim.gif", writer=PillowWriter(fps=int(1 / (CTRL_DT * step))))
        print("saved", out / "step4_sim.gif")


if __name__ == "__main__":
    main()
