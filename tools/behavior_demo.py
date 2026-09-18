"""STEP 6 の確認用: シミュレータ上で人の座標を動かし、一連の振る舞いを GIF と時系列グラフにする。

シナリオ（シミュレーション時間）:
   0 s  誰もいない → 巡回（時々よそ見）
  12 s  人 A が来場者側（y<0）に現れる → 0.4〜0.9 s 後に驚いて全停止 → 警戒 → 2段階で接近 → かかわる（かしげ・見つめる）
  50 s  頭をタッチ → 脱力（トルク 60%・首を下げる・目を暗く 2 s）
  62 s  A が急に近づく → ストレス → 退避（速く離れる）
  78 s  A が去る（範囲外）→ 3 s 保持 → 巡回
  会場が暑い設定なので、動き続けるとサーボが熱くなり Energy が落ちて「とぐろで休む」へ
使い方: python tools/behavior_demo.py [--out-dir output] [--seconds 170] [--no-gif]
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
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

from serpens.behavior.utility import STATE_LABELS_JA, STATES  # noqa: E402
from serpens.config import load_config  # noqa: E402
from serpens.sim.session import SimSession  # noqa: E402
from serpens.sim.virtual_camera import SimPerson  # noqa: E402
from serpens.sim.world import BodyPose  # noqa: E402

GIF_FPS = 5
VIEW_MM = (-1100.0, 1700.0)
VIEW_PX = 520
PANEL_W = 560
FONT = "C:/Windows/Fonts/YuGothM.ttc"
# 動くと熱くなりやすい設定（Energy の変化を 2 分で見せるため。実機の値ではない）
HOT = {"mock_servo": {"ambient_c": 28.0, "heat_tau_s": 25.0, "heat_gain_c": 300.0}}
TOUCH_S = (50.0, 50.4)            # この間、頭のタッチセンサを押す
COLORS = {"SLEEP": "#6c7a89", "PATROL": "#4caf50", "ALERT": "#ffb300", "OBSERVE": "#29b6f6", "APPROACH": "#26a69a",
          "ENGAGE": "#ec407a", "PETTED": "#f8bbd0", "RETREAT": "#ff5722",
          "COIL_REST_MOOD": "#7e57c2", "COIL_REST_HEAT": "#d84315"}


def person_at(t: float) -> list[SimPerson]:
    if t < 12.0 or t >= 78.0:
        return []
    if t < 62.0:
        return [SimPerson(700.0, -600.0)]
    k = min((t - 62.0) / 2.0, 1.0)                      # 2 秒で 500mm 近づく
    return [SimPerson(700.0, -600.0 + 500.0 * k)]


def draw_top(s: SimSession) -> np.ndarray:
    lo, hi = VIEW_MM
    sc = VIEW_PX / (hi - lo)
    P = lambda xy: (int((xy[0] - lo) * sc), int(VIEW_PX - (xy[1] - lo) * sc))  # noqa: E731
    img = np.full((VIEW_PX, VIEW_PX, 3), 250, np.uint8)
    cfg = s.cfg
    cv2.rectangle(img, P((0, cfg["mat"]["depth_mm"])), P((cfg["mat"]["width_mm"], 0)), (170, 205, 225), -1)
    pts = s.world.world_points()
    cv2.polylines(img, [np.array([P(p) for p in pts[:, :2]])], False, (40, 110, 40), max(int(cfg["body"]["diameter_mm"] * sc), 3))
    head = pts[-1]
    raised = pts[-1, 2] > cfg["sim"]["contact_height_mm"]
    cv2.circle(img, P(head[:2]), 7 if raised else 4, (0, 0, 200), -1)
    if s.snake is not None:
        c = np.array([s.snake.x, s.snake.y])
        cv2.arrowedLine(img, P(c), P(c + 220 * np.array([math.cos(s.snake.theta_head), math.sin(s.snake.theta_head)])),
                        (200, 60, 0), 2, tipLength=0.25)
    for p in s.people:
        cv2.circle(img, P((p.x_mm, p.y_mm)), 12, (90, 60, 160), -1)
        cv2.circle(img, P((p.x_mm, p.y_mm)), int(cfg["behavior"]["controller"]["near_person_mm"] * sc), (90, 60, 160), 1)
    return img


def draw_panel(s: SimSession, font: ImageFont.ImageFont, small: ImageFont.ImageFont) -> Image.Image:
    st = s.status
    im = Image.new("RGB", (PANEL_W, VIEW_PX), (20, 24, 30))
    d = ImageDraw.Draw(im)
    d.text((16, 12), f"t = {st.t:5.1f} s", fill=(255, 255, 255), font=small)
    d.text((16, 44), f"状態: {st.state_ja}  （次の切替まで {st.time_to_next_s:.1f} s）", fill=(255, 230, 120), font=font)
    d.text((16, 86), st.thought, fill=(255, 255, 255), font=small)
    d.text((16, 118), f"移動: {st.drive}", fill=(180, 220, 255), font=small)
    y = 160
    bars = [("Curiosity", st.internal["curiosity"]), ("Affection", st.internal["affection"]),
            ("Energy", st.internal["energy"]), ("Stress", st.internal["stress"]), ("Attention", st.internal["attention"])]
    for name, v in bars:
        d.text((16, y), name, fill=(255, 255, 255), font=small)
        d.rectangle([150, y + 4, 150 + 360, y + 26], outline=(200, 200, 200))
        d.rectangle([150, y + 4, 150 + int(360 * v), y + 26], fill=(80, 200, 120) if name != "Stress" else (255, 110, 80))
        d.text((520, y), f"{v:.2f}", fill=(255, 255, 255), font=small)
        y += 40
    heat = "-" if st.heat_c is None else f"{st.heat_c:.1f} ℃"
    d.text((16, y), f"Heat（サーボ最高温度）: {heat}", fill=(255, 180, 120), font=small)
    eyes = s.head.eye_rgb_brightness
    d.ellipse([430, y - 2, 470, y + 30], fill=tuple(int(c * eyes[3] / 255) for c in eyes[:3]))
    d.text((480, y), "目", fill=(255, 255, 255), font=small)
    return im


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", default="output")
    ap.add_argument("--seconds", type=float, default=170.0)
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--no-gif", action="store_true")
    args = ap.parse_args()
    plt.rcParams["font.family"] = ["Yu Gothic", "Meiryo", "MS Gothic", "sans-serif"]
    cfg = load_config()
    s = SimSession(cfg, BodyPose(120.0, 350.0, 0.2), seed=args.seed, overrides=HOT)
    font, small = ImageFont.truetype(FONT, 26), ImageFont.truetype(FONT, 20)
    frames, log, events = [], [], []
    frame_every = int(round(cfg["behavior"]["tick_hz"] / GIF_FPS))
    k, n_log = 0, 0
    while s.t < args.seconds:
        s.people = person_at(s.t)
        s.touch(TOUCH_S[0] <= s.t < TOUCH_S[1])
        st = s.step()
        events += [(st.t, e) for e in st.events]
        new = s.brain.expr.log[n_log:]
        n_log = len(s.brain.expr.log)
        events += [(t, n) for t, n in new if n in ("surprise", "glance_away", "petted") or n.startswith("tilt")]
        log.append((st.t, st.state, dict(st.internal), st.heat_c))
        if not args.no_gif and k % frame_every == 0:
            top = Image.fromarray(cv2.cvtColor(draw_top(s), cv2.COLOR_BGR2RGB))
            both = Image.new("RGB", (VIEW_PX + PANEL_W, VIEW_PX))
            both.paste(top, (0, 0))
            both.paste(draw_panel(s, font, small), (VIEW_PX, 0))
            frames.append(both.quantize(colors=128))
        k += 1
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    for t, e in events:
        print(f"{t:6.2f}s {e}")
    if frames:
        frames[0].save(out / "step6_behavior.gif", save_all=True, append_images=frames[1:], duration=int(1000 / GIF_FPS), loop=0)
        print("saved", out / "step6_behavior.gif", f"({len(frames)} frames)")

    ts = np.array([r[0] for r in log])
    fig, ax = plt.subplots(3, 1, figsize=(13, 9), sharex=True, gridspec_kw={"height_ratios": [3, 1, 1]})
    for key in ("curiosity", "affection", "energy", "stress", "attention"):
        ax[0].plot(ts, [r[2][key] for r in log], lw=2, label=key.capitalize())
    ax[0].set_ylim(-0.02, 1.02)
    ax[0].legend(ncol=5, loc="upper center")
    ax[0].set_ylabel("内部状態")
    for t0, t1, stt in _segments(log):
        ax[1].axvspan(t0, t1, color=COLORS[stt], alpha=0.9)
        if t1 - t0 > 3:
            ax[1].text((t0 + t1) / 2, 0.5, STATE_LABELS_JA[stt], ha="center", va="center", fontsize=10)
    ax[1].set_yticks([])
    ax[1].set_ylabel("状態")
    ax[2].plot(ts, [r[3] if r[3] is not None else np.nan for r in log], color="tab:red")
    ax[2].set_ylabel("Heat [℃]")
    ax[2].set_xlabel("時刻 [s]")
    for t, e in events:
        ax[0].axvline(t, color="k", lw=0.5, alpha=0.3)
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "step6_internal_state.png", dpi=85)
    print("saved", out / "step6_internal_state.png")
    assert set(r[1] for r in log) <= set(STATES)


def _segments(log: list) -> list[tuple[float, float, str]]:
    segs, start, cur = [], log[0][0], log[0][1]
    for t, stt, *_ in log[1:]:
        if stt != cur:
            segs.append((start, t, cur))
            start, cur = t, stt
    segs.append((start, log[-1][0], cur))
    return segs


if __name__ == "__main__":
    main()
