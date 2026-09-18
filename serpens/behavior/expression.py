"""生き物らしさの表現（時間の演出）。数値は全部 config の behavior.expression。

  a. 一次反応: 検出から 0.15〜0.25 秒で目を一瞬光らせ、J8 を刺激の方向へ 2.5° ピクッと動かす（primary_react）
  b. 人物検出から反応開始まで 0.4〜0.9 秒のランダム遅延（brain が schedule する）
  c. 反応の最初の 0.3 秒は全停止（surprise。呼吸だけは続ける）
  f. 舌のちらつき相当: J8 を ±3〜8° 0.2〜0.4 秒で往復。頻度は novelty に連動（maybe_flick）
  d. 頭の回転は ease-out ＋ 3〜5° のオーバーシュートと戻り、到達後 1.5〜2.5 秒ホールド（look_at）
  e. 首かしげ: J9 を 12〜18° 傾けて 1.2 秒保持（tilt）
  g. 3.5〜7 秒に1回、ランダムな方向を 1.5〜3 秒見てから戻る（glance_away。GAR 0.4〜0.6 が狙い）
  i. タッチでトルク 60% へ脱力＋首を下げる＋目を暗く、2 秒（petted）
"""
from __future__ import annotations

import random
from dataclasses import replace
from typing import Any, Callable

from serpens.hw.head_io import HeadIO
from serpens.hw.servo_bus import ServoBus
from serpens.motion.animator import Animator, Easing, Keyframe
from serpens.motion.poses import HEAD_ROLL, HEAD_YAW, NECK, Poses

Action = Callable[[float], None]


class Expression:
    """表情・しぐさの予定表と実行。"""

    def __init__(self, cfg: dict[str, Any], animator: Animator, poses: Poses, rng: random.Random,
                 bus: ServoBus | None = None, head: HeadIO | None = None) -> None:
        self.x = cfg["behavior"]["expression"]
        self.eyes_cfg = cfg["behavior"]["eyes"]
        self.head_modes = cfg["head_io"]["eye_modes"]
        self.anim, self.poses, self.rng = animator, poses, rng
        self.bus, self.head = bus, head
        self._events: list[tuple[float, str, Action]] = []
        self.look_yaw = 0.0
        self.hold_until = -1.0
        self.busy_until = -1.0            # かしげ・よそ見中は look_at を受け付けない
        self.next_tilt_t = float("inf")
        self.next_glance_t = self._draw_glance(0.0)
        self._flick_last_t: float | None = None
        self.person_tracked = False       # brain が毎周期入れる。True の間は J7 を look_max_deg までに抑える
        self.guarded = 0                  # guard が効いた回数（試験・ログ用）
        self.next_stretch_t = self._u_from(self.poses.stretch_timing()["interval_s"], 0.0)
        self.flick_until = -1.0
        self.flicks = 0
        self.log: list[tuple[float, str]] = []

    def _u(self, key: str) -> float:
        lo, hi = self.x[key]
        return self.rng.uniform(float(lo), float(hi))

    def _u_from(self, pair: Any, t: float) -> float:
        lo, hi = pair
        return t + self.rng.uniform(float(lo), float(hi))

    def play(self, kf: Keyframe, t: float) -> float:
        """姿勢を出す唯一の入口。人を追跡中は J7 を neck.look_max_deg までに抑える（威嚇に見せない）。"""
        if self.person_tracked and NECK in kf.pose and kf.pose[NECK] > self.poses.clamp_look_neck(kf.pose[NECK]):
            pose = dict(kf.pose)
            pose[NECK] = self.poses.clamp_look_neck(pose[NECK])
            kf = replace(kf, pose=pose)
            self.guarded += 1
        return self.anim.play(kf, t)

    def maybe_stretch(self, t: float) -> None:
        """伸び（人がいないときだけ。呼ばれる側が保証する）。"""
        if t < self.next_stretch_t or self.person_tracked or t < self.busy_until:
            return
        self.stretch(t)

    def stretch(self, t: float) -> None:
        st = self.poses.stretch_timing()
        dur = self.play(Keyframe(self.poses.stretch(), float(st["duration_s"]), Easing.IN_OUT), t)
        back = t + dur + float(st["hold_s"])
        self.busy_until = back + float(st["duration_s"])
        home = {k: v for k, v in self.poses.home().items() if k not in (HEAD_YAW, HEAD_ROLL)}
        self.schedule(back, "stretch_back", lambda tt: self.play(Keyframe(home, float(st["duration_s"])), tt))
        self.next_stretch_t = self._u_from(st["interval_s"], back)
        self.log.append((t, "stretch"))

    def _draw_glance(self, t: float) -> float:
        return t + self._u("glance_interval_s")

    # ---- 予定表 -----------------------------------------------------------------
    def schedule(self, t: float, name: str, fn: Action) -> None:
        self._events.append((t, name, fn))

    def cancel(self, name: str) -> None:
        self._events = [e for e in self._events if e[1] != name]

    def pending(self, name: str) -> bool:
        return any(e[1] == name for e in self._events)

    def clear(self) -> list[str]:
        """予約済みの演出を全部捨てる（停止時。解除後に勝手に動き出さないようにする）。"""
        names = [e[1] for e in self._events]
        self._events.clear()
        self.hold_until = self.busy_until = -1.0
        self.next_tilt_t = float("inf")
        return names

    def update(self, t: float) -> None:
        """時刻が来た予定を実行する。"""
        due = [e for e in self._events if e[0] <= t]
        self._events = [e for e in self._events if e[0] > t]
        for _t, name, fn in sorted(due, key=lambda e: e[0]):
            self.log.append((t, name))
            fn(t)

    # ---- しぐさ -----------------------------------------------------------------
    def primary_react(self, t: float, direction: float, state: str) -> None:
        """a. 一次反応: 目の輝度を一瞬上げ、J8 を direction の符号の側へ小さくピクッと動かす。"""
        if bool(self.x["primary_eye_flash"]):
            self.eye_flash(t, state)
        self.twitch(t, direction)
        self.log.append((t, "primary_react"))

    def eye_flash(self, t: float, state: str) -> None:
        """目を primary_flash_s だけ最大輝度で点灯し、元の状態の色へ戻す。"""
        if self.head is None:
            return
        r, g, b, _br, _mode = (int(v) for v in self.eyes_cfg[state])
        self.head.set_eye(r, g, b, int(self.x["primary_flash_brightness"]))
        self.head.set_mode(int(self.head_modes["on"]))
        self.cancel("eye_flash_end")
        self.schedule(t + float(self.x["primary_flash_s"]), "eye_flash_end", lambda _tt: self.set_eyes(state))

    def twitch(self, t: float, direction: float) -> None:
        """J8 をいまのベース角から direction の側へ primary_twitch_deg 動かし、同じ時間で戻す。"""
        base = self.anim.base.get(HEAD_YAW, 0.0)
        deg = float(self.x["primary_twitch_deg"]) * (1.0 if direction >= 0 else -1.0)
        dur = float(self.x["primary_twitch_s"])
        self.play(Keyframe(self.poses.head_look(base + deg, self.anim.base.get(HEAD_ROLL, 0.0)), dur, Easing.OUT), t)
        self.schedule(t + dur, "twitch_back",
                      lambda tt: self.play(Keyframe({HEAD_YAW: base}, dur, Easing.OUT), tt))
        self.log.append((t, f"twitch {deg:+.1f}°"))

    def maybe_flick(self, t: float, novelty: float) -> None:
        """f. 舌のちらつき相当。1周期あたり rate/60·dt の確率で J8 を小さく往復させる。"""
        f = self.x["flick"]
        last, self._flick_last_t = self._flick_last_t, t
        if last is None or t < self.flick_until or t < self.busy_until or self.anim.busy_joint(HEAD_YAW, t):
            return
        rate = float(f["base_rate_per_min"]) + (float(f["peak_rate_per_min"]) - float(f["base_rate_per_min"])) * \
            min(max(novelty, 0.0), 1.0)
        if self.rng.random() < rate / 60.0 * max(t - last, 0.0):
            self.flick(t)

    def flick(self, t: float) -> None:
        """J8 をいまのベース角から ±yaw_deg へ行って戻る（往復で duration_s）。"""
        f = self.x["flick"]
        base = self.anim.base.get(HEAD_YAW, 0.0)
        lo, hi = f["yaw_deg"]
        deg = self.rng.uniform(float(lo), float(hi)) * self.rng.choice((-1.0, 1.0))
        lo, hi = f["duration_s"]
        half = self.rng.uniform(float(lo), float(hi)) / 2.0
        self.play(Keyframe({HEAD_YAW: self.poses.head_look(base + deg)[HEAD_YAW]}, half, Easing.OUT), t)
        self.schedule(t + half, "flick_back", lambda tt: self.play(Keyframe({HEAD_YAW: base}, half, Easing.OUT), tt))
        self.flick_until = t + 2.0 * half
        self.flicks += 1
        self.log.append((t, f"flick {deg:+.0f}°"))

    def surprise(self, t: float) -> None:
        """c. 全停止 → surprise_freeze_s 後に解除。呼吸は surprise_keep_breath なら止めない。"""
        self.anim.freeze(t, keep_breath=bool(self.x["surprise_keep_breath"]))
        self.log.append((t, "surprise"))
        self.schedule(t + float(self.x["surprise_freeze_s"]), "unfreeze", self.anim.unfreeze)

    def look_at(self, t: float, yaw_deg: float, neck_deg: float | None = None, force: bool = False) -> bool:
        """d. 頭を向ける。ホールド中は小さな変化を無視する。向けたら True。"""
        if not force and (t < self.busy_until or
                          (t < self.hold_until and abs(yaw_deg - self.look_yaw) < float(self.x["look_retarget_deg"]))):
            return False
        pose = self.poses.head_look(yaw_deg, self.anim.base.get(HEAD_ROLL, 0.0), neck_deg)
        dur = self.play(Keyframe(pose, float(self.x["look_duration_s"]), Easing.OUT_OVERSHOOT,
                                      self._u("look_overshoot_deg")), t)
        self.look_yaw = pose[HEAD_YAW]
        self.hold_until = t + dur + self._u("look_hold_s")
        self.log.append((t, f"look {self.look_yaw:+.0f}°"))
        return True

    def tilt(self, t: float) -> None:
        """e. 首かしげ（左右ランダム）→ tilt_hold_s 保持 → 戻す。"""
        deg = self._u("tilt_deg") * self.rng.choice((-1.0, 1.0))
        dur = self.play(Keyframe({HEAD_ROLL: deg}, float(self.x["look_duration_s"]), Easing.OUT), t)
        back = t + dur + float(self.x["tilt_hold_s"])
        self.busy_until = back
        self.schedule(back, "untilt", lambda tt: self.play(Keyframe({HEAD_ROLL: 0.0}, float(self.x["look_duration_s"])), tt))
        self.log.append((t, f"tilt {deg:+.0f}°"))

    def start_tilting(self, t: float) -> None:
        self.next_tilt_t = t + self._u("tilt_interval_s")

    def stop_tilting(self) -> None:
        self.next_tilt_t = float("inf")

    def maybe_tilt(self, t: float) -> None:
        if t >= self.next_tilt_t:
            self.tilt(t)
            self.next_tilt_t = t + self._u("tilt_interval_s")

    def glance_away(self, t: float) -> None:
        """g. 視線をそらして、元の向きに戻る（主要動作。QUIET_STATES 以外で常時走る）。"""
        if t < self.next_glance_t or t < self.busy_until:
            return
        prev = self.look_yaw
        m = float(self.x["glance_max_yaw_deg"])
        self.look_at(t, self.rng.uniform(-m, m), force=True)
        back = t + self._u("glance_away_s")
        self.busy_until = back
        self.schedule(back, "glance_back", lambda tt: self.look_at(tt, prev, force=True))
        self.next_glance_t = self._draw_glance(t)
        self.log.append((t, "glance_away"))

    def petted(self, t: float) -> float:
        """i. 脱力: トルクを落とし、首を下げ、目を暗くする。petted_s 後にトルクを戻す。戻す時刻を返す。"""
        r = self.poses.relax()
        self.play(Keyframe(r.angles, float(self.x["look_duration_s"])), t)
        self.set_torque_ratio(r.torque_ratio if r.torque_ratio is not None else 1.0)
        end = t + float(self.x["petted_s"])
        self.cancel("petted_end")
        self.schedule(end, "petted_end", lambda tt: self.set_torque_ratio(1.0))
        self.log.append((t, "petted"))
        return end

    def set_torque_ratio(self, ratio: float) -> None:
        if self.bus is not None:
            for sid in self.bus.ids:
                self.bus.set_torque_limit(sid, ratio)

    def set_eyes(self, state: str) -> None:
        """状態ごとの目の色・明るさ・モードを送る（ESP32 のフェイルセーフより短い間隔で呼ぶこと）。"""
        if self.head is None:
            return
        r, g, b, br, mode = (int(v) for v in self.eyes_cfg[state])
        self.head.set_eye(r, g, b, br)
        self.head.set_mode(mode)
