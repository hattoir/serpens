"""動きの語彙（primitives）。時間軸の素材で、**状態を知らない**。

  freeze / twitch / flick / glance_away / tilt / look_at / sag / nuzzle / stretch / hold_breath / eye_flash

どれも「いつ撃つか」は決めない。それは文法（grammar.py、config の behavior.grammar）が決める。
数値はすべて config（behavior.expression / animator.anticipate / animator.settle / poses）。

予定表（schedule）: 戻り動作や解除を時刻で積み、update(t) で実行する。停止時は clear() で全部捨てる
（解除後に勝手に動き出さないため）。
guard: person_tracked の間は J7 を neck.look_max_deg までに抑える（威嚇に見せない）。
"""
from __future__ import annotations

import math
import random
from dataclasses import replace
from typing import Any, Callable

from serpens.hw.head_io import HeadIO
from serpens.hw.servo_bus import ServoBus
from serpens.motion.animator import Animator, Easing, Keyframe
from serpens.motion.poses import HEAD_ROLL, HEAD_YAW, NECK, Poses

Action = Callable[[float], None]


class Primitives:
    """しぐさの素材と予定表。"""

    def __init__(self, cfg: dict[str, Any], animator: Animator, poses: Poses, rng: random.Random,
                 bus: ServoBus | None = None, head: HeadIO | None = None) -> None:
        self.x = cfg["behavior"]["expression"]
        self.anticipate = cfg["animator"]["anticipate"]
        self.settle = cfg["animator"]["settle"]
        self.eyes_cfg = cfg["behavior"]["eyes"]
        self.head_modes = cfg["head_io"]["eye_modes"]
        self.anim, self.poses, self.rng = animator, poses, rng
        self.bus, self.head = bus, head
        self._events: list[tuple[float, str, Action]] = []
        self.look_yaw = 0.0
        self.hold_until = -1.0
        self.busy_until = -1.0            # かしげ・よそ見・伸びの最中は look_at を受け付けない
        self.person_tracked = False       # brain が毎周期入れる。True の間は J7 を look_max_deg までに抑える
        self.guarded = 0                  # guard が効いた回数（試験・ログ用）
        self.counts: dict[str, int] = {}  # 撃った回数（評価・試験用）
        self.log: list[tuple[float, str]] = []

    # ---- 共通 ---------------------------------------------------------------------
    def _u(self, key: str) -> float:
        lo, hi = self.x[key]
        return self.rng.uniform(float(lo), float(hi))

    def _u_from(self, pair: Any) -> float:
        lo, hi = pair
        return self.rng.uniform(float(lo), float(hi))

    def _note(self, t: float, name: str, detail: str = "") -> None:
        self.counts[name] = self.counts.get(name, 0) + 1
        self.log.append((t, name if not detail else f"{name} {detail}"))

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
        return names

    def update(self, t: float) -> None:
        """時刻が来た予定を実行する。"""
        due = [e for e in self._events if e[0] <= t]
        self._events = [e for e in self._events if e[0] > t]
        for _t, name, fn in sorted(due, key=lambda e: e[0]):
            self.log.append((t, name))
            fn(t)

    # ---- 姿勢の出口（guard） ---------------------------------------------------
    def play(self, kf: Keyframe, t: float) -> float:
        """姿勢を出す唯一の入口。人を追跡中は J7 を neck.look_max_deg までに抑える（威嚇に見せない）。"""
        if self.person_tracked and NECK in kf.pose and kf.pose[NECK] > self.poses.clamp_look_neck(kf.pose[NECK]):
            pose = dict(kf.pose)
            pose[NECK] = self.poses.clamp_look_neck(pose[NECK])
            kf = replace(kf, pose=pose)
            self.guarded += 1
        return self.anim.play(kf, t)

    def with_settle(self, kf: Keyframe) -> Keyframe:
        """胴体の大きな姿勢変化に follow-through（尾側の減衰振動）を付ける。"""
        return replace(kf, settle_amp_deg=float(self.settle["amp_deg"]), settle_tau_s=float(self.settle["tau_s"]),
                       settle_joints=tuple(self.settle["joints"]))

    # ---- 語彙 -------------------------------------------------------------------
    def freeze(self, t: float, hold_s: float, keep_breath: bool = True) -> None:
        """全停止（驚き）→ hold_s 後に解除。keep_breath なら呼吸だけは続く。"""
        self.anim.freeze(t, keep_breath=keep_breath)
        self.schedule(t + hold_s, "unfreeze", self.anim.unfreeze)
        self._note(t, "freeze")

    def twitch(self, t: float, direction: float) -> None:
        """J8 をいまのベース角から direction の側へ primary_twitch_deg 動かし、同じ時間で戻す。"""
        base = self.anim.base.get(HEAD_YAW, 0.0)
        deg = float(self.x["primary_twitch_deg"]) * (1.0 if direction >= 0 else -1.0)
        dur = float(self.x["primary_twitch_s"])
        self.play(Keyframe(self.poses.head_look(base + deg, self.anim.base.get(HEAD_ROLL, 0.0)), dur, Easing.OUT), t)
        self.schedule(t + dur, "twitch_back", lambda tt: self.play(Keyframe({HEAD_YAW: base}, dur, Easing.OUT), tt))
        self._note(t, "twitch", f"{deg:+.1f}°")

    def eye_flash(self, t: float, state: str) -> None:
        """目を primary_flash_s だけ最大輝度で点灯し、元の状態の色へ戻す。"""
        if self.head is None:
            return
        r, g, b, _br, _mode = (int(v) for v in self.eyes_cfg[state])
        self.head.set_eye(r, g, b, int(self.x["primary_flash_brightness"]))
        self.head.set_mode(int(self.head_modes["on"]))
        self.cancel("eye_flash_end")
        self.schedule(t + float(self.x["primary_flash_s"]), "eye_flash_end", lambda _tt: self.set_eyes(state))
        self._note(t, "eye_flash")

    def flick(self, t: float) -> float:
        """舌のちらつき相当: J8 を ±yaw_deg へ行って戻る（往復で duration_s）。終わる時刻を返す。"""
        f = self.x["flick"]
        base = self.anim.base.get(HEAD_YAW, 0.0)
        deg = self._u_from(f["yaw_deg"]) * self.rng.choice((-1.0, 1.0))
        half = self._u_from(f["duration_s"]) / 2.0
        self.play(Keyframe({HEAD_YAW: self.poses.head_look(base + deg)[HEAD_YAW]}, half, Easing.OUT), t)
        self.schedule(t + half, "flick_back", lambda tt: self.play(Keyframe({HEAD_YAW: base}, half, Easing.OUT), tt))
        self._note(t, "flick", f"{deg:+.0f}°")
        return t + 2.0 * half

    def look_at(self, t: float, yaw_deg: float, neck_deg: float | None = None, force: bool = False) -> bool:
        """頭を向ける（大きな向き直しには予備動作が付く）。ホールド中は小さな変化を無視する。向けたら True。"""
        if not force and (t < self.busy_until or
                          (t < self.hold_until and abs(yaw_deg - self.look_yaw) < float(self.x["look_retarget_deg"]))):
            return False
        pose = self.poses.head_look(yaw_deg, self.anim.base.get(HEAD_ROLL, 0.0), neck_deg)
        big = abs(pose[HEAD_YAW] - self.anim.base.get(HEAD_YAW, 0.0)) >= float(self.anticipate["min_travel_deg"])
        dur = self.play(Keyframe(pose, float(self.x["look_duration_s"]), Easing.OUT_OVERSHOOT,
                                 self._u("look_overshoot_deg"),
                                 anticipate_deg=self._u_from(self.anticipate["head_deg"]) if big else 0.0,
                                 anticipate_lead_s=self._u_from(self.anticipate["lead_s"]) if big else 0.0), t)
        self.look_yaw = pose[HEAD_YAW]
        self.hold_until = t + dur + self._u("look_hold_s")
        self._note(t, "look", f"{self.look_yaw:+.0f}°")
        return True

    def glance_away(self, t: float) -> None:
        """視線をそらして（glance_away_s）、元の向きに戻る。"""
        prev = self.look_yaw
        m, off = float(self.x["glance_max_yaw_deg"]), float(self.x["glance_min_offset_deg"])
        choices = [y for y in (prev - self.rng.uniform(off, m + off), prev + self.rng.uniform(off, m + off))
                   if -m <= y <= m] or [math.copysign(m, -prev)]
        self.look_at(t, self.rng.choice(choices), force=True)
        back = t + self._u("glance_away_s")
        self.busy_until = back
        self.schedule(back, "glance_back", lambda tt: self.look_at(tt, prev, force=True))
        self._note(t, "glance_away")

    def tilt(self, t: float) -> None:
        """首かしげ（左右ランダム）→ tilt_hold_s 保持 → 戻す。"""
        deg = self._u("tilt_deg") * self.rng.choice((-1.0, 1.0))
        dur = self.play(Keyframe({HEAD_ROLL: deg}, float(self.x["look_duration_s"]), Easing.OUT), t)
        back = t + dur + float(self.x["tilt_hold_s"])
        self.busy_until = back
        self.schedule(back, "untilt", lambda tt: self.play(Keyframe({HEAD_ROLL: 0.0}, float(self.x["look_duration_s"])), tt))
        self._note(t, "tilt", f"{deg:+.0f}°")

    def hold_breath(self, t: float, hold_s: float) -> None:
        """呼吸を一瞬止める（撫でへの一次反応）。"""
        self.anim.set_breathing(False)
        self.cancel("breathe_again")
        self.schedule(t + hold_s, "breathe_again", lambda _tt: self.anim.set_breathing(True))
        self._note(t, "hold_breath")

    def sag(self, t: float, torque_ratio: float, release_s: float) -> float:
        """脱力: トルクを落とし、首を下げ、release_s 後にトルクを戻す。戻す時刻を返す。"""
        r = self.poses.relax(torque_ratio)
        self.play(Keyframe(r.angles, float(self.x["look_duration_s"])), t)
        self.set_torque_ratio(r.torque_ratio if r.torque_ratio is not None else 1.0)
        end = t + release_s
        self.cancel("sag_end")
        self.schedule(end, "sag_end", lambda _tt: self.set_torque_ratio(1.0))
        self._note(t, "sag")
        return end

    def nuzzle(self, t: float, side: float) -> None:
        """すり寄る: J8 を side の側へ nuzzle_deg、nuzzle_duration_s かけて寄せる。"""
        p = self.x["petted"]
        base = self.anim.base.get(HEAD_YAW, 0.0)
        deg = self._u_from(p["nuzzle_deg"]) * (1.0 if side >= 0 else -1.0)
        self.play(Keyframe(self.poses.head_look(base + deg), self._u_from(p["nuzzle_duration_s"]), Easing.IN_OUT), t)
        self._note(t, "nuzzle", f"{deg:+.0f}°")

    def stretch(self, t: float) -> None:
        """伸び: 真上から見下ろすフル鎌首（呼ぶ側が「人がいない」を保証する）。"""
        st = self.poses.stretch_timing()
        dur = self.play(self.with_settle(Keyframe(self.poses.stretch(), float(st["duration_s"]), Easing.IN_OUT)), t)
        back = t + dur + float(st["hold_s"])
        self.busy_until = back + float(st["duration_s"])
        home = {k: v for k, v in self.poses.home().items() if k not in (HEAD_YAW, HEAD_ROLL)}
        self.schedule(back, "stretch_back", lambda tt: self.play(Keyframe(home, float(st["duration_s"])), tt))
        self._note(t, "stretch")

    # ---- 出力先 -------------------------------------------------------------------
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
