"""生き物らしさの表現（時間の演出）。数値は全部 config の behavior.expression。

  b. 人物検出から反応開始まで 0.4〜0.9 秒のランダム遅延（brain が schedule する）
  c. 反応の最初の 0.3 秒は呼吸も含めて全停止（surprise）
  d. 頭の回転は ease-out ＋ 3〜5° のオーバーシュートと戻り、到達後 1.5〜2.5 秒ホールド（look_at）
  e. 首かしげ: J9 を 12〜18° 傾けて 1.2 秒保持（tilt）
  g. 15〜40 秒に1回、ランダムな方向を 1.5 秒見る（maybe_distract）
  i. タッチでトルク 60% へ脱力＋首を下げる＋目を暗く、2 秒（petted）
"""
from __future__ import annotations

import random
from typing import Any, Callable

from serpens.hw.head_io import HeadIO
from serpens.hw.servo_bus import ServoBus
from serpens.motion.animator import Animator, Easing, Keyframe
from serpens.motion.poses import HEAD_ROLL, HEAD_YAW, Poses

Action = Callable[[float], None]


class Expression:
    """表情・しぐさの予定表と実行。"""

    def __init__(self, cfg: dict[str, Any], animator: Animator, poses: Poses, rng: random.Random,
                 bus: ServoBus | None = None, head: HeadIO | None = None) -> None:
        self.x = cfg["behavior"]["expression"]
        self.eyes_cfg = cfg["behavior"]["eyes"]
        self.anim, self.poses, self.rng = animator, poses, rng
        self.bus, self.head = bus, head
        self._events: list[tuple[float, str, Action]] = []
        self.look_yaw = 0.0
        self.hold_until = -1.0
        self.busy_until = -1.0            # かしげ・よそ見中は look_at を受け付けない
        self.next_tilt_t = float("inf")
        self.next_distract_t = self._draw_distract(0.0)
        self.log: list[tuple[float, str]] = []

    def _u(self, key: str) -> float:
        lo, hi = self.x[key]
        return self.rng.uniform(float(lo), float(hi))

    def _draw_distract(self, t: float) -> float:
        return t + self._u("distraction_interval_s")

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
    def surprise(self, t: float) -> None:
        """c. 全停止（呼吸も）→ surprise_freeze_s 後に解除。"""
        self.anim.freeze(t)
        self.log.append((t, "surprise"))
        self.schedule(t + float(self.x["surprise_freeze_s"]), "unfreeze", self.anim.unfreeze)

    def look_at(self, t: float, yaw_deg: float, neck_deg: float | None = None, force: bool = False) -> bool:
        """d. 頭を向ける。ホールド中は小さな変化を無視する。向けたら True。"""
        if not force and (t < self.busy_until or
                          (t < self.hold_until and abs(yaw_deg - self.look_yaw) < float(self.x["look_retarget_deg"]))):
            return False
        pose = self.poses.head_look(yaw_deg, self.anim.base.get(HEAD_ROLL, 0.0), neck_deg)
        dur = self.anim.play(Keyframe(pose, float(self.x["look_duration_s"]), Easing.OUT_OVERSHOOT,
                                      self._u("look_overshoot_deg")), t)
        self.look_yaw = pose[HEAD_YAW]
        self.hold_until = t + dur + self._u("look_hold_s")
        self.log.append((t, f"look {self.look_yaw:+.0f}°"))
        return True

    def tilt(self, t: float) -> None:
        """e. 首かしげ（左右ランダム）→ tilt_hold_s 保持 → 戻す。"""
        deg = self._u("tilt_deg") * self.rng.choice((-1.0, 1.0))
        dur = self.anim.play(Keyframe({HEAD_ROLL: deg}, float(self.x["look_duration_s"]), Easing.OUT), t)
        back = t + dur + float(self.x["tilt_hold_s"])
        self.busy_until = back
        self.schedule(back, "untilt", lambda tt: self.anim.play(Keyframe({HEAD_ROLL: 0.0}, float(self.x["look_duration_s"])), tt))
        self.log.append((t, f"tilt {deg:+.0f}°"))

    def start_tilting(self, t: float) -> None:
        self.next_tilt_t = t + self._u("tilt_interval_s")

    def stop_tilting(self) -> None:
        self.next_tilt_t = float("inf")

    def maybe_tilt(self, t: float) -> None:
        if t >= self.next_tilt_t:
            self.tilt(t)
            self.next_tilt_t = t + self._u("tilt_interval_s")

    def maybe_distract(self, t: float) -> None:
        """g. 時々よそ見をして、元の向きに戻る。"""
        if t < self.next_distract_t or t < self.busy_until:
            return
        prev = self.look_yaw
        m = float(self.x["distraction_max_yaw_deg"])
        self.look_at(t, self.rng.uniform(-m, m), force=True)
        back = t + float(self.x["distraction_look_s"])
        self.busy_until = back
        self.schedule(back, "distract_back", lambda tt: self.look_at(tt, prev, force=True))
        self.next_distract_t = self._draw_distract(t)
        self.log.append((t, "distraction"))

    def petted(self, t: float) -> float:
        """i. 脱力: トルクを落とし、首を下げ、目を暗くする。petted_s 後にトルクを戻す。戻す時刻を返す。"""
        r = self.poses.relax()
        self.anim.play(Keyframe(r.angles, float(self.x["look_duration_s"])), t)
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
