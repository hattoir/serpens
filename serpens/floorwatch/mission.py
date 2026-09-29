"""inspect_point の段取り: 地点へ行く → 止まる → 静止を待つ → 撮る → 判定する。

brain の状態機械（展示の行動）は使わず、locomotion の drive_to と同じ安全（人との距離・マット端）だけ通す。
撮影中は歩容を止め、動いていれば撮り直す（detect の MotionError）。数値は config の floor_watch.mission。
線から外れた候補（横に 1.5mm 以上）があれば、頭ヨーでその候補へ線を向けて撮り直す（AIM、最大 max_aims 回。線は頭に付いて
いるので、頭を θ 回すと候補の位置で (頭リンク長 + 前方距離) × sin θ 横に動く）。
`capture()` と `judge()` は注入する（模擬では合成画像、実機では頭カメラ。**この段階は模擬だけ**）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np

from serpens.behavior.controller import DriveCommand
from serpens.perception.snake_pose import SnakePose

PHASES = ("GOTO", "SETTLE", "ADJUST", "CREEP", "WAIT_CHILD", "CAPTURE", "JUDGE", "RETREAT", "DONE", "DEFERRED", "FAILED",
          "ABORTED")


@dataclass
class IdleHold:
    """Task が無いあいだ機体を止めておく（展示の巡回に戻らない）。Floor Watch では Task 以外で動かない。"""

    loco: Any
    active: bool = True

    def tick(self, t: float, snake: SnakePose | None, person_xy: np.ndarray | None) -> None:
        self.loco.stop("待機（Task 待ち）")

    def abort(self, reason: str) -> None:
        self.loco.stop(reason)


@dataclass
class InspectMission:
    cfg: dict[str, Any]
    loco: Any                                   # serpens.behavior.locomotion.Locomotion
    target_mm: np.ndarray
    capture: Callable[[], dict[str, np.ndarray]]
    judge: Callable[[dict[str, np.ndarray]], Any]
    t0: float
    aim: Callable[[float, float], None] | None = None   # (t, 頭ヨーの増分 deg)。None なら狙い直さない
    measure: Callable[[], tuple[float, float]] | None = None   # 頭先端から地点まで (前方, 左) [mm]。None なら位置合わせしない
    head_link_mm: float = 0.0                           # 頭ヨーの関節からカメラまでの長さ（狙いの計算に使う）
    attention_ok: Callable[[float], bool] | None = None  # CSAR: 物を照らしてよいか（子どもが遠いと確かめられた）。None なら常に可
    object_mm: np.ndarray | None = None                  # 見に行く物（地点）の位置。離れるときに避ける（CSAR R3）
    phase: str = "GOTO"
    reason: str = ""
    result: Any = None
    events: list[str] = field(default_factory=list)
    _phase_t: float = 0.0
    _retries: int = 0
    _aims: int = 0
    _adjusts: int = 0
    _pre_aims: int = 0
    _burst_until: float = 0.0
    _wait_t: float | None = None
    _retreat_from: np.ndarray | None = None
    _retreat_to: np.ndarray | None = None

    def __post_init__(self) -> None:
        self.m = self.cfg["floor_watch"]["mission"]
        self._phase_t = self.t0
        # 歩容は止めても振幅が消えるまで進む（blend_s）ので、その分だけ手前で止める
        self.coast_mm = float(self.m["speed_mm_s"]) * float(self.cfg["gait"]["blend_s"]) * float(self.m["coast_frac"])

    @property
    def active(self) -> bool:
        return self.phase in ("GOTO", "SETTLE", "ADJUST", "CREEP", "WAIT_CHILD", "CAPTURE", "JUDGE", "RETREAT")

    def _enter(self, phase: str, t: float, note: str = "") -> None:
        self.phase, self._phase_t = phase, t
        self.events.append(f"inspect: {phase}" + (f"（{note}）" if note else ""))

    def abort(self, reason: str) -> None:
        self.loco.stop(reason)
        self.phase, self.reason = "ABORTED", reason

    def tick(self, t: float, snake: SnakePose | None, person_xy: np.ndarray | None) -> None:
        if not self.active:
            return
        if snake is None:
            self.loco.stop("位置不明: 停止")
            return
        if self.phase == "GOTO":
            if t - self.t0 > float(self.m["goto_timeout_s"]):
                self.loco.stop("inspect: 目標に着けない")
                self.phase, self.reason = "FAILED", f"目標に {self.m['goto_timeout_s']}s 以内に着けなかった"
                return
            if float(np.linalg.norm(self.target_mm - [snake.x, snake.y])) <= float(self.m["arrive_mm"]) + self.coast_mm:
                self.loco.stop("inspect: 到着。静止を待つ")
                self._enter("SETTLE", t)
                return
            self.loco.set_drive(self.loco.ctrl.drive_to(t, snake, self.target_mm, float(self.m["speed_mm_s"]), person_xy,
                                                        allow_reverse=False), snake, person_xy)   # 後ろは見えない
        elif self.phase == "SETTLE":
            self.loco.stop("inspect: 静止中")
            if t - self._phase_t >= float(self.m["settle_s"]):
                if self.measure is not None:
                    self._enter("ADJUST", t)
                else:
                    self._to_capture(t)
        elif self.phase == "ADJUST":
            # 停止位置は歩容の位相で 10cm 近く散り、蛇行で頭は横にもずれる。頭から地点までの (前方, 左) を測り、
            # 前後は短い微調整、横は頭ヨーで地点を線の上に置く
            fwd, lat = (float(v) for v in self.measure())
            d = fwd - float(self.m["view_target_mm"])
            if abs(d) > float(self.m["view_tol_mm"]) and self._adjusts < int(self.m["max_adjusts"]):
                pass                                                     # 下で前後の微調整
            elif abs(lat) > float(self.m["lat_tol_mm"]) and self.aim is not None and self._pre_aims < int(self.m["max_aims"]):
                self._pre_aims += 1
                delta = math.degrees(math.asin(max(-1.0, min(1.0, lat / (self.head_link_mm + max(fwd, 1.0))))))
                lim = float(self.m["aim_max_deg"])
                self.aim(t, max(-lim, min(lim, delta)))
                self._enter("SETTLE", t, f"地点を線の上に（頭ヨー {delta:+.1f}°、横 {lat:+.0f}mm）")
                return
            else:
                self._to_capture(t, f"地点まで 前方 {fwd:.0f}mm 横 {lat:+.0f}mm")
                return
            self._adjusts += 1
            creep = float(self.m["creep_speed_mm_s"])
            self._burst_until = t + min(abs(d) / creep, float(self.m["burst_max_s"]))
            self.loco.set_drive(self.loco.ctrl.creep(creep, backward=d < 0, reason=f"inspect: 微調整 {d:+.0f}mm"), snake, person_xy)
            self._enter("CREEP", t)
        elif self.phase == "CREEP":
            if t >= self._burst_until:
                self.loco.anim.gait.stop(immediate=True)                 # 惰行させない（振幅を即 0）
                self.loco.stop("inspect: 微調整の後の静止")
                self._enter("SETTLE", t)
        elif self.phase == "WAIT_CHILD":
            self.loco.stop("inspect: 子どもが近いかもしれない。物を照らさずに待つ（CSAR R2）")
            if self._attention_ok(t):
                self._enter("CAPTURE", t, "近くに人がいないと確かめた → 撮影")
            elif t - float(self._wait_t or t) >= float(self.cfg["floor_watch"]["csar"]["capture_wait_s"]):
                self.phase, self.reason = "DEFERRED", ("CSAR: 子どもが近いかもしれない（または確かめられない）ので撮影を後回しにした。"
                                                       "候補は未確認（保護者への通知は Home AI）")
        elif self.phase == "CAPTURE":
            self.loco.stop("inspect: 撮影中（通常→斜め→線光→全消灯→通常）")
            if not self._attention_ok(t):                               # 撮影中に子どもが来た: 照らすのをやめて待つ
                self._to_capture(t, "撮影中に人が近づいた → 中断")
                return
            if t - self._phase_t >= float(self.m["capture_s"]):
                self.frames = self.capture()
                self._enter("JUDGE", t)
        elif self.phase == "JUDGE":
            try:
                self.result = self.judge(self.frames)
            except Exception as e:                                  # MotionError など: 撮り直し
                self._retries += 1
                if self._retries > int(self.m["max_retakes"]):
                    self.phase, self.reason = "FAILED", f"撮影をやり直しても判定できない: {e}"
                    return
                self._enter("SETTLE", t, f"撮り直し {self._retries}: {e}")
                return
            delta = self._aim_delta_deg()
            if delta is not None and self.aim is not None and self._aims < int(self.m["max_aims"]):
                self._aims += 1
                self.aim(t, delta)
                self._enter("SETTLE", t, f"線を候補へ向ける（頭ヨー {delta:+.1f}°）")
                return
            retreat = float(self.cfg["floor_watch"]["csar"]["retreat_mm"])
            if retreat > 0:                                              # 見つけたら物から離れる（CSAR R3）
                self._retreat_from = np.array([snake.x, snake.y])
                self._retreat_to = self._retreat_target(snake)
                if self._retreat_to is None:
                    self.loco.stop("inspect: 前へ離れる道が無い。後ろは見えないので下がらない（その場で頭をそらす）")
                    self._enter("DONE", t, "R3: 前へ離れる道が無い → その場に留まる（後ろへは下がらない）")
                    self.loco.set_drive(DriveCommand(False, reason="inspect: 完了（離れられない）"), snake, person_xy)
                    return
                self._enter("RETREAT", t, f"物から前へ回り込んで離れる（CSAR R3、目標 {self._retreat_to.round().tolist()}）")
                return
            self._enter("DONE", t)
            self.loco.set_drive(DriveCommand(False, reason="inspect: 完了"), snake, person_xy)
        elif self.phase == "RETREAT":
            moved = float(np.linalg.norm(np.array([snake.x, snake.y]) - self._retreat_from)) if self._retreat_from is not None else 0.0
            c = self.cfg["floor_watch"]["csar"]
            if moved >= float(c["retreat_mm"]) or t - self._phase_t >= float(c["retreat_timeout_s"]):
                self.loco.stop("inspect: 離れた")
                self._enter("DONE", t, f"{moved:.0f}mm 離れた")
                self.loco.set_drive(DriveCommand(False, reason="inspect: 完了"), snake, person_xy)
                return
            cmd = self.loco.ctrl.drive_to(t, snake, self._retreat_to, float(c["retreat_speed_mm_s"]), person_xy,
                                          allow_reverse=False)
            if cmd.blocked or (cmd.moving and cmd.params is not None and float(cmd.params.temporal_freq_hz) < 0):
                # controller はマット端で後退して向き直ることがある。離れるときは後ろが見えないので下がらない → その場に留まる
                self.loco.stop("inspect: 前が詰まった。後ろは見えないので下がらない")
                self._enter("DONE", t, f"R3: {moved:.0f}mm 離れたところで前が詰まった → 留まる（後退しない）")
                self.loco.set_drive(DriveCommand(False, reason="inspect: 完了（前が詰まった）"), snake, person_xy)
                return
            self.loco.set_drive(cmd, snake, person_xy)                    # 前向きに（頭のセンサーが見ている向きへ）

    def _retreat_target(self, snake: Any) -> np.ndarray | None:
        """物から離れる先: 進行方向から ±60/90/120° の候補のうち、そこへの直線が物から `retreat_clear_mm` 以上離れ、マットに余地がある点。
        **後ろ（尾の向き）へは行かない**（尾にセンサーが無く、子どもが後ろにいるかもしれない。Design ENTRY-0022）。無ければ None（留まる）。"""
        c = self.cfg["floor_watch"]["csar"]
        start = np.array([snake.x, snake.y], float)
        heading = float(snake.theta_head)                          # 頭の向き（前）
        dist = 2.0 * float(c["retreat_mm"])
        obj = self.object_mm
        best, best_clear = None, -1.0
        for deg in (60.0, -60.0, 90.0, -90.0, 120.0, -120.0):
            a = heading + math.radians(deg)
            tgt = start + dist * np.array([math.cos(a), math.sin(a)])
            if self.loco.ctrl.room_toward(snake, tgt) < float(c["retreat_mm"]):
                continue
            if obj is None:
                clear = float("inf")
            else:
                seg = tgt - start
                k = float(np.clip(np.dot(obj - start, seg) / max(float(seg @ seg), 1e-9), 0.0, 1.0))
                clear = float(np.linalg.norm(obj - (start + k * seg)))
            if clear >= float(c["retreat_clear_mm"]) and clear > best_clear:
                best, best_clear = tgt, clear
        return best

    def _attention_ok(self, t: float) -> bool:
        if self.attention_ok is None or not bool(self.cfg["floor_watch"]["csar"]["defer_capture_when_near"]):
            return True
        return bool(self.attention_ok(t))

    def _to_capture(self, t: float, note: str = "") -> None:
        """撮影へ。CSAR: 子どもが遠いと確かめられていなければ、物を照らさずに待つ（WAIT_CHILD）。"""
        if self._attention_ok(t):
            self._enter("CAPTURE", t, note)
            return
        if self._wait_t is None:
            self._wait_t = t
        self._enter("WAIT_CHILD", t, (note + " / " if note else "") + "子どもが近いかもしれない → 撮影を待つ")

    def _aim_delta_deg(self) -> float | None:
        """線を候補へ向け直すための頭ヨーの増分（カメラ x 右 → 右は負）。

        ずれ = 候補の横位置 − その前後位置で床の線が通る横位置（`line_x_mm`。検出が床の線から姿勢のずれを直した値。無ければ 0）。
        向け直すのは (a) 線が候補に当たっていない、または (b) 当たっているが中心から `line_aim_tol_mm` 以上ずれていて、まだ決め手
        （metal_disc か、測れた高さ）が無い候補。線が中心から 2mm ずれると鏡面の危険物の metal_disc が 3 分の 2 に、4mm で半分に落ちる
        （H2 VIS-0002、合成）。線を当てただけで満足しない。"""
        half_w = float(self.cfg["floor_watch"]["line_light"]["width_mm_initial"]) / 2
        tol = float(self.m.get("line_aim_tol_mm", half_w))

        def offset(c: Any) -> float:
            lx = getattr(c, "line_x_mm", None)
            return float(c.floor_xy_mm[0]) - (float(lx) if lx is not None and math.isfinite(lx) else 0.0)

        def decided(c: Any) -> bool:
            return getattr(c, "height_mm", None) is not None or any(k.get("kind") == "metal_disc" for k in getattr(c, "kinds", []))
        off = [c for c in (self.result or []) if float(c.floor_xy_mm[1]) > 0
               and ((not getattr(c, "on_line", True) and abs(offset(c)) > half_w) or (abs(offset(c)) > tol and not decided(c)))]
        if not off:
            return None
        c = max(off, key=lambda k: float(k.diameter_mm))
        x, y = offset(c), float(c.floor_xy_mm[1])
        delta = -math.degrees(math.asin(max(-1.0, min(1.0, x / (self.head_link_mm + y)))))
        lim = float(self.m["aim_max_deg"])
        return max(-lim, min(lim, delta))
