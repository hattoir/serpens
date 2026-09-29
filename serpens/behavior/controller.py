"""移動制御: 目標点 → 歩容パラメータ（旋回オフセット γ0 と時間周波数）。

  - 向き: γ0 = heading_gain × (目標の方位 − θ_body[1周期の移動平均])、±max_turn_deg にクランプ
  - 速さ: 周波数 f = 速さ / advance_per_cycle_mm。プリセットの周波数を上限にする
          （速さのために周期を短くはしない。遅くするときだけ周期を延ばす）
  - 人の near_person_mm 以内では最高速度 near_speed_limit_mm_s
  - 頭先端から人まで stop_distance_mm で必ず停止。歩容は止めてから blend_s かけて振幅が消えるので、
    そのぶんの惰性（速さ × blend_s × coast_ratio）だけ早めに止める
  - 仮想フェンス。旋回半径（γ=20° で約390mm）がマット 1.2m に対して大きいので、
    端に来てから曲がったのでは間に合わない。次の2段構え:
      柔 fence_soft_mm … 頭先端から端までが近いほど、目標の方位を中央の方位へ寄せる（早めに曲がる）
      硬 mat_margin_mm … forward →（端に迫る）→ back（後退しながら中央へ向き直る）→ forward。
                          前進で回ると旋回半径が大きくて頭が壁に当たるので、必ず後退しながら回る。
                          back には最小継続時間があり、条件式だけで切り替えて往復するのを防ぐ
  - ただし edge_is_goal=True（マットの外にいる人へ近づく）で、目標の方を向いたまま端に来たときは
    「これ以上近づけない」として止まる（blocked=True）
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from serpens.motion.gait import GaitParams
from serpens.perception.snake_pose import SnakePose, wrap_pi

BACK_ENTER_DEG = 120.0         # 中央との向きの差がこれ以上なら（＝壁を向いている）後退を始める
BACK_EXIT_DEG = 60.0           # これ以下まで向き直ったら後退をやめる（ヒステリシス）


def wrap_pi_deg(deg: float) -> float:
    """角度を (-180, 180] に折り返す。"""
    return math.degrees(wrap_pi(math.radians(deg)))


@dataclass(frozen=True)
class DriveCommand:
    """移動の指令。moving=False なら歩容を止める。"""

    moving: bool
    params: GaitParams | None = None
    gamma0_deg: float = 0.0
    speed_mm_s: float = 0.0
    reason: str = ""
    blocked: bool = False       # マット端に来て、目標にこれ以上近づけない


class Controller:
    """目標点へ向かう移動の指令を作る。"""

    def __init__(self, cfg: dict[str, Any]) -> None:
        c = cfg["behavior"]["controller"]
        self.c = c
        self.base = GaitParams.from_cfg(cfg["gait"]["presets"][c["gait"]])
        self.stalk = GaitParams.from_cfg(cfg["gait"]["presets"][c["stalk_gait"]])
        self._advance = {"base": float(c["advance_per_cycle_mm"]), "stalk": float(c["stalk_advance_per_cycle_mm"])}
        self.w, self.d = float(cfg["mat"]["width_mm"]), float(cfg["mat"]["depth_mm"])
        self.phase = "forward"       # forward → back（後退しながら向き直る）→ forward
        self._phase_t = 0.0
        self._turn_sign = 1.0
        self.coast_s = float(cfg["gait"]["blend_s"]) * float(c["coast_ratio"])
        m = cfg["markers"]
        self.marker_span_mm = float(m["neck_x_mm"]) - float(m["tail_x_mm"])   # 首マーカ↔尾マーカ

    # ---- 補助 -----------------------------------------------------------------
    def head_distance(self, pose: SnakePose, person_xy: np.ndarray) -> float:
        """頭先端（首から head_reach_mm 先と近似）から人までの距離 [mm]。"""
        return float(np.linalg.norm(np.asarray(person_xy) - [pose.x, pose.y])) - float(self.c["head_reach_mm"])

    def speed_limit(self, pose: SnakePose, person_xy: np.ndarray | None, wanted: float) -> float:
        """人の近くでは最高速度を制限する。"""
        if person_xy is not None and np.linalg.norm(np.asarray(person_xy) - [pose.x, pose.y]) <= float(self.c["near_person_mm"]):
            return min(wanted, float(self.c["near_speed_limit_mm_s"]))
        return wanted

    def _params(self, speed: float, backward: bool = False, gait: str = "base") -> GaitParams:
        """速さ → 歩容パラメータ。gait="stalk" は忍び寄り（小振幅・低周波）。"""
        base = self.stalk if gait == "stalk" else self.base
        f_max = abs(base.temporal_freq_hz)
        f = min(max(speed / self._advance[gait], float(self.c["min_temporal_freq_hz"])), f_max)
        return replace(base, temporal_freq_hz=-f if backward else f, turn_bias_deg=0.0)

    def creep(self, speed_mm_s: float, backward: bool = False, reason: str = "微調整") -> DriveCommand:
        """向きを変えずに少しだけ進む / 下がる（inspect の位置合わせ。人との距離の安全は set_drive 側でかかる）。"""
        return DriveCommand(True, self._params(speed_mm_s, backward=backward), 0.0, speed_mm_s, reason)

    def _heading_error(self, pose: SnakePose, target: np.ndarray) -> float:
        b = math.atan2(target[1] - pose.y, target[0] - pose.x)
        return math.degrees(wrap_pi(b - pose.theta_body))

    def near_edge(self, pose: SnakePose) -> bool:
        """頭先端がマット端に迫っているか（首から head_reach_mm 先を見る）。

        円を描いて回っているときは、まっすぐ先を長く読むと外向きの瞬間に毎回反応してしまうので、
        先読みは頭の長さぶんだけにして、早めの回避は柔らかいフェンス（fence_soft_mm）に任せる。
        """
        la, m = float(self.c["head_reach_mm"]), float(self.c["mat_margin_mm"])
        x = pose.x + la * math.cos(pose.theta_body)
        y = pose.y + la * math.sin(pose.theta_body)
        return not (m <= x <= self.w - m and m <= y <= self.d - m)

    def center(self) -> np.ndarray:
        return np.array([self.w / 2, self.d / 2])

    def head_tip(self, pose: SnakePose) -> np.ndarray:
        """頭先端の推定位置（首から head_reach_mm 先）。"""
        u = np.array([math.cos(pose.theta_body), math.sin(pose.theta_body)])
        return np.array([pose.x, pose.y]) + float(self.c["head_reach_mm"]) * u

    def edge_distance(self, pose: SnakePose) -> float:
        """頭先端からマットの4辺までの最短距離 [mm]（外に出ていれば負）。"""
        t = self.head_tip(pose)
        return float(min(t[0], self.w - t[0], t[1], self.d - t[1]))

    def _inside(self, p: np.ndarray) -> bool:
        m = float(self.c["mat_margin_mm"])
        return bool(m <= p[0] <= self.w - m and m <= p[1] <= self.d - m)

    def tail_has_room(self, pose: SnakePose) -> bool:
        """後退したときに尾がマット端に当たらないか（尾マーカの位置から先読みして判定）。"""
        u = np.array([math.cos(pose.theta_body), math.sin(pose.theta_body)])
        tail = np.array([pose.x, pose.y]) - self.marker_span_mm * u
        p = tail - float(self.c["lookahead_mm"]) * u
        m = float(self.c["tail_margin_mm"])       # 尾は物理的にマットに載っていればよい（フェンスの余裕は不要）
        return bool(m <= p[0] <= self.w - m and m <= p[1] <= self.d - m)

    # ---- 指令 -----------------------------------------------------------------
    def drive_to(self, t: float, pose: SnakePose, target_xy: np.ndarray, speed_mm_s: float,
                 person_xy: np.ndarray | None = None, stop_at_person: bool = False,
                 edge_is_goal: bool = False, gait: str = "base", allow_reverse: bool = True) -> DriveCommand:
        """target_xy へ向かう指令を作る。gait="stalk" で忍び寄りの波形にする。

        allow_reverse=False: マット端で「後退しながら向き直る」をしない（止まって blocked を返す）。尾にセンサーが無いので、
        人のいる所（Floor Watch）では見えない後ろへ下がらない（Design integration-log ENTRY-0022）。展示は従来どおり True。"""
        c = self.c
        speed = self.speed_limit(pose, person_xy, speed_mm_s)
        if person_xy is not None and stop_at_person and \
                self.head_distance(pose, person_xy) <= float(c["stop_distance_mm"]) + speed * self.coast_s:
            return DriveCommand(False, reason=f"停止: 人の {c['stop_distance_mm']:.0f}mm 手前", blocked=True)
        err_target = self._heading_error(pose, np.asarray(target_xy, float))
        err_center = self._heading_error(pose, self.center())
        turn_max = float(c["max_turn_deg"])
        gain = float(c["heading_gain"])
        if edge_is_goal and self.near_edge(pose) and abs(err_target) <= float(c["recover_heading_ok_deg"]):
            return DriveCommand(False, reason="マット端に着いた（これ以上近づけない）", blocked=True)
        # --- 仮想フェンスの3段階（最小継続時間つき。条件式だけで切り替えると往復して動けなくなる） ---
        soft, hard = float(c["fence_soft_mm"]), float(c["mat_margin_mm"])
        edge_d = self.edge_distance(pose)                # 頭先端から端まで（外なら負）
        if self.phase == "forward" and edge_d <= hard and self._phase_done(t, float(c["forward_min_s"])):
            self._enter("back", t)
            # 回る向きは入口で決める（中央がほぼ真後ろだと、誤差の符号が毎周期反転するため）
            self._turn_sign = math.copysign(1.0, err_center if abs(err_center) > 1e-6 else 1.0)
        elif self.phase == "back" and self._phase_done(t, float(c["back_min_s"])) and (
                (edge_d >= float(c["back_exit_mm"]) and abs(err_center) <= float(c["recover_heading_ok_deg"]))
                or not self.tail_has_room(pose)):    # 後退では尾が先頭。尾の余地が無くなったらやめる
            self._enter("forward", t)

        if self.phase == "back" and not allow_reverse:
            self._enter("forward", t)
            return DriveCommand(False, reason=f"マット端（端まで {edge_d:.0f}mm）: 後ろが見えないので下がらずに止まる", blocked=True)
        if self.phase == "back":
            # 後退しながら中央へ向き直る。前進で回ると旋回半径が大きく、頭が壁に当たる。
            # 後退では γ の符号と回る向きが逆になるので符号を反転する
            g = -turn_max * self._turn_sign
            return DriveCommand(True, self._params(speed, backward=True), g, speed,
                                f"マット端: 後退しながら中央へ向き直る（端まで {edge_d:.0f}mm, あと {abs(err_center):.0f}°）")
        # 通常: 端に近いほど、目標の方位を中央の方位へ寄せる（柔らかいフェンス）
        w = 0.0 if edge_d >= soft else min(max((soft - edge_d) / max(soft - hard, 1e-6), 0.0), 1.0)
        err = (1.0 - w) * err_target + w * err_center
        g = self._clamp_turn(gain * err, turn_max)
        note = "" if w == 0.0 else f"／端に寄ったので中央へ {w:.0%}"
        return DriveCommand(True, self._params(speed, gait=gait), g, speed, f"目標へ（向きの誤差 {err_target:+.0f}°{note}）")

    def _enter(self, phase: str, t: float) -> None:
        self.phase, self._phase_t = phase, t

    def _phase_done(self, t: float, min_s: float) -> bool:
        return t - self._phase_t >= min_s

    @staticmethod
    def _clamp_turn(value: float, turn_max: float) -> float:
        return max(-turn_max, min(turn_max, value))

    def arrived(self, pose: SnakePose, target_xy: np.ndarray) -> bool:
        return float(np.linalg.norm(np.asarray(target_xy) - [pose.x, pose.y])) <= float(self.c["arrive_mm"])

    def room_toward(self, pose: SnakePose, target_xy: np.ndarray) -> float:
        """target の方向へ、首の先読み点がマット端（margin）に届くまで進める距離 [mm]。"""
        start = np.array([pose.x, pose.y])
        d = np.asarray(target_xy, float) - start
        n = float(np.linalg.norm(d))
        if n == 0.0:
            return 0.0
        u = d / n
        la, m = float(self.c["lookahead_mm"]), float(self.c["mat_margin_mm"])
        lim = np.inf
        for k, (lo, hi) in enumerate(((m, self.w - m), (m, self.d - m))):
            if u[k] > 1e-9:
                lim = min(lim, (hi - start[k]) / u[k])
            elif u[k] < -1e-9:
                lim = min(lim, (lo - start[k]) / u[k])
        return max(float(lim) - la, 0.0)

    def retreat_point(self, pose: SnakePose, person_xy: np.ndarray) -> np.ndarray:
        """人から離れる方向の点（マットの内側にクランプ）。"""
        away = np.array([pose.x, pose.y]) - np.asarray(person_xy, float)
        n = float(np.linalg.norm(away))
        away = away / n if n > 0 else np.array([1.0, 0.0])
        p = np.array([pose.x, pose.y]) + away * float(self.c["retreat_distance_mm"])
        m = float(self.c["mat_margin_mm"])
        return np.clip(p, [m, m], [self.w - m, self.d - m])
