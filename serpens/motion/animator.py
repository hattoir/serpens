"""キーフレーム補間・歩容・呼吸を重ね合わせて、9軸の指令角を作る。

出力角 = キーフレーム（ベース姿勢） + 歩容（J1〜J6、加算） + 呼吸（全軸、軸ごとの振幅で加算）

- キーフレーム補間: ease-in-out / ease-out / オーバーシュート付き ease-out（serpens/motion/easing.py）
  + 予備動作（anticipate）と減衰振動（settle）。速度上限の計算にはどちらも含める
- 頭（胴体ヨーより先の軸）は先端の加速度を animator.head_max_accel_g 以下に、
  head_min_rise_deg 以上の動きは立ち上がりを head_min_rise_s 以上にする（打撃に見える速さを構造的に出さない。
  ガラガラヘビの打撃は 506 m/s²・35ms: Higham et al., Sci. Rep. 2017）
- 呼吸: 全軸にサイン波を常時加算。ON/OFF は fade_s でなめらかに。呼吸は壁時計で回る
- freeze(): 出力をその場で固定（驚きの全停止）。解除すると、止まっていた時間ぶん
  補間・歩容が一時停止していたように続きから動く。keep_breath=True なら呼吸だけは続く
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from serpens.hw.servo_bus import Goal, ServoBus
from serpens.motion.easing import (Easing, Track, ease_in_out, ease_out, min_duration_for_accel_s,  # noqa: F401
                                   min_duration_s, overshoot_value)
from serpens.motion.gait import GaitEngine, body_joint_names
from serpens.motion.poses import Pose, Poses

G_MPS2 = 9.80665


@dataclass(frozen=True)
class Keyframe:
    """目標姿勢（一部の関節だけでもよい）と到達までの時間。

    order と stagger_s を与えると、order の順に stagger_s ずつ遅れて動き始める
    （とぐろを尾から順に巻く、など）。duration_s は各関節の補間時間。
    anticipate_deg / anticipate_lead_s … 本動作の lead 秒前に進行方向と逆へ deg だけ引く
    settle_amp_deg / settle_tau_s / settle_joints … 到達後に settle_joints を減衰振動で止める
    """

    pose: Pose
    duration_s: float
    easing: Easing = Easing.IN_OUT
    overshoot_deg: float = 0.0
    order: tuple[str, ...] = ()
    stagger_s: float = 0.0
    anticipate_deg: float = 0.0
    anticipate_lead_s: float = 0.0
    settle_amp_deg: float = 0.0
    settle_tau_s: float = 0.0
    settle_joints: tuple[str, ...] = ()

    def delay_of(self, name: str) -> float:
        """関節 name が動き始めるまでの遅れ [s]。"""
        return self.order.index(name) * self.stagger_s if name in self.order else 0.0


def rest_keyframe(poses: Poses) -> Keyframe:
    """休憩姿勢（緩い弧）へのキーフレーム。尾から順に曲げ、頭を最後に引き込む。

    旧「とぐろ」は R03 の可動域に入らないため `legacy_poses` にある（`Poses.legacy("coil")`）。
    """
    seq = poses.coil_sequence()
    return Keyframe(poses.rest(), float(seq["per_joint_s"]), Easing.IN_OUT,
                    order=tuple(seq["order"]), stagger_s=float(seq["stagger_s"]))


class Animator:
    """9軸の指令角を毎周期計算する。時刻 t は呼び出し側が与える（テストしやすいように）。"""

    def __init__(self, cfg: dict[str, Any]) -> None:
        self.names: list[str] = [j["name"] for j in cfg["joints"]]
        self._ids = {j["name"]: int(j["servo_id"]) for j in cfg["joints"]}
        self._limits = {j["name"]: (float(j["min_deg"]), float(j["max_deg"])) for j in cfg["joints"]}
        self._vmax = {j["name"]: float(j["max_speed_dps"]) for j in cfg["joints"]}
        a, b = cfg["animator"], cfg["breath"]
        self._accel = float(a["default_accel_dps2"])
        self._peak_ratio = float(a["overshoot_peak_ratio"])
        self._settle_hz = float(a["settle_freq_hz"])
        # 頭（胴体ヨーより先）: 先端の加速度と立ち上がり時間の下限
        body = set(body_joint_names(cfg))
        self.head_joints = [n for n in self.names if n not in body]
        neck_x = next(float(j["x_mm"]) for j in cfg["joints"] if j["name"] not in body)
        self._head_lever_mm = float(cfg["body"]["head_tip_x_mm"]) - neck_x
        self._head_max_accel = float(a["head_max_accel_g"]) * G_MPS2
        self._head_min_rise_s = float(a["head_min_rise_s"])
        self._head_min_rise_deg = float(a["head_min_rise_deg"])
        by_axis = b["amplitude_by_axis"]
        self._b_amp = {n: float(by_axis.get(n, b["default_amplitude_deg"])) for n in self.names}
        self._b_period = float(b["period_s"])
        self._b_phase = math.radians(float(b["phase_step_deg"]))
        self._b_fade = float(b["fade_s"])
        self.gait = GaitEngine(cfg)
        home = cfg["poses"]["home"]
        self.base: Pose = {n: float(home.get(n, 0.0)) for n in self.names}
        self._tracks: dict[str, Track] = {}
        self._breath_env = 1.0
        self._breath_target = 1.0
        self._frozen_at: float | None = None
        self._frozen_out: Pose = {}
        self._frozen_keep_breath = False
        self._last_breath: Pose = {n: 0.0 for n in self.names}
        self._paused_total = 0.0
        self._last_anim_t: float | None = None
        self.last_output: Pose = dict(self.base)
        # 歩容と呼吸を足す前のベース姿勢。駆動リンク経路で「胴体の姿勢」として送る
        self.last_base: Pose = dict(self.base)

    # ---- 時刻 -----------------------------------------------------------------
    def _anim_t(self, t: float) -> float:
        """freeze していた時間を差し引いたアニメーション時刻。"""
        return t - self._paused_total

    # ---- 操作 -----------------------------------------------------------------
    def _min_head_s(self, name: str, travel_deg: float) -> float:
        """頭の軸に課す最短時間（先端の加速度 ≤ head_max_accel、大きな動きは立ち上がり ≥ head_min_rise_s）。"""
        if name not in self.head_joints or travel_deg <= 0.0:
            return 0.0
        need = min_duration_for_accel_s(travel_deg, self._head_lever_mm, self._head_max_accel)
        if travel_deg >= self._head_min_rise_deg:
            need = max(need, self._head_min_rise_s)
        return need

    def play(self, kf: Keyframe, t: float) -> float:
        """キーフレームを開始する。始点は各関節の現在のベース角。

        どれかの関節が速度・加速度の上限を超える場合は、全関節そろえて時間を延ばす
        （予備動作の時間・減衰振動の振幅も同じ上限に収める）。
        戻り値: 最後の関節が到達するまでの時間 [s]（減衰振動は含まない）
        """
        at = self._anim_t(t)
        starts = {name: self._base_value(name, at) for name in kf.pose}
        pulls = {n: (-math.copysign(kf.anticipate_deg, d - starts[n]) if kf.anticipate_deg and d != starts[n] else 0.0)
                 for n, d in kf.pose.items()}
        need, lead_need = 0.0, 0.0
        for n, d in kf.pose.items():
            s0 = starts[n] + pulls[n]
            need = max(need, min_duration_s(s0, d, kf.easing, kf.overshoot_deg, self._peak_ratio, self._vmax[n]),
                       self._min_head_s(n, abs(d - s0)))
            if pulls[n]:
                lead_need = max(lead_need, min_duration_s(starts[n], s0, Easing.OUT, 0.0, 1.0, self._vmax[n]),
                                self._min_head_s(n, abs(pulls[n])))
        dur = max(kf.duration_s, need)
        lead = max(kf.anticipate_lead_s, lead_need) if any(pulls.values()) else 0.0
        settle = {n: min(kf.settle_amp_deg, self._vmax[n] / (2.0 * math.pi * self._settle_hz))
                  for n in kf.settle_joints} if kf.settle_amp_deg > 0.0 else {}
        for name, deg in kf.pose.items():
            self._tracks[name] = Track(starts[name], deg, at + kf.delay_of(name), dur, kf.easing, kf.overshoot_deg,
                                       lead, pulls[name], settle.get(name, 0.0), kf.settle_tau_s, self._settle_hz)
        for name, amp in settle.items():
            if name not in kf.pose:                       # 動かさない関節も、到達に合わせて揺れて止まる
                v = self._base_value(name, at)
                self._tracks[name] = Track(v, v, at, lead + dur, Easing.OUT, 0.0, 0.0, 0.0,
                                           amp, kf.settle_tau_s, self._settle_hz)
        return lead + dur + max((kf.delay_of(n) for n in kf.pose), default=0.0)

    def set_pose_now(self, pose: Pose) -> None:
        """補間なしでベース姿勢を置き換える。"""
        for name, deg in pose.items():
            self._tracks.pop(name, None)
            self.base[name] = deg

    def busy(self, t: float) -> bool:
        """到達前の関節があれば True（減衰振動中は含めない）。"""
        at = self._anim_t(t)
        return any(at < tr.t_arrive for tr in self._tracks.values())

    def busy_joint(self, name: str, t: float) -> bool:
        """関節 name が到達前なら True。"""
        tr = self._tracks.get(name)
        return tr is not None and self._anim_t(t) < tr.t_arrive

    def set_breathing(self, on: bool) -> None:
        """呼吸の ON/OFF（fade_s かけて振幅を変える）。"""
        self._breath_target = 1.0 if on else 0.0

    def freeze(self, t: float, keep_breath: bool = False) -> None:
        """出力をその場で固定する。keep_breath=True なら呼吸だけは続ける。"""
        if self._frozen_at is None:
            self._frozen_at = t
            self._frozen_keep_breath = keep_breath
            self._frozen_out = ({n: self.last_output[n] - self._last_breath[n] for n in self.names}
                                if keep_breath else dict(self.last_output))

    def unfreeze(self, t: float) -> None:
        """固定を解除し、止まっていた時間ぶん全体を一時停止扱いにする。"""
        if self._frozen_at is not None:
            self._paused_total += t - self._frozen_at
            self._frozen_at = None

    @property
    def breathing(self) -> bool:
        """呼吸が ON か（リンク経路では機体側で生成する）。"""
        return self._breath_target > 0.0

    @property
    def frozen(self) -> bool:
        return self._frozen_at is not None

    # ---- 計算 -----------------------------------------------------------------
    def _base_value(self, name: str, at: float) -> float:
        tr = self._tracks.get(name)
        if tr is None:
            return self.base[name]
        v = tr.value(at, self._peak_ratio)
        if at >= tr.t_done:
            self.base[name] = tr.end
            del self._tracks[name]
        return v

    def breath_offset(self, index: int, t: float) -> float:
        """関節 index の呼吸オフセット [deg]。t は壁時計（freeze で止めない）。"""
        return self._breath_env * self._b_amp[self.names[index]] * math.sin(
            2.0 * math.pi * t / self._b_period + index * self._b_phase)

    def _clamped(self, name: str, v: float) -> float:
        lo, hi = self._limits[name]
        return min(max(v, lo), hi)

    def update(self, t: float) -> Pose:
        """時刻 t の9軸指令角（ソフトウェアリミット内）。"""
        if self._frozen_at is not None:
            if not self._frozen_keep_breath:
                return dict(self._frozen_out)
            for i, name in enumerate(self.names):
                self._last_breath[name] = self.breath_offset(i, t)
            self.last_output = {n: self._clamped(n, self._frozen_out[n] + self._last_breath[n]) for n in self.names}
            return dict(self.last_output)
        at = self._anim_t(t)
        dt = 0.0 if self._last_anim_t is None else max(at - self._last_anim_t, 0.0)
        self._last_anim_t = at
        step = dt / self._b_fade if self._b_fade > 0 else 1.0
        self._breath_env += max(-step, min(step, self._breath_target - self._breath_env))
        gait = self.gait.update(at)
        out: Pose = {}
        base: Pose = {}
        for i, name in enumerate(self.names):
            base[name] = self._base_value(name, at)
            self._last_breath[name] = self.breath_offset(i, t)
            out[name] = self._clamped(name, base[name] + gait.get(name, 0.0) + self._last_breath[name])
        self.last_base = base
        self.last_output = out
        return out

    def send(self, bus: ServoBus, pose: Pose) -> None:
        """指令角をサーボバスへ送る（SYNC WRITE）。サーボ側の速度上限は軸ごとの max_speed_dps。"""
        bus.sync_set_goals({self._ids[n]: Goal(d, self._vmax[n], self._accel) for n, d in pose.items()})
