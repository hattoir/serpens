"""機体側の出力生成（歩容・補間・呼吸）と、指令値の上限検査。

device.py（通信と状態機械）から切り離してある。**ここは PC の設定では緩められない上限**を持ち、
通信を一切知らないので、数値としてそのまま試験できる（完了条件 1/11）。

歩容は serpens/motion/gait.py と同じ式:  α(n,t) = A·sin(Ω·n + ω·t) + γ0·n/N
"""
from __future__ import annotations

import math
from typing import Any

from serpens.link.messages import Body, Drive, Head
from serpens.motion.gait import body_joint_names

HEAD_MAX = 3            # HEAD 指令が運べる軸数（payload 固定長）。実際の軸数は config から決まる


class DeviceMotion:
    """機体が実際にサーボへ書く角度を作る。"""

    def __init__(self, cfg: dict[str, Any]) -> None:
        self.joints = {j["name"]: j for j in cfg["joints"]}
        self.body = body_joint_names(cfg)
        # 胴体ヨーより先（首・頭）。**関節数を決め打ちしない**（最小構成でも 9軸でも同じコードで動く）
        self.head = [n for n in self.joints if n not in self.body][:HEAD_MAX]
        self.limits = cfg["link"]["limits"]
        self.ttl_max_ms = int(cfg["link"]["drive_ttl_max_ms"])
        b = cfg["breath"]
        self.breath = b
        self.breath_amp = {n: float(b["amplitude_by_axis"].get(n, b["default_amplitude_deg"])) for n in self.joints}
        self.breath_phase = math.radians(float(b["phase_step_deg"]))
        home = cfg["poses"]["home"]
        self.goals = {n: float(home.get(n, 0.0)) for n in self.joints}
        self.target = dict(self.goals)
        self.speed = {n: float(j["max_speed_dps"]) for n, j in self.joints.items()}
        self.phase = 0.0                     # 時間位相 [rad]（周波数を変えても飛ばない）

    def clamp(self, name: str, deg: float) -> float:
        """ソフトリミットへ収める（上限検査を通った後の最後の砦）。"""
        j = self.joints[name]
        return min(max(deg, float(j["min_deg"])), float(j["max_deg"]))

    # ---- 上限の強制 -------------------------------------------------------------------
    def drive_ok(self, d: Drive) -> bool:
        """DRIVE の値が機体の絶対上限に収まっているか。"""
        lim = self.limits
        body_max = min(float(self.joints[n]["max_deg"]) for n in self.body)
        return bool(1 <= d.ttl_ms <= self.ttl_max_ms
                    and 0.0 <= d.amplitude_deg <= float(lim["amplitude_deg"])
                    and 0.0 < d.spatial_freq_deg <= float(lim["spatial_freq_deg"])
                    and abs(d.temporal_freq_hz) <= float(lim["temporal_freq_hz"])
                    and abs(d.gamma_deg) <= float(lim["gamma_deg"])
                    and d.amplitude_deg + abs(d.gamma_deg) <= body_max)   # 合成しても範囲内

    def body_ok(self, b: Body) -> bool:
        """BODY（とぐろ・鎌首などの胴体姿勢）の角度・速度がソフトリミット内か。"""
        if not 1 <= b.ttl_ms <= self.ttl_max_ms:
            return False
        if not 0.0 < b.speed_dps <= float(self.limits["body_speed_dps"]):
            return False
        for name, deg in zip(self.body, b.angles_deg):
            j = self.joints[name]
            if not float(j["min_deg"]) <= deg <= float(j["max_deg"]):
                return False
        return True

    def head_ok(self, h: Head) -> bool:
        """HEAD の角度・速度が各軸のソフトリミット内か。"""
        if not 1 <= h.ttl_ms <= self.ttl_max_ms:
            return False
        if not 0.0 < h.speed_dps <= float(self.limits["head_speed_dps"]):
            return False
        for name, deg in zip(self.head, (h.j7_deg, h.j8_deg, h.j9_deg)):
            j = self.joints[name]
            if not float(j["min_deg"]) <= deg <= float(j["max_deg"]):
                return False
        return True

    # ---- 出力 -----------------------------------------------------------------------
    def set_head(self, h: Head) -> None:
        """頭部の目標角と速度を入れる（検査済みの値だけ渡すこと）。"""
        for name, deg in zip(self.head, (h.j7_deg, h.j8_deg, h.j9_deg)):
            self.target[name], self.speed[name] = deg, h.speed_dps

    def set_body(self, b: Body) -> None:
        """胴体ヨーの目標角を入れる（検査済みの値だけ渡すこと）。"""
        for name, deg in zip(self.body, b.angles_deg):
            self.target[name], self.speed[name] = deg, b.speed_dps

    def stop_body(self) -> None:
        """胴体だけ目標を現在角にする（BODY の期限切れ）。"""
        for n in self.body:
            self.target[n] = self.goals[n]

    def set_pose(self, pose: dict[str, Any]) -> None:
        """姿勢プリセットを目標にする（各軸の max_speed_dps で移る）。"""
        for n, j in self.joints.items():
            if n in pose:
                self.target[n] = self.clamp(n, float(pose[n]))
                self.speed[n] = float(j["max_speed_dps"])

    def hold(self) -> None:
        """いまの角度で保持する（停止時。ホーム姿勢へは動かさない）。"""
        self.target = dict(self.goals)

    def stop_head(self) -> None:
        """頭部だけ目標を現在角にする（HEAD の期限切れ）。"""
        for n in self.head:
            self.target[n] = self.goals[n]

    def step(self, dt: float, drive: Drive | None) -> None:
        """制御1周期ぶん進める。drive があれば胴体は歩容、それ以外は速度制限つきで目標へ。"""
        if drive is not None:
            self.phase += 2.0 * math.pi * drive.temporal_freq_hz * dt
            big = math.radians(drive.spatial_freq_deg)
            top = max(len(self.body) - 1, 1)
            for n, name in enumerate(self.body):          # γ(n) = γ0·n/N（head_weighted）
                a = drive.amplitude_deg * math.sin(big * n + self.phase) + drive.gamma_deg * n / top
                self.goals[name] = self.clamp(name, a)
        for name in self.goals:
            if drive is not None and name in self.body:
                continue
            step = self.speed[name] * dt
            diff = self.clamp(name, self.target[name]) - self.goals[name]
            self.goals[name] += max(-step, min(step, diff))

    def output(self, t: float, breathing: bool) -> dict[str, float]:
        """サーボへ書く角度。呼吸（軸ごとの振幅・尾→頭の位相勾配）はここで足す（PC 側 animator と同じ式）。"""
        out = dict(self.goals)
        if breathing:
            w = 2.0 * math.pi * t / float(self.breath["period_s"])
            for i, name in enumerate(self.joints):
                out[name] = self.clamp(name, out[name] + self.breath_amp[name] * math.sin(w + i * self.breath_phase))
        return out
