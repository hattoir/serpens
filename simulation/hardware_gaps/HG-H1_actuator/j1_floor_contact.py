"""J1（頭ピッチ）の床接触の較正 — 手順（アルゴリズム）と、不感帯・バックラッシが口の前縁の高さに与える誤差の見積もり。

**ACTUATOR_MODEL_SIM。実機の値ではない。** 不感帯 D・バックラッシ B・破断（動き出し）トルクは **UNKNOWN**（HG-H1 の prior に無い）。
ここでは値を仮定せず、格子で振って「誤差がどう決まるか」と「手順の選択（一方向から近づく）の効果」を見る。実測（`hardware/prototypes/H1_joint/j1_head_pitch.md`）で置き換える。

較正の手順（実機でも同じ）:
  1. 較正用の上限 τ_move（動かせる最小に近い値。実測 J1-4）を書く。窓の上（+1°）から、床へ向かって 1 ステップずつ目標を下げる（**必ず上から**）。
  2. 目標を下げたのに位置が追従しない（目標 − 読み値 ≥ lag_thresh のまま k_consec 回）= 床に触れた（上限で止まった）。その読み値 q_contact を記録する。
  3. そこから c 相当のステップだけ上げる（n_c = c / 0.0832 mm を最も近いステップへ）。**上げる向きは反転なので、不感帯・バックラッシが効く** → 上げたあと読み値を確かめ、足りなければ 1 ステップずつ足す。
  4. 作業中の上限 τ_hold（≲ 0.02 N·m）に落とす。

口の前縁の高さ = c₀（スキッドが床に触れたときの前縁のすき間。Design の設計値、既定 0.1 mm）+ (q − q_contact) × 0.0832 mm（軸から前縁 54.2 mm、4096/回転）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

STEP_DEG = 360.0 / 4096.0                                  # 0.0879°
ARM_FRONT_MM = 54.2                                        # J1 軸から口の前縁（Design、ENTRY-D-0008）
MM_PER_STEP = ARM_FRONT_MM * math.radians(STEP_DEG)        # 0.0832 mm


@dataclass
class ServoModel:
    """J1 の位置サーボ。**簡略化した ACTUATOR_MODEL_SIM**（不感帯・バックラッシは仮定の格子）。

    deadband_steps : 目標との差がこの以内なら駆動しない。近づくときは目標の手前 D で止まる（片側）
    backlash_steps : 向きを反転したとき、出力が動き出すまでにモーターが空転するステップ数
    encoder        : ENC_OUT = 位置の読み値は出力軸側（バックラッシが読み値に出る）/ ENC_MOTOR = モーター側（出ない）
    q_floor        : スキッドが床に触れる出力の位置 [step]。これより下へは行かない（床は剛。弾性・上限による押し込みは無視）
    """
    deadband_steps: float = 0.0
    backlash_steps: float = 0.0
    encoder: str = "ENC_OUT"
    q_floor: float = -20.0
    noise_steps: float = 0.0
    q_out: float = 12.0
    q_motor: float = 12.0
    _dir: int = 0
    rng: np.random.Generator = field(default_factory=lambda: np.random.default_rng(0))

    def read(self) -> int:
        q = self.q_out if self.encoder == "ENC_OUT" else self.q_motor
        return int(round(q + (self.rng.normal(0.0, self.noise_steps) if self.noise_steps else 0.0)))

    def goto(self, goal: int) -> None:
        """goal へ動けるだけ動く（追従に十分な時間を待った後の状態）。"""
        err = goal - self.read()
        if abs(err) <= self.deadband_steps:
            return
        d = 1 if err > 0 else -1
        target = goal - d * self.deadband_steps                         # 不感帯の縁で止まる
        move = target - (self.q_motor if self.encoder == "ENC_MOTOR" else self.q_out)
        if abs(move) < 1e-9:
            return
        lash = self.backlash_steps if (self._dir != 0 and d != self._dir) else 0.0
        self._dir = d
        self.q_motor += move
        self.q_out = max(self.q_out + move - d * min(abs(move), lash), self.q_floor)   # 出力はバックラッシの分だけ遅れ、床より下へ行かない

    def loaded(self) -> bool:
        """出力が床で止まり、モーターが上限で押している（負荷・電流が上限に張り付く）。実機では 現在負荷（60 番地）/ 電流（69 番地）で見る。"""
        return self.q_out <= self.q_floor + 1e-9 and self.q_motor < self.q_out - 0.5

    def release(self) -> None:
        """接触を検出したあと、モーター側の押し込み（床で止まった分）を戻す = 目標を現在の位置へ。"""
        self.q_motor = self.q_out


def lag_threshold(deadband_steps: float) -> int:
    """接触の判定に使う「目標 − 読み値」のしきい値。**不感帯より大きくしないと、不感帯だけで「接触」と誤判定する**（J1-1 の測定値で決める）。"""
    return int(math.ceil(deadband_steps)) + 2


def calibrate(servo: ServoModel, c_mm: float = 0.1, h_contact_mm: float = 0.0, q_start: int = 12, lag_thresh: int | None = None,
              k_consec: int = 3, max_steps: int = 200, signal: str | None = None) -> dict:
    """床接触の較正（手順どおり）。h_contact_mm = スキッドが床に触れたときの口の前縁の高さ（Design の配置 = 足の帯の底とスキッドの底が同一面なら 0。ASSUMED）。
    戻り値の err_mm = 口の前縁の高さの真値 − 狙い（c_mm）。"""
    lag_thresh = lag_threshold(servo.deadband_steps) if lag_thresh is None else lag_thresh
    signal = signal or ("position_lag" if servo.encoder == "ENC_OUT" else "load")     # モーター側のエンコーダは、床で止まっても位置が目標に追従する → 負荷で見る
    n_c = max(1, round(c_mm / MM_PER_STEP))                                 # 最も近いステップ（c = 0.1 → 1、0.3 → 4、1.0 → 12。Design と同じ）
    servo.goto(q_start)
    goal, stuck, q_contact = q_start, 0, None
    for _ in range(max_steps):
        goal -= 1
        servo.goto(goal)
        hit = servo.read() - goal >= lag_thresh if signal == "position_lag" else servo.loaded()
        stuck = stuck + 1 if hit else 0
        if stuck >= k_consec:
            q_contact = servo.read()
            break
    if q_contact is None:
        return {"ok": False, "reason": "no_contact"}
    servo.release()
    q_contact = servo.read()                                             # 押し込みを戻したあとの読み値を接触の位置とする
    target = q_contact + n_c
    servo.goto(target)
    for _ in range(6):                                                   # 足りなければ 1 ステップずつ足す（読み値が届くまで）
        if servo.read() >= target:
            break
        target += 1
        servo.goto(target)
    true = h_contact_mm + (servo.q_out - servo.q_floor) * MM_PER_STEP
    return {"ok": True, "q_contact": q_contact, "q_final": servo.read(), "n_c": n_c, "edge_mm_true": true, "err_mm": true - (h_contact_mm + c_mm),
            "q_contact_err_steps": q_contact - servo.q_floor}


def analytic_error_budget_steps(deadband_steps: float, backlash_steps: float, encoder: str, noise_steps: float = 0.0) -> dict:
    """口の前縁の高さの誤差の見積もり（ステップ。最悪側の和）。**モデルの式であり、実測ではない**。実測（J1-1〜J1-5）で D・B を置き換える。
    量子化 ±0.5、接触の読み値の雑音、上げる向きの不感帯 D（ENC_OUT では読み値で補正できるが、目標の手前 D で止まる）、
    バックラッシ B（ENC_MOTOR では読み値に出ないので、そのまま誤差になる。ENC_OUT では読み値で補正できる）。"""
    e = 0.5 + noise_steps + deadband_steps + (backlash_steps if encoder == "ENC_MOTOR" else 0.0)
    return {"steps": e, "mm": e * MM_PER_STEP}


if __name__ == "__main__":
    # 理想のサーボ（D = B = 0）での動作確認
    s = ServoModel(q_floor=-7.0)
    r = calibrate(s, c_mm=0.1)
    print(r)
    print(f"1 step = {MM_PER_STEP:.4f} mm（口の前縁）")
