"""各状態の効用（手書きの式）。係数は config の behavior.utility.weights。

記号: p = 人がいる(0/1)、near = 頭先端から人まで engage_distance 以内、
      c = Curiosity, a = Affection, s = Stress, n = Attention, e = Energy, nov = novelty
  SLEEP     = W · (1−p)(1−c)(1−n) · (0.5 + 0.5(1−e))
  PATROL    = W · (1−p) · c · e
  ALERT     = W · nov
  OBSERVE   = W · p · n · (1−s) · (1 − 0.5c)
  APPROACH  = W · p · (1−near) · c · (0.5 + 0.5a) · e · (1−s)
  ENGAGE    = W · p · near · (0.5 + 0.5a) · (0.3 + 0.7n) · (1−s)
            （near: 頭先端から engage_distance 以内、またはマット端でこれ以上近づけない）
  PETTED    = W · touch
  RETREAT   = W · p · s
  COIL_REST_MOOD = W · (1−e) · (1 − 0.5p)
  COIL_REST_HEAT … 効用では選ばない。過熱の安全割り込みでのみ入る（最小60秒・割り込み不可）
最後に各効用へ ±noise_ratio の乱数を掛ける（noise_period_s ごとに引き直す）。
"""
from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Any

from serpens.behavior.internal_state import InternalState

STATES = ("SLEEP", "PATROL", "ALERT", "OBSERVE", "APPROACH", "ENGAGE", "PETTED", "RETREAT",
          "COIL_REST_MOOD", "COIL_REST_HEAT")
STATE_LABELS_JA = {"SLEEP": "眠る", "PATROL": "巡回", "ALERT": "警戒", "OBSERVE": "観察", "APPROACH": "接近",
                   "ENGAGE": "かかわる", "PETTED": "撫でられ", "RETREAT": "退避",
                   "COIL_REST_MOOD": "とぐろで休む", "COIL_REST_HEAT": "熱いので休む"}
COIL_STATES = ("COIL_REST_MOOD", "COIL_REST_HEAT")
# 「いま何を考えているか」に出す、各状態の主な理由（内部状態の名前）
DRIVER_NAMES = {"SLEEP": "Sleepy", "PATROL": "Curiosity", "ALERT": "Novelty", "OBSERVE": "Attention",
                "APPROACH": "Curiosity", "ENGAGE": "Affection", "PETTED": "Touch", "RETREAT": "Stress",
                "COIL_REST_MOOD": "Rest", "COIL_REST_HEAT": "Heat"}


@dataclass(frozen=True)
class Context:
    """効用の計算に使う外界の情報。"""

    person: bool
    head_dist_mm: float | None
    touch: float
    novelty: float
    at_limit: bool = False     # マット端まで来て、これ以上人に近づけない（near とみなす）


@dataclass(frozen=True)
class Evaluation:
    """1回の評価結果。"""

    raw: dict[str, float]
    noisy: dict[str, float]
    drivers: dict[str, float]      # 各状態の主な理由の値


class UtilityModel:
    """効用の計算器。"""

    def __init__(self, cfg: dict[str, Any], rng: random.Random) -> None:
        u = cfg["behavior"]["utility"]
        self.w = {k: float(u["weights"][k]) for k in STATES}
        self.noise = float(u["noise_ratio"])
        self.noise_period = float(u["noise_period_s"])
        self.engage_mm = float(u["engage_distance_mm"])
        self.rng = rng
        self._factors = {k: 1.0 for k in STATES}
        self._next_noise_t = -1.0

    def evaluate(self, st: InternalState, ctx: Context, t: float) -> Evaluation:
        c, a, s, n, e = st.curiosity, st.affection, st.stress, st.attention, st.energy
        p = 1.0 if ctx.person else 0.0
        close = ctx.head_dist_mm is not None and ctx.head_dist_mm <= self.engage_mm
        near = 1.0 if (ctx.person and (close or ctx.at_limit)) else 0.0
        drv = {"SLEEP": (1 - c) * (1 - n), "PATROL": c, "ALERT": ctx.novelty, "OBSERVE": n, "APPROACH": c,
               "ENGAGE": a, "PETTED": ctx.touch, "RETREAT": s, "COIL_REST_MOOD": 1 - e, "COIL_REST_HEAT": 1 - e}
        raw = {
            "SLEEP": (1 - p) * (1 - c) * (1 - n) * (0.5 + 0.5 * (1 - e)),
            "PATROL": (1 - p) * c * e,
            "ALERT": ctx.novelty,
            "OBSERVE": p * n * (1 - s) * (1 - 0.5 * c),
            "APPROACH": p * (1 - near) * c * (0.5 + 0.5 * a) * e * (1 - s),
            "ENGAGE": p * near * (0.5 + 0.5 * a) * (0.3 + 0.7 * n) * (1 - s),
            "PETTED": ctx.touch,
            "RETREAT": p * s,
            "COIL_REST_MOOD": (1 - e) * (1 - 0.5 * p),
            "COIL_REST_HEAT": 0.0,
        }
        raw = {k: self.w[k] * v for k, v in raw.items()}
        if t >= self._next_noise_t:
            self._factors = {k: 1.0 + self.rng.uniform(-self.noise, self.noise) for k in STATES}
            self._next_noise_t = t + self.noise_period
        noisy = {k: v * self._factors[k] for k, v in raw.items()}
        return Evaluation(raw, noisy, drv)


def thought_line(ev: Evaluation, chosen: str, hold_s: float = 0.0) -> str:
    """「いま何を考えているか」の日本語1行。

    例: Curiosity 0.82 > Rest 0.31 → 接近
        最小継続時間で今の状態を保っているとき: Curiosity 0.52 < Attention 0.61 → 接近を続ける（あと 2.1 秒）
    数値は効用（乱数込み）、名前はその状態の主な理由。
    """
    u = ev.noisy
    rival = max((k for k in u if k != chosen), key=lambda k: u[k])
    left = f"{DRIVER_NAMES[chosen]} {u[chosen]:.2f}"
    right = f"{DRIVER_NAMES[rival]} {u[rival]:.2f}"
    if u[chosen] >= u[rival]:
        return f"{left} > {right} → {STATE_LABELS_JA[chosen]}"
    return f"{left} < {right} → {STATE_LABELS_JA[chosen]}を続ける（あと {hold_s:.1f} 秒）"
