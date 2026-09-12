"""「いま何を考えているか」を日本語の文にする。

効用の数値をそのまま並べず、読んで分かる1行にする。優先順位:
  1. 安全（過熱・掴まれた・マット端）
  2. 撫でられている
  3. 人まで stop_distance_mm で止まっている
  4. 状態が変わった理由（効用の比較）
  5. 今の状態を続けている理由（最小継続時間）
"""
from __future__ import annotations

import math
from typing import Any

from serpens.behavior.brain import BrainStatus
from serpens.behavior.utility import DRIVER_NAMES, STATE_LABELS_JA

# 内部状態の名前 → 日本語（文の中で使う）
DRIVER_JA = {"Curiosity": "好奇心", "Affection": "親しみ", "Attention": "注意", "Stress": "警戒",
             "Rest": "休みたさ", "Sleepy": "眠さ", "Novelty": "目新しさ", "Touch": "触られた感じ", "Heat": "熱さ"}
ACTION_JA = {"SLEEP": "眠ることにした", "PATROL": "見回ることにした", "ALERT": "身構えた", "OBSERVE": "様子を見ることにした",
             "APPROACH": "近づくことにした", "ENGAGE": "かかわることにした", "PETTED": "力を抜いた",
             "RETREAT": "離れることにした", "COIL_REST_MOOD": "とぐろを巻いて休むことにした",
             "COIL_REST_HEAT": "熱いので休むことにした"}


def sentence(st: BrainStatus, cfg: dict[str, Any], person_dist_mm: float | None) -> str:
    """状態と数値から、日本語1行を作る。"""
    if st.safety:
        if "過熱" in st.safety:
            heat = f"{st.heat_c:.0f}℃" if st.heat_c is not None else "高温"
            return f"サーボが{heat}。冷えるまで休む"
        if "掴まれた" in st.safety:
            return "強く掴まれた。力を抜いて待つ"
        return f"安全のため止まった（{st.safety}）"
    if st.state == "PETTED":
        return "撫でられている。力を抜いて、じっとしている"
    stop_mm = float(cfg["behavior"]["controller"]["stop_distance_mm"])
    if "停止: 人の" in st.drive or "安全: 人まで" in st.drive:
        near = f"人まで約 {person_dist_mm:.0f}mm。" if person_dist_mm is not None else ""
        return f"{near}これ以上は近づかない"
    if "マット端" in st.drive:
        return "台の端に来た。下がって向きを変える"
    if st.state == "APPROACH" and person_dist_mm is not None:
        return f"{_compare(st)}、近づくことにした（人まで約 {person_dist_mm:.0f}mm、{stop_mm:.0f}mm 手前で止まる）"
    if st.time_to_next_s > 0.05 and not _leading(st):
        return f"{STATE_LABELS_JA[st.state]}を続ける（あと {st.time_to_next_s:.1f} 秒）"
    return f"{_compare(st)}、{ACTION_JA.get(st.state, STATE_LABELS_JA[st.state])}"


def _ranked(st: BrainStatus) -> list[str]:
    return sorted(st.utilities, key=lambda k: st.utilities[k], reverse=True)


def _leading(st: BrainStatus) -> bool:
    return _ranked(st)[0] == st.state


def _compare(st: BrainStatus) -> str:
    """「好奇心 0.82 が 休みたさ 0.31 を上回ったので」の部分。"""
    rival = next((k for k in _ranked(st) if k != st.state), st.state)
    mine = DRIVER_JA.get(DRIVER_NAMES[st.state], DRIVER_NAMES[st.state])
    theirs = DRIVER_JA.get(DRIVER_NAMES[rival], DRIVER_NAMES[rival])
    return f"{mine} {st.utilities[st.state]:.2f} が {theirs} {st.utilities[rival]:.2f} を上回ったので"


def head_distance_mm(snake_xy: tuple[float, float] | None, person_xy: tuple[float, float] | None,
                     cfg: dict[str, Any]) -> float | None:
    """頭先端から人までのおおよその距離。"""
    if snake_xy is None or person_xy is None:
        return None
    d = math.hypot(person_xy[0] - snake_xy[0], person_xy[1] - snake_xy[1])
    return max(d - float(cfg["behavior"]["controller"]["head_reach_mm"]), 0.0)
