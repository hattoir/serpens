"""危険度の判定（決定事項 9: サイズのしきい値 1 つで決めない）。

  score = max_k( conf_k × kind_ingestion_k ) × size_factor × height_factor × distance_factor × reach_factor
  ボタン電池・磁石・薬（critical_kinds）は score とは**別枠**: 候補にあれば確信度が低くても mandatory_notify。
係数は config の floor_watch.risk（DESIGN 値。根拠は未）。
"""
from __future__ import annotations

from typing import Any


def size_factor(diameter_mm: float, lo: float, hi: float) -> float:
    """飲み込みうる大きさなら 1、外れるほど下げる（急に 0 にしない）。"""
    if diameter_mm != diameter_mm:                       # nan
        return 0.7
    if lo <= diameter_mm <= hi:
        return 1.0
    if diameter_mm < lo:
        return max(0.2, diameter_mm / lo)
    return max(0.1, hi / diameter_mm)


def assess(candidates: list[dict[str, Any]], diameter_mm: float, height_mm: float, height_measured: bool,
           line_dropout: bool, child_distance_m: float | None, child_reachable: bool, cfg: dict[str, Any]) -> dict[str, Any]:
    """floor_finding.risk を作る。candidates = [{kind, confidence}]。"""
    r = cfg["floor_watch"]["risk"]
    lo, hi = (float(v) for v in r["ingestion_size_mm"])
    ing = max((float(c["confidence"]) * float(r["kind_ingestion"].get(c["kind"], 0.5)) for c in candidates), default=0.0)
    sharp = max((float(c["confidence"]) * float(r["kind_sharp"].get(c["kind"], 0.3)) for c in candidates), default=0.0)
    sf = size_factor(diameter_mm, lo, hi)
    hf = 1.0 if (height_measured or line_dropout) else 0.5     # 高さが取れなくても 0 にしない（鏡面は測れない）
    near = float(r["child_near_m"])
    df = 1.0 if child_distance_m is None or child_distance_m <= near else max(0.3, near / child_distance_m)
    rf = 1.0 if child_reachable else float(r["unreachable_factor"])
    score = min(1.0, ing * sf * hf * df * rf)
    crit = [c["kind"] for c in candidates
            if c["kind"] in r["critical_kinds"] and float(c["confidence"]) >= float(r["critical_min_confidence"])]
    rationale = [f"種類×確信度 {ing:.2f}", f"大きさ {diameter_mm:.0f}mm → ×{sf:.2f}",
                 ("高さあり" if height_measured else ("線の途切れ" if line_dropout else "高さ不明 ×0.5")),
                 ("子どもの距離 不明 → 近いとみなす" if child_distance_m is None else f"子どもの距離 {child_distance_m:.1f}m → ×{df:.2f}"),
                 ("届く" if child_reachable else f"届かない ×{rf}")]
    if crit:
        rationale.append("危険物の候補 " + ",".join(crit) + " → 確信度に関わらず通知")
    return {"ingestion": round(min(1.0, ing * sf), 3), "sharp": round(min(1.0, sharp), 3), "child_reachable": child_reachable,
            "child_distance_m": child_distance_m, "score": round(score, 3), "mandatory_notify": bool(crit),
            "critical_kinds": sorted(set(crit)), "rationale": rationale[:10]}
