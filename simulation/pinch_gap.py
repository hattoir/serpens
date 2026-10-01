"""関節の挟み込み（指）: すき間が関節の角度でどう変わるかを、基準に照らして判定する。**KINEMATIC_SIM / CAD_CONCEPT の値を使う判定。**

基準（config `safety_limits.pinch`。**厳しい側を採る**）:
  - ASTM F963-11 4.18.1（96 か月未満の子ども）: 動く部分の間の、手の届くすき間が 5mm の棒を通すなら 13mm の棒も通すこと
    = すき間は「全ての位置で 5mm 未満」か「13mm 以上」。途中で 5〜13mm を通るのは不可
  - 構想設計書 16 章（docs/safety_limits.md §2）: 8mm 以下か 25mm 以上 → 下は 8mm だと ASTM より緩い（5〜8mm に子どもの指が入る）
  → **5mm 未満か 25mm 以上**。挟み込みは「すき間が閉じていく」ときに起きるので、全ての角度で満たすこと

製造のばらつき（公差）と、関節の軸のずれ（偏心）・遊び（バックラッシュ）ですき間は ± 動く → 名目のすき間に ± を足して判定する。
小さい側は「こすれない」（> 0）も見る（こすれるなら公差で詰めすぎ）。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable

SOURCE = "KINEMATIC_SIM"


@dataclass(frozen=True)
class PinchRule:
    small_below_mm: float      # これ未満なら指が入らない（ASTM F963 4.18.1 の 5mm の棒）
    large_from_mm: float       # これ以上なら指が挟まらない（構想設計書の 25mm。ASTM の 13mm より厳しい）
    tolerance_mm: float        # 製造のばらつき（±）
    eccentricity_mm: float     # 関節の軸のずれ・遊びで、すき間が片側に寄る量（±）

    @staticmethod
    def from_cfg(cfg: dict[str, Any]) -> "PinchRule":
        p = cfg["safety_limits"]["pinch"]
        return PinchRule(float(p["small_below_mm"]), float(p["large_from_mm"]), float(p["tolerance_mm"]),
                         float(p["eccentricity_mm"]))


def classify(rows: Iterable[dict[str, float]], rule: PinchRule, key: str = "gap_mm") -> dict[str, Any]:
    """rows = [{angle_deg, gap_mm}]。全ての角度で、名目 ± (公差 + 偏心) が「小」か「大」の片側に収まれば合格。"""
    rows = list(rows)
    spread = rule.tolerance_mm + rule.eccentricity_mm
    lo = [float(r[key]) - spread for r in rows]
    hi = [float(r[key]) + spread for r in rows]
    all_small = all(h < rule.small_below_mm for h in hi)
    all_large = all(v >= rule.large_from_mm for v in lo)
    in_band = [r.get("angle_deg") for r, a, b in zip(rows, lo, hi)
               if not (b < rule.small_below_mm or a >= rule.large_from_mm)]
    rubs = [r.get("angle_deg") for r, a in zip(rows, lo) if a <= 0.0]
    verdict = "PASS_SMALL" if all_small else "PASS_LARGE" if all_large else "FAIL"
    margin_small = rule.small_below_mm - max(hi) if rows else float("nan")
    return {"verdict": verdict, "angles_in_band": in_band, "rubbing_angles": rubs,
            "min_nominal_mm": min(float(r[key]) for r in rows), "max_nominal_mm": max(float(r[key]) for r in rows),
            "margin_to_small_mm": round(margin_small, 2), "spread_mm": spread, "source": SOURCE}


def max_constant_gap_mm(rule: PinchRule) -> float:
    """一定のすき間（同心の関節）で許される名目の最大: small_below − (公差 + 偏心)。これを超えると 5mm の棒が入りうる。"""
    return rule.small_below_mm - rule.tolerance_mm - rule.eccentricity_mm


def min_constant_gap_mm(rule: PinchRule) -> float:
    """こすれない名目の最小: 公差 + 偏心（これ以下だと最悪の組み合わせで当たる）。"""
    return rule.tolerance_mm + rule.eccentricity_mm
