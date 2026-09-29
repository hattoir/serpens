"""体が物を囲い込めるか（yaw の鎖の角度合計）。PC 側・機体側・サーボバスの 3 か所が同じ式を使う。"""
from __future__ import annotations

from collections.abc import Iterable


def max_contiguous_sum(values: Iterable[float]) -> float:
    """角度の列（尾 → 頭）のうち、連続する部分の和の絶対値の最大 = 体がその区間で囲む角 [deg]。

    同じ向きに曲がった区間の和が 180° を超えると、体が手首・首などを囲い込んで引っかかりうる
    （link.limits.yaw_sum_deg、PRODUCT.md §4「体が輪を作れない」）。
    """
    best = run_pos = run_neg = 0.0
    for x in values:
        run_pos = max(0.0, run_pos + x)
        run_neg = min(0.0, run_neg + x)
        best = max(best, run_pos, -run_neg)
    return best
