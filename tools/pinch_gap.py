r"""関節のすき間（角度ごと）を挟み込みの基準で判定して表にする。**KINEMATIC_SIM / CAD_CONCEPT の値の判定（実物ではない）。**

    .\.venv\Scripts\python.exe tools\pinch_gap.py <joints.json> [--out simulation/results/pinch_gap.md]

joints.json: Design の `docs/design/results/concept_metrics_*.json` の "joints"（{名前: {rows: [{yaw_deg, min_gap_mm, inner_side_gap_mm}]}}）
か、{名前: {rows: [{angle_deg, gap_mm}]}}。基準は config `safety_limits.pinch`。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from serpens.config import load_config  # noqa: E402
from simulation.pinch_gap import SOURCE, PinchRule, classify, max_constant_gap_mm, min_constant_gap_mm  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("joints", type=Path)
    ap.add_argument("--out", type=Path, default=ROOT / "simulation" / "results" / "pinch_gap.md")
    a = ap.parse_args()
    rule = PinchRule.from_cfg(load_config())
    data = json.loads(a.joints.read_text(encoding="utf-8"))
    joints = data.get("joints", data)
    lines = [f"# 関節の挟み込み（{SOURCE}。入力 `{a.joints.name}`）", "",
             f"基準: 全ての角度で **{rule.small_below_mm:g}mm 未満**か **{rule.large_from_mm:g}mm 以上**（ASTM F963-11 4.18.1 と構想設計書 16 章の厳しい側）。"
             f"名目 ±（公差 {rule.tolerance_mm:g} + 偏心 {rule.eccentricity_mm:g}）mm で判定。",
             f"一定のすき間（同心の関節）なら名目 **{min_constant_gap_mm(rule):.1f}〜{max_constant_gap_mm(rule):.1f}mm**（下はこすれ、上は 5mm の棒が入りうる）。", "",
             "| 関節 | すき間 | 判定 | 名目の範囲 mm | 5mm までの余裕 mm | 帯に入る角度 |", "|---|---|---|---|---|---|"]
    for name, j in joints.items():
        rows = j["rows"]
        keys = [k for k in ("gap_mm", "min_gap_mm", "inner_side_gap_mm") if k in rows[0]]
        for k in keys:
            rr = [{"angle_deg": r.get("angle_deg", r.get("yaw_deg")), "gap_mm": r[k]} for r in rows]
            c = classify(rr, rule)
            lines.append(f"| {name} | {k} | **{c['verdict']}** | {c['min_nominal_mm']:g}〜{c['max_nominal_mm']:g} | {c['margin_to_small_mm']:g} | "
                         f"{c['angles_in_band'] or '—'} |")
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(a.out.read_text(encoding="utf-8"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
