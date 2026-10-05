"""R-037（Design）: カメラの下向き角（cam_pitch_deg）の判断境界を細かく見る。nominal（UXGA・高さ 30 mm・65° FOV）から pitch だけを振る。
SYNTHETIC_VISION_SIM（合成画像）。実画像の性能ではない。HARDWARE_VERIFIED = 0。
    python pitch_boundary.py [--workers 4]
"""
from __future__ import annotations

import argparse
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run as R  # noqa: E402

PITCH = (3.0, 5.0, 8.0, 10.0, 12.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0)
OUT = R.HERE / "results" / "pitch_boundary_2026-10-05.json"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=4)
    a = ap.parse_args()
    nom = dict(R.A["nominal"])
    conds = [(f"cam_pitch_deg={p:g}", {**nom, "cam_pitch_deg": p}) for p in PITCH]
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        rows = list(ex.map(R.evaluate_condition, conds))
    out = []
    for p, r in zip(PITCH, rows):
        out.append({"pitch_deg": p, "ok": bool(R.ok(r)), "recall_patrol": r["recall_patrol"], "recall_inspect_online": r["recall_inspect_online"],
                    "false_alarm_patrol": r["false_alarm_patrol"], "false_alarm_inspect": r["false_alarm_inspect"], "far_phantom_inspect": r["far_phantom_inspect"],
                    "missed": r["missed"], "mm_per_px_center": r["mm_per_px_center"]})
        print(out[-1])
    OUT.write_text(json.dumps(out, indent=1, default=float), encoding="utf-8")


if __name__ == "__main__":
    main()
