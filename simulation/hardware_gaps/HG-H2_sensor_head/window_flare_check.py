"""R-032: Engineering の劣化器に窓の項（T・f）を入れたので、Design の掃引（`h2_window_flare.py`、別の描画器・半分の解像度）の境界の付近の点で突き合わせる。
Design の巡回 70% の境界（補間）: T = 1.0 で f ≈ 0.10、T = 0.7 で ≈ 0.054、T = 0.4 で ≈ 0.015。**描画器が違うので数字は一致しない前提**（向きと大きさの確認）。
SYNTHETIC_VISION_SIM。窓 A / B / C の実 T・f は未測定。
"""
from __future__ import annotations

import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run as R  # noqa: E402

PTS = ((1.0, 0.0), (1.0, 0.05), (1.0, 0.10), (1.0, 0.20), (0.7, 0.0), (0.7, 0.054), (0.7, 0.10), (0.4, 0.0), (0.4, 0.015), (0.4, 0.05))
OUT = R.HERE / "results" / "window_flare_check_2026-10-05.json"


def main() -> None:
    nom = dict(R.A["nominal"])
    conds = [(f"T={t:g},f={f:g}", {**nom, "window_T": t, "window_flare": f}) for t, f in PTS]
    with ProcessPoolExecutor(max_workers=10) as ex:
        rows = list(ex.map(R.evaluate_condition, conds))
    out = [{"tag": tag, "recall_patrol": r["recall_patrol"], "recall_inspect_online": r["recall_inspect_online"], "false_alarm_patrol": r["false_alarm_patrol"], "ok": bool(R.ok(r)), "missed": r["missed"]}
           for (tag, _), r in zip(conds, rows)]
    for o in out:
        print(o)
    OUT.write_text(json.dumps(out, indent=1, default=float), encoding="utf-8")


if __name__ == "__main__":
    main()
