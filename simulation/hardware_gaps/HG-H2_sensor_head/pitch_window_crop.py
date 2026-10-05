"""R-037（Design）: カメラの傾きを窓の足りなさとどう両立するか。Design の第 2 版（head_board_fit_2026-10-02）では、25° に傾けると窓の下端が
4.8 mm 高く、視野の下側 約 11.5° を失う。**下側の視野を切った**（窓で隠れた）画像で、傾き 15 / 20 / 25 / 30° の検出率を見る。
切り方 = 画像の下側（床の近い側）を、視野の下端から crop° ぶん黒く塗る（保守側。実際の窓のけられ方ではない）。SYNTHETIC_VISION_SIM。実画像ではない。
    python pitch_window_crop.py [--workers 13]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run as R  # noqa: E402


class WindowDegrader(R.Degrader):
    def __call__(self, img, rng):
        out = super().__call__(img, rng)
        crop = float(self.p.get("window_crop_deg", 0.0))
        if crop > 0:
            cam = self.cam
            a_bottom = math.atan((cam.height_px - cam.cy) / cam.f_px)
            y_cut = int(cam.cy + cam.f_px * math.tan(max(a_bottom - math.radians(crop), 0.0)))
            out = out.copy()
            out[y_cut:, :] = 0
        return out


R.Degrader = WindowDegrader
OUT = R.HERE / "results" / "pitch_window_crop_2026-10-05.json"
PITCH = (15.0, 20.0, 25.0, 30.0)
CROP = (0.0, 6.0, 11.5)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=13)
    a = ap.parse_args()
    nom = dict(R.A["nominal"])
    conds = [(f"pitch={p:g},crop={c:g}", {**nom, "cam_pitch_deg": p, "window_crop_deg": c}) for p in PITCH for c in CROP]
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        rows = list(ex.map(R.evaluate_condition, conds))
    out = []
    for (tag, _), r in zip(conds, rows):
        out.append({"tag": tag, "ok": bool(R.ok(r)), "recall_patrol": r["recall_patrol"], "recall_inspect_online": r["recall_inspect_online"],
                    "false_alarm_patrol": r["false_alarm_patrol"], "far_phantom_inspect": r["far_phantom_inspect"], "missed": r["missed"]})
        print(out[-1])
    OUT.write_text(json.dumps(out, indent=1, default=float), encoding="utf-8")


if __name__ == "__main__":
    main()
