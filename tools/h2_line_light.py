"""H2: ライン光の置き方（頬 / 眉）の三角測量を比べて表にする。**GEOMETRY_SIM（実写・実測ではない）。**

    .\\.venv\\Scripts\\python.exe tools\\h2_line_light.py
    → simulation/results/h2_line_light.{csv,md}
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from serpens.config import load_config  # noqa: E402
from serpens.floorwatch.geometry import Camera  # noqa: E402
from simulation.h2_line_light import SOURCE, evaluate, placements  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=ROOT / "simulation" / "results")
    ap.add_argument("--mode", default=None, help="floor_watch.camera.modes のキー（省略時 floor_mode）")
    args = ap.parse_args()
    cfg = load_config()
    cam = Camera.from_cfg(cfg, args.mode)
    rows = [evaluate(cam, p) for p in placements(cfg)]
    rows = [{k: (float(v) if hasattr(v, "item") else v) for k, v in r.items()} for r in rows]
    args.out.mkdir(parents=True, exist_ok=True)
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with (args.out / "h2_line_light.csv").open("w", newline="", encoding="utf-8") as fp:
        w = csv.DictWriter(fp, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    lines = [f"# H2 ライン光の置き方（{SOURCE}、カメラ {cam.width_px}×{cam.height_px} f={cam.f_px:.0f}px・高さ {cam.height_mm:.0f}mm・"
             f"pitch {cam.pitch_deg:.0f}°、すべて ASSUMED）", "",
             "| 置き方 | 狙いの手段 | 線の長さ mm | 感度 px/mm（最小/中央） | σ_H mm（σc 0.2px） | yaw 0.5° 誤較正の誤差 mm（1.5/5mm） "
             "| tilt 0.5°（1.5/5mm） | 線の移動 mm/°（pivot 40/60/80/104） |", "|---|---|---|---|---|---|---|---|"]
    for r in rows:
        lines.append(f"| {r['name']} | {r['aim_by']} | {r['line_length_mm']} | {r['sens_px_per_mm_min']} / {r['sens_px_per_mm_median']} "
                     f"| {r['sigma_h_mm_worst']} | {r['calib_yaw0.5deg_err_mm_h1.5']} / {r['calib_yaw0.5deg_err_mm_h5.0']} "
                     f"| {r['calib_tilt0.5deg_err_mm_h1.5']} / {r['calib_tilt0.5deg_err_mm_h5.0']} "
                     f"| {r['aim_mm_per_deg_pivot40']} / {r['aim_mm_per_deg_pivot60']} / {r['aim_mm_per_deg_pivot80']} / "
                     f"{r['aim_mm_per_deg_pivot104']} |")
    lines += ["", "- head_yaw: 線は前後に走る。候補へ当てるには頭 yaw で左右に振る（6 本目 = Head Yaw、または胴の旋回）",
              "- neck_pitch: 線は左右に走る（視野の横幅いっぱい）。前後を合わせるには J1 首 pitch（既存）か胴の微動",
              "- pivot: 首の回転軸からカメラまでの距離（ASSUMPTION。CAD で要確認）",
              "- 言えないこと: 線の太さ・ボケ・床での散乱、鏡面での途切れ、物自身の影、鼻先による光路のさえぎり"]
    (args.out / "h2_line_light.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print((args.out / "h2_line_light.md").read_text(encoding="utf-8"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
