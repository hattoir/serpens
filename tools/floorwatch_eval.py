r"""Floor Watch の評価: データセット（合成 or 実写）で検出率・誤報率・高さ/大きさの誤差を出す。

    .\.venv\Scripts\python.exe tools\floorwatch_eval.py --synthetic            # 合成データを作って評価（SIMULATED）
    .\.venv\Scripts\python.exe tools\floorwatch_eval.py --root data\floorwatch\phone_2026-10   # 実写（labels.csv 必須）

結果は output/floorwatch_eval.md。source（SYNTHETIC / PHONE / HEAD_CAMERA）を必ず載せる。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from serpens.config import load_config  # noqa: E402
from serpens.floorwatch.dataset import evaluate, make_synthetic  # noqa: E402
from serpens.floorwatch.geometry import Camera, LightPlane  # noqa: E402
from serpens.floorwatch.synthetic import default_lighting  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, default=Path("output/floorwatch_synth"))
    ap.add_argument("--synthetic", action="store_true", help="合成データセットを作り直してから評価")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=Path("output/floorwatch_eval.md"))
    args = ap.parse_args()
    cfg = load_config()
    cam, plane = Camera.from_cfg(cfg), LightPlane.design(cfg)
    if args.synthetic:
        make_synthetic(args.root, cam, plane, default_lighting(cfg), seed=args.seed)
    res = evaluate(args.root, cam, plane, cfg)
    lines = [f"# Floor Watch 評価（source = {res['source']}、n = {res['n']}）", "",
             "光の面は DESIGN 値（較正なし）。実写では 1 円玉の段で較正してから。", "",
             f"- 検出率: **{res['detection_rate']:.0%}**（{res['tp']}/{res['tp'] + res['fn']}）",
             f"- 誤報率: **{res['false_alarm_rate']:.0%}**（{res['fp']}/{res['negatives']}）", "",
             "| sample | kind | 正解 | 検出 | 一致 | 直径推定 mm（正解） | 高さ推定 mm（正解） | 高さ測定 | 線の途切れ | 影 |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for r in res["rows"]:
        lines.append(f"| {r['sample_id']} | {r['kind']} | {'物' if r['positive'] else '陰性'} | {r['detected']} | {r['hit']} | "
                     f"{r['diameter_est_mm']}（{r['label_d']:.0f}） | {r['height_est_mm']}（{r['label_h']:.1f}） | "
                     f"{r['height_measured']} | {r['dropout']} | {r['shadow']} |")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines[:6]))
    print(f"→ {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
