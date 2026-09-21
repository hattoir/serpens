r"""Human Perception Pilot 用の匿名化された比較クリップを作る（KINEMATIC_SIM の描画）。

    .\.venv\Scripts\python.exe tools\pilot_clips.py                       # pilot/conditions.json の条件を全部
    .\.venv\Scripts\python.exe tools\pilot_clips.py --seed 7 --duration 12 --out output\pilot

出力:
  output/pilot/clips/clip_A.avi, clip_B.avi, …   画面に条件名は出ない
  output/pilot/clip_key.json                      匿名 ID ↔ 条件（**被験者には見せない**）、乱数の種、描画条件

RAW と SPEED_MATCHED（simulation/body_compare.py の f）の両方を conditions.json に並べておく。
"""
from __future__ import annotations

import argparse
import json
import random
import string
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from simulation.body_compare import CONFIGS, evaluate, load, speed_matched_freq  # noqa: E402
from simulation.clips import SOURCE, ClipSpec, RenderSettings, write_clip  # noqa: E402

DEFAULT_CONDITIONS = Path("pilot/conditions.json")


def resolve(item: dict) -> ClipSpec:
    """conditions.json の 1 項目 → ClipSpec（狙い波数と RAW/SPEED_MATCHED を Ω・f に落とす）。"""
    if item.get("scene") == "arc":
        return ClipSpec(item["name"], item.get("overlay"), "arc", profile=item.get("profile"))
    cfg = load(item["config"])
    body = len([j for j in cfg["joints"] if j["axis"] == "yaw"]) - 1
    omega = float(item["target_waves"]) * 360.0 / body
    amp, f = float(item.get("amplitude_deg", 30.0)), float(item.get("temporal_freq_hz", 0.5))
    if item.get("condition") == "SPEED_MATCHED":
        ref = evaluate("yaw6", load("yaw6"), 1.0, amp, f, "RAW")
        raw = evaluate(item["config"], cfg, float(item["target_waves"]), amp, f, "RAW")
        f, _clipped = speed_matched_freq(cfg, raw, ref.forward_mm_s)
    return ClipSpec(item["name"], CONFIGS[item["config"]], "gait", amp, omega, f)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--conditions", type=Path, default=DEFAULT_CONDITIONS)
    ap.add_argument("--out", type=Path, default=Path("output/pilot"))
    ap.add_argument("--seed", type=int, default=1, help="匿名 ID の割り当ての種")
    ap.add_argument("--duration", type=float, default=12.0)
    ap.add_argument("--fps", type=float, default=10.0)
    args = ap.parse_args()
    items = json.loads(args.conditions.read_text(encoding="utf-8"))["conditions"]
    specs = [resolve(it) for it in items]
    rng = random.Random(args.seed)
    order = list(range(len(specs)))
    rng.shuffle(order)
    ids = [f"clip_{string.ascii_uppercase[i]}" for i in range(len(specs))]
    rs = RenderSettings(duration_s=args.duration, fps=args.fps)
    key = {"source": SOURCE, "blinded": True, "assignment_seed": args.seed,
           "render": rs.__dict__ | {"note": "同じカメラ・背景・時間・開始位置・ターゲット位置・表示スケール"},
           "clips": {}}
    for clip_id, idx in zip(ids, order):
        spec = specs[idx]
        path = args.out / "clips" / f"{clip_id}.avi"
        n = write_clip(spec, rs, path)
        key["clips"][clip_id] = {"condition": spec.condition, "overlay": spec.overlay, "scene": spec.scene,
                                 "amplitude_deg": spec.amplitude_deg, "spatial_freq_deg": round(spec.spatial_freq_deg, 2),
                                 "temporal_freq_hz": round(spec.temporal_freq_hz, 3), "frames": n, "file": str(path)}
        print(f"{clip_id}: {n} frames -> {path}")
    (args.out / "clip_key.json").write_text(json.dumps(key, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"key: {args.out / 'clip_key.json'}（被験者には見せない）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
