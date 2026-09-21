r"""被験者ごとにクリップの提示順をランダム化する（順番効果を減らす）。seed を保存する。

    .\.venv\Scripts\python.exe tools\pilot_order.py --participants 10 --seed 42
    → output/pilot/presentation_order.json
"""
from __future__ import annotations

import argparse
import json
import random
import sys
from pathlib import Path


def orders(clip_ids: list[str], participants: int, seed: int) -> list[dict]:
    out = []
    for i in range(participants):
        rng = random.Random(f"{seed}:{i}")
        order = list(clip_ids)
        rng.shuffle(order)
        out.append({"participant_anonymous_id": f"P{i + 1:02d}", "presentation_order": order})
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--key", type=Path, default=Path("output/pilot/clip_key.json"))
    ap.add_argument("--participants", type=int, default=10)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", type=Path, default=Path("output/pilot/presentation_order.json"))
    args = ap.parse_args()
    clip_ids = sorted(json.loads(args.key.read_text(encoding="utf-8"))["clips"])
    doc = {"random_seed": args.seed, "clip_ids": clip_ids, "participants": orders(clip_ids, args.participants, args.seed)}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(doc, ensure_ascii=False, indent=2), encoding="utf-8")
    for p in doc["participants"]:
        print(p["participant_anonymous_id"], " ".join(p["presentation_order"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
