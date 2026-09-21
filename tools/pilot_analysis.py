r"""Pilot 回答（pilot/response_template.csv の形式）の集計。高度な統計はしない。

    .\.venv\Scripts\python.exe tools\pilot_analysis.py output\pilot\responses.csv --key output\pilot\clip_key.json

出す物: クリップごとの median / mean / 分布、被験者内差（クリップの対ごと）、Fear が上がっていないか、
Snake Comfort との順位相関、Animacy と Affection の同時上昇。n=8〜12 なので小さな差を結論にしない。
結果の source は VIDEO_HUMAN_EVALUATION（大きさ・音・接触・距離感は評価できていない）。
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import defaultdict
from itertools import combinations
from pathlib import Path

import numpy as np

SCALES = ("snake_likeness", "animacy", "approachability", "affection", "fear", "smoothness")
SOURCE = "VIDEO_HUMAN_EVALUATION"


def read(path: Path) -> list[dict]:
    with path.open(encoding="utf-8", newline="") as fp:
        return [r for r in csv.DictReader(fp) if r.get("clip_id")]


def spearman(x: list[float], y: list[float]) -> float:
    """順位相関（同順位は平均順位）。n < 4 なら nan。"""
    if len(x) < 4:
        return float("nan")
    rx, ry = _ranks(x), _ranks(y)
    if np.std(rx) == 0 or np.std(ry) == 0:
        return float("nan")
    return float(np.corrcoef(rx, ry)[0, 1])


def _ranks(v: list[float]) -> np.ndarray:
    a = np.asarray(v, float)
    order = a.argsort()
    ranks = np.empty(len(a))
    ranks[order] = np.arange(1, len(a) + 1)
    for val in np.unique(a):                       # 同順位は平均
        idx = np.where(a == val)[0]
        ranks[idx] = ranks[idx].mean()
    return ranks


def summarize(rows: list[dict], key: dict | None) -> list[str]:
    out = [f"# Pilot 集計（source = {SOURCE}、n = {len({r['participant_anonymous_id'] for r in rows})}）", ""]
    by_clip: dict[str, dict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
    for r in rows:
        for s in SCALES:
            if r.get(s, "") != "":
                by_clip[r["clip_id"]][s].append(float(r[s]))
    out.append("## クリップごと（median / mean / sd / 分布 1..7）")
    out.append("| clip | 条件（key があれば） | 尺度 | median | mean | sd | 分布 |")
    out.append("|---|---|---|---|---|---|---|")
    for clip in sorted(by_clip):
        cond = key["clips"][clip]["condition"] if key and clip in key.get("clips", {}) else "（blinded）"
        for s in SCALES:
            v = by_clip[clip][s]
            if not v:
                continue
            hist = " ".join(str(int(np.sum(np.array(v) == k))) for k in range(1, 8))
            out.append(f"| {clip} | {cond} | {s} | {np.median(v):.1f} | {np.mean(v):.2f} | {np.std(v):.2f} | {hist} |")
    out += ["", "## 被験者内差（同じ人の 2 クリップの差の median。正 = 後者が高い）"]
    per = defaultdict(dict)
    for r in rows:
        per[r["participant_anonymous_id"]][r["clip_id"]] = r
    for a, b in combinations(sorted(by_clip), 2):
        diffs = {s: [float(p[b][s]) - float(p[a][s]) for p in per.values() if a in p and b in p and p[a].get(s) and p[b].get(s)]
                 for s in SCALES}
        cells = " / ".join(f"{s} {np.median(d):+.1f}" for s, d in diffs.items() if d)
        out.append(f"- {a} → {b}: {cells}")
    out += ["", "## 特に見るもの"]
    fear = {c: np.mean(v["fear"]) for c, v in by_clip.items() if v["fear"]}
    if fear:
        worst = max(fear, key=fear.get)
        out.append(f"- Fear が最も高いクリップ: {worst}（mean {fear[worst]:.2f}）。4 以上なら Fear 対策を先に")
    comfort = [float(r["snake_comfort"]) for r in rows if r.get("snake_comfort") and r.get("fear")]
    fears = [float(r["fear"]) for r in rows if r.get("snake_comfort") and r.get("fear")]
    out.append(f"- Snake Comfort と Fear の順位相関: {spearman(comfort, fears):+.2f}（苦手な人ほど怖いか）")
    an = [float(r["animacy"]) for r in rows if r.get("animacy") and r.get("affection")]
    af = [float(r["affection"]) for r in rows if r.get("animacy") and r.get("affection")]
    out.append(f"- Animacy と Affection の順位相関: {spearman(an, af):+.2f}（生きて見えると愛着も上がるか）")
    out.append("- 6→8、8→10 の差は被験者内差の median で見る。n が小さいので「傾向」以上に言わない")
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("responses", type=Path)
    ap.add_argument("--key", type=Path, default=None, help="clip_key.json（blind を解くとき）")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    key = json.loads(args.key.read_text(encoding="utf-8")) if args.key else None
    lines = summarize(read(args.responses), key)
    text = "\n".join(lines)
    if args.out:
        args.out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
