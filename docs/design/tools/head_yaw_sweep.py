"""Head Yaw ±15/30/45/60° の Design 側の比較（USER-DEC-SERPENS-0004、KINEMATIC_SIM / DESIGN_ESTIMATE）。

上から見た 2D。胴の曲げは Body Curvature Budget 145° の中（胴の yaw 合計の絶対値 ≤ 145）。
Head Yaw は首の先の独立した軸（`robot_fw6_headyaw.yaml` の CONCEPT: J1 の 50 mm 前）。
頭は SD-01 の Bean を幅 100・長さ 74 の角丸の輪郭（上から）で近似、胴は幅 92 の帯（首は 68、上の首は 52）。

調べること:
1. CSAR: 発見の構え（一直線、物は頭の前 90 mm）から、胴の前の方（J2）だけ ±50° と Head Yaw で、視線を物から何度そらせるか。
   子どもがいそうな横〜斜め後ろ（物から 90°〜150°）を見るのに何度要るか
2. 眠りの姿勢（胴 [50,45,30,20] = 145°）に Head Yaw を足したとき、頭が自分の体を見るか（視線と体の最も近い点の方向の角）
3. 頭と胴の開き: 眠りの姿勢で、頭の輪郭と胴の帯の最小のすき間（mm）。子どもの手首・首が入って閉じ込められうる開きの大きさの比較用
   **判定はしない。** 子どもの首径・手首径の分布（一次資料から Engineering がモデル化）と比べるための数値だけを出す

実行:  .venv/Scripts/python.exe docs/design/tools/head_yaw_sweep.py
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import concept_geometry as cg  # noqa: E402

YAWS = (0, 15, 30, 45, 60)
BODY_SLEEP = [50, 45, 30, 20]
HEAD_L, HEAD_W = 74.0, 100.0
UPPER_NECK = 50.0          # Head Yaw 軸から J1 まで（CONCEPT）
BODY_W = 92.0


def rot(v, deg):
    t = math.radians(deg); c, s = math.cos(t), math.sin(t)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def chain(body_yaws, head_yaw):
    """尾端から頭へ向かって並べる。戻り値: 胴の中心線の点（尾 → Head Yaw 軸）、頭の向き（単位）、Head Yaw 軸の位置。"""
    segs = [145.0, 95.0, 95.0, 95.0, 87.0 + UPPER_NECK]   # 尾 / L3 / L2 / L1 / 首＋上の首（J2 → Head Yaw 軸）
    pts = [np.array([0.0, 0.0])]
    d = np.array([-1.0, 0.0])          # 尾から頭へは −x
    turns = list(reversed(body_yaws))   # J5, J4, J3, J2
    for i, L in enumerate(segs):
        if i >= 1:
            d = rot(d, -turns[i - 1])
        pts.append(pts[-1] + d * L)
    head_dir = rot(d, -head_yaw)
    return np.array(pts), head_dir


def head_outline(axis_pt, head_dir, n=60):
    """Head Yaw 軸から前へ、角丸の長方形（幅 100、長さ 74、角の半径 30）。"""
    side = np.array([-head_dir[1], head_dir[0]])
    pts = []
    for t in np.linspace(0, 1, n):
        # 周に沿って: 簡単のため楕円＋長方形の混合（超楕円 n=3）
        a = 2 * math.pi * t
        c, s = math.cos(a), math.sin(a)
        x = (HEAD_L / 2) * math.copysign(abs(c) ** (2 / 3), c)
        y = (HEAD_W / 2) * math.copysign(abs(s) ** (2 / 3), s)
        pts.append(axis_pt + head_dir * (HEAD_L / 2 + 8) + head_dir * x + side * y)
    return np.array(pts)


def body_band_distance(p, centerline, half_w):
    """点 p から胴の帯（中心線 ± half_w）の表面までの距離（中なら負）。"""
    best = 1e9
    for a, b in zip(centerline[:-1], centerline[1:]):
        ab = b - a; t = max(0.0, min(1.0, float((p - a) @ ab / (ab @ ab))))
        best = min(best, float(np.linalg.norm(p - (a + t * ab))))
    return best - half_w


def csar_rows():
    rows = []
    for hy in YAWS:
        # J2 だけ動かす（胴の後ろは止めたまま）。視線を物（前方 0°）から最大どれだけ外せるか
        away = 50 + hy
        rows.append({"head_yaw_max": hy, "gaze_away_from_object_deg": away,
                     "reaches_90": away >= 90, "reaches_120": away >= 120, "reaches_150": away >= 150,
                     "body_moves_near_object": "J2 だけ（首と L1 の前が振れる）" if hy < 40 else "J2 を減らせる"})
    return rows


def sleep_rows():
    rows = []
    for hy in YAWS:
        cl, hd = chain(BODY_SLEEP, hy)
        axis = cl[-1]
        # 視線が自分の体（胴の帯の中心線上の最も近い点）の方向とどれだけずれているか
        best = 180.0
        for q in np.linspace(0, 1, 200):
            idx = min(int(q * (len(cl) - 1)), len(cl) - 2); t = q * (len(cl) - 1) - idx
            p = cl[idx] + (cl[idx + 1] - cl[idx]) * t
            v = p - axis
            if np.linalg.norm(v) < 150:     # 首のすぐ後ろは見ても「体を見る」にならない
                continue
            ang = math.degrees(math.acos(float(np.clip(hd @ v / np.linalg.norm(v), -1, 1))))
            best = min(best, ang)
        outline = head_outline(axis, hd)
        body_only = cl[:-1]                 # 首＋上の首は頭とつながっているので除く
        gap = min(body_band_distance(p, body_only, BODY_W / 2) for p in outline)
        rows.append({"head_yaw": hy, "gaze_to_own_body_deg": round(best), "head_to_body_min_gap_mm": round(gap, 1),
                     "head_hits_body": gap < 0})
    return rows


def main():
    out = {"source": "KINEMATIC_SIM / DESIGN_ESTIMATE（上から 2D、判定はしない）", "csar": csar_rows(), "sleep_145": sleep_rows()}
    (cg.RESULTS / "head_yaw_sweep_2026-09-29.json").write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
    for k in ("csar", "sleep_145"):
        for r in out[k]:
            print(k, r)


if __name__ == "__main__":
    main()
