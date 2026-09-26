"""評価用データセットの仕組み: 1 地点 = 5 枚 + 正解ラベル。合成でも実写でも同じ形。**基準床は使わない。**

  <root>/<sample_id>/{normal,raking,line,dark,normal2}.png     撮影順 通常→斜め→線光→全消灯→通常
  <root>/labels.csv    sample_id, floor, lighting, kind, diameter_mm, height_mm, x_mm, y_mm, specular, source, notes
source: SYNTHETIC / PHONE / HEAD_CAMERA（どれで撮ったか。混ぜない）。

評価は 2 段（レビュー 2026-09-26）:
  patrol  … 巡回中の発見。線なし（通常 + 斜めだけ）。見つかれば止まって inspect へ
  inspect … 止まって線光あり。線から外れた物の見逃しは「狙いの問題」として別に数える（線の性能ではない）
"""
from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from serpens.floorwatch.detect import Candidate, MotionError, detect
from serpens.floorwatch.geometry import Camera, LightPlane
from serpens.floorwatch.synthetic import Disc, Lighting, Renderer, Scene, Seam, Stain

FRAMES = ("normal", "raking", "line", "dark", "normal2")
LABEL_COLUMNS = ("sample_id", "floor", "lighting", "kind", "diameter_mm", "height_mm", "x_mm", "y_mm", "specular",
                 "source", "notes")
NEGATIVE_KINDS = ("stain_or_pattern", "seam", "none")

# 合成データセットの品目（レビューの指定 + 追加）: kind, 直径, 高さ, 反射率, 鏡面
SYNTHETIC_ITEMS = {
    "coin_1yen": ("coin", 20.0, 1.5, 0.85, False),
    "washer_m10": ("washer", 20.0, 1.5, 0.9, True),        # M10 ステンレス座金（CR2032 相当の鏡面）
    "washer_m6_x3": ("washer", 12.0, 4.8, 0.9, True),      # M6 座金 3 枚重ね（LR44 相当）
    "bead": ("bead", 6.0, 6.0, 0.7, False),
    "crumb": ("food_crumb", 4.0, 2.0, 0.45, False),
    "black_object": ("unknown", 15.0, 3.0, 0.08, False),   # 黒い物
    "clear_object": ("unknown", 15.0, 3.0, 0.5, True),     # 透明な物（反射率は床と同じ、線は乱れる）
}
SYNTHETIC_NEGATIVES = {"stain": 0.3, "pattern": 0.62, "empty": None}
# 床の継ぎ目（線状。誤報の主犯候補）: along, kind, 高さ/深さ mm
SYNTHETIC_SEAMS = [("y", "groove", 0.3), ("y", "step", 0.2), ("y", "step", 0.5), ("x", "groove", 0.5), ("x", "step", 0.3)]
# 位置: 線の上 3 / 線から外れ 3（x のずれ mm, y mm）
POSITIONS_ON_LINE = [(0.0, 64.0), (0.0, 80.0), (0.0, 55.0)]
POSITIONS_OFF_LINE = [(-8.0, 80.0), (6.0, 55.0), (-12.0, 64.0)]


def imread_gray(path: Path) -> np.ndarray:
    """日本語を含むパスでも読める（cv2.imread は Windows の非 ASCII パスで None を返す）。"""
    data = np.fromfile(str(path), dtype=np.uint8)
    img = cv2.imdecode(data, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(path)
    return img


def imwrite(path: Path, img: np.ndarray) -> None:
    ok, buf = cv2.imencode(path.suffix, img)
    if not ok:
        raise ValueError(f"書けない: {path}")
    buf.tofile(str(path))


@dataclass
class Sample:
    sample_id: str
    frames: dict[str, np.ndarray]
    label: dict[str, Any]


def write_sample(root: Path, sample_id: str, frames: dict[str, np.ndarray], label: dict[str, Any]) -> None:
    d = root / sample_id
    d.mkdir(parents=True, exist_ok=True)
    for k in FRAMES:
        if k in frames:
            imwrite(d / f"{k}.png", frames[k])
    path = root / "labels.csv"
    new = not path.exists()
    with path.open("a", newline="", encoding="utf-8") as fp:
        w = csv.DictWriter(fp, fieldnames=list(LABEL_COLUMNS))
        if new:
            w.writeheader()
        w.writerow({k: label.get(k, "") for k in LABEL_COLUMNS})


def read_samples(root: Path) -> list[Sample]:
    out = []
    with (root / "labels.csv").open(encoding="utf-8", newline="") as fp:
        for row in csv.DictReader(fp):
            d = root / row["sample_id"]
            frames = {k: imread_gray(d / f"{k}.png") for k in FRAMES if (d / f"{k}.png").exists()}
            out.append(Sample(row["sample_id"], frames, row))
    return out


def _label(sid: str, floor: str, kind: str, dia: float, h: float, x: float, y: float, spec: bool, notes: str) -> dict:
    return dict(sample_id=sid, floor=floor, lighting="synthetic", kind=kind, diameter_mm=dia, height_mm=h, x_mm=x, y_mm=y,
                specular=int(spec), source="SYNTHETIC", notes=notes)


def make_synthetic(root: Path, cam: Camera, plane: LightPlane, lighting: Lighting, seed: int = 0,
                   floor: str = "synthetic_wood") -> Path:
    """品目 × 位置（線上 3 + 線外 3）と、陰性（汚れ・模様・空・継ぎ目）を描く。基準床は書かない。"""
    r = Renderer(cam, plane, lighting)
    root.mkdir(parents=True, exist_ok=True)
    if (root / "labels.csv").exists():
        (root / "labels.csv").unlink()
    n = 0
    for name, (kind, dia, h, alb, spec) in SYNTHETIC_ITEMS.items():
        for x, y in POSITIONS_ON_LINE + POSITIONS_OFF_LINE:
            sid = f"{name}_{n:03d}"
            write_sample(root, sid, r.render(Scene([Disc(x, y, dia, h, alb, spec, kind)], seed=seed + n)),
                         _label(sid, floor, kind, dia, h, x, y, spec, name))
            n += 1
    for name, alb in SYNTHETIC_NEGATIVES.items():
        for x, y in POSITIONS_ON_LINE + POSITIONS_OFF_LINE:
            sid = f"{name}_{n:03d}"
            scene = Scene(stains=[Stain(x, y, 18.0, alb)] if alb is not None else [], seed=seed + n)
            write_sample(root, sid, r.render(scene),
                         _label(sid, floor, "stain_or_pattern" if alb is not None else "none",
                                18.0 if alb is not None else 0.0, 0.0, x, y, False, name))
            n += 1
    for along, kind, h in SYNTHETIC_SEAMS:
        for pos in (0.0, 8.0, 64.0, 75.0):
            if (along == "y") == (pos > 30.0):                     # y 方向の継ぎ目は x 位置、x 方向は y 位置
                continue
            sid = f"seam_{along}_{kind}_{n:03d}"
            write_sample(root, sid, r.render(Scene(seams=[Seam(along, pos, h, kind)], seed=seed + n)),
                         _label(sid, floor, "seam", 0.0, h, pos if along == "y" else 0.0, pos if along == "x" else 64.0,
                                False, f"seam {along} {kind} {h}mm"))
            n += 1
    return root


def _run(frames: dict[str, np.ndarray], cam: Camera, plane: LightPlane, cfg: dict[str, Any],
         with_line: bool) -> list[Candidate]:
    try:
        cands, _tr, _fg = detect(frames, cam, plane, cfg, with_line=with_line)
    except MotionError:
        return []
    return [c for c in cands if c.is_object]


def evaluate(root: Path, cam: Camera, plane: LightPlane, cfg: dict[str, Any]) -> dict[str, Any]:
    """2 段の検出率・誤報率。inspect では線外の見逃しを aiming_miss として分ける。"""
    samples = read_samples(root)
    stage: dict[str, dict[str, int]] = {"patrol": {"tp": 0, "fn": 0, "fp": 0, "neg": 0},
                                        "inspect": {"tp": 0, "fn": 0, "fp": 0, "neg": 0, "aiming_miss": 0}}
    rows = []
    for s in samples:
        positive = s.label["kind"] not in NEGATIVE_KINDS
        x, y = float(s.label["x_mm"]), float(s.label["y_mm"])
        row: dict[str, Any] = {"sample_id": s.sample_id, "kind": s.label["kind"], "positive": positive,
                               "on_line": abs(x) < 1e-6, "label_d": float(s.label["diameter_mm"]),
                               "label_h": float(s.label["height_mm"])}
        for st, with_line in (("patrol", False), ("inspect", True)):
            objs = _run(s.frames, cam, plane, cfg, with_line)
            hit = _nearest(objs, x, y) if positive else None
            c = stage[st]
            if positive:
                if hit is not None:
                    c["tp"] += 1
                elif st == "inspect" and not row["on_line"]:
                    c["aiming_miss"] += 1                          # 線が当たっていない: 狙いの問題として別枠
                else:
                    c["fn"] += 1
            else:
                c["neg"] += 1
                c["fp"] += bool(objs)
            row[f"{st}_detected"], row[f"{st}_hit"] = bool(objs), hit is not None
            if st == "inspect":
                row.update({"diameter_est_mm": None if hit is None else round(hit.diameter_mm, 1),
                            "height_est_mm": None if hit is None or hit.height_mm is None else round(hit.height_mm, 2),
                            "height_reason": None if hit is None else hit.height_reason,
                            "dropout": None if hit is None else hit.line_dropout,
                            "shadow": None if hit is None else hit.shadow,
                            "metal_disc": hit is not None and any(k["kind"] == "metal_disc" for k in hit.kinds)})
        rows.append(row)
    out: dict[str, Any] = {"source": "SYNTHETIC" if all(s.label["source"] == "SYNTHETIC" for s in samples) else "MIXED",
                           "n": len(samples), "rows": rows, "stages": {}}
    for st, c in stage.items():
        out["stages"][st] = {**c, "detection_rate": c["tp"] / max(c["tp"] + c["fn"], 1),
                             "false_alarm_rate": c["fp"] / max(c["neg"], 1),
                             "detection_ci95": wilson(c["tp"], c["tp"] + c["fn"]), "false_alarm_ci95": wilson(c["fp"], c["neg"])}
    return out


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    """二項比率の 95% 信頼区間（Wilson）。n が小さいときの楽観を数字で示す。"""
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    den = 1 + z * z / n
    mid = (p + z * z / (2 * n)) / den
    half = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (max(0.0, float(mid - half)), min(1.0, float(mid + half)))


def _nearest(objs: list[Candidate], x_mm: float, y_mm: float, tol_mm: float = 25.0) -> Candidate | None:
    best, bd = None, tol_mm
    for c in objs:
        fx, fy = c.floor_xy_mm
        d = float(np.hypot(fx - x_mm, fy - y_mm))
        if d < bd:
            best, bd = c, d
    return best
