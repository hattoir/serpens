"""評価用データセットの仕組み: 1 地点 = 4 枚 + 正解ラベル。合成でも実写でも同じ形。

  <root>/<sample_id>/{dark,normal,raking,line}.png
  <root>/reference_<floor>.png                    いつもの床（通常照明、全消灯を引く前）
  <root>/labels.csv                               sample_id, floor, lighting, kind, diameter_mm, height_mm,
                                                  x_mm, y_mm, specular, source, notes
source: SYNTHETIC / PHONE / HEAD_CAMERA（どれで撮ったか。混ぜない）。
評価: 検出率（正解の物体を is_object の候補が拾ったか）と誤報率（物が無い・汚れだけの地点で is_object が出たか）。
"""
from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from serpens.floorwatch.detect import Candidate, detect
from serpens.floorwatch.geometry import Camera, LightPlane
from serpens.floorwatch.synthetic import Disc, Lighting, Renderer, Scene, Stain

FRAMES = ("dark", "normal", "raking", "line")


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
LABEL_COLUMNS = ("sample_id", "floor", "lighting", "kind", "diameter_mm", "height_mm", "x_mm", "y_mm", "specular",
                 "source", "notes")

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


@dataclass
class Sample:
    sample_id: str
    frames: dict[str, np.ndarray]
    label: dict[str, Any]


def write_sample(root: Path, sample_id: str, frames: dict[str, np.ndarray], label: dict[str, Any]) -> None:
    d = root / sample_id
    d.mkdir(parents=True, exist_ok=True)
    for k in FRAMES:
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
            frames = {k: imread_gray(d / f"{k}.png") for k in FRAMES}
            out.append(Sample(row["sample_id"], frames, row))
    return out


def make_synthetic(root: Path, cam: Camera, plane: LightPlane, lighting: Lighting, seed: int = 0,
                   floor: str = "synthetic_wood") -> Path:
    """品目 × 位置 と、陰性（汚れ・模様・何も無い）を描いてデータセットにする。基準床も書く。"""
    r = Renderer(cam, plane, lighting)
    root.mkdir(parents=True, exist_ok=True)
    ref = r.render(Scene(seed=seed))["normal"]
    imwrite(root / f"reference_{floor}.png", ref)
    if (root / "labels.csv").exists():
        (root / "labels.csv").unlink()
    positions = [(0.0, 64.0), (-8.0, 80.0), (6.0, 55.0)]
    n = 0
    for name, (kind, dia, h, alb, spec) in SYNTHETIC_ITEMS.items():
        for x, y in positions:
            fr = r.render(Scene([Disc(x, y, dia, h, alb, spec, kind)], seed=seed))
            write_sample(root, f"{name}_{n:03d}", fr, dict(sample_id=f"{name}_{n:03d}", floor=floor, lighting="synthetic",
                                                          kind=kind, diameter_mm=dia, height_mm=h, x_mm=x, y_mm=y,
                                                          specular=int(spec), source="SYNTHETIC", notes=name))
            n += 1
    for name, alb in SYNTHETIC_NEGATIVES.items():
        for x, y in positions:
            scene = Scene(stains=[Stain(x, y, 18.0, alb)] if alb is not None else [], seed=seed)
            fr = r.render(scene)
            write_sample(root, f"{name}_{n:03d}", fr, dict(sample_id=f"{name}_{n:03d}", floor=floor, lighting="synthetic",
                                                          kind="stain_or_pattern" if alb is not None else "none",
                                                          diameter_mm=18.0 if alb is not None else 0.0, height_mm=0.0,
                                                          x_mm=x, y_mm=y, specular=0, source="SYNTHETIC", notes=name))
            n += 1
    return root


def evaluate(root: Path, cam: Camera, plane: LightPlane, cfg: dict[str, Any]) -> dict[str, Any]:
    """検出率・誤報率・高さ/大きさの誤差。"""
    samples = read_samples(root)
    refs = {p.stem.replace("reference_", ""): imread_gray(p) for p in root.glob("reference_*.png")}
    rows, tp, fn, fp, neg = [], 0, 0, 0, 0
    for s in samples:
        ref = refs[s.label["floor"]]
        dark = np.float32(s.frames["dark"])
        cands, _tr, _fg = detect(s.frames, np.clip(np.float32(ref) - dark, 0, None), cam, plane, cfg)
        objs = [c for c in cands if c.is_object]
        positive = s.label["kind"] not in ("stain_or_pattern", "none")
        hit = _nearest(objs, cam, float(s.label["x_mm"]), float(s.label["y_mm"])) if positive else None
        if positive:
            tp += hit is not None
            fn += hit is None
        else:
            neg += 1
            fp += bool(objs)
        rows.append({"sample_id": s.sample_id, "kind": s.label["kind"], "positive": positive, "detected": bool(objs),
                     "hit": hit is not None,
                     "diameter_est_mm": None if hit is None else round(hit.diameter_mm, 1),
                     "height_est_mm": None if hit is None else round(hit.height_mm, 2),
                     "height_measured": None if hit is None else hit.height_measured,
                     "dropout": None if hit is None else hit.line_dropout,
                     "shadow": None if hit is None else hit.shadow,
                     "label_d": float(s.label["diameter_mm"]), "label_h": float(s.label["height_mm"])})
    return {"source": "SYNTHETIC" if all(s.label["source"] == "SYNTHETIC" for s in samples) else "MIXED",
            "n": len(samples), "detection_rate": tp / max(tp + fn, 1), "false_alarm_rate": fp / max(neg, 1),
            "tp": tp, "fn": fn, "fp": fp, "negatives": neg, "rows": rows}


def _nearest(objs: list[Candidate], cam: Camera, x_mm: float, y_mm: float, tol_mm: float = 25.0) -> Candidate | None:
    best, bd = None, tol_mm
    for c in objs:
        fx, fy = c.floor_xy_mm
        d = float(np.hypot(fx - x_mm, fy - y_mm))
        if d < bd:
            best, bd = c, d
    return best
