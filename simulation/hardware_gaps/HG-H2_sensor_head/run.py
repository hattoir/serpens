r"""HG-H2 センサーヘッドの Hardware Gap: 実カメラの特性を合成画像に重ね、検出性能がどの特性に敏感かを調べる。

    .\.venv\Scripts\python.exe simulation\hardware_gaps\HG-H2_sensor_head\run.py
    .\.venv\Scripts\python.exe simulation\hardware_gaps\HG-H2_sensor_head\run.py --quick
    .\.venv\Scripts\python.exe simulation\hardware_gaps\HG-H2_sensor_head\run.py --measured-fov  fov.csv
    .\.venv\Scripts\python.exe simulation\hardware_gaps\HG-H2_sensor_head\run.py --measured-focus focus.csv
    .\.venv\Scripts\python.exe simulation\hardware_gaps\HG-H2_sensor_head\run.py --real-dataset data\floorwatch\head_2026-10

描画と検出は既存の `serpens/floorwatch`（Renderer / detect）をそのまま使う。ここで足すのは**実カメラの劣化**だけ:
  ピント（薄肉レンズの錯乱円 c = 口径 · f_px · |1/d_focus − 1/d|、画素ごとの距離で）、放射歪み k1（検出器は知らない）、
  横ぶれ、露出ゲインと飽和、ショット雑音 + 読み出し雑音。
**合成画像の性能（SYNTHETIC_SENSOR_SIM）を実画像の性能として扱わない。**
"""
from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import shutil
import sys
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT))

from serpens.config import load_config  # noqa: E402
from serpens.floorwatch.dataset import _nearest, evaluate as evaluate_dataset  # noqa: E402
from serpens.floorwatch.detect import MotionError, detect  # noqa: E402
from serpens.floorwatch.geometry import Camera, LightPlane  # noqa: E402
from serpens.floorwatch.synthetic import Disc, Lighting, Renderer, Scene, Seam, Stain  # noqa: E402

A = yaml.safe_load((HERE / "assumptions.yaml").read_text(encoding="utf-8"))
SW = yaml.safe_load((HERE / "sweep_config.yaml").read_text(encoding="utf-8"))
SOURCE = str(A["label"])
BANDS = np.array([0.0, 0.7, 1.4, 2.8, 5.6, 11.0, 22.0])      # ぼけの σ [px] の段（画素ごとに近い段を使う）


# ---- カメラ・設定 ---------------------------------------------------------------------------------
def camera(p: dict[str, Any]) -> Camera:
    w = int(p["width_px"])
    h = int(round(w * 3 / 4))
    f = w / 2.0 / math.tan(math.radians(float(p["fov_h_deg"])) / 2.0)
    return Camera(f, w / 2.0, h / 2.0, w, h, float(p["cam_height_mm"]), float(p["cam_pitch_deg"]))


def cfg_for(base: dict[str, Any], p: dict[str, Any]) -> dict[str, Any]:
    cfg = copy.deepcopy(base)
    fw = cfg["floor_watch"]
    fw["camera"]["height_mm"] = float(p["cam_height_mm"])
    fw["camera"]["pitch_deg"] = float(p["cam_pitch_deg"])
    fw["line_light"]["height_mm"] = float(p["cam_height_mm"])      # 投光部はカメラと同じ高さ（頭に一緒に付く）
    fw["line_light"]["width_mm_initial"] = float(p["line_width_mm"])
    fw["raking"]["led_height_mm"] = float(p["raking_led_height_mm"])
    return cfg


def lighting(p: dict[str, Any]) -> Lighting:
    return Lighting(raking_led_height_mm=float(p["raking_led_height_mm"]), shadow_factor=float(p["shadow_factor"]),
                    line_width_mm=float(p["line_width_mm"]), ambient_lux=float(p["ambient_lux"]), noise_sigma=0.0)


def view_center_mm(p: dict[str, Any]) -> float:
    return float(p["cam_height_mm"]) / math.tan(math.radians(float(p["cam_pitch_deg"])))


# ---- 劣化 ----------------------------------------------------------------------------------------
class Degrader:
    """1 つのカメラ条件の劣化（ぼけの段は条件ごとに 1 回だけ計算する）。"""

    def __init__(self, cam: Camera, rend: Renderer, p: dict[str, Any]) -> None:
        self.p, self.cam = p, cam
        xa, ya, za = cam._axes
        fxy = rend.floor_xy
        pts = np.dstack([np.nan_to_num(fxy[..., 0]), np.nan_to_num(fxy[..., 1]), np.zeros(fxy.shape[:2])])
        z = (pts - cam.center) @ za
        z = np.where(np.isnan(fxy[..., 0]) | (z <= 1.0), 1e5, z)              # 地平線より上は遠く
        c_px = float(p["aperture_mm"]) * cam.f_px * np.abs(1.0 / float(p["focus_mm"]) - 1.0 / z)
        sigma = c_px / 2.5
        self.band = np.abs(sigma[..., None] - BANDS[None, None, :]).argmin(axis=2)
        self.sigma_center = float(sigma[int(cam.cy), int(cam.cx)])
        k1 = float(p["distortion_k1"])
        if k1:
            v, u = np.mgrid[0:cam.height_px, 0:cam.width_px].astype(np.float32)
            xn, yn = (u - cam.cx) / cam.f_px, (v - cam.cy) / cam.f_px
            r2 = xn * xn + yn * yn
            s = 1.0 / (1.0 + k1 * r2)                                          # 歪んだ画素 → ピンホールの画素（1 次の逆）
            self.map = (cam.cx + xn * s * cam.f_px).astype(np.float32), (cam.cy + yn * s * cam.f_px).astype(np.float32)
        else:
            self.map = None

    def __call__(self, img: np.ndarray, rng: np.random.Generator) -> np.ndarray:
        x = img.astype(np.float32)
        used = np.unique(self.band)
        if len(used) > 1 or used[0] != 0:
            out = np.zeros_like(x)
            for b in used:
                s = float(BANDS[b])
                y = cv2.GaussianBlur(x, (0, 0), s) if s > 0 else x
                out = np.where(self.band == b, y, out)
            x = out
        L = int(round(float(self.p["motion_blur_px"])))
        if L >= 2:
            x = cv2.filter2D(x, -1, np.full((1, L), 1.0 / L, np.float32))
        if self.map is not None:
            x = cv2.remap(x, self.map[0], self.map[1], cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        x = x * float(self.p["exposure_gain"])
        sig = np.sqrt(float(self.p["read_noise"]) ** 2 + float(self.p["shot_noise_k"]) * np.clip(x, 0, None))
        x = x + rng.normal(0.0, 1.0, x.shape).astype(np.float32) * sig
        return np.clip(x, 0, 255).astype(np.uint8)


# ---- 1 条件の評価 --------------------------------------------------------------------------------
def scenes(p: dict[str, Any]) -> list[tuple[str, str, bool, Scene, float, float]]:
    """(名前, kind, 陽性か, Scene, x, y)。物は線上（x=0）と線外、陰性は 2 位置。"""
    yc = view_center_mm(p)
    xo = -12.0 * yc / 64.0
    out = []
    fa = float(p["floor_albedo"])
    n = 0
    for name, s in A["surrogates"].items():
        for x in (0.0, xo):
            d = Disc(x, yc, float(s["diameter_mm"]), float(s["height_mm"]), float(s["albedo"]), bool(s["specular"]), s["kind"])
            out.append((name, s["kind"], True, Scene([d], floor_albedo=fa, seed=n), x, yc))
            n += 1
    for name, s in A["negatives"].items():
        for x in (0.0, xo):
            if name.startswith("seam"):
                sc = Scene(seams=[Seam(s["along"], yc, float(s["height_mm"]), "groove")], floor_albedo=fa, seed=n)
            else:
                sc = Scene(stains=[Stain(x, yc, float(s["diameter_mm"]), float(s["albedo"]))], floor_albedo=fa, seed=n)
            out.append((name, "negative", False, sc, x, yc))
            n += 1
    return out


def evaluate_condition(args: tuple[str, dict[str, Any]]) -> dict[str, Any]:
    tag, p = args
    base = load_config()
    cfg = cfg_for(base, p)
    cam = camera(p)
    plane = LightPlane.design(cfg)
    rend = Renderer(cam, plane, lighting(p))
    deg = Degrader(cam, rend, p)
    rng = np.random.default_rng(int(SW["seed"]))
    rows = []
    for name, kind, positive, sc, x, y in scenes(p):
        frames = {k: deg(v, rng) for k, v in rend.render(sc).items()}
        res: dict[str, Any] = {"tag": tag, "name": name, "kind": kind, "positive": positive, "on_line": abs(x) < 1e-9}
        for stage, with_line in (("patrol", False), ("inspect", True)):
            try:
                cands, _, _ = detect(frames, cam, plane, cfg, with_line=with_line)
                objs = [c for c in cands if c.is_object]
                res[f"{stage}_retake"] = False
            except MotionError:
                objs = []
                res[f"{stage}_retake"] = True
            reach = float(cfg["floor_watch"]["mission"]["reach_mm"])
            near = [c for c in objs if math.hypot(*c.floor_xy_mm) <= reach]     # 頭から届く範囲（誤報として数える）
            hit = _nearest(objs, x, y) if positive else None
            res[f"{stage}_hit"] = hit is not None
            res[f"{stage}_any"] = bool(near)
            res[f"{stage}_far_phantom"] = len(objs) > len(near)                 # 範囲外の偽物（遠方の線の途切れ等）
            if stage == "inspect" and hit is not None:
                res["size_err"] = abs(hit.diameter_mm - float(A["surrogates"][name]["diameter_mm"])) / float(A["surrogates"][name]["diameter_mm"])
                res["height_mm"] = hit.height_mm
                res["dropout"] = hit.line_dropout
        rows.append(res)
    return summarize(tag, p, rows, deg.sigma_center, cam)


def summarize(tag: str, p: dict[str, Any], rows: list[dict[str, Any]], sigma_center: float, cam: Camera) -> dict[str, Any]:
    pos = [r for r in rows if r["positive"]]
    on = [r for r in pos if r["on_line"]]
    neg = [r for r in rows if not r["positive"]]
    crit = [r for r in on if r["kind"] in A["critical_kinds"]]
    mean = lambda xs: float(np.mean(xs)) if xs else float("nan")
    return {"tag": tag, **{k: p[k] for k in A["nominal"]},
            "f_px": cam.f_px, "mm_per_px_center": cam.mm_per_px_at(cam.cx, cam.cy), "blur_sigma_center_px": sigma_center,
            "recall_patrol": mean([r["patrol_hit"] for r in pos]),
            "recall_inspect_online": mean([r["inspect_hit"] for r in on]),
            "recall_critical_online": mean([r["inspect_hit"] for r in crit]),
            "false_alarm_patrol": mean([r["patrol_any"] for r in neg]),
            "false_alarm_inspect": mean([r["inspect_any"] for r in neg]),
            "far_phantom_inspect": mean([r["inspect_far_phantom"] for r in rows]),
            "size_err_mean": mean([r["size_err"] for r in on if "size_err" in r]),
            "retake_rate": mean([r["inspect_retake"] for r in rows]),
            "missed": ";".join(sorted({r["name"] for r in on if not r["inspect_hit"]})),
            "n_pos": len(pos), "n_neg": len(neg)}


def conditions(quick: bool) -> list[tuple[str, dict[str, Any]]]:
    nom = dict(A["nominal"])
    out = [("nominal", nom)]
    for k, vals in SW["oat"].items():
        for v in (vals[:1] if quick else vals):
            out.append((f"{k}={v}", {**nom, k: v}))
    if not quick:
        out.append(("worst_case", {**nom, **SW["worst_case"]}))
        out.append(("no_raking", {**nom, "shadow_factor": 1.0}))       # 斜め照明の効き（影が出ない場合）
    return out


def ok(r: dict[str, Any]) -> bool:
    c = A["criteria"]
    return (r["recall_inspect_online"] >= c["min_recall_inspect"] and r["recall_patrol"] >= c["min_recall_patrol"]
            and max(r["false_alarm_patrol"], r["false_alarm_inspect"]) <= c["max_false_alarm"])


def ok_with_phantoms(r: dict[str, Any]) -> bool:
    """範囲外の偽物も誤報として数えた場合（2026-10-01 の統合 branch の検出では 0 条件。以前の検出では 26/44 条件で出ていた）。"""
    return ok(r) and r["far_phantom_inspect"] <= A["criteria"]["max_false_alarm"]


def write_decision(rows: list[dict[str, Any]], stamp: str) -> None:
    nom = next(r for r in rows if r["tag"] == "nominal")
    L = [f"# HG-H2 センサーヘッド — 感度と判定の境界（自動生成 {stamp}。source = {SOURCE}。**合成画像。実画像の性能ではない**）\n",
         "既存の Renderer / detect に実カメラの劣化を重ねた。1 変数ずつ振り、残りは `assumptions.yaml` の nominal。"
         f"1 条件 = 小物 {len(A['surrogates'])} 種 × 2 位置 + 陰性 {len(A['negatives'])} 種 × 2 位置。**n が小さいので 1 件の差で割合が大きく動く**。\n",
         f"基準（ASSUMPTION）: 停止時（線上）の検出率 ≥ {A['criteria']['min_recall_inspect']:.0%}、巡回時 ≥ {A['criteria']['min_recall_patrol']:.0%}、"
         f"誤報率 ≤ {A['criteria']['max_false_alarm']:.0%}\n",
         "誤報は頭から `floor_watch.mission.reach_mm` 以内の候補だけを数える。それより遠い偽物（ぼけた遠方で線光が途切れて出る "
         "`specular_break` など）は「範囲外の偽物」として別に数える（2026-10-01 の統合 branch `f8f2229` で回し直した結果: **範囲外の偽物が出る条件は 26/44 → 0/44**（nominal 41% → 0%）。vision-sim の検出の修正（`d259f59` ほか）が既に除外している。farfield-roi の `line_max_range_mm` は統合していない。**SYNTHETIC_SENSOR_SIM。実カメラでは未確認**）。\n",
         "| 条件 | 巡回 検出 | 停止 検出（線上） | 危険物（線上） | 誤報 巡回 / 停止 | 範囲外の偽物 | 大きさ誤差 | 中心のぼけ σ px | 中心 mm/px | 撮り直し | 合否 | 見逃し |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        L.append(f"| {r['tag']} | {r['recall_patrol']:.0%} | {r['recall_inspect_online']:.0%} | {r['recall_critical_online']:.0%} | "
                 f"{r['false_alarm_patrol']:.0%} / {r['false_alarm_inspect']:.0%} | {r['far_phantom_inspect']:.0%} | {r['size_err_mean']:.0%} | "
                 f"{r['blur_sigma_center_px']:.1f} | {r['mm_per_px_center']:.3f} | {r['retake_rate']:.0%} | "
                 f"{'OK' if ok(r) else 'NG'}{'' if ok_with_phantoms(r) or not ok(r) else '（偽物込みで NG）'} | {r['missed'] or '—'} |")
    L.append("\n## 感度の順位（nominal からの停止時検出率・巡回検出率の最大の低下）\n")
    drops = {}
    for r in rows:
        if "=" not in r["tag"]:
            continue
        k = r["tag"].split("=")[0]
        d = max(nom["recall_inspect_online"] - r["recall_inspect_online"], nom["recall_patrol"] - r["recall_patrol"],
                r["false_alarm_patrol"] - nom["false_alarm_patrol"])
        drops[k] = max(drops.get(k, 0.0), d)
    L.append("| パラメータ | 最大の悪化 |\n|---|---|")
    for k, d in sorted(drops.items(), key=lambda kv: -kv[1]):
        L.append(f"| {k} | {d:+.0%} |")
    L.append("\n## 合格する範囲（掃引した値のうち、基準を満たしたもの）\n")
    for k, vals in SW["oat"].items():
        good = [v for v in [A["nominal"][k]] + list(vals)
                if (r := next((x for x in rows if x["tag"] == (f"{k}={v}" if v != A["nominal"][k] else "nominal")), None)) and ok(r)]
        L.append(f"- {k}: nominal {A['nominal'][k]} / 合格 {sorted(good)} / 掃引 {sorted([A['nominal'][k]] + list(vals))}")
    (HERE / "decision_boundary.md").write_text("\n".join(L) + "\n", encoding="utf-8")


def plots(rows: list[dict[str, Any]]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    (HERE / "plots").mkdir(exist_ok=True)
    keys = list(SW["oat"])
    fig, axes = plt.subplots(3, 5, figsize=(16, 9))
    nom = next(r for r in rows if r["tag"] == "nominal")
    for ax, k in zip(axes.flat, keys):
        pts = sorted([(A["nominal"][k], nom)] + [(float(r["tag"].split("=")[1]), r) for r in rows if r["tag"].startswith(k + "=")],
                     key=lambda t: t[0])
        xs = [t[0] for t in pts]
        ax.plot(xs, [t[1]["recall_inspect_online"] for t in pts], "o-", label="inspect (on line)")
        ax.plot(xs, [t[1]["recall_patrol"] for t in pts], "s-", label="patrol")
        ax.plot(xs, [t[1]["false_alarm_patrol"] for t in pts], "x--", label="false alarm (patrol)")
        ax.axvline(A["nominal"][k], color="gray", lw=0.8)
        ax.set_title(k, fontsize=9)
        ax.set_ylim(-0.05, 1.05)
        if k in ("focus_mm",):
            ax.set_xscale("log")
    axes.flat[0].legend(fontsize=7)
    fig.suptitle(f"{SOURCE} - synthetic images, not real camera")
    fig.tight_layout()
    fig.savefig(HERE / "plots" / "oat_sensitivity.png", dpi=110)
    plt.close(fig)


def run_all(quick: bool, workers: int) -> None:
    conds = conditions(quick)
    with ProcessPoolExecutor(max_workers=workers) as ex:
        rows = list(ex.map(evaluate_condition, conds))
    (HERE / "results").mkdir(exist_ok=True)
    name = "oat_quick.csv" if quick else "oat.csv"
    with (HERE / "results" / name).open("w", newline="", encoding="utf-8") as fp:
        w = csv.DictWriter(fp, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    if not quick:
        write_decision(rows, datetime.now().strftime("%Y-%m-%d %H:%M"))
        plots(rows)
    for r in rows:
        print(f"{r['tag']:28s} patrol {r['recall_patrol']:.0%} inspect {r['recall_inspect_online']:.0%} "
              f"FA {r['false_alarm_patrol']:.0%}/{r['false_alarm_inspect']:.0%} far {r['far_phantom_inspect']:.0%} "
              f"blur {r['blur_sigma_center_px']:.1f} {r['missed']}")


# ---- 実測の取り込み --------------------------------------------------------------------------------
def _keep_raw(src: Path, kind: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    d = HERE / "results" / "measured" / "raw"
    d.mkdir(parents=True, exist_ok=True)
    dst = d / f"{kind}_{src.name}_{stamp}"
    if src.is_dir():
        shutil.copytree(src, dst)
    else:
        shutil.copy2(src, dst)
    return dst


def ingest_fov(path: Path) -> dict[str, Any]:
    """定規の試験: 列 distance_mm, ruler_mm, ruler_px, width_px → f_px と水平画角。"""
    _keep_raw(path, "fov")
    with path.open(encoding="utf-8-sig", newline="") as fp:
        rows = [r for r in csv.DictReader(fp) if (r.get("ruler_px") or "").strip()]
    fs = [float(r["ruler_px"]) * float(r["distance_mm"]) / float(r["ruler_mm"]) for r in rows]
    w = float(rows[0]["width_px"])
    f = float(np.mean(fs))
    out = {"f_px": f, "f_px_spread": float(np.std(fs)), "width_px": w, "fov_h_deg": math.degrees(2 * math.atan(w / 2 / f)), "n": len(fs)}
    _write_measured("fov", out)
    return out


def ingest_focus(path: Path) -> dict[str, Any]:
    """ピントの試験: 列 distance_mm, blur_px（縁の 10–90% 幅 ≈ 2.56 σ）, f_px → 口径と合焦距離を格子で当てはめる。"""
    _keep_raw(path, "focus")
    with path.open(encoding="utf-8-sig", newline="") as fp:
        rows = [r for r in csv.DictReader(fp) if (r.get("blur_px") or "").strip()]
    d = np.array([float(r["distance_mm"]) for r in rows])
    sig = np.array([float(r["blur_px"]) for r in rows]) / 2.56
    f_px = float(rows[0]["f_px"])
    best = None
    for fo in np.geomspace(40, 10000, 200):
        base = f_px * np.abs(1 / fo - 1 / d) / 2.5
        a = float((base * sig).sum() / max((base * base).sum(), 1e-12))
        err = float(((a * base - sig) ** 2).sum())
        if best is None or err < best[0]:
            best = (err, fo, a)
    out = {"focus_mm": float(best[1]), "aperture_mm": float(best[2]), "rms_px": math.sqrt(best[0] / len(d)), "n": len(d)}
    _write_measured("focus", out)
    return out


def ingest_dataset(root: Path) -> dict[str, Any]:
    """実画像のデータセット（labels.csv 付き、serpens/floorwatch/dataset.py の形）を既存の評価器で測る。"""
    _keep_raw(root, "dataset")
    fov = _load_measured("fov")
    p = dict(A["nominal"])
    if fov:
        p["fov_h_deg"], p["width_px"] = fov["fov_h_deg"], int(fov["width_px"])
    cfg = cfg_for(load_config(), p)
    cam = camera(p)
    res = evaluate_dataset(root, cam, LightPlane.design(cfg), cfg)
    out = {"source": res["source"], "n": res["n"],
           **{f"{st}_{k}": res["stages"][st][k] for st in ("patrol", "inspect") for k in ("detection_rate", "false_alarm_rate")}}
    _write_measured("dataset", out)
    return out


def _measured_path(kind: str) -> Path:
    return HERE / "results" / "measured" / f"{kind}.json"


def _load_measured(kind: str) -> dict[str, Any] | None:
    p = _measured_path(kind)
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else None


def _write_measured(kind: str, data: dict[str, Any]) -> None:
    p = _measured_path(kind)
    p.parent.mkdir(parents=True, exist_ok=True)
    data = {**data, "recorded": datetime.now().isoformat(timespec="seconds"),
            "label": "HARDWARE_VERIFIED（この個体・この条件に限る）" if kind != "dataset" else "REAL_IMAGE_EVAL"}
    p.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
    ho = ROOT / "ai-outbox" / "handoffs" / f"{datetime.now():%Y-%m-%d}_HG-H2_measured.md"
    ho.parent.mkdir(parents=True, exist_ok=True)
    with ho.open("a", encoding="utf-8") as fp:
        fp.write(f"\n## {data['recorded']} H2 {kind} を取り込んだ\n\n```\n{json.dumps(data, ensure_ascii=False, indent=1)}\n```\n"
                 "- 次: `assumptions.yaml` の nominal をこの値で置き換えた条件で run.py を回し直す（手で確認してから）\n")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--workers", type=int, default=int(SW["workers"]))
    ap.add_argument("--measured-fov", type=Path)
    ap.add_argument("--measured-focus", type=Path)
    ap.add_argument("--real-dataset", type=Path)
    args = ap.parse_args()
    if args.measured_fov:
        print(ingest_fov(args.measured_fov))
    elif args.measured_focus:
        print(ingest_focus(args.measured_focus))
    elif args.real_dataset:
        print(ingest_dataset(args.real_dataset))
    else:
        run_all(args.quick, args.workers)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
