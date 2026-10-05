"""R-029（Design）: 囲い込みの検査を、頭の幅 103.2 mm（頬込み。半径 51.6。現入力は 100 mm = 半径 50）と、頭ヨー軸の位置
（軸から頭の先端まで 30 / 50 / 56.2（現入力）/ 70 mm。全長は変えない = 軸の手前のリンクがその分だけ伸び縮みする）で再実行する。
格子は v2（GRID_DEG 9 値、形 6181。`entrapment_check.py`）。指標 = 口の幅の最小（首の太い側 88.5 mm との差）。GEOMETRY_SIM（平面・剛体）。
頭ヨー軸は CAD に無い = ASSUMED。実機・実物で未確認（HARDWARE_VERIFIED = 0）。5.7 N・0.25 N·m は暫定（使わない）。
    python entrapment_head_sweep.py [--workers 12]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import entrapment_check as ec  # noqa: E402

OUT = ec.ROOT / "simulation" / "results" / "entrapment_head_sweep_2026-10-05.json"
VARIANTS = (("w100_L56.2 (current)", 100.0, 56.2), ("w103.2_L56.2", 103.2, 56.2), ("w103.2_L30", 103.2, 30.0), ("w103.2_L50", 103.2, 50.0), ("w103.2_L70", 103.2, 70.0))
YAWS = (30.0, 45.0, 60.0)
NCHUNK = 12


_ORIG = (ec.clearances, ec.trap_diameters, ec.pocket)               # ワーカーごとに 1 回だけ保存（毎回かぶせない）


def _with_radii(fn, radii):
    def wrapped(pts, r=None, *a, **k):
        return fn(pts, radii if r is None else r, *a, **k)
    return wrapped


def job(args: tuple[str, float, float, float, int, int]) -> dict:
    name, width, tip, yaw, ci, nchunk = args
    ec.LINKS_MM = (145.0, 95.0, 95.0, 95.0, 86.8 + (56.2 - tip), tip)
    radii = (46.0, 46.0, 46.0, 46.0, 46.0, width / 2.0)
    ec.RADII_MM = radii
    ec.clearances, ec.trap_diameters, ec.pocket = (_with_radii(f, radii) for f in _ORIG)
    cfgs = ec.body_configs()
    cfgs = [c for k, c in enumerate(cfgs) if k % nchunk == ci]
    r = ec.sweep(yaw, cfgs)
    return {"variant": name, "yaw": yaw, "n": r["n"], "pockets": r["pockets"], "min_head_to_body_mm": r["min_head_to_body_mm"]}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=13)
    ap.add_argument("--variant", type=int, default=None, help="VARIANTS の番号（1 回の実行を 10 分以内に収めるため 1 つずつ回す。結果は variant ごとの json）")
    a = ap.parse_args()
    use = VARIANTS if a.variant is None else (VARIANTS[a.variant],)
    jobs = [(n, w, t, s, ci, NCHUNK) for (n, w, t) in use for phi in YAWS for s in (phi, -phi) for ci in range(NCHUNK)]
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        res = list(ex.map(job, jobs))
    neck = ec.PARTS_MM["首"][1]
    out = {}
    for (n, w, t) in use:
        row = {}
        for phi in YAWS:
            pk = [p for r in res if r["variant"] == n and abs(r["yaw"]) == phi for p in r["pockets"]]
            if not pk:
                row[f"{phi:g}"] = None
                continue
            mouth = min(p["mouth_mm"] for p in pk)
            row[f"{phi:g}"] = {"pockets": len(pk), "min_mouth_mm": mouth, "margin_to_neck_mm": mouth - neck, "max_d_in_mm": max(p["d_in_mm"] for p in pk)}
        out[n] = {"width_mm": w, "tip_mm": t, "by_yaw": row}
        print(n, {k: (None if v is None else (v["pockets"], round(v["min_mouth_mm"]), round(v["margin_to_neck_mm"], 1))) for k, v in row.items()})
    out_path = OUT if a.variant is None else OUT.with_name(f"entrapment_head_sweep_v{a.variant}_2026-10-05.json")
    out_path.write_text(json.dumps(out, indent=1, default=float), encoding="utf-8")


if __name__ == "__main__":
    main()
