"""短ランプ（5 / 8 / 10 / 15 mm、傾斜 12°）× 巻き込みくちばし（ヒンジ高さ 10 / 12 / 15 mm）の掃引。

User 決定 2026-09-29: 先端の薄さ・低摩擦の掃引はしない。先端厚 0.4 / 0.6、スコップ μ 0.3 固定、回転時間 0.3 / 0.6 s。
「乗る」（前縁が床から 1mm 以上上がった）、「入る」（中心が空間に入った）、「成功」（入ってフタが閉じ終わった時点で中にある）を別々に出す。
**MUJOCO_SIM。実物ではない。**

腕の長さの読み方が 2 通りあるので両方を回す（指示: 腕長 11mm・隙間 0.5mm。ヒンジ 12mm だと 12 − 11 = 1.0mm で食い違う）:
  arm11  … 腕長 11mm 固定（隙間 = ヒンジ高さ − 11。ヒンジ 10mm は腕先が床の下 1mm になるので除外）
  clr0.5 … 隙間 0.5mm 固定（腕長 = ヒンジ高さ − 0.5）

    python tools/scoop_ramp_sweep.py N [--fillet 0.3] [--workers 14] [--out output/scoop_beak] [--objects ...]
出力: <out>_rows.csv（1 エピソード 1 行）、<out>_summary.csv、<out>_summary.md
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from simulation.scoop.model import load_config, short_ramp_shapes  # noqa: E402
from simulation.scoop.sweep import LAUNCH_SPEED_MM_S, make_cases, run_cases, summarize  # noqa: E402


def _pct(x: float) -> str:
    return f"{100 * x:.0f}%"


def _cell(r: dict, kind: str) -> str:
    n = r["n"]
    k, lo, hi = {
        "rode": (r["n_rode"], r["rode_lo"], r["rode_hi"]),
        "entered": (r["n_entered"], r["entered_lo"], r["entered_hi"]),
        "success": (r["successes"], r["ci_lo"], r["ci_hi"]),
    }[kind]
    return f"{_pct(k / n)} ({k}/{n}, {_pct(lo)}–{_pct(hi)})"


def beak_configs(cfg: dict) -> list[tuple[str, float, float]]:
    b = cfg["beak"]
    out = []
    for h in b["hinge_z_mm"]:
        if h - b["arm_mm"] >= 0.2:
            out.append(("arm11", h, b["arm_mm"]))
        out.append(("clr0.5", h, h - b["tip_clearance_mm"]))
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("n", type=int)
    ap.add_argument("--fillet", type=float, default=None, help="円柱（1円玉・CR2032）の縁の丸み [mm]。未指定 = 丸みなし（config 既定）")
    ap.add_argument("--workers", type=int, default=14)
    ap.add_argument("--out", default="output/scoop_beak")
    ap.add_argument("--objects", nargs="*", default=None)
    ap.add_argument("--variants", nargs="*", default=["arm11", "clr0.5"])
    ap.add_argument("--torque", type=float, default=None, help="フタ（腕）のトルク上限 [N·m]。感度試験用。未指定 = config（0.05）")
    args = ap.parse_args()
    cfg = load_config()
    b = cfg["beak"]
    shapes = short_ramp_shapes(cfg)
    cases = []
    for variant, h, arm in beak_configs(cfg):
        if variant not in args.variants:
            continue
        for ct in b["close_time_s"]:
            cases += make_cases(cfg, shapes, args.n, objects=args.objects, trigger=b["trigger"], close_time_s=ct, beak=(h, arm),
                                rim_fillet_mm=args.fillet, tag=f"{variant}|H{h:g}|close{ct:g}")
    print(f"{len(cases)} episodes (N={args.n}, fillet={args.fillet})", flush=True)
    t0 = time.time()
    cfg_path = None
    if args.torque is not None:
        import yaml
        c2 = load_config()
        c2["lid"]["torque_limit_nm"] = args.torque
        cfg_path = str(Path(args.out).parent / f"_cfg_torque_{args.torque:g}.yaml")
        Path(cfg_path).parent.mkdir(parents=True, exist_ok=True)
        Path(cfg_path).write_text(yaml.safe_dump(c2, allow_unicode=True), encoding="utf-8")
    rows = run_cases(cases, workers=args.workers, config_path=cfg_path)
    print(f"done in {time.time() - t0:.0f} s", flush=True)
    for r in rows:
        r["variant"], r["hinge_tag"], r["close_tag"] = r["tag"].split("|")
        r["beak"] = f"{r['variant']}/H{r['beak_hinge_mm']:g}/arm{r['beak_arm_mm']:g}"
        r["launched"] = r["obj_speed_max_mm_s"] > LAUNCH_SPEED_MM_S
        r["success_clean"] = bool(r["success"]) and not r["launched"]
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(f"{out}_rows.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    tables = {
        "ランプ長 × 対象物": ("ramp_mm", "obj"),
        "腕の構成 × 対象物": ("beak", "obj"),
        "ヒンジ高さ × 対象物": ("beak_hinge_mm", "obj"),
        "回転時間 × 対象物": ("close_time_s", "obj"),
        "床 × 対象物": ("floor", "obj"),
        "速度 × 対象物": ("speed", "obj"),
        "先端厚 × 対象物": ("tip_mm", "obj"),
        "ランプ長 × 回転時間": ("ramp_mm", "close_time_s"),
    }
    all_summ = []
    md = [f"# 短ランプ × 巻き込みくちばし の掃引（N = {args.n}、円柱の縁の丸み = {args.fillet if args.fillet is not None else 'なし'}、"
          f"{len(rows)} エピソード）\n",
          "source = MUJOCO_SIM。**実物ではない。** 各行の区間は Wilson 90%。\n\n"
          "- **乗る** = 物の前縁（中心より前の表面）が床から 1mm 以上上がった瞬間が一度でもあった（User 定義）。\n"
          "  ただし、フタの腕が物を床とのあいだで挟んではじき飛ばした場合もこれに数えられる。飛ばされた分を除いた値を「乗る（飛ばされた除く）」に別に出す。\n"
          f"- **飛ばされた** = 物の速さが 1 度でも {LAUNCH_SPEED_MM_S:.0f} mm/s を超えた（ASSUMED: 腕先の最大速度の 2 倍。押して運ぶだけなら超えない）。\n"
          "- **入る** = 途中で一度でも物の中心が空間（ランプ終端より奥）の内側に、床から離れて載った。\n"
          "- **成功** = 入って、フタが閉じ終わった（腕が奥向き水平）時点で物が空間の内側にある。\n"
          "- 閉じ始めるきっかけ = 物がスコップに触れた瞬間（`contact:0`、ASSUMED）。閉じる間も頭は前進を続ける。\n"]
    # ランプ長 < 直径 の条件（解析）
    md.append("\n## ランプ長 < 直径 が成り立つか（解析）\n\n物の代表寸法（円柱・球は直径、立方体は辺）と、斜面に沿ったランプ長・水平の長さ（ランプ長 × cos 12°）の比較。\n")
    md.append("| 対象物 | 寸法 [mm] | " + " | ".join(f"{L:g} mm" for L in b_ramps(cfg)) + " |")
    md.append("|---|---|" + "---|" * len(b_ramps(cfg)))
    for name, ob in cfg["objects"].items():
        dim = ob.get("diameter_mm", ob.get("edge_mm"))
        md.append(f"| {name} | {dim:g} | " + " | ".join(("成り立つ" if L < dim else "成り立たない") + f"（{L / dim:.2f} 倍）" for L in b_ramps(cfg)) + " |")
    for title, keys in tables.items():
        summ = sorted(summarize(rows, keys), key=lambda r: tuple(str(r[k]) for k in keys))
        for r in summ:
            r["table"] = title
        all_summ += summ
        md.append(f"\n## {title}\n\n| {keys[0]} | {keys[1]} | 乗る | 乗る（飛ばされた除く） | 入る | 成功 | 飛ばされた |\n|---|---|---|---|---|---|---|")
        for r in summ:
            md.append(f"| {r[keys[0]]} | {r[keys[1]]} | {_cell(r, 'rode')} | {_pct(r['rate_rode_clean'])} ({r['n_rode_clean']}/{r['n']}) | "
                      f"{_cell(r, 'entered')} | {_cell(r, 'success')} | {r['n_launched']}/{r['n']} |")
    md.append("\n## 逃げ距離・飛ばされ方（対象物ごと）\n\n| 対象物 | 押して逃げた回数 | 逃げ距離 平均 [mm] | 最大 [mm] | 成功のうち飛ばされた回数 | 物の最大速度 中央値 [mm/s] |\n|---|---|---|---|---|---|")
    for obj in sorted({r["obj"] for r in rows}):
        g = [r for r in rows if r["obj"] == obj]
        pushed = [r["forward_disp_mm"] for r in g if r["outcome"] == "pushed_ahead"]
        succ = [r for r in g if r["success"]]
        sp = sorted(r["obj_speed_max_mm_s"] for r in g)
        md.append(f"| {obj} | {len(pushed)} / {len(g)} | {sum(pushed) / len(pushed) if pushed else 0:.1f} | {max(pushed) if pushed else 0:.1f} | "
                  f"{sum(r['launched'] for r in succ)} / {len(succ)} | {sp[len(sp) // 2]:.0f} |")
    keys_all = sorted({k for r in all_summ for k in r})
    with open(f"{out}_summary.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["table"] + [k for k in keys_all if k != "table"])
        w.writeheader()
        w.writerows(all_summ)
    Path(f"{out}_summary.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print("\n".join(md[:60]))


def b_ramps(cfg: dict) -> list[float]:
    return list(cfg["ramp_sweep"]["ramp_mm"])


if __name__ == "__main__":
    main()
