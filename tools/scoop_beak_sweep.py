"""巻き込みくちばし（beak sweeper）と奥ヒンジフタ（ベースライン）の掃引。**MUJOCO_SIM。実物ではない。**

    python tools/scoop_beak_sweep.py stage1 [--n-cell 2]       設計を絞った全設計（くちばし 144 + ベースライン 48）× 対象物 4 × 床 2 × ずれ 3 × n-cell
    python tools/scoop_beak_sweep.py stage3 [--top 3]          stage1 の上位のくちばし設計だけを、1 設計 × 対象物あたり N = 30 で
    python tools/scoop_beak_sweep.py sens   [--top 3]          上位設計の感度（トルク 0.02 / 0.18、幅・側壁の残り 2 通り、先端厚 0.4）

1 設計 × 1 対象物あたりの試行数は 床 2 × ずれ 3 × n-cell（既定 2）= 12。
指標: 乗る（頭側の縁が床から 1mm 以上上がった）/ 入る（物の中心が空間の範囲に入った）/ 保持（閉じ終わりから 2 秒後まで空間の中にいた）
      逃げた距離（判定時の、空間の範囲からの距離）/ 飛ばされた（物の速さが 350 mm/s 超）。成功率は Wilson 90%。
出力: output/scoop_beak_<stage>_rows.csv（1 エピソード 1 行、git 管理外）、simulation/results/scoop_beak_<stage>_designs.csv（設計 × 対象物の集計）
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from simulation.scoop.model import Shape, load_config  # noqa: E402
from simulation.scoop.sweep import LAUNCH_SPEED_MM_S, Case, run_cases, summarize  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
OUT = REPO / "output"
RES = REPO / "simulation" / "results"
FACTORS = ("H", "ramp", "close", "ws", "speed", "mode")


def design_id(d: dict) -> str:
    return (f"{d['kind']}|H{d['H'] if d['H'] is not None else '-'}|r{d['ramp']:g}|c{d['close']:g}|w{d['w']:g}{'cup' if d['side'] else 'open'}"
            f"|v{d['speed']:g}|{'stop' if d['stop'] else 'adv'}|t{d['tip']:g}|q{d['torque'] if d['torque'] is not None else 'cfg'}"
            f"|g{d.get('trigger') or 'cfg'}")


def base_designs(cfg: dict) -> list[dict]:
    b = cfg["beak"]
    out = []
    for kind, hs in (("beak", b["hinge_z_mm"]), ("rear", [None])):
        for H in hs:
            for ramp in b["ramp_mm"]:
                for close in b["close_time_s"]:
                    for w, side in b["width_side"]:
                        for speed in b["speeds_mm_s"]:
                            for stop in b["stop_on_trigger"]:
                                out.append(dict(kind=kind, H=H, ramp=ramp, close=close, w=w, side=bool(side), speed=speed, stop=bool(stop),
                                                tip=b["tip_mm"], torque=None, trigger=None))
    return out


def cases_for(cfg: dict, d: dict, n_cell: int) -> list[Case]:
    b = cfg["beak"]
    shape = Shape(d["tip"], cfg["ramp_sweep"]["alpha_deg"], d["side"], d["w"], d["ramp"])
    beak = None if d["kind"] == "rear" else (d["H"], d["H"] - b["tip_clearance_mm"])
    ahead = 5.0 if d["kind"] == "rear" else None           # ベースライン: 開き 60° でフタの前縁がランプ先端より 5mm 前に出る長さ
    tag = design_id(d)
    return [Case(shape, obj, floor, d["speed"], off, k, trigger=d.get("trigger") or b["trigger"], close_time_s=d["close"], beak=beak, lid_front_ahead_mm=ahead,
                 stop_on_trigger=d["stop"], tag=tag)
            for obj in cfg["objects"] for floor in cfg["floors"] for off in cfg["placement"]["lateral_offsets_mm"] for k in range(n_cell)]


def run_designs(cfg: dict, designs: list[dict], n_cell: int, workers: int) -> list[dict]:
    """トルクが違う設計は別の config で回す。"""
    by_torque: dict = {}
    for d in designs:
        by_torque.setdefault(d["torque"], []).append(d)
    rows: list[dict] = []
    for tq, ds in by_torque.items():
        path = None
        if tq is not None:
            import yaml
            c2 = load_config()
            c2["lid"]["torque_limit_nm"] = tq
            OUT.mkdir(exist_ok=True)
            path = str(OUT / f"_cfg_torque_{tq:g}.yaml")
            Path(path).write_text(yaml.safe_dump(c2, allow_unicode=True), encoding="utf-8")
        cases = [c for d in ds for c in cases_for(cfg, d, n_cell)]
        print(f"  torque={tq}: {len(cases)} episodes", flush=True)
        part = run_cases(cases, workers=workers, config_path=path)
        by_tag = {design_id(d): d for d in ds}
        for r in part:
            d = by_tag[r["tag"]]
            r.update(kind=d["kind"], H=d["H"], ramp=d["ramp"], close=d["close"], w=d["w"], side=d["side"], speed=d["speed"],
                     mode="stop" if d["stop"] else "adv", trigger=d.get("trigger"), ws=f"{d['w']:g}{'cup' if d['side'] else 'open'}", tip=d["tip"], torque=d["torque"])
            r["launched"] = r["obj_speed_max_mm_s"] > LAUNCH_SPEED_MM_S
        rows += part
    return rows


def write_rows(name: str, rows: list[dict]) -> None:
    OUT.mkdir(exist_ok=True)
    with open(OUT / f"scoop_beak_{name}_rows.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def design_table(rows: list[dict], name: str) -> list[dict]:
    """設計 × 対象物の集計を CSV に書き、設計ごとの順位づけ用の値を返す。"""
    summ = summarize(rows, ("tag", "obj"))
    meta = {r["tag"]: r for r in rows}
    for s in summ:
        m = meta[s["tag"]]
        s.update({k: m[k] for k in ("kind", "H", "ramp", "close", "w", "side", "speed", "mode", "tip", "torque")})
    RES.mkdir(parents=True, exist_ok=True)
    keys = sorted({k for s in summ for k in s})
    with open(RES / f"scoop_beak_{name}_designs.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["tag", "obj"] + [k for k in keys if k not in ("tag", "obj")])
        w.writeheader()
        w.writerows(sorted(summ, key=lambda s: (s["tag"], s["obj"])))
    per: dict[str, dict] = {}
    for s in summ:
        p = per.setdefault(s["tag"], dict(tag=s["tag"], kind=s["kind"], objs={}))
        p["objs"][s["obj"]] = s
    out = []
    for tag, p in per.items():
        o = p["objs"].values()
        out.append(dict(tag=tag, kind=p["kind"],
                        success=sum(x["rate"] for x in o) / len(p["objs"]),
                        enter=sum(x["rate_entered"] for x in o) / len(p["objs"]),
                        ride=sum(x["rate_rode"] for x in o) / len(p["objs"]),
                        launched=sum(x["n_launched"] / x["n"] for x in o) / len(p["objs"]),
                        clean=sum(x["n_success_clean"] / x["n"] for x in o) / len(p["objs"]) if "n_success_clean" in next(iter(o)) else 0.0))
    return sorted(out, key=lambda x: (-x["success"], -x["enter"], -x["ride"], x["tag"]))


def _typed(v: str):
    if v in ("True", "False"):
        return v == "True"
    if v in ("None", ""):
        return None
    try:
        return float(v)
    except ValueError:
        return v


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("stage", choices=["stage1", "stage3", "sens", "resummarize"])
    ap.add_argument("--n-cell", type=int, default=2)
    ap.add_argument("--top", type=int, default=3)
    ap.add_argument("--workers", type=int, default=14)
    args = ap.parse_args()
    cfg = load_config()
    t0 = time.time()
    if args.stage == "resummarize":                  # シミュレーションは回さず、output/ の 1 エピソード 1 行の CSV から設計 × 対象物の集計を作り直す
        for name in ("stage1", "stage3", "sens"):
            rows = [{k: _typed(v) for k, v in r.items()} for r in csv.DictReader(open(OUT / f"scoop_beak_{name}_rows.csv", encoding="utf-8"))]
            design_table(rows, name)
            print("resummarized", name, len(rows))
        return
    if args.stage == "stage1":
        designs = base_designs(cfg)
        print(f"stage1: {len(designs)} designs, {len(designs) * 48 * args.n_cell // 2} episodes", flush=True)
        rows = run_designs(cfg, designs, args.n_cell, args.workers)
        write_rows("stage1", rows)
        rank = design_table(rows, "stage1")
    else:
        prev = list(csv.DictReader(open(RES / "scoop_beak_stage1_designs.csv", encoding="utf-8")))
        per: dict[str, dict] = {}
        for r in prev:
            per.setdefault(r["tag"], {"rate": 0.0, "row": r})["rate"] += float(r["rate"]) / 4
        beak_tags = [t for t, v in per.items() if v["row"]["kind"] == "beak"]
        # 順位: 保持率の対象物平均 → 入る率 → 乗る率
        def score(t: str) -> tuple:
            rs = [r for r in prev if r["tag"] == t]
            return (-sum(float(r["rate"]) for r in rs), -sum(float(r["rate_entered"]) for r in rs), -sum(float(r["rate_rode"]) for r in rs), t)
        top = sorted(beak_tags, key=score)[:args.top]
        tops = []
        for t in top:
            r = next(x for x in prev if x["tag"] == t)
            tops.append(dict(kind="beak", H=float(r["H"]), ramp=float(r["ramp"]), close=float(r["close"]), w=float(r["w"]), side=r["side"] == "True",
                             speed=float(r["speed"]), stop=r["mode"] == "stop", tip=float(r["tip"]), torque=None, trigger=None))
        if args.stage == "stage3":
            designs, n_cell = tops, 5                      # 1 設計 × 対象物あたり 床 2 × ずれ 3 × 5 = 30
        else:
            designs, n_cell = [], 2
            for d in tops:
                for tq in (0.02, 0.18):
                    designs.append({**d, "torque": tq})
                for w, side in ((30.0, True), (40.0, False)):
                    designs.append({**d, "w": w, "side": side})
                designs.append({**d, "tip": 0.4})
                for adv in (10, 30):                       # 触れてから前進 adv mm 後に閉じ始める（物がランプに乗ってから掃く）
                    designs.append({**d, "trigger": f"contact:{adv}"})
        designs = list({design_id(d): d for d in designs}.values())      # 上位 3 案の近傍が重なる分は 1 回だけ回す
        print(f"{args.stage}: {len(designs)} designs, n_cell={n_cell}", flush=True)
        rows = run_designs(cfg, designs, n_cell, args.workers)
        write_rows(args.stage, rows)
        rank = design_table(rows, args.stage)
    print(f"done in {time.time() - t0:.0f} s", flush=True)
    for x in rank[:20]:
        print(f"{x['success']:.3f} enter {x['enter']:.3f} ride {x['ride']:.3f} launched {x['launched']:.3f}  {x['tag']}")


if __name__ == "__main__":
    main()
