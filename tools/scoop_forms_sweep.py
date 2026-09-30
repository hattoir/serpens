"""形と機構（A〜G）の掃引。**MUJOCO_SIM。実物ではない。**

    python tools/scoop_forms_sweep.py screen           各案の有望なパラメータを絞る（N は 1 設計 × 対象物あたり 12。全体 1 万回未満）
    python tools/scoop_forms_sweep.py combos           A / B / C の上位設計 × F（壁・脚）× G（漏斗）× 前進速度 2 / 10
    python tools/scoop_forms_sweep.py stage3           上位 3 設計だけを 1 設計 × 対象物あたり N = 30（両方の速度・停止して閉じる / 前進しながら閉じる も別に）
    python tools/scoop_forms_sweep.py resummarize      output/ の 1 エピソード 1 行の CSV から集計を作り直す

1 設計 × 1 対象物あたりの試行数 = 床 2 × 位置ずれ（0 / 5 / 10。カップは 0 / 3 / 6 / 10）× n-cell（既定 2）。
保持（success）は、閉じ終わりから 2 秒後まで空間の中にあり、逃げず、**飛ばされていない**（飛ばされて入った分は knocked_in として別集計）。
出力: output/scoop_forms_<stage>_rows.csv（git 管理外）、simulation/results/scoop_forms_<stage>_designs.csv（設計 × 対象物の集計）
"""
from __future__ import annotations

import argparse
import csv
import itertools
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from simulation.scoop.forms.sweep import FCase, cases_for, run_fcases, summarize, tag_of  # noqa: E402
from simulation.scoop.model import load_config  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
OUT = REPO / "output"
RES = REPO / "simulation" / "results"


def screen_designs() -> list[tuple[str, dict, float, bool]]:
    """(案, パラメータ, 頭の速度, 停止して閉じる)。頭の速度は A / B / C では 10 に固定（2 は上位案で見る）。"""
    d: list[tuple[str, dict, float, bool]] = []
    for funnel, bs, v in itertools.product((False, True), (None, "wall", "leg"), (2.0, 10.0)):         # H / G: 受け身のフード（基準）
        d.append(("hood", {"funnel": funnel, "backstop": bs}, v, True))
    for L, S, T, soft in itertools.product((15, 20, 25), (60, 90, 120), (0.3, 0.6), (False, True)):    # A
        d.append(("sweeper", {"L": L, "S": S, "T": T, "soft": soft}, 10.0, True))
    for bs, mu, vb, dd in itertools.product((None, "wall"), (0.3, 0.6, 1.0), (10, 20, 40), (3, 4)):    # B
        d.append(("belt", {"mu": mu, "vb": vb, "d": dd, "backstop": bs}, 10.0, True))
    for bs, D, rpm, st in itertools.product((None, "wall"), (8, 12), (60, 120, 240), ("soft", "hard")):  # C
        d.append(("brush", {"D": D, "rpm": rpm, "stiff": st, "backstop": bs}, 10.0, True))
    for ID, gap in itertools.product((30, 40), (0.0, 0.3, 1.0)):                                       # D（前進しない）
        d.append(("cup", {"ID": ID, "gap_mm": gap}, 0.0, True))
    for tip, h, v in itertools.product(("round", "square"), (2, 4), (2.0, 10.0)):                      # E
        d.append(("hook", {"tip": tip, "h": h}, v, True))
    return d


def _typed(v: str):
    if v in ("True", "False"):
        return v == "True"
    if v in ("None", ""):
        return None
    try:
        return float(v)
    except ValueError:
        return v


def write_rows(name: str, rows: list[dict]) -> None:
    OUT.mkdir(exist_ok=True)
    keys = sorted({k for r in rows for k in r}, key=lambda k: (k.startswith("p_"), k))
    with open(OUT / f"scoop_forms_{name}_rows.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


def design_table(rows: list[dict], name: str) -> list[dict]:
    summ = summarize(rows, ("tag", "obj"))
    meta = {r["tag"]: r for r in rows}
    for s in summ:
        m = meta[s["tag"]]
        s["form"] = m["form"]
        s["speed"] = m["speed"]
        s["stop"] = m["stop"]
    RES.mkdir(parents=True, exist_ok=True)
    keys = sorted({k for s in summ for k in s})
    with open(RES / f"scoop_forms_{name}_designs.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["tag", "obj"] + [k for k in keys if k not in ("tag", "obj")])
        w.writeheader()
        w.writerows(sorted(summ, key=lambda s: (s["tag"], s["obj"])))
    per: dict[str, list] = {}
    for s in summ:
        per.setdefault(s["tag"], []).append(s)
    rank = []
    for tag, o in per.items():
        n = len(o)
        rank.append(dict(tag=tag, form=o[0]["form"], success=sum(x["rate"] for x in o) / n,
                         enter=sum(x["n_entered"] / x["n"] for x in o) / n,
                         launched=sum(x["n_launched"] / x["n"] for x in o) / n,
                         min_obj=min(x["rate"] for x in o)))
    return sorted(rank, key=lambda x: (-x["success"], -x["min_obj"], -x["enter"], x["tag"]))


def parse_tag(tag: str) -> tuple[str, dict, float, bool]:
    form, body, v, mode = tag.split("|")
    params = {}
    for kv in body.split(","):
        if not kv:
            continue
        k, val = kv.split("=", 1)
        params[k] = _parse_value(val)
    return form, params, float(v[1:]), mode == "stop"


def _parse_value(s: str):
    if s in ("True", "False"):
        return s == "True"
    if s == "None":
        return None
    try:
        f = float(s)
        return int(f) if f == int(f) and "." not in s else f
    except ValueError:
        return s


def top_per_form(rank: list[dict], forms: tuple[str, ...]) -> dict[str, dict]:
    best: dict[str, dict] = {}
    for x in rank:
        if x["form"] in forms and x["form"] not in best:
            best[x["form"]] = x
    return best


def load_rank(name: str) -> list[dict]:
    per: dict[str, list] = {}
    for r in csv.DictReader(open(RES / f"scoop_forms_{name}_designs.csv", encoding="utf-8")):
        per.setdefault(r["tag"], []).append(r)
    out = []
    for tag, o in per.items():
        n = len(o)
        out.append(dict(tag=tag, form=o[0]["form"], success=sum(float(x["rate"]) for x in o) / n,
                        enter=sum(float(x["n_entered"]) / float(x["n"]) for x in o) / n,
                        launched=sum(float(x["n_launched"]) / float(x["n"]) for x in o) / n,
                        min_obj=min(float(x["rate"]) for x in o)))
    return sorted(out, key=lambda x: (-x["success"], -x["min_obj"], -x["enter"], x["tag"]))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("stage", choices=["screen", "combos", "stage3", "resummarize"])
    ap.add_argument("--n-cell", type=int, default=2)
    ap.add_argument("--workers", type=int, default=14)
    args = ap.parse_args()
    cfg = load_config()
    t0 = time.time()
    if args.stage == "resummarize":
        for name in ("screen", "combos", "stage3"):
            p = OUT / f"scoop_forms_{name}_rows.csv"
            if p.exists():
                rows = [{k: _typed(v) for k, v in r.items()} for r in csv.DictReader(open(p, encoding="utf-8"))]
                design_table(rows, name)
                print("resummarized", name, len(rows))
        return
    if args.stage == "screen":
        designs, n_cell = screen_designs(), args.n_cell
    elif args.stage == "combos":
        top = top_per_form(load_rank("screen"), ("sweeper", "belt", "brush"))
        designs, n_cell = [], args.n_cell
        for form, x in top.items():
            _, params, _, _ = parse_tag(x["tag"])
            for funnel, bs, v in itertools.product((False, True), (None, "wall", "leg"), (2.0, 10.0)):
                designs.append((form, {**params, "funnel": funnel, "backstop": bs}, v, True))
    else:
        rank = load_rank("screen") + [r for r in (load_rank("combos") if (RES / "scoop_forms_combos_designs.csv").exists() else [])]
        rank = sorted(rank, key=lambda x: (-x["success"], -x["min_obj"], -x["enter"], x["tag"]))
        top3, seen = [], set()
        for x in rank:
            f_, p_, _, _ = parse_tag(x["tag"])
            key = (f_, tuple(sorted(p_.items(), key=lambda kv: kv[0])))          # 速度・閉じ方の違いは同じ設計として数える
            if f_ != "hood" and key not in seen and len(top3) < 3:
                seen.add(key)
                top3.append(x)
        designs, n_cell = [], 5
        for x in top3:
            form, params, v, stop = parse_tag(x["tag"])
            speeds = (2.0, 10.0) if form not in ("cup",) else (0.0,)
            for sp in speeds:
                for st in ((True, False) if form == "sweeper" else (True,)):
                    designs.append((form, params, sp, st))
    designs = list({tag_of(f, p, v, s): (f, p, v, s) for f, p, v, s in designs}.values())
    cases: list[FCase] = []
    for f, p, v, s in designs:
        cases += cases_for(cfg, f, p, v, s, n_cell)
    print(f"{args.stage}: {len(designs)} designs, {len(cases)} episodes", flush=True)
    rows = run_fcases(cases, workers=args.workers)
    write_rows(args.stage, rows)
    rank = design_table(rows, args.stage)
    print(f"done in {time.time() - t0:.0f} s", flush=True)
    for x in rank[:25]:
        print(f"{x['success']:.3f} min_obj {x['min_obj']:.2f} enter {x['enter']:.2f} launched {x['launched']:.2f}  {x['tag']}")


if __name__ == "__main__":
    main()
