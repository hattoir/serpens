"""形と機構（A〜G）の掃引。**MUJOCO_SIM。実物ではない。**

    python tools/scoop_forms_sweep.py screen           各案の有望なパラメータを絞る（N は 1 設計 × 対象物あたり 12。全体 1 万回未満）
    python tools/scoop_forms_sweep.py combos           A / B / C の上位設計 × F（壁・脚）× G（漏斗）× 前進速度 2 / 10
    python tools/scoop_forms_sweep.py stage3           上位 3 設計だけを 1 設計 × 対象物あたり N = 30（両方の速度・停止して閉じる / 前進しながら閉じる も別に）
    python tools/scoop_forms_sweep.py baseline         受け身のフードの基準（N = 30）: 漏斗の有無 × ゲートの有無 × 閉じ終わり後の後退（0 / 30 mm）× 頭の速度 2 / 10、
                                                       と、口の床の段差（0.1〜1.5 mm）を足した対照。物 × 床 × 速度 × 位置ずれの内訳の CSV も出す
    python tools/scoop_forms_sweep.py tolerance        口の前縁の段差 {0, 0.1, 0.2, 0.5} mm × 前縁と床のすき間 {0, 0.3, 1} mm（漏斗つき・ゲートあり。実物の許容差の材料）
    python tools/scoop_forms_sweep.py curtain          ゲートを受け身の TPU 垂れ布に（駆動なし。閉じる力 0.01〜1 N。ASSUMED）
    python tools/scoop_forms_sweep.py skirt            壁の下端の柔らかいスカート（高さ 3 / 6 mm）× 床の凹凸 ±0.5 mm（**絨毯の代用**）
    python tools/scoop_forms_sweep.py gateforce        駆動のゲートの力の上限を 5 → 0.25 N（暫定しきい値 5.7 N の半分 2.8 N 以下で成立するか）
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


def cell_table(rows: list[dict], name: str = "baseline") -> None:
    """設計 × 物 × 床 × 位置ずれの内訳（受け身のフードの基準。報告書の内訳表と「またぐ」の分類に使う）。"""
    g: dict[tuple, list[dict]] = {}
    for r in rows:
        g.setdefault((r["tag"], r["obj"], r["floor"], r["offset"]), []).append(r)
    RES.mkdir(parents=True, exist_ok=True)
    with open(RES / f"scoop_forms_{name}_cells.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["tag", "obj", "floor", "offset", "n", "success", "knocked_in", "escaped", "pushed_ahead", "not_entered", "pinched",
                    "entered", "wall_contact", "entered_nowall", "entered_wall", "fire_x_mean_mm", "fire_absy_mean_mm"])
        for (tag, obj, floor, off), rs in sorted(g.items()):
            oc = lambda k: sum(r["outcome"] == k for r in rs)
            ent = [r for r in rs if r["entered_ever"]]
            fx = [r["fire_rel_x_mm"] for r in rs if r["fire_rel_x_mm"] == r["fire_rel_x_mm"]]
            fy = [abs(r["fire_y_mm"]) for r in rs if r["fire_y_mm"] == r["fire_y_mm"]]
            w.writerow([tag, obj, floor, off, len(rs), oc("success"), oc("knocked_in"), oc("escaped"), oc("pushed_ahead"), oc("not_entered"), oc("pinched"),
                        len(ent), sum(bool(r["wall_contact"]) for r in rs), sum(not r["wall_contact"] for r in ent), sum(bool(r["wall_contact"]) for r in ent),
                        f"{sum(fx) / len(fx):.2f}" if fx else "", f"{sum(fy) / len(fy):.2f}" if fy else ""])


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
    ap.add_argument("stage", choices=["screen", "combos", "stage3", "baseline", "tolerance", "tolerance2", "tolerance3", "curtain", "curtain2", "skirt", "gateforce", "chamfer", "chamfer2", "rough", "solver", "realistic", "film", "cloche", "cloche2", "cloche3", "intake_c", "cover_intake", "resummarize"])
    ap.add_argument("--n-cell", type=int, default=2)
    ap.add_argument("--workers", type=int, default=14)
    args = ap.parse_args()
    cfg = load_config()
    t0 = time.time()
    if args.stage == "resummarize":
        for name in ("screen", "combos", "stage3", "baseline", "tolerance", "tolerance2", "tolerance3", "curtain", "curtain2", "skirt", "gateforce", "chamfer", "chamfer2", "rough", "solver", "realistic", "film", "cloche", "cloche2", "cloche3", "intake_c", "cover_intake"):
            p = OUT / f"scoop_forms_{name}_rows.csv"
            if p.exists():
                rows = [{k: _typed(v) for k, v in r.items()} for r in csv.DictReader(open(p, encoding="utf-8"))]
                design_table(rows, name)
                print("resummarized", name, len(rows))
        return
    if args.stage == "screen":
        designs, n_cell = screen_designs(), args.n_cell
    elif args.stage == "baseline":
        designs, n_cell = [], 5
        for funnel, gate, retreat, v in itertools.product((False, True), (True, False), (0, 30), (2.0, 10.0)):
            designs.append(("hood", {"funnel": funnel, "gate": gate, "retreat": retreat, "backstop": None}, v, True))
        for plate in (0.1, 0.3, 0.6, 1.0, 1.5):                       # 口の床の段差（前回のスコップの「先端の厚み」に当たる）を足した対照
            designs.append(("hood", {"funnel": False, "gate": True, "retreat": 0, "backstop": None, "plate": plate}, 10.0, True))
    elif args.stage == "tolerance":
        designs, n_cell = [], 5
        for c, plate in itertools.product((0.0, 0.3, 1.0), (0.0, 0.1, 0.2, 0.5)):
            designs.append(("hood", {"funnel": True, "gate": True, "retreat": 0, "backstop": None, "clearance_mm": c, "plate": plate}, 10.0, True))
    elif args.stage == "curtain":
        designs, n_cell = [], 5
        for F, retreat in itertools.product((0.01, 0.03, 0.1, 0.3, 1.0), (0, 30)):
            designs.append(("hood", {"funnel": True, "gate": "curtain", "curtain_f": F, "retreat": retreat, "backstop": None}, 10.0, True))
        for F in (0.1, 0.3, 1.0):
            designs.append(("hood", {"funnel": True, "gate": "curtain", "curtain_f": F, "retreat": 30, "backstop": None}, 2.0, True))
    elif args.stage == "tolerance2":                 # 段差の境界を細かく + 縁の丸み（実物の 1 円玉・CR2032 の縁は丸い）
        designs, n_cell = [], 5
        base = {"funnel": True, "gate": True, "retreat": 0, "backstop": None, "clearance_mm": 0.1}
        for fillet, plate in itertools.product((0.3,), (0.0, 0.02, 0.05, 0.1, 0.2, 0.5)):
            designs.append(("hood", {**base, "plate": plate, "rim_fillet_mm": fillet}, 10.0, True))
        for plate in (0.02, 0.05):
            designs.append(("hood", {**base, "plate": plate}, 10.0, True))
    elif args.stage == "tolerance3":                 # 段差の境界をさらに細かく（0.002〜0.01 mm）。接触のやわらかさの分解能に近い領域
        designs, n_cell = [], 5
        base = {"funnel": True, "gate": True, "retreat": 0, "backstop": None, "clearance_mm": 0.1}
        for fillet, plate in itertools.product((0.3, None), (0.002, 0.005, 0.01)):
            d = {**base, "plate": plate}
            if fillet:
                d["rim_fillet_mm"] = fillet
            designs.append(("hood", d, 10.0, True))
    elif args.stage == "chamfer":                    # 段差の縁の面取り・丸め（すき間 0 = 斜面が床から始まる）。t × 面取り角 × 丸み R
        designs, n_cell = [], 3
        for t, ang, R in itertools.product((0.05, 0.1, 0.2, 0.5), (5, 10, 20, 45, 90), (0.0, 0.1, 0.3)):
            designs.append(("hood", {"funnel": True, "gate": True, "retreat": 0, "backstop": None, "clearance_mm": 0.0, "plate": t,
                                     "plate_chamfer_deg": ang, "plate_round_mm": R}, 10.0, True))
    elif args.stage == "chamfer2":                   # 面取りのある段差の、ごく小さい t（0.002〜0.02 mm）
        designs, n_cell = [], 3
        for t, ang in itertools.product((0.002, 0.005, 0.01, 0.02), (10, 45)):
            designs.append(("hood", {"funnel": True, "gate": True, "retreat": 0, "backstop": None, "clearance_mm": 0.0, "plate": t,
                                     "plate_chamfer_deg": ang, "plate_round_mm": 0.1}, 10.0, True))
    elif args.stage == "film":                       # 薄いフィルムの縁（PET / シム。垂直の縁、面取りなし）。t × 床の粗さ × すき間 c
        designs, n_cell = [], 3
        for t, amp, c in itertools.product((0.01, 0.02, 0.03, 0.05), (0.0, 0.03, 0.05, 0.1), (0.1, 0.3)):
            d = {"funnel": True, "gate": True, "retreat": 0, "backstop": None, "clearance_mm": c, "plate": t}
            if amp:
                d["rough_mm"] = amp
            designs.append(("hood", d, 10.0, True))
    elif args.stage == "cloche":                     # フード昇降（平らな床）: 上げる高さ × すき間 c × 落とす速さ × 覆うだけ / 後退で運ぶ / 前進で運ぶ
        designs, n_cell = [], 1
        for lift, c, dv, rt in itertools.product((3.0, 5.0, 8.0, 10.0), (0.0, 0.3, 1.0), (5.0, 20.0), (0, 30, -30)):
            designs.append(("cloche", {"lift_mm": lift, "clearance_mm": c, "drop_speed_mm_s": dv, "retreat": rt, "backstop": None}, 10.0, True))
    elif args.stage == "cloche2":                    # フード昇降 + 絨毯の代用（床の粗さ ±0.1 / 凹凸 ±0.5）
        designs, n_cell = [], 1
        for fl, lift, c, rt in itertools.product(({"rough_mm": 0.1}, {"bump_mm": 0.5}), (3.0, 5.0, 10.0), (0.0, 1.0), (0, 30, -30)):
            designs.append(("cloche", {"lift_mm": lift, "clearance_mm": c, "drop_speed_mm_s": 20.0, "retreat": rt, "backstop": None, **fl}, 10.0, True))
    elif args.stage == "cloche3":                    # 対照: ゲートなしで運ぶ / 落とす力の上限（0.15 N = 自重に近い、〜 5 N）
        designs, n_cell = [], 1
        for gate, rt, c in itertools.product((True, False), (0, 30, -30), (0.0, 1.0)):
            designs.append(("cloche", {"lift_mm": 5.0, "clearance_mm": c, "drop_speed_mm_s": 20.0, "retreat": rt, "gate": gate, "backstop": None}, 10.0, True))
        for fN, dv in itertools.product((0.15, 0.5, 2.8, 5.0), (10.0, 20.0)):
            designs.append(("cloche", {"lift_mm": 5.0, "clearance_mm": 0.0, "drop_speed_mm_s": dv, "retreat": 0, "drop_force_n": fN, "backstop": None}, 10.0, True))
    elif args.stage == "intake_c":                   # J1 の範囲の最適化用: 口の下端のすき間 c（= J1 の角度）× 摩擦の prior の幅 × 床の代用（平ら / 凹凸 ±0.5）
        designs, n_cell = [], 1
        for c, mw, mf in itertools.product((0.0, 0.1, 0.3, 0.6, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0), (0.7, 1.0, 1.4), (0.7, 1.0, 1.4)):
            designs.append(("hood", {"funnel": True, "gate": True, "retreat": 30, "backstop": None, "clearance_mm": c, "mu_wall_scale": mw, "mu_floor_scale": mf}, 10.0, True))
        for c in (0.0, 0.1, 0.3, 0.6, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0):
            designs.append(("hood", {"funnel": True, "gate": True, "retreat": 30, "backstop": None, "clearance_mm": c, "bump_mm": 0.5}, 10.0, True))
    elif args.stage == "cover_intake":              # 上の輪の覆い（襟が前へ 7 mm、約 1.1 g）の有無で、取り込みが変わるか: 頭の質量 80 / 81.1 g × c × 段差 t × 床（平ら 2 種 + 凹凸 ±0.5）
        designs, n_cell = [], 1
        for hm, c, t in itertools.product((80.0, 81.1), (0.0, 0.1, 0.3, 0.6, 1.0), (0.0, 0.002)):
            base = {"funnel": True, "gate": True, "retreat": 30, "backstop": None, "clearance_mm": c, "head_mass_g": hm}
            if t:
                base["plate"] = t
            designs.append(("hood", base, 10.0, True))
            designs.append(("hood", {**base, "bump_mm": 0.5}, 10.0, True))
    elif args.stage == "rough":                      # 床の粗さ（細かい高さ場）× 垂直の段差 0.002〜0.1 mm
        designs, n_cell = [], 2
        for amp, t in itertools.product((0.01, 0.03, 0.1), (0.0, 0.002, 0.005, 0.01, 0.03, 0.1)):
            d = {"funnel": True, "gate": True, "retreat": 0, "backstop": None, "clearance_mm": 0.1, "rough_mm": amp}
            if t:
                d["plate"] = t
            designs.append(("hood", d, 10.0, True))
    elif args.stage == "solver":                     # 段差 0.002 mm の 0% が、接触の設定（solref・時間刻み・solimp・margin）に依存しないか
        designs, n_cell = [], 2
        variants = [{}, {"solref_s": 0.001}, {"solref_s": 0.004}, {"solref_s": 0.01}, {"timestep_s": 1.0e-4}, {"timestep_s": 5.0e-5},
                    {"solimp_width_mm": 0.0001}, {"solimp_width_mm": 0.005}, {"margin_mm": 0.05}, {"margin_mm": 0.2}]
        for v, t in itertools.product(variants, (0.002, 0.1)):
            designs.append(("hood", {"funnel": True, "gate": True, "retreat": 0, "backstop": None, "clearance_mm": 0.1, "plate": t, **v}, 10.0, True))
        for v in ({"solref_s": 0.01}, {"timestep_s": 5.0e-5}):        # 段差なしが、極端な設定でも成り立つか（設定が壊れていない確認）
            designs.append(("hood", {"funnel": True, "gate": True, "retreat": 0, "backstop": None, "clearance_mm": 0.1, **v}, 10.0, True))
    elif args.stage == "realistic":                  # 実物に近い: 面取りあり + 粗さあり（+ 物の縁の丸み 0.3 mm）
        designs, n_cell = [], 2
        for ang, t, amp in itertools.product((10, 20), (0.05, 0.1, 0.2, 0.5), (0.03, 0.1)):
            designs.append(("hood", {"funnel": True, "gate": True, "retreat": 0, "backstop": None, "clearance_mm": 0.0, "plate": t,
                                     "plate_chamfer_deg": ang, "plate_round_mm": 0.1, "rough_mm": amp}, 10.0, True))
        for t, amp in itertools.product((0.05, 0.1, 0.2, 0.5), (0.03, 0.1)):
            designs.append(("hood", {"funnel": True, "gate": True, "retreat": 0, "backstop": None, "clearance_mm": 0.0, "plate": t,
                                     "plate_chamfer_deg": 10, "plate_round_mm": 0.1, "rough_mm": amp, "rim_fillet_mm": 0.3}, 10.0, True))
    elif args.stage == "curtain2":                   # 垂れ布の追加の対照: 力をさらに下げる、垂れ布を短くして物の後ろへ落ちられるようにする
        designs, n_cell = [], 5
        for F, retreat in itertools.product((0.001, 0.003), (0, 30)):
            designs.append(("hood", {"funnel": True, "gate": "curtain", "curtain_f": F, "retreat": retreat, "backstop": None}, 10.0, True))
        for F, retreat in itertools.product((0.001, 0.003, 0.01, 0.03, 0.1), (0, 30)):
            designs.append(("hood", {"funnel": True, "gate": "curtain", "curtain_f": F, "curtain_len_mm": 8.0, "retreat": retreat, "backstop": None}, 10.0, True))
    elif args.stage == "skirt":
        designs, n_cell = [], 5
        for bump in (0.0, 0.5):
            designs.append(("hood", {"funnel": True, "gate": True, "retreat": 0, "backstop": None, "bump_mm": bump}, 10.0, True))
            for h, k in itertools.product((3.0, 6.0), (2.0e-4, 2.0e-3)):
                designs.append(("hood", {"funnel": True, "gate": True, "retreat": 0, "backstop": None, "bump_mm": bump, "skirt_mm": h, "skirt_k": k}, 10.0, True))
    elif args.stage == "gateforce":
        designs, n_cell = [], 5
        for F in (5.0, 2.8, 2.0, 1.0, 0.5, 0.25):
            designs.append(("hood", {"funnel": True, "gate": True, "retreat": 30, "backstop": None, "gate_force_n": F}, 10.0, True))
        designs.append(("hood", {"funnel": True, "gate": True, "retreat": 30, "backstop": None, "gate_force_n": 2.8}, 2.0, True))
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
        # 上位 3 案 = 異なる機構（A〜E）の中で、最良の設計を 1 つずつ。同点の設計が多いので、下の順（保持率の対象物平均 → 最悪の対象物 → 入る率 → 名前）の先頭を使う（同点は報告に書く）
        top3, seen = [], set()
        for x in rank:
            f_, p_, _, _ = parse_tag(x["tag"])
            if f_ != "hood" and f_ not in seen and len(top3) < 3:
                seen.add(f_)
                top3.append(x)
        hood_ref = next(x for x in rank if x["form"] == "hood")                  # 受け身のフード（基準）の最良の設計も同じ N で
        top3 = top3 + [hood_ref]
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
    if args.stage in ("baseline", "tolerance", "tolerance2", "tolerance3", "curtain", "curtain2", "skirt", "gateforce", "chamfer", "chamfer2", "rough", "solver", "realistic", "film", "cloche", "cloche2", "cloche3", "intake_c", "cover_intake"):
        cell_table(rows, args.stage)
    print(f"done in {time.time() - t0:.0f} s", flush=True)
    for x in rank[:25]:
        print(f"{x['success']:.3f} min_obj {x['min_obj']:.2f} enter {x['enter']:.2f} launched {x['launched']:.2f}  {x['tag']}")


if __name__ == "__main__":
    main()
