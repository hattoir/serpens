"""試験片 B2 の実物の観察（User が記録した CSV）と、シミュレーション（MUJOCO_SIM。`simulation/results/scoop_forms_*_cells.csv`）の予測を突き合わせる。

    python tools/b2_compare.py ai-shared/b2_results/B2_test_log_YYYYMMDD.csv [--out simulation/results/b2_vs_sim.md]

入力（どちらの形でも読む）:
  - Design の `docs/design/test_piece_b/B2/B2_test_log_template_v2.csv`（priority, trial_id, condition_t_mm, condition_c_mm, plate_T_mm(=c+t), plate_how, object, floor, trial_no, result(...), video_file, notes）
  - `ai-shared/B2_test_log_template.csv`（同じ列 + measured_t_mm, measured_c_mm, hood_sanded, push_speed_mm_s_est, offset_mm）
  result: enter（入る）/ return（戻る）/ underrun（下をくぐる）/ pushed_away（押されて逃げた）/ pinched（挟まった）。空欄は未観察として無視する。

対応（Design の B2）: t = 口の下に敷く板の厚み（垂直な前面）、c = 壁の下端が浮く高さ（床から。パッド）。シミュレーションの `plate` = t、`clearance_mm` = c（漏斗つき・ゲートあり・面取りなし）。
予測 = その条件のシミュレーションの保持率（位置ずれ 0・5 mm）。**「入る」= 保持（入って残る）に対応させる**（「戻る」は入ったが残らない = 失敗）。
シミュレーションに無い t は、最も近い t の結果を使い、`外挿` と印を付ける（垂直の板は t > 0 なら 0%）。**シミュレーションは実物ではない。不一致は「モデルが違う」の材料で、モデルの後付けの調整はしない**（原因をこの表から探して DECISIONS に書く）。
"""
from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "simulation" / "results"
OBJ = {"1yen": "coin_1yen", "coin_1yen": "coin_1yen", "cr2032": "battery_cr2032", "battery_cr2032": "battery_cr2032", "bead8": "bead", "bead": "bead",
       "cube10": "crumb_cube", "crumb_cube": "crumb_cube"}
FLOOR = {"flooring": "flooring", "floor": "flooring", "mat": "mat", "carpet": "carpet"}      # carpet はシミュに無い（予測なしとして表に出す。ENTRY-D-0023）
RESULT = {"enter": "enter", "入る": "enter", "return": "return", "戻る": "return", "underrun": "underrun", "下をくぐる": "underrun",
          "pushed_away": "pushed_away", "押されて逃げた": "pushed_away", "pinched": "pinched", "挟まった": "pinched"}
SIM_OUTCOME = {"success": "enter", "escaped": "return", "pushed_ahead": "pushed_away", "pinched": "pinched", "not_entered": "pushed_away", "knocked_in": "return"}
Z90 = 1.6448536269514722


def wilson(k: int, n: int, z: float = Z90) -> tuple[float, float]:
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    den = 1 + z * z / n
    mid = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (max(0.0, mid - half), min(1.0, mid + half))


def _params(tag: str) -> dict[str, str]:
    return dict(kv.split("=", 1) for kv in tag.split("|")[1].split(","))


def load_sim(res_dir: Path = RES) -> dict[tuple[float, float, str, str], dict[str, int]]:
    """(t, c, 物, 床) → {"n": 回数, "success": 回数, ...の結果別}。漏斗つき・ゲートあり・面取りなし・粗さなし・摩擦は既定・位置ずれ ≤ 5 mm のセルだけ。"""
    out: dict[tuple, dict[str, int]] = defaultdict(lambda: defaultdict(int))
    for f in sorted(res_dir.glob("scoop_forms_*_cells.csv")):
        for r in csv.DictReader(open(f, encoding="utf-8")):
            if not r["tag"].startswith("hood|") or float(r["offset"]) > 5.0:
                continue
            q = _params(r["tag"])
            if q.get("funnel") != "True" or q.get("gate") != "True" or any(k in q for k in (
                    "plate_chamfer_deg", "plate_round_mm", "rough_mm", "bump_mm", "skirt_mm", "curtain_f", "rim_fillet_mm", "mu_wall_scale", "mu_floor_scale",
                    "head_mass_g", "solref_s", "timestep_s", "solimp_width_mm", "margin_mm", "gate_force_n")):
                continue
            key = (float(q.get("plate", 0.0)), float(q.get("clearance_mm", 0.1)), r["obj"], r["floor"])
            d = out[key]
            d["n"] += int(r["n"])
            for k in ("success", "knocked_in", "escaped", "pushed_ahead", "not_entered", "pinched"):
                d[k] += int(r[k])
    return {k: dict(v) for k, v in out.items()}


def predict(sim: dict, t: float, c: float, obj: str, floor: str) -> tuple[dict[str, int] | None, float | None, bool]:
    """(集計, 使った t, 外挿か)。同じ c の中で、最も近い t を使う。"""
    ts = sorted({k[0] for k in sim if k[1] == c and k[2] == obj and k[3] == floor})
    if not ts:
        cs = sorted({k[1] for k in sim if k[2] == obj and k[3] == floor})
        if not cs:
            return None, None, True
        c2 = min(cs, key=lambda x: abs(x - c))
        ts = sorted({k[0] for k in sim if k[1] == c2 and k[2] == obj and k[3] == floor})
        best = min(ts, key=lambda x: abs(x - t))
        return sim[(best, c2, obj, floor)], best, True
    best = min(ts, key=lambda x: abs(x - t))
    return sim[(best, c, obj, floor)], best, abs(best - t) > 1e-9 or False


def read_log(path: Path, skipped: list[tuple[str, str]] | None = None) -> list[dict]:
    """結果が書かれた行を読む。**結果が空の行は未観察として黙って飛ばす**が、結果があるのに読めない行（結果の語・t / c・物・床が未知）は
    `skipped` に (理由, 値) で積む（None なら捨てる。呼び出し側が警告に出す）。"""
    rows = []
    for r in csv.DictReader(open(path, encoding="utf-8-sig")):
        raw = (r.get("result(enter/return/underrun/pushed_away)") or r.get("result(enter/return/underrun/pushed_away/pinched)") or r.get("result") or "").strip()
        res = RESULT.get(raw.lower(), RESULT.get(raw))
        if not raw:
            continue
        if res is None:
            if skipped is not None:
                skipped.append(("結果の語が未知", raw))
            continue
        try:
            t, c = float(r["condition_t_mm"]), float(r["condition_c_mm"])
        except (KeyError, ValueError):
            if skipped is not None:
                skipped.append(("t / c が数でない", f"{r.get('condition_t_mm')}/{r.get('condition_c_mm')}"))
            continue
        obj, floor = OBJ.get(r["object"].strip().lower()), FLOOR.get(r["floor"].strip().lower())
        if obj is None or floor is None:
            if skipped is not None:
                skipped.append(("物が未知" if obj is None else "床が未知", r["object"] if obj is None else r["floor"]))
            continue
        rows.append({"t": t, "c": c, "obj": obj, "floor": floor, "result": res, "video": r.get("video_file", ""), "notes": r.get("notes", "")})
    return rows


def compare(rows: list[dict], sim: dict) -> list[dict]:
    g: dict[tuple, list[str]] = defaultdict(list)
    for r in rows:
        g[(r["t"], r["c"], r["obj"], r["floor"])].append(r["result"])
    out = []
    for (t, c, obj, floor), res in sorted(g.items()):
        n, k = len(res), sum(x == "enter" for x in res)
        pred, t_used, extra = predict(sim, t, c, obj, floor)
        if pred is None:
            out.append({"t": t, "c": c, "obj": obj, "floor": floor, "n_obs": n, "k_obs": k, "pred": None, "verdict": "予測なし"})
            continue
        pr = pred["success"] / pred["n"]
        lo, hi = wilson(k, n)
        verdict = "一致" if lo <= pr <= hi else ("実物が良い（シミュが悲観）" if k / n > pr else "実物が悪い（シミュが楽観）")
        modes = defaultdict(int)
        for kk, v in pred.items():
            if kk in SIM_OUTCOME and v:
                modes[SIM_OUTCOME[kk]] += v
        out.append({"t": t, "c": c, "obj": obj, "floor": floor, "n_obs": n, "k_obs": k, "obs_lo": lo, "obs_hi": hi, "pred": pr, "pred_n": pred["n"], "t_used": t_used,
                    "extrapolated": extra, "verdict": verdict, "obs_modes": {m: res.count(m) for m in set(res)}, "pred_modes": dict(modes)})
    return out


def write_md(rows: list[dict], path: Path, src: str) -> None:
    L = ["# B2 の実物の観察 vs シミュレーション\n", f"入力: `{src}`。**シミュレーションは MUJOCO_SIM（ASSUMED の摩擦・剛体）。実物の観察は User の記録。不一致は後付けで合わせない。**\n",
         "| t [mm] | c [mm] | 物 | 床 | 観察（入る / n） | 観察の 90% 区間 | 予測（保持率）| 判定 | 観察の内訳 | 予測の内訳（シミュ）|\n|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        if r["pred"] is None:
            L.append(f"| {r['t']:g} | {r['c']:g} | {r['obj']} | {r['floor']} | {r['k_obs']}/{r['n_obs']} | — | — | {r['verdict']} | | |")
            continue
        L.append(f"| {r['t']:g} | {r['c']:g} | {r['obj']} | {r['floor']} | {r['k_obs']}/{r['n_obs']} | {r['obs_lo']:.0%}〜{r['obs_hi']:.0%} | {r['pred']:.0%}（{r['pred_n']} 回{'、外挿: t=' + format(r['t_used'], 'g') if r['extrapolated'] else ''}）| **{r['verdict']}** | "
                 f"{', '.join(f'{k} {v}' for k, v in sorted(r['obs_modes'].items()))} | {', '.join(f'{k} {v}' for k, v in sorted(r['pred_modes'].items()))} |")
    n = len([r for r in rows if r["pred"] is not None])
    agree = sum(r["verdict"] == "一致" for r in rows if r["pred"] is not None)
    L.append(f"\n**一致 {agree} / {n} 条件**。不一致の条件は、原因の候補（面取り・粗さ・摩擦・板の厚みの実測 `measured_t_mm`・すき間の実測）を `agent/DECISIONS.md` に書く（モデルの調整は、原因を特定してから別の変更として行う）。")
    path.write_text("\n".join(L) + "\n", encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("log", type=Path)
    ap.add_argument("--out", type=Path, default=RES / "b2_vs_sim.md")
    a = ap.parse_args()
    skipped: list[tuple[str, str]] = []
    rows = read_log(a.log, skipped)
    if skipped:
        from collections import Counter
        print(f"警告: 結果が書かれているのに読めなかった行が {len(skipped)} 行ある: " + "、".join(f"{k[0]}「{k[1]}」×{n}" for k, n in Counter(skipped).items()))
    cmp_ = compare(rows, load_sim())
    write_md(cmp_, a.out, str(a.log))
    print(a.out.read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
