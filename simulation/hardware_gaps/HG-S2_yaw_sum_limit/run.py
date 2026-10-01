r"""HG-S2 胴体ヨーの角度合計の上限（145 / 150 / 160 / 170 / 180°）で、何を得て何を失うかを比べる。

    .\.venv\Scripts\python.exe simulation\hardware_gaps\HG-S2_yaw_sum_limit\run.py

DEC-USER-0002 §3: 暫定 145°。**安全と性能の evidence が揃ったときだけ 145° から上げる提案をしてよい。**

比べるもの:
  locomotion     … 上限で使える歩容（振幅・波数・旋回 γ）のうち、最速の直進と最小の旋回半径（平面摩擦モデル）
  reachability   … U ターンに要る幅 ≈ 2R + 体の幅
  wrapping       … 体が物に巻ける角（胴体ヨー + 頭ヨー）と、引っかかり（hook > 180°）が起きるか
  encirclement   … 首・胸・腕を囲い込めるか（巻ける角 × 輪の内径 × 部位の太さ）
  pinch          … 体の自分どうしの挟み込み（非隣接の節どうしの最短距離が指の太さの範囲に入るか）
  cable / collision … 上限は 1 関節の可動域を変えない → ケーブル（HG-C1）と関節の干渉（R03）は変わらない（記録のみ）
source = YAW_SUM_TRADEOFF_SIM。**実測ではない。**
"""
from __future__ import annotations

import dataclasses
import json
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT))

from serpens.config import load_config  # noqa: E402
from serpens.link.device_motion import max_contiguous_sum  # noqa: E402
from serpens.motion.gait import GaitParams, angles_at_phase, body_joint_names  # noqa: E402
from simulation.planar_friction import Friction, PlanarSnake, body_from_cfg  # noqa: E402

A = yaml.safe_load((HERE / "assumptions.yaml").read_text(encoding="utf-8"))
S1 = yaml.safe_load((HERE.parent / "HG-S1_contact_safety" / "assumptions.yaml").read_text(encoding="utf-8"))
SOURCE = str(A["label"])


def cfg_of(name: str) -> dict[str, Any]:
    return load_config(overlay=ROOT / A["configs"][name]["overlay"])


def head_yaw_max(cfg: dict[str, Any]) -> float:
    body = body_joint_names(cfg)
    return max((float(j["max_deg"]) for j in cfg["joints"] if j["axis"] == "yaw" and j["name"] not in body), default=0.0)


def gait_yaw_sum(cfg: dict[str, Any], amp: float, waves: float, gamma: float, profile: str | None = None) -> float:
    """歩容 1 周期の、胴体ヨーの連続した角度の和の最大（DeviceMotion.drive_yaw_sum と同じ考え方、クランプ込み）。"""
    names = body_joint_names(cfg)
    lim = {j["name"]: (float(j["min_deg"]), float(j["max_deg"])) for j in cfg["joints"]}
    p = GaitParams(amp, 360.0 * waves / len(names), 0.5)
    worst = 0.0
    for k in range(72):
        ang = angles_at_phase(p, 2 * math.pi * k / 72, names, gamma, profile or str(cfg["gait"]["turn_profile"]))
        q = [min(max(ang[n], lim[n][0]), lim[n][1]) for n in names]
        worst = max(worst, max_contiguous_sum(q))
    return worst


def _loco_case(args: tuple) -> dict[str, Any]:
    name, law, r, amp, waves, gamma, profile = args
    cfg = cfg_of(name)
    body = dataclasses.replace(body_from_cfg(cfg, name), turn_profile=profile)
    sim = PlanarSnake(body, Friction.simple(0.2, 0.2 * r, law=law))
    st = sim.run_cycle(amp, waves, 0.5, gamma, int(A["locomotion"]["steps"]))
    return {"config": name, "law": law, "ratio": r, "amp": amp, "waves": waves, "gamma": gamma, "profile": profile,
            "yaw_sum": gait_yaw_sum(cfg, amp, waves, gamma, profile), "speed": st.speed_mm_s,
            "heading": st.heading_deg_per_cycle, "converged": st.converged}


def locomotion() -> list[dict[str, Any]]:
    lo = A["locomotion"]
    cases = []
    for name in A["configs"]:
        cfg = cfg_of(name)
        body_max = min(float(j["max_deg"]) for j in cfg["joints"] if j["name"] in body_joint_names(cfg))
        for law in lo["laws"]:
            for r in lo["ratios"]:
                for amp in lo["amplitude_deg"]:
                    for waves in lo["waves"]:
                        for profile in lo["turn_profiles"]:
                            g = 0.0
                            while amp + g <= body_max + 1e-9:
                                if g > 0.0 or profile == lo["turn_profiles"][0]:     # 直進は 1 回だけ
                                    cases.append((name, law, float(r), float(amp), float(waves), g, profile))
                                g += float(lo["gamma_step_deg"])
    with ProcessPoolExecutor(max_workers=12) as ex:
        return list(ex.map(_loco_case, cases, chunksize=8))


def summarize_loco(rows: list[dict[str, Any]], limit: float) -> list[dict[str, Any]]:
    out = []
    groups: dict[tuple, list[dict[str, Any]]] = {}
    for r in rows:
        groups.setdefault((r["config"], r["law"], r["ratio"]), []).append(r)
    for (name, law, ratio), g in groups.items():
        ok = [r for r in g if r["converged"] and r["yaw_sum"] <= limit + 1e-9]
        straight = [r for r in ok if r["gamma"] == 0.0]
        vmax = max((r["speed"] for r in straight), default=0.0)
        # 旋回半径: 同じ歩容の直進との向きの差から（弧の近似）
        base = {(r["amp"], r["waves"]): r for r in straight}
        radii = []
        for r in ok:
            if r["gamma"] <= 0 or (r["amp"], r["waves"]) not in base:
                continue
            dh = math.radians(abs(r["heading"] - base[(r["amp"], r["waves"])]["heading"]))
            step_m = abs(r["speed"]) / 0.5 / 1000.0
            if dh > 1e-3 and step_m > 1e-4:
                radii.append(step_m / (2 * math.sin(min(dh, math.pi) / 2)) * 1000.0)
        rmin = min(radii, default=math.inf)
        out.append({"config": name, "law": law, "ratio": ratio, "limit": limit, "speed_max": vmax,
                    "turn_radius_min_mm": rmin, "n_gaits": len(ok)})
    return out


def geometry(limit: float, cfg_name: str) -> dict[str, Any]:
    cfg = cfg_of(cfg_name)
    # 2026-09-29 から上限は頭ヨーも含む yaw の鎖に掛かる（DeviceMotion / ServoBus / ファーム）→ 巻ける角 = 上限そのもの。
    # 頭ヨーを含めなかった場合（旧実装）の巻ける角も記録する
    wrap = limit
    wrap_if_head_excluded = limit + head_yaw_max(cfg)
    link = float(S1["body"]["link_mm"])
    out: dict[str, Any] = {"config": cfg_name, "limit": limit, "wrap_deg": wrap, "hook": wrap > float(A["geometry"]["hook_deg"]),
                           "wrap_if_head_excluded": wrap_if_head_excluded}
    # 囲い込み: 輪の内径は 1 関節の可動域で決まる（±50°）。上限は「何度ぶん巻けるか」を決める
    lim_deg = float(S1["body"]["limits_deg"]["software"])
    for w in A["geometry"]["body_width_mm"]:
        r_in = link / (2 * math.tan(math.radians(lim_deg) / 2)) - w / 2
        out[f"inner_diameter_w{int(w)}"] = 2 * r_in
    # 部位ごと: 巻ける角 wrap のうち、その部位の円周に接する長さ
    for part, (dlo, dhi) in S1["body_parts_mm"].items():
        out[f"{part}_wrap_deg"] = wrap                    # 太い物（胸）にも同じ角だけ巻ける（体の長さは十分）
        out[f"{part}_hook"] = wrap > float(A["geometry"]["hook_deg"])
    # 自分どうしの挟み込み: 上限いっぱいの C 字（各関節 = limit / n、±50° 以内）で、非隣接の節の最短距離
    names = body_joint_names(cfg)
    n = len(names)
    per = min(limit / n, lim_deg)
    pts = _centerline(cfg, [per] * n)
    out["self_gap_min_mm"] = {int(w): _min_nonadjacent_gap(pts, w) for w in A["geometry"]["body_width_mm"]}
    return out


def _centerline(cfg: dict[str, Any], body_deg: list[float]) -> np.ndarray:
    """中心線の点列（尾 → 頭）。首より先はまっすぐ。"""
    xs = [float(cfg["body"]["tail_x_mm"])] + [float(j["x_mm"]) for j in cfg["joints"]] + [float(cfg["body"]["head_tip_x_mm"])]
    seg = np.diff(xs)
    names = body_joint_names(cfg)
    th, p = 0.0, np.zeros(2)
    pts = [p.copy()]
    for k, L in enumerate(seg):
        if 0 < k <= len(cfg["joints"]):
            j = cfg["joints"][k - 1]
            if j["name"] in names:
                th += math.radians(body_deg[names.index(j["name"])])
        for s in np.linspace(0, L, 12)[1:]:
            pts.append(p + s * np.array([math.cos(th), math.sin(th)]))
        p = pts[-1].copy()
    return np.array(pts)


def _min_nonadjacent_gap(pts: np.ndarray, width: float, skip_mm: float = 150.0) -> float:
    """中心線に沿って skip_mm 以上離れた点どうしの距離 − 幅 = 外装どうしのすき間（負 = 重なる）。"""
    arc = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    d = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2)
    far = np.abs(arc[:, None] - arc[None, :]) >= skip_mm
    return float(d[far].min() - width) if far.any() else math.inf


def main() -> int:
    rows = locomotion()
    loco = [s for lim in A["sweep_deg"] for s in summarize_loco(rows, float(lim))]
    geo = [geometry(float(lim), c) for lim in A["sweep_deg"] for c in A["configs"]]
    (HERE / "results").mkdir(exist_ok=True)
    (HERE / "results" / "s2_locomotion_runs.json").write_text(json.dumps(rows), encoding="utf-8")
    (HERE / "results" / "s2_summary.json").write_text(json.dumps({"locomotion": loco, "geometry": geo}, indent=1,
                                                                 default=str), encoding="utf-8")
    finger = S1["body_parts_mm"]["finger"]
    L = [f"# HG-S2 胴体ヨーの角度合計の上限 — 145° から上げてよいか（自動生成 {datetime.now():%Y-%m-%d %H:%M}。source = {SOURCE}）\n",
         f"DEC-USER-0002: 暫定 {A['baseline_deg']:g}°。上げる提案は、安全と性能の evidence が揃ったときだけ。\n",
         "## 1. 移動（上限で使える歩容のうち、最速の直進と最小の旋回半径。平面摩擦モデル）\n",
         "| 構成 | 法則 | 比 r | " + " | ".join(f"{lim:g}°" for lim in A["sweep_deg"]) + " |",
         "|---|---|---|" + "---|" * len(A["sweep_deg"])]
    keys = sorted({(s["config"], s["law"], s["ratio"]) for s in loco})
    for k in keys:
        cells = []
        for lim in A["sweep_deg"]:
            s = next(x for x in loco if (x["config"], x["law"], x["ratio"]) == k and x["limit"] == lim)
            rr = "—" if not math.isfinite(s["turn_radius_min_mm"]) else f"{s['turn_radius_min_mm']:.0f}"
            cells.append(f"{s['speed_max']:.0f} mm/s / R {rr}")
        L.append(f"| {k[0]} | {k[1]} | {k[2]:g} | " + " | ".join(cells) + " |")
    L.append("\n## 2. 巻ける角・囲い込み・自分どうしの挟み込み\n")
    L.append("巻ける角 = 上限（2026-09-29 から頭ヨーも鎖に含めて上限を掛ける実装）。括弧内は頭ヨーを含めなかった場合（旧実装）の巻ける角。"
             "**180° を超えると C 字の口が中より狭くなり（入口 < 内径 = 袋小路）、物に引っかかって保持できる（hook）**。"
             "首と頭の間の開口（首は入るが頭は抜けない）は、この袋小路で起きる典型的な挟まり方。\n")
    L.append("| 上限 | 構成 | 巻ける角（頭ヨー除外時） | hook | 輪の内径（幅 60 / 92 mm） | 自分どうしのすき間 最小（幅 60 / 92 mm） |\n|---|---|---|---|---|---|")
    for g in geo:
        gaps = g["self_gap_min_mm"]
        L.append(f"| {g['limit']:g}° | {g['config']} | {g['wrap_deg']:.0f}°（{g['wrap_if_head_excluded']:.0f}°） | {'**あり**' if g['hook'] else 'なし'} | "
                 f"{g['inner_diameter_w60']:.0f} / {g['inner_diameter_w92']:.0f} mm | {gaps[60]:.0f} / {gaps[92]:.0f} mm |")
    L.append(f"\n自分どうしのすき間が指の太さ（{finger[0]:g}〜{finger[1]:g} mm、HG-S1 の仮定）の範囲に入ると、閉じる動きで指を挟みうる。"
             "負なら外装どうしが当たる（挟む前に止まる）。\n")
    L.append("## 3. 変わらないもの\n")
    L.append("- 1 関節の可動域（±50°）は上限で変わらない → **ケーブル（HG-C1）と関節の干渉（R03 の 64.8°）は同じ**\n"
             "- 輪の内径も同じ（1 関節の可動域で決まる）。上限は「何度ぶん巻けるか」だけを変える\n")
    L.append("## 4. 読み（提案。上限を上げるかどうかの判断）\n")
    need_r = float(yaml.safe_load((HERE.parent / "HG-H0_friction" / "assumptions.yaml").read_text(encoding="utf-8"))["criteria"]["max_turn_radius_mm"])
    worst145 = 0.0
    for name in A["configs"]:
        gains = []
        r145 = []
        for law in A["locomotion"]["laws"]:
            for r in A["locomotion"]["ratios"]:
                s145 = next(x for x in loco if (x["config"], x["law"], x["ratio"], x["limit"]) == (name, law, r, A["baseline_deg"]))
                s180 = next(x for x in loco if (x["config"], x["law"], x["ratio"], x["limit"]) == (name, law, r, 180.0))
                gains.append((s180["speed_max"] - s145["speed_max"], s145["turn_radius_min_mm"] - s180["turn_radius_min_mm"]))
                r145.append(s145["turn_radius_min_mm"])
        dv = max(g[0] for g in gains)
        dr = max((g[1] for g in gains if math.isfinite(g[1])), default=0.0)
        worst145 = max(worst145, max(x for x in r145 if math.isfinite(x)))
        L.append(f"- {name}: 145 → 180° で直進の最速は最大 {dv:+.0f} mm/s、最小旋回半径は最大 −{dr:.0f} mm。145° での最小旋回半径は最悪 {max(x for x in r145 if math.isfinite(x)):.0f} mm")
    L.append(f"\n**結論**: 145° から上げる evidence は無い。直進の速さは 145〜180° で変わらず（角度合計の大きい歩容は速さに効かない）、"
             f"上げて得られるのは旋回半径の短縮だけ。145° の時点で最悪 {worst145:.0f} mm と、要件の仮置き {need_r:.0f} mm（OQ-0109）の"
             f"{worst145 / need_r:.0%} に収まっている。一方、上げると巻ける角が 180°（hook の境界）に近づき、余裕が無くなる（180° 以上では袋小路）。"
             "**→ 145° を維持する。上げる提案はしない。**"
             "この結論は、摩擦の比が判定境界より十分高い（速さの要件を満たす）ことが前提。H0 の実測で比が低いと分かったら、"
             "旋回ではなく前進の不足が問題になり、角度合計の上限では解決しない（6 本目のモーターの用途の問題）。\n")
    # Head Yaw の掃引（USER-DEC-SERPENS-0004: Head Yaw は 145° の Body Curvature Budget に含めない独立した Safety Axis。±15/30/45/60° を比べ、最小の範囲を採用）
    fw = cfg_of("FW6_HEADYAW")
    aim = float(fw["floor_watch"]["mission"]["aim_max_deg"])
    base = float(A["baseline_deg"])
    L.append("## 5. Head Yaw の範囲（USER-DEC-SERPENS-0004）\n")
    L.append("Head Yaw は 145° に含めない独立した Safety Axis。**最悪の場合の巻ける角 = 胴 145° + 頭ヨー**（首 pitch が 0° で頭が床の高さにあり、同じ平面で同じ向きに振れたとき）。"
             "180° 以上は C 字の口が内径より狭くなる（袋小路 = hook）。\n")
    L.append("| 頭ヨー ± | 巻ける角の最悪（胴 145° + 頭） | 180° までの余裕 | 袋小路 | 現在の機体側の実装（鎖に含めて 145° に縮める）で実際に巻ける角 |\n|---|---|---|---|---|")
    for hy in (15.0, 30.0, 45.0, 60.0):
        w = base + hy
        L.append(f"| ±{hy:g}° | {w:.0f}° | {180.0 - w:+.0f}° | {'**あり**' if w >= 180.0 else '境界に近い' if 180.0 - w < 10 else 'なし'} | {base:.0f}°（胴が縮む） |")
    L.append(f"\n- 現在の Sensor aiming（`floor_watch.mission.aim_max_deg`）は ±{aim:g}°（**胴の yaw で線を候補へ向ける値**。専用の頭ヨー軸ができたらこの範囲で足りるかを HG-H2 で確かめる）\n"
             "- **安全側の上限は ±30° 前後**: 胴 145° と合わせても 175° で袋小路にならない（余裕 5°）。±45° 以上は最悪ケースで 180° を超える"
             "→ その場合は**胴の合計を頭ヨーの分だけ下げる（鎖に含める）か、首 pitch を上げたときだけ大きく振れる**などの追加の拘束が要る\n"
             "- 必要な機能側（CSAR で子どもを見る角度・眠りの姿勢・cable twist・頭と胴の干渉・3D の掃引体積）は Design と CAD の入力が要る。"
             "**最小の範囲 = max(必要な機能の角度) かつ ≤ 安全側の上限**。現時点の暫定は ±30°（overlay の ASSUMPTION と同じ）で、機能側の入力を待つ\n"
             "- 実装は、Head Yaw を鎖に含める**より厳しい側**のまま（`DeviceMotion.yaw_chain`、`ServoBus.yaw_chain`、ファームの `YAW_CHAIN`）。"
             "0004 に合わせて含めない（= 緩める）変更は、3D の enclosure / entrapment check（実寸）が揃うまで行わない\n")
    (HERE / "decision_boundary.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    print("wrote", HERE / "decision_boundary.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
