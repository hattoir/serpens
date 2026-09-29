r"""HG-S1 接触安全の Hardware Gap（SAFETY GAP）: 巻き付き・圧迫・引っ張り・曲げ・踏みつけ・持ち上げを worst-case で見積もる。

    .\.venv\Scripts\python.exe simulation\hardware_gaps\HG-S1_contact_safety\run.py

**危険側に倒した解析（SAFETY_WORST_CASE_MODEL）。実測でも人の評価でもない。安全のしきい値は入れない（OQ-0006）。**

何を出すか:
  1. 横関節の角度合計の上限（PRODUCT.md §4「体が輪を作れない」）をどこに置けるかの**窓**:
       下限 = 歩容と旋回に要る「連続した関節の角度の和」の最大、上限 = 巻き付き（囲む角 ≥ 180°）を起こさない値
  2. 最大に曲げたときの輪の内径と、子どもの体の太さ（首・手首など）の比較（曲率で締め付けられるか）
  3. 外力（引く・曲げる・踏む・持ち上げる）で関節に掛かるトルクと、トルク上限・ストール・脱力時の比較
  4. 機械式トルクリミッター（クラッチ）の滑りトルクの窓
"""
from __future__ import annotations

import json
import math
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT))

from serpens.config import load_config  # noqa: E402
from serpens.motion.gait import GaitParams, angles_at_phase, body_joint_names  # noqa: E402

A = yaml.safe_load((HERE / "assumptions.yaml").read_text(encoding="utf-8"))
SOURCE = str(A["label"])


def b_width() -> list[float]:
    return [float(w) for w in A["body"]["width_mm"]]


def hole_diameter_mm(limit_deg: float, link_mm: float, width_mm: float) -> float:
    """全関節を limit まで同じ向きに曲げたときの、輪の内側の直径（リンクの中点が内側に最も近い）。"""
    r_in = link_mm / (2.0 * math.tan(math.radians(limit_deg) / 2.0))
    return 2.0 * (r_in - width_mm / 2.0)


def max_contiguous_sum(q: np.ndarray) -> float:
    """関節角の列（尾 → 頭）のうち、連続する部分の和の絶対値の最大（= 体がその区間で囲む角）。"""
    best, run_pos, run_neg = 0.0, 0.0, 0.0
    for x in q:
        run_pos = max(0.0, run_pos + x)
        run_neg = min(0.0, run_neg + x)
        best = max(best, run_pos, -run_neg)
    return best


def gait_need(cfg: dict[str, Any], amp: float, waves: float, gamma: float, extra_head_deg: float = 0.0) -> float:
    """歩容 1 周期で「連続した関節の角度の和」が最大いくつになるか（首より頭側の yaw は extra_head_deg で加える）。"""
    names = body_joint_names(cfg)
    n = len(names)
    p = GaitParams(amp, 360.0 * waves / n, 0.5)
    worst = 0.0
    for k in range(int(A["gait"]["steps"])):
        ang = angles_at_phase(p, 2 * math.pi * k / int(A["gait"]["steps"]), names, gamma, A["gait"]["turn_profile"])
        q = np.array([ang[nm] for nm in names])
        # 頭 yaw（照準）が同じ向きに振れる最悪を足す
        worst = max(worst, max_contiguous_sum(np.append(q, math.copysign(extra_head_deg, q[-1] if q[-1] else 1.0))))
    return worst


def run() -> dict[str, Any]:
    b = A["body"]
    lims = b["limits_deg"]
    out: dict[str, Any] = {"holes": [], "enclosure": [], "gait": [], "compression": [], "external": []}
    # 1) 輪の内径
    for lim_name, lim in lims.items():
        for w in b["width_mm"]:
            d = hole_diameter_mm(float(lim), float(b["link_mm"]), float(w))
            fits = {part: ("入る（締め付けられない）" if rng[1] < d else ("境目" if rng[0] < d else "入らない（押し付けられうる）"))
                    for part, rng in A["body_parts_mm"].items()}
            out["holes"].append({"limit": lim_name, "limit_deg": lim, "width_mm": w, "hole_mm": d, **fits})
    # 2) 囲める最大の角と、歩容に要る角
    mission_aim = float(load_config()["floor_watch"]["mission"]["aim_max_deg"])
    for name, c in A["configs"].items():
        cfg = load_config(overlay=ROOT / c["overlay"])
        n_body = len(body_joint_names(cfg))
        head_yaw = [j for j in cfg["joints"] if j["axis"] == "yaw" and j["name"] not in body_joint_names(cfg)]
        head_lim = {k: float(head_yaw[0]["max_deg"] if k == "software" else head_yaw[0]["mechanical_max_deg"]) for k in ("software", "mechanical")} \
            if head_yaw else {"software": 0.0, "mechanical": 0.0}
        for lim_name in ("software", "mechanical"):
            out["enclosure"].append({"config": name, "limit": lim_name, "body_yaw": n_body,
                                     "max_enclosure_deg": n_body * float(lims[lim_name]) + head_lim[lim_name]})
        for amp in A["gait"]["amplitude_deg"]:
            for waves in A["gait"]["waves"]:
                gamma = max(float(lims["software"]) - amp, 0.0)
                extra = mission_aim if head_yaw else 0.0
                out["gait"].append({"config": name, "amp": amp, "waves": waves,
                                    "need_straight_deg": gait_need(cfg, amp, waves, 0.0, extra),
                                    "need_turn_deg": gait_need(cfg, amp, waves, gamma, extra), "gamma": gamma})
    # 3) 圧迫（輪の内径より太い物に押し付けたとき、関節トルクが節の中点に掛ける力）
    t = A["torque"]
    L = float(b["link_mm"]) / 1000.0
    for label, tau in ([(f"上限 {e:+.0%}", float(t["software_limit_nm"]) * (1 + e)) for e in t["cap_error"]]
                       + [("上限が効かない（ストール）", float(t["stall_nm"][1])), ("脱力（バックドライブ）", float(t["backdrive_nm"][1]))]):
        out["compression"].append({"case": label, "torque_nm": tau, "force_per_contact_n_mid": tau / (L / 2),
                                   "force_per_contact_n_20mm": tau / 0.020})
    # 4) 外力
    e = A["external"]
    for case, lo, hi, lever_lo, lever_hi, axis in (
            ("尾・頭をつかんで引く（体が曲がっていると横にてこが出る）", e["pull_n"][0], e["pull_n"][1], 0.05, 0.30, "yaw"),
            ("手で節の先を押して曲げる", e["bend_force_n"][0], e["bend_force_n"][1], L, L, "yaw"),
            ("片足で節を踏む（関節の横に掛かる曲げ）", e["step_n"][0], e["step_n"][1], L / 2, L / 2, "pitch/roll（構造）"),
    ):
        out["external"].append({"case": case, "axis": axis, "torque_lo_nm": lo * lever_lo, "torque_hi_nm": hi * lever_hi})
    for name, c in A["configs"].items():
        cfg = load_config(overlay=ROOT / c["overlay"])
        length = float(cfg["body"]["head_tip_x_mm"]) / 1000.0
        for m in A["external"]["lift_mass_kg"]:
            out["external"].append({"case": f"{name}: 頭をつかんで持ち上げる（首 J1 に体重のモーメント）", "axis": "pitch（J1）",
                                    "torque_lo_nm": m * 9.81 * length * 0.45, "torque_hi_nm": m * 9.81 * length * 0.55})
            break
    return out


def write(out: dict[str, Any]) -> None:
    t = A["torque"]
    cap_lo = float(t["software_limit_nm"]) * (1 + min(t["cap_error"]))
    cap_hi = float(t["software_limit_nm"]) * (1 + max(t["cap_error"]))
    L = [f"# HG-S1 接触安全 — worst-case の見積もり（自動生成 {datetime.now():%Y-%m-%d %H:%M}。source = {SOURCE}）\n",
         "**解析的な worst-case。実測でも人の評価でもない。安全のしきい値（力・圧力）は User が決める（OQ-0006）。**\n",
         "## 1. 最大に曲げたときの輪の内径と、子どもの体の太さ\n",
         "全関節を同じ向きに可動域いっぱいまで曲げた輪の内側の直径。体の部位がこれより細ければ、**曲率で締め付けることはできない**"
         "（輪の中で緩い）。太ければ、関節のトルクで押し付けうる。\n",
         "| 可動域 | 角度 | 関節部の幅 mm | 内径 mm | " + " | ".join(A["body_parts_mm"]) + " |",
         "|---|---|---|---|" + "---|" * len(A["body_parts_mm"])]
    for h in out["holes"]:
        L.append(f"| {h['limit']} | ±{h['limit_deg']:g}° | {h['width_mm']:g} | {h['hole_mm']:.0f} | "
                 + " | ".join(h[p] for p in A["body_parts_mm"]) + " |")
    L.append("\n### 機械ストッパーの角度の上限（外力で押し込まれても、首を締め付けられない角度）\n")
    L.append("内径 ≥ 首の太さ（上限）+ 余裕 になる最大の角度。θ = 2·atan(L / (w + D + 2·余裕))\n")
    L.append("| 関節部の幅 mm | 首の上限 mm | 余裕 5 mm | 余裕 10 mm |\n|---|---|---|---|")
    for w in b_width():
        for dn in (A["body_parts_mm"]["neck"][1], A["body_parts_mm"]["neck"][1] + 10.0):
            cells = [f"{math.degrees(2 * math.atan(float(A['body']['link_mm']) / (w + dn + 2 * m))):.1f}°" for m in (5.0, 10.0)]
            L.append(f"| {w:g} | {dn:g} | " + " | ".join(cells) + " |")
    L.append("\n現在の機械の可動域 ±55°（PROVISIONAL）は、幅 92 mm・首 90 mm では余裕 0。**機械ストッパーは ±52° 前後以下にする必要がある**"
             "（関節部の幅が太いほど厳しい。幅 60 mm なら ±58〜61°）。\n")
    L.append("\n## 2. 囲める角と、角度合計の上限の窓\n")
    L.append("囲める最大の角 = 胴体 yaw の本数 × 可動域（+ 頭 yaw）。**180° を超えると、手首・首などを囲い込める**。\n")
    L.append("| 構成 | 可動域 | 胴体 yaw | 囲める最大の角 |\n|---|---|---|---|")
    for r in out["enclosure"]:
        L.append(f"| {r['config']} | {r['limit']} | {r['body_yaw']} | {r['max_enclosure_deg']:.0f}° |")
    L.append("\n歩容に要る「連続した関節の角度の和」の最大（頭 yaw がある構成は照準 "
             f"±{load_config()['floor_watch']['mission']['aim_max_deg']:g}° を同じ向きに足した最悪）:\n")
    L.append("| 構成 | 振幅 | 波数 | 直進 | 旋回（γ = 50° − 振幅） |\n|---|---|---|---|---|")
    for g in out["gait"]:
        L.append(f"| {g['config']} | {g['amp']:g} | {g['waves']:g} | {g['need_straight_deg']:.0f}° | {g['need_turn_deg']:.0f}° |")
    L.append("\n**読み**:")
    for name in A["configs"]:
        gs = [g for g in out["gait"] if g["config"] == name]
        straight = min(g["need_straight_deg"] for g in gs)
        turn = min(g["need_turn_deg"] for g in gs if g["gamma"] > 0)
        worst = max(max(g["need_straight_deg"], g["need_turn_deg"]) for g in gs)
        L.append(f"- {name}: 直進だけなら角度合計 {straight:.0f}° 以上、旋回も含めると {turn:.0f}° 以上が要る（最も曲げる歩容で {worst:.0f}°）。"
                 f"上限を 180° 未満に置けば囲い込みは起きない → 窓は **{turn:.0f}°〜180°**"
                 + ("" if turn < 180 else "（**窓が無い**: 旋回に 180° 以上要る）"))
    L.append("\n## 3. 押し付ける力（輪の内径より太い物に巻いたとき）\n")
    L.append("関節トルクが両側の節の中点（腕 47.5 mm）で押す力。最悪は関節の近く（20 mm）で押す場合。\n")
    L.append("| 場合 | 関節トルク N·m | 節の中点での力 N | 関節から 20 mm での力 N |\n|---|---|---|---|")
    for c in out["compression"]:
        L.append(f"| {c['case']} | {c['torque_nm']:.2f} | {c['force_per_contact_n_mid']:.1f} | {c['force_per_contact_n_20mm']:.1f} |")
    L.append("\n## 4. 子どもが加える外力 → 関節のトルク\n")
    L.append(f"比較: トルク上限 {cap_lo:.2f}〜{cap_hi:.2f} N·m（誤差 ±40%）、ストール {t['stall_nm'][0]}〜{t['stall_nm'][1]} N·m、脱力時 {t['backdrive_nm'][0]}〜{t['backdrive_nm'][1]} N·m\n")
    L.append("| 外力 | 軸 | 関節のトルク N·m |\n|---|---|---|")
    for x in out["external"]:
        L.append(f"| {x['case']} | {x['axis']} | {x['torque_lo_nm']:.2f}〜{x['torque_hi_nm']:.2f} |")
    L.append("""
**読み**:
- 引く・曲げる外力はトルク上限を 1 桁以上超える → サーボは負けて回される（上限が効いていれば指への力は小さい）。
  そのとき**減速機の歯車に外力がそのまま掛かる**。歯車の破断トルクは未知 → 壊れた歯車・破片が誤飲物になる危険（CAD.md の禁止事項）
- 踏みつけは yaw 軸では受けない（pitch/roll = 構造）。ホーン・ヨーク・外装が受ける
- 頭を持って持ち上げると、首 J1 に体重のモーメント（約 2〜3.5 N·m）が掛かり、上限（0.27〜0.63）もストールも超えうる
  → **J1 は機械のストッパーで荷重を受ける設計が要る**（J1 の可動域 0〜45° の端で構造が受ける）
- 機械式トルクリミッター（クラッチ）の滑りトルクの窓: 下限 = 歩容の最大の必要トルク × 余裕（HG-H1 で μ_横 0.8 のとき約 0.47 N·m → 約 0.6）、
  上限 = 歯車の破断トルク（**未知**）と、子どもに加わる力の上限（しきい値未決定）の小さい方
""")
    (HERE / "decision_boundary.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    (HERE / "results").mkdir(exist_ok=True)
    (HERE / "results" / "s1_safety.json").write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")


def main() -> int:
    out = run()
    write(out)
    for h in out["holes"]:
        print(h["limit"], h["width_mm"], round(h["hole_mm"]), {p: h[p][:3] for p in A["body_parts_mm"]})
    for name in A["configs"]:
        gs = [g for g in out["gait"] if g["config"] == name]
        print(name, "straight min", round(min(g["need_straight_deg"] for g in gs)), "turn min",
              round(min(g["need_turn_deg"] for g in gs if g["gamma"] > 0)), "max", round(max(g["need_turn_deg"] for g in gs)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
