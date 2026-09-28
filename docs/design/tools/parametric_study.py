"""SD-01 の派生案とパラメータを振って比べる（DESIGN_ESTIMATE / KINEMATIC_SIM）。

- 幾何の数値（質量・J1・視錐台・ライン光の遮り・目の見かけ面積・床の見える範囲）は concept_geometry.py と同じモデルで計算する。
- 「蛇らしさ」「かわいさ」「怖さ」などの点は **Design の経験則（HEURISTIC）** を式にしたもので、人の評価ではない（HUMAN_EVALUATED = 0）。
  式は下の heuristics() に全部書いてある。人の評価が出たら、式ではなく人の値で置き換える。
- Engineering の安全の数値（すき間 8 / 25 mm の規則など）は判定に使うだけで、変えない。

実行:  .venv/Scripts/python.exe docs/design/tools/parametric_study.py
出力:  docs/design/results/parametric_2026-09-29.{md,json}
"""
from __future__ import annotations

import json
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import concept_geometry as cg  # noqa: E402

RESULTS = cg.RESULTS
NECK_W = 68.0   # SD-01 の首幅（Fusion Recovery 版）


def bean(width=100.0, length=78.0, crown=77.0, snout_top=56.0, eye_d=24.0, eye_y=37.0, eye_z=60.0,
         n=3.0, key="SD-01A", name="Bean") -> cg.HeadConcept:
    """H1 Bean を基準に、幅・長さ・頭頂・目を振る。前面 X−236 は固定（センサー面）。"""
    base_x = [-236, -233, -226, -215, -200, -185, -170, -158]
    s = length / 78.0
    xs = [-236 + (x + 236) * s for x in base_x]
    hw0 = [30, 36, 42, 48, 50, 48, 40, 32]
    k = (width / 2) / 50.0
    hw = [min(h * k, width / 2) if i > 0 else 30 * min(k, 1.05) for i, h in enumerate(hw0)]
    zt0 = [56, 60, 67, 74, 77, 76, 72, 66]
    zt = [snout_top, snout_top + 4] + [56 + (z - 56) * (crown - 56) / (77 - 56) for z in zt0[2:]]
    zb = [4, 3, 3, 3, 3, 4, 6, 10]
    ex = -236 + (-212 + 236) * s
    # 目の法線: 基準（y=37）で前 38°・外 46°。目を外へ置くほど横向きになる（1 mm で 3°）
    th = math.radians(46.0 + (eye_y - 37.0) * 3.0)
    normal = cg._unit([-math.cos(th), math.sin(th), 0.30])
    return cg.HeadConcept(key=key, name=name, xs=xs, half_w=hw, z_bot=zb, z_top=zt,
                          eye=cg.Eye((ex, eye_y, eye_z), eye_d, tuple(normal)),
                          n_super=n, lens_x=-234.0)


def eye_protrusion(hc: cg.HeadConcept) -> float:
    """目の外端が、その高さの頭の輪郭からどれだけ横に出るか（mm）。大きいと耳・カエルの目に見える。"""
    ex, ey, ez = hc.eye.center
    w, zb, zt = hc.section(ex)
    h = (zt - zb) / 2; zc = (zt + zb) / 2
    u = min(1.0, abs(ez - zc) / h)
    half = w * (1 - u ** hc.n_super) ** (1 / hc.n_super)
    outer = ey + hc.eye.diameter / 2 * abs(hc.eye.normal[1])
    return round(outer - half, 1)


VARIANTS = {
    "SD-01A Bean (現行の推奨)": bean(),
    "SD-01B 細い Bean (幅 92 = 胴)": bean(width=92, eye_y=34, key="SD-01B"),
    "SD-01C 丸い子ども向け (短く高い頭頂、目 28)": bean(length=70, crown=82, eye_d=28, eye_z=62, n=2.6, key="SD-01C"),
    "SD-01D センサー強調の顔 (平らな顔、目 20)": bean(n=3.6, eye_d=20, eye_y=38, key="SD-01D"),
    "SD-01E A と C の中間 (L74、頭頂 80、目 26、n 2.8)": bean(length=74, crown=80, eye_d=26, eye_z=61, n=2.8, key="SD-01E"),
}

SWEEPS = {
    "head_width_mm": [("92", bean(width=92, eye_y=34)), ("96", bean(width=96, eye_y=35.5)), ("100", bean()), ("104", bean(width=104, eye_y=38.5))],
    "head_length": [("short 70", bean(length=70)), ("medium 78", bean()), ("long 90", bean(length=90))],
    "eye_spacing_y": [("narrow ±32", bean(eye_y=32)), ("medium ±37", bean()), ("wide ±42", bean(eye_y=42))],
}


def floor_band(cam_z: float, tilt_deg: float = cg.CAM_TILT_DEG, vfov: float = cg.CAM_VFOV_DEG) -> tuple[float, float]:
    near = cam_z / math.tan(math.radians(tilt_deg + vfov / 2))
    a = tilt_deg - vfov / 2
    far = cam_z / math.tan(math.radians(a)) if a > 0.5 else float("inf")
    return round(near, 1), round(far, 1)


def head_row(hc: cg.HeadConcept) -> dict:
    m = cg.head_metrics(hc)
    ll = cg.line_light_occlusion(hc, -233.0, 66.0)
    return {
        "width": m["max_width_mm"], "height": m["height_mm"], "length": m["length_mm"],
        "head_neck": round(m["max_width_mm"] / NECK_W, 2), "head_body": m["head_to_body_width"],
        "eye_ratio": m["eye_to_head_height"], "mass_g": m["head_mass_g_ASSUMED"],
        "j1_Nm": m["j1_static_torque_max_Nm_ESTIMATE"], "j1_pct": round(100 * m["j1_torque_fraction_of_software_limit"], 1),
        "fov_clear": not m["fov_self_occlusion"]["occluded"], "line_light_clear": ll["clear"],
        "eye_child_front": m["eye_visibility_mm2"]["child_front_eye_level (1 m, 目の高さ 0.4 m)"],
        "eye_parent_side": m["eye_visibility_mm2"]["parent_side_standing (2 m, 1.5 m)"],
        "eye_top": m["eye_visibility_mm2"]["top_down"],
        "roundness_n": hc.n_super,
        "eye_protrusion_mm": eye_protrusion(hc),
    }


def heuristics(r: dict) -> dict:
    """Design の経験則（HEURISTIC、1〜5）。根拠は各行のコメント。人の評価で置き換える前提。"""
    clamp = lambda v: round(max(1.0, min(5.0, v)), 1)
    # 蛇らしさ: 頭 > 首 がいちばん効く（1.0 で 2、1.5 で 4）。短すぎる頭（L < 72）は -0.5
    snake = 2 + 4 * (r["head_neck"] - 1.0) - (0.5 if r["length"] < 72 else 0) + (0.3 if r["length"] > 85 else 0)
    # かわいさ: 目 / 頭高（0.2 で 2、0.35 で 5）＋ 丸さ（n 小さいほど丸い）
    cute = 2 + 20 * (r["eye_ratio"] - 0.2) + (3.0 - r["roundness_n"]) * 1.0
    # 怖さ（高いほど悪い）: 頭 > 胴 が大きいと毒蛇の幅広の頭を連想（1.1 超で加点）、長い鼻先、角ばり（n 大）
    fear = 1 + max(0.0, r["head_body"] - 1.1) * 8 + (0.7 if r["length"] > 85 else 0) + max(0.0, r["roundness_n"] - 3.0) * 1.2
    # 子どもへの親しみ: かわいさ − 怖さ の混合
    child = 3 + 0.5 * (cute - 3) - 0.6 * (fear - 1)
    # センサーの成立: 視錐台とライン光が両方通れば 5、片方で 3
    sensor = 5 if (r["fov_clear"] and r["line_light_clear"]) else (3 if (r["fov_clear"] or r["line_light_clear"]) else 1)
    # Floor Watch の読みやすさ: 保護者の横・真上からの目の見かけ面積（H0 の横 111 / 真上 159 を 2 点とする）
    read = 2 + 1.5 * math.log2(max(r["eye_parent_side"], 1) / 111) * 0.5 + 1.5 * math.log2(max(r["eye_top"], 1) / 159) * 0.5
    # 目の出っ張り: 0〜6 mm は「ぱっちり」、8 mm を超えると耳・カエルに見える（かわいさ・蛇らしさを減点）
    over = max(0.0, r["eye_protrusion_mm"] - 8.0)
    cute -= 0.3 * over; snake -= 0.2 * over
    # 印刷: 幅 104 以上・長さ 90 以上は分割が増える（A1/K1 Max では問題ないが、背と腹の 2 分割で済むかが効く）
    printab = 4 - (0.5 if r["width"] >= 104 else 0) - (0.5 if r["length"] >= 90 else 0) - (0.5 if r["roundness_n"] < 2.8 else 0)
    return {"snake": clamp(snake), "cute": clamp(cute), "fear(↓)": clamp(fear), "child": clamp(child),
            "sensor": clamp(sensor), "readability": clamp(read), "printability": clamp(printab)}


def camera_rows() -> list[dict]:
    rows = []
    for label, z in (("鼻の中央 Z30（Engineering の初期値）", 30.0), ("少し上 Z34", 34.0), ("少し下 Z26", 26.0)):
        near, far = floor_band(z)
        rows.append({"camera": label, "floor_near_mm": near, "floor_far_mm": far,
                     "note": "近い端はレンズ前 38〜147 mm の位置合わせ目標に入るか" + ("（入る）" if near < 38 else "（38 mm を超える: 目標の手前が見えない）"),
                     "design": {"Z30": "鼻の中央。ボタン鼻が顔の真ん中に来る", "Z34": "鼻が少し上がり、あごが長く見える（大人びる）", "Z26": "鼻が下がり、あごが短く見える（幼く見えるが床の影に近い）"}[f"Z{int(z)}"]})
    return rows


TOF_OPTIONS = [
    {"tof": "鼻孔（前面 Z50、2 つの小窓）", "fov_risk": "低: 前方 ±13.5° が鼻先の上を通る", "face": "蛇の鼻孔の位置。数も 2 で合う", "snake": 4, "cute": 4, "dirt": "上向きの面にほこり", "engineering": "カバー窓のクロストーク（ST の窓条件）"},
    {"tof": "頬（側面、斜め前向き）", "fov_risk": "中: 前方の中心が見えない。左右 2 個が要る", "face": "ピット器官の位置（写実）", "snake": 5, "cute": 3, "dirt": "少ない", "engineering": "個数と I2C アドレス、前方の段差検知には不向き"},
    {"tof": "下の顔に隠す（Z15 前向き）", "fov_risk": "高: 床の反射を拾いやすい。障害物の上端を見落とす", "face": "見えない", "snake": 3, "cute": 3, "dirt": "床のほこりに近い", "engineering": "床反射の誤検知"},
]

GAP_OPTIONS = []
for g in (3, 4, 5, 6):
    GAP_OPTIONS.append({
        "gap_mm": g,
        "finger_rule": "8 mm 未満（指が入らない側）" if g < 8 else "要確認",
        "tolerance_risk": {3: "高: 印刷公差 ±0.3〜0.5 と組付けで擦る恐れ", 4: "中", 5: "低", 6: "低"}[g],
        "seam_visibility": {3: "細い線", 4: "細い線", 5: "溝として見える", 6: "溝がはっきり（鱗の境目に見せられる）"}[g],
        "hair_dirt_trap": {3: "髪が挟まりやすい", 4: "中", 5: "中", 6: "拭きやすいがゴミが入る"}[g],
        "note": "すき間は関節の同心面で一定（2D の形の計算）。安全の判定は Engineering（User Decision 3）",
    })

PLATE_OPTIONS = [
    {"back_plate": "flush（胴の面と同じ、Recovery 版）", "silhouette": 5, "seam": "輪と下の 2 mm の線", "snag": "低", "clean": "良", "print": "背板の下面が平らで印刷しやすい"},
    {"back_plate": "+2 mm（初版）", "silhouette": 2, "seam": "板が載ったように見える", "snag": "中: 縁に指・髪がかかる", "clean": "縁にほこり", "print": "同じ"},
    {"back_plate": "+4 mm", "silhouette": 1, "seam": "甲羅・鎧に見える（亀・アルマジロ）", "snag": "高", "clean": "悪", "print": "同じ"},
    {"back_plate": "recessed −2 mm", "silhouette": 4, "seam": "くぼんだ鱗の目地", "snag": "低", "clean": "溝にほこり", "print": "同じ"},
]


def md_table(rows: list[dict]) -> str:
    keys = list(rows[0].keys())
    out = ["| " + " | ".join(keys) + " |", "|" + "---|" * len(keys)]
    for r in rows:
        out.append("| " + " | ".join(str(r[k]) for k in keys) + " |")
    return "\n".join(out)


def main() -> None:
    res = {"source": "DESIGN_ESTIMATE + HEURISTIC（HUMAN_EVALUATED = 0）", "variants": {}, "sweeps": {}}
    lines = ["# SD-01 パラメータ比較（2026-09-29、DESIGN_ESTIMATE / HEURISTIC）", "",
             "`docs/design/tools/parametric_study.py` の出力。幾何の値は concept_geometry.py と同じモデル。",
             "**点（snake / cute / fear / child / readability / printability）は Design の経験則の式で、人の評価ではない。** 式は parametric_study.py の heuristics()。", ""]
    rows = []
    for name, hc in VARIANTS.items():
        r = head_row(hc); h = heuristics(r)
        res["variants"][name] = {**r, **h}
        rows.append({"variant": name, "W×H×L": f'{r["width"]:.0f}×{r["height"]:.0f}×{r["length"]:.0f}', "頭/首": r["head_neck"], "頭/胴": r["head_body"],
                     "目/頭高": r["eye_ratio"], "目の出 mm": r["eye_protrusion_mm"], "質量 g": r["mass_g"], "J1 %上限": r["j1_pct"], "FOV": "○" if r["fov_clear"] else "×",
                     "ライン光": "○" if r["line_light_clear"] else "×", **h})
    lines += ["## 1. 派生案 SD-01A〜D", "", md_table(rows), ""]
    for sname, items in SWEEPS.items():
        srows = []
        for label, hc in items:
            r = head_row(hc); h = heuristics(r)
            res["sweeps"].setdefault(sname, {})[label] = {**r, **h}
            srows.append({sname: label, "W×H×L": f'{r["width"]:.0f}×{r["height"]:.0f}×{r["length"]:.0f}', "頭/胴": r["head_body"], "質量 g": r["mass_g"],
                          "J1 %上限": r["j1_pct"], "目 正面 mm²": r["eye_child_front"], "目 横 mm²": r["eye_parent_side"], "目 真上 mm²": r["eye_top"], "目の出 mm": r["eye_protrusion_mm"], **h})
        lines += [f"## 2. {sname}", "", md_table(srows), ""]
    cam = camera_rows(); res["camera"] = cam
    lines += ["## 3. カメラの高さ（下向き 25°、垂直視野 42° ASSUMED）", "", md_table([{k: v for k, v in c.items()} for c in cam]), "",
              "カメラの高さは Engineering の初期値（Z30）。Design は変更を求めない。表は見た目との関係を示すだけ。", ""]
    res["tof"] = TOF_OPTIONS
    lines += ["## 4. 前 ToF の位置", "", md_table(TOF_OPTIONS), ""]
    res["knuckle_gap"] = GAP_OPTIONS
    lines += ["## 5. ナックルのすき間（KINEMATIC_SIM、安全の判定ではない）", "", md_table(GAP_OPTIONS), ""]
    res["back_plate"] = PLATE_OPTIONS
    lines += ["## 6. 背板の高さ（silhouette = 横から見た連続感、5 が最良。Fusion の描き出しで確認）", "", md_table(PLATE_OPTIONS), ""]
    (RESULTS / "parametric_2026-09-29.json").write_text(json.dumps(res, ensure_ascii=False, indent=2), encoding="utf-8")
    (RESULTS / "parametric_2026-09-29.md").write_text("\n".join(lines), encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
