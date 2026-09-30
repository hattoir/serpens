"""コミット済みの集計 CSV（simulation/results/scoop_forms_{screen,combos,stage3}_designs.csv と、前回のくちばしの scoop_beak_*）から、
報告書 simulation/results/scoop_forms_2026-09-30.md の表を作って書く。文章は下の定数。MUJOCO_SIM。実物ではない。

    python tools/scoop_forms_report.py
"""
from __future__ import annotations

import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from scoop_forms_sweep import parse_tag  # noqa: E402

RES = Path(__file__).resolve().parents[1] / "simulation" / "results"
Z90 = 1.6448536269514722
OBJS = ["coin_1yen", "battery_cr2032", "bead", "crumb_cube"]
NAME = {"coin_1yen": "1 円玉", "battery_cr2032": "CR2032", "bead": "ビーズ φ8", "crumb_cube": "立方体 10mm"}
FORM_NAME = {"hood": "受け身のフード（H / G）", "sweeper": "A 縦軸サイドスイーパー", "belt": "B ベルト式ランプ", "brush": "C 回転ブラシ",
             "cup": "D 被せるカップ", "hook": "E 引くフック"}
DRIVES = {"hood": 1, "sweeper": 2, "belt": 2, "brush": 2, "cup": 1, "hook": 3}


def wil(k: int, n: int) -> tuple[float, float]:
    if n == 0:
        return 0.0, 1.0
    p = k / n
    den = 1 + Z90 * Z90 / n
    mid = (p + Z90 * Z90 / (2 * n)) / den
    h = Z90 * math.sqrt(p * (1 - p) / n + Z90 * Z90 / (4 * n * n)) / den
    return max(0.0, mid - h), min(1.0, mid + h)


def load(name: str) -> dict[str, dict[str, dict]]:
    per: dict[str, dict[str, dict]] = defaultdict(dict)
    p = RES / f"scoop_forms_{name}_designs.csv"
    if not p.exists():
        return per
    for r in csv.DictReader(open(p, encoding="utf-8")):
        per[r["tag"]][r["obj"]] = r
    return per


def I(r: dict, k: str) -> int:
    return int(float(r[k]))


def macro(o: dict[str, dict]) -> float:
    return sum(float(o[k]["rate"]) for k in OBJS) / len(OBJS)


def cell(r: dict, ci: bool = False) -> str:
    n, k = I(r, "n"), I(r, "successes")
    if ci:
        lo, hi = wil(k, n)
        return f"{100 * k / n:.0f}% ({k}/{n}, {100 * lo:.0f}–{100 * hi:.0f}%)"
    return f"{100 * k / n:.0f}% ({k}/{n})"


def pooled(designs: dict[str, dict[str, dict]], tags: list[str]) -> tuple[int, int, int, int]:
    n = sum(I(designs[t][o], "n") for t in tags for o in OBJS)
    k = sum(I(designs[t][o], "successes") for t in tags for o in OBJS)
    kn = sum(I(designs[t][o], "n_knocked_in") for t in tags for o in OBJS)
    pin = sum(I(designs[t][o], "n_pinched") for t in tags for o in OBJS)
    return n, k, kn, pin


def short(tag: str) -> str:
    return tag.replace("|", " / ")


def best_per_form(designs) -> dict[str, str]:
    best: dict[str, str] = {}
    for t in sorted(designs, key=lambda t: (-macro(designs[t]), -min(float(designs[t][k]["rate"]) for k in OBJS), t)):
        f = t.split("|")[0]
        best.setdefault(f, t)
    return best



def load_cells() -> list[dict]:
    p = RES / "scoop_forms_baseline_cells.csv"
    return list(csv.DictReader(open(p, encoding="utf-8"))) if p.exists() else []


def T(funnel: bool, gate: bool, retreat: int, v: float, plate=None) -> str:
    """受け身のフードの設計のタグ（tag_of は params を名前順に並べる: backstop, funnel, gate, plate, retreat）。"""
    parts = {"backstop": "None", "funnel": str(funnel), "gate": str(gate), "retreat": str(retreat)}
    if plate is not None:
        parts["plate"] = str(plate)
    body = ",".join(f"{k}={parts[k]}" for k in sorted(parts))
    return f"hood|{body}|v{v:g}|stop"


def baseline_sections(A) -> None:
    cells = load_cells()
    if not cells:
        return
    bd = load("baseline")
    A("## 7. 受け身のフードの内訳（N = 30 の基準。機構なし）\n")
    A("**平均だけでは報告しない**。物（4 種）× 床（2 種）× 頭の前進速度（2 / 10 mm/s）× 位置ずれ（0 / 5 / 10 mm）の 1 マスは 5 回（保持 / 5）。"
      "設計: 機構なし・ゲートあり・閉じ終わり後の後退なし。\n")
    for funnel in (False, True):
        t10, t2 = T(funnel, True, 0, 10.0), T(funnel, True, 0, 2.0)
        A(f"\n### 7.{2 if funnel else 1} {'漏斗の口（60 → 30 mm）' if funnel else '直線の口（30 mm）'}\n")
        A("| 物 | 床 | 2 mm/s ずれ 0 | 2 mm/s ずれ 5 | 2 mm/s ずれ 10 | 10 mm/s ずれ 0 | 10 mm/s ずれ 5 | 10 mm/s ずれ 10 | 計 |\n|---|---|---|---|---|---|---|---|---|")
        for o in OBJS:
            for fl in ("flooring", "mat"):
                row, tot_k, tot_n = [], 0, 0
                for tg in (t2, t10):
                    for off in ("0.0", "5.0", "10.0"):
                        c = next((x for x in cells if x["tag"] == tg and x["obj"] == o and x["floor"] == fl and x["offset"] == off), None)
                        k, n = (int(c["success"]), int(c["n"])) if c else (0, 0)
                        tot_k += k
                        tot_n += n
                        row.append(f"{k}/{n}")
                A(f"| {NAME[o]} | {'フローリング' if fl == 'flooring' else 'マット'} | " + " | ".join(row) + f" | {tot_k}/{tot_n} |")
    A("\n読み: **直線の口**の失敗は、位置ずれ 10 mm の 1 円玉・CR2032（口の幅 30 に対して余裕 5 mm）だけで、**床（フローリング / マット）と頭の速さ（2 / 10 mm/s）を変えても結果は同じ**（各マスが 5/5 か 0/5 にそろう）。**漏斗の口**は、ずれ 10 mm の 1 円玉が速さと床で分かれる（2 mm/s: フローリング 1/5・マット 0/5、10 mm/s: フローリング 3/5・マット 1/5）。CR2032 はほぼ全部入る。マス 5 回なので、この差は目安（N = 5）。\n")

    A("### 7.3 縁と床のすき間（clearance）と、各物の厚み\n")
    A("フードの壁の下端は床から **0.1 mm**（`config/scoop.yaml` の `forms.cavity.clearance_mm`、ASSUMED）浮いている。屋根の下面は床から 15 mm、壁の厚みは 1 mm。\n")
    A("| 物 | 厚み（床から上面まで） | すき間 0.1 mm との比 | 壁の下をくぐれるか |\n|---|---|---|---|")
    thick = {"coin_1yen": 1.5, "battery_cr2032": 3.2, "bead": 8.0, "crumb_cube": 10.0}
    for o in OBJS:
        A(f"| {NAME[o]} | {thick[o]:g} mm | {thick[o] / 0.1:.0f} 倍 | くぐれない（厚みがすき間より大きい） |")
    A("\n**4 種とも、物はすき間より厚い**（15〜100 倍）。したがって物は壁の下へは逃げられず、失敗は「壁の前の縁に**押されて逃げた**」、成功は「壁の間を**またいで入った**」のどちらかになる"
      "（壁の下端の 0.1 mm は、この 4 種の物を通さない）。**絨毯の毛に隙間が埋まる可能性は別**（下の前提）。\n")

    A("### 7.4 「またぐ」の定義と、入り方の分類\n")
    A("- **またぐ** = 頭が前進して、物が**両脇の壁の間（幅 30 mm）へ入り、屋根（下面が床から 15 mm）の下を、壁にも屋根にも触れずに通る**こと。頭のうち物の上を通るのは**屋根だけ**（厚み 1 mm、下面 15 mm。物の高さは最大 10 mm）で、壁の下端は物の**脇**を通る（物の下は通らない）。ゲートは開いたとき屋根の上にあり、物の上を通らない。")
    A("- **ゲートが閉じたときの物の位置** = 合図（物の中心が、外接半径 + 1 mm 以上、口の面から奥へ入った）の瞬間の物の中心。下の表の「合図のときの位置」（口の面からの奥行き x と、中心線からの横ずれ |y|）。")
    A("- **保持** = ゲートが閉じ終わってから **2 秒後**も、物の中心が空間の中（0 ≤ x ≤ 30 mm、|y| ≤ 15 mm、床から 15 mm 以下）にある。")
    A("- **分類**（物の中心が口の面に届く前に、壁の前の縁・漏斗に触れたかで分ける）: **またいで入った**（壁に触れずに入る）/ **壁・漏斗に当たって誘導されて入った**（触れて、入った）/ **押されて逃げた**（入らなかった）。\n")
    A("| 物 | ずれ [mm] | 口 | 回数 | またいで入った | 壁・漏斗に当たって誘導されて入った | 押されて逃げた（入らなかった） | 合図のときの位置 x / \\|y\\| [mm] |\n|---|---|---|---|---|---|---|---|")
    for funnel in (False, True):
        for o in OBJS:
            for off in ("0.0", "5.0", "10.0"):
                cs = [x for x in cells if x["obj"] == o and x["offset"] == off and x["tag"] in (T(funnel, True, 0, 2.0), T(funnel, True, 0, 10.0))]
                n = sum(int(x["n"]) for x in cs)
                nw = sum(int(x["entered_nowall"]) for x in cs)
                ew = sum(int(x["entered_wall"]) for x in cs)
                ent = sum(int(x["entered"]) for x in cs)
                fx = [float(x["fire_x_mean_mm"]) for x in cs if x["fire_x_mean_mm"]]
                fy = [float(x["fire_absy_mean_mm"]) for x in cs if x["fire_absy_mean_mm"]]
                pos = f"{sum(fx) / len(fx):.1f} / {sum(fy) / len(fy):.1f}" if fx else "—"
                A(f"| {NAME[o]} | {off.split('.')[0]} | {'漏斗' if funnel else '直線'} | {n} | {nw} | {ew} | {n - ent} | {pos} |")
    A("")

    A("### 7.5 ゲートの効果（ゲートなし・閉じ終わり後に頭が 30 mm 後退する場合）\n")
    A("ゲートなしの対照: 同じ合図で頭が止まり、ゲートが閉じ終わるのと同じ時間（0.2 s）待って「閉じ終わり」とする。**頭が止まったままなら、ゲートの有無で結果は変わらない**"
      "（物は奥の壁の前に残るだけ）。ゲートが効くのは、**頭が後退して口から離れるとき**（物は床に残り、頭だけが後ろへ下がると口が物から抜ける）。そこで、閉じ終わり後に頭が 30 mm 後退する場合も入れた。\n")
    A("| ゲート | 閉じ終わり後の後退 | 頭 mm/s | 1 円玉 | CR2032 | ビーズ | 立方体 |\n|---|---|---|---|---|---|---|")
    for gate in (True, False):
        for retreat in (0, 30):
            for v in (2.0, 10.0):
                tg = T(False, gate, retreat, v)
                if tg not in bd:
                    continue
                o = bd[tg]
                A(f"| {'あり' if gate else 'なし'} | {retreat} mm | {v:g} | " + " | ".join(cell(o[k]) for k in OBJS) + " |")
    A("\n読み: ゲートの効果は、後退したときにだけ出る（ゲートなし + 後退 30 mm では、入っていた物が口から取り残される = `escaped`）。後退しない場合は、ゲートの有無で保持は変わらない（表のとおり）。\n")

    A("### 7.6 前の押す方式（0%）との違いを生んだ設計変数: 口の床の段差\n")
    A("**違いを生んだ設計変数は、「口の床の段差（前回のスコップの傾斜板の先端の厚み）を 0 にし、床をそのまま空間の床にした」ことの 1 つ。**"
      "対照: 受け身のフードの口に、厚さ t の床の板（垂直な前面の段差）を足すと、押されて逃げる側に戻る（下の表）。\n")
    A("| 口の床の段差 t [mm] | 1 円玉 | CR2032 | ビーズ | 立方体 |\n|---|---|---|---|---|")
    base = T(False, True, 0, 10.0)
    if base in bd:
        A("| 0（フード。板なし） | " + " | ".join(cell(bd[base][k]) for k in OBJS) + " |")
    for pl in (0.1, 0.3, 0.6, 1.0, 1.5):
        tg = T(False, True, 0, 10.0, plate=pl)
        if tg in bd:
            A(f"| {pl:g} | " + " | ".join(cell(bd[tg][k]) for k in OBJS) + " |")
    A("\n理由: 段差があると、頭は物を**押す**しかなく、押された物は頭と同じ速さで運ばれるだけ（構造的。自由に滑る・転がる物は、ゆっくり押す力では段差を越えない）。"
      "段差が 0 なら、物は押されず、頭が物の上を**またぐ**。\n")



def recommend_sections(A, scr, s3, bd) -> None:
    def rate(d, tg):
        o = d.get(tg)
        if not o:
            return None
        return sum(I(o[x], "successes") for x in OBJS), sum(I(o[x], "n") for x in OBJS)

    def pc(x):
        return f"{100 * x[0] / x[1]:.0f}%（{x[0]}/{x[1]}）" if x else "—"

    gc, cc, cc2 = load_cells_of("gateforce"), load_cells_of("curtain"), load_cells_of("curtain2")

    def gsum(cells, par, v, key="success"):
        tg = tag_of_dict("hood", par, v)
        k = [cell_sum(cells, tg, o, key=key) for o in OBJS]
        return sum(x[0] for x in k), sum(x[1] for x in k)

    fun10, fun2 = rate(bd, T(True, True, 0, 10.0)), rate(bd, T(True, True, 0, 2.0))
    g28 = gsum(gc, {"funnel": True, "gate": True, "retreat": 30, "backstop": None, "gate_force_n": 2.8}, 10.0)
    g28v2 = gsum(gc, {"funnel": True, "gate": True, "retreat": 30, "backstop": None, "gate_force_n": 2.8}, 2.0)
    g025 = gsum(gc, {"funnel": True, "gate": True, "retreat": 30, "backstop": None, "gate_force_n": 0.25}, 10.0)
    g5 = gsum(gc, {"funnel": True, "gate": True, "retreat": 30, "backstop": None, "gate_force_n": 5.0}, 10.0)
    c_spec = [gsum(cc, {"funnel": True, "gate": "curtain", "curtain_f": F, "retreat": 0, "backstop": None}, 10.0) for F in (0.1, 0.3, 1.0)]
    c_spec_r = [gsum(cc, {"funnel": True, "gate": "curtain", "curtain_f": F, "retreat": 30, "backstop": None}, 10.0) for F in (0.01, 0.03, 0.1, 0.3, 1.0)]
    c_full = gsum(cc2, {"funnel": True, "gate": "curtain", "curtain_f": 0.003, "retreat": 30, "backstop": None}, 10.0)
    c_short = {F: gsum(cc2, {"funnel": True, "gate": "curtain", "curtain_f": F, "curtain_len_mm": 8.0, "retreat": 30, "backstop": None}, 10.0) for F in (0.001, 0.003, 0.01, 0.03, 0.1)}
    c_short_co = {F: [cell_sum(cc2, tag_of_dict("hood", {"funnel": True, "gate": "curtain", "curtain_f": F, "curtain_len_mm": 8.0, "retreat": 30, "backstop": None}, 10.0), o) for o in ("coin_1yen", "battery_cr2032")] for F in (0.003, 0.01)}
    cup = rate(s3, "cup|ID=40,gap_mm=0.0|v0|stop")

    A("## 8. 推奨（1〜2 案）\n")
    A("**推奨案の前提（守れないなら、推奨は成り立たない）**:\n")
    A("- **絨毯は未対応**（剛体の平らな床だけ）。壁の下端の**すき間 0.1 mm は、絨毯の毛（数 mm〜）に埋まる**可能性が高い。壁の下端が毛に沈む・毛が壁の下から物を押す・物が毛に埋もれて壁の前の縁で止まる、のいずれも未検証。"
      "**推奨案は、毛に埋まらない硬い床（フローリング・マット）が前提**で、絨毯で成り立つとは言えない（床の凹凸 ±0.5 mm の代用では、保持は 93%（凹凸）対 95%（平ら）で差が小さい。§12.3。毛の柔らかさ・厚みは入っていない）。")
    A("- **口の前縁に、床の板・リップ（剛体の段差）を付けない**（`plate` = 0）。0.002 mm の段差でも 0%（§12.1b、確率版 `scoop_forms_tolerance_mc.md`）。段差の許容差は実質ゼロで、面一を印刷の精度で保証できない。すき間 `clearance_mm` は 0〜1 mm では効かない。")
    A("- 物の位置を知る理想センサー（物の中心が奥へ入ったことを検知する）と、頭が物にまっすぐ近づく動きが前提（ASSUMED）。")
    A("- 安全は SAFETY_UNVERIFIED。下の「安全上の懸念」は、暫定しきい値（手・指 5.7 N。PROVISIONAL）との比較であって、確認ではない。\n")
    A("### 推奨 1: 漏斗つきフード + 出口の閉じ（受け身の垂れ布は条件つき / 駆動のゲートは 2.8 N 以下で成立）\n")
    A(f"- **フード本体（機構なし）**: 漏斗つき N = 30 / 対象物で 10 mm/s **{pc(fun10)}**、2 mm/s **{pc(fun2)}**（§7）。")
    A("- **(a) 受け身の TPU 垂れ布（駆動なし）は、依頼の範囲（閉じる力 0.1〜1 N、全高）では成立しない**。"
      f"開く力 0.1 / 0.3 / 1 N は、1 円玉の床の摩擦（約 3 mN）より大きく、物が押されて逃げる（保持 {pc(c_spec[0])} / {pc(c_spec[1])} / {pc(c_spec[2])}）。"
      f"境界は 0.01〜0.03 N（0.01 N で 37%、0.03 N で 6%）。**全高の垂れ布は、入っても物の上に垂れかかったまま落ちず、頭が後退すると物が出ていく**（0%）。"
      "成り立つのは、**8 mm の短い垂れ布 + 閉じる力 3 mN 以下**（`curtain_len_mm`）の 1 円玉・CR2032 だけ"
      f"（F = 0.003 N で 1 円玉 {kn(c_short_co[0.003][0])}・CR2032 {kn(c_short_co[0.003][1])}。後退 30 mm でも保持）。**ビーズは、力 1 mN でも押されて逃げる**（転がるので押す力に抵抗しない）。立方体（10 mm）は短い垂れ布で入らなかった（原因は未確認）。"
      "**条件**: 開く力 ≲ 3 mN（TPU の薄板で実現できるかは未確認）・短い垂れ布・ビーズと立方体は対象外。")
    A(f"- **(b) 駆動のゲートは、2.8 N 以下（暫定しきい値 5.7 N の半分以下）で成立する**。上限を 5 → 0.25 N に下げても保持は変わらない（10 mm/s: 5 N {pc(g5)}、**2.8 N {pc(g28)}**、0.25 N {pc(g025)}。2 mm/s の 2.8 N は {pc(g28v2)}。挟まった 0 回。§12.4）。"
      "ゲートが閉じるのは物が口の面より奥に入ったあとなので、力は要らない。**推奨は (b)**（垂れ布 (a) は条件が厳しい）。")
    A("- **駆動数**: (b) 1（ゲート）/ (a) 0（駆動なし）")
    A("- **印刷しやすさ（見立て）**: フード 易（壁の下端のすき間 0.1 mm は印刷後の平面度に依存。**リップ・板は付けない**）/ (b) ゲート板 易 / (a) 垂れ布は、開く力 3 mN 以下にするには TPU をごく薄く（0.05 mm 級）する必要があり、**難**（Design の試験片 B は 0.4〜0.6 mm）。")
    A("- **安全上の懸念**: (b) ゲートの力は **2.8 N 以下に設定できる**（暫定しきい値 5.7 N の半分以下。余裕 2.9 N）。ただし降りる途中の隙間は 18 → 0 mm で、指が入る 5〜12 mm の禁止帯を通り、口に入った指・ペットのしっぽを板の下端と床が挟む恐れは残る。"
      "対策の候補（未検証）: 力を 1 N 以下・ゆっくり・下端を柔らかいリップに・閉じる前に口の中の異物を検知して止める。(a) 垂れ布は力が小さい（≤ 0.03 N）が、薄い板の縁の**圧力**が暫定の圧力限界（8.2 N/cm²）を超えないかは `flank_v.md` の垂れ布の節で確認（力 1 N・縁 0.4 mm × 30 mm で 8.3 N/cm² と同程度）。挟み込みの力の実測が要る（SAFETY_UNVERIFIED）。\n")
    A("### 推奨 2（条件つき）: 被せるカップ（内径 40 mm）\n")
    A(f"- 結果（40 回 / 対象物: 位置ずれ 0 / 3 / 6 / 10 mm × 床 2 × 5）: **{pc(cup)}**（位置ずれ 10 mm まで全部入る）。内径 30 mm は 6 mm 以上ずれると縁が 1 円玉・CR2032 に乗って挟まることがある。")
    A("- **条件**: カップを下ろす前に、頭の位置を物の真上へ ±10 mm 以内に合わせられること（位置合わせの動きは含めていない = ASSUMED）。" + camera_line())
    A("- **駆動数: 1**（昇降。ゲートなし。位置合わせの動きは別）")
    A("- **印刷しやすさ（見立て）: 易**（カップと屋根。縁と床のすき間 0〜1 mm は保持に効かない。**縁に段差を付けない**）")
    A("- **安全上の懸念**: 上から降りる縁と床の間（隙間 0〜1 mm）が指・しっぽを挟む。降りる力の上限 5 N（ASSUMED）は暫定しきい値 5.7 N に近い → **2.8 N 以下**にする（カップの力の掃引は未実施。ゲートと同様に成り立つ見込み）。縁の下に入った指を検知して止めるのが前提。SAFETY_UNVERIFIED。\n")
    A("**推奨しない**: B（ベルト式ランプ）は壁・脚に当てたときだけ成功し、駆動 2、印刷は難。A（サイドスイーパー）・C（ブラシ）・E（フック）は、受け身のフードより悪い（§3、§9）。スカート（§12.3）は、この 4 種の物・床の凹凸 ±0.5 mm では効果がなく、平らな床では保持が下がる（3 mm で 92%、6 mm で 83%。スカートなし 95%）。\n")


def failure_sections(A, scr) -> None:
    A("## 9. 失敗した理由（機構ごと）\n")
    A("スクリーニングの失敗の内訳（回数。1 設計 × 対象物あたり 12 回（カップは 16 回）の合計）:\n")
    A("| 案 | 物 | 保持 | 叩かれて入った | 押して逃げた | 挟まった | 入らなかった | 逃げた（閉じ後に出た） |\n|---|---|---|---|---|---|---|---|")
    for f in ("hood", "sweeper", "belt", "brush", "cup", "hook"):
        tags = [t for t in scr if t.startswith(f + "|")]
        for o in OBJS:
            n = sum(I(scr[t][o], "n") for t in tags)
            g = lambda k: sum(I(scr[t][o], k) for t in tags)
            A(f"| {FORM_NAME[f]} | {NAME[o]} | {g('successes')}/{n} | {g('n_knocked_in')} | {g('n_pushed_ahead')} | {g('n_pinched')} | {g('n_not_entered')} | {g('n_escaped')} |")
    A("\n**理由（トレースで確認したもの。§9 の表の数字と合わせて読む）**:\n")
    A("- **受け身のフード**: 失敗は、物が口の幅より大きくずれた（1 円玉・CR2032 が 10 mm ずれ）ときに、壁の前の縁に押される場合だけ。漏斗（60 → 30）で減るが、口の壁と物の摩擦（0.3）で完全には消えない。")
    A("- **A サイドスイーパー**: 腕が硬い・速いと物を叩いて飛ばす（CR2032 が最大 1,056 mm/s で口の前へ飛ばされたトレース、1 円玉が 306 mm/s）。飛ばされて入った分は保持に数えない（`knocked_in`）。腕の先端と床の隙間 0.5 mm は、1 円玉（1.5 mm）の縁に当たる高さ。")
    A("- **B ベルト**: ベルトが物を引き上げる力は μ × 垂直抗力。自由な物は床の摩擦の分（数 mN）でしか押されず、物の重さより小さいので持ち上がらない（壁なしで 0/864）。壁・脚に当てて頭が最大 1 N まで押すと、垂直抗力が出て乗る。頭 2 mm/s では押す力が小さく、N = 30 で 70% に落ちる（10 mm/s は 98%）。")
    A("- **C ブラシ**: ハブの下を通れる高さは（中心の高さ − ハブ半径）= R − 1.6 mm ほど（径 12 で 4.4 mm）。ビーズ（8 mm）と立方体（10 mm）はハブの下を通れず、ブラシの前で押されて逃げる（0/288 ずつ）。1 円玉・CR2032 は通るが、フラップが速いと叩いて飛ばす。")
    A("- **E フック**: 位置ずれ 10 mm のとき、フックが物を横に引きずって口の壁の前の縁へ当て、挟まる・叩かれて入る（トレース: 1 円玉が口の面で横ずれ 7.5 mm のまま止まり、ゲートに挟まれた）。ずれ 0・5 mm は全部入る。駆動が 3 つに増える。")
    A("- **D カップ**: 内径 30 mm の余裕は（30 − 20）/ 2 = 5 mm。6 mm 以上ずれると、縁が 1 円玉・CR2032 の上に乗って下りきれない（`pinched`）。内径 40 mm なら 10 mm ずれても入る（位置合わせが ±10 mm に入ることが前提）。\n")
    A("## 10. 限界・ASSUMED（実測されていない）\n")
    A("- 摩擦 μ 0.3（壁・屋根・ゲート・腕・フックと物）、物と床の μ 0.3 / 0.6、質量、ゲートの力 5 N・0.2 s・質量 5 g、壁の下端のすき間 0.1 mm、ベルト（ローラー列で近似。本物のベルトは面の連続性・たるみが違う）、ブラシのフラップの剛性・質量、フック・腕・カップの速度と力、閉じ始めの合図（理想センサー）")
    A("- **絨毯は未対応**。剛体の平らな床だけ。頭の姿勢（ピッチ・ヨー）の揺れ、ヘビの蛇行による頭の動き、摩擦の速度依存、物が床の凹凸・段差に乗ることは入れていない")
    A("- 「飛ばされた」の閾値（350 mm/s）は、前回の数値感度で接触のやわらかさに依存して収束しなかった量。ここでは分類のために使っているだけで、定量には使えない")
    A("- 安全（指・しっぽの挟み込み、閉じ込めた物が出ない、小部品が外れない、ボタン電池を溜める危険）は機構設計と ST 基準の扱い。ここではゲートの力と暫定しきい値の比較を書いただけで、確認ではない")
    A("- 1 設計 × 対象物あたり 12 回（スクリーニング）は少なく、100% の 90% 区間の下限は約 79%。同点の設計は順位をつけられない\n")
    A("## 11. 次に人が決めること\n")
    A("- 絨毯で成り立たせる必要があるか。あるなら、毛に埋まる壁の下端の扱い（スカートの形・すき間）を実物の試験片で先に見る（手で押して、絨毯で 1 円玉が入るか）")
    A("- ゲートの閉じる力・速さの上限（指・しっぽを挟まない設計値）。暫定しきい値（5.7 N）に近い 5 N は、決め直しが要る")
    A("- 推奨 1（機構なしのフード）で進めるか、位置合わせ（±10 mm）を前提に推奨 2（カップ）を進めるか\n")



def load_cells_of(name: str) -> list[dict]:
    p = RES / f"scoop_forms_{name}_cells.csv"
    return list(csv.DictReader(open(p, encoding="utf-8"))) if p.exists() else []


def cell_sum(cells: list[dict], tag: str, obj: str | None = None, max_offset: float = 99.0, key: str = "success") -> tuple[int, int]:
    cs = [x for x in cells if x["tag"] == tag and (obj is None or x["obj"] == obj) and float(x["offset"]) <= max_offset]
    return sum(int(x[key]) for x in cs), sum(int(x["n"]) for x in cs)


def kn(k_n: tuple[int, int]) -> str:
    k, n = k_n
    return f"{100 * k / n:.0f}% ({k}/{n})" if n else "—"


def tag_of_dict(form: str, params: dict, v: float) -> str:
    body = ",".join(f"{k}={params[k]}" for k in sorted(params))
    return f"{form}|{body}|v{v:g}|stop"


def variant_sections(A) -> None:
    A("## 12. 実物の許容差・受け身の垂れ布・スカート・ゲート力（追加の検討）\n")
    # ---- 1. 段差 × すき間 ----
    tc = load_cells_of("tolerance")
    if tc:
        A("### 12.1 口の前縁の段差 × 前縁と床のすき間（許容差の材料）\n")
        A("漏斗つきのフード + ゲート、頭 10 mm/s、N = 30 / 対象物。**段差**は、口の前縁に足した床の板（垂直な前面）の厚み。**すき間**は、壁と板の下端の床からの高さ。"
          "位置ずれ 0・5 mm の分だけで集計（10 mm ずれの 1 円玉・CR2032 は、段差・すき間と関係なく失敗するので除く）。\n")
        A("| すき間 [mm] | 段差 [mm] | " + " | ".join(NAME[o] for o in OBJS) + " | 4 種の計 |\n|---|---|---|---|---|---|---|")
        for c in (0.0, 0.3, 1.0):
            for st in (0.0, 0.1, 0.2, 0.5):
                tg = tag_of_dict("hood", {"funnel": True, "gate": True, "retreat": 0, "backstop": None, "clearance_mm": c, "plate": st}, 10.0)
                cells_o = [cell_sum(tc, tg, o, 5.0) for o in OBJS]
                tot = (sum(x[0] for x in cells_o), sum(x[1] for x in cells_o))
                A(f"| {c:g} | {st:g} | " + " | ".join(kn(x) for x in cells_o) + f" | {kn(tot)} |")
        A("")
        A("読み: 段差が 0 なら、すき間 0〜1 mm でも保持は変わらない（物はすき間より厚いので壁の下をくぐれない）。**段差は 0.1 mm でも 0%**（次の 12.1b で 0.002 mm まで細かくしても 0%）。すき間は 0〜1 mm では効かない。\n")

    # ---- 2. 垂れ布 ----
    cc = load_cells_of("curtain")
    if cc:
        A("### 12.2 ゲートを受け身の TPU 垂れ布にした変種（駆動なし）\n")
        A("口の面の屋根の縁に蝶番で吊るした薄い板（TPU 0.2 mm、質量 約 0.15 g、ASSUMED）。頭の前進で物に押されて奥へ開き、通したあとは自分のばねで閉じる。**外へは開かない**。"
          "「閉じる力」F は下端を 45° 開いたときのばねの力（ばね定数 k = F × 長さ / 45°、**0.1〜1 N は ASSUMED**。開く力も同じばね）。"
          "0.01 / 0.03 N は、成立の境界を見るために足した。漏斗つき、N = 30 / 対象物。後退 30 mm は、閉じ終わり後に頭が下がっても物が残るか（垂れ布が出口を塞ぐか）。\n")
        A("| 閉じる力 F [N] | 頭 mm/s | 後退 | " + " | ".join(NAME[o] for o in OBJS) + " | 4 種の計 |\n|---|---|---|---|---|---|---|---|")
        for v, F, retreat in [(10.0, F, r) for F in (0.01, 0.03, 0.1, 0.3, 1.0) for r in (0, 30)] + [(2.0, F, 30) for F in (0.1, 0.3, 1.0)]:
            tg = tag_of_dict("hood", {"funnel": True, "gate": "curtain", "curtain_f": F, "retreat": retreat, "backstop": None}, v)
            cells_o = [cell_sum(cc, tg, o) for o in OBJS]
            tot = (sum(x[0] for x in cells_o), sum(x[1] for x in cells_o))
            A(f"| {F:g} | {v:g} | {retreat} mm | " + " | ".join(kn(x) for x in cells_o) + f" | {kn(tot)} |")
        A("")

    # ---- 3. スカート + 床の凹凸 ----
    sc = load_cells_of("skirt")
    if sc:
        A("### 12.3 壁の下端の柔らかいスカート（高さ 3 / 6 mm）と、床の凹凸 ±0.5 mm\n")
        A("**絨毯は未対応のまま。「毛に埋まる」の代わりに、床の凹凸 ±0.5 mm（高さ場。相関長 約 8 mm、すべての設計で同じ床）で代用した**（絨毯の毛の柔らかさ・厚みは入っていない。凹凸で壁の下端のすき間が 0.1 mm から変わる影響だけを見る）。"
          "スカート = 壁の下端に外向き 30° に吊った TPU の帯（0.3 mm）。ヒンジ + ばね（k は ASSUMED、2 水準）でたわみ、下端が床に届く。壁の下端は床から skirt の高さに上がる。漏斗つき + ゲート、頭 10 mm/s、N = 30 / 対象物。\n")
        A("| 床 | スカート高さ [mm] | k [N·m/rad] | " + " | ".join(NAME[o] for o in OBJS) + " | 4 種の計 |\n|---|---|---|---|---|---|---|---|")
        for bump in (0.0, 0.5):
            for h, k in [(0.0, None)] + [(h, k) for h in (3.0, 6.0) for k in (2.0e-4, 2.0e-3)]:
                par = {"funnel": True, "gate": True, "retreat": 0, "backstop": None, "bump_mm": bump}
                if h:
                    par.update({"skirt_mm": h, "skirt_k": k})
                tg = tag_of_dict("hood", par, 10.0)
                cells_o = [cell_sum(sc, tg, o) for o in OBJS]
                tot = (sum(x[0] for x in cells_o), sum(x[1] for x in cells_o))
                A(f"| {'平ら' if not bump else '凹凸 ±0.5 mm'} | {h:g} | {'—' if k is None else f'{k:g}'} | " + " | ".join(kn(x) for x in cells_o) + f" | {kn(tot)} |")
        A("")

    # ---- 5. ゲート力 ----
    gc = load_cells_of("gateforce")
    if gc:
        A("### 12.4 ゲート力の上限（5 N の想定をやめ、暫定しきい値 5.7 N の半分 2.8 N 以下で成立するか）\n")
        A("駆動のゲートの力の上限だけを変える（閉じる時間 0.2 s は同じ）。漏斗つき、閉じ終わり後に頭が 30 mm 後退する（ゲートが物を保持していないと物が残る）、N = 30 / 対象物。"
          "「挟まった」= 物がゲートの下端に挟まって閉じ切れなかった回数。\n")
        A("| ゲート力の上限 [N] | 頭 mm/s | " + " | ".join(NAME[o] for o in OBJS) + " | 4 種の計 | 挟まった |\n|---|---|---|---|---|---|---|---|")
        for F, v in [(5.0, 10.0), (2.8, 10.0), (2.0, 10.0), (1.0, 10.0), (0.5, 10.0), (0.25, 10.0), (2.8, 2.0)]:
            tg = tag_of_dict("hood", {"funnel": True, "gate": True, "retreat": 30, "backstop": None, "gate_force_n": F}, v)
            cells_o = [cell_sum(gc, tg, o) for o in OBJS]
            tot = (sum(x[0] for x in cells_o), sum(x[1] for x in cells_o))
            pin = sum(cell_sum(gc, tg, o, key="pinched")[0] for o in OBJS)
            A(f"| {F:g} | {v:g} | " + " | ".join(kn(x) for x in cells_o) + f" | {kn(tot)} | {pin} |")
        A("")



def camera_line() -> str:
    """カップの位置合わせ ±10 mm を、カメラの誤差（VGA / UXGA、距離 100〜300 mm）から逆算する 1 行。FOV 65° は ASSUMED（config/robot.yaml）。"""
    fov = math.radians(65.0)
    def mm_px(d, w):
        return 2 * d * math.tan(fov / 2) / w
    v1, v3 = mm_px(100, 640), mm_px(300, 640)
    u1, u3 = mm_px(100, 1600), mm_px(300, 1600)
    return (f"**カップに要る位置合わせ ±10 mm は、カメラの画素では効かない**: ±10 mm は VGA（{v1:.2f}〜{v3:.2f} mm/px）で {10 / v3:.0f}〜{10 / v1:.0f} 画素、"
            f"UXGA（{u1:.2f}〜{u3:.2f} mm/px）で {10 / u3:.0f}〜{10 / u1:.0f} 画素に当たり、画素の量子化（±0.5 px = ±{0.5 * u1:.2f}〜{0.5 * v3:.1f} mm）は許容の 1/{10 / (0.5 * v3):.0f} 以下。"
            f"**支配するのは取り付け・姿勢の角度誤差**で、距離 300 mm なら ±{math.degrees(math.atan(10 / 300)):.1f}°、100 mm なら ±{math.degrees(math.atan(10 / 100)):.1f}° 以内なら ±10 mm を満たす"
            "（FOV 65°・距離誤差なしは ASSUMED。FOV は定規で実測待ち = OQ-0002）。")


def variant2_sections(A) -> None:
    tc = load_cells_of("tolerance2") + load_cells_of("tolerance3")
    if tc:
        A("### 12.1b 段差の境界を細かく + 縁の丸み（実物の許容差）\n")
        A("1 円玉・CR2032 の縁は実物では丸い。既定（直角）は段差に対して悲観側なので、**縁の丸み 0.3 mm** の場合も見た（`rim_fillet_mm`）。漏斗つき・ゲートあり・すき間 0.1 mm、頭 10 mm/s、N = 30 / 対象物、位置ずれ 0・5 mm のみ。\n")
        A("| 縁の丸み | 段差 [mm] | " + " | ".join(NAME[o] for o in OBJS) + " | 4 種の計 |\n|---|---|---|---|---|---|---|")
        rows = [(0.3, pl) for pl in (0.0, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5)] + [(None, pl) for pl in (0.002, 0.005, 0.01, 0.02, 0.05)]
        for fil, pl in rows:
            par = {"funnel": True, "gate": True, "retreat": 0, "backstop": None, "clearance_mm": 0.1, "plate": pl}
            if fil is not None:
                par["rim_fillet_mm"] = fil
            tg = tag_of_dict("hood", par, 10.0)
            cells_o = [cell_sum(tc, tg, o, 5.0) for o in OBJS]
            tot = (sum(x[0] for x in cells_o), sum(x[1] for x in cells_o))
            A(f"| {'0.3 mm' if fil else '直角'} | {pl:g} | " + " | ".join(kn(x) for x in cells_o) + f" | {kn(tot)} |")
        A("\n読み: **0.002 mm の段差でも 0%**（縁の丸み 0.3 mm でも同じ）。境界は「段差が有る / 無い」で、大きさではない。実物の許容差の材料は `simulation/results/scoop_forms_tolerance_mc.md`（段差 t ~ U(0, 0.3) mm、すき間 c ~ U(0, 0.5) mm の確率版。期待値 0〜0.7%）。\n")
    cc = load_cells_of("curtain2")
    if cc:
        A("### 12.2b 垂れ布の追加の対照（開く力をさらに下げる / 垂れ布を短くする）\n")
        A("**診断（トレース）**: 全高（15 mm）の垂れ布は、物の前の縁が口の面の 1 mm 奥にあるため、**物の上に垂れかかったまま（CR2032 で 38° 開いたまま）落ちない**。後退すると物は垂れ布の下をくぐって出る（後退 30 mm で 0%）。"
          "垂れ布が物の後ろへ落ちるには、物の前の縁が垂れ布の長さより奥にある必要がある（30 mm の空間に 1 円玉 φ20 が入ると、前の縁は最大でも口から 10 mm）。"
          "そこで、**8 mm の短い垂れ布**（蝶番を壁の途中の高さ 8.1 mm に付ける）と、頭を物の前の縁が 9 mm 以上奥へ入るまで進める合図に変えた変種を足した。開く力は、物の床の摩擦（1 円玉 約 3 mN）より小さくないと物が押されて逃げるので、**1〜3 mN** も試した。"
          "**これは依頼の仕様（全高の垂れ布・0.1〜1 N）からの変更で、原因の切り分けのための追加の対照**。\n")
        A("| 垂れ布 | 閉じる力 F [N] | 後退 | " + " | ".join(NAME[o] for o in OBJS) + " | 4 種の計 |\n|---|---|---|---|---|---|---|---|")
        rows = [("全高 15 mm", None, F, r) for F in (0.001, 0.003) for r in (0, 30)] + [("短い 8 mm", 8.0, F, r) for F in (0.001, 0.003, 0.01, 0.03, 0.1) for r in (0, 30)]
        for lab, ln, F, r in rows:
            par = {"funnel": True, "gate": "curtain", "curtain_f": F, "retreat": r, "backstop": None}
            if ln:
                par["curtain_len_mm"] = ln
            tg = tag_of_dict("hood", par, 10.0)
            cells_o = [cell_sum(cc, tg, o) for o in OBJS]
            tot = (sum(x[0] for x in cells_o), sum(x[1] for x in cells_o))
            A(f"| {lab} | {F:g} | {r} mm | " + " | ".join(kn(x) for x in cells_o) + f" | {kn(tot)} |")
        A("")


def main() -> None:
    scr, com, s3 = load("screen"), load("combos"), load("stage3")
    allg = {**scr, **com}
    best = best_per_form(scr)
    beak3 = {}
    bp = RES / "scoop_beak_stage3_designs.csv"
    if bp.exists():
        for r in csv.DictReader(open(bp, encoding="utf-8")):
            beak3.setdefault(r["tag"], {})[r["obj"]] = r
    L: list[str] = []
    A = L.append
    A("# 口に入れやすい形と機構の探索（2026-09-30）\n")
    A("source = **MUJOCO_SIM**（`simulation/scoop/forms/`、`config/scoop.yaml` の `forms`）。**実物ではない。HARDWARE_VERIFIED ではない。** "
      "摩擦・質量・トルク・速度・センサーはすべて ASSUMED（理想センサー）。実現性の調査であり、製品仕様の確定ではない。絨毯は未対応（剛体平面のみ）。\n")
    A("前回（`scoop_beak_2026-09-30.md`）: ちょうつがい式の腕（くちばし）は、1 円玉・立方体 0/1,728、CR2032 7/1,728、ビーズだけ 70%（叩かれて飛んだ分を含む）。"
      "原因は構造的（押して入れる方式は物が頭と同じ速さで運ばれるだけ。腕は床に近いのが真下の 1 点だけで、その前の物には上から当たる）。今回は形と機構を変えた。\n")

    # ---- §0 用語の定義 ----
    A("## 0. 用語の定義（段差 `plate` と すき間 `clearance_mm`。Design の試験片 B との対応）\n")
    A("この報告書の「段差」と「すき間」は、次の 2 つの**独立な**設計変数（`config` のパラメータ名）。断面（横から見た図。x = 奥へ、z = 上）:\n")
    A("```")
    A("                口の面 x = 0                                   奥の壁 x = 30")
    A("                    |                                              |")
    A("   屋根 ============|==============================================|====  下面 z = 15")
    A("                    |                                              |")
    A("   前（物が来る） ->  |            空間（30 x 30 x 15 mm）            |")
    A("                    |                                              |")
    A("               +----+----------------------------------------------+---  上面 z = clearance_mm + plate")
    A("   段差の面 -> |          床の板（plate。0 なら板は無い）             |      板の前面は垂直（口の面 x = 0 に立つ）")
    A("               +----+----------------------------------------------+---  下面 z = clearance_mm")
    A("        ^ clearance_mm（壁・板の下端の、床からのすき間。壁も板も同じ値）")
    A("   ======================== 床 z = 0 ================================")
    A("```\n")
    A("- **`plate`（段差）= 口の下の床の板の厚み [mm]**。物は床の上を来て、板の前面（高さ `clearance_mm` 〜 `clearance_mm + plate`）に当たる。**0 = 板なし**（口の面には左右の壁だけ。物は床の上をそのまま入る = 面一）。")
    A("- **`clearance_mm`（すき間）= 壁の下端（と、板があるときは板の下面）の床からの高さ [mm]**。既定 0.1。物はこのすき間より厚い（1 円玉 1.5 mm 以上）ので、すき間の下はくぐれない。")
    A("- 2 つは**独立**（板なしでも、すき間は壁の下端に効く）。段差 0.1 mm でも保持が 0% になり、すき間は 0〜1 mm でほぼ効かない（§12.1）。")
    A("- **Design の試験片 B（`docs/design/test_piece_b/`）との対応**: Design は「前縁の段差 0 / 0.3 / 1.0 mm」を**リップ板の下面が床から浮く高さ**と解釈した（= ここの `clearance_mm` に近い）。"
      "リップ板そのものの厚み（= ここの `plate`。B1_LIP は厚さ 1.2、前縁は 12° のランプで先端厚 0.6）は固定。**この 2 つを独立に変えられる形（`plate` = リップの厚み、`clearance_mm` = 浮く高さ）に直す**よう、Design に依頼済み。"
      "シミュレーションの予測: `plate` が 0 でない（リップがある）と、`clearance_mm` によらず押されて逃げる。\n")

    # ---- 結論 ----
    bd0 = load("baseline")

    def agg(tag):
        o = bd0.get(tag)
        if not o:
            return None
        return sum(I(o[x], "successes") for x in OBJS), sum(I(o[x], "n") for x in OBJS)

    def pcs(x):
        return f"{100 * x[0] / x[1]:.0f}%（{x[0]}/{x[1]}）" if x else "—"

    A("## 結論（先に）\n")
    plain10, plain2 = agg(T(False, True, 0, 10.0)), agg(T(False, True, 0, 2.0))
    fun10, fun2 = agg(T(True, True, 0, 10.0)), agg(T(True, True, 0, 2.0))
    A("- **口の形を「開放底のフード + ゲート」に変えるだけで、機構なしで 1 円玉・CR2032・立方体・ビーズを取り込める**（前回のくちばしは 0〜ビーズのみ）。"
      f"受け身のフード（機構なし、駆動はゲートだけ）の保持（N = 30 / 対象物、内訳は §7）: 直線の口 10 mm/s {pcs(plain10)}・2 mm/s {pcs(plain2)}、漏斗の口（60 → 30）10 mm/s {pcs(fun10)}・2 mm/s {pcs(fun2)}。"
      "**失敗は、位置ずれ 10 mm の 1 円玉・CR2032 だけ**（口の幅 30 に対して余裕が 5 mm しかなく、壁の前の縁に押し出される）。床（フローリング / マット）と頭の速さでは、直線の口の結果は変わらない。")
    A("- **前の押す方式（0%）との違いを生んだ設計変数は、「口の床の段差（傾斜板の先端の厚み）を 0 にし、床をそのまま空間の床にした」ことの 1 つ**。"
      "口に 0.1 mm の床の板（段差）を足しただけで、全対象物が 0% に戻る（§7.6）。段差が無いと、物は押されず、頭が物を**またぐ**。")
    A("- **ゲートの効果は、頭が後退して口から離れるときにだけ出る**（ゲートなし + 後退 30 mm は 0%、ゲートありは変わらない。§7.5）。頭が止まったままなら、ゲートの有無で結果は変わらない。")
    A("- **実物の許容差**: 口の前縁の段差は、**0.002 mm でも 0%**（段差が有る / 無いで決まり、大きさは効かない）。段差 t ~ U(0, 0.3) mm・すき間 c ~ U(0, 0.5) mm の確率版で、**保持の期待値は 0〜0.7%（下限 ≈ 0%）**（`scoop_forms_tolerance_mc.md`）。すき間は 0〜1 mm では効かない。**口の前縁に床の板・リップ（剛体の段差）を付けない**ことが前提。")
    A("- **ゲートを受け身の TPU 垂れ布にした変種は、依頼の範囲（0.1〜1 N）では成立しない**（0%）。境界は 0.01〜0.03 N で、成り立つのは 8 mm の短い垂れ布 + 3 mN 以下の 1 円玉・CR2032 だけ（ビーズ・立方体は入らない）。**駆動のゲートは 2.8 N 以下でも成立する**（0.25 N まで 95%）。")
    A("- 壁の下端のスカート（3 / 6 mm）は、床の凹凸 ±0.5 mm（**絨毯の代用**）でも効果が見えず、平らな床では保持が下がる。カップの位置合わせ ±10 mm は、カメラの画素では効かず、**取り付け角の誤差（300 mm で ±1.9°）が支配する**（§8）。")
    A("- **機構を足して成功が増えるのは条件つき**: B（ベルト式ランプ）は**壁・脚に当てたときだけ**成功し（当てないと保持 0/864）、D（カップ）は内径 40 mm なら位置ずれ 10 mm まで全部入る"
      "（内径 30 mm は 6 mm 以上ずれると縁が物に乗って挟まることがある）。A（サイドスイーパー）・C（ブラシ）・E（フック）は、受け身のフードより悪い。")
    A("- 「叩かれて飛び込んだ」は保持に数えず、別集計（`knocked_in`）にした。A と E で目立つ。")
    A("- **推奨は §8**。ただし、開放底のフードは剛体の平らな床の理想化で、**絨毯では壁の下端のすき間 0.1 mm が毛に埋まり、前提が崩れる可能性が高い**（未対応）。\n")

    # ---- 前提・定義 ----
    A("## 1. 前提と定義\n")
    A("- **口 = 開放底のフード**: 屋根・両脇の壁・奥の壁で 30 × 30 × 15 mm の空間を作り、**床がそのまま空間の床になる**（壁の下端は床から 0.1 mm 浮く、ASSUMED）。前回のスコップの傾斜板と、先端の厚みの制約が無い。"
      "頭が前進して物に被さり、物の中心が空間の奥へ入ったらゲート（口の面に立つ板。上から降り、閉じたら開かない = ラッチ）を閉じる。")
    A("- **乗る（ride）**: 物の**傾斜板に向かう頭側（+x 側）の縁**が床から 1mm 以上持ち上がった瞬間が一度でもあった（前回から変えていない）。床が空間の床の案では起きない（N/A）。乗るが起きるのはベルト（B）だけ。")
    A("- **入る（enter）**: 物の中心が空間の範囲に入った瞬間が一度でもあった。")
    A("- **保持（success）**: ゲートが閉じ終わり、**閉じ終わりから 2 秒後まで空間の中にあり、逃げず、飛ばされていない**。飛ばされて入った分は `knocked_in` として別集計（保持に数えない）。")
    A("- **飛ばされた（launched）**: 物の速さが一度でも閾値を超えた（350 mm/s。機構が速い案は、機構の最大速度の 1.5 倍）。")
    A("- **逃げた距離（escape）**: 判定時の物の中心の、空間の範囲からの距離。")
    A("- 物: 1 円玉 / CR2032 / ビーズ φ8 / 立方体 10mm。床: フローリング / マット（**絨毯は未対応**）。位置ずれ 0 / 5 / 10 mm（カップは 0 / 3 / 6 / 10）、頭の前進 2 / 10 mm/s、空間 30 × 30 × 15 mm、ラッチで閉じる。")
    A("- **吸引は対象外**（音と消費電力のため）。\n")

    # ---- 案の一覧 ----
    A("## 2. 案の一覧（駆動数・印刷しやすさ・絨毯での懸念）\n")
    A("印刷しやすさ・絨毯での懸念は Engineering の**見立て（未検証）**。駆動数はゲートを含む（SG90 相当 1 個を上限の目安。超える案は「駆動数が増える」）。\n")
    A("| 案 | 内容 | 駆動数 | 印刷しやすさ（見立て） | 絨毯での懸念（見立て・未検証） |\n|---|---|---|---|---|")
    A("| 受け身のフード（H / G） | 機構なし。漏斗（G）= 口が 60 → 奥で 30 | 1（ゲート） | 易。壁の下端の隙間 0.1 mm が印刷精度に依存 | 壁の下端が毛に沈む・毛が壁の下から物を押す・物が毛に埋もれて壁の前で止まる |")
    A("| A 縦軸サイドスイーパー | 口の左右の角に縦軸の腕 2 本。V に開き、閉じながら物を中央・奥へ寄せる | 2（腕 1 + ゲート。左右はリンク）。**増える** | 中。縦軸のヒンジとリンク。柔らかい先端は TPU | 腕の下端 0.5 mm が毛に引っかかる・毛を掻く |")
    A("| B ベルト式ランプ | ランプ先端の φ3 / 4 ローラーと、奥へ動くベルト（ここではローラー列で近似） | 2（ベルト + ゲート。全ローラーを 1 つのモーターで）。**増える** | 難。φ3〜4 mm のローラーと薄いベルトは市販部品 | ローラーが毛を巻き込む・毛で物が浮いて μ が変わる |")
    A("| C 回転ブラシ | 口の縁の下部、床側が奥へ動く向きに回る | 2（ブラシ + ゲート）。**増える** | 中。ハブ + 薄い柔らかいフラップ（TPU 0.4 mm） | 毛を巻き込む。毛の中の物は取り込みやすくなる可能性もある |")
    A("| D 被せるカップ | 上から頭のカップを物に被せ、縁を床に付けて閉じ込める | 1（昇降。ゲートなし） | 易。縁の平らさと隙間 | 縁が毛に沈む・隙間が毛で埋まる。毛の中の物を上から覆えるので有利な可能性もある |")
    A("| E 引くフック | 頭を上げて物を越え、後ろに下ろして手前へ引き込む | 3（前後 + 上下 + ゲート）。**増える** | 中。薄い板とレール | フックが毛に食い込んで物の向こう側へ下ろせない・毛が物を止める |")
    A("| F 壁・脚に押し付ける | 物を壁や脚（φ30）に当てて止め、その間に取り込む（A / B / C との組み合わせ） | 0（環境の条件） | — | 壁際・脚の際は毛の縁でも起きる |")
    A("| G 漏斗の口 | 口が 60 → 奥で 30 に狭まる | 0（形） | 易 | 受け身のフードと同じ |\n")

    # ---- 比較表 ----
    A("## 3. 比較表（スクリーニング。1 設計 × 対象物あたり 12 回（カップは 16 回）、Wilson 区間は上位のみ）\n")
    A("前回のちょうつがい式の腕（くちばし）を基準に、各案の最良の設計。**保持 = 閉じ終わりから 2 秒後も中にいて、飛ばされていない**。\n")
    A("| 案（最良の設計） | 駆動数 | 1 円玉 | CR2032 | ビーズ | 立方体 | 平均 | 叩かれて入った（knocked_in） |\n|---|---|---|---|---|---|---|---|")
    if beak3:
        bt = sorted(beak3, key=lambda t: -sum(I(beak3[t][o], "n_success_clean") / I(beak3[t][o], "n") for o in OBJS))[0]
        cells = []
        tot = 0
        for o in OBJS:
            r = beak3[bt][o]
            n, k = I(r, "n"), I(r, "n_success_clean")
            cells.append(f"{100 * k / n:.0f}% ({k}/{n})")
            tot += I(r, "successes") - k
        avg = sum(I(beak3[bt][o], "n_success_clean") / I(beak3[bt][o], "n") for o in OBJS) / 4
        A(f"| **前回: ちょうつがい式の腕（くちばし）**（`{bt.replace('beak|', '')}`、N = 30。飛ばされた分を除く） | 2 | " + " | ".join(cells) + f" | {100 * avg:.0f}% | {tot}（保持に数えていない） |")
    bd1 = load("baseline")
    for lab, tg in (("受け身のフード（機構なし・直線の口・ゲートあり）N = 30、頭 10 mm/s", T(False, True, 0, 10.0)),
                    ("受け身のフード + 漏斗（G）N = 30、頭 10 mm/s", T(True, True, 0, 10.0)),
                    ("受け身のフード + 漏斗（G）N = 30、頭 2 mm/s", T(True, True, 0, 2.0)),
                    ("ゲートなしのフード（直線の口）N = 30、頭 10 mm/s", T(False, False, 0, 10.0))):
        if tg in bd1:
            o = bd1[tg]
            A(f"| {lab} | {'0' if 'ゲートなし' in lab else '1'} | " + " | ".join(cell(o[k]) for k in OBJS) + f" | {100 * macro(o):.0f}% | {sum(I(o[k], 'n_knocked_in') for k in OBJS)} |")
    for f in ("sweeper", "belt", "brush", "cup", "hook"):
        t = best.get(f)
        if not t:
            continue
        o = scr[t]
        A(f"| {FORM_NAME[f]}（`{short(t)}`） | {DRIVES[f]} | " + " | ".join(cell(o[k]) for k in OBJS) + f" | {100 * macro(o):.0f}% | {sum(I(o[k], 'n_knocked_in') for k in OBJS)} |")
    bn = [t for t in scr if t.startswith("belt|") and "backstop=None" in t]
    if bn:
        tb = sorted(bn, key=lambda t: -macro(scr[t]))[0]
        o = scr[tb]
        A(f"| B ベルト（壁・脚なし）の最良（`{short(tb)}`） | 2 | " + " | ".join(cell(o[k]) for k in OBJS) + f" | {100 * macro(o):.0f}% | {sum(I(o[k], 'n_knocked_in') for k in OBJS)} |")
    A("")
    hb = best.get("hood")
    if hb:
        o = scr[hb]
        A(f"\n参考: スクリーニングの受け身のフードの最良は壁・脚（F）つき（`{short(hb)}`、N = 12 / 対象物）で {100 * macro(o):.0f}%。壁・脚は物を止めるので、機構なしのフードでも効く。")
    ties = [t for t in scr if t.startswith("belt|") and macro(scr[t]) == 1.0]
    tie_c = [t for t in scr if t.startswith("cup|") and macro(scr[t]) == 1.0]
    A(f"- **同点が多い**: 保持 100%（全対象物・全条件）の設計は、B で {len(ties)} 設計、D で {len(tie_c)} 設計。1 設計 × 対象物あたり 12 回なので、100% の 90% 区間は下限 79% 程度。同点の中では順位をつけられない（下の N = 30 でも先頭の設計を使った）。\n")

    # ---- パラメータの効き ----
    A("## 4. 案ごとのパラメータの効き（スクリーニング、他のパラメータを合算）\n")
    A("保持の回数 / 回数（対象物 4 種を合算）。\n")
    groups = {"hood": (("funnel", "漏斗（G）"), ("backstop", "壁・脚（F）"), ("speed", "頭の前進 mm/s")),
              "sweeper": (("L", "腕長 mm"), ("S", "掃引角 °"), ("T", "時間 s"), ("soft", "柔らかい先端")),
              "belt": (("backstop", "壁・脚（F）"), ("mu", "ベルトと物の μ"), ("vb", "ベルト速度 mm/s"), ("d", "ローラー径 mm")),
              "brush": (("D", "径 mm"), ("rpm", "回転 rpm"), ("stiff", "フラップ剛性"), ("backstop", "壁・脚（F）")),
              "cup": (("ID", "内径 mm"), ("gap_mm", "縁と床の隙間 mm")),
              "hook": (("tip", "先端"), ("h", "高さ mm"), ("v", "頭の速度 mm/s"))}
    for f, facs in groups.items():
        tags = [t for t in scr if t.startswith(f + "|")]
        A(f"\n### {FORM_NAME[f]}\n")
        for key, label in facs:
            lv: dict[str, list[str]] = defaultdict(list)
            for t in tags:
                _, p, v, st = parse_tag(t)
                val = v if key == "v" else p.get(key)
                lv[str(val)].append(t)
            cells = []
            for val in sorted(lv, key=lambda x: (x == "None", float(x) if x.replace(".", "", 1).replace("-", "", 1).isdigit() else 0, x)):
                n, k, kn, pin = pooled(scr, lv[val])
                cells.append(f"{val}: {k}/{n}（{100 * k / n:.0f}%、knocked_in {kn}、pinched {pin}）")
            A(f"- {label} — " + " / ".join(cells))
    A("")

    # ---- F・G ----
    if com:
        A("## 5. F（壁・脚）と G（漏斗）の効果（A / B / C の上位設計 × 漏斗 × 壁・脚 × 前進 2 / 10 mm/s）\n")
        A("| 案 | 漏斗 | 壁・脚 | 頭 mm/s | 1 円玉 | CR2032 | ビーズ | 立方体 | 平均 |\n|---|---|---|---|---|---|---|---|---|")
        for t in sorted(com, key=lambda t: (t.split("|")[0], parse_tag(t)[1].get("funnel"), str(parse_tag(t)[1].get("backstop")), parse_tag(t)[2])):
            f, p, v, st = parse_tag(t)
            o = com[t]
            A(f"| {FORM_NAME[f]} | {'あり' if p.get('funnel') else 'なし'} | {p.get('backstop') or 'なし'} | {v:g} | " + " | ".join(cell(o[k]) for k in OBJS) + f" | {100 * macro(o):.0f}% |")
        A("")

    # ---- 上位 3 ----
    if s3:
        A("## 6. 上位 3 案 + 受け身のフード（N = 30、Wilson 90% 区間）\n")
        A("上位 3 案 = 異なる機構の最良の設計（スクリーニングと F・G の組み合わせの後）。受け身のフード（基準）も同じ N で。1 設計 × 対象物あたり 30 回（床 2 × 位置ずれ 3 × 5）。\n")
        for t in sorted(s3, key=lambda t: (-macro(s3[t]), t)):
            o = s3[t]
            A(f"\n`{short(t)}`（駆動 {DRIVES[t.split('|')[0]]}）\n")
            A("| 対象物 | 回数 | 保持 | 入る | 叩かれて入った | 押して逃げた | 挟まった | 逃げた距離 平均 [mm] |\n|---|---|---|---|---|---|---|---|")
            for k in OBJS:
                r = o[k]
                n = I(r, "n")
                A(f"| {NAME[k]} | {n} | {cell(r, True)} | {I(r, 'n_entered')}/{n} | {I(r, 'n_knocked_in')} | {I(r, 'n_pushed_ahead')} | {I(r, 'n_pinched')} | {float(r['escape_mean_mm']):.1f} |")
        A("")
    baseline_sections(A)
    bd = load("baseline")
    if bd:
        recommend_sections(A, scr, s3, bd)
    failure_sections(A, scr)
    variant_sections(A)
    variant2_sections(A)
    p = RES / "scoop_forms_extra.md"
    if p.exists():
        A(p.read_text(encoding="utf-8"))
    (RES / "scoop_forms_2026-09-30.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    print("\n".join(L))


if __name__ == "__main__":
    main()
