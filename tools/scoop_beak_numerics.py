"""くちばしの「飛ばされた」が数値（時間刻み・接触のやわらかさ）のせいかを確かめる感度試験。MUJOCO_SIM。

    python tools/scoop_beak_numerics.py

上位設計（H15・ランプ 5・回転 0.6 s・幅 30・側壁なし・10 mm/s・停止して閉じる）で、時間刻みと solref を振り、
ビーズ / CR2032 / 1 円玉の「飛ばされた率」「保持率」が変わるかを見る。結果は simulation/results/scoop_beak_numerics_2026-09-30.md。
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from simulation.scoop.model import Shape, load_config  # noqa: E402
from simulation.scoop.sweep import LAUNCH_SPEED_MM_S, Case, run_cases  # noqa: E402

SETTINGS = {
    "基準（dt 2e-4, solref 0.002）": (),
    "dt 1e-4": (("timestep_s", 1.0e-4),),
    "dt 5e-5": (("timestep_s", 5.0e-5),),
    "solref 0.001（硬い）": (("solref_time_const_s", 0.001),),
    "solref 0.004（やわらかい）": (("solref_time_const_s", 0.004),),
}


def main() -> None:
    cfg = load_config()
    shape = Shape(0.6, 12.0, False, 30.0, 5.0)
    cases = []
    for name, num in SETTINGS.items():
        for obj in ("bead", "battery_cr2032", "coin_1yen"):
            for floor in cfg["floors"]:
                for off in cfg["placement"]["lateral_offsets_mm"]:
                    for k in range(2):
                        cases.append(Case(shape, obj, floor, 10.0, off, k, trigger="contact:0", close_time_s=0.6, beak=(15.0, 14.5),
                                          stop_on_trigger=True, numerics=num, tag=name))
    rows = run_cases(cases, workers=14)
    lines = ["# くちばしの数値感度（時間刻み・接触のやわらかさ）", "",
             "source = MUJOCO_SIM。上位設計（H15・ランプ 5・回転 0.6 s・幅 30・側壁なし・10 mm/s・停止して閉じる）、1 設定 × 対象物あたり 12 回。", "",
             f"飛ばされた = 物の速さが 1 度でも {LAUNCH_SPEED_MM_S:.0f} mm/s を超えた。", "",
             "| 設定 | 対象物 | 保持 | 飛ばされた | 乗る | 入る | 物の最大速度 中央値 [mm/s] |", "|---|---|---|---|---|---|---|"]
    for name in SETTINGS:
        for obj in ("bead", "battery_cr2032", "coin_1yen"):
            g = [r for r in rows if r["tag"] == name and r["obj"] == obj]
            n = len(g)
            sp = sorted(r["obj_speed_max_mm_s"] for r in g)
            lines.append(f"| {name} | {obj} | {sum(r['success'] for r in g)}/{n} | {sum(r['obj_speed_max_mm_s'] > LAUNCH_SPEED_MM_S for r in g)}/{n} | "
                         f"{sum(r['rode_ever'] for r in g)}/{n} | {sum(r['entered_ever'] for r in g)}/{n} | {sp[n // 2]:.0f} |")
    out = Path(__file__).resolve().parents[1] / "simulation" / "results" / "scoop_beak_numerics_2026-09-30.md"
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
