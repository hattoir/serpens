"""Sim-to-Real 校正: 実測の前進量から belly プロファイルの摩擦（tangential）を同定する。

実機が届いたら最初にこれを回す。床材ごとに1行ずつ data/real_runs.csv に足していく。

CSV の列（1行目はヘッダ。Excel で保存するなら「CSV UTF-8」を推奨。Shift_JIS でも読める）:
  gait         歩容名（config の gait.presets のキー。校正には forward / slow / backward の直進系を使う）
  period_s     歩容の周期 [s]
  cycles       測った周期数
  distance_mm  その間に首マーカ（または胴体の中心）が進んだ正味の距離 [mm]
  floor        床材（例: felt_mat）。床材ごとに別々に同定する
  date         日付
  note         メモ

処理: いま選ばれている belly プロファイルの tangential を sim.fit_ratio_min〜max でスイープし、
      シミュレータの「1周期あたりの前進量」と実測の差の二乗和が最小になる値を求める。
出力: 床材ごとの最適値・残差、比較グラフ（output/fit_sim.png）

使い方: python tools/fit_sim.py [--csv data/real_runs.csv] [--out output/fit_sim.png]
"""
from __future__ import annotations

import argparse
import copy
import csv
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from serpens.config import load_config  # noqa: E402
from serpens.sim.measure import gait_with_period, per_cycle_advance  # noqa: E402

REQUIRED = ("gait", "period_s", "cycles", "distance_mm", "floor")
ENCODINGS = ("utf-8-sig", "cp932")


@dataclass(frozen=True)
class Run:
    gait: str
    period_s: float
    cycles: float
    distance_mm: float
    floor: str
    date: str
    note: str

    @property
    def per_cycle_mm(self) -> float:
        return self.distance_mm / self.cycles


def read_runs(path: Path, presets: dict) -> list[Run]:
    """CSV を読む。不正な行は理由を表示して飛ばす。"""
    text = None
    for enc in ENCODINGS:
        try:
            text = path.read_text(encoding=enc)
            break
        except UnicodeDecodeError:
            continue
    if text is None:
        raise SystemExit(f"{path} の文字コードを判別できません（UTF-8 か Shift_JIS で保存してください）")
    runs: list[Run] = []
    rows = csv.DictReader(text.splitlines())
    missing = [c for c in REQUIRED if c not in (rows.fieldnames or [])]
    if missing:
        raise SystemExit(f"CSV に列がありません: {missing}")
    for i, r in enumerate(rows, start=2):
        try:
            run = Run(r["gait"].strip(), float(r["period_s"]), float(r["cycles"]), float(r["distance_mm"]),
                      r["floor"].strip(), (r.get("date") or "").strip(), (r.get("note") or "").strip())
        except (TypeError, ValueError) as e:
            print(f"  {i}行目を飛ばします: {e}")
            continue
        if run.gait not in presets or run.period_s <= 0 or run.cycles <= 0:
            print(f"  {i}行目を飛ばします: 歩容名・周期・周期数を確認（{run.gait}）")
            continue
        runs.append(run)
    return runs


def sweep(cfg: dict, runs: list[Run], ratios: np.ndarray) -> dict[tuple[str, float], np.ndarray]:
    """(歩容, 周期) ごとに、各 ratio でのシミュレータの1周期前進量。"""
    s = cfg["sim"]
    curves: dict[tuple[str, float], np.ndarray] = {}
    for key in sorted({(r.gait, r.period_s) for r in runs}):
        vals = []
        for ratio in ratios:
            c = copy.deepcopy(cfg)
            prof = c["belly"]["profiles"][c["belly"]["type"]][c["belly"]["friction_profile"]]
            prof["tangential"] = float(ratio)
            p = gait_with_period(c, key[0], key[1])
            vals.append(per_cycle_advance(c, p, float(s["fit_warmup_cycles"]), float(s["fit_measure_cycles"])).per_cycle_mm)
        curves[key] = np.array(vals)
    return curves


def fit(runs: list[Run], curves: dict, ratios: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    """二乗和最小の ratio（格子の最小点を放物線で補間）。戻り値: (最適値, 各格子点の損失, 最適値での残差)"""
    loss = np.zeros_like(ratios)
    for r in runs:
        loss += (curves[(r.gait, r.period_s)] - r.per_cycle_mm) ** 2
    k = int(np.argmin(loss))
    best = float(ratios[k])
    if 0 < k < len(ratios) - 1:
        y0, y1, y2 = loss[k - 1], loss[k], loss[k + 1]
        denom = y0 - 2 * y1 + y2
        if denom > 0:
            best = float(ratios[k] + 0.5 * (y0 - y2) / denom * (ratios[1] - ratios[0]))
    resid = np.array([np.interp(best, ratios, curves[(r.gait, r.period_s)]) - r.per_cycle_mm for r in runs])
    return best, loss, resid


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", default="data/real_runs.csv")
    ap.add_argument("--out", default="output/fit_sim.png")
    args = ap.parse_args()
    plt.rcParams["font.family"] = ["Yu Gothic", "Meiryo", "MS Gothic", "sans-serif"]
    cfg = load_config()
    s = cfg["sim"]
    ratios = np.linspace(float(s["fit_ratio_min"]), float(s["fit_ratio_max"]), int(s["fit_ratio_steps"]))
    runs = read_runs(Path(args.csv), cfg["gait"]["presets"])
    if not runs:
        raise SystemExit("有効な行がありません")
    by_floor: dict[str, list[Run]] = defaultdict(list)
    for r in runs:
        by_floor[r.floor].append(r)
    print(f"実測 {len(runs)} 行 / 床材 {len(by_floor)} 種。ratio を {ratios[0]}〜{ratios[-1]} で {len(ratios)} 点スイープ…")
    curves = sweep(cfg, runs, ratios)

    fig, axes = plt.subplots(1, len(by_floor), figsize=(7 * len(by_floor), 5.5), squeeze=False)
    for ax, (floor, fr) in zip(axes[0], sorted(by_floor.items())):
        best, loss, resid = fit(fr, curves, ratios)
        rms = float(np.sqrt(np.mean(resid ** 2)))
        edge = best <= ratios[0] + 1e-9 or best >= ratios[-1] - 1e-9
        print(f"\n[{floor}] 最適 tangential（belly プロファイル） = {best:.4f}   残差 RMS = {rms:.1f} mm/周期"
              + ("   ※スイープ範囲の端。モデルか実測を見直すこと" if edge else ""))
        print(f"  {'歩容':<10}{'周期':>6}{'実測':>12}{'シム':>12}{'差':>9}")
        for r, e in zip(fr, resid):
            print(f"  {r.gait:<10}{r.period_s:6.2f}{r.per_cycle_mm:10.1f}mm{r.per_cycle_mm + e:10.1f}mm{e:+8.1f}  {r.note}")
        for key in sorted({(r.gait, r.period_s) for r in fr}):
            ax.plot(ratios, curves[key], "-o", ms=3, label=f"シム {key[0]} T={key[1]}s")
        for r in fr:
            ax.axhline(r.per_cycle_mm, color="k", ls=":", lw=1)
            ax.plot([best], [r.per_cycle_mm], "k*", ms=12)
        ax.axvline(best, color="tab:red", lw=2, label=f"最適 {best:.3f}")
        ax.set_xlabel("belly の tangential")
        ax.set_ylabel("1周期あたりの前進量 [mm]")
        ax.set_title(f"床材: {floor}（点線 = 実測）  残差 RMS {rms:.1f}mm")
        ax.grid(alpha=0.3)
        ax.legend()
    fig.tight_layout()
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=90)
    print(f"\nsaved {args.out}")
    print("→ 採用するなら config/robot.yaml の belly.profiles[type][profile].tangential を書き換える"
      "（床材ごとに値が違えば展示会場の床に合わせる）")


if __name__ == "__main__":
    main()
