"""HT-001（J1-0）: J1（Engineering の J7）に小さな角度（既定 +5°）を指令して、**頭が上がるか下がるか**を人が確かめ、記録する対話ツール。
ストッパーの窓（Design の θ_E −4〜+3°）を切る前に必須（`ai-shared/HARDWARE_TODO.md` HT-001、ENTRY-E-0009）。

  python tools/j1_sign_check.py                                  # モックで練習（実機なしで最後まで動く。モックの符号は「+ = 上げ」の設定どおり）
  python tools/j1_sign_check.py --bus feetech --port COM5        # 実機（**動かすのは User が実行したときだけ**）

安全: トルク制限は安全上限（0.167 × ストール = 約 0.45 N·m）に対する割合で、既定 0.5（**HG-H1 の prior ではバックドライブが 0.05〜0.5 N·m なので、低すぎると頭が動かず判定できない**。動かなければ 1.0 = 安全上限まで。それ以上は書かない）。速さ 20 °/s、角度は ±8° まで（それ以上は拒否）。**指を入れない。範囲外へ出たらすぐ電源を切れるスイッチを手元に。**
`config/robot.yaml` は変えない。`floor_watch_enforce`（範囲の強制）は関係しない（このツールは PC から直接サーボへ書く）。
**この結果（符号）は、その個体・その取り付け・その条件に限る HARDWARE_VERIFIED。** 別の個体・取り付けの向きの変更のあとは測り直す。
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from datetime import date, datetime
from pathlib import Path
from typing import Callable

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from serpens.config import load_config  # noqa: E402
from serpens.hw.servo_bus import ServoBus, make_bus  # noqa: E402

MAX_ABS_DEG = 8.0
SPEED_DPS = 20.0
ACCEL = 100.0
SETTLE_S = 1.5
CSV_COLUMNS = ["date", "time", "operator", "servo_id", "test_id", "torque_ratio", "commanded_deg", "present_deg_before", "present_deg_after",
               "head_moved", "front_edge_height_before_mm", "front_edge_height_after_mm", "sign_matches_engineering", "note"]


def run(bus: ServoBus, cfg: dict, sid: int, angle: float, torque_ratio: float, ask: Callable[[str], str], out_dir: Path,
        sleep: Callable[[float], None] = time.sleep, operator: str = "") -> dict:
    """対話の本体（`ask` と `sleep` を差し替えて試験できる）。戻り値 = 記録した行。"""
    if not 0.0 < abs(angle) <= MAX_ABS_DEG:
        raise ValueError(f"角度は 0 < |角度| ≤ {MAX_ABS_DEG:g}° に限る（指定 {angle:g}°）")
    if not 0.0 < torque_ratio <= 1.0:
        raise ValueError("トルク比は 0 < 比 ≤ 1.0（安全上限）に限る")
    if ask("  頭の模型（または実機の頭）を付け、指を入れない状態で、手元に電源断スイッチがありますか？ 続けるなら YES と入力: ").strip() != "YES":
        raise SystemExit("中止しました（YES が入力されなかった）")
    bus.set_torque_limit(sid, torque_ratio)
    bus.set_torque(sid, True)
    try:
        bus.set_goal(sid, 0.0, SPEED_DPS, ACCEL)
        sleep(SETTLE_S)
        before = bus.read_state(sid).pos_deg
        print(f"  0° を指令: 現在角 {before:+.2f}°")
        h0 = ask("  頭の前縁（口の先）の床からの高さ [mm]（ダイヤルゲージ / 定規。不明なら Enter）: ").strip()
        bus.set_goal(sid, angle, SPEED_DPS, ACCEL)
        sleep(SETTLE_S)
        after = bus.read_state(sid).pos_deg
        print(f"  {angle:+.1f}° を指令: 現在角 {after:+.2f}°")
        h1 = ask("  頭の前縁の高さ [mm]（不明なら Enter）: ").strip()
        ans = ask("  頭は「上がった(u)」「下がった(d)」「動かない(n)」のどれですか？: ").strip().lower()
        bus.set_goal(sid, 0.0, SPEED_DPS, ACCEL)
        sleep(SETTLE_S)
    finally:
        bus.set_torque(sid, False)
    moved = {"u": "up", "d": "down", "n": "none"}.get(ans[:1], "unknown")
    expected = "up" if angle > 0 else "down"
    matches = {"up": angle > 0, "down": angle < 0, "none": None, "unknown": None}[moved]
    row = {"date": date.today().isoformat(), "time": datetime.now().strftime("%H:%M:%S"), "operator": operator, "servo_id": sid, "test_id": "J1-0",
           "torque_ratio": torque_ratio, "commanded_deg": angle, "present_deg_before": round(before, 3), "present_deg_after": round(after, 3), "head_moved": moved,
           "front_edge_height_before_mm": h0, "front_edge_height_after_mm": h1, "sign_matches_engineering": "" if matches is None else str(matches), "note": ""}
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"j1_sign_check_{date.today():%Y%m%d}.csv"
    new = not path.exists()
    with open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        if new:
            w.writeheader()
        w.writerow(row)
    if matches is True:
        print(f"  → 符号は Engineering と一致（{angle:+.1f}° で頭が{'上がった' if angle > 0 else '下がった'}。+ = 頭を上げる）。{path} に記録しました")
    elif matches is False:
        print(f"  → **符号が逆**（期待: {expected}、実際: {moved}）。`direction` の扱いを Engineering に伝える（robot.yaml はこのツールでは変えません）。{path} に記録しました")
    else:
        print(f"  → 判定できません（{moved}）。動かなかった場合は、トルク比を 1.0（安全上限）まで上げて（--torque 1.0）もう一度。それでも動かなければ、取り付け・配線・電源を確認する（HT-004 の動き出しの測定へ）。{path} に記録しました")
    row["expected"] = expected
    return row


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bus", choices=["mock", "feetech"], default="mock")
    ap.add_argument("--port", default=None)
    ap.add_argument("--id", type=int, default=None, help="J7 のサーボ ID（既定は config の J7）")
    ap.add_argument("--angle", type=float, default=5.0)
    ap.add_argument("--torque", type=float, default=0.5, help="安全上限に対する割合（0 < 比 ≤ 1.0）。動かなければ 1.0 まで上げる")
    ap.add_argument("--operator", default="")
    a = ap.parse_args()
    cfg = load_config()
    sid = a.id if a.id is not None else int(next(j["servo_id"] for j in cfg["joints"] if j["name"] == "J7"))
    bus = make_bus(a.bus, cfg, a.port)
    bus.connect()
    try:
        run(bus, cfg, sid, a.angle, a.torque, input, Path(__file__).resolve().parent.parent / "hardware" / "prototypes" / "H1_joint", operator=a.operator)
    finally:
        bus.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
