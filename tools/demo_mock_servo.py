"""MockServoBus の動作確認デモ（STEP 2 の確認用）。

1. 9軸に目標角を出して、角度が一次遅れで追従する様子を表示
2. J1〜J6 を往復させ続けて、負荷と温度が上がる様子を表示
3. 止めて、温度が環境温度に戻る様子を表示

時間はシミュレーション時間（実時間を待たない）。
使い方: python tools/demo_mock_servo.py [--heat-tau 60]
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from serpens.config import load_config  # noqa: E402
from serpens.hw.mock_bus import MockServoBus  # noqa: E402
from serpens.hw.servo_bus import Goal  # noqa: E402


class SimClock:
    """手で進める時計。"""

    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


def row(bus: MockServoBus, t: float, field: str) -> str:
    st = bus.sync_read_states()
    vals = " ".join(f"{getattr(st[s], field):7.1f}" for s in bus.ids)
    return f"{t:7.2f}s |{vals}"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--heat-tau", type=float, default=None, help="温度時定数[s]（省略時は config）")
    ap.add_argument("--speed", type=float, default=120.0, help="指令速度 [deg/s]")
    ap.add_argument("--accel", type=float, default=1500.0, help="指令加速度 [deg/s²]")
    ap.add_argument("--swing", type=float, default=35.0, help="往復振幅 [deg]")
    ap.add_argument("--half-period", type=float, default=0.6, help="往復の片道時間 [s]")
    ap.add_argument("--work", type=float, default=240.0, help="往復を続ける時間 [s]")
    args = ap.parse_args()

    cfg = load_config()
    if args.heat_tau is not None:
        cfg["mock_servo"]["heat_tau_s"] = args.heat_tau
    clock = SimClock()
    bus = MockServoBus(cfg, clock=clock)
    bus.connect()
    for sid in bus.ids:
        bus.set_torque(sid, True)
    names = " ".join(f"{j.name:>7}" for j in bus.joints.values())
    header = f"{'時刻':>6}  |{names}"

    # ---- 1. 追従 ----
    targets = {1: 30, 2: -30, 3: 20, 4: -20, 5: 10, 6: -10, 7: 60, 8: 25, 9: 15}
    print("=== 1. 目標角への追従  目標:", {bus.joints[s].name: d for s, d in targets.items()})
    bus.sync_set_goals({s: Goal(float(d), args.speed, args.accel) for s, d in targets.items()})
    print(header)
    for k in range(9):
        print(row(bus, clock.t, "pos_deg"))
        clock.t += 0.1
    print(row(bus, clock.t, "pos_deg"), " ← 収束")

    # ---- 2. 往復運動で発熱 ----
    tau = cfg["mock_servo"]["heat_tau_s"]
    print(f"\n=== 2. J1〜J6 を ±{args.swing}° で往復（温度時定数 {tau}s）  温度[℃]")
    print(header)
    t_end = clock.t + args.work
    k, next_print = 0, clock.t
    while clock.t < t_end:
        sign = 1 if k % 2 == 0 else -1
        bus.sync_set_goals({s: Goal(sign * args.swing, args.speed, args.accel) for s in range(1, 7)})
        k += 1
        clock.t += args.half_period
        if clock.t >= next_print:
            print(row(bus, clock.t, "temp_c"))
            next_print += args.work / 8
    st = bus.sync_read_states()
    print("  動作中の負荷(J1〜J6):", " ".join(f"{st[s].load:+.2f}" for s in range(1, 7)),
          f"  電圧 J1: {st[1].volt:.2f}V")

    # ---- 3. 停止して冷却 ----
    print("\n=== 3. 停止（J1〜J6 は 0° で保持）→ 冷える  温度[℃]")
    bus.sync_set_goals({s: Goal(0.0, args.speed, args.accel) for s in range(1, 7)})
    print(header)
    for _ in range(6):
        clock.t += tau / 2
        print(row(bus, clock.t, "temp_c"))
    bus.disconnect()


if __name__ == "__main__":
    main()
