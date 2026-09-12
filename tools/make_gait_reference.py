"""歩容の参照角度列を CSV に書き出す（data/gait_reference.csv）。

ファームウェアが同じ DRIVE を受けたときの出力と、この表を突き合わせる（Phase 2 完了条件 1）。
**PC が角度列を送るためのものではない。** 照合用の物差し。

    .\\.venv\\Scripts\\python.exe tools\\make_gait_reference.py
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from serpens.config import load_config
from serpens.motion.gait import GaitParams, angles_at_phase, body_joint_names

OUT = Path("data/gait_reference.csv")


def main() -> None:
    ap = argparse.ArgumentParser(description="歩容の参照角度列を書き出す")
    ap.add_argument("--preset", default="forward", help="config の gait.presets 名")
    ap.add_argument("--gamma", type=float, default=0.0, help="旋回オフセット γ0 [deg]")
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--hz", type=float, default=None, help="刻み（既定は link.control_hz）")
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args()

    cfg = load_config()
    p = GaitParams.from_cfg(cfg["gait"]["presets"][args.preset])
    names = body_joint_names(cfg)
    profile = str(cfg["gait"]["turn_profile"])
    hz = args.hz if args.hz else float(cfg["link"]["control_hz"])
    dt = 1.0 / hz

    args.out.parent.mkdir(parents=True, exist_ok=True)
    rows = int(round(args.seconds * hz)) + 1
    with args.out.open("w", newline="", encoding="utf-8") as fp:
        w = csv.writer(fp)
        w.writerow([f"# preset={args.preset} A={p.amplitude_deg} Omega={p.spatial_freq_deg}"
                    f" f={p.temporal_freq_hz} gamma0={args.gamma} profile={profile}"])
        w.writerow(["t_s", *names])
        for k in range(rows):
            t = k * dt
            ang = angles_at_phase(p, 2.0 * math.pi * p.temporal_freq_hz * t, names, args.gamma,
                                  profile)
            w.writerow([f"{t:.4f}", *[f"{ang[n]:.4f}" for n in names]])
    print(f"{args.out} に {rows} 行（{args.seconds}s / {hz:.0f}Hz）を書きました。")
    print("ファーム側は t=0 で位相 0 から始め、同じ DRIVE を受けたときの出力と比べること。")


if __name__ == "__main__":
    main()
