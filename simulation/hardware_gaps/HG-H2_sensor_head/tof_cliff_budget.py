"""ToF（VL53L1X）を「崖の 2 値」に使うときの、検出の遅れ（時間予算）と、口が崖を越える量（オーバーハング）の見積もり。**モデルの式。実測ではない（ASSUMED）**。

置き場所（Design、ENTRY-D-0008）: 横スキッドの前端（X −226、|y| 40、Z ≤ 6）。口の前の外側の角（漏斗の先端）は X −236.5 → **センサーは漏斗の先端の 10.5 mm 後ろ**。
崖を検出するのは、センサーが崖の縁を越えた時点。そのとき漏斗の先端は 10.5 mm 越えている。さらに、検出の遅れ + 停止の間に v × t だけ進む。
  オーバーハング [mm] = lead + v × (t_range + t_filter + t_link + t_stop)
  t_range  : 測距のタイミング予算（VL53L1X: 最短 20 ms（短距離モード）/ 33 ms）+ 読み出し周期
  t_filter : 連続 n フレームの一致（誤検出を減らす）= (n − 1) × 周期
  t_link   : 頭の XIAO → 胴の XIAO の伝送（UART。ASSUMED 5 ms）
  t_stop   : 胴の XIAO が DRIVE を止めるまで（ファームの制御周期 100 Hz = 10 ms。`docs/safety_limits.md` §3）
**頭の PC 経由（10 Hz + 往復）では届かない**（`docs/safety_limits.md`）ので、判定・停止は ESP32 側で完結させる前提。
    python simulation/hardware_gaps/HG-H2_sensor_head/tof_cliff_budget.py
"""
from __future__ import annotations

LEAD_MM = 10.5               # センサー → 漏斗の先端（X −226 → −236.5）
T_LINK_S = 0.005             # ASSUMED
T_STOP_S = 0.010             # 100 Hz の制御周期
FRAME_S = {"20 ms（短距離）": 0.020, "33 ms": 0.033, "50 ms": 0.050}


def overhang_mm(v_mm_s: float, frame_s: float, n_consecutive: int, lead_mm: float = LEAD_MM) -> float:
    t = frame_s + (n_consecutive - 1) * frame_s + T_LINK_S + T_STOP_S
    return lead_mm + v_mm_s * t


def table(speeds=(10.0, 30.0, 60.0, 80.0), ns=(1, 2, 3)) -> list[dict]:
    rows = []
    for v in speeds:
        for fname, fs in FRAME_S.items():
            rows.append({"v_mm_s": v, "frame": fname, **{f"n{n}": overhang_mm(v, fs, n) for n in ns}})
    return rows


if __name__ == "__main__":
    print("オーバーハング [mm]（漏斗の先端が崖の縁を越える量）。lead 10.5 mm を含む。n = 一致を要するフレーム数")
    for r in table():
        print(f"  v={r['v_mm_s']:>4g} mm/s  {r['frame']:<12s}  n=1 {r['n1']:5.1f}   n=2 {r['n2']:5.1f}   n=3 {r['n3']:5.1f}")
