import sys, json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
sys.path.insert(0, "simulation/hardware_gaps/HG-H2_sensor_head")
import run as R
if __name__ == "__main__":
    nom = dict(R.A["nominal"])
    base = [("nominal", {}), ("shadow=0.6", {"shadow_factor": 0.6}), ("gain=0.5", {"exposure_gain": 0.5}), ("noise=5", {"read_noise": 5.0}), ("ambient=150", {"ambient_lux": 150.0})]
    wins = [("T1f0", {}), ("T0.4f0.05", {"window_T": 0.4, "window_flare": 0.05}), ("T0.4f0.2", {"window_T": 0.4, "window_flare": 0.2})]
    conds = [(f"{b}+{w}", {**nom, **bp, **wp}) for b, bp in base for w, wp in wins]
    with ProcessPoolExecutor(max_workers=13) as ex:
        rows = list(ex.map(R.evaluate_condition, conds))
    out = []
    for (tag, _), r in zip(conds, rows):
        out.append({"tag": tag, "patrol": r["recall_patrol"], "inspect": r["recall_inspect_online"], "fa": r["false_alarm_patrol"], "ok": bool(R.ok(r))})
        print(out[-1])
    Path("simulation/hardware_gaps/HG-H2_sensor_head/results/window_flare_weak_light_2026-10-05.json").write_text(json.dumps(out, indent=1), encoding="utf-8")
