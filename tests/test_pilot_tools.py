"""Human Perception Pilot の道具: 6/8/10 の比較、匿名クリップ、提示順のランダム化、回答の集計。

すべて実機ゼロで動く。動画の中身は KINEMATIC_SIM の描画（HUMAN_EVALUATED は 0 のまま）。
"""
from __future__ import annotations

import csv
import json
import subprocess
import sys
from pathlib import Path

import cv2
import pytest

from serpens.config import load_config
from serpens.motion.gait import body_joint_names
from simulation.body_compare import CONFIGS, evaluate, load, speed_matched_freq
from simulation.clips import ClipSpec, RenderSettings, write_clip
from tools.pilot_analysis import SCALES, summarize
from tools.pilot_order import orders

ROOT = Path(__file__).resolve().parent.parent


@pytest.mark.parametrize("name, n", [("yaw6", 6), ("yaw8", 8), ("yaw10", 10), ("yaw8_samelen", 8), ("yaw10_samelen", 10)])
def test_body_overlays_load_with_expected_axes(name: str, n: int) -> None:
    cfg = load(name)
    assert len(body_joint_names(cfg)) == n
    assert len(cfg["joints"]) == n + 3
    assert set(cfg["breath"]["amplitude_by_axis"]) == {j["name"] for j in cfg["joints"]}
    if name.endswith("samelen"):
        assert cfg["body"]["length_mm"] == load_config()["body"]["length_mm"]      # A: 同等の身体長


def test_wave_targets_are_realized_per_configuration() -> None:
    """同じ Ω を使い回さない: 狙い波数 → Ω = w·360/N で、可視波数が狙いに追従する。"""
    for name in ("yaw6", "yaw8", "yaw10"):
        cfg = load(name)
        w1 = evaluate(name, cfg, 1.0, 30.0, 0.5, "RAW")
        w2 = evaluate(name, cfg, 2.0, 30.0, 0.5, "RAW")
        assert w1.visible_waves < w2.visible_waves
        assert abs(w2.visible_waves - 2.0) < 0.4 and abs(w1.visible_waves - 1.0) < 0.5
        assert w2.forward_mm_per_cycle < w1.forward_mm_per_cycle          # 2 波は前進が落ちる（既知のトレードオフ）


def test_speed_matching_respects_servo_and_link_limits() -> None:
    ref = evaluate("yaw6", load("yaw6"), 1.0, 30.0, 0.5, "RAW")
    for name in ("yaw8", "yaw10"):
        cfg = load(name)
        raw = evaluate(name, cfg, 2.0, 30.0, 0.5, "RAW")
        f, clipped = speed_matched_freq(cfg, raw, ref.forward_mm_s)
        assert f <= float(cfg["link"]["limits"]["temporal_freq_hz"]) + 1e-9
        assert 30.0 * 2 * 3.141592653589793 * f <= 240.0 + 1e-6            # 関節速度の上限
        matched = evaluate(name, cfg, 2.0, 30.0, f, "SPEED_MATCHED")
        if not clipped:
            assert abs(matched.forward_mm_s - ref.forward_mm_s) / ref.forward_mm_s < 0.05


def test_clip_is_written_blind_and_key_holds_the_mapping(tmp_path: Path) -> None:
    rs = RenderSettings(duration_s=1.0, fps=5.0, width_px=480)
    n = write_clip(ClipSpec("yaw8_w2.0_RAW", CONFIGS["yaw8"], "gait", 30.0, 90.0, 0.5), rs, tmp_path / "clip_A.avi")
    assert n == 5
    cap = cv2.VideoCapture(str(tmp_path / "clip_A.avi"))
    ok, frame = cap.read()
    cap.release()
    assert ok and frame.shape[1] == 480
    assert "yaw" not in "clip_A.avi"                                      # ファイル名にも条件を出さない
    n2 = write_clip(ClipSpec("arc", None, "arc", profile="config/profile_exhibition.yaml"), rs, tmp_path / "clip_B.avi")
    assert n2 == 5


def test_pilot_clips_cli_writes_key_with_seed(tmp_path: Path) -> None:
    conditions = tmp_path / "c.json"
    conditions.write_text(json.dumps({"conditions": [
        {"name": "yaw6_w1.0_RAW", "config": "yaw6", "target_waves": 1.0, "condition": "RAW"},
        {"name": "yaw8_w2.0_SPEED_MATCHED", "config": "yaw8", "target_waves": 2.0, "condition": "SPEED_MATCHED"}]}),
        encoding="utf-8")
    r = subprocess.run([sys.executable, "tools/pilot_clips.py", "--conditions", str(conditions), "--out", str(tmp_path / "p"),
                        "--seed", "3", "--duration", "1", "--fps", "5"], cwd=ROOT, capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    key = json.loads((tmp_path / "p" / "clip_key.json").read_text(encoding="utf-8"))
    assert key["blinded"] and key["assignment_seed"] == 3 and key["source"] == "KINEMATIC_SIM"
    assert set(key["clips"]) == {"clip_A", "clip_B"}
    assert {v["condition"] for v in key["clips"].values()} == {"yaw6_w1.0_RAW", "yaw8_w2.0_SPEED_MATCHED"}
    assert all((tmp_path / "p" / "clips" / f"{c}.avi").exists() for c in key["clips"])


def test_presentation_order_is_seeded_and_complete() -> None:
    ids = ["clip_A", "clip_B", "clip_C", "clip_D"]
    a, b = orders(ids, 6, 42), orders(ids, 6, 42)
    assert a == b                                                         # 同じ seed → 同じ順
    assert orders(ids, 6, 43) != a
    for p in a:
        assert sorted(p["presentation_order"]) == ids                     # 全員が全クリップを 1 回ずつ
    assert len({tuple(p["presentation_order"]) for p in a}) >= 3          # 順番がばらける


def test_analysis_summarizes_without_overclaiming(tmp_path: Path) -> None:
    rows = []
    for p in range(1, 9):
        for clip, base in (("clip_A", 3), ("clip_B", 5)):
            row = {"participant_anonymous_id": f"P{p:02d}", "presentation_order": "", "clip_id": clip,
                   "random_seed": "42", "snake_comfort": str(3 + p % 4), "optional_comment": ""}
            for s in SCALES:
                row[s] = str(min(7, max(1, base + (p % 3) - 1 + (2 if s == "fear" and clip == "clip_B" else 0))))
            rows.append(row)
    text = "\n".join(summarize(rows, None))
    assert "VIDEO_HUMAN_EVALUATION" in text and "n = 8" in text
    assert "clip_A → clip_B" in text and "Fear" in text
    assert "傾向" in text                                                 # 小さな差を結論にしない注意書き
    csv_path = tmp_path / "r.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as fp:
        w = csv.DictWriter(fp, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    r = subprocess.run([sys.executable, "tools/pilot_analysis.py", str(csv_path)], cwd=ROOT, capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    assert r.returncode == 0, r.stderr


def test_response_template_has_required_columns() -> None:
    header = (ROOT / "pilot" / "response_template.csv").read_text(encoding="utf-8").strip().split(",")
    for col in ("participant_anonymous_id", "presentation_order", "clip_id", "random_seed", "snake_comfort",
                *SCALES, "optional_comment"):
        assert col in header
    god = json.loads((ROOT / "pilot" / "godspeed_items.json").read_text(encoding="utf-8"))
    assert "Godspeed" in god["instrument"] and set(god["subscales"]) >= {"animacy", "likeability", "perceived_safety"}
