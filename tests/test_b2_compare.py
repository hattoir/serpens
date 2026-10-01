"""B2 の実物の観察 CSV とシミュレーションの突き合わせ（`tools/b2_compare.py`）。**実物のデータは無い。ここで確かめるのは、読み込み・対応づけ・判定の仕組みだけ**（合成の観察で）。"""
from __future__ import annotations

import csv
import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope="module")
def bc():
    spec = importlib.util.spec_from_file_location("b2_compare", ROOT / "tools" / "b2_compare.py")
    m = importlib.util.module_from_spec(spec)
    sys.modules["b2_compare"] = m
    spec.loader.exec_module(m)
    return m


HEADER = ["priority", "trial_id", "condition_t_mm", "condition_c_mm", "plate_T_mm(=c+t)", "plate_how", "object", "floor", "trial_no",
          "result(enter/return/underrun/pushed_away)", "video_file", "notes"]


def write_log(path: Path, rows: list[tuple]) -> Path:
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(HEADER)
        for i, (t, c, obj, floor, res) in enumerate(rows):
            w.writerow(["A", f"x{i}", t, c, t + c, "", obj, floor, 1, res, "", ""])
    return path


def test_the_template_in_ai_shared_has_the_design_columns_first() -> None:
    tpl = ROOT.parent / "ai-shared" / "B2_test_log_template.csv"
    if not tpl.exists():
        pytest.skip("ai-shared は git 管理外（main の作業ディレクトリ）")
    head = next(csv.reader(open(tpl, encoding="utf-8-sig")))
    assert head[:12] == HEADER and "measured_t_mm" in head


def test_log_is_read_and_unobserved_rows_are_ignored(bc, tmp_path: Path) -> None:
    p = write_log(tmp_path / "log.csv", [(0, 0, "1yen", "flooring", "enter"), (0, 0, "CR2032", "flooring", ""), (0.5, 0, "bead8", "mat", "pushed_away"), (0, 0, "??", "flooring", "enter")])
    rows = bc.read_log(p)
    assert [(r["obj"], r["floor"], r["result"]) for r in rows] == [("coin_1yen", "flooring", "enter"), ("bead", "mat", "pushed_away")]


def test_simulation_predicts_hold_without_a_step_and_zero_with_a_step(bc) -> None:
    sim = bc.load_sim()
    p0, t0, ex0 = bc.predict(sim, 0.0, 0.3, "crumb_cube", "flooring")
    assert p0["success"] == p0["n"] and not ex0                       # 段差なし・すき間 0.3 mm は保持
    p1, t1, ex1 = bc.predict(sim, 0.5, 0.3, "crumb_cube", "flooring")
    assert p1["success"] == 0 and t1 == 0.5                            # 段差 0.5 mm は 0%
    p2, t2, ex2 = bc.predict(sim, 1.0, 0.3, "crumb_cube", "flooring")  # 1.0 mm はシミュレーションに無い → 最も近い t = 0.5 を使い、外挿と印を付ける
    assert ex2 and t2 == 0.5 and p2["success"] == 0


def test_agreement_and_disagreement_are_told_apart(bc, tmp_path: Path) -> None:
    rows = [(0, 0.3, "cube10", "flooring", "enter")] * 4 + [(0.5, 0.3, "cube10", "flooring", "enter")] * 3 + [(0.5, 0.3, "bead8", "mat", "pushed_away")] * 3
    cmp_ = bc.compare(bc.read_log(write_log(tmp_path / "log.csv", rows)), bc.load_sim())
    by = {(r["t"], r["obj"]): r for r in cmp_}
    assert by[(0.0, "crumb_cube")]["verdict"] == "一致"
    assert by[(0.5, "crumb_cube")]["verdict"].startswith("実物が良い")     # シミュは 0%、実物は 3/3 入った → シミュが悲観
    assert by[(0.5, "bead")]["verdict"] == "一致"
    out = tmp_path / "out.md"
    bc.write_md(cmp_, out, "log.csv")
    assert "一致 2 / 3 条件" in out.read_text(encoding="utf-8")
