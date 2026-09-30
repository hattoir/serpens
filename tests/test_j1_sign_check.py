"""HT-001（J1-0）の対話ツール（`tools/j1_sign_check.py`）。モックのサーボで最後まで動くことと、安全側の拒否を確かめる。**実機の符号ではない**（モックの符号は設定どおり）。"""
from __future__ import annotations

import csv
import importlib.util
import sys
from pathlib import Path

import pytest

from serpens.config import load_config
from serpens.hw.mock_bus import MockServoBus

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope="module")
def tool():
    spec = importlib.util.spec_from_file_location("j1_sign_check", ROOT / "tools" / "j1_sign_check.py")
    m = importlib.util.module_from_spec(spec)
    sys.modules["j1_sign_check"] = m
    spec.loader.exec_module(m)
    return m


def answers(*a: str):
    it = iter(a)
    return lambda prompt: next(it)


class FakeTime:
    """モックのサーボに偽の時計を渡し、sleep で進める（実時間を待たない）。"""

    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t

    def sleep(self, s: float) -> None:
        self.t += s


def bus_for(cfg, clock=None):
    b = MockServoBus(cfg, clock=clock)
    b.connect()
    return b


def test_mock_run_records_a_row_and_reports_the_engineering_sign(tool, tmp_path: Path) -> None:
    cfg = load_config()
    clk = FakeTime()
    b = bus_for(cfg, clk)
    row = tool.run(b, cfg, 7, 5.0, 0.5, answers("YES", "10.0", "14.7", "u"), tmp_path, sleep=clk.sleep)
    assert row["head_moved"] == "up" and row["sign_matches_engineering"] == "True"
    files = list(tmp_path.glob("j1_sign_check_*.csv"))
    assert len(files) == 1
    rec = list(csv.DictReader(open(files[0], encoding="utf-8")))[0]
    assert rec["test_id"] == "J1-0" and rec["commanded_deg"] == "5.0" and float(rec["present_deg_after"]) == pytest.approx(5.0, abs=0.6)


def test_a_reversed_sign_is_reported_as_a_mismatch(tool, tmp_path: Path) -> None:
    cfg = load_config()
    clk = FakeTime()
    row = tool.run(bus_for(cfg, clk), cfg, 7, 5.0, 0.5, answers("YES", "", "", "d"), tmp_path, sleep=clk.sleep)
    assert row["sign_matches_engineering"] == "False"


def test_unsafe_arguments_and_missing_confirmation_are_refused(tool, tmp_path: Path) -> None:
    cfg = load_config()
    b = bus_for(cfg)
    for bad in (dict(angle=12.0, torque_ratio=0.5), dict(angle=0.0, torque_ratio=0.5), dict(angle=5.0, torque_ratio=1.5)):
        with pytest.raises(ValueError):
            tool.run(b, cfg, 7, bad["angle"], bad["torque_ratio"], answers("YES"), tmp_path, sleep=lambda s: None)
    with pytest.raises(SystemExit):
        tool.run(b, cfg, 7, 5.0, 0.5, answers("no"), tmp_path, sleep=lambda s: None)
    assert not list(tmp_path.glob("*.csv"))                                   # 何も記録しない・動かさない
