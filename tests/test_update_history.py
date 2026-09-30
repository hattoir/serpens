"""tools/update_history.py: 自動台帳が git から作れ、手書き部分（narrative）を壊さないこと。"""
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location("update_history", ROOT / "tools" / "update_history.py")
uh = importlib.util.module_from_spec(spec)
spec.loader.exec_module(uh)


def test_build_has_all_sections():
    text = uh.build()
    for head in ("### 1. ブランチの地図", "### 2. 日付ごとの全コミット", "### 3. 判断の記録", "### 4. Design ↔ Engineering"):
        assert head in text
    assert "`main`" in text


def test_history_file_keeps_both_marker_pairs():
    t = (ROOT / "agent" / "HISTORY.md").read_text(encoding="utf-8")
    for m in ("<!-- BEGIN:narrative -->", "<!-- END:narrative -->", uh.BEGIN, uh.END):
        assert m in t
    assert t.index("<!-- END:narrative -->") < t.index(uh.BEGIN)
