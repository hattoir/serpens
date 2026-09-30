"""H / G: 受け身のフード（機構なし）。頭が前進して物に被さり、物の中心が奥へ入ったらゲートを閉じる。
G（漏斗）= 口が 60 → 奥で 30 に狭まる形。他の案（A / B / C）の補助として組み合わせる形でもある。
駆動数 = ゲート 1（ゲートの駆動を含めて 1）。ここが「機構を足す価値があるか」の比較の基準になる。
"""
from __future__ import annotations

from simulation.scoop.forms.common import Form, Parts, backstop_parts, hood, merge


class Passive(Form):
    name = "hood"
    drives = 1
    drive_note = "ゲート 1（機構なし）"

    def build(self) -> Parts:
        funnel = bool(self.p.get("funnel", False))
        return merge(hood(self, funnel=funnel), backstop_parts(self))
