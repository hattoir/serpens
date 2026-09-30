"""H / G: 受け身のフード（機構なし）。頭が前進して物に被さり、物の中心が奥へ入ったらゲートを閉じる。
G（漏斗）= 口が 60 → 奥で 30 に狭まる形。他の案（A / B / C）の補助として組み合わせる形でもある。
駆動数 = ゲート 1（ゲートの駆動を含めて 1）。ここが「機構を足す価値があるか」の比較の基準になる。
"""
from __future__ import annotations

from simulation.scoop.forms.common import CLS, Form, Parts, backstop_parts, hood, merge
from simulation.scoop.model import MM, _f


class Passive(Form):
    name = "hood"
    drives = 1
    drive_note = "ゲート 1（機構なし）"

    def __init__(self, cfg, params, obj_name, floor_name):
        super().__init__(cfg, params, obj_name, floor_name)
        self.has_gate = bool(params.get("gate", True))                  # False = ゲートなし（ゲートの効果の切り分け）
        self.plate = float(params.get("plate", 0.0)) * MM              # > 0: 口に厚さ plate の床の板（垂直な前面の段差）を足す = 前回のスコップの「先端の厚み」の対照

    def build(self) -> Parts:
        funnel = bool(self.p.get("funnel", False))
        zb = self.clear + self.plate if self.plate > 0 else 0.0
        parts = merge(hood(self, funnel=funnel, zb=zb, gate=self.has_gate), backstop_parts(self))
        if self.plate > 0:
            parts.head_geoms.append(f'<geom name="cavplate" type="box" size="{_f(self.depth / 2)} {_f(self.half_w)} {_f(self.plate / 2)}" '
                                    f'pos="{_f(self.depth / 2)} 0 {_f(self.clear + self.plate / 2)}" {CLS}/>')
            parts.obj_pairs.append(("cavplate", self.mu_wall))
        return parts
