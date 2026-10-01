"""Child-Safe Attention Rules（CSAR）の実行条件: 子どもが近いか（NEAR / FAR / UNKNOWN）を Serpens 側で決める。

User 採用（USER-DEC-SERPENS-DESIGN-0001 #1、2026-09-29）: 危険物を見つけてもその場で見せびらかさない。再確認と位置登録の後、
基本は危険物から少し離れる。子どもが近い場合は危険物ではなく子ども側を見る。Design の規則は `docs/design/concepts_2026-09-29.md` §4。

Engineering の判断（ENTRY-0008 の問い (1)(2)(3) への答え。すべて config の `floor_watch.csar`、後から戻せる）:
  - **判定は Serpens 側**（安全は最下層で）。Home AI からの情報は安全側にだけ効く（「近い」は受け取る、「遠い」で上書きはしない）
  - **子どもと大人を区別できない**（人の検出だけ）→ `child_near_m` 以内の人は誰でも「子どもかもしれない」= NEAR
  - **FAR は「最後に周りを見て、近くに人がいなかった」から `far_trust_s` 秒だけ信じる**（OPEN-SERPENS-DESIGN-010 の案 A）。
    床を向いて撮影している間は人を見ていないので、長く信じない。何も見ていなければ UNKNOWN（= NEAR 扱い）
  - NEAR は最後に近くで見えてから `near_hold_s` 秒は続ける（一瞬見えなくなっただけで FAR にしない）
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

NEAR, FAR, UNKNOWN = "NEAR", "FAR", "UNKNOWN"


@dataclass
class ChildProximity:
    near_m: float
    far_trust_s: float
    near_hold_s: float
    last_near_t: float | None = None
    last_far_t: float | None = None
    hint_near_until: float | None = None

    @staticmethod
    def from_cfg(cfg: dict[str, Any]) -> "ChildProximity":
        c = cfg["floor_watch"]["csar"]
        return ChildProximity(float(cfg["floor_watch"]["risk"]["child_near_m"]), float(c["far_trust_s"]), float(c["near_hold_s"]))

    def observe(self, t: float, nearest_person_m: float | None, sensing: bool) -> None:
        """Serpens 自身の人の観測。sensing = いま人を見られる向き・状態か（床を向いて撮影中は False）。
        nearest_person_m = 一番近い人までの距離（見えていなければ None）。"""
        if nearest_person_m is not None and nearest_person_m <= self.near_m:
            self.last_near_t = t
        elif sensing:
            self.last_far_t = t                                       # 見られる状態で、近くに人がいなかった

    def hint_near(self, t: float, duration_s: float) -> None:
        """Home AI などからの「子どもが近い」（安全側だけ受け取る）。"""
        self.hint_near_until = max(self.hint_near_until or t, t + duration_s)

    def state(self, t: float) -> str:
        if self.hint_near_until is not None and t < self.hint_near_until:
            return NEAR
        if self.last_near_t is not None and t - self.last_near_t < self.near_hold_s:
            return NEAR
        if self.last_far_t is not None and t - self.last_far_t <= self.far_trust_s:
            return FAR
        return UNKNOWN

    def allows_attention_to_object(self, t: float) -> bool:
        """物を見る・照らす・指す（撮影の閃光、highlight_point）をしてよいか。FAR のときだけ（R1・R2）。"""
        return self.state(t) == FAR
