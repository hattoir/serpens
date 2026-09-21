"""文法（grammar）: 状態ごとに「どの語彙（primitives）をいつ撃つか」の表。**表は config の behavior.grammar。**

    on_notice:            人に気づいた出来事（状態ではない）で撃つ列
    <STATE>.on_enter:     状態に入ってから at 秒後に撃つ列（at は数か [lo, hi]）
    <STATE>.while:        状態の間ずっと。every: [lo, hi] 秒ごと / rate_per_min: 回/分（novelty で base→peak）

項目の形: {do: <語彙>, at: 0.2 | [0.15, 0.25], every: [3.5, 7.0], rate_per_min: novelty,
          times: 3, gap_s: 0.5, only: paused | alone_paused, ...語彙ごとの引数}

狙いは「config だけで動きの人格をチューニングできる」こと。展示会場で「もう少し臆病に」を
config 1 行で試せるようにする。語彙そのものは primitives.py（状態を知らない）。
"""
from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Any, Callable

from serpens.behavior.primitives import Primitives
from serpens.motion.poses import HEAD_YAW

VOCABULARY = ("eye_flash", "twitch", "flick", "glance_away", "tilt", "stretch", "hold_breath", "sag", "nuzzle",
              "look_at")
CONDITIONS = ("paused", "alone_paused", "person")
EVENT_PREFIX = "on_"


@dataclass(frozen=True)
class GrammarCtx:
    """語彙を撃つ瞬間に要る外界の情報（brain が作る）。"""

    state: str
    novelty: float
    direction: float          # 人を見るための J8 [deg]（人がいなければ 0）
    has_person: bool
    paused: bool              # 巡回の静止中か


class Grammar:
    """config の表に従って primitives を撃つ。"""

    def __init__(self, cfg: dict[str, Any], prim: Primitives, rng: random.Random,
                 ctx_fn: Callable[[], GrammarCtx]) -> None:
        self.g = cfg["behavior"]["grammar"]
        self.flick_cfg = cfg["behavior"]["expression"]["flick"]
        self.prim, self.rng, self.ctx_fn = prim, rng, ctx_fn
        self._check()
        self._next: dict[tuple[str, int], float] = {}     # every 項目の次の時刻
        self._last_tick_t: float | None = None
        self.flick_until = -1.0
        self.state = ""

    def _check(self) -> None:
        """未知の語彙・条件を起動時に弾く（展示当日に config を打ち間違えても黙って無視しない）。"""
        for key, block in self.g.items():
            items = block if key.startswith(EVENT_PREFIX) else [*block.get("on_enter", []), *block.get("while", [])]
            for it in items:
                if it["do"] not in VOCABULARY:
                    raise ValueError(f"behavior.grammar.{key}: 未知の語彙 {it['do']!r}（{VOCABULARY}）")
                if it.get("only") is not None and it["only"] not in CONDITIONS:
                    raise ValueError(f"behavior.grammar.{key}: 未知の条件 {it['only']!r}（{CONDITIONS}）")

    # ---- 入口 -------------------------------------------------------------------
    def _u(self, v: Any) -> float:
        if isinstance(v, (list, tuple)):
            return self.rng.uniform(float(v[0]), float(v[1]))
        return float(v)

    def event(self, t: float, name: str) -> None:
        """出来事（on_<name>）の列を予定表へ積む。同じ出来事の予約は捨てて積み直す。"""
        tag = f"grammar:{EVENT_PREFIX}{name}"
        self.prim.cancel(tag)
        for it in self.g.get(f"{EVENT_PREFIX}{name}", []):
            self._schedule(t, tag, it)

    def enter(self, t: float, state: str) -> None:
        """状態に入った。前の状態の予約を捨て、on_enter を積み、while のタイマーを引き直す。"""
        self.prim.cancel(f"grammar:{self.state}")
        self.state = state
        block = self.g.get(state, {})
        for it in block.get("on_enter", []):
            self._schedule(t, f"grammar:{state}", it)
        self._next = {}
        for i, it in enumerate(block.get("while", [])):
            if "every" in it:
                self._next[(state, i)] = t + self._u(it["every"])

    def tick(self, t: float) -> None:
        """状態の間ずっと（while）の項目を回す。"""
        last, self._last_tick_t = self._last_tick_t, t
        dt = 0.0 if last is None else max(t - last, 0.0)
        ctx = self.ctx_fn()
        if ctx.state != self.state:                      # 起動直後の状態など、enter を通っていないとき
            self.enter(t, ctx.state)
        for i, it in enumerate(self.g.get(self.state, {}).get("while", [])):
            if not self._allowed(it, ctx):
                continue
            if "every" in it:
                if t >= self._next.get((self.state, i), float("inf")):
                    if self._fire(t, it, ctx):
                        self._next[(self.state, i)] = t + self._u(it["every"])
            elif "rate_per_min" in it:
                rate = self._rate(it, ctx)
                if self.rng.random() < rate / 60.0 * dt:
                    self._fire(t, it, ctx)

    # ---- 内部 -------------------------------------------------------------------
    def _schedule(self, t: float, tag: str, it: dict[str, Any]) -> None:
        at = self._u(it.get("at", 0.0))
        for k in range(int(it.get("times", 1))):
            when = t + at + k * float(it.get("gap_s", 0.0))
            self.prim.schedule(when, tag, lambda tt, it=it: self._fire(tt, it, self.ctx_fn()))

    def _rate(self, it: dict[str, Any], ctx: GrammarCtx) -> float:
        r = it["rate_per_min"]
        if r == "novelty":
            f = self.flick_cfg
            base, peak = float(f["base_rate_per_min"]), float(f["peak_rate_per_min"])
            return base + (peak - base) * min(max(ctx.novelty, 0.0), 1.0)
        return float(r)

    @staticmethod
    def _allowed(it: dict[str, Any], ctx: GrammarCtx) -> bool:
        only = it.get("only")
        if only == "paused":
            return ctx.paused
        if only == "alone_paused":
            return ctx.paused and not ctx.has_person
        if only == "person":
            return ctx.has_person
        return True

    def _fire(self, t: float, it: dict[str, Any], ctx: GrammarCtx) -> bool:
        """語彙を1つ撃つ。撃てなかったら False（別の動きの最中など）。"""
        p, do = self.prim, it["do"]
        if not self._allowed(it, ctx):
            return False
        if do == "eye_flash":
            p.eye_flash(t, ctx.state)
        elif do == "twitch":
            p.twitch(t, ctx.direction)
        elif do == "flick":
            if t < self.flick_until or t < p.busy_until or p.anim.busy_joint(HEAD_YAW, t):
                return False
            self.flick_until = p.flick(t)
        elif do == "glance_away":
            if t < p.busy_until:
                return False
            p.glance_away(t)
        elif do == "tilt":
            if t < p.busy_until:
                return False
            p.tilt(t)
        elif do == "stretch":
            if t < p.busy_until or p.person_tracked:
                return False
            p.stretch(t)
        elif do == "hold_breath":
            p.hold_breath(t, float(it["hold_s"]))
        elif do == "sag":
            p.sag(t, float(it["torque_ratio"]), float(it["release_s"]))
        elif do == "nuzzle":
            side = ctx.direction if (it.get("toward") == "person" and ctx.has_person) else self.rng.choice((-1.0, 1.0))
            p.nuzzle(t, side)
        elif do == "look_at":
            p.look_at(t, ctx.direction, it.get("neck_deg"), force=bool(it.get("force", False)))
        return True
