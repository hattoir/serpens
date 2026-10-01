"""H0 摩擦クーポンの実測 CSV → 摩擦係数 → MuJoCo で推進を予測 → Pure Snake / Wheel Belly・6 本目の判定材料。

ラベルの付け方（混ぜない）:
  摩擦係数（クーポン × 床）… 実測。記録に日付・条件・道具があれば HARDWARE_VERIFIED（その組み合わせに限る）
  前進の予測            … PHYSICS_SIM（実機の走行ではない。質量・kp・接触形状は未実測のまま）

手順は `hardware/prototypes/H0_friction/README.md`。
"""
from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from statistics import mean

from simulation.floorwatch_body import CONFIGS, MIN_PATROL_SPEED_MM_S, best_gait
from simulation.mujoco.belly import BellyProfile

DIRECTIONS = ("forward", "backward", "lateral")


@dataclass(frozen=True)
class Stat:
    mean: float
    lo: float
    hi: float
    n: int


@dataclass(frozen=True)
class CouponFloor:
    """1 つの床 × 1 つのクーポン（材料込み）の測定結果。"""

    floor_id: str
    coupon: str                     # 例: FLAT-PETG
    mu: dict[str, Stat]             # direction → 統計

    def complete(self) -> bool:
        return all(d in self.mu for d in DIRECTIONS)

    @property
    def ratio_optimistic(self) -> float:
        """横 / 前向き。MuJoCo は前後の差を表せないので、前向きを前後の値にした楽観側。"""
        return self.mu["lateral"].mean / self.mu["forward"].mean

    @property
    def ratio_conservative(self) -> float:
        """横 / (前向きと後ろ向きの平均)。保守側。"""
        return self.mu["lateral"].mean / mean([self.mu["forward"].mean, self.mu["backward"].mean])


def mu_of(row: dict[str, str]) -> float | None:
    """1 行から μ を出す。値が無い行は None（推測で埋めない）。"""
    method = (row.get("method") or "").strip().upper()
    if method == "T":
        a = (row.get("angle_deg") or "").strip()
        return math.tan(math.radians(float(a))) if a else None
    if method == "P":
        load = float(row["load_g"]) if (row.get("load_g") or "").strip() else None
        f = (row.get("force_kinetic_g") or "").strip() or (row.get("force_static_g") or "").strip()
        return float(f) / load if (f and load) else None
    return None


def load_csv(path: Path) -> list[CouponFloor]:
    groups: dict[tuple[str, str], dict[str, list[float]]] = {}
    with path.open(encoding="utf-8-sig", newline="") as fp:
        for row in csv.DictReader(fp):
            mu = mu_of(row)
            direction = (row.get("direction") or "").strip().lower()
            if mu is None or direction not in DIRECTIONS:
                continue
            key = (row["floor_id"].strip(), f'{row["coupon_id"].strip()}-{row["material"].strip()}')
            groups.setdefault(key, {}).setdefault(direction, []).append(mu)
    return [CouponFloor(f, c, {d: Stat(mean(v), min(v), max(v), len(v)) for d, v in ds.items()})
            for (f, c), ds in sorted(groups.items())]


@dataclass(frozen=True)
class Prediction:
    floor_id: str
    coupon: str
    variant: str                    # optimistic / conservative
    along: float
    across: float
    speed_mm_s: dict[str, float]    # config → 最良の歩容での前進（PHYSICS_SIM）


def predict(cf: CouponFloor, seconds: float = 8.0) -> list[Prediction]:
    fwd, back, lat = (cf.mu[d].mean for d in DIRECTIONS)
    out = []
    for variant, along in (("optimistic", fwd), ("conservative", mean([fwd, back]))):
        prof = BellyProfile(f"H0_{cf.floor_id}_{cf.coupon}_{variant}", along, lat, "H0 実測の μ")
        out.append(Prediction(cf.floor_id, cf.coupon, variant, along, lat,
                              {c: best_gait(c, prof, seconds=seconds).speed_mm_s for c in CONFIGS}))
    return out


def verdict(preds: list[Prediction], variant: str = "conservative",
            min_speed: float = MIN_PATROL_SPEED_MM_S) -> dict[str, str]:
    """床ごとの読み（**提案。正式な Decision ではない**）。各床で最も良いクーポンを見る。

      PROPULSION_OK      … FW5 か FW6_HEADYAW で足りる → Pure Snake 継続、6 本目は Head Yaw 寄りで評価
      NEEDS_BODY_YAW     … FW6_YAW5 でだけ足りる → 推進が律速、6 本目は Body Yaw 寄り
      SNAKE_INSUFFICIENT … どの構成でも足りない → Wheel Belly / Hybrid の候補
    """
    out: dict[str, str] = {}
    for floor in sorted({p.floor_id for p in preds}):
        ps = [p for p in preds if p.floor_id == floor and p.variant == variant]
        best = {c: max(p.speed_mm_s[c] for p in ps) for c in CONFIGS}
        if max(best["FW5"], best["FW6_HEADYAW"]) >= min_speed:
            out[floor] = "PROPULSION_OK"
        elif best["FW6_YAW5"] >= min_speed:
            out[floor] = "NEEDS_BODY_YAW"
        else:
            out[floor] = "SNAKE_INSUFFICIENT"
    return out
