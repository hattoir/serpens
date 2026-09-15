"""腹側（Belly）ごとの接触設定。**摩擦係数はすべて未実測（HARDWARE_UNVERIFIED）。**

MuJoCo の接触は摩擦係数を5つ持つ（接線2・ねじり1・転がり2）。接線の2方向に別の値を
入れられるので、鱗の腹の異方性（前後に滑り、横に滑らない）をそのまま表せる。
出典: https://mujoco.readthedocs.io/en/stable/modeling.html

  WHEEL             … 受動輪。横は滑らず、前後はよく転がる
  SNAKE_ISOTROPIC   … 鱗なしの等方摩擦（比較用の基準）
  SNAKE_ANISOTROPIC … 鱗あり。前後 < 横（異方性）

**接触フレームの接線方向がどちらを向くかは MuJoCo の実装依存**なので、
「異方性を入れたら前へ進むか」は実行して確かめる（`tests/test_mujoco_belly.py`）。
値を現実の摩擦として扱わないこと。
"""
from __future__ import annotations

from dataclasses import dataclass

TORSION = 0.005        # ねじり摩擦（MuJoCo 既定 0.005 のまま）
ROLL = 0.0001          # 転がり摩擦（同上）


@dataclass(frozen=True)
class BellyProfile:
    """床と腹の摩擦。`slide_along` は体の長手方向、`slide_across` は横方向。"""

    name: str
    slide_along: float
    slide_across: float
    note: str

    @property
    def friction(self) -> tuple[float, float, float, float, float]:
        """MuJoCo の5係数（接線2・ねじり1・転がり2）。"""
        return (self.slide_along, self.slide_across, TORSION, ROLL, ROLL)

    @property
    def anisotropy(self) -> float:
        """横 / 前後。1.0 で等方。大きいほど「前へ進みやすく横へ滑りにくい」。"""
        return self.slide_across / self.slide_along if self.slide_along else float("inf")


PROFILES: dict[str, BellyProfile] = {
    # 受動輪: 転がり方向はほぼ抵抗なし、横はゴムのグリップ
    "WHEEL": BellyProfile("WHEEL", 0.02, 1.0, "受動輪14個の近似（車輪を個別に置かない）"),
    # 鱗なし（つるりとした腹）。前後も横も同じ
    "SNAKE_ISOTROPIC": BellyProfile("SNAKE_ISOTROPIC", 0.35, 0.35, "等方。推進が成立するかの基準"),
    # 鱗あり。前後に滑り、横に食いつく
    "SNAKE_ANISOTROPIC": BellyProfile("SNAKE_ANISOTROPIC", 0.12, 0.80, "鱗の異方性（仮の比 6.7）"),
}


def profile(name: str) -> BellyProfile:
    if name not in PROFILES:
        raise KeyError(f"未知の belly プロファイル: {name}（{list(PROFILES)}）")
    return PROFILES[name]


def sweep_profiles(along: list[float], across: list[float]) -> list[BellyProfile]:
    """摩擦の掃引用（実機で試す候補を絞るため）。"""
    return [BellyProfile(f"SWEEP_{a:g}_{c:g}", a, c, "掃引") for a in along for c in across]
