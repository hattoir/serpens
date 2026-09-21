"""検証レベルと結果の出どころ（このリポジトリの「確認済み」の語彙）。

  SOFTWARE_VERIFIED   ソフトの論理として自動試験で確かめた（物理は含まない）
  KINEMATIC_SIM       簡易シミュレータ（車輪の横滑りゼロ拘束・指令角そのまま）でそう動いた
  PHYSICS_SIM         MuJoCo（接触・摩擦・サーボの一次遅れ）でそう動いた。摩擦もゲインも未同定
  HARDWARE_VERIFIED   実機で測った（日付・条件・測定器つき）。**現在 0 件**
  HUMAN_EVALUATED     人が見て評価した。**現在 0 件**（動画 = VIDEO_HUMAN_EVALUATION / 実物 = PHYSICAL_HUMAN_EVALUATION）

Snake-likeness / Animacy / Approachability / Affection / Fear は Simulation だけでは合格判定しない。
motion_quality の指標は REGRESSION_METRIC / SIMULATION_DIAGNOSTIC であって、人の評価の代替ではない。
一次反応レイテンシの出どころ: BEHAVIOR_INTERNAL（PC 内） / END_TO_END_SIMULATED（カメラ予算込みの推定） /
HARDWARE_MEASURED（実機。未）。
"""
from __future__ import annotations

from enum import Enum


class Level(str, Enum):
    SOFTWARE_VERIFIED = "SOFTWARE_VERIFIED"
    KINEMATIC_SIM = "KINEMATIC_SIM"
    PHYSICS_SIM = "PHYSICS_SIM"
    HARDWARE_VERIFIED = "HARDWARE_VERIFIED"
    HUMAN_EVALUATED = "HUMAN_EVALUATED"


class HumanSource(str, Enum):
    VIDEO_HUMAN_EVALUATION = "VIDEO_HUMAN_EVALUATION"          # 動画を見た評価（大きさ・音・接触・距離感は評価できない）
    PHYSICAL_HUMAN_EVALUATION = "PHYSICAL_HUMAN_EVALUATION"    # 実物の前での評価（将来）


class LatencySource(str, Enum):
    BEHAVIOR_INTERNAL = "BEHAVIOR_INTERNAL"          # 行動エンジン内（人の座標が入ってから可視変化まで）
    END_TO_END_SIMULATED = "END_TO_END_SIMULATED"    # + カメラの検出周期と遅れ（config の想定値）
    HARDWARE_MEASURED = "HARDWARE_MEASURED"          # 実カメラ → 実機の可視変化（未測定）


# 「Simulation だけでは合格にしない」評価軸
HUMAN_ONLY_AXES = ("snake_likeness", "animacy", "approachability", "affection", "fear", "smoothness")
HARDWARE_VERIFIED_COUNT = 0
HUMAN_EVALUATED_COUNT = 0
