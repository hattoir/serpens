# 動きの質（蛇らしさ・愛着）— 外部レビューへの対応と測定

作成 2026-09-21。**すべて SIMULATED / SOFTWARE_VERIFIED。実機で見た人は一人もいない。**
文献の数値は外部レビューが出典つきで示した実測値。本機の数値は `tools/motion_quality.py`（KINEMATIC_SIM）。

## 1. 致命バグ（動きより先に直した）

| | 症状 | 直し | 回帰テスト |
|---|---|---|---|
| Bug-1 | Stress の approach gain 0.90 → 平衡値が approach 0.18 で 1.0 に飽和し、RETREAT だけが残る | gain 0.12（平衡 0.77）。駆け込みは別の出来事（`rush_mm_s` 超で Stress を一段上げる） | 歩いて来る人から逃げない / 駆け込みには逃げる |
| Bug-2 | Energy が温度だけで決まり、冷えている間は気分の休憩（COIL_REST_MOOD）が一度も起きない | 活動由来の疲れ `fatigue`（gain_move 0.008、tau 120s）。energy = min(温度, 1 − fatigue) | 5 分の間欠移動で休憩が巡回に勝つ |
| Bug-3 | 効用ノイズ ±8% は hysteresis 1.15 をぎりぎりしか超えない | ±15% | 最大比 ≥ 1.15 × 1.15 |

## 2. 実装したもの（文献 → 本機）

| 項目 | 文献 | 本機 | 場所 |
|---|---|---|---|
| stop-and-go | 間欠移動の静止 約 50%（Integr. Comp. Biol. 41(2):137） | 巡回: 動く 2〜4s / 止まる 3〜7s。静止中も呼吸 | `behavior/locomotion.py` |
| 舌のちらつき | 3〜9 回/分、novelty で増える（Chiszar et al.） | J8 ±3〜8° を 0.2〜0.4s で往復 | `primitives.flick`, grammar `rate_per_min: novelty` |
| 忍び寄り | rectilinear 0.02〜0.07 BL/s（PMC3673153） | 接近は stalk 歩容 A8° f0.25 = 0.036 BL/s | `gait.presets.stalk` |
| 威嚇姿勢の封印 | 防御段階 ②威嚇の S 字（California Herps） | rear_up の土台 arc、s_curve は legacy、フル鎌首は人がいないときの「伸び」だけ。guard が J7 ≤ look_max | `primitives.play` |
| 頭の加速度上限 | 打撃 506 m/s²・35ms（Higham 2017） | 先端 1G 以下、5° 以上は立ち上がり 200ms 以上 | `animator.head_*` |
| 一次反応 | 因果の窓 140ms〜1s（Front. Psychol. 14:1167809） | 150〜250ms で目の点灯 + J8 2.5° のピクッ。「間」はその後 | grammar `on_notice` |
| 呼吸 | 注視 +10.65s、やりとり +27.68s（IJSR 2026） | 位相勾配 30°/軸、J7 5°・胴体 1.5〜2°、驚きの全停止中も続く | `breath` |
| anticipation / follow-through | 魅力 4.27→4.83、知的印象 3.86→4.72（Takayama 2011） | Keyframe の anticipate（3〜8°、0.1〜0.3s 前）と settle（尾側 3 関節 5°→0、τ 0.3s） | `motion/easing.py` |
| 視線 | 相互凝視 平均 3.3s、GAR 0.5〜0.7（Binetti; Front. Robot. AI 10:1062714） | hold 2.5〜4s、4〜8s ごとに 2.5〜4.5s そらす | `primitives.glance_away` |
| 撫で 3 段 | Paro（Sci. Rep. 10:9747） | 呼吸を止める（≤200ms）→ 脱力 40%（0.3〜0.8s）→ すり寄る 10〜20°（1〜1.4s） | grammar `PETTED` |
| 語彙 / 文法 | — | primitives（状態を知らない）+ grammar（config の表） | `behavior/grammar.py` |

## 3. 測定（`tools/motion_quality.py --seeds 3`、2026-09-21、KINEMATIC_SIM）

**位置づけ: REGRESSION_METRIC / SIMULATION_DIAGNOSTIC。** PRODUCT SUCCESS METRIC ではなく、人の評価
（`docs/human_pilot.md`、HUMAN_EVALUATED = 0）の代替でもない。人の評価との相関が取れたときだけ代理指標として再評価する。
一次反応の 0.22s は BEHAVIOR_INTERNAL（人の座標が行動へ入ってから）であり「人検出から 220ms」ではない。

| 指標 | 目標 | 旧実装 | 今回 | 判定 |
|---|---|---|---|---|
| 静止率（巡回中、歩容の振幅がゼロの割合） | 0.40〜0.60 | ほぼ 0 | **0.44** | 内 |
| 一次反応レイテンシ（模擬。カメラの遅れを除く） | ≤ 0.30s | 1.4〜2.1s | **0.22s** | 内 |
| 同、カメラの予算を足した見込み（detect 5Hz + 遅れ 0.1s） | — | — | 0.22 + 0.30 = 約 0.52s | **窓の外側**。実カメラで検出周期を上げる必要 |
| GAR | 0.40〜0.60 | 約 0.04 | **0.51** | 内 |
| 可視波数（forward） | 2 | 1 | **1.3** | 構造的制約（6 軸 × 60° = 1 波） |
| 頭部軌道の LDJ（120s） | 相対比較 | — | −25.7 | 帯 [−30, −20] で回帰 |

8 軸案（`config/robot_yaw8.yaml`、Ω=90°）: 可視波数 **2.2**、前進 199mm/周期（1 波の Ω=45° なら 519mm）。
2 波にすると簡易シミュレータでは前進量が約 4 割になる。**波数と速さのトレードオフ**はハード側と共有する。

## 4. 正直に書くこと

- 上の数値は指令角と簡易シミュレータの値。**サーボの遅れ・摩擦・実カメラの遅れは入っていない。**
- 一次反応は PC 内で 220ms だが、カメラ経路（5Hz 検出 + 100ms）を足すと 300ms を超える。
  展示では `person.detect_hz` を上げるか、目の LED を検出直後に光らせる経路（ESP32 側）が要る。
- 「蛇らしさ」「愛着」は最終的に人が見て決める。ここの指標は**壊したときに気づくため**の回帰。
- 接触方向は頭部 2 点のタッチでは分からない。すり寄る向きは追跡中の人の方向（胴体タッチはハード側へ要求済み）。
