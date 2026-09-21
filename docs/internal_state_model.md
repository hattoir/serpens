# 内部状態モデル — 6 状態のダイナミクス仕様

作成 2026-09-21。実装は `serpens/behavior/internal_state.py`、係数は `config/robot.yaml` の `behavior.internal` /
`behavior.energy` / `behavior.kicks` / `behavior.thermal`。**すべて SIMULATED（人の評価は未）。**

## 1. 共通の式

```
dx/dt = −(x − baseline)/τ + Σ gain_k × 刺激_k          0 ≤ x ≤ saturation
τ = decay_tau_s  (x > baseline: 刺激が消えたあと下がる速さ)
  = rise_tau_s   (x < baseline: 枯れた状態から戻る速さ)
kick(name, amount): 一度きりの出来事で一段動かす（0〜saturation にクランプ）
```

刺激（0〜1）: presence / proximity / approach / touch / novelty / looking / alone / calm。
gains の負値は抑制（inhibition）。刺激が無ければ必ず baseline へ戻る（回復条件の最低保証。
`tests/test_internal_state_recovery.py` が全状態で確かめる）。

## 2. 状態ごとの仕様

| 状態 | baseline | rise_tau | decay_tau | saturation | stimulus_gain | inhibition | recovery_condition | behavior_effect |
|---|---|---|---|---|---|---|---|---|
| **Curiosity** | 0.40 | 20s | 20s | 1.0 | novelty 0.60, alone 0.015 | proximity −0.02 | 刺激なしで 20s で baseline | PATROL・APPROACH の主因。SLEEP を抑える |
| **Affection** | 0.30 | 60s | 60s | 1.0 | touch 0.25, proximity 0.02, looking 0.01 | — | 60s で baseline | ENGAGE・APPROACH を強める（0.5+0.5a） |
| **Stress** | 0.05 | 6s | 6s | **0.9** | approach 0.12（慣れで最大 50% 減）, novelty 0.05; 駆け込み kick +0.45（慣れで減） | — | 6s で baseline。**saturation 0.9 で (1−s) ≥ 0.1**: 社会的な状態が同時に 0 にならない | RETREAT の主因。OBSERVE / APPROACH / ENGAGE を (1−s) で弱める |
| **Attention** | 0.10 | 3s | 3s | 1.0 | presence 0.25, proximity 0.30, looking 0.20 | — | 3s で baseline | OBSERVE の主因、ENGAGE を (0.3+0.7n) |
| **Familiarity** | 0.0 | 60s | **300s** | 0.9 | calm 0.005, proximity 0.004, touch 0.05; RETREAT kick −0.15, 駆け込み kick −0.10; 人の入れ替わりで ×0.5 | — | 誰もいないと 300s で抜ける | Stress の approach gain と駆け込みを (1 − 0.5·fam) 倍、APPROACH / ENGAGE を (1 + 0.3·fam) 倍 |
| **Sleepiness** | 0.10 | 60s | 300s | 1.0 | alone 0.006 | presence −0.05, novelty −0.10 | 人が来れば数十秒で抜ける | SLEEP = (1−p)(1−c)(1−n)(0.3 + 0.7·sleepy) |
| **Energy**（活動） | 1.0 | — | 疲れ tau 120s | — | 歩容が動いている間 fatigue +0.008/s | — | 止まれば 120s で回復 | PATROL・APPROACH を e 倍、COIL_REST_MOOD = (1−e)(1−0.5p) |

Familiarity の入力候補（レビュー）と対応: 同一人物の追跡時間 → calm / proximity の積分（追跡中だけ立つ）、
距離 → proximity、安全なやりとり → calm（Stress ≤ 0.3 かつ近づいて来ない）、接近履歴 → proximity の積分、
退避 → kick −0.15、タッチ → +0.05/s、人の入れ替わり → ×0.5。**単純なタイマーではない。**

## 3. 温度（機械の状態）は Energy に混ぜない

- サーボ最高温度は **Safety layer**: `behavior.safety.overheat_c`（55℃）で COIL_REST_HEAT を強制（人が来ても抜けない）。
  さらに手前の `behavior.thermal.rest_request_c`（45℃）から Behavior 側へ「休ませたい」を出し、COIL_REST_MOOD の効用に
  `rest_request_boost`（0.8）を足す。
- **Energy（キャラクター）は活動由来だけ。** 旧実装の min(温度由来, 活動由来) はやめた。
- 温度の上限（60℃ で機体側 EMERGENCY）は `link.faults.temp_limit_c`。

## 4. 排他的ゼロ抑制の監査（`tests/test_internal_state_recovery.py`）

| 検査 | 内容 |
|---|---|
| 回復 | 全 6 状態 + Energy: 飽和 / 0 から刺激なしで 5τ 以内に baseline の ±5% へ |
| 固定されない | 全状態 × {飽和, 0} × 5 文脈（誰もいない / 遠い / 近い / 撫で / 新奇）で、選ばれる行動が文脈で変わる |
| 社会的な状態 | 人が近い・撫での文脈では正の効用が 2 つ以上 |
| Stress 飽和 | ENGAGE / OBSERVE > 0 のまま。RETREAT が勝つのは正しいが、3τ で ENGAGE が勝ち返す |

Bug-1 の構造（Stress 1.0 → (1−s)=0 → APPROACH/ENGAGE/OBSERVE 同時 0 → RETREAT 独走）は、
gain の修正だけでなく **saturation 0.9** で構造的に起きなくした。
