# Human Perception Pilot（動画）— 手順・尺度・データ形式

作成 2026-09-21。**HUMAN_EVALUATED = 0（まだ誰にも見せていない）。** 結果の source は `VIDEO_HUMAN_EVALUATION`。
動画では、実際に近づいた時の恐怖・大きさ・音・振動・接触・距離感・見られている感覚は評価できない
（それは将来 `PHYSICAL_HUMAN_EVALUATION` で分ける）。

## 1. 目的（合格 / 不合格を決めない）

n = 8〜12 で、Baseline の取得・尺度の問題の発見・大きな差の発見・Fear の確認。
6→8 で大きな変化があるか、8→10 でさらに変化するか、Fear が上がっていないか、Snake Comfort との関係、
Animacy と Affection が同時に上がるか。**小さな差を確定的な結論にしない。**

## 2. 素材を作る

```powershell
.\.venv\Scripts\python.exe tools\body_compare.py                 # 6/8/10 の数値（KINEMATIC_SIM）
.\.venv\Scripts\python.exe tools\pilot_clips.py --seed 1         # 匿名クリップ clip_A.avi … + clip_key.json
.\.venv\Scripts\python.exe tools\pilot_order.py --participants 10 --seed 42   # 被験者ごとの提示順
```

- 条件は `pilot/conditions.json`（RAW と SPEED_MATCHED、同等身体長 / 同等リンク長、体験弧）。
- 全クリップ: 同じカメラ・背景・12 秒・開始位置・ターゲット位置・表示スケール。**条件名は画面に出ない。**
- `output/pilot/clip_key.json` は**被験者に見せない**（匿名 ID ↔ 条件、割り当ての seed）。
- `output/pilot/presentation_order.json` に提示順と seed（順番効果を減らす）。

## 3. 実施

1. 被験者に匿名 ID（P01…）を割り当てる。個人を特定する情報は取らない。
2. 最初に Snake Comfort を 1 問: 「あなたは普段、蛇がどの程度苦手ですか」（1 = とても苦手 … 7 = 全く平気）。
3. `presentation_order.json` の順に各クリップを見せ、直後に 6 項目を 1〜7 で答えてもらう。
4. 任意でコメント。
5. 回答は `pilot/response_template.csv` の列で 1 行 = 1 被験者 × 1 クリップ。

## 4. Serpens 独自尺度（1 = 全くそう思わない … 7 = とてもそう思う）

| 列 | 質問 |
|---|---|
| snake_likeness | 蛇らしい動きに見える |
| animacy | 生きているように感じる |
| approachability | 近づいてみたい |
| affection | かわいい／愛着を持てそう |
| fear | 怖い／距離を取りたい |
| smoothness | 動きが自然で滑らか |
| snake_comfort | （最初に 1 回）あなたは普段、蛇がどの程度苦手ですか |

## 5. Godspeed（使う場合）

`pilot/godspeed_items.json` の語対をそのまま使い、**独自尺度と混ぜない・改変しない**。
raw response は `pilot/godspeed_responses.csv`（1 行 = 被験者 × クリップ × 項目）に保存する。研究用途へ発展させる可能性があるため。

## 6. データ形式（`pilot/response_template.csv`）

`participant_anonymous_id, presentation_order, clip_id, random_seed, snake_comfort, snake_likeness, animacy,
approachability, affection, fear, smoothness, optional_comment`

## 7. 集計

```powershell
.\.venv\Scripts\python.exe tools\pilot_analysis.py output\pilot\responses.csv --key output\pilot\clip_key.json
```

median / mean / 分布 / 被験者内差 / Fear の最大 / Snake Comfort と Fear の順位相関 / Animacy と Affection の順位相関。
結果は `docs/body_configuration_decision_template.md` の表へ写す。**Pilot 前に 8 軸が最適などと決めない。**
