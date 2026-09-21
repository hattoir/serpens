# EXHIBITION profile と Fear 監査

作成 2026-09-21。`config/profile_exhibition.yaml`（`robot.yaml` への上書き）。**SIMULATED。人の評価は未（HUMAN_EVALUATED = 0）。**

## 1. 体験弧（60〜90 秒）

固定スクリプトではない。Utility AI / 内部状態 / grammar / primitives の係数だけで、次の弧が**起きやすい**ようにした
（`tests/test_exhibition_profile.py`、3 seeds）。

| 段 | 何が起きるか | 仕組み | 模擬での時刻（来場者 5.0s に出現） |
|---|---|---|---|
| NOTICE | 目の点灯 + J8 が来場者の側へ 2.5° ピクッ | grammar `on_notice`（150〜250ms） | 5.2〜5.3s（BEHAVIOR_INTERNAL） |
| LOOK | 全停止（呼吸は続く）→ ALERT で頭を向ける | `reaction_delay_s` → `_react` → look_at | 5.5〜5.8s |
| HESITATE / TRACK | OBSERVE: 動く来場者を頭で追い、舌のちらつき・かしげ・視線そらし | ALERT dwell 2.5s、OBSERVE 重み 1.3、APPROACH × (1 − 0.9·novelty) | 8.0〜8.5s |
| APPROACH | 落ち着いて見ている間に好奇心が育ち（calm gain）、自分から忍び寄る（stalk、途中で一度止まる） | curiosity calm +0.03/s、APPROACH 重み 1.7 | 16〜20s |
| OBSERVE | 途中の一時停止「様子を見る」 | locomotion の 2 段階接近 | — |
| ENGAGE | 450mm 以内でかかわる | ENGAGE | 22〜27s |
| RELAX | 10〜16 秒後に力を抜き頭を少し下げる（トルク 70%、8 秒） | grammar ENGAGE `sag` | 33〜39s |
| LEAVE / REST | 来場者が去る → 巡回（疲れていれば休憩） | utility | 58s |

「こいつ、俺を見てる？」の瞬間 = NOTICE〜LOOK（出現から 0.2〜0.8s）。

## 2. Fear 監査（人へのやりとり中に威嚇的に見える動き）

| 動き | 判定 | 処置 |
|---|---|---|
| s_curve（S 字構え） | 打撃直前の構えに見える | `legacy_poses` へ。土台は arc。**使わない** |
| rear_up | 「威嚇」でなく「見上げる」として扱う | 人を追跡中は J7 ≤ `neck.look_max_deg`（65°）を `Primitives.play` の guard が強制。ALERT 55° / ENGAGE 58° |
| full_rear_up（伸び） | コブラの直前に見える | 誰もいない巡回の静止中だけ（grammar `only: alone_paused`）。人がいる状態の grammar には無い |
| fast head turn | 速い首振りは威嚇 | `look_duration_s` 0.8s、先端加速度 ≤ 1G、5° 以上は立ち上がり ≥ 200ms |
| sudden acceleration | 急加速 | anticipate / settle、歩容の blend、stalk 歩容（f 0.25） |
| direct charge | 直進突撃 | 接近は 40mm/s の忍び寄り + 途中で一度止まる + 400mm 手前で必ず停止 |
| 駆け込まれたとき | RETREAT（後退）。逃げるが威嚇はしない | Stress kick（慣れで弱まる） |

**動画では評価できないもの**（大きさ・音・振動・接触・距離感・見られている感覚）は
`PHYSICAL_HUMAN_EVALUATION` で別に測る。

## 3. 当日の「人格」のつまみ（config 1 行）

| もう少し… | 変える場所 |
|---|---|
| 臆病に | `utility.approach_novelty_hesitation` ↑、`stimuli.rush_stress_kick` ↑、`internal.stress.gains.novelty` ↑ |
| 積極的に | `utility.weights.APPROACH` ↑、`internal.curiosity.gains.calm` ↑ |
| よく見つめる / よそ見 | `expression.look_hold_s`、grammar `glance_away.every` |
| 落ち着き | `expression.flick.base_rate_per_min`、`breath.period_s` |
| 休みがち | `energy.fatigue.gain_move`、`thermal.rest_request_c` |
