# Decision candidate DEC-CAND-DESIGN-0001 — 外観の方向 SD-01「Bean head ＋ 同心ナックル」

- Project: serpens / Agent: Design / Run: DESIGN-2026-09-29-01 / Date: 2026-09-29
- **状態: 提案（正式 Decision ではない）。** Engineering の確認（integration-log ENTRY-0005〜0011）と User の判断が要る
- 根拠の詳細: `docs/design/concepts_2026-09-29.md`、数値: `docs/design/results/concept_metrics_2026-09-29.json`（DESIGN_ESTIMATE、HUMAN_EVALUATED = 0）

## Decision（案）

1. 頭は H1 Bean: 幅 100（胴 92 より広い）× 高さ 74 × 長さ 78、平らなあご・丸い鼻先、目 D24。センサーを顔の造作に割り当てる（カメラ = ボタン鼻、前 ToF = 鼻孔、斜め照明 = 下唇の影、ライン光 = 眉）
2. 首を幅 64〜68 に絞る（頭 > 首 > 胴の順に読める）。6 本目が頭 yaw なら首に置く
3. 胴は B1 同心ナックル（関節軸と同心の面だけで向き合い、すき間 4 mm 一定。背板＝大きな鱗）
4. 色は背 sage・腹 ivory（側面の下 40 %）・背線クリーム、つや消し

## Why

- FW03 は頭 76 < 首 88 < 胴 92 で、蛇の最大の手がかり（頭 > 首）と赤ちゃん図式に反していた
- FW03 の卵殻は関節のすき間が 8〜25 mm の挟み込み域を通って閉じる（2D 計算で 10〜40°）。B1 は 4 mm 一定
- 頭 +30 g でも J1 静的トルクは +0.003 N·m（0.020 N·m、上限 0.45 の 4.4 %）

## Alternatives

H2 Wedge / H3 Cyclops / H4 Visor、B2 重ね鱗 / B3 TPU ベローズ / B4 ニット（将来の研究候補）。不採用の理由は concepts の §1.2・§2.2

## Trade-offs / いつ間違いになるか

- B1 は各関節の後ろ 0〜46 mm の中段を前の節の円柱にするので、FW03 の電装位置と衝突する（ENTRY-0011）。電装を動かせないなら段付きか別案
- R04 上ループ配線とは両立しない（R05 横通し＋関節の渡りが前提）
- スコアは Design Agent の見立て。人の評価で覆りうる（DEC-CAND の前に EXP-DESIGN-0001）
