# H2 視覚: 1 因子ずつ（SYNTHETIC_VISION_SIM。112 試行、0.4 分、解像度 UXGA×0.5、1 水準 = 対象 11 × 4 回 + 陰性 3 × 4 回）

名目: 木目・模様 1.0・凹凸 0・ぼけ 0・環境光 8・線 3mm・斜め照明 6mm・カメラのずれ 0・狙いの誤差 0・側面あり。
critical = ボタン電池・磁石・錠剤。flagged = 鏡面の危険物が metal_disc（危険物側）に回った割合。

| 因子 | 水準 | patrol 検出 | inspect 検出 | critical inspect | 鏡面 critical flagged | 誤報 patrol | 誤報 inspect | 高さ誤差 中央 mm | 大きさ誤差 中央 | 位置誤差 中央 mm |
|---|---|---|---|---|---|---|---|---|---|---|
| physical_falloff | False | 1.00 | 1.00 | 1.00 | 1.00 | 0.00 | 0.00 | 2.24 | 0.08 | 5.90 |
| physical_falloff | True | 0.82 | 1.00 | 1.00 | 0.94 | 0.00 | 0.17 | 2.50 | 0.25 | 5.33 |

## 対象ごとの inspect 検出（因子 × 水準）

| 因子 | 水準 | coin_1yen | coin_10yen | button_cr2032 | button_lr44 | bead_6 | bead_10_dark | magnet_disc_10x3 | magnet_ball_5 | pill_8 | plastic_part_8 | floor_colored_12 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| physical_falloff | False | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| physical_falloff | True | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
