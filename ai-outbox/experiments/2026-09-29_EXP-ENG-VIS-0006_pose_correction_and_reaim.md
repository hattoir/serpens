# EXP-ENG-VIS-0006 — 姿勢の補正を検出に入れ、mission の狙い直しを線の本当の位置で行う

- Date: 2026-09-29 / Branch: `agent/engineering-vision-sim` / Source: **SYNTHETIC_VISION_SIM**
- Code: `serpens/floorwatch/pose.py`、`serpens/floorwatch/detect.py`（`_pose_corrected`・`_merge_by_shadow`・`line_x_mm`）、
  `serpens/floorwatch/mission.py`（`_aim_delta_deg`）、`simulation/h2_vision.py`（`reaim`・`pose_from_line`）
- Tests: `tests/test_floorwatch_pose.py`、`tests/test_floorwatch_mission_aim.py`、`tests/test_h2_vision.py`（すべて修正を外すと落ちることを確認）

## Hypothesis
VIS-0005 に残った「姿勢のずれ → 位置の誤差 11〜32mm → 線が外れて metal_disc が落ちる」は、床の線から姿勢を推定して位置を直し、
mission がその位置へ線を向け直せば消える。

## Findings
1. **名目の姿勢で線を探すと、ずれの大きい側（近い半分）で線が探す窓から外れ、点の約半分が外れ値**になっていた（推定が −5mm に対し −3.9mm）。
   → 最初の推定を最小トリム二乗（良い半分）で行い、**直した姿勢の線を中心に探し直す** → ±0.02mm / ±0.01°、残差 0.17px。
   ボタン電池の手前の縁が 5 通りの姿勢すべてで (0.1, 65.0)mm（真値 (0, 65)、以前は 9〜16mm ずれ）
2. **mission の狙い直しは線を常に x=0 とみなしていた**。姿勢がずれると床の線そのものが横へ動く（高さ −5mm で約 +5mm）。
   また「線が当たっていない」候補しか向け直さなかった（当たっていても 4mm ずれると metal_disc は半分）。
   → 候補に `line_x_mm`（その前後位置での線の横位置）を持たせ、ずれ = 候補 − 線。当たっていても `line_aim_tol_mm`(2.0) 以上ずれ、
   決め手（metal_disc・測れた高さ）が無ければ向け直す
3. **10×3mm の磁石が 3mm の小片 2 つに割れていた**（上面が床に紛れ、左右の側面だけ）→ 同じ影の塊が真上にある小片は 1 つの物としてつなぐ（9.6〜10.1mm に）。
   ただし**つないだ結果が物でなくなるなら元の小片を残す**（ガードを入れる前はカーペットの巡回が 1.00 → 0.73 に落ちた）
4. trace_line の床の線の予測を、行ごとの二分法から「投影した直線」に（1e-13px で一致）。3 回探しても検出 0.2〜0.3 秒（UXGA×0.5）

## Result（Monte Carlo 300 条件、mission と同じく向け直しあり）
| | 朝 | VIS-0005 | **VIS-0006** |
|---|---|---|---|
| patrol 検出 | 0.89 | 0.99 | **0.99** |
| critical 検出 | 0.94 | 1.00 | **1.00** |
| 鏡面の危険物が metal_disc | 0.50 | 0.58 | **0.82** |
| 誤報 patrol / inspect | 0.21 / 0.70 | 0.10 / 0.22 | **0.09 / 0.18** |
| 位置の誤差（中央、手前の縁 − 中心） | — | 10.9mm | **5.9mm**（= ほぼ半径ぶん。姿勢のずれの影響は消えた） |

## Remaining
- 高さ 20mm 以上は巡回で見つからない（物自身が影を隠す）。斜め LED で物の前面が明るくなる手がかり → 描画の照明模型から
- ラグ・カーペットの誤報 0.11〜0.22 は線の途切れ（暗い繊維）。実写（HA-04）で合わせる
- 眉の線にする場合: `line_x_mm` は前後に走る線の式。眉では前後位置（`line_y_mm`）で J1 pitch の狙い直しになる（式は同じ形）
