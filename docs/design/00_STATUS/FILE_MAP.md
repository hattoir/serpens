# 成果物の場所（`serpens/` の中）

> 先頭の注意: KNUCKLE DRUM 後も、まっすぐ比で約 20 倍（18〜24 倍）のすき間の体積が残る。

## レポート（`docs/design/`）
- `gap_check_report2_2026-09-30.md` — **最新**のすき間チェック（r の範囲、KNUCKLE DRUM、J1 ピッチ、限界）
- `gap_check_report_2026-09-30.md` — 前の報告（一部を訂正済み）
- `mass_addendum_2026-09-30.md` — 質量（中実 / 中空）
- `concepts_2026-09-29.md`、`sd01_variants_and_fallbacks_2026-09-29.md`、`csar_choreography_2026-09-29.md`、`child_anthropometry_sources_2026-09-29.md` — 設計の検討
- `intake_study/HEAD_INTAKE_STUDY01_report.md` — くちばしの取り込み機構

## 印刷用 STL（印刷は User）
- `knuckle_test/`（README、`print/`: R46・R34 の板・玉・椀、探針）
- `test_piece_b/`（README、`print/`: フード、リップ、スペーサー、クランプ棒、スカート TPU、`b1_check.png`）
- `intake_study/print/`（くちばしの試験片 A）

## 数字・図
- `results/gap_rad_v3_base_*`（カバーなし）、`gap_rad_v3_skirt_*`（フラップ）、`gap_rad_v5_drum_*`（KNUCKLE DRUM）、`gap_pitch_v3.json`（J1）、`knuckle_drum_mass_2026-09-30.json`
- `assets/gap_drum_before_after.png` ほか

## ツール（`docs/design/tools/`）
- `gap_check.py`（検査）、`gap_radial.py`（r の範囲）、`gap_pitch.py`、`flank_skirt.py`（試した案のモデル）、`fusion_knuckle_piece.py` / `fusion_hood_b.py`（Fusion で試験片を作るスクリプト）

## やりとり（`ai-shared/`、`ai-outbox/`）
- `ai-shared/design-state.md`（Design の状態）、`integration-log.md`（ENTRY-D-0001〜0005）、`open-questions.md`（OPEN-SERPENS-DESIGN-014 ほか）
- `ai-outbox/handoffs/`（0001〜0006）、`lessons/`、`decisions/`

## Fusion（Autodesk、Serpens フォルダ）
- `Serpens_DESIGN_SD01_BEAN_KNUCKLE_RECOVERY_20260929`（作業の複製。KNUCKLE DRUM、埋め物、試験片の組立）
- `Serpens_FW02_HEAD_INTAKE_STUDY01_NOT_FOR_PRINT`（試験片 A・B1）
- 元のファイル（`…CONCEPT_NOT_FOR_PRINT`、FW02 v5）は**未変更**

## Git
- 枝 `agent/design-floor-watch`（`https://github.com/hattoir/sesrpens.git`）。main には merge していない
