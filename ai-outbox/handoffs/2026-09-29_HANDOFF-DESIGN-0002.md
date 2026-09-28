# Handoff — Design（DESIGN-2026-09-29-01、User Decision 反映後）

HANDOFF-DESIGN-0001 の続き。User Decision **USER-DEC-SERPENS-DESIGN-0001**（`ai-outbox/decisions/`、integration-log ENTRY-0012）を反映した。

- **Git branch**: `agent/design-floor-watch`（Design worktree。User Decision 4 により Design はこれ以外の branch を切り替えない）
- **Latest commit / push**: この handoff を含む commit と push の結果は run の最終報告に書く
- **Changed files（この更新）**: `ai-shared/design-state.md`、`ai-shared/integration-log.md`（ENTRY-0004/0007/0008/0009/0011 の Status、ENTRY-0012 追加）、
  `ai-shared/open-questions.md`（OPEN-003/004/005 の Status）、`ai-outbox/decisions/*`（USER-DEC 追加、DEC-CAND の状態）、この handoff

## 現在の Design

- SD-01「Bean Head ＋ 同心ナックル」を**継続検討**（CAD_CONCEPT / KINEMATIC_SIM。4 mm 一定は安全確定ではない）
- 振る舞いは **CSAR（User 採用）**: 見つける → 再確認 → 位置登録 → 少し離れる → 保護者へ通知。子どもが近ければ子ども側を見る。危険物のそばのとぐろは MVP 不採用
- 6 本目は**暫定 Head Yaw**（H0 摩擦試験しだいで Body Yaw を再検討）

## Engineering Agent へ伝えること

1. CSAR の実行条件（子どもの近さ・不明の扱い、位置登録の後に離れる動き、highlight_point の扱い）の設計をお願いしたい（ENTRY-0008 / 0012）
2. H0 摩擦試験の結果が 6 本目の決定に直結する（ENTRY-0001 / 0009）
3. ENTRY-0011（電装配置とナックル）は回答待ち。Design は段付きナックル（r ≈ 32）を案として持つだけで確定しない
4. worktree 分離が正式方針になった（ENTRY-0004 / 0012）

## 未解決事項

- **Fusion**: `Serpens_DESIGN_SD01_BEAN_KNUCKLE_CONCEPT_NOT_FOR_PRINT` の API が応答しないまま。最後の正常保存はジョイント追加の直後
- ENTRY-0005 / 0006 / 0007 / 0010 / 0011 の Engineering 回答、OPEN-SERPENS-DESIGN-001 / 002 / 003 / 006〜009

## User 判断が必要な点

- Fusion の復帰（Fusion 側のダイアログ・計算中かの確認）
- 残りの OPEN（ライン光の位置は Engineering、音・停止ボタン・尾の先・色は評価のあと）

## 次に試す案（この順）

1. Fusion 復帰 → 状態を読み取り専用で確認 → **Recovery copy を別名保存**（元ファイルは最後の正常保存版のまま）→ Recovery copy 上で背板を胴と同じ高さに作り直す
2. 暫定 Head Yaw の首の意匠（首に頭 yaw サーボ、Body Yaw に戻っても頭・首を流用できる形）
3. 静止画の予備評価 EXP-DESIGN-0001 の素材
4. 将来の実験候補: 遮蔽としてのとぐろの遮蔽効果と誘導リスク（MVP 外）
