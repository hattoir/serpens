# Handoff — Design（DESIGN-2026-09-29-01）

- **Git branch**: `agent/design-floor-watch`（worktree: Claude の scratchpad 内 `serpens-design-wt`。共有ツリー `serpens/` の HEAD は Design が動かさない）
- **Latest commit**: `dd9369b` design: explore floor watch head and body concepts（この handoff と run summary は次の commit）
- **Changed files**: run summary の「Files changed」を参照

## 現在の Design

SD-01「Bean head ＋ 同心ナックル」＋ CSAR。状態の正本は `ai-shared/design-state.md`、根拠は `docs/design/concepts_2026-09-29.md`。

## 今回変更したこと

- 頭 5 案・胴 5 案を比べ、数値（質量・J1・視錐台・目の見え方・挟み込み・ライン光の遮り）を出した
- 振る舞いを CSAR（子どもがいる間は危険物を見ない・指さない・目立たせない、離れる、保護者へ、子どもの注意を引き受ける）に組み直した
- Fusion に SD-01 の概念モデルを一から作った（関節つき、姿勢の描き出しあり）

## Engineering Agent へ伝えること

- ENTRY-0004: 共有ツリーで HEAD を取り合った。commit の前に branch を確認すること
- ENTRY-0011: B1 だと関節の後ろ 0〜46 mm が前の節の空間になり、FW03 の電装配置と衝突する。どこまで動かせるかを知りたい
- ENTRY-0006: ライン光の眉位置は顔の面から 6 mm 以内（縦基線は FW03 とほぼ同じ）。撮影中は表現用 LED を全部消したい
- ENTRY-0008: CSAR は安全に関わるので、実行条件の設計と Human Approval が必要

## 未解決事項

OPEN-SERPENS-DESIGN-001〜009、ENTRY-0005〜0011 の返答待ち、Fusion の未保存の変更

## User 判断が必要な点

CSAR の採用 / 発見後に残るか / 6 本目の使い方 / worktree の運用 / design branch の push

## 次に試す案

背板を胴と同じ高さに作り直す → 段付きナックル（中段 r ≈ 32 ＋ 外側 r46）→ 頭 yaw を首に置く意匠 → 静止画の予備評価（EXP-DESIGN-0001）
