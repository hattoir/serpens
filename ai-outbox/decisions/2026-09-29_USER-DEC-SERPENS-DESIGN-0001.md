# USER-DEC-SERPENS-DESIGN-0001 — Design Run DESIGN-2026-09-29-01 に対する User Decision

- Project: serpens / Date: 2026-09-29 / Decided by: **User**（Design Agent が記録）
- 対象: DEC-CAND-DESIGN-0001（SD-01）、DEC-CAND-DESIGN-0002（CSAR）、integration-log ENTRY-0004 / 0007 / 0008 / 0009 / 0011

| # | Decision |
|---|---|
| 1 | **CSAR を採用。** 危険物を見つけてもその場で見せびらかさない。再確認と位置登録の後、基本は危険物から少し離れる。子どもが近い場合は危険物ではなく子ども側を見る。危険物のそばでとぐろを巻く Behavior は MVP で不採用。将来、遮蔽効果と誘導リスクを実験して再評価する |
| 2 | **6 本目の Motor は暫定で Head Yaw。** 最終決定ではない。H0 摩擦試験で Pure Snake Locomotion が十分なら Head Yaw を採用、移動性能が足りなければ Body Yaw 優先を再検討 |
| 3 | **SD-01（Bean Head ＋ concentric knuckle）は継続検討。** 4 mm 一定の gap は安全確定ではなく Engineering 検証待ち。CAD_CONCEPT / KINEMATIC_SIM として扱う |
| 4 | **Agent ごとに Git worktree を分離する方針を正式採用。** Design Agent は Design worktree 以外の branch を切り替えない。shared / main worktree の rebase・conflict に触れない |
| 5 | `agent/design-floor-watch` は Design worktree が clean かつ rebase 中でないことを確認して push してよい。main への merge は禁止 |
| 6 | Fusion の SD-01 は、復帰しても既存ファイルを即上書きせず Recovery copy として別名保存。最後の正常保存版を保持 |
| 7 | 背板の修正は Recovery の後に継続。unsaved state が不明な状態で破壊的な編集をしない |
| 8 | ENTRY-0011（電装配置と同心ナックルの衝突）は Engineering 回答待ち。Design 側だけで解決を確定しない |

Knowledge Vault へ同期するときは、DEC-CAND-DESIGN-0002 を「User 採用」、DEC-CAND-DESIGN-0001 を「継続検討（CAD_CONCEPT）」に更新する。
