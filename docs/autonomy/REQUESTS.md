# REQUESTS — 未処理の依頼の一覧（Design ↔ Engineering）

`integration-log.md` の ENTRY の `Needs response` を、優先度つきの行にしたもの。**1 依頼 = 1 行。処理したら「状態」を更新**し、ENTRY 番号を書く。
優先度: **P0** = 安全・他の作業を止める / **P1** = 今日中 / **P2** = 次の周。詳細は `AUTONOMY.md`。
**依頼を出す側が行を足し、受けた側が状態を更新する。**（2026-09-30 作成、Engineering）

| # | 優先 | 依頼元 → 宛先 | 内容 | 元の ENTRY | 状態 |
|---|---|---|---|---|---|
| R-001 | P0 | User → Engineering | 自律運用の仕組み（CLAUDE.md・AUTONOMY・HARDWARE_TODO・REQUESTS） | E-0010 | 完了（2026-09-30） |
| R-002 | P0 | User → Engineering | J1（J7）の範囲・ストッパーの荷重の最適化（パレート・膝・モンテカルロ・別コミットで設定） | E-0011 | 完了（2026-09-30）。範囲 [−4, +3]、鋼ダウエル + TPU パッド。実測（HT-001〜004）で見直す |
| R-003 | P0 | Engineering → Design | 最適な範囲での形状条件: 窓 −4〜+3°、鋼ダウエル φ3 + TPU パッド、覆いの採否（見た目 = User）、+4° / −4° の危険体積の計算 | E-0011 | 依頼済み（Design の回答待ち） |
| R-004 | P1 | Design → Engineering | J1 サーボの箱の実寸（y ±22.6 は ASSUMED） | D-0009 / D-0010 | 未（実寸は HARDWARE_TODO 側。データシート値を Engineering が確認） |
| R-005 | P1 | Design → Engineering | フード昇降: 上げる側の力の制限（≤ 2.8 N）・自重降下・すき間 4 mm 以下 | D-0009 | 回答済み（E-0008 (6)）。Design の設計待ち |
| R-006 | P1 | Design → Engineering | J1 の符号 | D-0010 | 回答済み（E-0009）。実機確認は HT-001 |
| R-007 | P1 | Engineering → Design | ToF + LED を横スキッド前端に入れる寸法（素子だけの小基板）、スキッド底と足の帯の底の同一面 | E-0008 | 未（Design の CAD 待ち） |
| R-008 | P2 | Engineering → Design | 鋼ダウエル φ3・TPU 緩衝を入れるストッパーの形 | E-0009 | R-003 に統合（E-0011） |
| R-009 | P1 | Engineering（自分） | `neck.floor_watch_*`（作業窓・ストッパー・窓の端の手前の速さ）を firmware / executor で強制する配線（今は設定の値だけ） | E-0011 | 未（次の周。上限の強制は機体側でも行う原則に従う） |
| R-010 | P2 | Engineering（自分） | HT-001〜004 の実測が来たら、`assumptions.yaml` に D・B・J・k・e を入れて `j1_range_opt_report.py` を回し直す | E-0011 | 待機（User の測定待ち） |

