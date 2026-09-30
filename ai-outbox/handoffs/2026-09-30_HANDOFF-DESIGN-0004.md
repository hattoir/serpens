# Handoff — Design（2026-09-30、すき間チェック）

- **Git**: `agent/design-floor-watch`（Design worktree のみ）。main には merge していない
- **Fusion**: 元 `Serpens_DESIGN_SD01_BEAN_KNUCKLE_CONCEPT_NOT_FOR_PRINT` は v6 のまま。編集は Recovery 複製 `Serpens_DESIGN_SD01_BEAN_KNUCKLE_RECOVERY_20260929` のみ（尾の帯の切り落としを一度入れて、検査の不具合と分かって削除。**現在の形は切り落とし前と同じ**）

## この区切りでやったこと

- ENTRY-E-0003 (5)「全角度で 5〜12 mm の隙間なし」の検査: `docs/design/tools/gap_check.py`（STL ボクセル、0.5 mm、6 姿勢）、報告 `docs/design/gap_check_report_2026-09-30.md`、**ENTRY-D-0003**、OPEN-SERPENS-DESIGN-014
- 検査ツールの不具合（箱の始点で行が埋まる）を見つけて直し、全部取り直した（教訓 17）
- 静止: 直した 5 件、残り 3 か所（頭と首の下の後ろ 170、J4・J5 の継ぎ目の V の接点 264 + 260 mm³）
- 曲げた姿勢: 全関節の脇に V のくさび（合計 6500〜6900 mm³）。頭を下げるとあごと首の前にも（856〜1228）。**形では消せない**

## 次に Design がやること

1. FLANK SKIRT STUDY（脇の柔らかいカバー。TPU の蛇腹・重ね鱗）の概念 CAD ＋ 同じ検査で数を出す。蛇腹の中の挟み込みは別の問題として残る
2. 頭と首の下の後ろ（r32 の縁の材料）を直す。あごと首の前のくさび（J1 ピッチ下限、顎板の後ろ端）
3. HEAD E2-V v2 の外観（セージ・アイボリー・ストライプ・目の黒）を復元（いまは名前のない体に鋼の外観）
4. Head Yaw の首の CAD（上の首 52〜56、同心の頭巾）

## Engineering へ / User 判断

- Engineering: ENTRY-D-0003（脇の V の扱い、閉じる力の算術 0.26 N·m の確認、小部品の保持）
- User: 脇の柔らかいカバー（外観・触感が変わる）を進めてよいか（OPEN-SERPENS-DESIGN-014）
