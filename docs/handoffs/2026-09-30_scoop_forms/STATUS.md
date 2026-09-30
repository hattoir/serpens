# 進捗（2026-09-30）

## できたこと

### スコップ（`agent/engineering-scoop`）
- **奥ヒンジのフタ + くちばし（ちょうつがいの腕）の調査**: 1 円玉・立方体 0/1,728、CR2032 7/1,728、ビーズだけ 70%（叩かれて飛んだ分を含む）。原因は構造的。→ `simulation/results/scoop_beak_2026-09-30.md`
- **形と機構 A〜G の探索**（開放底のフード + ゲートを基準）→ `simulation/results/scoop_forms_2026-09-30.md`
  - 受け身のフード（機構なし）N=30: 直線の口 83%、漏斗つき 95%（10 mm/s）。物 4 種 × 床 2 × 速さ 2 × ずれ 3 の内訳あり
  - 前の押す方式との違い = 口の床の段差を 0 にしたこと（0.1 mm の段差で全対象物 0%）
  - ゲートの効果は、頭が後退するときだけ（ゲートなし + 後退 30 mm = 0%）
  - B ベルトは壁・脚に当てたときだけ成功。D カップ（内径 40）は位置ずれ 10 mm まで 100%（位置合わせが前提）。A・C・E は受け身のフードより悪い
  - 推奨（現行）: ①漏斗つきフード + ゲート、②カップ内径 40（条件つき）
- テスト: 全体 **482 passed / 1 skipped**（`tests/test_scoop_model.py`、`tests/test_scoop_forms.py` を含む）

### 安全（`agent/engineering-floor-watch`）
- 脇の V: **カバーで塞ぐを主、トルク上限は補助**。許容トルクを接触点の r で再計算（`flank_v.md`）。5.7 N は暫定のまま。リミッターの窓 0.7〜1.0 N·m との食い違いを OPEN（OQ-0116）
- Design の r（ENTRY-D-0004）: **J3〜J5 は r = 44.4〜49.8 mm → 許容トルク約 0.25 N·m**（0.45 N·m の上限と不一致 = OPEN、OQ-0117）。**J1 の下げる側（r = 5〜44 mm）は力の制限だけでは成立しない**。暫定 J1 範囲 −5°〜+10° を提案。中空 10〜19 g なら J1 静的トルク 0.024〜0.035 N·m（上限の 5〜8%）（`j1_head.md`）
- 共有ファイル（`ai-shared/`、git 管理外）に ENTRY-E-0005 / ENTRY-E-0006、OQ-0116 / OQ-0117 を追記済み

## 途中（スコップの追加検討 6 点。実装の一部だけ済み・**未検証**）

`simulation/scoop/forms/common.py` と `passive.py` に、次の変種のコードを足した（**未コミットだった WIP。動作確認は途中**）:
- `gate="curtain"` + `curtain_f`（受け身の TPU 垂れ布。駆動なし、閉じる力 [N]）
- `skirt_mm`（3 / 6）+ `skirt_k`（壁の下端の柔らかいスカート）
- `bump_mm`（床の凹凸 ±。高さ場。**絨毯の代用**）
- `gate_force_n`（ゲートの力の上限）
- 段差 `plate` とすき間 `clearance_mm` は、すでにパラメータとして使える

スモークテスト（`scratchpad/var_n1.py`）は、各変種を 8 回ずつ走らせる確認で、**まだ結果を読んでいない**。

## 未着手
- `tools/scoop_forms_sweep.py` に掃引の段階（許容差 `tolerance` / 垂れ布 `curtain` / スカート `skirt` / ゲート力 `gateforce`）を足す
- 掃引の実行（合計 約 4,900 回、40〜60 分の見込み）
- `tools/scoop_forms_report.py` に節を足す（許容差・垂れ布・スカート・ゲート力・カップの位置合わせ）と、推奨の更新
- カップの位置合わせ精度のカメラからの逆算（下の REQUESTS.md の 4）を報告書へ
- `agent/DECISIONS.md` と `docs/verification_status.md` §4.14 の更新、テスト追加、commit・push

## 動いているもの・注意
- バックグラウンドの Python（スモークテスト）が残っている可能性がある。止めてよい
- Windows で `python -m pytest` の頭に WMI の警告（`0x8007000e`）が出ることがあるが、テストは通る（負荷が高いときの一過性）
- worktree は一時フォルダ。**未コミットの変更は、このフォルダのコミットで branch に載せた**

---

## 更新（追記: A の 1〜8 を実行後）

- ✅ 垂れ布・スカート・床の凹凸・ゲート力・許容差（段差 × すき間）・許容差の確率版・カップの位置合わせ（カメラ）・推奨の更新・試験片 B 用の試験計画・DECISIONS・verification_status を反映。報告書の §0（定義と断面図）・§8（推奨）・§12（追加の検討）
- 主な結果: 段差は **0.002 mm でも 0%**（確率版の期待値 0〜0.7%）。受け身の垂れ布は依頼の範囲（0.1〜1 N）では成立しない（0.01〜0.03 N が境界。短い垂れ布 + 3 mN で 1 円玉・CR2032 だけ）。**駆動のゲートは 2.8 N 以下（0.25 N まで）で成立**。スカートは効果なし
- 残り: 安全側（B）— 質量表・ENTRY-E-0007・垂れ布の挟み込みの節（`agent/engineering-floor-watch`）。J1 の範囲は User の判断待ち（robot.yaml は変えない）

---

## 更新（追記: 段差 0.002 mm の 0% の頑健性を確認後）

- ✅ 面取り（θ 5〜90°・R 0〜0.3・t 0.002〜0.5）・床の粗さ（±0.01〜0.1）・接触設定（solref・時間刻み・影響幅・margin）・実物に近い組（面取り + 粗さ + 物の縁の丸み）を掃引。`scoop_forms_2026-09-30.md` §12.5〜12.9。
- 結論: **シミュレーションでは、口の前縁の段差はほぼゼロ（0.05 mm 以下）が必須**。面取りは「入る」（ほぼ立方体）だけ戻す。例外は 面取り 10 / 20° + 粗さ ±0.1 mm + t 0.05 mm の 1 組（保持 44〜59%）。期待値は A 0〜0.7%、B 0〜0.7%、C 約 10〜27%（粗さ ±0.1 mm の仮定のとき）。`scoop_forms_tolerance_mc.md`。
- 推奨案の比較表（§8）: カップはすき間 0〜1 mm に強い（各 64/64）。J1 でカップを昇降すると −8〜−13.5°（暫定 −5…+10° の外）→ 昇降は別の駆動。
- ENTRY-E-0006 に J1 静的トルクの訂正（0.204 N·m = 上限の 45%）を追記済み。
- 次: 試験片 B の実測（面一が実現できるか。面取り・粗さで救われるか）。

---

## 進行中（2026-09-30 夜。User の「次の指示」A → B → C。途中経過。セッションが切れたらここから再開）

**方針**: 今後 2 日間は User が B2 を印刷中。結果は待たない。実物の観察が要る項目は「印刷待ち」。B2 が出たら、シミュとの比較を報告書に別の節で足す。

- ✅ **B の 2（薄いフィルムの縁）**: `python tools/scoop_forms_sweep.py film` 済み（2,304 回。`scoop_forms_film_{designs,cells}.csv`）。**垂直の縁（面取りなし）は、t = 0.01〜0.05 mm・粗さ ±0〜0.1・すき間 0.1 / 0.3 のどれも保持 0/48**。許容できる厚みは無い（0.002 mm でも 0）。
- ✅ **A（J1・ToF・LED の確認）**: ブランチ `agent/engineering-h1-j1`（`agent/engineering-vision-sim` から分岐。worktree `scratchpad/serpens-h1-wt`）。`hardware/prototypes/H1_joint/j1_head_pitch.md`、`HG-H1_actuator/j1_floor_contact.py`・`j1_head_pitch_check.py`、`HG-H2_sensor_head/tof_cliff.md`・`tof_cliff_budget.py`・`led_layout_recheck.py`、`tests/test_j1_head_pitch.py`。LED の追試は Design と一致（巡回 0.95）。
- 🔄 **B の 1・3（フード昇降 cloche）**: `simulation/scoop/forms/cloche.py` を追加（動作確認 N=1 済み）。掃引 `cloche`（72 設計、平らな床）→ `cloche2`（床の粗さ ±0.1 / 凹凸 ±0.5）を実行中（`scratchpad/run_cloche.sh`）。
- 🔄 **C（安全の枠組み）**: `agent/engineering-floor-watch` の `flank_v.md` §F と `hood_lift.py`（書いた。未コミット）。
- ⏳ 残り: 報告書 §13、ENTRY-E-0008、DECISIONS / verification_status §4.14、テスト、commit・push（3 ブランチ）。

### 更新（B・C 完了）

- ✅ **B の 1・3（フード昇降）**: `cloche` / `cloche2` / `cloche3`（合計 4,096 回）済み。報告書 **§13**。位置ずれ 0・5 mm は 100%、1 円玉・CR2032 は ±5 mm が限界。覆う・運ぶ（ゲートあり）とも成立、ゲートなしの後退は 0/32。
- ✅ **C**: `agent/engineering-floor-watch`（`f12dc4e`）の `flank_v.md` §F・`hood_lift.py`。
- ✅ **A**: `agent/engineering-h1-j1`（`03ddfc8`）。ENTRY-E-0008 は `ai-shared/integration-log.md`（git 管理外）。
- ⏳ **B2 の実物の観察が出たら**、シミュレーション（t = 0.05 で 0%、c 0〜1 で差なし）との比較を報告書に別の節で足す（**印刷待ち**）。
