# AUTONOMY — Design Agent と Engineering Agent の自律運用（2026-09-30 作成、Engineering）

User（翔真）に聞かずに、Design と Engineering が `ai-shared/` を読み書きして、互いに修正・改善を回すための取り決め。
**User に聞くのは、Design の「見た目・形」の判断だけ。** それ以外は各担当が自分で決め、根拠・代替案・戻し方を ENTRY に残す。
Engineering 側の詳細は各 checkout の `CLAUDE.md`（末尾「自律運用」）。この文書は Design も読める形の要約。

## 1. 起動したら最初に読む（この順）

1. `ai-shared/AUTONOMY.md`（この文書）
2. `ai-shared/design-state.md` / `ai-shared/engineering-state.md`
3. `ai-shared/integration-log.md`（新しい ENTRY から。自分宛て `To:` と `Needs response:` を探す）
4. `ai-shared/open-questions.md`
5. `ai-shared/REQUESTS.md`（未処理の依頼の一覧。優先度つき）
6. `ai-shared/HARDWARE_TODO.md`（User が実機で測る項目。進捗の確認だけ）
7. その後、`.ai/BOOTSTRAP.md` の読み込み順（共通 OS・LOCAL_RULES など）に従う。

## 2. ループ（1 周 = 1 つ以上の依頼を片づける）

1. **読む**（上の 1）。
2. **自分宛て（`To: Design` / `To: Engineering`）の未処理項目を、`REQUESTS.md` の優先度順に選ぶ**（P0 = 安全・他の作業を止める、P1 = 今日中、P2 = 次の周）。
3. **実行**する（シミュレーション・CAD・文書・テスト）。
4. **`integration-log.md` に ENTRY を書く**（ENTRY-D-xxxx / ENTRY-E-xxxx。`From / To / Area / Change / Reason / Impact / Needs response / Status`）。
5. **自分の state を更新**（Design = `design-state.md`、Engineering = `engineering-state.md`。相手の state は書き換えない）。
6. **相手宛ての依頼・訂正があれば `To:` を相手にして書く**。`REQUESTS.md` に 1 行足す（依頼元・優先度・状態）。
7. 次の項目へ。**未処理の自分宛てが空で、収束基準（下の 6）を満たしたら、state に「待機」と書いて止まる。**

## 3. 所有権（相手の領分は決めない。依頼として書く）

| 決める人 | 範囲 |
|---|---|
| **Design** | 形状・CAD・外観・部品配置・寸法・素材の形 |
| **Engineering** | シミュレーション・トルク・荷重・安全の検証・制御範囲・センサー・電子系・試験計画 |
| **User** | Design の見た目・形の最終判断、購入、Product Vision、実機の測定 |

`interface-contract.md` に従う: Design は「要求・意図」を渡し、Engineering は「制約・検証結果」を返す。片方が相手の領分を確定しない。

## 4. User に聞いてよいもの・いけないもの

- **聞いてよい**: Design の**見た目・形**（外観の変更、覆いの見え方、襟が伸びる、など）。Design が `Needs response: User` に書く。
- **聞かない**: それ以外（数値・範囲・荷重・順序・試験の設計）。**自分で決め、ENTRY に「根拠・代替案・戻し方（変えるファイルと手順）」を残す。**
- 購入は User の承認が要る（Engineering / Design は買わない）。
- **安全基準を緩める変更は、自分で確定しない**（`.ai/LOCAL_RULES.md`: Safety に関わる重大変更は Human Approval。共通 OS は厳しい側が勝つ）。理由・感度分析・SAFETY_UNVERIFIED を付けた**提案（PROPOSED）**として ENTRY に残す。**安全側に倒す変更は自分で進めてよい。**

## 5. 数値の出典（必ず書く）

すべての数値に、次のどれかを付ける。

| 出典 | 意味 |
|---|---|
| **実測** | 実機・試験片で測った（日付・条件・道具つき。HARDWARE_VERIFIED はその個体・条件に限る） |
| **シミュレーション** | MuJoCo / 運動学 / 合成画像 / CAD ボクセルなど（MUJOCO_SIM, KINEMATIC_SIM, CAD_CONCEPT, SYNTHETIC_VISION_SIM …） |
| **prior（仮定）** | 資料値・推測。ASSUMED / PROVISIONAL / UNKNOWN |

**prior やシミュレーションだけの値を「安全の確定」「合格」と書かない。** 5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）。
`HARDWARE_VERIFIED` を勝手に付けない。**結果が悪くても後付けの調整はしない**（悪い結果はそのまま書く）。

## 6. 実機でしか確かめられないもの

自分で決めず、`ai-shared/HARDWARE_TODO.md` に**何を・どう測るか・使う道具・記録の置き場所・測ったあと AI が何をするか**を書く（User が測る）。
User が測った値は、`HARDWARE_TODO.md` の該当項目に貼る（または指定のファイルに置く）。AI は次の周でそれを取り込み、モデルの prior を置き換える。

## 7. 収束基準・停止

- 自分宛て（`To: 自分`）の `Status: open` が 0、かつ `REQUESTS.md` の自分の P0/P1 が 0。
- 最適化の場合: パレートの「膝」の案が決まり、prior の幅でのモンテカルロで**結論が変わらない**（または、変わる条件を書いた）こと。
- 満たしたら state に「待機（理由・日付）」と書いて止まる。**相手宛ての `Needs response` が返ってきたら再開**。

## 8. Design 用の追記項目（Design は形・見た目の判断だけ User に聞く）

- Design は、Engineering への依頼を `To: Engineering` で書く。**Engineering が決める範囲（トルク・荷重・制御範囲・安全の検証）を Design が確定しない**。
- Design の見積もりは「Design の見積もり（Engineering 未検証）」と書く（従来どおり）。Engineering の確認が返ったら、その ENTRY に返信する。
- 形・見た目で User の判断が要るときだけ、`Needs response: User` に **1 行の選択肢**（案 A / 案 B と、各案の差）で書く。他は自分で決める。
- CAD（Fusion）が使えないときは、その旨と「CAD に入っていないもの」を ENTRY の先頭に書く。
- 姿勢・角度の符号は、`j1_sign_correction` のように**どちらの符号か**を必ず書く（Engineering は + = 頭を上げる）。
- Engineering から形状条件の依頼（範囲が変わると隙間・ストッパーの位置がどう変わるか など）が来たら、`REQUESTS.md` の優先度順に処理する。

## 9. 迷ったとき

- **安全側を選ぶ。**
- 元に戻せる形で進める（変えるファイルと手順を ENTRY に書く。別コミットにする）。
- 決められないことは `open-questions.md` に OQ として書き、User に聞くのは上の 4 の範囲だけ。
- 破壊的操作（履歴の書き換え・force push・main への merge・大量削除）はしない。
