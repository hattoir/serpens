# Design State

Design Agent が現在の設計状態を書く（Engineering Agent は書き換えない）。

Last updated: 2026-09-29（Run DESIGN-2026-09-29-01、User Decision USER-DEC-SERPENS-DESIGN-0001 を反映）
Updated by: Design Agent（Claude Code）
Git branch: `agent/design-floor-watch`（commit は別 worktree で行う。共有の作業ツリーの HEAD は動かさない。ENTRY-0004）

**HUMAN_EVALUATED = 0 / HARDWARE_UNVERIFIED。数値は DESIGN_ESTIMATE。Design の提案であって正式 Decision ではない。**
詳細: `../docs/design/concepts_2026-09-29.md`、数値: `../docs/design/results/concept_metrics_2026-09-29.json`

## User Decisions（2026-09-29、USER-DEC-SERPENS-DESIGN-0001。Design Run の成果に対する User の判断）

| # | 決定 | Design への効き方 |
|---|---|---|
| 1 | **CSAR を採用**。危険物を見つけてもその場で見せびらかさない / 再確認と位置登録の後、基本は危険物から少し離れる / 子どもが近い時は危険物ではなく子ども側を見る / **危険物のそばでとぐろは MVP で不採用** / 将来、遮蔽効果と誘導リスクを実験して再評価 | OPEN-SERPENS-DESIGN-004 = 採用（A）、005 = 離れる（A）。「遮蔽としてのとぐろ」は将来の実験候補に移す（MVP の行動には入れない） |
| 2 | **6 本目は暫定で Head Yaw** として Design を進めてよい。**最終決定ではない**。H0 摩擦試験で純粋な蛇行で十分動けるなら Head Yaw、移動性能が足りなければ Body Yaw を再検討 | 頭 yaw を首に置く意匠を進める。Body Yaw（全長 668 mm）に戻っても成り立つよう、頭・首の意匠を胴の節数に依存させない |
| 3 | **SD-01（Bean Head ＋ 同心ナックル）は継続検討してよい**。ただし **4 mm 一定のすき間は安全確定ではない**（Engineering 検証待ち）。**CAD_CONCEPT / KINEMATIC_SIM として扱う** | 挟み込みの数値は「2D の形の計算（KINEMATIC_SIM）」とだけ書く。安全・合格の語を使わない |
| 4 | **Agent ごとに Git worktree を分ける方針を正式採用**。Design Agent は Design worktree 以外の branch を切り替えない。shared / main の worktree の rebase や conflict には触れない | Design の commit は Design worktree（`agent/design-floor-watch`）だけ。共有ツリーの `ai-shared/` は追記のみ |
| 5 | `agent/design-floor-watch` は、**Design worktree が clean で rebase 中でないことを確認してから push してよい**。main への merge は禁止 | push 前に `git status` と rebase 状態を確認する |
| 6 | Fusion の SD-01 は、Fusion が復帰しても**既存ファイルを上書きせず、Recovery copy として別名保存**。最後の正常保存版を保持 | 復帰したら、まず状態を読み取り → 別名保存（例 `..._RECOVERY_20260929`）→ 元ファイルは触らない |
| 7 | **背板の修正は Recovery の後に続ける**。未保存の状態が分からないまま破壊的な編集をしない | 背板の作り直しは Recovery copy 上で、状態を読んでから |
| 8 | **ENTRY-0011（電装配置と同心ナックルの衝突）は Engineering の回答待ち**。Design だけで解決を確定しない | 段付きナックル等は「案」として出すだけ |

## Current concept

**SD-01「Bean head ＋ 同心ナックル」**（User が継続検討を承認。**CAD_CONCEPT / KINEMATIC_SIM**。Engineering の確認待ち）
**6 本目は暫定 Head Yaw**（H0 摩擦試験しだいで Body Yaw を再検討）。**CSAR は User 採用**

- 最重要原則への答え: **「見つけたら、見せびらかさず、離れて、大人に知らせる。子どもが来たら、こっちを見てもらう。」**
  幼児は視線・指さしの先を見るので、発見の合図（物を見る・指す・物の横でとぐろ）は子どもを危険物へ連れていく。
  振る舞いの規則 CSAR（R1〜R5）にして、守る行為と「ペットが大人に知らせに来る」を同じ動きにした。

## Current geometry direction

- Fusion: **`Serpens_DESIGN_SD01_BEAN_KNUCKLE_CONCEPT_NOT_FOR_PRINT`**（Serpens フォルダ、2026-09-29 に新規作成。FW03 など既存 CAD は変更していない）
  - コンポーネント: HEAD / NECK / LINK1 / LINK2 / LINK3 / TAIL（外装の外形。中は詰まったまま、部品ではない）、
    REF（FW03 v6 から読んだ Engineering の参照箱 41 個、非表示）、SCENE（姿勢の絵用の硬貨）
  - as-built 回転ジョイント J1〜J5（yaw ±50°、J1 は**このファイルでは負が頭上げ**、0〜−45°）。LINK2 を固定
  - 描き出し: `../docs/design/renders/SD01_*.png`（巡回・発見・人を見る・子どもが近い・眠り、各 iso / top）
- 座標は CAD と同じ（mm、頭が −X、Z 床基準）。関節軸 X: J1 −181.8（Z32.35）、J2 −95、J3 0、J4 95、J5 190

## Head concept（H1 Bean）

- 幅 100（胴 92 より広い）× 高さ 74 × 長さ 78。平らなあご・丸い鼻先・盛り上がった頭頂。角を作らない（毒蛇の三角頭を避ける）
- 顔の造作 = センサー: カメラ = **ボタン鼻**（濃色の平窓 D14、Z30）、前 ToF = **鼻孔**（2 つの小窓、Z50）、
  斜め照明 = 下唇の影、ライン光 = **眉**（顔の面から 6 mm 以内、X −233〜−229、Z64〜70。それより後ろだと鼻先が近い床への光を遮る）
- 目: 径 24 の半球ドーム (−212, ±37, 60)、丸い瞳、まばたきせず光の強弱。WS2812B を虹彩リングへ移す提案
- 首: 幅 68（半幅 34）へ絞る。頭 yaw を足すなら首に置く
- 数値: 頭 93 g（ASSUMED）、J1 静的 0.020 N·m（ソフト上限 0.45 の 4.4 %）、視錐台への殻のかかりなし、目の見かけ面積は H0 の 2〜3 倍

## Body concept（B1 同心ナックル）

- 前の節の後端 = 関節軸まわりの円柱（半径 46。J2 は 34）、後ろの節の前端 = 半径 +4 の受け（唇 ±40°）、関節の上を後ろの節の丸い背板（半径 48 / 36）が覆う
- 挟み込み（2D の形の計算、KINEMATIC_SIM）: B0 卵殻は 10〜40° で側面のすき間が 22.6 → 8.8 mm と 8〜25 mm の帯を通って閉じる。B1 は 0〜50° で 4.0 mm 一定、V 溝 約 97°。
  **4 mm 一定は安全確定ではない**（公差・上下の板・ケーブル・指の模型での確認は未。Engineering 検証待ち、User Decision 3）
- **Fusion で見つかった収まりの問題**: B1 では各関節の後ろ 0〜46 mm（中段）が**前の節の円柱**になる。FW03 の電池・尾 MCU・ヒューズ・スピーカー／アンプが関節の継ぎ目をまたぐ（ENTRY-0011）。
  電装を約 20〜35 mm 後ろへずらすか、中段の円柱を小さく（半径 ≈ 32）して外側を段付きにする必要がある

## Surface concept

- 背 sage、腹・側面の下 40 %（Z ≤ 40）ivory、背線クリーム（幅 18、尾から鼻先まで。発見の構えで物を指す矢印）
- つや消し。警告色（赤黄黒の縞）・菱形・縦長瞳孔・三角頭・牙・S 字の鎌首は使わない
- 腹: 交換式摩擦インサートを腹板（横の段）形にする提案（幅 40、ピッチ 10 mm、Fusion に形だけ）。摩擦比は H0 クーポンで実測（OQ-0102）
- 尾: 赤い停止ボタン（D12）を濃色の輪 D18 に少し沈めて尾の上（Engineering の位置 X296）に

## Interaction concept

- **CSAR は User 採用（2026-09-29）**。MVP の流れ: 見つける → 再確認（撮影）→ 位置登録 → **少し離れる** → 保護者へ通知。子どもが近ければ子ども側を見る。危険物のそばのとぐろは MVP で不採用（遮蔽効果と誘導リスクは将来の実験）
- CSAR: R1 子どもが近い（2.0 m 以内または不明）間は物を見ない・指さない / R2 物の場所を目立たせない（撮影の閃光を含む）/
  R3 見つけたら離れる（とぐろ・生きたピンはしない）/ R4 まず保護者へ通知 / R5 子どもが来たら頭と目を子どもへ、物から外す
- Discovery Loop（子どもがいない時）: 気づく → 体ごと一直線に静止して撮影（表現用 LED 全消灯＝息を止めて見つめる）→ 周りを見る → 保護者を見る（短い明るい音）→ もう一度物 → 離れる
- 姿勢: 巡回 / 発見（一直線）/ 人を見る（J2 45°、J1 30°）/ 子どもが近い（J2 −50°、J3 −20°、物から約 70° 外す）/ 眠り（三日月、yaw 合計 180°）

## Open design questions

OPEN-SERPENS-DESIGN-001〜009（`open-questions.md`）。特に 004（CSAR の採用、Human Approval）、001（ライン光の位置）、003（目の作り、OQ-0101 次第）

## Engineering dependencies

ENTRY-0005〜0011（`integration-log.md`）: 首幅 68 の収まり / ライン光の眉位置 / WS2812B と撮影中の全消灯 / ナックルとケーブルの渡り・R04 廃止 /
ナックルと電装の位置（ENTRY-0011）/ CSAR の実行条件 / 6 本目は頭 yaw を首に（Design 意見）/ ローラーの隠し方と腹板クーポン

## Rejected directions

| 案 | 理由 |
|---|---|
| H0 Egg（現行） | 頭 < 首 < 胴、目が小さく横・上から見えない、前面が球でセンサーが収まらない |
| H2 Wedge（写実） | かわいさが落ちる。鼻先が低くライン光の縦基線が入らない。重心が J1 から遠い |
| H3 Cyclops（カメラ = 目） | 見張られている印象（保護者のプライバシーの感じ方）、常に下向きの視線、蛇らしさが消える |
| H4 Visor（黒い窓の顔） | 窓の内面反射で LED・カメラ・ToF の光が混ざる。ロボット顔になる |
| B0 卵殻 15 mm 間隔 | 挟み込み域（8〜25 mm）を通って閉じる。数珠・芋虫に見える |
| B2 重ね鱗 / B3 TPU ベローズ | すべる縁のせん断、部品点数 / 曲げ剛性・割れ・熱 |
| B4 ニット袋 | MVP では不採用（見えない所で布が噛む、熱、衛生）。「撫でる・持ち上げる」段階の研究候補 |
| 危険物の横でとぐろ・生きたピン | 子どもを物のそばへ呼び寄せる。5 サーボでは輪も作れない（yaw 合計 ≤ 200°） |
| 背板を胴より 2 mm 厚く出す（Fusion 初版） | 横から見て屋根の板が載ったように見えた → 胴の面と同じ高さにした |
| 眉のライン光を顔の面から 10 mm 以上下げる | 鼻先が近い床（38〜48 mm）への光を遮る（計算） |

## Fusion の状態（2026-09-29）

- `Serpens_DESIGN_SD01_BEAN_KNUCKLE_CONCEPT_NOT_FOR_PRINT`: 最後の正常保存はジョイント追加の直後。背板の作り直しの実行中に Fusion の API が応答しなくなり、以後の変更の保存状態は**不明**
- User Decision 6・7: 復帰したら上書きせず **Recovery copy として別名保存**し、元ファイル（最後の正常保存版）を保持する。背板の修正は Recovery copy の上で、状態を読み取ってから続ける

## Next experiments

1. （Fusion 復帰後）状態の読み取り → Recovery copy を別名保存 → 背板を胴と同じ高さに作り直す
2. 6 本目 = 暫定 Head Yaw の首の意匠（首 50 mm 延長に頭 yaw サーボ）。Body Yaw に戻っても頭・首を流用できる形に
3. 静止画の予備評価: H0 / H1 / H2（同じ色・角度）を human_pilot の尺度で（ai-outbox/experiments）
4. ENTRY-0011 は Engineering の回答待ち（段付きナックル r ≈ 32 は案として保持、確定しない）
5. 将来の実験候補: 「遮蔽としてのとぐろ」の遮蔽効果と誘導リスク（MVP 外）
6. 腹板インサートの形を H0 クーポンに入れる相談

## 更新（2026-09-29 後半、Continuous Design Mode）

- Fusion 復旧: 元ファイルは v6 のまま保持。Recovery copy `Serpens_DESIGN_SD01_BEAN_KNUCKLE_RECOVERY_20260929` で、背板を胴と同じ高さにし、J3〜J5 の背板を r45 に変更（r48 は隣と 1 mm 重なっていた）。NECK の外形トリムも掛け直し、J1 の頭と首を同心にした。3D 干渉は全関節の全範囲で 0（CAD_CONCEPT）。J1 の頭の後ろに閉じる溝が残る（ENTRY-0013）
- 派生案: SD-01A〜E を比較（`docs/design/results/parametric_2026-09-29.md`）。**CAD へ持ち込む上位は SD-01E（A と C の中間）と SD-01A**
- CSAR の振り付け: `docs/design/csar_choreography_2026-09-29.md`（状態 × 子どもの状況、UNKNOWN = NEAR 扱い、禁止候補、Engineering への数値の問い、OPEN-010）
- Head Yaw の評価、ENTRY-0011 の代替案 A〜G（Design の推し順は A → C → G＋E）、人の評価のフォーマット、backlog: `docs/design/sd01_variants_and_fallbacks_2026-09-29.md`
- 次: SD-01E の頭を Recovery copy に別コンポーネントで作る → 人の評価用の描き出し（`tools/fusion_render_set.md`）→ Head Yaw の首の意匠

## 更新（2026-09-29 夜）

- Fusion が一度戻った間に、Recovery copy（v4 で保存）へ **SD-01E の頭**を別コンポーネントとして追加（HEAD A は無変更、J1E ジョイントつき、既定は非表示）。人の評価用の描き出し 16 枚（A / E × 4 姿勢 × 横・真上）
- **J1 頭–首の入れ子 VISOR** を 2D で検討: いまの形は頭を上げると外から届くすき間が閉じる（25° から）。VISOR ならどの角度でも届く 8〜25 mm のすき間なし・重なり 0（KINEMATIC_SIM、ENTRY-0013 に追記）。**P0 の案として Engineering 待ち**
- 描き出しの途中で Fusion がまた応答しなくなった。正面・3/4 の撮り直しは次の Fusion 作業
- 次: (1) 正面・3/4 の撮り直し (2) VISOR を Recovery copy の別コンポーネントで形にする（元の首・頭は残す）(3) Head Yaw の首の意匠 (4) 目の光り方の絵

## 更新（2026-09-29 深夜）

- Recovery copy に追加（すべて別コンポーネント、元は無変更）: **SD-01E2**（顔の読まれ方の修正）、**MOUTH LINE**（E3 = E2 ＋ 横の口の線）、**VISOR**（NECK-V ＋ HEAD E2-V、J1 0〜45° で交差 0）
- 人の評価の画像: `docs/design/renders/eval/`（A / E / E2 / E3 × 姿勢 × 画角、同じ画角）
- **現在の Design の第一候補: SD-01E3（= E2 ＋ 口の線）＋ VISOR の J1 ＋ 同心ナックルの胴**（CAD_CONCEPT。人の評価・Engineering の判定待ち）
- 次: 芯の色、Head Yaw の首（VISOR の芯の前に頭 yaw サーボ）、ENTRY-0011 の A 案で電装を動かした絵

## 更新（2026-09-29 Engineering の回答を受けて）

- Engineering の回答: ENTRY-0015（ライン光は眉を推奨）・0004（あご LED 6 mm・カメラ 25°）・0019（照明を後ろへ）・0020（スキッドの沈み）・0021（CSAR を模擬で実装）。Design の回答は **ENTRY-0022**
- **OPEN-SERPENS-DESIGN-001 は解決**（眉の段、X −233〜−229）。首軸 → レンズ 50.3 mm（A）/ 52.3 mm（E 系）
- 6 本目 = Head Yaw の Design 上の理由は、ライン光の向け直しではなく **CSAR の視線**だけになった
- 照明: 頬の下（斜め）と口の線（通常）の左右 2 灯で、近/遠の照度比 40 → 15 倍・33 → 11 倍（`tools/light_placement.py`）
- Fusion: Recovery copy が一度閉じていた（v9 で保存済みを確認して開き直した）。`LIGHT+SKID option` を追加（v10）
- 次: そりを顎板の形に、VISOR の芯の色、Head Yaw の首、ENTRY-0011 の A 案の絵

## User Decision（2026-09-29 追加）の反映

- **USER-DEC-SERPENS-DESIGN-0002（CSAR の撮影）**: CHILD_NEAR では写真確認を延期、passive snapshot のみ可、後ろ向きではなく向きを変えて前へ離脱、reinspect_point、highlight_point は CHILD_NEAR の hazard に禁止 → `csar_choreography_2026-09-29.md` §7
- **USER-DEC-SERPENS-0003**: 対象 3 歳未満・Safety Envelope 6 歳以下（人体寸法は一次資料から、`child_anthropometry_sources_2026-09-29.md`）/ 接触の閾値は Engineering が導出 / **yaw 合計 145°（暫定）** / 機械式トルクリミッターは検証する Safety Layer / Safety 値は逐一聞かずに検証
- Design への影響: 眠りの三日月（180°）は使わない → **「首をかしげる」[50,45,30,20]（合計 145°）**を眠り・充電の案に（OPEN-011: Head Yaw を合計に含めるか）。関節の外形にトルクリミッターの厚みを見込む（Engineering の寸法待ち）

## USER-DEC-SERPENS-0004（Head Yaw と 145°）の反映

- 145° = Body Curvature Budget。Head Yaw は合計に入れないが独立した Safety Axis。3D enclosure / entrapment check は必須（Engineering）
- Design 側の sweep（ENTRY-0026）: **Design の提案は Head Yaw ±30°**（J3 の補助つきで CSAR の視線を満たす最小。眠りの見た目のために広げない）
- OPEN-SERPENS-DESIGN-011 は解決

## 更新（2026-09-30）HEAD INTAKE STUDY 01（User の依頼）

- 依頼: 巻き込みくちばし（TPU のランプ ＋ 11.5 mm の腕 ＋ 30×30×15 の空間）の試験片 A と頭部レイアウト。報告 `docs/design/intake_study/HEAD_INTAKE_STUDY01_report.md`、STL 6 点、Fusion `Serpens_FW02_HEAD_INTAKE_STUDY01_NOT_FOR_PRINT`（FW02 v5 の複製、元は未変更）
- 主な所見: 卵の頭には入らない（空間の 42〜66 % が外）/ Bean 頭なら 約 92 % 入る / hinge_h 12.0 だと腕がランプを削る（12.2〜12.4 以上で解消）/ 閉じた腕は入口の 2 mm の帯しか塞がない / スキッド・下向き ToF と衝突
- 安全・範囲の注意（User 判断）: PRODUCT.md「やらないこと: 物を拾う・回収する」（OPEN-012）、挟み込みの力（SAFETY_UNVERIFIED）、CSAR との整合。Design は PRODUCT.md を変えていない
- ENTRY-D-0002、OPEN-SERPENS-DESIGN-012 / 013

## 更新（2026-09-30 後半）すき間チェック（ENTRY-E-0003 (5)）

- `docs/design/gap_check_report_2026-09-30.md`。STL ボクセル検査（0.5 mm、外形のみ、スクリーニング）。**静止**: 実欠陥（背板の尖り・頭巾の溝・ポケット・J2 の低い板・尾の刃）を CAD で直した。残り 3 か所（頭と首の下の三日月 170 mm³、J4 の脇の V 入口、尾の先）
- **曲げた姿勢では「全角度で 5〜12 mm なし」が成り立たない**: 脇の V（幅 5〜12 mm の長さ ≈ 7.5/tan φ）。形では消せず、柔らかいカバー（FLANK SKIRT STUDY、次）か閉じる力の制限（Engineering）が要る。OPEN-SERPENS-DESIGN-014、ENTRY-D-0003
- Fusion Recovery 複製: 尾の腹の帯の刃（x ≥ 318、z < 6.5）を切り落とした（押し出しの切り取り）。頭 HEAD E2-V v2 の外観（セージ・アイボリー・ストライプ・目の黒）は未復元（名前のない体に鋼の外観）

## 更新（2026-09-30 夜）すき間チェックの続き（User 承認: FLANK SKIRT STUDY・r の範囲・頭と首）

- 報告 `docs/design/gap_check_report2_2026-09-30.md`、ENTRY-D-0004（**ENTRY-D-0003 の一部を訂正**）。
- **KNUCKLE DRUM（形の直し）を CAD に入れた**: J2〜J5 の玉の脇を z 8〜76 で完全な円柱（R 46、J2 は R 34）に戻す。胴の層のすき間 約 90 % 減（p1 6807 → 2558）。外観は関節に「膝」ができる（人の評価が要る）。既存の外側に足す体積は 11〜13 cm³/個
- 頭の下の後ろの埋め物（`E2-V lower-rear fill`）で p0 のすき間 172 → 2 mm³
- **FLANK SKIRT STUDY**: 脇のフラップの概念 CAD と運動学モデル。**すき間は減らず増えた**ので採らない（上下まで閉じる袖状のカバーは未設計）。唇・あごのフラップも効かなかった
- 残り: 背板の層 z ≥ 77（約 1020 mm³/姿勢）、J1 ピッチ（−5° で 601、+22° で 4307 mm³）、J2 の背板の層
- r の範囲: J3〜J5 は r = 42.5〜50 mm の輪、J2 は 28〜49、J1 はピッチで変わる

## 更新（2026-09-30 深夜）KNUCKLE DRUM 承認・試験片・試験片 B

- User 承認: KNUCKLE DRUM を進める。**レポートの冒頭に「まっすぐ比 約 20 倍の体積が残る」を書き続ける**（`gap_check_report*.md`、`mass_addendum`、試験片 README、ENTRY に反映）。背板の層は未解決のまま、J1 ピッチは動作範囲の回答待ち
- 円柱肩の試験片 R46 / R34（`docs/design/knuckle_test/`）、質量は中実（最大）61.1 g / 中空 55.8 g（`mass_addendum_2026-09-30.md`）
- 試験片 B（`docs/design/test_piece_b/`）: 漏斗つきフード ほか。**段差の解釈は Design の仮定**
- Fusion: Recovery 複製に `KNUCKLE TEST R46/R34 …`、FW02 intake の複製に `TEST PIECE B1 …`（元は未変更）

## 更新（2026-09-30 夜）試験片 B2・頭への統合・背板 2 案・J1 範囲

- **先頭の注意**: 現行 CAD（KNUCKLE DRUM のみ）は、まっすぐ比で約 20 倍のすき間の体積が残る。背板は現行 CAD では未解決のまま（案は別コンポーネント）。J1 は範囲の確認待ち
- `test_piece_b/B2/`（t と c を独立）、`head_intake_e3/REPORT.md`、`plate_layer_2026-09-30.md`（(a) DRUM-FULL / (b) 同心ドーム、J2〜J5 は p1 で 1373 → 52 / 71）、`j1_pitch_2026-09-30.md`（作業窓 −2〜+1°）
- Fusion: Recovery 複製に `HEAD INTAKE on E3 STUDY`、`PLATE VARIANT A/B`。FW02 intake 複製に `TEST PIECE B2`。元は未変更
- ENTRY-D-0006

## 更新（2026-09-30 深夜）口の前の配置・ENTRY-D-0006 への回答

- **先頭の注意**: 現行 CAD（KNUCKLE DRUM のみ）は、まっすぐ比で約 20 倍のすき間が残る。背板は未解決、J1 は範囲の確認待ち
- `mouth_front_layout_2026-09-30.md`: 口の前の禁止域に入る部品 15,290 mm³ → 案 0。c は床から（シミュのコード）。B2 の板は訂正（`B2/CORRECTION_2026-09-30.md`）。J1 の取り込み窓 −0.1〜+0.9°。照明: 斜めを口の前の角へ移すと 15 → 35 倍
- **Fusion 切断のため CAD 未変更**（スキッド・ToF・LED の移動は次回）。ENTRY-D-0007

## 更新（2026-09-30 深夜）ENTRY-D-0006 の 4 点への回答（ENTRY-D-0008）

- **先頭の注意**: 現行 CAD（KNUCKLE DRUM のみ）は、まっすぐ比で約 20 倍のすき間が残る。背板は未解決、J1 は範囲の確認待ち
- `engineering_answers_2026-09-30.md`: c は床から（確定）、J1 の窓は取り込みで 12 ステップ・受動の柔らかさは成り立たない（床接触の較正が要る）、VL53L1X は 4 cm 未満は不正確・±20〜25 mm で数 mm は測れない、斜め LED は **横スキッドの前端**が最良（巡回 0.94、17.4 倍）

## 更新（2026-09-30 夜）B2 の印刷セット・回答の位置づけ訂正・フード昇降（ENTRY-D-0009）

- **先頭の注意**: KNUCKLE DRUM 後も、まっすぐ比で約 20 倍（18〜24 倍）のすき間が残る。背板は現行 CAD では未解決のまま。5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）。安全・合格の語は使わない
- **B2 は User が印刷中（2 日間、結果は待たない）**: `test_piece_b/B2/PRINT_LIST.md`、`T0p05_HOWTO.md`、`B2_test_log_template_v2.csv`（t = 0.05・4 分類・動画名）。**印刷待ち**: 実測厚さ・エレファントフット・0.05 の貼り方の浮き。B2 の結果を前提にしたフードの大きな変更は入れない
- `engineering_answers_2026-09-30.md` は **Design の見積もり（Engineering 未検証）** に位置づけを訂正（追記 + §6 の確認依頼）。D-0007 / D-0008 の「確定」は同じ意味で読み替え
- `hood_lift_2026-09-30.md`: s = 5 まで頭 E3 に入る（あご +13.9 cm³、J1 サーボ 792 mm³ は要確認）、8・10 はカメラと J1 サーボに当たる。J1 は使わない
- **Fusion 待ち（まとめて実行）**: スキッド・ToF・斜め LED の移設、背板 (b)、J1 の箱形カバーとストッパー、フード昇降の CAD

## 更新（2026-09-30 夜、Fusion 復帰後）CAD 反映と J1 符号の訂正（ENTRY-D-0010）

- **先頭の注意**: KNUCKLE DRUM 後も、まっすぐ比で約 20 倍（18〜24 倍）のすき間が残る。背板は現行 CAD では未解決のまま。5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）。安全・合格の語は使わない
- **J1 の符号**: CAD のラベルは + が頭を下げる。Engineering の符号では、範囲の端は −5° = 401、+10° = 1562 mm³、作業窓 −2〜+1° は最大 40 mm³（`j1_sign_correction_2026-09-30.md`）
- 上の輪の覆い（別コンポーネント、STL あり）で −5° 401 → 29。下のくさびは形で埋まらない（+3° に狭める / 動く覆い）
- Fusion に別コンポーネントを追加（`MOUTH FRONT LAYOUT STUDY 01`、`HOOD LIFT STUDY 01`、`J1 COVER + STOPPER STUDY 01`）。既存は未変更、保存済み

## 更新（2026-09-30 夜）頭 E3 の統合版（ENTRY-D-0011）

- **先頭の注意**: KNUCKLE DRUM 後も、まっすぐ比で約 20 倍（18〜24 倍）のすき間が残る。背板は現行 CAD では未解決のまま。5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）。安全・合格の語は使わない
- User の許可（組み替えてよい、デフォルメ調でかわいく）で `HEAD E3 INTEGRATED v1 - cute deformed` を別コンポーネントで作成（元は残す）。あご −56.5 cm³（−34 %）は Engineering の確認待ち

## 更新（2026-09-30 夜、自律運用）ストッパー v2・覆い 3 案・ToF 基板・フード昇降 3 案・統合版 v2（ENTRY-D-0012）

- **先頭の注意**: KNUCKLE DRUM 後も、まっすぐ比で約 20 倍（18〜24 倍）のすき間が残る。背板は現行 CAD では未解決のまま。5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）。安全・合格の語は使わない
- R-003: 窓 φ 30.51〜48.49°（パッド 3 mm）、鋼ダウエル φ3 × 10.5 + TPU パッド 3.0×3.6×2.0。θ_E +4° = 151、−4° = 覆いなし 319 / 覆いあり 23（E-0011 の補間 205 は過小）。窓・穴は HT-001 の後に切る
- 覆い 3 案（A 推奨・安全側）: User の見た目の判断待ち（R-011）
- R-007: ToF 小基板は「肉球」（横スキッド前端を広げる、|y| 51.6）。後ろは不可。スキッド底は足の帯の底より c₀ 0.1 mm 低い
- R-005: フード昇降は s = 4、推奨 L2（TPU 板ばね耳 1.2 N、prior）
- Fusion: `HEAD E3 INTEGRATED v2`（あご −32.7%）ほか追加。v1・元は残し非表示。HARDWARE_TODO に HT-008〜011 追記
- **待機の条件**: User の覆いの判断（R-011）と Engineering の返信（R-012）が来たら再開。B2 は印刷待ち

## 更新（2026-09-30 夜）覆いは案 A に決定（ENTRY-D-0013）

- User が案 A（8° の殻）を採用。`J1 COVER A ADOPTED`（Fusion、STL あり）。次: 襟への接合の設計、Head Yaw との干渉、B2 の印刷待ち
