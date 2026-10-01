# Serpens の軌跡（HISTORY）

<!-- BEGIN:narrative -->

**このファイルは「自動台帳」。** 下の「自動部分」は `python tools/update_history.py` が git と `ai-shared/` から機械的に作る（手で書き換えない。`--check` で古さを確認できる）。

**手書きの事細かな年表は `ai-shared/PRODUCT_HISTORY.md`（正本。Design と Engineering が共有・追記専用。ブランチには `docs/PRODUCT_HISTORY.md` のコピー）。** 現在地（§1）、日ごとの年表（§2）、ブランチと担当（§3）、User の判断（§4）、数値の推移（§6）、訂正・撤回（§7）、更新ログ（§9）はそちらにある。二重に書かない。

- 実機（ハードウェア）での確認は今のところ 0 件（`HARDWARE_VERIFIED=0`）。数字は特記が無い限りシミュレーションか仮定（prior）。
- 期間: 2026-09-12（STEP1）〜 2026-10-01。全コミット・全ブランチ・判断の見出し・ENTRY の索引は下の自動部分。
- 更新ルール: 作業のまとまりごとに ① `PRODUCT_HISTORY.md` §9 に 1 節足し、必要なら §1・§6・§7 を直す ② `python tools/update_history.py` を実行して、この台帳を最新にする（`CLAUDE.md`「軌跡の記録」）。
- 別件（KiCad `snake-main-board`）の経緯は `PRODUCT_HISTORY.md` §9 と `snake-main-board/` 自身の git ログ。

<!-- END:narrative -->

## 自動部分

<!-- BEGIN:auto -->
### 1. ブランチの地図

| ブランチ | 先頭 | 日付 | main との差（コミット数）| 先頭の件名 |
|---|---|---|---|---|
| `agent/design-floor-watch` | ee1a4fe | 2026-10-01 | 37 | Design: PRODUCT_HISTORY.md (detailed product trajectory, append-only update log) and AUTON |
| `agent/engineering-floor-watch` | e30262e | 2026-09-30 | 12 | safety: L2 leaf-ear force check and the hood-down 1-bit requirement (PROVISIONAL) |
| `agent/engineering-h1-j1` | 5bfaabc | 2026-09-30 | 54 | R-012 answer (pad options P1/P2, window bearing) and head COG y tolerance (<= 4 mm, ASSUME |
| `agent/engineering-scoop` | 285f014 | 2026-09-30 | 12 | scoop: cover-mass effect on intake, B2 observation handover and compare script |
| `agent/engineering-vision-sim` | 8f5fd32 | 2026-09-29 | 42 | docs: measured test count 563 passed / 1 skipped |
| `agent/floorwatch-farfield-roi` | 7a5f536 | 2026-09-29 | 6 | engineering: contact-safety and cable-routing hardware gaps |
| `archive/mqtt-fix-pre-rebase` | 38c6c3e | 2026-09-29 | 1 | api: fix MQTT safety_state retain race (DEC-SERPENS-0001) |
| `main` | 70601cd | 2026-09-29 | 0 | test: MQTT live tests must not match stale retains or same-tick timestamps |

### 2. 日付ごとの全コミット（古い順。ブランチ = そのコミットを含むブランチ。main に含まれるものは main）

#### 2026-09-12（14 コミット）

- `4293c29` [main] STEP1: 環境構築
- `598ddb9` [main] STEP2: サーボ抽象層とモックを追加
- `7b82fba` [main] STEP3: 歩容エンジン・姿勢・アニメーターを追加
- `8dcce0c` [main] STEP4: 2Dシミュレータを追加
- `e1411b5` [main] STEP4: レビュー回答を反映
- `43b1b9f` [main] STEP4.5: Sim-to-Real 校正の器を追加
- `d4f10d3` [main] STEP5: 知覚（ArUco・ホモグラフィ・人物追跡）を追加
- `12ac851` [main] STEP5: README に知覚の手順と YOLO 重みの置き方を追記
- `ea3d802` [main] STEP6: 内部状態と行動を追加
- `0b46a5e` [main] STEP6: 安全割り込みと展示レイアウトの変更を反映
- `7b24a69` [main] STEP7: 展示用GUI（4分割）を追加
- `b803df9` [main] YOLO の重みを配置し、オフライン起動を検証
- `86bd35a` [main] STEP8: 実機用の対話式ツール servo_setup.py を追加
- `5f00df6` [main] 実機ポートに接続できないときのメッセージを分かりやすくした

#### 2026-09-13（2 コミット）

- `7577e35` [main] Phase1: 実機接続の配線・停止の分離・終了処理を修正
- `06676e9` [main] Phase2: PC⇄ESP32 駆動リンク（機体側watchdogと緊急停止ラッチ）を追加

#### 2026-09-14（4 コミット）

- `99229b2` [main] 製品憲章との対応表を追加し、関節数の決め打ちを外した
- `543bec3` [main] 駆動リンクに BODY（胴体姿勢）と TORQUE（脱力）を追加
- `a4494f3` [main] 安全の絶対値（構想設計書16章）を設定・実装・テストへ落とした
- `15896c5` [main] Phase3: 行動 → 駆動リンク → 機体 を1本に繋いだ（--robot link）

#### 2026-09-15（7 コミット）

- `7b5be56` [main] 可動域を3段にし、CAD R03（±64°）へ合わせた
- `a974dde` [main] Virtual ESP32 を7状態化し、仮想サーボバスとテレメトリ v2 を入れた
- `2efc440` [main] 故障注入を1か所に集め、検証レベルの区分を文書化した
- `1d723ed` [main] Belly・摩擦プロファイル・歩容パラメータ・サーボ版数を config 化した
- `74072fd` [main] GUI に関節ペインを追加し、摩擦の値を1か所にまとめた
- `0ded164` [main] SimulatedSnake / RealSnake を用意し、Phase 3B の物理エンジンを選定した
- `72d2b29` [main] agent/STATE.md と CLAUDE.md を Phase 2/3 の結果へ更新

#### 2026-09-16（7 コミット）

- `df85280` [main] Stage B/C: CAD R03 の可動域を意味別に分け、C044 プロファイルへ移した
- `374200c` [main] Stage D: 不変条件をテストで固定し、fault 公開の race を直した
- `217c3b6` [main] Stage E: ファームウェアのコンパイル検証（書き込みはしない）
- `247c305` [main] Stage F〜N: MuJoCo 導入と閉ループ。**再起動後の自動再走行の穴を塞いだ**
- `48852cd` [main] Stage I: MuJoCo 版の歩容掃引（Pareto 候補）と最終の文書更新
- `efec2c9` [main] ELECTRICAL_SAFETY_GATE を実機自律走行の開始条件に追加、接触→脱力の要求整理
- `de4628b` [main] Phase 4: 模擬画像の Vision で閉ループ、見失い時の保持と偽マーカ対策

#### 2026-09-18（12 コミット）

- `b14a765` [main] 実画像経路 CameraObserver、見失い後の位置確認（K）、制御周期の実測、電子系の設計案
- `6c52043` [main] Bug-1: Stress の飽和を直す（approach gain 0.90→0.12）、駆け込みは別の出来事として扱う
- `cb467e3` [main] Bug-2: Energy に活動由来の疲れを追加（冷えている間も気分の休憩が起きる）
- `4ca7914` [main] Bug-3: 効用ノイズ 0.08→0.15（hysteresis 1.15 を実際に超えるゆらぎにする）
- `494d9e8` [main] PATROL を stop-and-go にする（静止 40〜60%、静止中も呼吸は続く）
- `8b90bb4` [main] 一次反応（150〜250ms で目の点灯 + J8 のピクッ）、驚きの全停止中も呼吸は続ける
- `f945612` [main] 舌のちらつき相当（J8 の微小往復）。頻度は novelty に連動（3〜9回/分）
- `ae0d3a8` [main] 呼吸に尾→頭の位相勾配（30°/軸）と軸別振幅（J7 を 5° に）を入れる
- `3fa19cd` [main] 視線の時定数を gaze 研究に合わせる（hold 2.5〜4s、glance_away 3.5〜7s ごとに 1.5〜3s）
- `a363ce8` [main] 威嚇に見える姿勢を演出から外す: rear_up の土台を arc に、s_curve は legacy、フル鎌首は「伸び」専用
- `308b398` [main] Animator に予備動作（anticipate）・減衰振動（settle）・頭の加速度上限を追加
- `d0e0e7b` [main] 接近を忍び寄りの波形（stalk: A8° Ω60° f0.25）にする。速度差ではなく波形の差で意図を伝える

#### 2026-09-21（6 コミット）

- `050fd6b` [main] primitives（語彙）/ grammar（文法・config）/ locomotion（移動の段取り）に分離。撫での 3 段応答
- `4f46f9e` [main] 評価指標 5 種（LDJ・静止率・可視波数・一次反応・GAR）と閾値テスト、8 軸案 config の事前検証
- `ac02f40` [main] docs: 動きの質（蛇らしさ・愛着）の対応表・測定・正直な限界を記録
- `e4b21cf` [main] 内部状態を 6 種の共通ダイナミクスに（Familiarity / Sleepiness 追加、Stress saturation 0.9、Energy と温度を分離）
- `046f94e` [main] EXHIBITION profile: 60〜90 秒の体験弧（NOTICE→LOOK→HESITATE/TRACK→APPROACH→ENGAGE→RELAX→LEAVE）と Fear 監査
- `6b5ef09` [main] Human Perception Pilot の準備: 6/8/10 Yaw の比較、匿名クリップ、提示順のランダム化、回答テンプレート、集計

#### 2026-09-25（5 コミット）

- `1274cd3` [main] docs: 床見守り機ブリーフ（2026-09-25）への実装前の 3 点（質問リスト・snake.yaml と MQTT 形式の案・フェーズ 1 計画）
- `1e0ab8a` [main] docs: Floor Watch 縦1本 — 実装前の 4 点（現状の要約・決定事項との差分・質問・フェーズ1計画）
- `ef4af08` [main] Floor Watch フェーズ 1: Home AI ↔ Serpens の Task / Event API（スキーマ・検証器・Loopback ブローカー・端点・モック）
- `21676a6` [main] フェーズ 1 レビュー対応: stop は何があっても受理、stop/FAULT で待ち行列破棄と人の操作までロック、stop 前の Task と retain 付き Task を拒否、LWT で OFFLINE、危険物の別枠通知、resolved は人だけ
- `40bae46` [main] Floor Watch フェーズ 2: 4 枚（全消灯/通常/斜め/線光）から出っ張りを判定する処理を合成画像で通す

#### 2026-09-26（3 コミット）

- `c18c69f` [main] Floor Watch レビュー対応: 基準床なし・高さ null・metal_disc・あご照明・2 段評価、API の周期送信/OFFLINE/再開制限、Mosquitto 実試験
- `8f3db67` [main] Floor Watch フェーズ 3: 自己位置（AprilTag + IMU + 歩容オドメトリ、座標系 home）を模擬で実装。開始条件は σ で書く
- `9d67094` [main] Floor Watch フェーズ 5: 縦一本を模擬でつなぐ（inspect_point → 移動 → 位置合わせ → 撮影 → 判定 → floor_finding → 通知）

#### 2026-09-27（1 コミット）

- `7fbafd0` [main] 2026-09-27 の回答を反映: カメラは解像度ごとに f（床見 UXGA / タグ探し VGA、すべて ASSUMED）、減速比は暫定 1:345

#### 2026-09-29（71 コミット）

- `69b7179` [main] engineering: establish mujoco floor watch baseline
- `ca76785` [main] engineering: RUN-ENG-0001 summary and handoff
- `32dc843` [agent/engineering-floor-watch, agent/engineering-h1-j1, agent/engineering-vision-sim, agent/floorwatch-farfield-roi] engineering: H0 friction coupon kit and H1 C044 joint plan
- `9fddcaf` [agent/engineering-floor-watch, agent/engineering-h1-j1, agent/engineering-vision-sim, agent/floorwatch-farfield-roi] engineering: RUN-ENG-0002 summary
- `140e156` [main] Revert "engineering: RUN-ENG-0001 summary and handoff"
- `6be5d67` [main] Revert "engineering: establish mujoco floor watch baseline"
- `38c6c3e` [archive/mqtt-fix-pre-rebase] api: fix MQTT safety_state retain race (DEC-SERPENS-0001)
- `d2ce23a` [main] api: fix MQTT safety_state retain race (DEC-SERPENS-0001)
- `1e0d717` [agent/engineering-floor-watch, agent/engineering-h1-j1, agent/engineering-vision-sim, agent/floorwatch-farfield-roi] safety: C044 torque reference from current vendor data, register ratio 0.287 -> 0.167
- `dd9369b` [agent/design-floor-watch] design: explore floor watch head and body concepts (SD-01 Bean + concentric knuckle, CSAR)
- `244f99c` [agent/design-floor-watch] design: DESIGN-2026-09-29-01 run summary and handoff
- `27cfea3` [main] api: build Endpoint on Paho normally, survive broker outage, close TOCTOU
- `9a8d417` [agent/design-floor-watch] design: record USER-DEC-SERPENS-DESIGN-0001 (CSAR adopted, provisional head yaw, SD-01 as CAD_CONCEPT)
- `ad48f17` [agent/engineering-h1-j1, agent/engineering-vision-sim] h2: line-light placement geometry (cheek vs brow) GEOMETRY_SIM
- `47aa97d` [agent/engineering-floor-watch, agent/engineering-h1-j1, agent/engineering-vision-sim, agent/floorwatch-farfield-roi] engineering: hardware-gap simulations for H0 friction and H1 actuator (WIP)
- `694dd3c` [agent/design-floor-watch] design: Fusion recovery (flush r45 back-plates, J1 concentric), SD-01A-E parametric study, CSAR choreography, ENTRY-0011 fallbacks
- `d9d49e2` [agent/engineering-h1-j1, agent/engineering-vision-sim] h2: WIP synthetic vision variations and trial runner (not yet verified)
- `649d89d` [agent/engineering-h1-j1, agent/engineering-vision-sim] floorwatch: metal_disc roundness must allow for object height (LR44)
- `6abee8c` [agent/engineering-h1-j1, agent/engineering-vision-sim] floorwatch: line-shaped blobs must not suppress shadow-only candidates
- `c5975e1` [agent/design-floor-watch] design: SD-01E head in recovery CAD, eval renders, J1 head-neck VISOR study
- `935d8a8` [agent/design-floor-watch] design: face and 3/4 eval renders (SD-01A/E), face read critique
- `5c2e250` [agent/design-floor-watch] design: SD-01E2 face-read fixes (eyes 35deg, inner glint, smaller nose, cream brow), consistent eval renders
- `e6768b5` [agent/design-floor-watch] design: SD-01E3 side mouth line, eye-ring light patterns
- `80c43e9` [agent/design-floor-watch] design: VISOR J1 interface in recovery CAD (3D sweep 0 mm3), E2/E3 renders, state update
- `340a80d` [agent/engineering-h1-j1, agent/engineering-vision-sim] floorwatch: stop line-only false alarms under pose error; flag 5 mm magnets
- `1a28796` [agent/engineering-floor-watch, agent/engineering-h1-j1, agent/engineering-vision-sim, agent/floorwatch-farfield-roi] engineering: hardware-gap simulations H0/H1/H2 with decision boundaries and test plans
- `e8f66cb` [agent/engineering-h1-j1, agent/engineering-vision-sim] h2: floor-vision hardware-gap results (VIS-0002) and pose from line (VIS-0003)
- `7526a53` [agent/engineering-h1-j1, agent/engineering-vision-sim] Merge remote-tracking branch 'origin/agent/engineering-floor-watch' into agent/engineering-vision-sim
- `4e838c9` [agent/engineering-h1-j1, agent/engineering-vision-sim] h2: brow line traced by column in synthetic images (VIS-0004)
- `dec9065` [agent/engineering-h1-j1, agent/engineering-vision-sim] floorwatch: only object candidates may suppress shadow/line candidates
- `4701c22` [agent/engineering-h1-j1, agent/engineering-vision-sim] h2: pose-from-line prediction is an exact projected line (20 s -> 63 ms)
- `d259f59` [agent/engineering-h1-j1, agent/engineering-vision-sim] floorwatch: bound carpet false alarms (shadow SNR guard, roughness threshold)
- `251e40d` [agent/engineering-h1-j1, agent/engineering-vision-sim] h2: VIS-0005 results; move pose-from-line into serpens/floorwatch/pose.py
- `e8a511d` [agent/engineering-h1-j1, agent/engineering-vision-sim] floorwatch: correct floor positions from the line's own pose estimate
- `abf779a` [agent/engineering-h1-j1, agent/engineering-vision-sim] floorwatch: merge fragments that share one shadow into one object
- `c47e8a3` [agent/engineering-h1-j1, agent/engineering-vision-sim] floorwatch: re-aim at the line's actual position, and when off-centre
- `ce75a3d` [agent/engineering-h1-j1, agent/engineering-vision-sim] floorwatch: merging shadow-sharing fragments must not remove an object
- `948ecc4` [agent/engineering-h1-j1, agent/engineering-vision-sim] test: pin the merge-guard regression to the exact sweep trials that failed
- `f8178ef` [agent/engineering-h1-j1, agent/engineering-vision-sim] lessons: verify regression tests fail without the fix; suspect synthetic scoring first
- `cf715c3` [agent/engineering-h1-j1, agent/engineering-vision-sim] h2: VIS-0006 record and results (pose correction + mission re-aim)
- `177f9ce` [agent/engineering-h1-j1, agent/engineering-vision-sim] docs: measured test count 530 passed / 1 skipped on engineering-vision-sim
- `ad0b487` [agent/engineering-h1-j1, agent/engineering-vision-sim] floorwatch: point-light falloff, flat-field calibration, relative line threshold
- `260e631` [agent/engineering-h1-j1, agent/engineering-vision-sim] h2: VIS-0007 record (point-light falloff, flat-field, LED placement)
- `57f3d89` [agent/engineering-h1-j1, agent/engineering-vision-sim] h2: realistic lighting x head pose (VIS-0008); pose-aware flat field (opt-in)
- `1c7a4cd` [agent/engineering-h1-j1, agent/engineering-vision-sim] docs: measured test count 535 passed / 1 skipped (VIS-0008)
- `83b8538` [agent/engineering-h1-j1, agent/engineering-vision-sim] floorwatch: CSAR execution conditions (child-safe attention rules)
- `e5cff35` [agent/engineering-h1-j1, agent/engineering-vision-sim] docs: CSAR and detector-default decisions; measured count 545 passed / 1 skipped
- `89b693a` [agent/engineering-h1-j1, agent/engineering-vision-sim] api: optional Task.child_near hint (can only make Serpens more cautious)
- `88a6891` [agent/engineering-h1-j1, agent/engineering-vision-sim] docs: measured test count 547 passed / 1 skipped
- `b34bf85` [agent/engineering-h1-j1, agent/engineering-vision-sim] api: fix MQTT safety_state retain race (DEC-SERPENS-0001)
- `625f671` [agent/engineering-h1-j1, agent/engineering-vision-sim] api: build Endpoint on Paho normally, survive broker outage, close TOCTOU
- `097ed77` [agent/engineering-h1-j1, agent/engineering-vision-sim] safety: finger-pinch gap rule tightened to < 5 mm or >= 25 mm, with a checker
- `7e8183d` [agent/design-floor-watch] design: replies to Engineering (line light brow, light placement 40->15x / 33->11x, skid), LIGHT+SKID overlay
- `b9a1ed1` [agent/design-floor-watch] design: chin-shield skid option (ivory, follows jaw), renders
- `eb51e21` [agent/design-floor-watch] design: 6th-motor silhouette comparison (5 servo / head yaw / body yaw)
- `7ea8fb4` [agent/engineering-h1-j1, agent/engineering-vision-sim] floorwatch: CSAR retreat goes forward around the object, never backward
- `960bc85` [agent/design-floor-watch] design: ENTRY-0011 option A applied per part (battery overruns tail by 12 mm)
- `3b901c6` [agent/design-floor-watch] design: handoff 0003
- `e12949f` [agent/engineering-h1-j1, agent/engineering-vision-sim] behavior: Floor Watch never reverses blind at the mat edge
- `7a5f536` [agent/engineering-floor-watch, agent/floorwatch-farfield-roi] engineering: contact-safety and cable-routing hardware gaps
- `991bdd4` [agent/engineering-h1-j1, agent/engineering-vision-sim] h2: multi-LED 3-D shadows; Design's 2+2 LED layout keeps detection
- `ed7e556` [agent/design-floor-watch] design: record user decisions (CSAR capture, safety envelope / 145 deg), sleep pose within 145 deg, child anthropometry sources
- `6a1af24` [agent/design-floor-watch] design: charging renders re-done at 145 deg yaw sum, turn-and-forward departure guide
- `bd2bad6` [agent/design-floor-watch] design: record USER-DEC-SERPENS-0004 (head yaw vs 145), design-side head yaw sweep (propose +-30 deg)
- `8edc87c` [agent/engineering-h1-j1, agent/engineering-vision-sim] h2: J1 head-height hold recovers patrol when the head sinks (VIS-0009)
- `f02cc9b` [agent/design-floor-watch] design: reply to ENTRY-0006 (stop <= 52 deg fits knuckle, dome back-plate gives 21 mm cable clearance), scute renders
- `622cf76` [agent/engineering-h1-j1, agent/engineering-vision-sim] test: MQTT live tests must not match stale retains or same-tick timestamps
- `70601cd` [main] test: MQTT live tests must not match stale retains or same-tick timestamps
- `6a9fd2e` [agent/engineering-floor-watch] safety: yaw-sum limit 145 deg (4 layers), mechanical torque limiter design values, provisional child contact thresholds
- `8f5fd32` [agent/engineering-h1-j1, agent/engineering-vision-sim] docs: measured test count 563 passed / 1 skipped
- `32e73e4` [agent/engineering-scoop] scoop: MuJoCo model for the head scoop + lid, and exploration showing 0% success in the briefed range

#### 2026-09-30（45 コミット）

- `6eecff1` [agent/engineering-scoop] scoop: short ramp, beak sweeper lid, ride/enter metrics, hardware test plan draft (WIP; no sweep results yet)
- `98b5b1e` [agent/design-floor-watch] design: HEAD INTAKE STUDY 01 (beak intake test piece A x2, head layout, interference report, STLs)
- `47ba107` [agent/engineering-scoop] scoop: beak sweeper study (144 beak + 48 baseline designs, N=30 top-3, sensitivities), latch fix, report, test plan
- `06ba7bd` [agent/design-floor-watch] design: gap check of SD-01 (6 poses, 0.5 mm voxels; screening only). Flank V wedges at every bent joint and chin wedge on head pitch-down cannot be removed by shape; ENTRY-D-0003 / OPEN-014. Fixed a voxelizer bug and withdrew a false tail finding.
- `d2ae08f` [agent/engineering-scoop] scoop forms: open-bottom hood + gate, and mechanisms A (side sweepers) B (roller belt) C (brush) D (cup) E (hook), F backstop, G funnel
- `494e95a` [agent/engineering-floor-watch] safety: flank V handled by cover (primary), torque limit auxiliary; allowed torque recomputed per contact radius; limiter-window discrepancy recorded OPEN
- `b760200` [agent/engineering-scoop] scoop forms: study of shapes and mechanisms A-G; open-bottom hood + gate as baseline (N=30), gate/no-gate, step ablation, straddle classification
- `23eb267` [agent/design-floor-watch] design: KNUCKLE DRUM (full-cylinder knuckle flanks) cuts body-layer gaps ~90%, r ranges per joint/pose, J1 pitch series, head lower-rear fill (172->2 mm3). FLANK SKIRT flap concept tried: did not reduce gaps. Corrects ENTRY-D-0003 (2)(3). ENTRY-D-0004.
- `96c74a9` [agent/engineering-floor-watch] safety: Design's contact radii (ENTRY-D-0004) -> J3-J5 allowed torque ~0.25 N.m vs 0.45 N.m cap (OPEN); J1 lowering side needs range limit/cover; J1 range -5..+10 proposed; hollow head mass check
- `24d0b58` [agent/design-floor-watch] design: KNUCKLE DRUM approved. Print-ready test pieces (knuckle R46/R34 with gauge; funnel hood B1 with lip/spacers/clamp/skirts/rod hole), mass addendum (solid 61 g / hollow 56 g). Reports keep the ~20x residual banner; back-plate layer and J1 pitch stay open.
- `5db4e07` [agent/design-floor-watch] design: 00_STATUS folder (progress, user to-do, engineering asks, file map). Keeps the ~20x residual banner.
- `7510a92` [agent/engineering-scoop] handoff: progress and pending requests (scoop forms study, safety notes); WIP passive variants (curtain, skirt, bump floor, gate force) - untested
- `64b9487` [agent/engineering-scoop] scoop forms: follow-up study (tolerance, passive curtain, skirt + bumpy floor, gate force), probabilistic tolerance, updated recommendation, test plan for piece B
- `dcf5dc7` [agent/engineering-floor-watch] safety: record KNUCKLE DRUM mass additions (PROPOSED), correct J7 static torque (0.204 N.m from robot.yaml lifted mass), passive-curtain pinch guide; J1 range awaits User
- `3a9b1a8` [agent/engineering-scoop] handoff: mark follow-up requests done; J1 range awaits User
- `8e1027a` [agent/design-floor-watch] design: test piece B2 (step t and clearance c independent), HEAD INTAKE on E3 study, plate-layer options (a) DRUM-FULL / (b) concentric dome caps, J1 pitch window. ~20x residual banner kept for current CAD.
- `5166111` [agent/design-floor-watch] design: no floor-touching parts in front of the mouth (15,290 mm3 -> 0), c measured from floor (B2 plate corrected), J1 intake window, lighting recompute with hood. Fusion disconnected: CAD not changed. ~20x residual banner kept.
- `001158d` [agent/design-floor-watch] design: answers to ENTRY-D-0006 (c from floor, J1 resolution/compliance, VL53L1X datasheet, HG-H2 layout comparison: side-skid-front LED best). No post-hoc tuning; 5.7 N / 0.25 N.m provisional. ~20x residual banner kept.
- `8558eb0` [agent/engineering-scoop] scoop: check whether the 0% at a 0.002 mm step is an idealization (chamfer, roughness, solver)
- `fa60caa` [agent/design-floor-watch] Design: B2 print list, t=0.05 howto, log v2, relabel engineering answers as Design estimate, hood-lift concept (ENTRY-D-0009)
- `53f8c1a` [agent/design-floor-watch] Design: keep integration-log.md out of the Design branch (it is maintained in the main working dir, as for ENTRY-D-0001..0008)
- `03ddfc8` [agent/engineering-h1-j1] H1/H2: J1 backlash/dead-band/push-force tests, floor-contact calibration, ToF cliff (skid front), LED recheck
- `f12dc4e` [agent/engineering-floor-watch] safety: framework for hood-lift (cloche drop) force, speed and edge pressure
- `fca5ed3` [agent/engineering-scoop] scoop: thin film edge (0%) and hood lift / cloche drop (steps avoided), cover vs carry, comparison table
- `929319d` [agent/design-floor-watch] Design: Fusion CAD reflection (mouth-front layout, hood-lift, J1 cover+stopper STL), J1 sign correction (CAD + = head down), cover inspection (ENTRY-D-0010)
- `a894824` [agent/engineering-h1-j1] H1: J1 sign check (J1-0), stopper load estimate, answer to Design's three J1 points
- `e2119e2` [agent/design-floor-watch] Design: integrated cute-deformed head E3 v1 (user-approved rearrangement, originals kept), hood-lift J1-servo recompute (ENTRY-D-0011)
- `2e66d7a` [agent/engineering-h1-j1] autonomy: Design<->Engineering self-running loop (CLAUDE.md section, docs/autonomy mirror)
- `63261c3` [agent/engineering-scoop] scoop: intake vs wall clearance sweep with friction priors (input to the J1 range optimization)
- `f86d90a` [agent/engineering-h1-j1] J1 range and stopper load optimization (Pareto, knee, Monte Carlo over priors)
- `7ce43bc` [agent/engineering-h1-j1] config: add Floor Watch J7 (J1) working range keys (provisional); existing J7 limits unchanged
- `53fee9b` [agent/engineering-h1-j1] autonomy: refresh docs/autonomy mirror (REQUESTS after R-002)
- `1e930db` [agent/design-floor-watch] Design: autonomy loop round 1 - R-003 stopper v2 + +/-4deg hazards, 3 cover variants (A recommended), ToF board placement, hood-lift 3 options (L2 TPU leaf), integrated head v2, B2 test procedure, HARDWARE_TODO HT-008..011 (ENTRY-D-0012)
- `285f014` [agent/engineering-scoop] scoop: cover-mass effect on intake, B2 observation handover and compare script
- `7271c51` [agent/design-floor-watch] Design: User adopted cover A; add J1 COVER A ADOPTED STL, REQUESTS R-011 done (ENTRY-D-0013)
- `9344cf9` [agent/design-floor-watch] Design: fix REQUESTS R-011 row
- `0434ebf` [agent/engineering-h1-j1] R-009: enforce the Floor Watch J7 range and near-limit speed on the device and PC side (default OFF)
- `89b65a9` [agent/engineering-h1-j1] HT-001 sign-check tool (mock-first) and CSV templates for the force / impact measurements
- `e70b460` [agent/engineering-h1-j1] J1 optimization update: Design's +/-4 deg, pad k/thickness sweep, D/B prior grid, cover effect; docs
- `227e7da` [agent/design-floor-watch] Design: cover A integral with neck collar, head v2 mass/COG estimate (ENTRY-D-0014)
- `e30262e` [agent/engineering-floor-watch] safety: L2 leaf-ear force check and the hood-down 1-bit requirement (PROVISIONAL)
- `d5972f2` [agent/engineering-h1-j1] docs: verification status for the J1 range/guard/ToF/LED items; pin the wide-J7 test and record the firmware compile
- `9cd853b` [agent/engineering-h1-j1] autonomy: refresh mirror (REQUESTS R-017/R-018)
- `5bfaabc` [agent/engineering-h1-j1] R-012 answer (pad options P1/P2, window bearing) and head COG y tolerance (<= 4 mm, ASSUMED); HT-012
- `41039f1` [agent/design-floor-watch] Design: answer E-0012/E-0013/E-0014 (P1 default + P2 STLs, wide-angle hazards +6..+9/-7.5 deg, head shell mass/COG, hood end-stop hall sensing, servo y dim); retract E3MPa and dummy-mass (ENTRY-D-0015)

#### 2026-10-01（2 コミット）

- `80e8be3` [agent/design-floor-watch] Design: consolidated print list (print_all P0-P3), new HT-008/009/010/011 test pieces, ToF board length fix (ENTRY-D-0016)
- `ee1a4fe` [agent/design-floor-watch] Design: PRODUCT_HISTORY.md (detailed product trajectory, append-only update log) and AUTONOMY step 5b (ENTRY-D-0017)

### 3. 判断の記録（各ブランチの `agent/DECISIONS.md` の見出し。重複は 1 つ）

- 2026-09-14 — Autonomous Product Development OS を導入した（`agent/design-floor-watch`, `agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-14 — トルク比を「安全上限に対する割合」に変えた（`agent/design-floor-watch`, `agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-14 — フォルダを C:/2026/serpens へ移動しないことにした（`agent/design-floor-watch`, `agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-14 — 出力先を RobotInterface で差し替え可能にした（Phase 3）（`agent/design-floor-watch`, `agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-14 — 制御ループ例外時、fault の公開を緊急停止の後に移した（`agent/design-floor-watch`, `agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-14 — 機体の ARM は「人の開始操作」でしか行わない（`agent/design-floor-watch`, `agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-14 — 通信断で「とぐろ」へは移らず、その場で保持する（`agent/design-floor-watch`, `agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-15 — 3D 物理は MuJoCo を第一候補にする（まだ導入しない）（`agent/design-floor-watch`, `agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-15 — 摩擦の値は belly プロファイルだけが持つ（`agent/design-floor-watch`, `agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-15 — 機体の状態を7つにし、機体の中に仮想サーボバスを置いた（`agent/design-floor-watch`, `agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-16 — ARM に boot_id を持たせ、再起動後の自動再走行を機体側で拒否する（`agent/design-floor-watch`, `agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-16 — ELECTRICAL_SAFETY_GATE を自律走行の開始条件に入れ、接触→脱力は要求整理で止めた（`agent/design-floor-watch`, `agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-16 — Vision は「観測器」として差し込み、見失ったら保持して自動再開しない（`agent/design-floor-watch`, `agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-16 — サーボは STS3215-C044（7.4V/1:191）。トルクは4つに分ける（`agent/design-floor-watch`, `agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-16 — 可動域を「意味の違う3つ」に分け、±50°（CONDITIONAL）を clamp にした（`agent/design-floor-watch`, `agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-18 — 実画像経路は観測器で差し込み、録画は実機の開始条件を満たさない。見失い後は人が確認する（`agent/design-floor-watch`, `agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-21 — Human Evaluation を最優先ゲートにし、身体構成の決定は Pilot 後へ（`agent/design-floor-watch`, `agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-21 — 動きの語彙（primitives）と文法（grammar, config）を分け、人格を config で変える（`agent/design-floor-watch`, `agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-25 — Floor Watch: Home AI との境界は Task / Event API だけ。MQTT を関節制御に使わない（`agent/design-floor-watch`, `agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-26 — Floor Watch の inspect は展示の行動を通さず、Task が無ければ止まっている（`agent/design-floor-watch`, `agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-26 — Floor Watch 画像処理: 基準床を使わない / 測れない高さは null / 鏡面の円盤は危険物側 / Mosquitto はテストが起動する（`agent/design-floor-watch`, `agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-26 — 自己位置の開始条件は σ と局所センサーの健全性で書く（出どころの名前の置換にしない）（`agent/design-floor-watch`, `agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-29 — C044 のトルク参照値を現行資料へ更新し、トルク制限レジスタ比を 0.287 → 0.167 に下げた（User 承認）（`agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`）
- 2026-09-29 — CSAR の実行条件は Serpens 側で決め、「子どもかもしれない」を安全側に倒す（`agent/engineering-h1-j1`, `agent/engineering-vision-sim`）
- 2026-09-29 — DEC-SERPENS-0001 MQTT safety_state retain race fix（PahoBroker はネットワークスレッド上では PUBACK を待たない）（`agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `archive/mqtt-fix-pre-rebase`, `main`）
- 2026-09-29 — MQTT 残りの 3 件: Endpoint(PahoBroker) の通常構築 / 切断中の publish / close の TOCTOU（DEC-SERPENS-0001 の続き）（`agent/engineering-h1-j1`, `agent/engineering-scoop`, `agent/engineering-vision-sim`, `main`）
- 2026-09-29 — MuJoCo の pitch 軸の符号を直した（+ で頭が上がる）（`agent/engineering-floor-watch`, `agent/engineering-h1-j1`, `agent/engineering-vision-sim`, `agent/floorwatch-farfield-roi`）
- 2026-09-29 — 安全値の採用: 対象年齢・角度合計 145°・機械式トルクリミッター（User 決定 DEC-USER-0002）と、接触しきい値の暫定導出（`agent/engineering-floor-watch`）
- 2026-09-29 — 床見の検出の既定を変えた（H2 の合成で見つけた穴）。照明の姿勢合わせは既定で切（`agent/engineering-h1-j1`, `agent/engineering-vision-sim`）
- 2026-09-29 — 頭内取り込み機構（スコップ＋フタ）は第 2 段階の製品機能。今回の作業はその成立性調査（User 決定）（`agent/engineering-scoop`）
- 2026-09-30 — Design の J1 の 3 点（符号・上げ側 +3° への縮小・ストッパーの荷重）への Engineering の回答（`agent/engineering-h1-j1`）
- 2026-09-30 — Design の r（ENTRY-D-0004）での許容トルク: J3〜J5 は約 0.25 N·m（0.45 N·m の上限と不一致 = OPEN）、J1 の下げる側は力の制限では成立しない（`agent/engineering-floor-watch`）
- 2026-09-30 — J1（J7）の動作範囲とストッパーの荷重の最適化: 範囲 [−4°, +3°]（覆いあり）、鋼ダウエル + TPU パッド（厚 3〜6 mm）、窓の端の手前 ≤ 40 °/s（PROVISIONAL）（`agent/engineering-h1-j1`）
- 2026-09-30 — J1（頭ピッチ）の不感帯・バックラッシ・押す力を H1 の試験計画へ / 床接触の較正の手順 / ToF は崖の 2 値・横スキッドの前端に置く / 斜め LED の追試（`agent/engineering-h1-j1`）
- 2026-09-30 — R-009: Floor Watch の頭（J7）の範囲・速さの強制（既定オフ）を機体側（device_motion / ファーム）と PC 側に入れた。ENTRY-E-0011 の 2 点を訂正（膝・TPU パッド）（`agent/engineering-h1-j1`）
- 2026-09-30 — R-012（TPU パッドは P1: 95A 級 E 20〜30 MPa・t 3 mm / P2: E ≈ 10 MPa・t 4〜6 mm）と頭の重心 y の許容（≤ 4 mm）（ENTRY-E-0014）（`agent/engineering-h1-j1`）
- 2026-09-30 — フード昇降（cloche drop）の落とす力・速さ・縁の圧力の確認の枠組み（`flank_v.md` §F、`hood_lift.py`）（`agent/engineering-floor-watch`）
- 2026-09-30 — 取り込みの追加検討: 段差の許容差は実質ゼロ、受け身の垂れ布は範囲外、ゲートは 2.8 N 以下で成立（推奨を更新）（`agent/engineering-scoop`）
- 2026-09-30 — 案 L2（TPU の板ばね耳）の力の確認と、固着の検出の要求（ENTRY-E-0013）（`agent/engineering-floor-watch`）
- 2026-09-30 — 段差 0.002 mm の 0% は理想化か: 面取り・粗さ・接触設定で確かめた。シミュレーションでは「口の前縁の段差はほぼゼロが必須」と結論する（`agent/engineering-scoop`）
- 2026-09-30 — 段差に強い機構: 薄いフィルムの縁は 0%、フード昇降（cloche）は段差を避けて成立（位置合わせ ±5 mm）。推奨の比較表を更新（`agent/engineering-scoop`）
- 2026-09-30 — 脇の V は「カバーで塞ぐ」を主にする。トルク上限は補助。許容トルクは r で再計算（OPEN: リミッターの窓との食い違い）（`agent/engineering-floor-watch`）
- 2026-09-30 — 覆い（襟が前へ 5〜7 mm、+1.1 g）は取り込みを変えない / B2 の観察の受け渡しと比較スクリプト（`agent/engineering-scoop`）
- 2026-09-30 — 質量の追加（KNUCKLE DRUM）を記入、J7 の静的トルクを訂正（0.020 → 0.204 N·m）、受け身の垂れ布の挟み込みの目安（OPEN は継続）（`agent/engineering-floor-watch`）
- 2026-09-30 — 頭内取り込みの「形と機構」の探索: 口を開放底のフード + ゲートにすると機構なしで入る（推奨 = フード + 漏斗。絨毯は未対応）（`agent/engineering-scoop`）
- 2026-09-30 — 頭内取り込みは「巻き込みくちばし」で調べたが、1 円玉・電池・立方体は保持 0（推奨案なし）（`agent/engineering-scoop`）

### 4. Design ↔ Engineering の ENTRY の索引（`ai-shared/integration-log.md`。git 管理外）

- **ENTRY-D-0001**（Design Agent）番号の付け方 ＋ 機械ストッパー ±52°・関節渡りのケーブル・首 J1 の荷重（CAD_CONCEPT）
- **ENTRY-D-0002**（Design Agent）HEAD INTAKE STUDY 01（User の依頼: 巻き込みくちばしの試験片と頭部レイアウト）— シミュへの推奨と衝突する点
- **ENTRY-D-0003**（Design Agent）すき間チェック（全角度で 5〜12 mm を作らない）・J2 の襟・Head Yaw の角度・小部品
- **ENTRY-D-0004**（Design Agent）5〜12 mm のすき間の r の範囲・FLANK SKIRT STUDY の結果・KNUCKLE DRUM（形の直し）・J1 ピッチ — **ENTRY-D-0003 (2)(3) の訂正を含む**
- **ENTRY-D-0005**（Design Agent）KNUCKLE DRUM の承認後: 試験片（円柱肩 R46 / R34、A1 印刷用）・質量の追記・試験片 B（漏斗つき開放底フード）
- **ENTRY-D-0006**（Design Agent）試験片 B2（段差 t とすき間 c を独立）・頭への統合（HEAD INTAKE on E3）・背板の層の 2 案・J1 ピッチ −5〜+10°
- **ENTRY-D-0007**（Design Agent）ENTRY-D-0006 の確認事項への回答 ＋ 口の前に床に触れる部品を置かない配置
- **ENTRY-D-0008**（Design Agent）ENTRY-D-0006 の確認事項 4 点（c の基準・J1 の分解能と追従・ToF の最短距離・斜め LED の移設と合成視覚）への回答
- **ENTRY-D-0009**（Design Agent）B2 の印刷リスト・t = 0.05 mm・エンジニアリング回答の位置づけの訂正・フード昇降（cloche drop）の概念
- **ENTRY-D-0010**（Design Agent）Fusion 復帰後の CAD 反映（口の前の配置・フード昇降・J1 カバーとストッパー）と **J1 ピッチの符号の訂正**
- **ENTRY-D-0011**（Design Agent）頭 E3 の統合版（User の許可で組み替え、デフォルメ調のかわいい仕上げ）
- **ENTRY-D-0012**（Design Agent）ストッパー v2（窓 θ_E −4〜+3°）・±4° の危険体積・上の輪の覆い 3 案・ToF 小基板・フード昇降 3 案・頭 E3 統合版 v2
- **ENTRY-D-0013**（Design Agent）上の輪の覆い — User が案 A を採用（R-011 完了）
- **ENTRY-D-0014**（Design Agent）覆い A の首との一体化・頭 E3 v2 の質量・重心の見積もり（User の「まだ再開できない？」を受けて、待機せずに進められる作業）
- **ENTRY-D-0015**（Design Agent）R-012 の選択（P1 / P2）・+6〜+9° と −7.5° の危険体積・頭の質量と重心（殻の仮定）・フードの端の 1 ビット・J1 サーボ箱の y・訂正（E 3 MPa の撤回、ダミー質量の撤回）
- **ENTRY-D-0016**（Design Agent）印刷するものの整理（2026-10-01）と ToF 基板の長さの訂正
- **ENTRY-D-0017**（Design Agent）軌跡の文書（`ai-shared/PRODUCT_HISTORY.md`）の新設と、以後の更新ルール
- **ENTRY-E-0001**（Engineering）番号の付け方 / CSAR R3（離れ方）/ ライン光・照明 2 灯・スキッド
- **ENTRY-E-0002**（Engineering）照明の左右 2 灯（影が斜め）
- **ENTRY-E-0003**（Engineering）軸の上のケーブル・機械ストッパー・Head Yaw の範囲・角度合計 145° の実装・接触しきい値
- **ENTRY-E-0005**（Engineering）脇の V（曲げた姿勢で胴の脇にできるくさび）の扱い・閉じる力の許容トルク
- **ENTRY-E-0006**（Engineering）J3〜J5 の挟み込みの許容トルク・J1 の動作範囲・頭に足す質量
- **ENTRY-E-0007**（Engineering）質量表（KNUCKLE DRUM）・J1 の動作範囲・取り込み機構の頭下げ・J7 の静的トルクの訂正
- **ENTRY-E-0008**（Engineering）Design の見積もり（J1・ToF・斜め LED）の確認・段差に強い機構（薄いフィルム・フード昇降）・フード昇降の安全の枠組み
- **ENTRY-E-0009**（Engineering）J1 の符号・上げ側を +3° に狭めてよいか・ストッパーの荷重
- **ENTRY-E-0010**（Engineering）自律運用の仕組み（Design ↔ Engineering が User に聞かずに `ai-shared/` で回す）
- **ENTRY-E-0011**（Engineering）J1（J7）の動作範囲・ストッパーの荷重の最適化（R-002）と、Design への形状条件（R-003）
- **ENTRY-E-0012**（Engineering）R-009（範囲・速さの強制の配線。既定オフ）・ENTRY-E-0011 の未確認部分の詰め（覆い・TPU パッド・D/B の幅）・B2 の受け渡し・HARDWARE_TODO の見直し・R-012 / D-0011 への回答
- **ENTRY-E-0013**（Engineering）実機なしで進められる項目（ファームのコンパイル・有効化の先行確認・案 L2 への回答・R-004）
- **ENTRY-E-0014**（Engineering）R-012（TPU パッドの材質・窓の端面の支圧）と 頭の重心 y の偏りの許容

（30 件）

<!-- END:auto -->
