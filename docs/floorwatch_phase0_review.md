# Floor Watch 縦 1 本 — 実装前の 4 点（2026-09-25）

プロンプト「serpens_floorwatch_claude_code_prompt」への最初の返答。**まだ実装していない。**
前提: 実サーボ・EEPROM・ファーム書き込みは行わない（本セッションの禁止事項）。HARDWARE_VERIFIED = 0、HUMAN_EVALUATED = 0。

---

## 1. リポジトリの現状（どこに何があるか）

| 責務 | 場所 | 状態 |
|---|---|---|
| **出力先の抽象** `RobotInterface`（`send / hold / apply_stop_state / on_operator_start / on_clear_emergency / poll / device_status / close`） | `serpens/robot.py` | `DirectRobot`（サーボバスへ角度を直接書く）と `LinkRobot`（`serpens/link/robot.py`、ESP32 へ**歩容パラメータ**を送る）。`--robot direct/link` |
| PC→機体の契約（v2）: フレーム `A5 5A ver type seq len payload crc16`、HEARTBEAT / ARM(boot_id) / DRIVE / HEAD / BODY / TORQUE / STOP / EMERGENCY / CLEAR_FAULT / LIMITS / PING、NACK 理由 | `serpens/link/protocol.py`, `messages.py`, `docs/link_protocol.md` | SOFTWARE_VERIFIED（USB CDC のみ。MQTT 無し） |
| **ESP32 側 Safety State Machine**（7 状態: BOOT / DISARMED / ARMED_HOLD / DRIVING / FAULT_HOLD / EMERGENCY_LATCHED / TORQUE_DISABLED） | 仮想機体 `serpens/link/device.py`、実ファーム `firmware/serpens_esp32/`（コンパイルのみ、未書き込み） | Heartbeat 400ms / DRIVE TTL 300ms / DISARM 2000ms（`config/robot.yaml` `link`、**要実測**）。FAULT_HOLD からの復帰は DISARMED まで、再走行は明示 ARM。ARM は boot_id を名指し（再起動後の自動再走行を拒否） |
| 機体側の上限強制 | `serpens/link/device_motion.py` | 角度・速度・歩容値の範囲外は NACK。呼吸は機体側で生成 |
| PC 側の停止（RUN / HOLD / DISABLED / EMERGENCY ラッチ） | `serpens/safety.py` `StopSupervisor`、開始条件 `autonomy_blockers`（実観測・校正・駆動リンク・トルク上限・ELECTRICAL_SAFETY_GATE） | 実機は待機から始まり、ゲート 8 項目が未実測なら自律走行しない |
| 安全の絶対値（トルク 4 分割、最小曲げ半径、1m 以内 8cm/s、温度、接触→脱力 20ms 未達） | `config/robot.yaml` `safety_limits`、`docs/safety_limits.md`、`docs/contact_release_requirements.md` | 接触検出の要求 Q1〜Q6 は未回答（決定事項 7 と同じ立場） |
| 行動: 内部状態 6 種 → 効用 → 10 状態 | `serpens/behavior/{internal_state,utility,fsm,brain}.py` | `docs/internal_state_model.md` |
| **gaze / glance / micro-behavior** | 語彙 `serpens/behavior/primitives.py`（look_at / glance_away / flick / tilt / twitch / eye_flash / sag / nuzzle / stretch / hold_breath / freeze）、文法 `serpens/behavior/grammar.py`（`config/robot.yaml` `behavior.grammar`、`config/profile_exhibition.yaml`） | 「物を見る → 人を見る → 物を見る」は look_at と grammar の列で表現できる |
| 移動の段取り（stop-and-go 巡回、2 段階の接近、退避） | `serpens/behavior/locomotion.py`、`controller.py` | 目標点は 2D 座標。**inspect_point 相当の「地点へ行く」は無い** |
| 知覚（外部固定カメラ前提） | `serpens/perception/`（ArUco DICT_4X4_50、床ホモグラフィ、YOLO 人物、`camera_observer.py`） | AprilTag / 機体カメラ / IMU は無い |
| 頭部 I/O | `serpens/hw/head_io.py`（XIAO、行プロトコル: ToF 1 個・タッチ 2・目 LED） | カメラ・照明 LED・スピーカー・BNO085 は無い |
| 世界の状態と出どころ | `serpens/world_state.py`（GROUND_TRUTH_SIM / ARUCO / PERSON_DETECTOR / DEAD_RECKONING） | 不確かさの欄は無い |
| シミュレーション | `serpens/sim/`（簡易、車輪の横滑りゼロ）、`simulation/mujoco/`（任意依存）、`config/robot_yaw{6,8,10}*.yaml` overlay | 関節数は config で変わる（`body_joint_names` は最初の pitch 軸より尾側の yaw） |
| 検証の語彙 | `serpens/verification.py`、`docs/verification_status.md` | 5 段。**`docs/verification.md` は無い**（`verification_status.md` がその役） |

---

## 2. 決定事項と既存コードの差分・矛盾

| # | 決定事項 | 既存 | 差分 / 矛盾 | 扱い |
|---|---|---|---|---|
| 1 | 5 サーボは MVP 専用。関節数を前提にしない | 9 軸が既定。関節数非依存は歩容・機体側・overlay で済み。**首 J7 / 頭ヨー J8 / 頭ロール J9 の名前はコードに固定**（`motion/poses.py` `NECK/HEAD_YAW/HEAD_ROLL`、`gait.py` 旧 `NECK_JOINT`）。5 軸案は頭ヨー・頭ロールが無い | primitives の glance / flick / nuzzle は J8 前提 → **5 軸では「頭を振る」語彙が使えない**。config で「頭ヨー無し」を許す変更が要る | 質問 Q1 |
| 2 | RobotInterface を維持、ROS 2 不要 | 一致 | — | — |
| 3 | MQTT はシステム間の境界だけ | MQTT は未使用。PC⇄機体は USB CDC | 追加のみ。既存契約は変えない | フェーズ 1 |
| 4 | PC は Task だけ送る | **PC は歩容パラメータと HEAD 角を送っている**（`LinkRobot`）。これは「Home AI → Serpens PC」ではなく「Serpens PC → 機体」の話 | 決定 4 は Home AI ↔ Serpens の境界と解釈する。Serpens 内部の PC → 機体の DRIVE/HEAD は維持 | 質問 Q2 |
| 5 | 既存 Heartbeat + TTL + SSM を使い、時間は実測で | 一致（400 / 300 / 2000ms は「要実測」と注記済み） | Wi-Fi 経由になるなら値が変わる。**機体⇄PC は USB のままか** | 質問 Q3 |
| 6 | 通信断は ESP32 単独で HOLD、帰還は別機能 | 一致（HOLD。とぐろへは移らない: DECISIONS） | — | — |
| 7 | load で接触検出は HARDWARE_UNVERIFIED、試験コードと記録を用意 | 一致（`contact_release_requirements.md` に Q1〜Q6、`brain` の grab は PC 側 800ms） | 試験スクリプトは無い | フェーズ 4 |
| 8 | トルクリミッター・外れる関節は研究候補 | 一致（未実装） | 決定 7 の「脱力すれば解放」も 1:345 では未確認 | — |
| 9 | 誤飲リスクは複合判定 | 無し | 新規（フェーズ 2） | — |
| 10 | 固定カメラ 0 台 | **知覚は外部固定カメラ前提**（ArUco 2 枚を上から見る、床ホモグラフィ、人物検出、自己位置の開始条件 `require_real_pose` は「ArUco」を要求） | 自己位置・人物検出・開始条件の前提が丸ごと変わる。EX-01 展示機の経路は残す必要がある | 質問 Q4 |
| 11 | 止まる → 静止画 → 判定 | 無し。`CameraObserver` はストリーム前提 | 新規（頭カメラは XIAO Sense 側） | フェーズ 2/4 |
| 12 | 発見 → 確認 → 位置 → 通知 の 1 本 | 通知の仕組みは無い | 新規 | フェーズ 1/5 |
| 13 | 発見を身体表現で伝える | primitives / grammar で可能。**「物を見る」は床の一点を見る = 首 Pitch 下向き + 頭ヨー**。5 軸案では頭ヨーが無い | 決定 1 と同じ | 質問 Q1 |
| 14 | 別リポジトリ、Task / Event API だけ | 一致（`CLAUDE.md`「Home AI 本体をここに作らない」） | Home AI 側モックはこのリポジトリの `tests/` に置く | — |
| HW | XIAO ESP32S3 + Seeed サーボドライバ（胴）、XIAO Sense（頭） | ファームは XIAO_ESP32S3 向け（FQBN 一致）。頭の XIAO は行プロトコルで ToF 1・タッチ 2・LED 2 | **タッチセンサが消える**（撫で応答 PETTED の入力）。ToF が 2 個、カメラ・照明・WS2812B・スピーカーが増える | 質問 Q5 |
| HW | BNO085 IMU | 無し（`safety_limits.md`「持ち上げ検知が作れない」） | 新規。IMU は胴か頭か | 質問 Q6 |
| HW | 2S リポ 1000mAh + 5V DC-DC + ヒューズ | 12V テザー前提の注記。ELECTRICAL_SAFETY_GATE 8 項目未実測 | サーボ電源（7.4V 版の入力上限、番地 14 = 8.0V 初期値 vs 満充電 8.4V）は `brief_2026-09-25` B1 のまま未解決 | 質問 Q7 |
| 安全 | 「体が輪を作る姿勢を受け付けない」 | 機体側は**各関節の範囲**だけ検査。合計角の上限は無い。PC 側に自己干渉検査（中心線間 60mm）と休憩姿勢 `rest_arc`（合計 240°）がある | 合計角の上限を機体側に足すと `rest_arc` は禁止になる | 質問 Q8 |
| 安全 | 下向き ToF で段差 → 前進停止 | ToF は頭部 I/O で読むだけ。判定は無し（`safety_limits.md` §2 落下・段差 未実装） | 機体側（胴の XIAO）で判定するには、頭の XIAO → 胴の XIAO の経路が要る（現状は頭 → PC） | 質問 Q9 |
| 安全 | レーザー不使用、画像は家の外に出さない | 該当コード無し | 新規のルールとして明記 | — |
| 検証 | `docs/verification.md` | `docs/verification_status.md`（5 段） | 同じ物にするか分けるか | 質問 Q10 |

---

## 3. 質問リスト（答えが無いと進めないもの）

- **Q1 頭ヨー無しの 5 軸で「物を見る → 人を見る」をどう作るか。** J1 首 Pitch だけでは左右を向けない。胴体 Yaw で体ごと向くか、5 軸目を頭ヨーにするか（J2〜J5 のうち 1 つを頭に）。決定 13 の身体表現に直結する。
- **Q2 決定 4「PC は関節角を送らない」の適用範囲。** Home AI → Serpens の境界（Task だけ）でよいか。Serpens 内部の PC → 機体（歩容パラメータ・首角・停止）は既存契約のままか。
- **Q3 機体 ⇄ Serpens PC の物理経路。** 現在は USB CDC。MVP でも PC に USB で繋ぐか、Wi-Fi にするか。Wi-Fi なら Heartbeat / TTL の再実測が要り、単独 HOLD の意味が変わる。
- **Q4 EX-01 展示機の経路（外部カメラ・ArUco・人物検出）を残すか。** 残すなら「観測器」を profile で切り替える形にする。自己位置の開始条件 `require_real_pose` の「ArUco」を「AprilTag（機体カメラ）」に読み替えてよいか。
- **Q5 タッチセンサは無くなるか。** 撫で応答（PETTED の 3 段）は入力を失う。load による接触検出（HARDWARE_UNVERIFIED）で代替する前提か。
- **Q6 BNO085 の搭載位置と接続先**（胴の XIAO か頭の XIAO か）。首 Pitch で頭の姿勢は変わる。
- **Q7 サーボ電源。** STS3215 7.4V 版の型番（秋月 116312 = C001 / 1:345 と理解）と、2S 満充電 8.4V との整合（レギュレータか、レジスタ 14 の書き換えか）。ヒューズ値は決定どおりフェーズ 4 の実測後。
- **Q8 「輪を作る姿勢」の定義を機体側に持つ形。** 隣接 Yaw の合計角の上限（例 180°）を機体側で検査し、`rest_arc`（240°）は MVP では使わない、でよいか。
- **Q9 下向き ToF の判定は誰が行うか。** 安全ルール「ESP32 側で完結」に従うなら胴の XIAO が ToF を直接読む（頭の XIAO から UART で流す）配線が要る。頭 → PC → 胴 では完結しない。
- **Q10 `docs/verification.md`** を新設するか、既存 `docs/verification_status.md` に統合するか。
- **Q11 Task の座標系。** inspect_point の「位置」は家側 AprilTag 基準の絶対座標か、部屋ごとの座標か。単位（mm）と原点の定義。
- **Q12 通知の最小限**: ログ + Webhook の宛先は Home AI 側の URL か、Serpens 側で直接叩くか（決定 14 との整合）。
- **Q13 データセットの床**: フローリングの色・木目の種類、照明条件。手持ちカメラの機種（フェーズ 2 は XIAO Sense の実カメラではなく手持ちで始めてよいか）。
- **Q14 撮影の 3 枚（通常 / 斜め照明 / 線光）の照明配置**（LED の位置・角度・スリット幅）は決まっているか。線光の高さ推定はカメラと線光源の基線が要る。
- **Q15 私の作業範囲**: 実サーボ・書き込みは引き続きしない前提でよいか（フェーズ 4 の試験ツールは「ユーザーが実行、私が解析」）。

---

## 4. フェーズ 1 の作業計画（境界と契約。ハード不要）

**目的**: Home AI ↔ Serpens の Task / Event API を「スキーマ + テスト + モック受信」で固める。既存の PC ⇄ 機体契約（USB 駆動リンク）は触らない。

### 4.1 成果物

| 成果物 | 場所（案） | 内容 |
|---|---|---|
| API 仕様 | `docs/task_event_api.md` | MQTT トピック、版、QoS/retain、エラー時の扱い、状態遷移（Task の受理 → 実行 → 完了 / 失敗 / 中断） |
| JSON スキーマ | `schemas/task/*.json`, `schemas/event/*.json`（JSON Schema draft 2020-12） | Task 5 種、Event 4 種、共通ヘッダ |
| 検証コード | `serpens/api/schemas.py`（読み込みと検証。`jsonschema` は既存依存に無い → 標準ライブラリで最小検証を書くか、依存追加を確認） | 版の不一致・未知フィールド・範囲外を拒否 |
| MQTT 境界（送受信の薄い層） | `serpens/api/mqtt_bridge.py` | `paho-mqtt` は未依存 → **依存追加は確認**。無くても `LoopbackBroker` でテストできる形に |
| Home AI 側モック | `tests/mocks/home_ai_mock.py` | Task を発行し Event を受け取って検証するだけ（Home AI 本体は作らない） |
| テスト | `tests/test_task_event_api.py` | スキーマ合格 / 不合格例、版違い、往復、`stop` が最優先で処理されること |
| 記録 | `docs/verification_status.md`（Q10 次第で `verification.md`） | 「何をどう測って何が出たか」 |

### 4.2 API の骨子（案。フェーズ 1 で確定）

- トピック: `home/serpens/task`（Home AI → Serpens、QoS 1）、`home/serpens/event`（Serpens → Home AI、QoS 1、`safety_state` は retain）、`home/serpens/task_status`（Event の一種）。
- 共通ヘッダ: `v`（API 版）、`id`（UUID）、`t_ms`、`source`。
- Task: `inspect_point{pose_mm, priority}` / `patrol_route{waypoints}` / `highlight_point{pose_mm, duration_s}` / `return_dock` / `stop{reason}`。
  **stop 以外は「実行できない理由」を返せる**（安全ゲート未完了、自己位置なし、電池低下）。
- Event `floor_finding`: `id, t_ms, pose{x_mm,y_mm,theta_rad, cov_mm2 or sigma_mm, source}, photos{normal,raking,line}（切り抜きのパス。原画像は外へ出さない）, candidates[{class,conf}], size_mm{est,sigma}, risk{ingestion,sharp,child_reachable,score,rationale}, state（candidate / confirmed / resolved / dismissed）`。
- Event `safety_state`: 既存 7 状態と停止理由（`docs/link_protocol.md` の語彙）をそのまま外へ出す。**外から安全設定を変える経路は作らない**（Task に該当フィールドを置かない）。
- Event `battery`: `v, pct（推定）, source（HARDWARE / SIMULATION）`。
- エラー: 受理できない Task は `task_status{status: rejected, reason}`。スキーマ違反は `rejected(schema)`。同 id の再送は冪等。

### 4.3 手順（時間を区切る）

1. スキーマと文書（半日）→ 2. 検証コードとテスト（半日）→ 3. Loopback での往復とモック（半日）→ 4. 記録と自己レビュー（決定 14 の境界を越えていないか、安全設定が外から触れないか）。
既存 415 件のテストは緑のまま。安全に関わる変更は無い（境界の追加のみ）。

### 4.4 フェーズ 1 でやらないこと

関節角・歩容の MQTT 化、Home AI 本体、画像処理、AprilTag、IMU、実サーボ、ファーム書き込み、ROS 2。

---

## 5. 回答（2026-09-25。決定事項への追記）

| Q | 決まったこと | 残る確認 |
|---|---|---|
| Q1 | 関節の割り当ては変えない。**首 = J1（上下）+ J2（横）**として gaze / glance を抽象化。J3〜J5 は姿勢保持（または反対側に少し曲げて釣り合い）。J2 の範囲を超える向きは、先にその場旋回で体を向ける。将来の頭ヨーへ差し替え可能に | — |
| Q2 | 決定 4 は Home AI ↔ Serpens の境界だけ。PC → 機体の DRIVE/HEAD は既存のまま | — |
| Q3 | フェーズ 1〜4 は USB。フェーズ 5 で Wi-Fi。Heartbeat / TTL は通信方式ごとに設定を分け、Wi-Fi 化で実測し直す | — |
| Q4 | 外部カメラ経路は削除せず、検証用（位置の真値計測）として残す。開始条件の「ArUco」は「AprilTag（機体カメラ）」に読み替え | ArUco 依存の一覧は §6 |
| Q5 | タッチは残す（ESP32-S3 の静電容量タッチ端子）。load 検出は補助 | **タッチパッドの場所（背中など）** |
| Q6 | BNO085 は胴 2（胴の XIAO と同じ節）、I2C | — |
| Q7 | 秋月 116312 = 7.4V 版・1:345・19.5kgf·cm。型番末尾は実物ラベルで確認。電源は a) 12V 版 STS3215（秋月 g130969、4〜14V）に変更して 2S 直結（推奨）/ b) 7.4V 版 + 約 7.0V 降圧。決まるまで電圧は設定値 | **a か b か** |
| Q8 | 機体側で 1) 関節ごとの上限 2) **任意の連続する横関節区間の合計角の絶対値の上限**（汎用）。rest_arc は MVP で不使用 | — |
| Q9 | 下向き ToF は胴の XIAO に直結（I2C を首の関節に通す）。前方 ToF は頭の XIAO | — |
| Q10 | `docs/verification_status.md` に統合 | — |
| Q11 | 座標系 `home`: `tags.yaml`（各タグの ID と姿勢）で定義。原点 = ドックのタグ中心、x = 部屋の方向、y = 左、z = 上、m / rad、yaw は反時計回り。Task / Event に `frame_id` と `map_version` を必ず入れる | — |
| Q12 | 通知は Home AI 側。MVP は「受けてログに出す」モック | — |
| Q13 | フェーズ 2 は手持ちカメラ。治具で頭と同条件（床から約 30mm、下向き 20〜35°、LED 照明）。床は自宅フローリング + 後で 1 種類。照明は昼 / 夜の室内灯 / ソファ下の 3 条件 | **スマホの機種** |
| Q14 | 初期値（設定値、HARDWARE_UNVERIFIED）: カメラ床から約 30mm・下向き 25° 前後。斜め照明: 床から 5〜8mm の白色 LED を横から 5〜10° で。線光: LED + スリットをカメラから横に 30〜40mm（基線）、前方 150〜250mm に線。レーザー不使用 | — |
| Q15 | 実サーボ操作・書き込みはしない。フェーズ 4 は「ユーザーが実行、私が解析」。手順書と記録フォーマットを用意 | — |

## 6. ArUco に依存する既存コード（Q4 の報告）

| 場所 | 依存の内容 | MVP での扱い |
|---|---|---|
| `serpens/perception/aruco_locator.py` | 検出器（DICT_4X4_50）、2 枚のマーカから自己位置 | 検証用（外部カメラの真値計測）として残す |
| `serpens/perception/camera_observer.py`, `serpens/sim/sim_vision.py`, `serpens/sim/virtual_camera.py` | ArUco を描く / 検出する観測器 | 同上。MVP の動作経路からは外す（観測器の差し替えで） |
| `serpens/perception/homography.py`, `snake_pose.py` | 床ホモグラフィ、マーカ観測からの姿勢 | AprilTag（機体カメラ）版は別に作る（フェーズ 3） |
| `serpens/safety.py` `autonomy_blockers` | `pose_source != "aruco"` で実機の自律走行を拒否 | **「aprilTag」を実観測として認める読み替えが要る**（config `require_real_pose` の語彙を増やす） |
| `serpens/sim/session.py`, `serpens/runner.py`, `serpens/gui/app.py`, `simulation/bridge.py`, `simulation/vision_loop.py`, `serpens/world_state.py`（`PoseSource.ARUCO`） | 出どころの名前・GUI 表示・World State の語彙 | `APRILTAG` を追加し、`ARUCO` は `ARUCO_EXTERNAL`（検証用）と読み替える |
| `serpens/sim/world.py` | マーカの位置（背中の 2 枚）を世界へ置く | 検証用に残す |
| `config/robot.yaml` `markers` / `aruco` / `vision_sim` | マーカ寸法・検出パラメータ | 残す（検証用）。AprilTag の `tags.yaml` は別ファイル |
| `tools/make_aruco.py`, `perception_demo.py`, `perception_live.py`, `vision_check.py`, `loop_timing.py`, `check_env.py` | 印刷・デモ・評価 | 残す |
| `tests/test_perception.py`, `test_vision_bridge.py`, `test_camera_observer.py`, `test_closed_loop.py`, `test_phase1_wiring.py`, `test_safety_limits.py`, `test_electrical_gate.py` | 上記の試験 | 残す（検証用経路の回帰） |

**フェーズ 1 では触らない**（境界の追加だけ）。読み替えはフェーズ 3 で `PoseSource.APRILTAG` を足すときに一括で行う。
