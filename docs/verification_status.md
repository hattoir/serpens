# 検証レベル — 何が確かめられていて、何が確かめられていないか

作成 2026-09-15、2026-09-21 に 5 段へ。**このリポジトリで「確認済み」と書くときは、必ずこのどれかを指す**
（`serpens/verification.py`）。実機が 1 台も無い状態で開発しているので、**模擬の結果を実測として扱わない**ことが最優先。

| レベル | 意味 | 根拠になるもの |
|---|---|---|
| `SOFTWARE_VERIFIED` | **ソフトの論理として正しい**ことを自動試験で確かめた（物理は含まない） | `pytest`。式の一致・状態遷移・拒否条件・値の整合 |
| `KINEMATIC_SIM` | 簡易シミュレータで**そう動いた**（車輪の横滑りゼロ拘束・指令角そのまま）。旧表記 `SIMULATED` | `serpens/sim`、`tools/motion_quality.py`、`tools/body_compare.py` |
| `PHYSICS_SIM` | MuJoCo（接触・摩擦・サーボの一次遅れ）でそう動いた。摩擦もゲインも未同定 | `simulation/mujoco` |
| `HARDWARE_VERIFIED` | **実機で測った**。日付・条件・測定器つき | 実測記録 |
| `HUMAN_EVALUATED` | **人が見て評価した**。source は `VIDEO_HUMAN_EVALUATION`（動画）/ `PHYSICAL_HUMAN_EVALUATION`（実物） | `docs/human_pilot.md` の回答 |

設計値・推定値で実機が要るものは従来どおり `HARDWARE_UNVERIFIED` と書く。
現在 `HARDWARE_VERIFIED` は **0 件**、`HUMAN_EVALUATED` は **0 件**。
**Snake-likeness / Animacy / Approachability / Affection / Fear / Smoothness は Simulation だけでは合格判定しない。**
`tools/motion_quality.py` の指標は REGRESSION_METRIC / SIMULATION_DIAGNOSTIC であり、人の評価の代替ではない。

## 1. 駆動リンクと安全

| 項目 | レベル | 根拠 |
|---|---|---|
| フレーム（CRC・分割・混入・seq） | `SOFTWARE_VERIFIED` | `tests/test_phase2_frame.py`（8） |
| 7状態の状態機械 | `SOFTWARE_VERIFIED` | `tests/test_phase2_device.py` / `test_phase2_safety.py` |
| PC 強制終了・USB 抜去で機体が止まる | `SIMULATED` | 偽経路で再現。**実機の停止時間は未測定** |
| 通信断 → 保持まで 210〜300ms | `SIMULATED` | `tools/link_check.py`。実機ではシリアル往復ぶん増える |
| 緊急停止 → 出力停止 10ms | `SIMULATED` | 同上。**実機では 9軸同期書き込みの時間が乗る** |
| 上限外の角度・速度・歩容値の拒否 | `SOFTWARE_VERIFIED` | `tests/test_phase2_device.py`（11 種） |
| NaN / Inf / 桁あふれを送らない | `SOFTWARE_VERIFIED` | `tests/test_phase2_faults.py` |
| packet loss / delay / 順序入れ替え / 重複 / CRC 破損 | `SIMULATED` | `tests/test_phase2_faults.py`（17） |
| サーボ応答なし → FAULT_HOLD | `SIMULATED` | 模擬サーボを offline にして確認 |
| 過熱・fault ビットでラッチ | `SIMULATED` | 模擬サーボの値。**実機の温度挙動は未測定** |
| ファームと Python の値の一致 | `SOFTWARE_VERIFIED` | `tests/test_firmware_sync.py`（11） |
| ファームのビルド | `SOFTWARE_VERIFIED` | `tools/build_firmware.py`（arduino-cli + esp32 core 2.0.17, XIAO_ESP32S3）。フラッシュ 7% / RAM 5%。**書き込みはしていない** |

## 2. 機構・サーボ

| 項目 | レベル | 根拠 |
|---|---|---|
| 可動域 geometry ±90° | `HARDWARE_UNVERIFIED` | 設計値 |
| 干渉の始まり 64.8°（干渉なし）〜 64.9°（干渉） | `HARDWARE_UNVERIFIED` | CAD R03 の検討結果（verified_in_cad）。**clamp には使わない。** ケーブル込み・実物では未確認 |
| mechanical_design_limit ±55°（PROVISIONAL） | `HARDWARE_UNVERIFIED` | CAD R03 の提案値 |
| software_operational_limit ±50°（CONDITIONAL） | `HARDWARE_UNVERIFIED` | 実機試験の**候補値**であって実機運転の許可ではない。`config/robot.yaml` の `joint_limit_policy` |
| 最小曲げ半径 82.3mm / 輪の内径 114mm | `SOFTWARE_VERIFIED` | 幾何計算（`tests/test_safety_limits.py`）。**外皮を着せた実物では変わりうる** |
| とぐろが自己干渉しない（クリアランス 89.6mm） | `SOFTWARE_VERIFIED` | 中心線の距離計算。外皮の厚みは未考慮 |
| ソフトのトルク上限 0.450N·m（ストール参照比 0.167） | `HARDWARE_UNVERIFIED` | C044（7.4V/1:191）の**参照値**（2026-09-29 から現行資料の rated 9 / stall 27.4 kgf·cm。旧 5.2 / 16 は stale / source-unverified）から。`measured_safe_torque_nm` は null。H1 で実測する |
| ELECTRICAL_SAFETY_GATE（8項目） | **全項目 INCOMPLETE** | `docs/safety_limits.md` §5。閉じている間は実機の自律走行を拒否（`SOFTWARE_VERIFIED`: `tests/test_electrical_gate.py`） |
| 接触 → 脱力 20ms | **未達・要求未確定** | `docs/contact_release_requirements.md` |
| 挟み込み力 12.4N | `HARDWARE_UNVERIFIED` | 計算値。ISO/TS 15066 との比較も未（規格本文を参照できていない） |
| サーボの応答（一次遅れ・速度上限・温度上昇） | `SIMULATED` | `mock_servo` の係数はすべて仮値 |
| STS3215 のレジスタ仕様 | `HARDWARE_UNVERIFIED` | 一次資料（Waveshare/Feetech）で確認済みだが実機で試していない。`docs/sts3215_registers.md` |
| 最高入力電圧の初期値 8.0V（12V の罠） | `HARDWARE_UNVERIFIED` | メモリテーブル記載。**実機で最優先に確認する** |
| 負荷の符号ビット（bit10 仮説） | `HARDWARE_UNVERIFIED` | 公式資料に記載なし。仮説のまま |

## 3. 移動・歩容

| 項目 | レベル | 根拠 |
|---|---|---|
| 歩容の式と機体の出力が一致（差 < 1e-9°） | `SOFTWARE_VERIFIED` | `tests/test_phase2_device.py` |
| 1周期あたり 390mm 前進（6軸・振幅30°） | `SIMULATED` | `tangential_drag_ratio = 0.02` は**推定値**。実測で変わる |
| 旋回半径 388mm（γ0=20°） | `SIMULATED` | 同上 |
| 胴体ヨーの本数と移動性能の関係 | `SIMULATED` | `docs/product_status.md` §3。相対比較にのみ使う |
| マット端に寄らない（0.00%） | `SIMULATED` | 仮想フェンス。実機の滑りは未検証 |
| 摩擦（Wheel Belly / Snake Belly） | `HARDWARE_UNVERIFIED` | **Snake Belly の摩擦係数は未実測。** 仮値を現実として扱わない |
| 騒音 | **未測定** | 構想設計書が「最大のリスク」とする項目。実機が来たら最初に測る |

## 4. 知覚・行動

| 項目 | レベル | 根拠 |
|---|---|---|
| ArUco 検出（1080p で 50mm マーカ） | `SOFTWARE_VERIFIED` | 実写画像で検出率を測定済み（印刷物は実物） |
| 床ホモグラフィ・視差補正 | `SIMULATED` | 仮想カメラでの検証。実カメラでの誤差は未測定 |
| YOLO 人物検出（オフライン） | `SOFTWARE_VERIFIED` | `tests/test_offline.py`。ネット遮断で最後まで動く |
| 行動の状態遷移・効用調停 | `SOFTWARE_VERIFIED` | `tests/test_behavior_*.py` |
| 「生き物らしさ」 | **未評価** | 人が見ての評価。実機でしか判断できない |

## 4.5 仮想実機（Phase 2/3）

| 項目 | レベル | 根拠 |
|---|---|---|
| `SimulatedSnake`（PC → 仮想ESP32 → 仮想サーボ → シミュレータ） | `SIMULATED` | `tests/test_phase3_link_robot.py`（12） |
| `RealSnake`（PC → 実 ESP32 → STS3215） | **`HARDWARE_UNVERIFIED`** | 一度も実ポートで通信していない |
| 7状態の遷移（FAULT_HOLD からの復帰は DISARMED まで） | `SOFTWARE_VERIFIED` | `tests/test_phase2_safety.py` |
| テレメトリ v2（velocity / age / ttl / overrun / source） | `SOFTWARE_VERIFIED` | `tests/test_phase2_frame.py` |
| 仮想サーボの応答（一次遅れ・速度上限・温度・電圧降下） | `SIMULATED` | `mock_servo` の係数は仮値 |
| GUI の関節ペイン（指令角 / 実測角 / 機体状態） | `SOFTWARE_VERIFIED` | オフスクリーン描画テスト + 目視（output/gui_joints.png） |
| Belly（wheel / snake）と摩擦プロファイル | `SIMULATED` | `tests/test_belly_and_gait_config.py`。**係数は未実測** |
| 歩容の掃引（速さ・蛇らしさ・滑らかさ・負荷） | `SIMULATED` | `tools/gait_sweep.py` |

## 4.6 3D 物理・閉ループ（Stage F〜N）

| 項目 | レベル | 根拠 |
|---|---|---|
| MuJoCo モデル（9軸・質量・可動域・トルク上限が config と一致） | `SOFTWARE_VERIFIED` | `tests/test_mujoco_model.py`（10） |
| MuJoCo での推進・旋回・転倒しないこと | `SIMULATED` | `tests/test_mujoco_belly.py`（6）。**摩擦も質量も未実測** |
| 等方摩擦では進まない / 異方性で進む | `SIMULATED` | 簡易シミュレータと **別モデルで独立に一致** |
| サーボゲイン（未同定）で前進量が変わる（kp ×0.5 / ×2 で約 1.16 倍） | `PHYSICS_SIM` | 実機が来たら最初に同定する（下記）。2026-09-29 まで MuJoCo の pitch 軸が逆向きで（+ で頭が床へ潜る）、±45% はその誤りを含む値だった（`ai-outbox/lessons/2026-09-29_LES-ENG-0001_mujoco_pitch_sign.md`） |
| Floor Watch 腹面: 推進は横/前後 摩擦比で決まり、しきい値は約 1.5〜2.5 | `PHYSICS_SIM` | `simulation/results/fw_body_study.md`（E2）。摩擦の実測（H0 クーポン）で確かめる |
| Floor Watch: 平床の蛇行でトルク上限 0.45 N·m は律速でない | `PHYSICS_SIM` | 同上（E3）。摩擦・質量・kp は未実測 |
| World State の出どころ分離（真値 ≠ Vision） | `SOFTWARE_VERIFIED` | `tests/test_closed_loop.py` |
| 閉ループ（行動 → 機体 → 世界）と故障注入で安全が勝つこと | `SIMULATED` | `tests/test_closed_loop.py`（13） |
| run の記録から同じ run を作れること | `SOFTWARE_VERIFIED` | 同上（seed / config hash / model digest） |

## 4.7 Vision Bridge（Phase 4、`docs/phase4_vision_bridge.md`）

| 項目 | レベル | 根拠 |
|---|---|---|
| 仮想カメラ画像 → ArUco → 自己位置 → 行動 → 仮想機体 の閉ループ | `SIMULATED` | `tests/test_vision_bridge.py`（9）。位置誤差 p95 約 31mm（**模擬画像**） |
| 模擬 Vision を実機の開始条件として通さない（`aruco_sim`） | `SOFTWARE_VERIFIED` | 同上 |
| 自己位置が 0.5s 古くなったら保持し、自動で再開しない | `SIMULATED` | 暗転 3 秒で確認。停止までの惰性 約 31mm |
| 同じ ID の重複を使わない / 位置の飛びを捨てる | `SOFTWARE_VERIFIED` | 同上 |
| 近い偽マーカの区別 | **未対策** | 方式の限界。運用規則で避ける |
| 実カメラでの誤差・遅れ | **未測定** | `vision_sim.latency_s` / `frame_hz` は想定値 |
| 実画像（録画ファイル）→ デコード → ArUco → 自己位置 → セッション | `SIMULATED` | `tests/test_camera_observer.py`（5）。仮想カメラの画像を MJPG 動画にして読み直した。p95 誤差 < 25mm。**実カメラの画像は未** |
| 録画（`aruco_file`）は実機の開始条件を満たさない / 生きたカメラ（`aruco`）だけが満たす | `SOFTWARE_VERIFIED` | 同上 |
| 見失った後（と最初）の自己位置は人が K で確認するまで走行に使わない | `SOFTWARE_VERIFIED` | `tests/test_vision_bridge.py` / `test_camera_observer.py` |
| 50Hz 制御周期が知覚スレッド（ArUco + YOLO 59ms/枚）と同時でも遅れない | `SOFTWARE_VERIFIED`（このPC） | `tools/loop_timing.py`: 平均 19.96ms / 最悪 20.9ms / 遅れ 0/502（2026-09-18） |
| 電子系（遮断器・E-STOP・電流検出）の設計 | **設計案のみ** | `docs/electronics.md`。C044 の電流値は **UNKNOWN**（12V 品の 2.7A は流用しない） |

## 4.8 動きの質（蛇らしさ・愛着。`docs/motion_quality.md`）

| 項目 | レベル | 根拠 |
|---|---|---|
| 静止率 0.44 / 一次反応 0.22s / GAR 0.51 / 可視波数 1.3 / LDJ −25.7 | `SIMULATED` | `tools/motion_quality.py`、`tests/test_motion_quality.py`（7）。KINEMATIC_SIM |
| Bug-1/2/3（Stress 飽和・Energy・ノイズ）の回帰 | `SOFTWARE_VERIFIED` | `tests/test_behavior_units.py` / `test_behavior_sim.py` |
| 人を追跡中に J7 > look_max_deg を指令しない（威嚇の封印） | `SOFTWARE_VERIFIED` | `tests/test_behavior_sim.py` |
| 頭の先端加速度 ≤ 1G・立ち上がり ≥ 200ms | `SOFTWARE_VERIFIED` | `tests/test_animator.py`（指令角。サーボ応答は含まない） |
| 8 軸案で体に 2 波（Ω=90°）、前進は 1 波の約 4 割 | `SIMULATED` | `config/robot_yaw8.yaml`。実機は存在しない |
| カメラ経路込みの一次反応（約 0.52s） | **未測定・窓の外** | detect 5Hz + 遅れ 0.1s は想定値 |
| 「生き物らしさ」「愛着」 | **未評価** | 人が見ての評価。実機でしか判断できない |

## 4.9 Human Perception Pilot の準備（2026-09-21）

| 項目 | レベル | 根拠 |
|---|---|---|
| 6 / 8 / 10 Yaw の overlay（同等リンク長 / 同等身体長）と波数掃引・速度合わせ | `KINEMATIC_SIM` | `tools/body_compare.py` → `output/body_compare.md` |
| 匿名クリップ生成・提示順のランダム化・回答テンプレート・集計 | `SOFTWARE_VERIFIED` | `tests/test_pilot_tools.py`（8） |
| EXHIBITION profile の体験弧（NOTICE→…→REST が 60 秒で起きる） | `KINEMATIC_SIM` | `tests/test_exhibition_profile.py`（3 seeds） |
| 内部状態 6 種の回復可能性 | `SOFTWARE_VERIFIED` | `tests/test_internal_state_recovery.py`（24） |
| 一次反応 0.22s（BEHAVIOR_INTERNAL）/ 約 0.52s（END_TO_END_SIMULATED） | `KINEMATIC_SIM` / 推定 | HARDWARE_MEASURED は未 |
| 人が見た蛇らしさ・愛着・Fear | **HUMAN_EVALUATED = 0** | ユーザーが Pilot を実施するまで変わらない |

**正式な Risk**: 人物検出 5Hz（`person.detect_hz`）が展示の反応速度のボトルネックになりうる。
実機カメラ到着後に camera capture → detector → world state → behavior → command → visible response の
End-to-End latency を測る。

## 4.10 Floor Watch フェーズ 1 — 境界と契約（2026-09-25）

| 項目 | レベル | 根拠 |
|---|---|---|
| Task / Event API（schemas/*.json、5 Task・4 Event、frame `home`、floor_finding の不確かさ・危険度の内訳） | `SOFTWARE_VERIFIED` | `tests/test_task_event_api.py`（10）。スキーマ合格/不合格、版違い、map_version 違い、未知フィールド、安全設定の不在 |
| Loopback での往復（Task → 受理/拒否/冪等/stop 最優先 → Event）、Home AI 側モックが受け取る | `SOFTWARE_VERIFIED` | 同上。retain した safety_state が後からの購読に届く |
| フェーズ 1 追加確認 8〜11（operator_resume は MQTT から到達しない / safety_state の 2 s 周期送信と受け側時計での失効 / 正常終了の OFFLINE と再接続の上書き / stop と緊急停止の将来の分離を文書化） | `SOFTWARE_VERIFIED`（Loopback） | `tests/test_task_event_api_review.py`（3）+ `tests/test_task_event_api.py`（16） |
| 本物の MQTT ブローカーとの疎通 | 下の 4.11 を見る | |
| Home AI 本体 | 作らない（決定 14） | モックのみ |

## 4.11 Floor Watch フェーズ 2 — 画像処理（2026-09-25、`docs/floorwatch_phase2.md`）

| 項目 | レベル | 根拠 |
|---|---|---|
| 幾何（1.5mm → 21.3px、0.3mm → 4.3px、視野中心 71mm）と光の面の較正（名目からずれた面でも高さが戻る） | `SOFTWARE_VERIFIED` | `tests/test_floorwatch.py` |
| 合成画像（**基準床なし**、あごの斜め照明、継ぎ目を含む n = 70）: 設計値モード（f=1000）で patrol / inspect とも 98〜100%（95%CI 88〜100%。雑音で実行ごとに揺れる）、実機モード（UXGA f ≈ 1256 ASSUMED）で patrol 98% / inspect 100%・誤報 0%（0/28、95%CI 上限 12%）、線上の高さ ±1mm、鏡面 → height null + specular_break → metal_disc | **SIMULATED（合成画像。影も線も理想的なので楽観値）** | `tools/floorwatch_eval.py --synthetic` → `output/floorwatch_eval.md`、`tests/test_floorwatch.py`（9） |
| レビュー 2026-09-26 の 1〜7（高さ null + 理由 / 測れない = 出っ張り扱い / metal_disc 別枠 / 基準床なし / 撮影順と動き検出 / あご照明・影は奥 / 2 段評価と信頼区間 / 継ぎ目の合成） | `SOFTWARE_VERIFIED`（合成） | 同上 |
| 実写（手持ちカメラ、治具）での検出率・誤報率 | **未測定** | フェーズ 2 の完成条件。カメラ機種の回答待ち |
| 候補の種類の分類 | **未実装** | 実写データが揃ってから（合成で作ると過適合） |
| フェーズ 1 レビュー 1〜5（stop 最優先・待ち行列破棄・stop 前の Task 拒否・retain 拒否・LWT OFFLINE・危険物別枠・resolved は人だけ） | `SOFTWARE_VERIFIED` | `tests/test_task_event_api.py`（16） |
| 本物の MQTT（retain / LWT が keepalive の 1.5 倍で出る / 正常終了の OFFLINE / 再接続の上書き / QoS1 再送） | `SOFTWARE_VERIFIED`（本物の Mosquitto 2.1.2、2026-09-26。4 件: 黙ったクライアントの LWT は keepalive 2 s に対し 2〜5 s の窓で到着、ソケット断は即時、正常終了の OFFLINE は即時、再接続後の購読者は OFFLINE を見ない、QoS1 再送） | `tests/test_mqtt_live.py`（Mosquitto を 127.0.0.1 の一時ポートでサブプロセス起動。無ければ skip） |

## 4.12 Floor Watch フェーズ 3 — 自己位置（2026-09-26、`docs/floorwatch_phase3.md`）

| 項目 | レベル | 根拠 |
|---|---|---|
| 地図 `config/tags.yaml`（座標系 home、map_version）、推定器（予測・タグ補正・ゲート・IMU の磁北差）、開始条件が σ と IMU / オドメトリの健全性で決まる | `SOFTWARE_VERIFIED` | `tests/test_localization.py`（7） |
| 周回の閉ループで σ が正直（\|誤差\| ≤ 3σ が 98〜100%、滑り 5〜10%）、タグを見ずに走れる距離 ≈ 0.7〜1.1 m | **KINEMATIC_SIM 相当（SIMULATED）** | `tools/localization_sim.py` → `output/localization_sim.md` |
| AprilTag 検出 → 距離 3% / 方位 0.02 rad / 面の向き 0.06 rad | **合成画像** | `tests/test_localization.py` |
| 実カメラでの検出、実 IMU、実歩容の滑り、部屋のタグ配置 | **未測定** | フェーズ 4 の試験ツールで |

## 4.13 Floor Watch フェーズ 5 — 縦一本（2026-09-26、`docs/floorwatch_phase5.md`）

| 項目 | レベル | 根拠 |
|---|---|---|
| Home AI モック → inspect_point → 移動（KINEMATIC_SIM）→ 位置合わせ（前後の微調整 + 頭ヨーで線を地点へ）→ 撮影（合成）→ 判定 → floor_finding（推定姿勢 + σ、切り抜きだけ保存）→ モックが通知 | **SIMULATED（模擬の縦一本）** | `tests/test_floorwatch_slice.py`（3） |
| 開始条件（機体の停止・位置の確認待ち・σ・IMU・オドメトリ）で rejected、stop で aborted + HOLD、再開は operator_resume と機体側の開始操作の両方（人） | `SOFTWARE_VERIFIED` | 同上 |
| 実機での移動・撮影・照明制御・実タグ・実 IMU | **未実施** | フェーズ 4 の試験ツールと部品到着後 |

## 4.14 頭のスコップ＋フタ（頭内取り込み。第 2 段階の成立性調査、2026-09-29、`simulation/scoop/`）

**すべて `PHYSICS_SIM`（MuJoCo）。実物の試験片の結果が出るまで「候補」。** 詳しい結果は `simulation/results/scoop_exploration_2026-09-29.md`。

| 項目 | レベル | 根拠・限界 |
|---|---|---|
| モデルが解析と一致する（傾斜板の保持 tan α ≤ μ） | `SOFTWARE_VERIFIED` | `tests/test_scoop_model.py`。α = 8 / 12° は保持、18° は μ = 0.3 で滑り μ = 0.5 で保持 |
| 指示書の範囲（先端厚 0.4〜1.0、縁は直角、一定速度）で閉じ込めが成功する | **成功 0（1,728 回）** `PHYSICS_SIM` | 物は頭と同じ速さで運ばれる。摩擦・速度・先端の浮きを変えても不変 = 構造的。掃引は未実施 |
| 縁の丸み 0.1〜0.5 mm・前面を床に垂直・先端 0.02〜0.4 mm で成功する | 成功 0（1,040 回）`PHYSICS_SIM` | 先端 ≤ 0.1 mm・丸み ≥ 0.3 mm・低摩擦ではコインが傾斜板に乗る（on_ramp）が、空間に入らない |
| 実物の試験片での閉じ込め | **未実施** | 3D プリントの試験片（棒の先に付けて手で押す）を作ってから |

**追記 2026-09-30: 巻き込みくちばし（beak sweeper）と短ランプ**（`simulation/results/scoop_beak_2026-09-30.md`、`tools/scoop_beak_*.py`）。すべて `PHYSICS_SIM`。

| 項目 | レベル | 根拠・限界 |
|---|---|---|
| 奥ヒンジのフタ（ベースライン）: ランプ 5〜10mm・先端厚 0.6・μ 0.3 で乗る・入る・保持 | **0%（各 2,304 回）** `PHYSICS_SIM` | 物は乗らず押して逃げる。フタは幾何的に物を奥へ押せない（前縁は前・下へ動き、天井より下へ降りない） |
| くちばし（H 10 / 12 / 15、ランプ 5 / 8 / 10、回転 0.3 / 0.6 s、前進 2 / 10 mm/s、停止 / 前進、幅 30・開 / 幅 40・壁、144 設計 × 12 回）で 1 円玉・立方体の保持 | **0 / 1,728 ずつ** `PHYSICS_SIM` | 腕が物の後ろに回り込めない（腕先が床に近づくのは x = 0 の一点だけ）。トレースで確認。時間刻み・接触のやわらかさを変えても 0（各 0 / 60） |
| 同 立方体 10mm の閉じ動作 | 合図が出た回の 8〜10 割で腕が 180° に届かない（ジャム） `PHYSICS_SIM` | 腕のトルク上限（0.05 N·m）では、物の上に乗って止まる。H 10 は開いた腕が物に当たり合図が出ない |
| 同 CR2032 の保持 | 7 / 1,728（0.4%）。上位 3 設計（N = 30）と感度 19 設計では 0 | 腕が物を床との間に挟んではじく |
| 同 ビーズ φ8 の保持 | 上位設計 70%（N = 30、Wilson 90% 55–82%）、飛ばされた分を除くと 50%（36–64%） | **掃き込みではなく叩き込み**（関節角 約 58° で当たり 650〜830 mm/s）。入口が開いたままで、跳ね返って出るものもある |
| 「飛ばされた」率・挟まれてはじかれた分を含む乗る率 | **収束していない（定量に使えない）** | CR2032 の飛ばされた率: solref 0.001 = 12/12、0.004 = 4/12、dt 5e-5 = 5/12 |
| 実物の試験片（短ランプ 8 / 15 + 手で回すピン留めの腕） | **未実施** | `simulation/scoop/hardware_test_plan.md`、`scoop_test_template.csv` |

**指標（User 定義 2026-09-30）**: 乗る = 物の縁が床から 1mm 以上上がった瞬間があった（**頭側の縁**。−x 側の縁は既知ケースで偽になったため）、入る = 物の中心が空間の範囲に入った、
保持 = 閉じ終わりから 2 秒後まで空間の中、逃げた距離 = 判定時の空間の範囲からの距離。閉じ終わりは機械ラッチ（可動域を閉じ位置に絞ってモデル化）。
**追加の ASSUMED**: ヒンジ高さ 10 / 12 / 15mm、腕長 = H − 0.5mm、腕の幅 = 空間の幅、SG90 相当のトルク上限 0.05 N·m（停動 約 0.18 N·m は未実測。感度で 0.02 / 0.18）、
閉じ始めの合図 = 物がスコップに触れた瞬間（`contact:0`）、「飛ばされた」の閾値 350 mm/s。
**依頼の記述との食い違い**: 閉じ終わり（腕が奥向き水平、床上 H）は入口を塞がない。塞ぐのは腕が真下の姿勢（90°）。この変種は未実施。

**追記 2026-09-30（2）: 形と機構の探索（`simulation/scoop/forms/`、`simulation/results/scoop_forms_2026-09-30.md`）**。すべて `PHYSICS_SIM`。

| 項目 | レベル | 根拠・限界 |
|---|---|---|
| 受け身のフード（機構なし。開放底 + ゲート）の保持 | 直線の口 83%（100/120）、漏斗つき 95%（114/120）`PHYSICS_SIM` | 物 4 種 × 床 2 × 速さ 2 × ずれ 3、N = 30 / 対象物。失敗は位置ずれ 10 mm の 1 円玉・CR2032 のみ。**絨毯は未対応。壁の下端 0.1 mm は毛に埋まる可能性が高い** |
| 前の押す方式（0%）との違い | 口の床の段差を 0 にしたこと `PHYSICS_SIM` | 段差 0.1〜1.5 mm を足すと全対象物 0%（N = 30） |
| ゲートの効果 | 頭が後退したときにだけ `PHYSICS_SIM` | ゲートなし + 後退 30 mm = 0%、ゲートあり = 83%。後退なしなら差なし |
| B ベルト（ローラー列で近似）/ D カップ / A・C・E | B は壁ありで頭 10 mm/s なら 98〜100%（2 mm/s では 70% 前後）、壁・脚なしで 0/864。D は内径 40 mm で 100%（位置合わせ ±10 mm が前提）。A・C・E は受け身のフードより悪い | ベルトの面の連続性・たるみ、位置合わせの動きは含まない |
| ゲートが指・しっぽを挟まないか | **未確認**（SAFETY_UNVERIFIED） | 力 5 N（ASSUMED）は暫定しきい値 5.7 N に近く、降りる途中で 5〜12 mm の禁止帯を通る。挟み込みの力の実測が要る |
| 実物の試験片 | **未実施** | |

**追記 2026-09-30（3）: 許容差・垂れ布・スカート・ゲート力**（`simulation/results/scoop_forms_2026-09-30.md` §12、`scoop_forms_tolerance_mc.md`）。すべて `PHYSICS_SIM`。

| 項目 | レベル | 根拠・限界 |
|---|---|---|
| 口の前縁の段差の許容差 | **0.002 mm でも 0%** `PHYSICS_SIM`（縁の丸み 0.3 mm も同じ）。確率版（t ~ U(0, 0.3) mm）で期待値 0〜0.7% | N = 30。剛体・平らな床。実物の表面粗さ・弾性は未確認 → 試験片 B |
| すき間 `clearance_mm` 0 / 0.3 / 1 mm | 段差 0 では保持に効かない `PHYSICS_SIM` | 物はすき間より厚い。絨毯の毛の埋まりは別（未対応） |
| 受け身の TPU 垂れ布（駆動なし、全高、0.1〜1 N） | **成立しない（0%）** `PHYSICS_SIM` | 開く力 > 物の床の摩擦。全高は物に垂れかかり、後退で物が出る。8 mm の短い垂れ布 + 3 mN なら 1 円玉・CR2032 は成立（ASSUMED の力・質量） |
| 駆動のゲートの力の上限 | **2.8 N 以下（0.25 N まで）で成立** `PHYSICS_SIM`（95%） | 閉じるのは物が奥に入ったあと。指・しっぽの挟み込みの安全は未確認（SAFETY_UNVERIFIED） |
| スカート 3 / 6 mm + 床の凹凸 ±0.5 mm（絨毯の代用） | 効果なし `PHYSICS_SIM` | 毛の柔らかさ・厚みは入っていない。絨毯は未対応 |
| カップの位置合わせ ±10 mm のカメラからの逆算 | 角度誤差が支配（300 mm で ±1.9°）`CONCEPT` | FOV 65° は ASSUMED（実測待ち。OQ-0002） |
| 実物（試験片 B） | **未実施** | `simulation/scoop/hardware_test_plan.md`（User が印刷・観察） |

**追記 2026-09-30（4）: 段差 0.002 mm の 0% は理想化か（面取り・粗さ・接触設定）**（`simulation/results/scoop_forms_2026-09-30.md` §12.5〜12.9、`scoop_forms_tolerance_mc.md`）。すべて `PHYSICS_SIM`。

| 項目 | レベル | 根拠・限界 |
|---|---|---|
| 縁の面取り（5〜45°）・丸め（R 0〜0.3 mm）・段差 0.002〜0.5 mm | 保持 **0**、「入る」だけ戻る（ほぼ立方体）`PHYSICS_SIM` | 面取りで乗った物は頭と一緒に入口で運ばれる。R は効かない |
| 床の粗さ ±0.01 / 0.03 / 0.1 mm × 垂直の段差 0.002〜0.1 mm | 保持 **0**（32 回ずつ）`PHYSICS_SIM` | 粗さは 1 mm 格子のランダムな高さ場 1 つ（seed 固定）。実物の粗さの実測ではない |
| 接触設定（solref 0.5〜5 倍、時間刻み 1/2・1/4、影響幅の縮小、margin） | 段差 0.002 mm の保持は 0 のまま `PHYSICS_SIM` | margin > 段差では「入る」だけ出る（検出の意味の変化）。段差なしは 32/32 |
| 面取り 10 / 20° + 粗さ ±0.1 mm + 段差 0.05 mm | 保持 44〜59%（14〜19/32）`PHYSICS_SIM`。**この 1 組だけ** | 段差 0.1 mm 以上・粗さ ±0.03 mm では 0。ビーズは 0/8。仮説（粗さが板の高さを打ち消す）は未確認 |
| 許容差の期待値（t ~ U(0, 0.3) mm、ASSUMED） | A 直角・平ら 0〜0.7% / B 面取り 0〜0.7% / C 面取り + 粗さ ±0.03 mm 0〜16.7%、±0.1 mm 約 10〜27% | 格子の間は下限 / 上限で置いた範囲。C の下限はこの 1 組に依存。**結論: シミュレーションでは段差ほぼゼロが必須** |

**追記 2026-09-30（5）: 薄いフィルムの縁・フード昇降（cloche）**（`simulation/results/scoop_forms_2026-09-30.md` §13）。すべて `PHYSICS_SIM`。実物（B2）は**印刷待ち**。

| 項目 | レベル | 根拠・限界 |
|---|---|---|
| 薄いフィルム 0.01〜0.05 mm（垂直、面取りなし、粗さ 0〜±0.1、c 0.1 / 0.3） | 保持 **0/96 ずつ** `PHYSICS_SIM` | 許容できる厚みが無い（境界は 0.002 mm より下）。実物の t = 0.05 と同条件も 0% と予測 |
| フード昇降（位置ずれ 0・5 mm） | **100%** `PHYSICS_SIM`（上げる高さ 3〜10 mm、c 0〜1、速さ 5 / 20 mm/s、力 0.15〜5 N のすべて） | 段差を避ける。**位置合わせ ±5 mm（1 円玉・CR2032）**。10 mm で 1 円玉 48/144、CR2032 12/144。ビーズ・立方体は 10 mm でも 100% |
| 覆う / 運ぶ | その場で覆う 32/32、運ぶ（ゲートあり、30 mm 後退・前進）32/32 `PHYSICS_SIM`。ゲートなしの後退は 0/32 | 運ぶ力の大きさ・摩擦は未評価。カップの「運ぶ」は未実施 |
| 床の粗さ ±0.1 / 凹凸 ±0.5（絨毯の代用） | 位置ずれ 0 は 100%、5 mm は 1 円玉・CR2032 が境界（13〜16/16） | 毛の柔らかさ・厚みは入っていない。絨毯は未対応 |
| 落とす力・速さ・縁の圧力 | **枠組みのみ**（PROVISIONAL / SAFETY_UNVERIFIED）| `flank_v.md` §F。自重降下 0.04〜0.53 N、サーボが降ろす形は不可、上げる側は力の制限が要る、すき間 4 mm 以下 |

**仮定（ASSUMED。`config/scoop.yaml` に出典コメント）**: 頭 80 g、押しつけ力の上限 1.0 N、スコップ – 物 μ 0.3、フタ – 物 μ 0.3、フタのトルク上限 0.05 N·m、
立方体 – 床 μ 0.8、ビーズの転がり抵抗（c_rr ≈ 0.005）、板厚 = 先端厚、傾斜板の奥行き = 斜面に沿って 30 mm、先端の浮き 0.1 mm、物の初期距離 30〜60 mm。

**再現できていないもの**:
- **絨毯**（毛への沈み込み・毛が物を止める / 持ち上げる作用）。**未対応**。剛体平面の結果は、絨毯では**悲観側**の可能性が高い
- **縁の丸み・表面の粗さ・板のたわみ・床の凹凸**（実物のスコップが物の下に入る作用の多くはここにある。剛体の完全な平面では起きない）
- 頭のピッチ・ヨー、ヘビの蛇行による頭の動きの揺れ、摩擦の速度依存
- **安全に関わる論点（閉じ込めた物が外に出ない、小部品が外れない、ボタン電池を溜める危険）は、シミュレーションの対象外。機構設計と ST 基準で扱う**（先送り）

**数値（接触）の扱い**: 薄い先端（0.02〜1.0 mm）の接触は、timestep 0.2 ms・solref 時定数 2 ms・elliptic cone・multi-CCD で解いた。
先端の浮き・スコップの摩擦・床の摩擦・速度を変えても「押して逃げる距離」が 114.7 mm で不変なので、この結果は接触パラメータの数値の選び方には**依存しない**（構造的）。
成功が現れる領域（先端 ≤ 0.1 mm の on_ramp）の収束確認（timestep 0.5 倍 / 0.25 倍、solref・solimp・margin）は**まだ**。

## 5. 書くときの約束

- 表・コメント・コミットメッセージで「確認済み」とだけ書かない。**上の4段のどれかを書く。**
- テレメトリは `source`（SIMULATION / HARDWARE）と `SIMULATED` フラグを持つ。
  **画面にも記録にもそのまま持ち回す。** 模擬の数値が実測として独り歩きしないようにするため。
- 実機で測ったら、この表と `docs/phase2_acceptance.md` §3 を**同時に**更新する。
  測定日・条件・測定器を書く。書けないものは `HARDWARE_VERIFIED` にしない。
