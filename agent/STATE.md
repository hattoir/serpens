# STATE — serpens

<!-- 作業のたびに更新する。ここが次のセッションの出発点になる。 -->

_last updated: 2026-09-14_

## Current Goal（2026-09-25 Floor Watch フェーズ 2 更新）

フェーズ 1 のレビュー対応（stop 最優先・待ち行列・retain/LWT・危険物別枠・resolved は人だけ）と、
フェーズ 2 の画像処理を**合成画像で**通した（`serpens/floorwatch/`: 幾何・光の面の較正・4 枚 → 判定・危険度・データセット）。
合成での検出率 90% / 誤報率 0%。**実写は未**（カメラ機種の回答待ち）。Mosquitto 疎通は道具だけ用意（未実施）。
次: 実写データセット（治具・1 円玉の較正）→ 閾値調整 → フェーズ 3（AprilTag + IMU + オドメトリの自己位置）。

## Current Goal（2026-09-25 Floor Watch 更新・旧）

**Floor Watch 縦 1 本**（発見 → 確認 → 位置 → 通知）へ向かう。前提と回答は `docs/floorwatch_phase0_review.md`。
フェーズ 1（境界と契約）を実装: `schemas/*.json`、`serpens/api/`（検証器・Loopback/Paho ブローカー・端点）、
`tests/mocks/home_ai_mock.py`、`docs/task_event_api.md`。**安全の設定は Task に無い。stop は常に最優先。**
次: フェーズ 2（3 枚撮影の画像処理とデータセット。手持ちカメラ、治具で頭と同条件）。
残る確認: タッチパッドの場所、電源 a/b、スマホの機種。ArUco 依存の一覧は review §6。

## Current Goal（2026-09-21 Phase 2 更新・旧）

**Serpens の身体と動きを、初めて実際の人間へ見せて評価できる状態にした。** 6 / 8 / 10 Yaw の overlay（同等リンク長 /
同等身体長）、波数掃引と速度合わせ（`tools/body_compare.py`）、匿名クリップ（`tools/pilot_clips.py`）、提示順の
ランダム化、回答テンプレート・Godspeed 分離・集計（`docs/human_pilot.md`）、EXHIBITION profile の体験弧、
内部状態 6 種の仕様（`docs/internal_state_model.md`）と回復可能性テスト、Familiarity / Sleepiness、Energy と温度の分離。
**HARDWARE_VERIFIED = 0、HUMAN_EVALUATED = 0。** 次はユーザーが Pilot（n=8〜12）を実施し、回答を `pilot/` の形式で保存する。

## Current Goal（2026-09-21 更新・旧）

外部レビュー（蛇らしさ・愛着）の 13 ステップを実装した（`docs/motion_quality.md`）。致命バグ 3 件（Stress 飽和・
Energy・ノイズ）、stop-and-go、一次反応 150〜250ms、舌のちらつき、呼吸の位相波、視線、威嚇姿勢の封印、
anticipate/settle、忍び寄り歩容、撫での 3 段、primitives/grammar/locomotion の分離、評価指標 5 種、8 軸案 config。
**動きの人格は `config/robot.yaml` の `behavior.grammar` だけで変えられる。** 全 374 件緑。

## Current Goal（2026-09-18 更新・旧）

実機なしで進められるものを進めた: **実画像経路 `CameraObserver`**（カメラ / 録画 → 自己位置・人）、
**見失い後の位置確認（K）**、**制御周期の実測**（知覚スレッド同時で遅れ 0）、**電子系の設計案**
（`docs/electronics.md`。C044 の電流は UNKNOWN のまま）。
次に実機が来たら: C044 1 個の電流実測（ゲートの real_current）→ 実カメラで `CameraObserver` の誤差・遅れ。
それまでのソフト作業: 実カメラの校正手順の見直し、firmware のサーボ層（配線が決まってから）。

## Current Goal（2026-09-16 Phase 4 更新・旧）

**Phase 4 Vision Bridge（模擬）が閉じた。** 仮想カメラ画像 → 本物の ArUco 検出 → 自己位置 → 行動 →
仮想 ESP32 → 世界。故障注入（欠落・暗転・遮蔽・偽マーカ・偽の人）で、見失ったら保持・自動再開しないことを確認。
`ELECTRICAL_SAFETY_GATE`（8項目すべて INCOMPLETE）が実機の自律走行を止めている。
次: 実カメラ用の観測器（RealCamera → session の自己位置）、再開前の位置確認の手順、電子系 BOM。

## Current Goal（2026-09-16 更新・旧）

Phase 2/3 の仮想実機は安定し、**MuJoCo の 3D モデルと閉ループ**まで繋がった。
次は実機が来た日に前へ進むための準備（電子系 BOM / 接触→脱力の機体側実装 / サーボ層）。

### 直近で見つけて直した安全上の穴（詳細は DECISIONS）

1. **再起動後の自動再走行**（240ms で DRIVING へ）→ ARM が boot_id を名指しし、機体側が拒否
2. **fault 公開の race**（異常を検知したのに画面は「異常なし」）→ `latest()` が最新値を載せる
3. **CAD と可動域の不一致**（±60° は CAD の最新提案と違う）→ 意味別に3分割し ±50° CONDITIONAL へ
4. **旧12V トルク値の混入** → C044（7.4V/1:191）へ分離し、実測前の確定を禁止

## 旧 Current Goal

Phase 2/3 の**仮想実機**が揃った（2026-09-15）。SimulatedSnake（PC → 仮想ESP32 → 仮想サーボ →
シミュレータ）で通常走行と異常（通信断・再起動・緊急停止・サーボ故障）を再現できる。
次は**実機が来た日に前へ進むための準備**: 接触→脱力を機体側へ、電子系の BOM、ファームのサーボ層。

## Current Architecture

| 層 | 責任 | 主なファイル |
|---|---|---|
| 知覚 | ArUco で自己位置、YOLO で人。**外部固定カメラ前提** | `serpens/perception/` |
| 行動 | 内部状態 → 効用 → 10 状態 → `DriveCommand`（歩容パラメータ）としぐさ | `serpens/behavior/` |
| モーション | 歩容式・キーフレーム・呼吸を合成して**9軸の角度**を作る | `serpens/motion/` |
| セッション | 上記を 50Hz で回し、停止を出力へ届ける | `serpens/sim/session.py` |
| 安全（PC 側） | RUN / HOLD / DISABLED / EMERGENCY の分離とラッチ | `serpens/safety.py` |
| 駆動リンク | PC ⇄ 機体の契約。**機体側が自分で止まる** | `serpens/link/`, `docs/link_protocol.md` |
| 機体（模擬） | 状態機械・watchdog・歩容生成・上限強制 | `serpens/link/device.py` |

**継ぎ目の問題**: `session.step()` は `anim.send(bus, angles)` で 9軸の角度を直接サーボへ書く。
駆動リンクは歩容**パラメータ**を送る設計なので、両者が繋がっていない。
`Brain._set_drive()` は既に `GaitParams + γ0` を作っているので、変換は要らず継ぎ目を1枚入れるだけでよい。

## Completed（確かめたもの）

- Phase 1: 停止の3分離・実機経路の配線・終了処理（`tests/test_phase1_*.py` 20 件）
- Phase 2: 駆動リンク（`tests/test_phase2_*.py` 48 件、`tools/link_check.py` で条件 1〜12 と時間の実測）
- 縦スライスの接続（`serpens/robot.py` / `serpens/link/robot.py`、`--robot link`）。
  **アプリ経路でも PC 強制終了・USB 抜去で機体が自分で止まる**ことを試験で確認
- Virtual ESP32（7状態）+ 仮想サーボバス + テレメトリ v2（source=SIMULATION）
- 故障注入 4層（経路 / 機体 / サーボ / 指令値）と `docs/verification_status.md` の4段区分
- 可動域を意味別に分割（干渉開始 64.8/64.9° / mechanical ±55° / software ±50° CONDITIONAL）
- Belly（wheel/snake）× 摩擦プロファイル、歩容パラメータの掃引（`tools/gait_sweep.py`）
- GUI 下段に関節ペイン（指令角・実測角・安全・機体状態）
- 全 274 件が緑（`.venv/Scripts/python.exe -m pytest -q`、約 65 秒）
- 安全の絶対値（構想設計書 16章）を `config/robot.yaml` の `safety_limits` と
  `tests/test_safety_limits.py`（10件）に落とした。トルク上限（C044 参照値から 0.450N·m、比 0.287）を**経路を問わず**強制
- 駆動リンクに BODY（胴体姿勢）と TORQUE（脱力）を追加。とぐろ・鎌首・脱力がリンク越しに出せる
- 胴体ヨーの本数と移動性能の関係を測定（`docs/product_status.md` §3）
- 関節数の決め打ちを除去（`device_motion.py`。3軸構成で `KeyError` だったのを修正）

## Current Problems

000. （2026-09-21 Phase 2）**人物検出 5Hz が展示の反応速度のボトルネックになりうる**（正式 Risk）。一次反応は
     BEHAVIOR_INTERNAL 0.22s / END_TO_END_SIMULATED 約 0.52s（因果の窓の外）。実カメラ到着後に E2E を測る。
     6 / 8 / 10 の MuJoCo（PHYSICS_SIM）比較は未（MJCF は 9 軸案のみ）。Pitch は別実験（未着手）。

00. （2026-09-21）一次反応は PC 内 220ms だがカメラ経路（5Hz + 100ms）込みだと約 0.52s で因果の窓（≤300ms）の外。
    可視波数は 6 軸で 1 波が上限（8 軸案は 2 波だが前進量が約 4 割に落ちる）。「蛇らしさ」は人の評価が未

0. （2026-09-18）実カメラ経路は接続したが**実カメラの画像では未検証**（録画は仮想カメラ製）。
   近い偽マーカは区別できない（`docs/phase4_vision_bridge.md` §4）。接触→脱力は要求未確定
   （`docs/contact_release_requirements.md`）。電気安全ゲートは全項目未実測。C044 の電流値 UNKNOWN

1. **実 ESP32 が無い。** リンク経路は模擬機体でしか動かしていない（`--robot link --link-port COMx` は未検証）
2. **接触 → 脱力が 800ms**（構想設計書の目標 20ms）。PC を経由する限り届かない。機体側で負荷を見る必要がある
3. **電気的制限（層2）が無い**。電流検出も、独立して電源を切る安全 MCU も無い（`docs/safety_limits.md` §3）
4. **輪の内径 53.6mm** は構想設計書の「80mm 以上」に届かない。±72° まで狭めれば満たすが、
   可動域ととぐろの形は機構担当の領分なので変更していない（要判断）
5. ファームは未コンパイル・未書き込み、ESP32 とサーボバスの配線が未確定
6. 知覚が外部固定カメラ前提。室内を動き回る前提では成立しない
7. `tangential_drag_ratio = 0.02` は推定値。実機校正まで前進量の絶対値は信用しない

## 直近で判断が要ること

- **胴体ヨーの software limit ±50° は CONDITIONAL**。展示用とぐろは R03 の範囲で再設計が要る（機構担当）
- **STS3215 7.4V 1:191 のストールトルクが未確認** → 安全のトルク上限を出し直せない
- ESP32 ⇄ サーボバスの配線方式（ファームを書き込めない理由）

## Assumptions（覆るかもしれない仮定）

- **A1** 第一号機は「3軸ベンチ → 展示用 9軸」の順。室内ペット（バッテリ・搭載カメラ）はその後。
  理由: 実機が1台も無く、モータ制御と安全停止の実証が先。可逆（config を足すだけ）
- **A2** 電源は当面 12V テザー。バッテリ・BMS は電子系の設計と同時に検討する
- **A3** 床はフローリング等の平滑面。絨毯・段差は受動輪 14 個の前提を壊すので後で再検討
- **A4** 所有者認識は未着手。方式（顔 / 声 / タグ）は知覚の作り直しと一緒に決める
- **A5** 巻き付き動作は**実装しない**（憲章どおり）。とぐろは自分の体だけで巻く平面渦

## Blockers

- 実サーボ・ESP32 が無い（条件 15、ファームのビルド、電流・温度の実測）
- ESP32 ⇄ サーボバスの配線方式（`firmware/serpens_esp32/README.md` の「決まっていないこと」）
- CAD / PCB の担当（憲章はこのリポジトリの作業に含めているが、これまで機械設計は別担当）

## Next Best Actions（2026-09-25 フェーズ 2 後）

0. **実写**: 手持ちカメラ + 治具で `docs/floorwatch_phase2.md` §3 の品目を撮り、`tools/floorwatch_eval.py --root` で評価
1. Mosquitto を PC に入れて `tests/test_mqtt_live.py`（retain / LWT / QoS1 再送）を回す（`pip install paho-mqtt`）
2. フェーズ 3: `PoseSource.APRILTAG`、IMU + 歩容オドメトリ、σ の増え方。開始条件は「σ と局所センサーの健全性」で書く（ARUCO→APRILTAG の置換にしない）
3. 段差（線状）の合成サンプルと形での区別

## Next Best Actions（2026-09-25 Floor Watch フェーズ 1 後・旧）

0. フェーズ 2: 3 枚（通常 / 斜め照明 / 線光）から「床から出っ張っているか」（基準画像との差分・局所影・線光の曲がり）、
   大きさ推定、危険度の複合判定、データセットの仕組み（1 円玉・20mm ワッシャー・ビーズ・食べかす・模様/汚れ）
1. フェーズ 3: AprilTag（機体カメラ）+ IMU + 歩容オドメトリの自己位置と不確かさ（`PoseSource.APRILTAG` 追加、ArUco は検証用に）
2. フェーズ 4: HARDWARE_UNVERIFIED 項目の試験ツールと記録フォーマット（ユーザーが実行、私が解析）
3. 機体側: 連続する横関節の合計角の上限（汎用）、下向き ToF の段差停止（胴の XIAO）、通信方式ごとの Heartbeat/TTL 設定

## Next Best Actions（2026-09-21 Phase 2 後・旧）

0. **ユーザーが Pilot を実施**（`docs/human_pilot.md`）。回答を `output/pilot/responses.csv` に保存 → `tools/pilot_analysis.py`
1. Pilot の結果を `docs/body_configuration_decision_template.md` に写す（決定はその後）
2. yaw8 / yaw10 の MJCF（PHYSICS_SIM）比較。前進量のトレードオフを物理でも見る
3. Yaw が決まったら Limited Body Pitch +1 / +2 を別実験として

## Next Best Actions（2026-09-21 後・旧）

0. 一次反応の実経路: 検出周期を上げる（`person.detect_hz`）か、ESP32 側で検出直後に目を光らせる経路
1. 展示当日の「人格」調整の手順書（`behavior.grammar` の項目と効き方）
2. 実カメラが来たら `tools/motion_quality.py` の一次反応をカメラ込みで測り直す

## Next Best Actions（2026-09-18 後・旧）

1. ~~実カメラ用の観測器~~ 完了（`CameraObserver`）。実カメラが繋がったら `--camera 0` で誤差・遅れを測る
2. ~~再開前の位置確認~~ 完了（K キー）
3. ~~電子系のブロック図と BOM~~ 骨格まで完了。品番と容量は **C044 1 個の電流実測**（購入が要る）後
4. 接触→脱力の Q1〜Q5 を決めてもらう（製品・機構の判断）
5. firmware のサーボ層（writeServos / readServos）。配線方式が決まってから。コンパイルのみ

## Next Best Actions（旧）

1. **接触 → 脱力を機体側へ**（`link.faults` に負荷しきい値、100Hz で判定）。20ms 目標への唯一の道
2. 電子系のブロック図と BOM（MCU・ドライバ・配電・電流検出・IMU）。層2 が丸ごと無い
3. ファームのサーボ層を偽サーボで検証できる形にする
4. 3軸ベンチの config プロファイル（サーボが届いた日に試せる状態にする）
