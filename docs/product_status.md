# Serpens 製品憲章と現物の対応

作成 2026-09-14。**この文書は「いま何があって、何が無くて、次に何をするか」の一枚。**
実装の根拠は各ファイルへのリンク。推測は「未確認」と明記する。

前提の変化: これまでの実装は**展示会用（高さ600mmの台・1.2mマット・上方の外部カメラ・12Vテザー）**
として作った。憲章は**室内で暮らす自律ペット（バッテリ・充電ドック・搭載センサ）**を求めている。
**既存資産の大半（歩容・駆動リンク・安全・行動）はどちらでも使える**が、電源・知覚・機構は別物になる。
どちらを第一号機にするかは §4 の決定事項。

## 1. サブシステムの現在地

| サブシステム | 状態 | いまある物 | 憲章との差 |
|---|---|---|---|
| Mechanical | 部分 | 寸法・質量収支と**3段の可動域**（geometry / mechanical / operational）。CAD R03 の干渉開始 64.8/64.9° を境界として持ち、mechanical ±55°・software ±50°（CONDITIONAL） | 図面はリポジトリに無い。シェル・ケーブル経路・修理性・製造性の検討が無い。頭部の干渉検査は未 |
| Actuation | 部分 | STS3215 の仕様を一次情報で確認（[docs/sts3215_registers.md](sts3215_registers.md)）。トルク・速度・分解能・電圧の値は config 済み | ギア比・スリップ機構・実測トルク余裕は未検討。**サーボ実機ゼロ** |
| Electronics | 設計案 | ブロック図・ゲート 8 項目の測り方・BOM の骨格（[electronics.md](electronics.md)）。ESP32 に依存しない遮断器 + 常閉 E-STOP の構成 | 回路図・PCB・品番は無い。**C044 の電流値が UNKNOWN** なので容量・ヒューズ・閾値を決められない。KiCad 未着手 |
| Power | **未着手** | 12V テザー給電の注意書きのみ（README §2.4） | バッテリ・BMS・電流監視・突入電流・電圧降下の設計が無い |
| Embedded | 部分 | 駆動リンク v2、Virtual ESP32（7状態）+ 仮想サーボバス、故障注入、値の一致検査、**ファームのコンパイル成功**（フラッシュ7%/RAM5%） | **未書き込み**。サーボ読み書き（writeServos/readServos）と配線が未確定 |
| Locomotion | 実装済み（模擬） | serpenoid 歩容、2D シミュレータ、**MuJoCo の 3D モデル**（任意依存）、Belly 3種、歩容掃引（Pareto 候補） | 摩擦もサーボ応答もすべて未実測。**サーボゲインで前進量が ±45% 変わる**ので、実機での同定が最優先 |
| Sensors | 部分 | 頭部 I/O の行プロトコル（ToF・タッチ×2・LED×2）とモック（[head_io.py](../serpens/hw/head_io.py)）、サーボの位置/負荷/温度/電圧の読み出し（[state_poller.py](../serpens/hw/state_poller.py)） | **IMU が無い。** 搭載カメラも無い。電流センサも無い |
| Vision | 実装済み（外部カメラ前提） | ArUco・床ホモグラフィ・YOLO。**模擬画像の Vision で閉ループ**（[phase4_vision_bridge.md](phase4_vision_bridge.md)）。実画像経路 `CameraObserver`（カメラ / 録画 → 自己位置・人）を `--camera` に接続。見失い後は人が K で位置を確認 | 実カメラでは未測定。上方の外部カメラ前提。所有者の個人識別は無い |
| Networking | **未着手** | USB CDC（PC 直結）だけ | Wi-Fi、PC が落ちたときの network fallback が無い |
| Behavior | 実装済み（模擬） | 内部状態 4 種 + 効用、10 状態、**語彙（primitives）/ 文法（grammar, config）/ 移動（locomotion）**。stop-and-go・一次反応・呼吸の位相波・予備動作・視線そらし（[motion_quality.md](motion_quality.md)） | 充電要求・所有者探索の状態が無い。「蛇らしさ」は人が見ての評価が未。LLM は不使用（憲章どおり） |
| Charging | **未着手** | 無し | ドック・姿勢・接点・充電制御のすべて |
| Safety | 部分 | PC 側の 3 種停止とラッチ（[safety.py](../serpens/safety.py)）、機体側の watchdog・緊急停止ラッチ・上限強制（[device.py](../serpens/link/device.py)）、**安全の絶対値**（[safety_limits.md](safety_limits.md)、`tests/test_safety_limits.py`） | 接触→脱力 20ms が未達（現状 800ms）。電気的制限（層2）が丸ごと無い。物理の緊急停止スイッチが無い |
| PC AI Integration | 実装済み | 50Hz 制御ループ（[runner.py](../serpens/runner.py)）、GUI（[gui/](../serpens/gui/)）、駆動リンクの PC 側（[client.py](../serpens/link/client.py)）、出力先の継ぎ目（[robot.py](../serpens/robot.py)） | **行動 → 駆動リンク → 機体は 2026-09-14 に接続済み**（`--robot link`）。実 ESP32 は未接続 |

## 2. FIRST VERTICAL SLICE の現在地

| 憲章の段 | 状態 | 根拠 |
|---|---|---|
| 2〜数 segment | 設計は**関節数に依存しない**ことを確認（§3）。実物は無い | `config/robot.yaml` の `joints` を減らすだけで動く |
| motor control | 模擬で完成。実機は未 | `MockServoBus` / `FeetechServoBus`、`tools/servo_setup.py`。出力先は `--robot direct/link` で差し替え |
| basic gait | 模擬で完成 | 参照式と機体側の出力が 1e-9° 以内で一致 |
| PC communication | **完成**（模擬） | [docs/phase2_acceptance.md](phase2_acceptance.md) の条件 1〜12 |
| safe start/stop | **完成**（模擬） | 同上。USB 抜去・PC 強制終了で機体が単独停止。**アプリ経路でも同じ**（`tests/test_phase3_link_robot.py`） |

**つまり縦スライスは「実機のサーボ 1 個を回す」以外は揃っている。** 律速はハードウェア。

## 3. 測定: 胴体ヨーの本数と移動性能（2026-09-14、シミュレータ）

振幅 30°、周期 2 秒、`tangential_drag_ratio` 0.02、Ω は「胴体でちょうど 1 波」になる値。

| 胴体ヨー | Ω | 全長 | 1周期の前進 | γ0=20° の旋回 | 旋回半径 |
|---|---|---|---|---|---|
| 3 軸 | 120° | 660 mm | 98 mm | 9.0°/周期 | R≈554 mm |
| 4 軸 | 90° | 755 mm | 198 mm | 19.3°/周期 | R≈507 mm |
| 5 軸 | 72° | 850 mm | 296 mm | 30.5°/周期 | R≈452 mm |
| 6 軸 | 60° | 945 mm | 390 mm | 42.1°/周期 | R≈388 mm |

読み方: **3 軸は 6 軸の 1/4 しか進まず、ほとんど曲がれない。**
縦スライス（モータ制御・通信・安全停止の検証）には 3 軸で十分だが、
**「生き物らしく室内を動く」性能は 5〜6 軸から。** 最初のベンチは 3 軸、製品は 6 軸以上を推奨。

この測定は `tangential_drag_ratio = 0.02`（推定値）に依存する。**実機で校正するまで相対比較にのみ使う。**

## 4. 決めていただきたいこと

0. **展示機の身体構成。** Body Yaw 6 / 8 / 10、Limited Body Pitch 0 / 1 / 2 を、
   **Human Evaluation（`docs/human_pilot.md`）・Simulation（`output/body_compare.md`）・CAD・BOM・Safety** で決める。
   材料は揃っている（overlay `config/robot_yaw{6,8,10}*.yaml`、匿名クリップ、決定レポートの雛形
   `docs/body_configuration_decision_template.md`）。**Pilot 前に決めない。** Pitch は Yaw 決定後の別実験。
0'. **胴体ヨーの operational limit（現在 ±50° CONDITIONAL）。** CAD R03 の「64.8° 干渉なし / 64.9° 干渉」に
   合わせて下げた。旧とぐろ（329°）は入らず、休憩姿勢は緩い弧（rest_arc, 240°）。最終値が決まったら
   `config/robot.yaml` の `min/max_deg` と展示用とぐろ（`legacy_poses` の代替）を作り直す。
1. **第一号機はどちらか。**（a）展示会用の 9 軸テザー機を仕上げる（b）室内ペットとして電源・知覚から作り直す
   （c）3 軸のベンチを先に作って（a)(b) の共通部分を実機検証する。
   → 既存の知覚（外部固定カメラ + ArUco + 床ホモグラフィ）は (b) では**使えない**。
2. **電源。** テザー 12V のままか、バッテリ + BMS へ行くか。バッテリなら重量・充電・保護の設計が機構より先に要る。
3. **室内の床。** フローリング前提か、絨毯・段差・敷居を含むか。受動輪 14 個は平滑面前提の設計。
4. **所有者認識の方式。** 顔・声・タグ（ArUco / BLE）。個人情報の扱いが方式で変わる。
5. **機構の担当。** CAD・PCB を誰が持つか。これまで「機械設計は別担当」だったが、
   憲章は CAD / KiCad / BOM もこのリポジトリの作業に含めている。

## 5. 実機が無くても進められる作業（PRIORITY 順）

憲章の Architecture → Segment prototype → Actuation → Power → Motor control → … に沿う。

| # | 作業 | 前提 | 成果物 |
|---|---|---|---|
| 1 | **安全の数値を決める**（最大力・最大トルク・電流制限・関節速度・停止距離）。STS3215 の 30kgf·cm から、指を挟んだときの力を計算して上限を決める | 決定不要 | `docs/safety_limits.md` + `config` の上限 |
| 2 | **3 軸ベンチの構成を確定**（config プロファイル + 部品表）。サーボ 2 個が届いたら即日試せる状態にする | 決定 1 | `config/robot_bench3.yaml`、BOM |
| 3 | ~~電子系のブロック図と BOM~~ → **骨格まで完了 2026-09-18**（品番・容量は C044 の実測後） | 決定 2 | `docs/electronics.md` |
| 4 | **ファームのサーボ層**（同期書き込み・状態読み出し）をモックで検証できる形に | 決定なし（配線は後） | `firmware/` + 偽サーボでの試験 |
| 5 | ~~GUI/行動と駆動リンクの接続（Phase 3）~~ | **完了 2026-09-14** | `serpens/robot.py` + `serpens/link/robot.py` |
| 6 | ~~**IMU を前提にした自己位置**の検討（外部カメラ依存を外す道筋）~~ → **模擬で実装 2026-09-26**（AprilTag + IMU + 歩容オドメトリ、σ で書いた開始条件） | 決定 1・3 | `docs/floorwatch_phase3.md`、`serpens/localization/` |

## 6. 境界（憲章どおり）

- Home AI 本体はここに作らない。必要なら **Client / Protocol / Adapter だけ**。現状、依存も接続も無い。
- robot-codesign 本体も再実装しない。
- **Serpens は単体で安全に止まれる。** これは Phase 2 で機体側 watchdog として実装済み
  （PC が落ちても USB が抜けても、機体が自分で保持へ入る）。
