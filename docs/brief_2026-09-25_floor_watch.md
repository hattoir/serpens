# 「床見守りヘビ型ロボット」ブリーフ（2026-09-25）— 実装前の 3 点

このブリーフは、既存リポジトリ（展示機 EX-1、9 軸、USB 駆動リンク、ROS 2 不使用）と前提が何か所も違う。
**実装に入る前に、ここに挙げた質問の答えをもらう。** 答えが無い項目は仮定を置かず止める（安全要件に関わるため）。

---

## 1. 不明・矛盾している点の質問リスト

### A. 既存リポジトリ・憲章との衝突（先に決めないと二重実装になる）

| # | 質問 | 既存の事実 |
|---|---|---|
| A1 | **これは EX-1（展示機）の後継か、別プロダクトか。** 同じリポジトリで進めるなら、既存の `serpens/`（9 軸・USB 駆動リンク）と `firmware/serpens_esp32/` を残すか置き換えるか | 憲章 (`docs/product_status.md` §6) は「展示機を第一号機」とし、`agent/DECISIONS.md` にその判断がある |
| A2 | **ROS 2 Jazzy を使う** とあるが、既存憲章と `CLAUDE.md` は「ROS 2 を使わない」。デスクトップ側だけ ROS 2 で、ロボットとの契約は MQTT だけ、という理解でよいか。`desktop/bridge/` は Home AI 側の資産か、このリポジトリか | `CLAUDE.md`「ROS 2 を使わない」「Home AI 本体をここに作らない。Client / Protocol / Adapter だけ」 |
| A3 | **関節構成が違う**: ブリーフは J1=首ピッチ + J2〜J5=ヨー（4 軸）。既存は胴体ヨー 6 + 首ピッチ + 頭ヨー + 頭ロール。直前に 6/8/10 ヨーの人間評価を準備したばかり（`docs/human_pilot.md`）。4 ヨーで蛇行する前提でよいか（簡易シミュレータでは 3 軸 98mm/周期、4 軸 198mm、6 軸 390mm: `docs/product_status.md` §3） | 4 軸は 1 波あたり 4 関節 = 1 波しか乗らず、旋回も弱い |
| A4 | J1 が首ピッチで**頭ヨーが無い**。カメラで床を撮る・「人を見る」ときの左右の向きは胴体全体で作るのか | 既存は J8（頭ヨー）で視線を作っている |
| A5 | 設定ファイル名が `config/snake.yaml`。既存の `config/robot.yaml`（1 ソース、テスト 415 件が読む）と統合するか、overlay として足すか | `load_config(overlay=...)` の仕組みがある |
| A6 | リポジトリ構成案（`firmware/main`, `sim/`, `desktop/...`）は既存（`serpens/`, `simulation/`, `firmware/serpens_esp32/`, `tools/`）と違う。**移動・改名は履歴とテストを壊す**ので、新構成へ寄せる範囲を決めたい | `C:/2026/CLAUDE.md`「フォルダの改名・移動を思いつきで実行しない」 |

### B. ハードウェア・電気（一次資料で確認できていない）

| # | 質問 | 根拠 |
|---|---|---|
| B1 | **2S リポ（満充電 8.4V）を STS3215 7.4V 版に直結するのか。** レジスタ 14「最高入力電圧」の初期値は **80 = 8.0V**。満充電で電圧異常（番地 65 bit0）になる可能性がある。レギュレータを入れるか、レジスタを書き換えるか（EEPROM 書き換えは私の作業禁止事項） | `docs/sts3215_registers.md` §注意 1 |
| B2 | STS3215 **7.4V 版の型番**（C044?）とストール電流・無負荷電流。既存は 12V 品の 2.7A / 200mA を流用していない（UNKNOWN のまま） | `docs/electronics.md` §2 |
| B3 | ヒューズの定格、電圧監視の回路（ESP32 の ADC 直か専用 IC か）、**本体停止ボタンはサーボ電源を物理的に切るか**（切らないなら要件 8「サーボバスの電源を切る」を誰が実行するか） | `docs/safety_limits.md` §5 ELECTRICAL_SAFETY_GATE 8 項目すべて未実測 |
| B4 | サーボバスの電源遮断の手段（ハイサイド MOSFET / リレー）。**ESP32 が暴走したときも切れる**必要があるか（独立 watchdog） | 同上 independent_power_cut |
| B5 | 「節は強く引くと外れる構造」— 外れたときサーボの電源線・信号線はどうなるか（短絡の可能性）。外れ検知をバスの無応答で行うと、**バスの先の全サーボが同時に無応答**になる。それで「どの節か」を判定してよいか（最後に応答した ID の次） | 要件 8 |
| B6 | 「機械側にもトルクリミッター」— スリップクラッチのトルク値と再現性。ソフトのトルク上限（現在 0.450N·m、ストール参照比 0.287）より上か下か | `config/robot.yaml` safety_limits.torque |
| B7 | IMU の型番と搭載位置（頭の XIAO Sense 内蔵 IMU か、胴体か）。頭だけだと首ピッチで姿勢が変わる | 既存 IMU 無し |
| B8 | 頭の XIAO ESP32S3 Sense と本体 ESP32-S3 の接続（UART? 既存は行プロトコル）。カメラ映像は XIAO から直接 Wi-Fi か | `serpens/hw/head_io.py` |
| B9 | 受動車輪の**横滑りゼロ**は絨毯・敷居でも成り立つか（簡易シミュレータの前提そのもの） | `agent/STATE.md` A3 |
| B10 | 小型スピーカーの「話す」は録音再生か TTS か。デスクトップが音声を送るのか | 未定義 |

### C. 安全要件の意味（マイコン内で完結させるには数値と定義が要る）

| # | 質問 | 既存の状態 |
|---|---|---|
| C1 | 要件 1「横関節の角度の合計の上限」— **いくつか**。既存の休憩姿勢（緩い弧 240°、`poses.rest_arc`）や充電ドッキングの「とぐろ」は禁止になる。4 ヨー × ±50° = 最大 200° なので 4 軸なら輪は作れないが、増設後は要る | `docs/safety_limits.md` 最小曲げ半径 40mm |
| C2 | 要件 2「負荷のしきい値」— 何 % を何 ms 続けたら切るか。STS3215 の「現在負荷」は PWM デューティ比であって力ではなく、符号ビットも仮説。**歩容中の自己負荷（模擬で飽和率 ~12%）と区別する方法** | `docs/contact_release_requirements.md` Q1〜Q6 未回答 |
| C3 | 要件 2 と 9「力を抜く」— 1:191 の減速比で**脱力しても外から回せない可能性**。脱力が「解放」になる根拠 | 同上 §2 バックドライブ未測定 |
| C4 | 要件 3「heartbeat 1 秒」— 既存は USB で 400ms（DRIVE TTL 300ms）。Wi-Fi/MQTT では 1 秒でよいとして、**停止時に「その場保持」か「脱力」か**。子どもの家では脱力（要件 2 と同じ）で統一するか | `serpens/link/device.py` は保持（HOLD） |
| C5 | 要件 4「0.15 m/s、人の近くで 0.05 m/s」— 既存は「1m 以内 0.08 m/s」（構想設計書 16 章）。**「人の近く」フラグは誰が立てるか**（デスクトップの検出 → MQTT → マイコン。検出が落ちたらフラグはどうなるか: 安全側は「不明なら近くとみなす」） | `config/robot.yaml` near_person_mm / near_speed_limit_mm_s |
| C6 | 要件 5「温度と電圧の下限」— 数値（既存: 温度 60℃ で機体側緊急停止、45℃ で休憩要求）。電圧は 2S の何 V（例 6.6V）で止め、**止めたあと再充電までロックするか** | `config/robot.yaml` link.faults |
| C7 | 要件 6「声の停止」— 音声認識はデスクトップにしかない（マイクは頭？）。声 → MQTT → 停止 は**マイコン内で完結しない**。「ボタンと MQTT は機体内・声はベストエフォート」と分けてよいか | — |
| C8 | 要件 7「人の操作なしでは再開しない」— 再開の操作は**本体ボタンのみ**か、MQTT の `clear_fault` も許すか（既存は CLEAR_FAULT → DISARMED → 明示 ARM の 2 段） | `docs/link_protocol.md` |
| C9 | 要件 9「滑りの検出と記録」— 指令角と実角のずれの**しきい値（度）と時間**。歩容中の追従誤差（模擬で RMS 3.5°）と区別する値 | `docs/phase3b_physics_sim.md` |
| C10 | 要件 10「段差の検知」— ToF の取り付け高さと角度、段差とみなす距離差（例 60mm）、**後退の距離と速度**。後退中の後方は見えない（尾側にセンサ無し） | `docs/safety_limits.md` §2 落下・段差 |

### D. 通信・位置

| # | 質問 |
|---|---|
| D1 | MQTT の QoS と retain（heartbeat は QoS 0 / retain なし、safety は QoS 1 / retain あり、が妥当）。TLS と認証は家庭内 LAN 前提で省くか |
| D2 | ロボットの Wi-Fi が切れたときの動作 = heartbeat 途絶と同じ扱いでよいか（家具の下で電波が弱くなる） |
| D3 | 位置: **AprilTag**（tag36h11?）か既存の **ArUco**（DICT_4X4_50、検出は実装済み）か。天井カメラの機種・高さ・解像度。「見えない所」は家具の下 = 巡回の主目的地なので、見えない時間の方が長い前提で推定精度をどう検証するか |
| D4 | 家具の下の高さ（ロボットの高さ 50mm + 頭 ToF）。潜り込んで**抜けられなくなったとき**の扱い |

### E. 進め方・作業範囲

| # | 質問 |
|---|---|
| E1 | 現在の私の禁止事項は「実サーボ操作・EEPROM 変更・ファーム書き込みをしない」。**フェーズ 1 は実サーボを回す作業そのもの**なので、誰が通電・書き込みを行うか（ユーザーが手元で実行し、私はコードと手順と判定基準を出す、でよいか） |
| E2 | フェーズ 1 の「手で関節を止めたとき 10 回中 10 回、力が抜ける」の**測り方**: 脱力までの時間の上限（既存目標 20ms は PC 経由では届かず、マイコン内でも判定回数で 10〜20ms）と、それを何で測るか（高速度動画 / ロジックアナライザでトルク OFF コマンドの送出時刻） |
| E3 | PlatformIO（ブリーフ）か arduino-cli（既存、コンパイル実績あり）か |

---

## 2. `config/snake.yaml` と MQTT メッセージ形式の案

### 2.1 `config/snake.yaml`（案）

既存の `config/robot.yaml` の語彙（`min_deg` / `max_deg` / `max_speed_dps` / `safety_limits`）に合わせ、
**overlay として読める形**にした（`load_config(overlay="config/snake.yaml")`）。値のうち「要確認」は仮置き。

```yaml
# Serpens 床見守り機（案）。関節の数・向き・ID・角度制限はここだけで定義する。コードに直書きしない
version: 1
body:
  length_mm: 700              # 要確認: 5 関節での全長
  diameter_mm: 50
  mass_g: 800                 # 要確認（実測なし）
joints:                       # 尾 → 頭の順。axis: yaw（横）/ pitch（上下）。増設はここに行を足すだけ
  - {name: J5, servo_id: 5, axis: yaw,   x_mm: 100, direction: 1, min_deg: -50, max_deg: 50, max_speed_dps: 240}
  - {name: J4, servo_id: 4, axis: yaw,   x_mm: 195, direction: 1, min_deg: -50, max_deg: 50, max_speed_dps: 240}
  - {name: J3, servo_id: 3, axis: yaw,   x_mm: 290, direction: 1, min_deg: -50, max_deg: 50, max_speed_dps: 240}
  - {name: J2, servo_id: 2, axis: yaw,   x_mm: 385, direction: 1, min_deg: -50, max_deg: 50, max_speed_dps: 240}
  - {name: J1, servo_id: 1, axis: pitch, x_mm: 480, direction: 1, min_deg: -8,  max_deg: 60, max_speed_dps: 120}
  # 角度制限の出どころ（CAD 干渉開始 / 機械設計 / ソフト運用）は robot.yaml の joint_limit_policy と同じ 3 分割で持つ
servo:
  model: STS3215-7.4V         # 要確認: 型番（C044?）
  bus_baud: 1000000
  max_input_voltage_reg: 80   # 番地 14 の初期値 = 8.0V。**2S 満充電 8.4V との整合を確認**（B1）
  load_sign_bit: 10           # 仮説（一次資料に無い）
safety:                       # **マイコン内で完結。MQTT からは変更不可**（値はビルド時に config から埋め込む）
  yaw_sum_max_deg: 180        # 要件 1: 横関節の角度の合計の上限（輪の禁止）。C1 で決める
  load_release:               # 要件 2 / 9: 負荷で脱力
    ratio: 0.60               # 現在負荷（デューティ比）のしきい値。要件 2 の C2 で決める
    hold_ms: 20               # この時間続いたら（100Hz なら 2 周期）
    action: torque_off        # torque_off / hold
  slip_detect:                # 要件 9: 指令角と実角のずれ（記録のみ。停止はしない）
    error_deg: 15
    hold_ms: 200
  heartbeat_timeout_ms: 1000  # 要件 3
  heartbeat_action: torque_off   # C4: hold か torque_off か
  speed:                      # 要件 4
    max_mm_s: 150
    near_person_mm_s: 50
    near_person_unknown_is_near: true   # 検出が途絶えたら「近い」とみなす
  temp_stop_c: 60             # 要件 5
  voltage_stop_v: 6.6         # 要件 5: 2S の下限（要確認）
  voltage_resume_v: 7.2       # これ以上に戻るまで再開しない
  detach:                     # 要件 8: 節の外れ
    no_response_count: 3      # 連続何回の無応答で判断するか
    action: bus_power_off     # サーボバスの電源を切る（B3/B4 の回路が要る）
  cliff:                      # 要件 10
    tof_drop_mm: 60           # 段差とみなす距離差
    back_off_mm: 150
    back_off_mm_s: 50
  resume_requires: button     # 要件 7: button / button_or_mqtt_clear（C8）
gait:
  # θ_i = α·sin(ωt + i·β) + γ（i は尾側から 0..N−1 のヨー関節）。生成はマイコン内
  limits: {alpha_deg: 40, omega_hz: 1.0, beta_deg: 150, gamma_deg: 30}
  presets:
    forward: {alpha_deg: 30, beta_deg: 90, omega_hz: 0.5, gamma_deg: 0}   # 4 ヨーで 1 波（β = 360/4）
  advance_per_cycle_mm: null  # 実機で校正するまで未定（シミュレーションの値を入れない）
control:
  loop_hz: 100
  loop_overrun_ms: 15         # 周期がこれを超えたら安全側（速度 0）
  cmd_ttl_ms: 500             # 指令の有効期限（heartbeat より短く）
mqtt:
  broker: mosquitto.local
  base: robot/serpens
  state_hz: 10
```

### 2.2 MQTT メッセージ（案）— すべて JSON、`v` は形式の版、`seq` は送り手の連番

**`robot/serpens/cmd`**（デスクトップ → ロボット。QoS 0、retain なし。マイコンは**最新の 1 つだけ**を使い、`ttl_ms` 経過で捨てる）

```json
{"v":1,"seq":1234,"t_ms":1727000000000,"ttl_ms":500,
 "gait":{"alpha_deg":30,"beta_deg":90,"omega_hz":0.5,"gamma_deg":5},
 "speed_mm_s":100,
 "neck_deg":20,
 "led":{"status":"green","ir":true},
 "say":"item_found",
 "near_person":false}
```
- 安全上限はマイコン側の config が優先。範囲外は**丸めずに拒否**して `safety` に `cmd_rejected` を出す。
- 停止は別トピックにしない: `{"v":1,"seq":..,"stop":"halt"}`（保持）/ `"stop":"release"`（脱力）。

**`robot/serpens/state`**（ロボット → デスクトップ、10Hz、QoS 0）

```json
{"v":1,"seq":998,"t_ms":1727000000100,"uptime_ms":123456,"boot_id":17,
 "mode":"DRIVING",
 "joints":[{"id":1,"cmd_deg":20.0,"pos_deg":19.4,"load":0.12,"temp_c":41,"volt_v":7.9,"ok":true}, ...],
 "imu":{"ax":0.0,"ay":0.0,"az":9.8,"gx":0.0,"gy":0.0,"gz":0.0},
 "tof_mm":52,
 "batt_v":7.9,
 "loop":{"period_us":10010,"busy_us":3200,"overruns":0},
 "near_person":false,
 "source":"HARDWARE"}
```
- `mode`: BOOT / DISARMED / ARMED_HOLD / DRIVING / FAULT_HOLD / EMERGENCY_LATCHED / TORQUE_DISABLED（既存の 7 状態をそのまま）。
- `source` は HARDWARE / SIMULATION（模擬の値を実測と混ぜない）。

**`robot/serpens/safety`**（ロボット → 全員。QoS 1、retain あり = 最後の理由が残る）

```json
{"v":1,"seq":31,"t_ms":...,"boot_id":17,
 "event":"stopped",
 "reason":"LOAD_RELEASE",
 "detail":{"joint_id":3,"load":0.71,"hold_ms":20},
 "action":"torque_off",
 "resume":"button"}
```
- `reason` の語彙: JOINT_LIMIT / YAW_SUM_LIMIT / LOAD_RELEASE / HEARTBEAT_LOST / OVERSPEED / OVERHEAT / UNDERVOLTAGE /
  BUTTON / MQTT_STOP / VOICE_STOP / DETACHED（`detail.joint_id` = 外れた節の直前の ID）/ SLIP（記録のみ）/ CLIFF / LOOP_OVERRUN / CMD_REJECTED。
- 再開したときも `"event":"resumed","by":"button"` を出す。

**`robot/serpens/heartbeat`**（デスクトップ → ロボット、5Hz、QoS 0）

```json
{"v":1,"seq":55,"t_ms":...,"near_person":false}
```
- `near_person` は heartbeat に載せる（検出が止まれば heartbeat も止まり、`near_person_unknown_is_near` が効く）。

**`home/events/serpens`**（デスクトップの機能 → 通知サービス）

```json
{"v":1,"t_ms":...,"module":"serpens","type":"item_found","pos_mm":[1200,3400],"image":"…/shot_0012.jpg","confidence":0.83}
```

---

## 3. フェーズ 1 の作業計画（サーボ 1 個）

**完成条件**: 手で関節を止めたとき **10 回中 10 回、力が抜ける**（ログの文字ではなく、動画と時間の実測で）。
**前提**: 実サーボの通電・レジスタ書き換え・書き込みはユーザーが手元で行う（E1）。私はコード・手順・判定基準・ログの解析を出す。

### 3.1 準備（ハード無しで私ができる）

1. **安全ロジックを純粋関数に切る**（`serpens/safety_rules.py` 案。入力は「角度・負荷・温度・電圧・heartbeat 経過・応答マップ」、出力は「停止理由と処置」）。ホストで `pytest`（要件 1・2・3・5・8・9 の単体テスト）。これがフェーズ 2 の土台。
2. マイコン側の**最小プロトコル実装**（既存 `firmware/serpens_esp32/` に 1 軸用の `servo_bus` を足す: PING / READ 56〜63 / WRITE 42 / トルク ON-OFF 40 / トルク上限 48）。**コンパイルのみ**（`tools/build_firmware.py`）。
3. 既存 `MockServoBus` を使って PC 上で同じ手順を通す（負荷を注入 → 脱力 → 記録）。
4. 手順書 `docs/phase1_servo_bench.md`: 配線図、レジスタ確認の順（**最初に番地 14 の最高入力電圧**）、電源（安定化電源 7.4V。2S 直結は B1 の答えまで不可）、判定基準、記録様式。

### 3.2 実機（ユーザーが実行、私が判定）

| 手順 | 何をするか | 記録する物 |
|---|---|---|
| 1 | 安定化電源 7.4V・電流計直列で 1 個だけ接続。PING → ID 1、番地 14 の値、ファーム版 | 電流（無負荷・保持）、レジスタのダンプ |
| 2 | トルク上限（番地 48）を config の比（0.287）で書く（**RAM 側のみ。EEPROM は書かない**） | 読み戻し |
| 3 | 位置指令 ±30° を 0.5Hz で往復、50Hz で位置・負荷・温度を読む | 追従誤差、負荷の分布（歩容中の自己負荷の基準になる） |
| 4 | 手で関節を止める × 10 回（毎回、角度と方向を変える） | **各回**: 接触の動画（30fps 以上）、負荷の時系列、脱力コマンド送出時刻、脱力までの時間 |
| 5 | しきい値・保持時間を調整（`safety.load_release`）。変更は**私が値を提案し、ユーザーが承認してから** | 変更前後の 10 回ずつ |
| 6 | 温度: 連続往復 10 分。番地 63 の温度と実温度（熱電対があれば）| 温度上昇の曲線 |

判定: 10/10 で脱力し、脱力までの時間（接触の動画フレーム → トルク OFF）が config の `hold_ms` + 通信 1 往復（約 2ms）+ 動画の 1 フレーム以内。
**脱力しても指が抜けない（バックドライブ不足）なら、それも結果として記録**（C3。要件の見直しに戻る）。

### 3.3 出口

- `docs/verification.md`（既存 `docs/verification_status.md` に統合するか要確認）に「何をどう測って何が出たか」を記録。
  レベルは HARDWARE_VERIFIED（日付・条件・測定器つき）。
- ELECTRICAL_SAFETY_GATE の `real_current` / `servo_temperature` が初めて埋まる。
- しきい値・上限の変更は毎回ユーザー承認（検証のルール）。時間は 1 セッション 2 時間で区切り、テスト → 診断 → 修正 → 再テスト。

### 3.4 このフェーズでやらないこと

MQTT・歩容・5 関節・AprilTag・カメラ配信。電池直結（B1 の答えまで）。EEPROM の書き換え。
