# Serpens EX-1 — 展示用ヘビ型ロボット 制御ソフトウェア

9軸（胴体ヨー×6 + 首ピッチ J7 + 頭ヨー J8 + 頭ロール J9）のヘビ型ロボットを、Python だけで動かすための
ソフトウェアです。**モックファースト**で作っており、実機が1台もなくてもシミュレータ上で全機能が動きます。

- ROS 2 不使用 / LLM 不使用（行動選択は手書きの効用関数）
- 寸法・しきい値・ゲインは全部 `config/robot.yaml`（コードにマジックナンバーを書かない）
- 実機接続は `--bus feetech --port COM5`。**ただし引数を足すだけで展示ができるわけではない**。
  実機は待機（停止）から始まり、**実観測の自己位置・床の校正・ESP32 駆動リンク**が揃うまで
  自律走行は開始できない（`serpens/safety.py`）。ESP32 ファームは Phase 2 で作る（未実装）

---

## 1. セットアップ（Windows 11 / Python 3.12）

PowerShell で、このフォルダ（`serpens/`）に移動してから実行します。

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m pip install --no-deps -r requirements-nodeps.txt
.\.venv\Scripts\python.exe tools\check_env.py
.\.venv\Scripts\python.exe -m pytest
```

`Activate.ps1` で venv を有効化したい場合、PowerShell の実行ポリシーで止められることがあります。
その場合は上のように `.\.venv\Scripts\python.exe` を直接呼べば有効化は不要です。
VSCode ではコマンドパレット →「Python: Select Interpreter」→ `.venv` を選んでください。

### なぜ2段階インストールなのか（opencv の競合）

ArUco を使うので **`opencv-contrib-python` だけ**を入れます。ところが `ultralytics` は
`opencv-python` に依存しており、普通に `pip install ultralytics` すると両方が入って `cv2` が壊れます
（`cv2.aruco` が消える、import が不定になる等）。

そこで:
1. `requirements.txt` … ultralytics の依存を **opencv-python だけ除いて** 列挙
2. `requirements-nodeps.txt` … ultralytics 本体を `--no-deps` で入れる

`pip check` は「ultralytics requires opencv-python」と警告しますが、**想定どおりなので無視**してください。
万一 `opencv-python` が入ってしまったら:

```powershell
.\.venv\Scripts\python.exe -m pip uninstall -y opencv-python opencv-python-headless opencv-contrib-python
.\.venv\Scripts\python.exe -m pip install opencv-contrib-python
```


### サーボ SDK について

`ftservo-python-sdk`（Feetech 公式リポジトリ由来、import 名 `scservo_sdk`）を使います。
`WritePosEx()`（位置・速度・加速度の同時指定）、`read1ByteTxRx()` / `read2ByteTxRx()`、
Sync Read / Sync Write が揃っています。
よく似た名前の `feetech-servo-sdk` は機能削減版（`sms_sts` が無い）で、しかも同じ `scservo_sdk` という
名前でインストールされて上書きし合うので、**入れないでください**（`tools/check_env.py` が検出します）。
なお `scservo-sdk` という名前のパッケージは PyPI にありません。

### YOLO の重み（人物検出に必要。自動ダウンロードはしない）

**展示会場にネットは無い前提**です。ultralytics は重みのダウンロードや更新確認で勝手に外へ出るので、
`serpens/app.py` の先頭で次を設定してから import しています（import より前でないと効きません）。

```
YOLO_AUTOINSTALL=false   足りないパッケージを勝手に pip install しない
YOLO_OFFLINE=true        オンライン判定を常に false にする（重みの取得・更新確認をしない）
YOLO_HUB_OFFLINE=true    Ultralytics HUB へ通信しない
```

**置いてある重み**（取得済み。git には入れない）

| 項目 | 値 |
|---|---|
| ファイル | `models/yolo11n.pt`（場所は config の `person.model_path`） |
| 取得元 | https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt |
| サイズ | 5,613,764 バイト |
| SHA256 | `0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1` |
| 取得日 | 2026-09-12 |

置き直すときは上の URL から取得し、SHA256 が一致することを確認してください。

```powershell
.\.venv\Scripts\python.exe -c "import hashlib,pathlib;print(hashlib.sha256(pathlib.Path('models/yolo11n.pt').read_bytes()).hexdigest())"
```

**動作確認**（ultralytics 同梱の bus.jpg。4人写っています）

```powershell
.\.venv\Scripts\python.exe tools\perception_live.py --source .venv\Lib\site-packages\ultralytics\assets\bus.jpg --homography output\_test_h.json --frames 1 --no-window
# → [  0.00s] markers=[]  人=4
```

**オフライン起動の確認（展示前に必ず実行）**

ネットワークを遮断した状態で最後まで動くことを、次のツールで確認できます（外へ出ようとした瞬間にエラーになります）。

```powershell
.\.venv\Scripts\python.exe tools\offline_check.py --camera 0 --gui --seconds 12
# → [オフライン検証] 最後まで動きました
```

`pytest tests/test_offline.py` でも、遮断下で重みの読み込みと推論ができることを確認しています。

**注意**: YOLO の CPU 推論中は Python の GIL を握るため、制御周期が跳ねます（実測: 最悪 96ms、12秒で11回）。
`person.detect_hz`（既定 5Hz）と `person.imgsz`（既定 480）で調整できます。

---

## 2. Windows 固有の注意

### 2.1 COM ポート番号の確認

サーボ用（Waveshare Bus Servo Adapter (A)）と頭部用（XIAO ESP32S3）で **USB が2本**あり、
それぞれ別の COM 番号になります。どちらがどちらか、1本ずつ抜き差しして確認してください。

- **デバイスマネージャ**: スタートボタン右クリック → デバイスマネージャ →「ポート (COM と LPT)」
- **PowerShell**: `[System.IO.Ports.SerialPort]::GetPortNames()`
- **Python（pyserial）**: `.\.venv\Scripts\python.exe -m serial.tools.list_ports -v`

COM 番号は USB の挿し口を変えると変わることがあります。展示会場では**毎回同じ口に挿す**こと。
番号を固定したい場合はデバイスマネージャ → ポートのプロパティ → ポートの設定 → 詳細設定 →
「COM ポート番号」で変更できます。

### 2.2 Waveshare Bus Servo Adapter (A) の接続

- 基板に **USB Type-C** があり、ジャンパ2個で切り替えます:
  **位置A = UART制御 / 位置B = USB制御（PC直結）** → 本プロジェクトは **位置B**。
  別途 USB-TTL 変換器は不要で、これ1枚で PC から直結できます（LeRobot SO-100/SO-101 と同じ標準構成）。
- 半二重（TX/RX 1本線）の方向切替は基板上で処理されるので、**PC側でエコーバックを読み捨てる処理は不要**です。
- USB 側はネイティブの USB CDC と見込んでいます（LeRobot のポート例が `ttyACM0` / `usbmodem` のため）。
  Windows では「USB シリアル デバイス (COMx)」として**標準ドライバで認識され、CH340 等のドライバ導入は不要**の見込み。
  （`ttyACM` は「CDC-ACM クラス」という意味なので、CDC 対応の変換チップの可能性もあります。
  どちらでも Windows 標準ドライバで動く点は同じです。）→ **実機で要確認**

### 2.3 USB シリアルの latency timer（CH340系の USB-TTL を使う場合のみ）

**Bus Servo Adapter (A) はネイティブ CDC なので、この設定は不要の見込みです（実機で要確認）。**
代わりに CH340 / FTDI 系の USB-TTL 変換器でサーボをつなぐ場合だけ、以下を行ってください。

これらのドライバは既定で受信データを最大 16ms 溜めてから PC に渡します。サーボは
「送信 → 応答待ち」を繰り返すので、1往復ごとに最大 16ms 待たされ、9軸 × 50Hz で
キーフレームを送ると**動きがカクつきます**。

手順（FTDI の場合）:
1. デバイスマネージャ →「ポート (COM と LPT)」→ 該当 COM ポートを右クリック →「プロパティ」
2. 「ポートの設定」タブ →「詳細設定」
3. 「待ち時間 (msec)」（英語表示では *Latency Timer (msec)*）を **16 → 1** に変更 → OK
4. **USB を抜き差し**して反映（または PC 再起動）

どの構成でも、実際の往復時間は STEP 8 の `tools/servo_setup.py` で測って確認します。

### 2.4 電源と配線

```
 12V 電源 (5A以上, 推奨10A以上)
    │  DCジャック or ネジ端子
    ▼
 ┌──────────────────────┐  USB-C (ジャンパ=B)
 │ Bus Servo Adapter (A)│──────────────────── PC (COMx)
 └──────────┬───────────┘
            │ 3線 (GND / 12V / 信号) デイジーチェーン
   J1 ─ J2 ─ J3 ─ J4 ─ J5 ─ [★] ─ J6 ─ J7 ─ J8 ─ J9
                              ↑
            必要ならここで 12V を分岐注入（GND 共通）
```

- STS3215 は 1 個あたりロック電流 2.7A（Waveshare Wiki の仕様値）。9 個が同時に踏ん張ると
  電源側は数十 A になり得ます。**律速になるのは電源容量と、デイジーチェーンのコネクタ・細い線の電流容量**です。
- 末端（頭側）で電圧が落ちる場合は、バスの中間で 12V を**分岐注入**してください（GND は必ず共通）。
- 電圧は `read_state()` の電圧で監視できます。展示中の電圧低下はログに残す予定です（STEP 7）。

XIAO ESP32S3（頭部）は別の USB で PC に接続します（115200 bps、行指向 ASCII。仕様は STEP 2 の `head_io.py` 参照）。

### 2.5 その他
- ユーザー名に日本語が含まれていても動作は確認済み（venv / pip / pytest）。
- **`cv2.imread` / `cv2.imwrite` は日本語を含むパスを開けません**（Windows 版 OpenCV の制限）。
  画像の読み書きは `np.fromfile` + `cv2.imdecode` か PIL を使うこと（`perception/camera.py` の ImageSource 参照）。
- COM ポートは同時に1プロセスしか開けません。Arduino IDE や別のシリアルモニタを閉じてから起動すること。

---

## 3. 実行方法（STEP ごとに更新）

### 動作確認用ツール

```powershell
# STEP 2: モックサーボの追従・負荷・温度変化をコンソールに表示
.\.venv\Scripts\python.exe tools\demo_mock_servo.py --heat-tau 60
# STEP 3: 9軸の角度列と、姿勢（home / とぐろ / 鎌首）の形を図にする → output/step3_motion.png
.\.venv\Scripts\python.exe tools\plot_motion.py
# STEP 4: シミュレータを上から見た図（+GIF）と、歩容ごとの「1周期あたりの前進量」
.\.venv\Scripts\python.exe tools\sim_view.py --gif
# STEP 5: 仮想カメラで知覚を試す（本物の ArUco 検出、人は仮想の検出器）→ GIF と時系列グラフ
.\.venv\Scripts\python.exe tools\perception_demo.py
# STEP 5: 実写（印刷したマーカ）。先に四隅をクリックして床の校正
.\.venv\Scripts\python.exe tools\make_aruco.py            # → output/aruco_markers_A4.png を実寸で印刷
.\.venv\Scripts\python.exe tools\calibrate_floor.py --source 0
.\.venv\Scripts\python.exe tools\perception_live.py --source 0
# STEP 6: 人の座標を動かして一連の振る舞いを見る → output/step6_behavior.gif と内部状態の時系列
.\.venv\Scripts\python.exe tools\behavior_demo.py
```

### 行動の設計（STEP 6）

毎制御周期（50Hz）の流れ（`behavior/brain.py`）:

```
知覚（ヘビの位置姿勢・追跡中の1人・タッチ・サーボ最高温度）
 → 刺激（presence / proximity / approach / touch / novelty / looking / alone）
 → 内部状態（Curiosity / Affection / Stress / Attention = 一次遅れ、Energy = サーボ温度から）
 → 効用（9状態、±8% の乱数）→ 状態機械（ヒステリシス 1.15 倍・最小継続 4 秒）
 → 状態ごとの動作（移動 controller.py / しぐさ expression.py）→ アニメーター → サーボ
```

- `internal_state.py` … dx/dt = −(x−x0)/τ + Σ gain×刺激。係数は `behavior.internal`。
  Energy は最高温度 `temp_fresh_c`（=1）〜`temp_tired_c`（=0）の直線。熱い = 疲れている
- `utility.py` … 手書きの効用（式は docstring）。係数は `behavior.utility.weights`。
  GUI の「いま何を考えているか」は `thought_line()`（例: `Curiosity 0.82 > Rest 0.31 → 接近`）
- `fsm.py` … 最小継続時間（状態ごとに上書き可: PETTED 2s / ALERT 2s / COIL_REST 20s / SLEEP 8s）、
  割り込みは PETTED だけ。ALERT は「新しい人に気づいた瞬間」に直接切り替える
- `controller.py` … 目標点 → γ0（向きの誤差 × ゲイン）と周波数（速さ ÷ 1周期の前進量）。
  周期は遅くするときだけ延ばす（速くするために短くしない）。1m 以内 8cm/s（**気づく前でも必ず**）、
  頭先端から 400mm で停止（歩容が止まるまでの惰性ぶん早めに止める）。
  マット端では後退しながら中央へ向き直る。マットの外の人へ近づくときは、人の方を向いて端に着いたら止まる
- `expression.py` … 生き物らしさ a〜j（数値はすべて `behavior.expression`）
- `sim/session.py` … 実機なしで全部をつないだもの（GUI の `--sim` もこれを使う予定）
- 目の色・明るさは状態ごと（`behavior.eyes`）。ESP32 のフェイルセーフ（3 秒）より短い間隔で送り直す

### 知覚の設計（STEP 5）

- `perception/homography.py` … マット四隅4点 → 画像↔床の射影変換（JSON 保存）。胴体上面のマーカ（高さ 50mm）は
  `camera.position_mm` を与えると視差補正する（高さ 1.5m・水平 1m で約 33mm のずれ）。広角レンズの歪みは取れない
- `perception/aruco_locator.py` … DICT_4X4_50 の ID0（尾）/ ID1（首）。小さいマーカ向けに検出パラメータを調整済み
  （`aruco.detector_params`）。2枚の距離が `aruco.max_marker_gap_mm` を超えたら誤検出として尾を捨てる
- `perception/snake_pose.py` … θ_body は1周期の移動平均（`snake_pose.heading_filter`）。一次ローパスは
  定常旋回で ω·τ（約 40°）遅れるため。移動平均の遅れは τ/2
- `perception/person_detector.py` … YOLO（CPU、person のみ）→ bbox 下辺中央を足元として床座標へ。
  追跡は「ヘビの首に最も近い1人」だけ、切替は 2 秒ヒステリシス、0人でも 3 秒保持、マット中心から 1.5m の外は無視
- `sim/virtual_camera.py` … カメラ無しでも知覚を通しで試せる仮想カメラ（マーカは本物の画像を貼って描く）
- **マーカの大きさ**: 40mm は 1280×720 だとマット奥で 11〜20px しかなく、仮想カメラでも検出率 93〜98%。
  1920×1080 なら 100%。**カメラは 1080p で使う**（`camera.width_px`）
- **カメラとマットの間に人が立つと、マーカが隠れる**（仮想カメラで確認）。カメラの置き場所に注意

### シミュレータの設計（STEP 4）

- `sim/world.py` … 車輪のあるリンク（`sim.wheel_links` = 尾端〜J7 の7リンク × 2輪 = 14輪）は横滑りしない、
  という拘束を全リンク分まとめて最小二乗で解き、胴体全体の剛体速度 (vx, vy, ω) を毎ステップ求める。物理エンジンなし。
  - 車輪の進行方向には転がり抵抗の重み `sim.tangential_drag_ratio`（推定値 0.02。**要実機校正**）
  - J7 より先は車輪なし。頭部が床にあるときは PTFE / フェルトのパッド（`sim.pad_links`、等方の軽い摩擦）
  - 床から浮いたリンク（鎌首の頭側）は拘束に入れない。マットからはみ出したら全体を内側へ押し戻す
- ヘビの位置姿勢（`perception/snake_pose.py`。ArUco でもシミュレータでも同じ処理）
  - 位置 = 首マーカ。**首マーカは J7 より胴体側（J6-J7 リンク上）に貼る**（J7 より先だと鎌首で傾いて見えない）。
    尾マーカは尾端-J1 リンク上。どちらも上面
  - θ_body = 尾マーカ → 首マーカ。蛇行で振れるので、時定数 = 歩容1周期のローパス後の値を移動制御に使う（生値も保持）
  - θ_head = θ_body + J8（「人を見ているか」の判定用）
- **1周期あたりの前進量（シミュレータ, 転がり抵抗 0.02）**: forward 約 384mm。
  実機では 200〜300mm/周期（周期2秒で 10〜15cm/s）と予想。**速度のために周期を短くしない**（静粛性優先）。
- 旋回は γ（オフセット）だけ。`gait.turn_profile: head_weighted`（γ(n) = γ0·n/N）が既定で、
  γ0 = ±20° で半径 約480mm。γ0 は1周期かけてランプする。
  - uniform と head_weighted を同じ旋回率で比べると半径は同じで、尾が頭の軌跡をなぞる精度が head_weighted の方が上
    （ずれ平均 91mm vs 115mm）。
- とぐろは**尾から順に巻き、頭を最後に引き込む**（`poses.coil_sequence`）。全関節同時より滑走が少ない。

### モーション層の設計（STEP 3）

- 関節角の符号: yaw + = 上から見て左、pitch + = 頭が上がる、roll + = 右に傾く（`motion/kinematics.py`）
- `motion/gait.py` … α(n,t) = A·sin(Ω·n + ω·t) + γ(n)。n=0 が J1（尾側）。ω>0 で前進、ω<0 で後退。
  旋回入力は γ だけ（振幅勾配は平面の蛇行では旋回しないことを確認して削除）。
- `motion/poses.py` … home（J7 = +8°：頭をわずかに浮かせる）/ coil / rear_up（土台 s_curve / partial_coil）/
  full_rear_up / head_look / relax。
  head_look で J7 を指定すると「人を見る」範囲（`neck.look_min_deg`〜`look_max_deg`）に制限される。
- `motion/animator.py` … 出力 = キーフレーム + 歩容（加算）+ 呼吸（加算）。
  キーフレームがその軸の `max_speed_dps`（胴体 240 / J7 120 / J8・J9 90 °/s）を超える速さを要求したら、
  自動で時間を延ばす。`order` と `stagger_s` で関節ごとに時間差をつけられる（とぐろを尾から順に）。
  `freeze()` で呼吸も含めて全停止し、`unfreeze()` で続きから再開する。

### ハードウェア層の設計（STEP 2）

- `serpens/hw/servo_bus.py` … 抽象クラス `ServoBus`。角度は**関節角 [deg]**（0° = まっすぐ）。
  ソフトリミット（config の `min_deg` / `max_deg`）でのクランプはここで共通に行う。
  機械リミット（`mech_min_deg` / `mech_max_deg`）は別に持ち、ソフトリミットがその内側かを起動時に確認する。
- `serpens/hw/mock_bus.py` … 一次遅れ追従。温度は一次系で、時定数 `mock_servo.heat_tau_s` を変えられる。
- `serpens/hw/feetech_bus.py` … 実機用。根拠は [docs/sts3215_registers.md](docs/sts3215_registers.md)。
  取り付け向き（`direction`）とホーン取付角オフセット（`horn_offset_deg`）は config で関節ごとに設定する。
- `serpens/hw/state_poller.py` … 位置と熱系（負荷・電圧・温度）を別頻度で読む。SYNC READ 非対応なら位置の頻度を落とす。
- `serpens/hw/head_io.py` … 頭部 XIAO ESP32S3。`SerialHeadIO`（自動再接続）と `MockHeadIO`（同じ行形式を生成）。
- 切り替えは `make_bus("mock" | "feetech", cfg, port)`。

### アプリ（STEP 7）

```powershell
# シミュレータのみ（実機なし）。GUI が開く
.\.venv\Scripts\python.exe -m serpens.app --sim
# 人の出入りを自動で再現（展示のリハーサル）
.\.venv\Scripts\python.exe -m serpens.app --sim --demo
# Webカメラで人検出 + シミュレータのヘビ
.\.venv\Scripts\python.exe -m serpens.app --sim --camera 0
# 実機（実機テストは未実施）
.\.venv\Scripts\python.exe -m serpens.app --bus feetech --port COM5
# GUI 無しで動作だけ確認 / GUI を GIF に記録
.\.venv\Scripts\python.exe -m serpens.app --sim --no-gui --seconds 10
.\.venv\Scripts\python.exe -m serpens.app --sim --demo --seconds 40 --record output\step7_gui.gif
```

キーボード

| キー | 意味 |
|---|---|
| **G** | 開始（待機 → 走行）。実機は開始条件を満たさないと動かない |
| **S** | **通常停止**。歩容を即時に止め、現在姿勢を保持（出力へ保持指令を送り続ける） |
| **E** | **緊急停止**（ラッチ）。通常操作・予約済みの演出では解除されない |
| **U** | 緊急停止の解除 → **待機**（走行は再開しない。再開は G） |
| **L** | 駆動無効化（脱力）。**停止中の明示操作のみ**。実機未検証 |
| **P** | シミュレーションの一時停止（実機では使わない） |
| **R** / **H** | ホーム復帰 / ホーム姿勢（**走行中のみ**。停止中は動かさない） |
| **D** / **C** / **T** / **Q** | デモの人 / 校正案内 / タッチ（モック） / 終了 |

### 停止の3種類（`serpens/safety.py`）

| 状態 | 意味 | 出力 | 解除 |
|---|---|---|---|
| RUN | 走行 | 行動が決めた指令 | — |
| HOLD | 通常停止 | 歩容を即時停止し、現在姿勢の保持指令を送り続ける（トルクは入れたまま） | G |
| DISABLED | 駆動無効化 | トルク OFF（脱力）。明示操作のみ。**実機未検証** | G |
| EMERGENCY | 緊急停止 | HOLD と同じ（config で DISABLED も選べる。**実機未検証**） | U → 待機 |

- **停止で勝手にホーム姿勢へ動かさない／全軸を無条件に脱力しない。** 停止中に動くのが最も危険
- 停止中も**監視は続ける**（温度・電圧・負荷を読み続ける）
- 停止時に**予約済みの演出（反応・見る・かしげる）を破棄**する。解除後に勝手に動き出さない
- 終了時は「停止を出力へ送る → スレッド join → 切断」を必ず通り、失敗はコンソールと GUI に出す。
  終了時にトルクを切るかは `behavior.safety.stop.disable_torque_on_exit`（既定 false = 保持したまま切断。**実機未検証**）

### ⚠ PC 側だけでは機体の安全は完成しない

`S` / `E` は **PC が生きている間だけ**有効。**PC の強制終了・USB 断・アプリのクラッシュでは、
サーボは最後に受けた目標角を保持し続ける**（Feetech サーボは自分で止まらない）。

**機体側の watchdog（heartbeat 途絶で ESP32 が単独で停止）が Phase 2 の必須項目。**
現状の `head_io` のフェイルセーフは目の表示を戻すだけで、駆動には効かない。

### 駆動リンク（Phase 2。PC ⇄ ESP32-S3）

仕様は [`docs/link_protocol.md`](docs/link_protocol.md)、確認結果は
[`docs/phase2_acceptance.md`](docs/phase2_acceptance.md)。

- PC が送るのは**歩容のパラメータ（10 バイトの `DRIVE`）だけ**。9軸の角度は ESP32 が作る
- **止まり方は二段構え。** `DRIVE` は期限（TTL 300ms）付きで、切れれば保持。
  `heartbeat` が 400ms 途絶すれば待機まで落ちる。**PC が死んでも USB が抜けても機体が自分で止まる**
- 緊急停止は**機体側でラッチ**。再接続でも `ARM` でも解除されず、`CLEAR_FAULT` は待機へ戻すだけ
- 上限（振幅・周波数・旋回・頭部角・速度）は**機体が持つ**。PC の設定では緩められない

```powershell
# 完了条件 1〜12 の確認 + 時間の実測（模擬機体。実機は不要）
.\.venv\Scripts\python.exe tools\link_check.py
# ファームと突き合わせる参照角度列 → data/gait_reference.csv
.\.venv\Scripts\python.exe tools\make_gait_reference.py
```

| 場所 | 中身 |
|---|---|
| `serpens/link/protocol.py` `messages.py` | フレームと payload（通信路を持たない） |
| `serpens/link/device.py` `device_motion.py` | **機体側の参照実装**（ファームはこれを写す） |
| `serpens/link/client.py` | PC 側。heartbeat・TTL・停止理由の履歴 |
| `serpens/link/transport.py` `harness.py` | 偽経路（USB 抜去・PC 強制終了・再起動・重複・CRC 破損）と足場 |
| `firmware/serpens_esp32/` | Arduino スケッチ。**未コンパイル・未書き込み**、配線が未確定 |

⚠ **まだ `serpens.app`（GUI・行動）はこのリンクを使っていない。接続は Phase 3。**

### 実機経路の接続（Phase 1 で直した部分）

```powershell
# サーボのみ（頭部 I/O 未接続 → ToF・タッチ・目は使えない。モックを実センサーとして使わない）
.\.venv\Scripts\python.exe -m serpens.app --bus feetech --port COM5
# 頭部も繋ぐ
.\.venv\Scripts\python.exe -m serpens.app --bus feetech --port COM5 --head-port COM6
```

- サーボバスと頭部 I/O は**セッションより先に作って注入**する。これで
  状態監視（`ServoStatePoller`）・位置指令（`Animator.send`）・トルク操作（`Expression`）が
  **同じ接続先**を参照する（以前は `session.bus` だけを差し替えていて、監視と脱力がモックに残っていた）
- **実機のロボット位置はまだ実観測ではない**（`world` の運動モデル由来）。
  そのため実機の自律走行は禁止したままで、開始条件として画面とコンソールに理由を出す。
  ArUco → World State の結線は Phase 4
- テレメトリは軸ごとに最終取得時刻を持ち、`behavior.safety.telemetry.stale_after_s` より古い軸は
  GUI で「—」になる。値の出どころ（MockServoBus / FeetechServoBus）も表示する

### GUI の設計（STEP 7）

- 4分割。左上 カメラ映像（`--sim` では仮想カメラ）、右上 俯瞰マップ、左下 内部状態、右下「いま何を考えているか」
- 文字は大きく（状態名 26pt / 本文 20pt）、細いグレー文字は使わない（`gui/style.py`）
- 右下の1行は効用の数値をそのまま出さず、日本語の文にする（`gui/wording.py`）
  - 例: 「好奇心 0.78 が 注意 0.38 を上回ったので、近づくことにした（人まで約 886mm、400mm 手前で止まる）」
  - 例: 「撫でられている。力を抜いて、じっとしている」「サーボが58℃。冷えるまで休む」
- **制御と描画を分ける**: 制御は専用スレッドで 50Hz（`serpens/runner.py`）、GUI は 10fps で最新値を読むだけ。
  Windows ではタイマ分解能を 1ms にし、制御スレッドの優先度を上げている
- **制御周期の実測**（30秒、この PC）

  | 条件 | 平均 | 最悪 | 20%以上の遅延 |
  |---|---|---|---|
  | GUI なし | 19.99ms | 20.78ms | 0 / 1505 |
  | GUI あり | 19.99ms | 58.1ms | 5 / 1512（0.3%） |
  | GUI あり + GIF 記録 | 20.01ms | 64.7ms | 135 / 2999（4.5%） |

  GUI ありで時々跳ねるのは Python の GIL（描画中は制御スレッドが待たされる）。
  サーボへは「目標角」を送るので、数周期の遅れは動きの途切れにはならない。GIF 記録は開発用で、展示では使わない。

---

## 4. 実機が届いたら最初にやること

1. **`tools/servo_setup.py` を回す**（実機が届いた日に最初に使うツール。モックでも練習できます）

   ```powershell
   .\.venv\Scripts\python.exe tools\servo_setup.py                      # モックで練習
   .\.venv\Scripts\python.exe tools\servo_setup.py --bus feetech --port COM5
   ```

   | メニュー | 内容 |
   |---|---|
   | 1 | ポートの一覧と自動検出（サーボ用と頭部用の2本を見分ける） |
   | 2 | サーボのスキャン（ID 範囲を指定。見つからないときの確認手順も表示） |
   | 3 | **入力電圧の設定（最優先）**。EEPROM の最高/最低入力電圧を読み、電源電圧より低ければ警告して書き換える |
   | 4 | ID の一括設定（1個ずつ繋いで 1→9 を順に振る） |
   | 5 | 温度・電圧・負荷のライブ表示（1Hz、Ctrl+C で戻る） |
   | 6 | 現在負荷の符号ビットの検証（仮説 bit10 を実機で確かめる） |
   | 7 | SYNC READ の対応判定（非対応なら個別 READ に落ちることを確認） |
   | 8 | 中立位置（2047）で保持してホーンを取り付ける |

   EEPROM への書き込みは「トルク OFF → ロック解除（55番地に 0）→ 書く → ロック（1）」の順に行います。
   トルク OFF が必須かどうかは資料に記載が無いので、安全側に倒して必ず切っています（実機で要確認）。

2. **Sim-to-Real 校正**: 平らな床で直進歩容（forward / slow）を N 周期歩かせ、首マーカの移動距離を測る。
   `data/real_runs.csv` に1行ずつ足して（床材ごとに）、次を回す:
   ```powershell
   .\.venv\Scripts\python.exe tools\fit_sim.py
   ```
   シミュレータの `sim.tangential_drag_ratio` を 0〜0.2 でスイープし、実測に最も合う値を出す（図: `output/fit_sim.png`）。
   その値を config に書けば、以後シミュレータの移動量が実機に合う。**サンプルのダミー行は消すこと。**
   - 例: 300mm/周期なら ratio ≈ 0.054。予想の 200〜300mm/周期は ratio 0.05〜0.10 に当たる
   - 最適値がスイープ範囲の端に張り付いたら、モデルが実機を説明できていない（車輪の滑り方が違う等）。相談すること

## 5. トラブルシュート

### サーボが PING に応答するのに、まったく動かない

**まず最高入力電圧（EEPROM 14番地）を疑ってください。** Waveshare のメモリテーブル（7.4V 版の表）では
初期値が **80 = 8.0V** です。12V 版のサーボでも同じ値のままだと、12V を入れた時点で過電圧になり、
一切動かない可能性があります（「実機を繋いだ初日に全部動かず数時間溶かす」タイプの罠）。

1. `tools/servo_setup.py`（STEP 8）で 13〜16 番地（最高温度・最高/最低入力電圧・最大トルク）と
   65 番地（サーボ状態。bit0 = 電圧エラー）を読む
2. 最高入力電圧が電源電圧より低ければ書き換える（EEPROM のロック解除 → 書き込み → ロック）
3. EEPROM 書き込み時にトルク OFF が必要な機種があるので、ツールは書き込み前にトルクを切る

### その他の実機確認事項（届いたら）
- 現在負荷の符号: 「下位10bit = 大きさ、bit10 = 方向」は**仮説**（`servo.load_sign_bit: 10`）。手で押して符号を確認する
- SYNC READ に応答するか（しなければ自動で個別 READ に切り替わり、位置の読み出しは 10Hz に落ちる）
- **TODO: 外皮ニットを着せたら可動範囲を再測定し、`joints[].mech_*` / `min_deg` / `max_deg` を更新する**
- サーボホーンの取付角を測り、`joints[].horn_offset_deg` に入れる

---

## 6. ディレクトリ構成

```
serpens/
  README.md  requirements.txt  requirements-nodeps.txt  pytest.ini
  config/robot.yaml          全パラメータ（寸法・しきい値・ゲイン）
  serpens/
    hw/          servo_bus.py(抽象) feetech_bus.py mock_bus.py head_io.py
    motion/      gait.py poses.py animator.py
    perception/  camera.py aruco_locator.py person_detector.py homography.py
    behavior/    internal_state.py fsm.py utility.py controller.py
    sim/         world.py
    gui/         app.py
    app.py  keys.py
  tools/         check_env.py make_aruco.py calibrate_floor.py servo_setup.py
  tests/
```

---

## 7. 進捗

| STEP | 内容 | 状態 |
|---|---|---|
| 1 | 環境構築 | ✅ |
| 2 | サーボ抽象層とモック / 頭部 I/O | ✅（実機テストは未実施） |
| 3 | 歩容エンジン・姿勢・アニメーター | ✅ |
| 4 | 2D シミュレータ | ✅ |
| 4.5 | Sim-to-Real 校正の器（fit_sim.py） | ✅ |
| 5 | 知覚（ArUco・ホモグラフィ・人の追跡） | ✅（YOLO は重みを置いてから確認） |
| 6 | 内部状態と行動 | ✅ |
| 7 | 展示用 GUI | ✅ |
| 8 | 実機用ツール（servo_setup.py） | ✅（モックで全メニュー確認。実機テストは未実施） |
