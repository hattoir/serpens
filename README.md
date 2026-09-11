# Serpens EX-1 — 展示用ヘビ型ロボット 制御ソフトウェア

9軸（胴体ヨー×6 + 首ピッチ J7 + 頭ヨー J8 + 頭ロール J9）のヘビ型ロボットを、Python だけで動かすための
ソフトウェアです。**モックファースト**で作っており、実機が1台もなくてもシミュレータ上で全機能が動きます。

- ROS 2 不使用 / LLM 不使用（行動選択は手書きの効用関数）
- 実機への切り替えはコマンドライン引数だけ: `--bus feetech --port COM5`
- 寸法・しきい値・ゲインは全部 `config/robot.yaml`（コードにマジックナンバーを書かない）

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

ultralytics は、足りないパッケージの pip install や重みのダウンロードを勝手に行う機能があります。
本プロジェクトは `YOLO_AUTOINSTALL=false` / `YOLO_OFFLINE=true` にしてから import するので、**重みは手で置きます**。

1. ultralytics の公式リリース（GitHub `ultralytics/assets` の Releases）から `yolo11n.pt`（約 5MB）を取得
2. `models/yolo11n.pt` に置く（場所は config の `person.model_path`。`models/` と `*.pt` は git に入れない）
3. 動作確認（同梱のサンプル画像）:
   `.\.venv\Scripts\python.exe tools\perception_live.py --source .venv\Lib\site-packages\ultralytics\assets\bus.jpg --homography output\_test_h.json --frames 1 --no-window`

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
```

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

### アプリ（予定）

```powershell
# シミュレータのみ（実機なし）
.\.venv\Scripts\python.exe -m serpens.app --sim
# Webカメラで人検出 + シミュレータのヘビ
.\.venv\Scripts\python.exe -m serpens.app --camera 0 --sim-robot
# 実機
.\.venv\Scripts\python.exe -m serpens.app --bus feetech --port COM5
```

---

## 4. 実機が届いたら最初にやること

1. `tools/servo_setup.py`（STEP 8）で全サーボを PING、**最高入力電圧（14番地）を確認**（下のトラブルシュート参照）
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
| 6 | 内部状態と行動 | – |
| 7 | GUI | – |
| 8 | 実機用ツール | – |
