# Serpens EX-1 — 展示用ヘビ型ロボット 制御ソフトウェア

9軸（胴体ヨー×6 + 首ピッチ + 頭ヨー + 頭ピッチ）のヘビ型ロボットを、Python だけで動かすための
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

### サーボ SDK について

`ftservo-python-sdk`（Feetech 公式リポジトリ由来、import 名 `scservo_sdk`）を使います。
`WritePosEx()`（位置・速度・加速度の同時指定）、`read1ByteTxRx()` / `read2ByteTxRx()`、
Sync Read / Sync Write が揃っています。
よく似た名前の `feetech-servo-sdk` は機能削減版（`sms_sts` が無い）で、しかも同じ `scservo_sdk` という
名前でインストールされて上書きし合うので、**入れないでください**（`tools/check_env.py` が検出します）。
なお `scservo-sdk` という名前のパッケージは PyPI にありません。
万一 `opencv-python` が入ってしまったら:

```powershell
.\.venv\Scripts\python.exe -m pip uninstall -y opencv-python opencv-python-headless opencv-contrib-python
.\.venv\Scripts\python.exe -m pip install opencv-contrib-python
```

ultralytics は実行時に足りないパッケージを勝手に pip install する機能があります。
本プロジェクトではこれを環境変数 `YOLO_AUTOINSTALL=false` で無効化してから import します（STEP 5 で実装）。

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
- COM ポートは同時に1プロセスしか開けません。Arduino IDE や別のシリアルモニタを閉じてから起動すること。

---

## 3. 実行方法（STEP ごとに更新）

### 動作確認用ツール

```powershell
# STEP 2: モックサーボの追従・負荷・温度変化をコンソールに表示
.\.venv\Scripts\python.exe tools\demo_mock_servo.py --heat-tau 60
```

### ハードウェア層の設計（STEP 2）

- `serpens/hw/servo_bus.py` … 抽象クラス `ServoBus`。角度は**関節角 [deg]**（0° = まっすぐ）。
  ソフトウェアリミット（config の `min_deg` / `max_deg`）でのクランプはここで共通に行う。
- `serpens/hw/mock_bus.py` … 一次遅れ追従。温度は一次系で、時定数 `mock_servo.heat_tau_s` を変えられる。
- `serpens/hw/feetech_bus.py` … 実機用。根拠は [docs/sts3215_registers.md](docs/sts3215_registers.md)。
  取り付け向き（`direction`）と組立オフセット（`offset_deg`）は config で関節ごとに設定する。
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

## 4. ディレクトリ構成

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

## 5. 進捗

| STEP | 内容 | 状態 |
|---|---|---|
| 1 | 環境構築 | ✅ |
| 2 | サーボ抽象層とモック / 頭部 I/O | ✅（実機テストは未実施） |
| 3 | 歩容エンジン | – |
| 4 | 2D シミュレータ | – |
| 5 | 知覚 | – |
| 6 | 内部状態と行動 | – |
| 7 | GUI | – |
| 8 | 実機用ツール | – |
