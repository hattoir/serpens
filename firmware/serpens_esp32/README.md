# Serpens EX-1 駆動リンク ファームウェア（雛形）

PC が死んでも機体だけでヘビを止めるための基板側。仕様は [`docs/link_protocol.md`](../../docs/link_protocol.md) v1。

## 状態: **コンパイル可・未書き込み・実機未検証**

2026-09-16: `arduino-cli` で**コンパイルが通ることを確認**（`SOFTWARE_VERIFIED`）。

```bash
.venv/Scripts/python.exe tools/build_firmware.py
```

結果（esp32 core 2.0.17 / FQBN `esp32:esp32:XIAO_ESP32S3`）:
フラッシュ **255,809 バイト（7%）** / RAM **18,632 バイト（5%）**。

**書き込みは一度もしていない。** 実機（ESP32-S3 と STS3215）が手元に無いため、
振る舞いの正しさは Python の参照実装（`serpens/link/device.py`）で検証してある
（`tests/test_phase2_device.py` / `tests/test_phase2_safety.py`、48 件）。
このスケッチは**その参照実装を同じ判断順で C++ に写したもの**で、実機が来たら
`tools/link_check.py` の手順で条件 1〜15 を測り直す。

| ファイル | 中身 |
|---|---|
| `link.h` | フレーム・CRC・enum（`serpens/link/protocol.py` の写し） |
| `config.h` | 時定数・上限・関節（`config/robot.yaml` の写し） |
| `serpens_esp32.ino` | 状態機械（7状態）・watchdog・歩容生成・テレメトリ v2 |

コンパイル時にも大きさを検査している（`link.h` の `static_assert`）:
テレメトリ 9軸 = 26 + 9×11 = **125 バイト ≤ MAX_PAYLOAD 192**。

**三つ（robot.yaml / protocol.py / この二つのヘッダ）は同じ値を持つ。片方だけ直さないこと。**

## 決まっていないこと（勝手に決めない）

1. **駆動リンク用の基板**を、頭部センサ用の XIAO ESP32S3 と兼用にするか、別基板にするか。
   兼用なら頭部の ASCII 行プロトコル（`serpens/hw/head_io.py`）と同じ USB に同居することになる。
2. **サーボの半二重 TTL をどう作るか。** Waveshare Bus Servo Adapter (A) を挟むのか、
   方向制御ピン付きのトランシーバを基板に載せるのか。`config.h` の `SERVO_*_PIN` は `-1`（未定）。
3. 電源の取り方（12V → 基板の 5V/3.3V）。

これが決まるまで **書き込まないこと**。`writeServos()` と温度・fault の読み出しは空のままにしてある。

## 未実装（TODO）

- `writeServos()`: 9軸の同期書き込み（`docs/sts3215_registers.md` の 41〜47 ブロック）
- サーボの状態読み出し（56〜63 ブロック）→ テレメトリの位置・負荷・温度・電圧・fault
- 過熱・過電流の実測値による緊急停止（いまは読み値が 0 なので発火しない）
- 起動時に現在角を読んでから `gGoal` に入れる（いまはホーム姿勢を仮定している。
  **実機でここを直さないと、電源投入時に現在角からホームへ飛ぶ危険がある**）
- とぐろ（`POSE` の id 1 以降）。いまは home のみ

## ビルド（実機が来たら）

- ボード: XIAO ESP32S3 または ESP32-S3 DevKit
- USB CDC On Boot: **Enabled**（`Serial` がネイティブ USB になる）
- ライブラリ: サーボは `SCServo` 系または `ftservo` 系（PC 側の `ftservo-python-sdk` と同じ系統）

## 実機で最初に確かめること（順番）

1. `PING` に ACK が返る。`TELEMETRY` が 10Hz で届く（`tools/link_check.py --port COMx`）
2. **サーボを繋がない状態**で、`ARM` → `DRIVE` → `boot_id`・`stop_reason` が仕様どおり動く
3. USB を抜く → 出力が止まる（サーボ未接続でも、テレメトリの `flags` から分かる）
4. サーボ1個だけ繋いで、ホーム姿勢の保持と `STOP(mode=disable)` の脱力を確認
5. 9軸を繋いで `data/gait_reference.csv` と出力を突き合わせる（条件 1）
6. 条件 13/14/15 の実測を `docs/phase2_acceptance.md` に記録する
