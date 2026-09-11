# STS3215 レジスタと通信仕様（一次情報のまとめ）

`serpens/hw/feetech_bus.py` はこの表だけを根拠に書いています。
表にない仕様を使うときは、まずこのファイルに出典つきで追記してください。

## 出典

| 記号 | 資料 | URL | 確認日 |
|---|---|---|---|
| [W] | Waveshare Wiki「ST3215 Servo」本文 | https://www.waveshare.com/wiki/ST3215_Servo | 2026-09-12 |
| [M] | ST3215 memory register map-EN.xls（Sheet2 英語版 / Sheet1 中国語版、「STS3215 内存表参数解析 V3.7」） | https://files.waveshare.com/upload/2/27/ST3215%20memory%20register%20map-EN.xls | 2026-09-12 |
| [P] | Communication Protocol User Manual (EN, 191218-0923) | https://files.waveshare.com/upload/2/27/Communication_Protocol_User_Manual-EN%28191218-0923%29.pdf | 2026-09-12 |
| [S] | ftservo-python-sdk 2.0.0 `scservo_sdk/sms_sts.py`（Feetech 公式リポジトリ由来） | https://github.com/ftservo/FTServo_Python | 2026-09-12 |

## 1. 基本パラメータ

| 項目 | 値 | 出典 | メモ |
|---|---|---|---|
| 分解能 | 4096 step / 360°（0.087890625°/step） | [W][M] | |
| 中立位置 | 2047 | [W] | [M] のトルクスイッチ説明では「128 を書くと現在位置を **2048** に校正」。1 step（0.09°）の差。本プロジェクトは config で 2047 |
| 位置範囲（位置サーボモード） | 0〜4095 | [W][M] | 最小/最大角度制限の初期値 0 / 4095 |
| 速度の単位 | step/s（50 step/s ≒ 0.732 rpm） | [W][M] | |
| 速度の上限 | 3400 step/s（無負荷速度の欄） | [M] | [W] の GUI 説明では「約 3073」。上限は config で持つ |
| 加速度の単位 | 100 step/s²（レジスタ値 10 → 1000 step/s²） | [M] | 範囲 0〜254 |
| 通信速度 | 1,000,000 bps（Baudrate レジスタ 0 = 1M） | [W][M] | 8N1（1 start, 8 data, 1 stop, パリティなし）[P] |
| 工場出荷 ID | 1 | [W] | 同じバスに同じ ID を2個つながない |

## 2. バイト順

**リトルエンディアン（下位バイトが先の番地）。** [M] の表見出しに明記。
[P] §1.0 も「磁気エンコーダ系は下位バイトが先、ポテンショメータ系は上位バイトが先」と説明し、
[P] Example 2 では現在位置の読み出し結果 `18 05` を 0x0518 = 1304 と解釈している。
SDK [S] の `sms_sts` は `protocol_end = 0`（= 下位バイトが先）で初期化される。

## 3. パケット [P]

```
指示:  FF FF | ID | LEN | INST | PARAM1 … PARAMn | CHK
応答:  FF FF | ID | LEN | ERR  | PARAM1 … PARAMn | CHK
LEN = パラメータ数 + 2
CHK = ~(ID + LEN + INST(またはERR) + ΣPARAM) の下位 1 バイト
```

| 命令 | 値 | 用途 |
|---|---|---|
| PING | 0x01 | 生存確認 |
| READ | 0x02 | param = 先頭番地, 長さ |
| WRITE | 0x03 | param = 先頭番地, データ… |
| REG WRITE / ACTION | 0x04 / 0x05 | 書き込みを予約して一斉実行 |
| SYNC READ | 0x82 | ID=0xFE。**[P] に「一部のサーボのみ対応」とある** → STS3215 での対応は実機で確認 |
| SYNC WRITE | 0x83 | ID=0xFE。応答なし |
| RESET | 0x06 | 工場出荷値に戻す（使わない） |

ブロードキャスト ID 254（0xFE）には応答が返らない（PING は例外だが、複数台接続時は使えない）。

## 4. メモリテーブル（本プロジェクトで使う番地）[M]、SDK 定数名 [S]

| 番地 | 名前 | バイト | 領域 | R/W | 単位・意味 | SDK 定数 |
|---|---|---|---|---|---|---|
| 3–4 | サーボ型番（主/副） | 1+1 | EPROM | R | `ping()` がここを読んで型番を返す | `SMS_STS_MODEL_L/H` |
| 5 | ID | 1 | EPROM | RW | 0〜253 | `SMS_STS_ID` |
| 6 | ボーレート | 1 | EPROM | RW | 0=1M, 1=500k, 2=250k, 3=128k, 4=115200, 5=76800, 6=57600, 7=38400 | `SMS_STS_BAUD_RATE` |
| 9 | 最小角度制限 | 2 | EPROM | RW | step（初期値 0） | `SMS_STS_MIN_ANGLE_LIMIT_L` |
| 11 | 最大角度制限 | 2 | EPROM | RW | step（初期値 4095） | `SMS_STS_MAX_ANGLE_LIMIT_L` |
| 13 | 最高温度 | 1 | EPROM | RW | °C（初期値 70） | （なし） |
| 14 | 最高入力電圧 | 1 | EPROM | RW | 0.1V（**初期値 80 = 8.0V**、下の注意参照） | （なし） |
| 15 | 最低入力電圧 | 1 | EPROM | RW | 0.1V（初期値 40 = 4.0V） | （なし） |
| 16 | 最大トルク | 2 | EPROM | RW | 1000 = ストールトルク 100%。電源投入時に 48 番地へコピーされる | （なし） |
| 31 | 位置補正 | 2 | EPROM | RW | step。**BIT11 が符号ビット** | `SMS_STS_OFS_L` |
| 33 | 動作モード | 1 | EPROM | RW | 0=位置サーボ, 1=定速, 2=PWM, 3=ステップ | `SMS_STS_MODE` |
| 40 | トルクスイッチ | 1 | SRAM | RW | 0=OFF, 1=ON, 128=現在位置を 2048 に校正 | `SMS_STS_TORQUE_ENABLE` |
| 41 | 加速度 | 1 | SRAM | RW | 100 step/s² | `SMS_STS_ACC` |
| 42 | 目標位置 | 2 | SRAM | RW | step | `SMS_STS_GOAL_POSITION_L` |
| 44 | 運転時間 | 2 | SRAM | RW | PWM モード用。位置モードでは 0 を書く | `SMS_STS_GOAL_TIME_L` |
| 46 | 運転速度 | 2 | SRAM | RW | step/s（0〜3400） | `SMS_STS_GOAL_SPEED_L` |
| 48 | トルク制限 | 2 | SRAM | RW | 0〜1000（初期値 = 16 番地の値）。脱力演出に使う | （なし → 自前定義） |
| 55 | ロックフラグ | 1 | SRAM | RW | 0 = EPROM 書き込みを保存する / 1 = 保存しない | `SMS_STS_LOCK` |
| 56 | 現在位置 | 2 | SRAM | R | step | `SMS_STS_PRESENT_POSITION_L` |
| 58 | 現在速度 | 2 | SRAM | R | step/s | `SMS_STS_PRESENT_SPEED_L` |
| 60 | 現在負荷 | 2 | SRAM | R | 0.001（モータ駆動電圧のデューティ比） | `SMS_STS_PRESENT_LOAD_L` |
| 62 | 現在電圧 | 1 | SRAM | R | 0.1V | `SMS_STS_PRESENT_VOLTAGE` |
| 63 | 現在温度 | 1 | SRAM | R | °C | `SMS_STS_PRESENT_TEMPERATURE` |
| 65 | サーボ状態 | 1 | SRAM | R | bit0 電圧, bit1 センサ, bit2 温度, bit3 電流, bit4 角度, bit5 過負荷 | （なし） |
| 66 | 移動中フラグ | 1 | SRAM | R | 1 = 動作中 | `SMS_STS_MOVING` |
| 69 | 現在電流 | 2 | SRAM | R | 6.5mA | `SMS_STS_PRESENT_CURRENT_L` |

### まとめて読み書きする範囲

- **書き込み**: 41〜47 の 7 バイト = `[加速度, 位置L, 位置H, 時間L, 時間H, 速度L, 速度H]`。
  SDK の `WritePosEx()` / `SyncWritePosEx()` がこの並びで書く。[P] Example 4 と 7 も同じ番地・並び。
- **読み出し**: 56〜63 の 8 バイト = `[位置L, 位置H, 速度L, 速度H, 負荷L, 負荷H, 電圧, 温度]`。
  [P] Example 8（SYNC READ で 0x38 から 8 バイト）と同じ範囲。

## 5. 符号の扱い（負の値）

| 値 | 表現 | 根拠 |
|---|---|---|
| 位置補正（31） | BIT11 が符号、残りが絶対値 | [M] |
| 動作モード 1 / 3 の速度・ステップ数 | BIT15 が符号 | [M] |
| 現在位置・現在速度（56, 58） | BIT15 を符号とする符号＋絶対値 | [S] `ReadPos` / `ReadSpeed` が `scs_tohost(x, 15)`（公式表には記載なし） |
| **現在負荷（60）** | **公式資料・SDK のどちらにも記載なし** | → 符号ビット不明。config `load_sign_bit: null` の間は生値 ÷ 1000 をそのまま返す |

位置サーボモード（0〜4095）で使う限り、目標位置に負の値は書きません。

## 6. 注意点・未確認事項（実機で確認すること）

1. **最高入力電圧の初期値が 80（8.0V）**。[M] は試験電圧 7.4V の表で、12V 版の STS3215 で
   初期値が同じかは確認できていない。12V で使って電圧エラー（状態 bit0）が出る場合は 14 番地を確認。
   なお初期値では、電圧エラーは「LED 点滅」の条件には入っているが「脱力」の条件（19 番地 = 44 = bit2,3,5）には入っていない。
   → `tools/servo_setup.py`（STEP 8）で 13〜16 番地を読み出して表示する。
2. トルク制限（48）の単位が、[M] Sheet2 では「0.01」、最大トルク（16）では「0.001」と食い違っている。
   範囲がどちらも 0〜1000 なので 1000 = 100% と解釈した（Sheet1 中国語版も同じ記載）。
3. 加速度 0、速度 0 を書いたときの意味は資料に記載なし。本プロジェクトでは config の最小値（1）に丸め、0 は書かない。
4. SYNC READ の対応は [P] で「一部サーボのみ」。非対応だった場合は個別 READ に切り替える実装にしてある。
5. 現在負荷の符号ビット（§5）。
