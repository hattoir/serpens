# PC ⇄ ESP32-S3 駆動リンク仕様 v1

Phase 2 の契約。**PC 側（`serpens/link/`）と ESP32 ファーム（`firmware/serpens_esp32/`）は
この1枚だけを根拠に実装する。** 頭部の ASCII 行プロトコル（`serpens/hw/head_io.py`）とは別物で、
そちらを駆動に流用しない。

作成 2026-09-13。実機検証: **未実施**（条件 2/3/13/14/15。`docs/phase2_acceptance.md` に記録欄あり）。
参照実装は `serpens/link/device.py`（機体側）と `serpens/link/client.py`（PC 側）。
ファーム `firmware/serpens_esp32/` は**この2つを写したもので、未コンパイル**。

## 0. 役割

| | ESP32-S3（機体側） | PC 側 |
|---|---|---|
| 周期 | 制御 100Hz / テレメトリ 10Hz | 指令 10〜20Hz、heartbeat 10Hz |
| 担当 | 歩容生成、9軸同期書き込み、頭部角の補間、呼吸、上限の強制、watchdog、緊急停止ラッチ、状態取得 | 知覚、行動判断、しぐさの時間設計、停止操作、記録、GUI |
| 入れない | LLM・人物認識・画像処理 | — |

**機体側は PC が死んでも自分で止まる。** これが Phase 2 の存在理由。

## 1. フレーム形式（バイナリ・リトルエンディアン）

```
0xA5 0x5A | ver(1) | type(1) | seq(2) | len(1) | payload(len) | crc16(2)
```

- `ver` = 1。異なる版は NACK(`BAD_VERSION`) で拒否し、処理しない。
- `seq` … PC → 機体のフレームに付ける単調増加値（65536 で巡回）。機体 → PC は 0 固定。
- `len` … payload のバイト数。**最大 128**（テレメトリ 9軸 = 93 バイトが最大のフレーム）。超える宣言は読み捨てて NACK(`BAD_LENGTH`)。
- `crc16` … CRC-16/CCITT-FALSE（初期値 0xFFFF、多項式 0x1021）。対象は `ver` から payload の末尾まで。
- 同期は SOF（0xA5 0x5A）の走査で取り直す。フレーム境界は SOF + len + CRC で判定する。
  途中で分割されて届いてもよい（受信は状態機械）。

### 受信の拒否条件（機体側）

| 条件 | 応答 |
|---|---|
| CRC 不一致 | NACK(`BAD_CRC`)。状態は変えない |
| 版違い | NACK(`BAD_VERSION`) |
| len > 64、または payload 長が type と合わない | NACK(`BAD_LENGTH`) |
| `seq` が**進んでいない**（重複・巻き戻り） | NACK(`STALE_SEQ`)。**指令は実行しない** |
| 値が上限外 | NACK(`OUT_OF_RANGE`)。**状態は変えない** |
| DISARMED なのに DRIVE | NACK(`DISARMED`) |
| EMERGENCY ラッチ中の ARM / DRIVE | NACK(`LATCHED`) |

`seq` の判定: `(seq - last_seq) mod 65536` が `1..4096` のときだけ「進んだ」とみなす。
再起動直後は `last_seq` を未設定にし、最初の有効フレームで初期化する。

## 2. 指令（PC → 機体）

| type | 名前 | payload | 意味 |
|---|---|---|---|
| 0x01 | `HEARTBEAT` | なし | 生存通知。**これで DRIVE の期限は延びない** |
| 0x02 | `ARM` | なし | 走行可能状態へ（DISARMED → ARMED）。ラッチ中・heartbeat 無しでは拒否 |
| 0x03 | `DISARM` | なし | ARMED → DISARMED（保持） |
| 0x10 | `DRIVE` | ttl_ms(u16), amp(i16 0.1°), spatial(i16 0.1°), freq(i16 0.001Hz 符号=前後), gamma(i16 0.1°) | 歩容パラメータ。ARMED のときだけ有効 |
| 0x11 | `HEAD` | ttl_ms(u16), j7(i16 0.1°), j8(i16 0.1°), j9(i16 0.1°), speed(u16 0.1°/s) | 頭部の目標角 |
| 0x12 | `POSE` | pose_id(u8) | 0=home / 1=coil / 2=relax |
| 0x13 | `BREATH` | on(u8) | 呼吸の ON/OFF（機体側で生成し続ける） |
| 0x20 | `STOP` | mode(u8: 0=hold, 1=disable), reason(u8) | **即時停止**。TTL に依存しない。ARMED → **DISARMED** |
| 0x21 | `EMERGENCY` | reason(u8) | 緊急停止。**機体側でラッチ** |
| 0x22 | `CLEAR_FAULT` | nonce(u32) | ラッチ解除 → **DISARMED**。同じ nonce の再利用は拒否 |
| 0x30 | `LIMITS` | 省略可（実装は Phase 9） | PC 設定の同期。機体の絶対上限は超えられない |
| 0x40 | `PING` | なし | ACK を返すだけ |

## 3. 機体 → PC

| type | 名前 | payload |
|---|---|---|
| 0x80 | `TELEMETRY` | boot_id(u16), uptime_ms(u32), state(u8), stop_reason(u8), last_seq(u16), flags(u8), n_axes(u8), 各軸 [pos(i16 0.1°), load(i16 0.001), temp(u8 ℃), volt(u8 0.1V), fault(u8), current(u16 mA, 0xFFFF=不明)] |
| 0x81 | `ACK` | ack_seq(u16), type(u8) |
| 0x82 | `NACK` | ack_seq(u16), type(u8), reason(u8) |
| 0x83 | `EVENT` | code(u8), detail(u8) — 停止理由・起動・ラッチの変化を通知（取りこぼしても TELEMETRY で分かる） |

`flags`: bit0 駆動中(driving) / bit1 呼吸中 / bit2 heartbeat 有効 / bit3 DRIVE 有効 / bit4 トルク ON。

## 4. 状態機械（機体側）

```
起動 ─▶ DISARMED ──ARM──▶ ARMED ──有効なDRIVE──▶ ARMED(driving)
          ▲  ▲                │                        │
          │  └──STOP / DISARM─┘                        │
          │                    DRIVE期限切れ・heartbeat途絶・上限違反 ─┘（保持）
          │
  CLEAR_FAULT
          │
     EMERGENCY ◀── EMERGENCY指令 / 機体の致命異常（過熱・過電流・サーボfault）
```

**不変条件**

1. **起動時は必ず DISARMED。** `boot_id` は起動ごとに変わる。PC は `boot_id` の変化を見て
   **自動で ARM しない**（条件 6）。
2. `DRIVE` は TTL 付き。期限切れで**保持へ**（`stop_reason=DRIVE_TTL`）。`ttl_disarm_ms` を超えて
   DRIVE が来なければ DISARMED（条件 4）。
3. heartbeat が `heartbeat_timeout_ms` 途絶 → 保持 + DISARMED（`HEARTBEAT_LOST`。条件 5）。
   **USB 断・PC 強制終了も同じ経路**（条件 2/3）。
4. `EMERGENCY` はラッチ。`ARM`・`DRIVE`・再接続・`STOP` では解除されない（条件 8/9）。
5. `CLEAR_FAULT` は **DISARMED へ戻すだけ**。走行は `ARM` + 有効な `DRIVE` が揃って初めて再開（条件 10）。
6. 上限外の値は機体が拒否し、状態も出力も変えない（条件 11）。
7. 停止理由は `stop_reason` として保持し、TELEMETRY と EVENT の両方で PC から読める（条件 12）。
8. 停止（保持）で**ホーム姿勢へ動かさない**。脱力は `STOP(mode=disable)` のときだけ。

### stop_reason

| 値 | 名前 | 意味 |
|---|---|---|
| 0 | `NONE` | 停止理由なし |
| 1 | `OPERATOR_STOP` | PC からの STOP |
| 2 | `DRIVE_TTL` | DRIVE の期限切れ |
| 3 | `HEARTBEAT_LOST` | heartbeat 途絶（USB 断・PC 停止を含む） |
| 4 | `EMERGENCY_CMD` | PC からの EMERGENCY |
| 5 | `OVERHEAT` | サーボ温度が上限超え |
| 6 | `OVERCURRENT` | 電流が上限超え（測れない機体では未使用） |
| 7 | `SERVO_FAULT` | サーボの fault ビット |
| 8 | `OUT_OF_RANGE` | 上限外の指令（拒否のみ。状態は変えない） |
| 9 | `BOOT` | 起動直後（未 ARM） |

## 5. 時間の既定値（`config/robot.yaml` の `link`）

| 項目 | 既定 | 根拠 |
|---|---|---|
| heartbeat 周期 | 100ms | 10Hz。テレメトリと同じ頻度 |
| heartbeat タイムアウト | 400ms | 3回の取りこぼしを許す。**要実測**（条件 13） |
| DRIVE の TTL 既定 | 300ms | PC の指令 10〜20Hz に対し 3〜6 周期ぶん。**要実測** |
| DRIVE 途絶 → DISARM | 2000ms | 保持のまま放置しない |
| テレメトリ | 10Hz | GUI と記録に十分 |
| 制御周期（機体） | 100Hz | 歩容生成・補間 |

**TTL（300ms）はわざと heartbeat タイムアウト（400ms）より短くしてある。** 通信が切れると
まず TTL が効いて保持に入り、続いて heartbeat 途絶が待機まで落とす。二段構えで、どちらが欠けても止まる。

**この表の数値は暫定。** 条件 13/14 の実測（通信断 → HOLD、緊急停止 → 停止出力）で確定する。

## 6. 検証の対応表

結果と根拠は **`docs/phase2_acceptance.md`**（自動試験 48 件 + `tools/link_check.py`）。

| 完了条件 | 検証方法 | 状態 |
|---|---|---|
| 1 歩容を機体が生成 | 機体の出力と `serpens/motion/gait.py` の式を突き合わせ（差 < 1e-9°）。実機は `data/gait_reference.csv` | ソフトで確認済み |
| 2 PC 強制終了 → HOLD | 偽経路で送信だけ止める | 模擬のみ |
| 3 USB 抜去 → HOLD | 偽経路で切断 | 模擬のみ |
| 4 DRIVE 停止・heartbeat 継続 | 自動試験 | 確認済み |
| 5 heartbeat 停止 | 自動試験 | 確認済み |
| 6 再起動で自動再開しない | `boot_id` 変化の自動試験 | 確認済み |
| 7 古い・重複・破損フレーム | `seq` / CRC の自動試験 | 確認済み |
| 8 EMERGENCY ラッチ | 自動試験 | 確認済み |
| 9 再接続で解除されない | 自動試験 | 確認済み |
| 10 CLEAR_FAULT → DISARMED のみ | 自動試験（nonce 再利用も） | 確認済み |
| 11 上限外を機体が拒否 | 自動試験（DRIVE 6 種 / HEAD 5 種） | 確認済み |
| 12 停止理由を PC から確認 | 6 種類を実際に起こして自動試験 | 確認済み |
| 13 通信断 → HOLD 実測 | `tools/link_check.py` | 模擬値のみ |
| 14 緊急停止 → 停止出力 実測 | 同上 | 模擬値のみ |
| 15 実サーボ | 実機 | **未実施** |
