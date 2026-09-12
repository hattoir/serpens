# Phase 2 完了条件 — 確認結果

対象: PC ⇄ ESP32-S3 駆動リンク（`docs/link_protocol.md` v1）
確認日 2026-09-13 / 確認方法: 偽時計 + 偽経路 + シミュレート ESP32（`serpens/link/device.py`）

**実機は1台も無い。** サーボも ESP32 も未入手のため、条件 2/3/13/14 は**模擬での値**、
条件 15 は**未実施**。実機が来たら `tools/link_check.py` で測り直し、この表を更新する。

再現手順:

```bash
.venv\Scripts\python.exe -m pytest tests/test_phase2_frame.py tests/test_phase2_device.py tests/test_phase2_safety.py -q
.venv\Scripts\python.exe tools\link_check.py --out docs\phase2_measured.md
```

## 1. 条件ごとの結果

| # | 条件 | 状態 | 根拠 |
|---|---|---|---|
| 1 | PC が DRIVE を送り、ESP32 が歩容を生成 | ソフトで確認 | `test_device_generates_gait_matching_reference`: 機体の出力が `serpens/motion/gait.py` の式と 2 秒間で最大 1e-9° しか違わない。PC が送るのは 10 バイトの DRIVE のみ |
| 2 | PC のプロセスを殺す → 単独で HOLD | **模擬のみ** | `test_pc_process_killed_leads_to_hold`: 送信が止まった 210〜300ms 後に保持（DRIVE TTL）、400ms で待機（heartbeat 途絶）。以後 1 秒間の移動 0.0000° |
| 3 | USB を抜く → HOLD | **模擬のみ** | `test_usb_unplug_leads_to_hold`: 同じ経路。PC 側もテレメトリが古くなることで検知 |
| 4 | DRIVE だけ止める（heartbeat 継続）→ 停止 | 確認済み | `test_drive_ttl_stops_motion_while_heartbeat_alive`: 最後の DRIVE から 300ms（TTL）で保持。ARMED のまま、2000ms 放置で待機へ |
| 5 | heartbeat だけ止める → 停止 | 確認済み | `test_heartbeat_loss_stops_even_while_drive_arrives`: DRIVE が届き続けていても 310〜400ms で停止 |
| 6 | ESP32 再起動で自動再開しない | 確認済み | `test_device_reboot_starts_disarmed`: `boot_id` が変わり DISARMED。PC が heartbeat と DRIVE を送り続けても 3 秒間走り出さない。PC 側も `rebooted` を立てて DRIVE 送信を止める |
| 7 | 古い・重複パケットで再開しない | 確認済み | `test_replayed_drive_is_rejected`（録音した DRIVE の再送 → NACK STALE_SEQ）、`test_duplicate_frames_do_not_double_apply`、`test_corrupted_frames_are_rejected`（NACK BAD_CRC） |
| 8 | EMERGENCY を機体側でラッチ | 確認済み | `test_emergency_latches_on_device`: ARM も DRIVE も NACK LATCHED。`test_device_faults_latch_as_emergency`: 過熱は冷えても自動復帰しない |
| 9 | PC 再接続で解除されない | 確認済み | `test_emergency_survives_reconnect`: USB 抜き差し + PC 側オブジェクト再生成でも EMERGENCY のまま |
| 10 | CLEAR_FAULT は DISARMED まで | 確認済み | `test_clear_fault_returns_to_disarmed_only`: 解除後 DRIVE だけでは動かず、ARM + DRIVE で再開。`test_clear_fault_nonce_cannot_be_replayed`: 同じ nonce の再送は NACK NONCE_REUSED |
| 11 | 上限外の角度・速度・歩容値を機体が拒否 | 確認済み | `test_device_rejects_out_of_range_drive`（6 種）、`test_device_rejects_out_of_range_head`（5 種）: いずれも NACK OUT_OF_RANGE、状態も出力も変わらない |
| 12 | 停止理由が PC から確認できる | 確認済み | `test_all_stop_reasons_are_visible_from_pc`: 6 種類（PC 停止 / TTL / heartbeat 途絶 / 緊急停止 / 過熱 / サーボ異常）を実際に起こし、テレメトリの日本語と履歴で確認 |
| 13 | 通信断 → HOLD の実測 | **模擬値のみ** | 下表 |
| 14 | 緊急停止 → 停止出力の実測 | **模擬値のみ** | 下表 |
| 15 | 実サーボでの確認 | **未実施** | 実機が無い。記録欄は §3 |

## 2. 時間（`tools/link_check.py`、20 回）

制御 100Hz / heartbeat 10Hz / DRIVE TTL 300ms / heartbeat タイムアウト 400ms のとき:

| 測定 | 平均 | 最小 | 最大 | 実機 |
|---|---|---|---|---|
| 13 通信断（USB 抜去）→ 保持 | 255 ms | 210 ms | 300 ms | 未測定 |
| 13b heartbeat のみ途絶 → 停止 | 355 ms | 310 ms | 400 ms | 未測定 |
| 14 緊急停止の指令 → 出力停止 | 10 ms | 10 ms | 10 ms | 未測定 |

読み方:

- **止まり方は二段構え。** 通信が切れると、まず速い方の **DRIVE TTL（300ms）** が効いて保持に入り、
  続いて **heartbeat タイムアウト（400ms）** が待機まで落とす。どちらが欠けても止まる。
- 条件 14 の 10ms は模擬の値（経路 1 回 + 制御 1 周期）。**実機ではここに USB CDC の往復と
  9軸同期書き込み（1Mbps）の時間が乗る。** 実測すること。
- この 3 つの数値はすべて `config/robot.yaml` の `link` 節から来ている。実測後にそこを直す。

## 3. 実機での記録欄（条件 15。**未記入**）

実機が揃ったら、`firmware/serpens_esp32/README.md` の「最初に確かめること」の順に行い、ここへ書く。

| 項目 | 測定日 | 条件 | 結果 | 備考 |
|---|---|---|---|---|
| サーボ 1 個での保持・脱力 | | ID / 電圧 / 温度 | | |
| 9軸同期書き込みの所要時間 | | 1Mbps / 9軸 | | 制御周期 10ms に収まるか |
| 歩容の一致（条件 1） | | `data/gait_reference.csv` と比較 | | 最大差 [deg] |
| USB 抜去 → 出力停止（条件 13） | | | | オシロまたはサーボ読み値で |
| PC 強制終了 → 出力停止（条件 2） | | | | |
| 緊急停止 → 出力停止（条件 14） | | | | |
| 連続運転での温度（`link.faults.temp_limit_c`） | | 環境温度 / 分 | | 展示は数時間連続 |
| 12V 入力での動作（最高入力電圧の初期値 8.0V 問題） | | `tools/servo_setup.py` で確認済みか | | docs/sts3215_registers.md |

## 4. 残る制約

- **このファームはコンパイルしていない。** 実機が来たらまずビルドを通すところから。
- ESP32 とサーボバスの**配線が未確定**（`firmware/serpens_esp32/README.md` の「決まっていないこと」）。
- 機体側の過熱・過電流検知は、サーボ読み出しが未実装のため**現状は発火しない**（枠だけある）。
- PC 側の `serpens/app.py`（GUI・行動）はまだこのリンクを使っていない。**接続は Phase 3。**
  いまは `serpens/safety.py` の停止と、この駆動リンクが別々に存在している。
- `POSE` は home のみ。とぐろは列で作るので PC 側に残す。
