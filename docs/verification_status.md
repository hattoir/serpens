# 検証レベル — 何が確かめられていて、何が確かめられていないか

作成 2026-09-15。**このリポジトリで「確認済み」と書くときは、必ずこの4段のどれかを指す。**
実機が 1 台も無い状態で開発しているので、**模擬の結果を実測として扱わない**ことが最優先。

| レベル | 意味 | 根拠になるもの |
|---|---|---|
| `SIMULATED` | シミュレーション上で**そう動いた**。現実がそうなるとは限らない | 模擬機体・模擬サーボ・偽経路での実行結果 |
| `SOFTWARE_VERIFIED` | **ソフトの論理として正しい**ことを自動試験で確かめた（物理は含まない） | `pytest`。式の一致・状態遷移・拒否条件・値の整合 |
| `HARDWARE_UNVERIFIED` | 実機が要るが、**まだ確かめていない**。設計値・推定値のまま | 一次資料からの計算、データシート、CAD 検討 |
| `HARDWARE_VERIFIED` | **実機で測った**。日付・条件・測定器つき | 実測記録（`docs/phase2_acceptance.md` §3 など） |

現在 `HARDWARE_VERIFIED` は **0 件**。実サーボも ESP32 も 1 台も無い。

## 1. 駆動リンクと安全

| 項目 | レベル | 根拠 |
|---|---|---|
| フレーム（CRC・分割・混入・seq） | `SOFTWARE_VERIFIED` | `tests/test_phase2_frame.py`（8） |
| 7状態の状態機械 | `SOFTWARE_VERIFIED` | `tests/test_phase2_device.py` / `test_phase2_safety.py` |
| PC 強制終了・USB 抜去で機体が止まる | `SIMULATED` | 偽経路で再現。**実機の停止時間は未測定** |
| 通信断 → 保持まで 210〜300ms | `SIMULATED` | `tools/link_check.py`。実機ではシリアル往復ぶん増える |
| 緊急停止 → 出力停止 10ms | `SIMULATED` | 同上。**実機では 9軸同期書き込みの時間が乗る** |
| 上限外の角度・速度・歩容値の拒否 | `SOFTWARE_VERIFIED` | `tests/test_phase2_device.py`（11 種） |
| NaN / Inf / 桁あふれを送らない | `SOFTWARE_VERIFIED` | `tests/test_phase2_faults.py` |
| packet loss / delay / 順序入れ替え / 重複 / CRC 破損 | `SIMULATED` | `tests/test_phase2_faults.py`（17） |
| サーボ応答なし → FAULT_HOLD | `SIMULATED` | 模擬サーボを offline にして確認 |
| 過熱・fault ビットでラッチ | `SIMULATED` | 模擬サーボの値。**実機の温度挙動は未測定** |
| ファームと Python の値の一致 | `SOFTWARE_VERIFIED` | `tests/test_firmware_sync.py`（11） |
| ファームのビルド | `SOFTWARE_VERIFIED` | `tools/build_firmware.py`（arduino-cli + esp32 core 2.0.17, XIAO_ESP32S3）。フラッシュ 7% / RAM 5%。**書き込みはしていない** |

## 2. 機構・サーボ

| 項目 | レベル | 根拠 |
|---|---|---|
| 可動域 geometry ±90° | `HARDWARE_UNVERIFIED` | 設計値 |
| 可動域 mechanical ±64°（±65° で Belly Shell 干渉） | `HARDWARE_UNVERIFIED` | CAD `Serpens_BELLY_R03_TWO_LINK_REVIEW` の検討結果。**実物での確認は未** |
| operational ±60°（暫定） | `HARDWARE_UNVERIFIED` | 上記から呼吸 ±2° ぶんの余裕を引いた値。最終値は機械側の決定待ち |
| 最小曲げ半径 82.3mm / 輪の内径 114mm | `SOFTWARE_VERIFIED` | 幾何計算（`tests/test_safety_limits.py`）。**外皮を着せた実物では変わりうる** |
| とぐろが自己干渉しない（クリアランス 89.6mm） | `SOFTWARE_VERIFIED` | 中心線の距離計算。外皮の厚みは未考慮 |
| トルク上限 1.18N·m（ストール比 0.40） | `HARDWARE_UNVERIFIED` | **12V 版**の 30kgf·cm からの逆算。第一候補が **7.4V 1:191** に変わったため要再計算 |
| 挟み込み力 12.4N | `HARDWARE_UNVERIFIED` | 計算値。ISO/TS 15066 との比較も未（規格本文を参照できていない） |
| サーボの応答（一次遅れ・速度上限・温度上昇） | `SIMULATED` | `mock_servo` の係数はすべて仮値 |
| STS3215 のレジスタ仕様 | `HARDWARE_UNVERIFIED` | 一次資料（Waveshare/Feetech）で確認済みだが実機で試していない。`docs/sts3215_registers.md` |
| 最高入力電圧の初期値 8.0V（12V の罠） | `HARDWARE_UNVERIFIED` | メモリテーブル記載。**実機で最優先に確認する** |
| 負荷の符号ビット（bit10 仮説） | `HARDWARE_UNVERIFIED` | 公式資料に記載なし。仮説のまま |

## 3. 移動・歩容

| 項目 | レベル | 根拠 |
|---|---|---|
| 歩容の式と機体の出力が一致（差 < 1e-9°） | `SOFTWARE_VERIFIED` | `tests/test_phase2_device.py` |
| 1周期あたり 390mm 前進（6軸・振幅30°） | `SIMULATED` | `tangential_drag_ratio = 0.02` は**推定値**。実測で変わる |
| 旋回半径 388mm（γ0=20°） | `SIMULATED` | 同上 |
| 胴体ヨーの本数と移動性能の関係 | `SIMULATED` | `docs/product_status.md` §3。相対比較にのみ使う |
| マット端に寄らない（0.00%） | `SIMULATED` | 仮想フェンス。実機の滑りは未検証 |
| 摩擦（Wheel Belly / Snake Belly） | `HARDWARE_UNVERIFIED` | **Snake Belly の摩擦係数は未実測。** 仮値を現実として扱わない |
| 騒音 | **未測定** | 構想設計書が「最大のリスク」とする項目。実機が来たら最初に測る |

## 4. 知覚・行動

| 項目 | レベル | 根拠 |
|---|---|---|
| ArUco 検出（1080p で 50mm マーカ） | `SOFTWARE_VERIFIED` | 実写画像で検出率を測定済み（印刷物は実物） |
| 床ホモグラフィ・視差補正 | `SIMULATED` | 仮想カメラでの検証。実カメラでの誤差は未測定 |
| YOLO 人物検出（オフライン） | `SOFTWARE_VERIFIED` | `tests/test_offline.py`。ネット遮断で最後まで動く |
| 行動の状態遷移・効用調停 | `SOFTWARE_VERIFIED` | `tests/test_behavior_*.py` |
| 「生き物らしさ」 | **未評価** | 人が見ての評価。実機でしか判断できない |

## 4.5 仮想実機（Phase 2/3）

| 項目 | レベル | 根拠 |
|---|---|---|
| `SimulatedSnake`（PC → 仮想ESP32 → 仮想サーボ → シミュレータ） | `SIMULATED` | `tests/test_phase3_link_robot.py`（12） |
| `RealSnake`（PC → 実 ESP32 → STS3215） | **`HARDWARE_UNVERIFIED`** | 一度も実ポートで通信していない |
| 7状態の遷移（FAULT_HOLD からの復帰は DISARMED まで） | `SOFTWARE_VERIFIED` | `tests/test_phase2_safety.py` |
| テレメトリ v2（velocity / age / ttl / overrun / source） | `SOFTWARE_VERIFIED` | `tests/test_phase2_frame.py` |
| 仮想サーボの応答（一次遅れ・速度上限・温度・電圧降下） | `SIMULATED` | `mock_servo` の係数は仮値 |
| GUI の関節ペイン（指令角 / 実測角 / 機体状態） | `SOFTWARE_VERIFIED` | オフスクリーン描画テスト + 目視（output/gui_joints.png） |
| Belly（wheel / snake）と摩擦プロファイル | `SIMULATED` | `tests/test_belly_and_gait_config.py`。**係数は未実測** |
| 歩容の掃引（速さ・蛇らしさ・滑らかさ・負荷） | `SIMULATED` | `tools/gait_sweep.py` |

## 5. 書くときの約束

- 表・コメント・コミットメッセージで「確認済み」とだけ書かない。**上の4段のどれかを書く。**
- テレメトリは `source`（SIMULATION / HARDWARE）と `SIMULATED` フラグを持つ。
  **画面にも記録にもそのまま持ち回す。** 模擬の数値が実測として独り歩きしないようにするため。
- 実機で測ったら、この表と `docs/phase2_acceptance.md` §3 を**同時に**更新する。
  測定日・条件・測定器を書く。書けないものは `HARDWARE_VERIFIED` にしない。
