# ELECTRICAL_SAFETY_GATE 8 項目: 実機の前にやれることを全部やった状態（B-完了）と、人に渡す測定

作成 2026-10-02（Engineering、LB-E-046〜053 / SE-E10）。**HARDWARE_VERIFIED = 0。GATE の 8 項目は INCOMPLETE のまま**（`config/robot.yaml` の `safety_limits.electrical_safety_gate`。**証拠なしに状態を書き換えない**。`serpens/electrical_gate.py`、`tests/test_electrical_gate.py`）。
この文書は「実機を待つ以外にやれることを全部やった」ことを、項目ごとに示す（HBCL `closed-loop-protocol.md` §5）: ①測る量 ②Hardware-Gap の成果物（判断境界）③最小の実機試験 ④Human Action ⑤取り込み受け口（合成データの試験つき）⑥安全の境界。
数値はすべて prior（ASSUMPTION）。C044 の電流は **UNKNOWN**（12V 品の 2.7 A / 200 mA は使わない = `docs/electronics.md` §2）。再現: `simulation/hardware_gaps/HG-E1_electrical_gate/run.py`（`decision_boundary.md`、`results/e1_electrical.json`）。

**この文書で B-完了と書けないもの**（正直に）: `independent_power_cut` と `physical_estop` は**回路が無い**（部品未選定。LB-E-018）。測る対象が無いので、測定の手順と受け口は用意したが、「実機を待つ以外にやれること」には**回路の設計**が残る（A）。`real_stop_time` の通信断側は、実 ESP32 のサーボ層（ファームの `writeServos` / `readServos`）が未実装（LB-E-016）で、実機リンクが無い。

## 共通の安全の境界（すべての HA-E に適用）

- **最初の通電は人が立ち会う。Agent は通電を指示しない。** 電流制限つきの安定化電源（**3 A から**。HA-02 / OQ-0104）。**12 V をサーボに入れない**（レジスタ 14 の最高入力電圧の初期値 8.0 V。満充電 2S = 8.4 V も超える。`docs/sts3215_registers.md`）。手元に電源断スイッチ。指を入れない。
- 購入は User の承認（C）。測定の結果は、日付・条件・測定器・生データのファイルを揃えて渡す（無いものは `HARDWARE_VERIFIED` にしない）。
- 取り込みは `python tools/ingest_measurements.py <kind> <csv>`（`config/robot.yaml` も GATE の状態も書き換えない。要約を出すだけ。合成データで試験済み: `tests/test_ingest_measurements.py`）。

## 項目ごと

| GATE 項目 | ①測る量 | ②判断境界（HG-E1。prior）| ③最小の実機試験 | ④HA | ⑤取り込み（kind）| 残る A / 状態 |
|---|---|---|---|---|---|---|
| `real_current` | C044 1 個の無負荷・保持（レジスタ比 0.167）・歩容相当・ストールの電流 [A] | N 軸の合計: 保持 p50 2.8 A・歩容の実効 p50 4.5 A・全軸ストール p50 10.5 A（MVP 5 軸。prior の幅は `decision_boundary.md`）。**ストール電流が 3 A を超えるなら、電源・保護の前提が変わる** | C044 × 1、7.4 V・電流制限 3 A、電流計直列。無負荷 → 保持 → 往復動作 → ストール（1 秒以内、制限 3 A）| HA-E-01 | `current` | B-完了（購入と通電は人。HA-03）|
| `power_capacity` | 起動・ストール時の電圧降下 [V]・突入電流・電源の全抵抗 R [Ω] | **R ≤ 0.12 Ω（MVP 5 軸・公称 7.4 V）なら p95 の最悪電流でも 6 V を保つ**。満充電 8.4 V なら ≤ 0.20 Ω。EX-1 9 軸は ≤ 0.06 / 0.11 Ω | 実電源（電池または DC/DC）+ 実配線で、C044 1 個のストール / 起動の電圧波形（シャント + オシロまたは INA226）。R = 降下 ÷ ピーク電流で外挿 | HA-E-02 | `power_sag`（R を出す）| B-完了。電源の選択（OQ-0104・HA-03）は C |
| `overcurrent_protection` | 保護の動作電流 [A]・動作時間 [ms]（ヒューズ / PTC / 電子遮断）| 設定電流 T と「通常で誤動作する確率 / 全軸ストールで動作しない確率」の表。**釣り合う T ≈ 9.4 A（MVP 5 軸。誤動作 0.35・不動作 0.38）= ヒューズ 1 本では分けられない**（窓が開く個体は 0.62）→ **遅延つきの電子遮断 / 軸ごとの電流制限が要る** | 擬似負荷（電子負荷またはパワー抵抗）に、候補の保護部品を入れ、電流を上げて動作電流と時間を記録（部品ごと 5 回）| HA-E-03 | `trip`（動作しなかった試行を数える）| **回路・部品が未選定（A: LB-E-018）**。測る部品が決まるまで試験できない |
| `wiring_heat` | 幹線・コネクタの温度上昇 ΔT [K]（定常と時定数）| 配線: ΔT ≤ 20 K の実効電流は AWG24 で約 2.2 A・AWG22 で 2.8 A・AWG20 で 3.6 A（束ねた被覆線）。**幹線の実効電流 p95 は 8.6 A（MVP）= 細い線では足りない**。コネクタ: 接触抵抗 5.4 mΩ 以上で ΔT ≥ 20 K | 擬似負荷で p95 の実効電流（MVP で約 9 A）を 10 分流し、幹線の線・コネクタに熱電対（または放射温度計）。**電流制限と保護つき** | HA-E-04 | `thermal`（ΔT・τ）| B-完了（配線の太さの選定は設計 = A）|
| `independent_power_cut` | ESP32 を止めた（ハングさせた）状態で、サーボ電源が遮断されるまでの時間 [ms] | 電源の遮断 p50 約 25 ms（リレー 3〜15 ms + 出力の低下 1〜30 ms。prior）。**20 ms 以内に届く確率 0.34** | 遮断器 + 安全 MCU / watchdog を作り、ESP32 のパルスを止めて V+ が落ちるまでをオシロで | HA-E-05 | `stop_time`（what = supply_cut, esp32_running = 0）| **回路が無い**（LB-E-018）。測る対象ができてから |
| `physical_estop` | 押してから遮断まで [ms]、ケーブルを抜いたとき（常閉）も遮断されるか | 同上 | E-STOP スイッチを遮断器の駆動に直列に入れ、押す / 抜くで V+ を測る | HA-E-06 | `stop_time`（trigger = estop）| **部品・回路が無い**（Design の OPEN-007 = 見た目は C）|
| `servo_temperature` | 連続運転の温度上昇 [℃/min] と、機体が 60 ℃ で止まる実際の温度 | 保持の実効電流が約 0.9 A 以上だと 10 分以内に 60 ℃ に届く（中央の熱パラメータ）。保持 0.3 A では届かない。**順序 60 < 70 < 80 ℃（機体側 < サーボのレジスタ < 安全の停止）** | C044 × 1 に負荷（保持）をかけ、レジスタ 63 の温度と熱電対を 30 分記録。60 ℃ で機体が止まるかは、サーボ層のファーム（LB-E-016）が要る | HA-E-07 | `servo_temp` | B-完了（60 ℃ で止まる確認の部分は、ファームのサーボ層が先 = A）|
| `real_stop_time` | 緊急停止・通信断・DRIVE の TTL 切れから、動きが止まるまで [ms] | 通信断 → 保持 p50 約 450 ms（heartbeat 400 ms + 周期 + 動き）、緊急停止 → 動きの停止 p50 約 67 ms、TTL 切れ → 保持 約 350 ms。**接触 → 脱力 20 ms は要求そのものが未確定**（Q1〜Q5 = C）| 実 ESP32 + サーボ 1 個で、PC の送信停止 / USB 抜去 / 緊急停止コマンドから、サーボの位置（または電流）が止まるまでをロジアナ / オシロで | HA-E-08 | `stop_time`（trigger = comm_loss / estop, what = motion_stop）| **実機リンクが無い**（ファームのサーボ層 LB-E-016）。接触検出の要求（Q1〜Q5）は C |

## 読み方

- 上の「判断境界」は prior の幅に従属する（HG-E1 の限界）。**実測が来たら、`assumptions.yaml` を実測に置き換えて `run.py` を回し直す**。
- 5 サーボ（CAD）と 6 モーター（方針）の併存は C（LB-E-063）。N が変わると全部の数字が動く（`decision_boundary.md` に MVP 5 / EX-1 9 の両方）。
- **B-完了の判定（protocol §5）**: 各項目に ①〜⑤ を書いた。ただし `overcurrent_protection` / `independent_power_cut` / `physical_estop` は**測る対象（回路・部品）が未選定**、`real_stop_time` はサーボ層が未実装のため、「実機を待つ以外にやれること」が残る（A: LB-E-018 / LB-E-016）。Ledger では B\* のまま（B-完了と書かない）。
