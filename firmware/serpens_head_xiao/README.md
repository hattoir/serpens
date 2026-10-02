# 頭の XIAO ESP32S3（Sense）ファーム — 骨組み（LB-E-082、2026-10-02）

**コンパイルだけ。書き込み・通電はしていない。実機（基板）が無い。HARDWARE_UNVERIFIED。HARDWARE_VERIFIED = 0。**

```bash
.venv/Scripts/python.exe tools/build_firmware.py --sketch firmware/serpens_head_xiao
```

結果（esp32 core 2.0.17 / `esp32:esp32:XIAO_ESP32S3`、2026-10-02 に自分で実行）: フラッシュ 267,421 バイト（8%）/ RAM 18,632 バイト（5%）。書き込みはしていない。
**C++ としては一度も実行していない**（ホストに C++ コンパイラが無い）。`tests/test_firmware_head_sync.py` は定数と行の書式の一致だけを見る。

## 仕様の出どころ

- GPIO: ENTRY-K-0001 (1)（User 承認 2026-10-01、head-sensor-board `85a61fe`）。D0 = XSHUT_L / D1 = XSHUT_R / D2 = LED_L の PWM / D3 = LED_R の PWM / D4 = SDA / D5 = SCL。D8・D9・D10 は microSD と共用（Seeed Wiki で確認 = `HG-H2_sensor_head/hardware_test_plan.md` (a)）。
- ToF の起動順: K-0001 (2)（2 個の VL53L1X は既定アドレス 0x29 が同じ → XSHUT で 1 個ずつ起こしてアドレスを変える）。参照レジスタ（0x010F = 0xEA、0x0110 = 0xCC）はデータシート Table 8。
- 行プロトコル: `serpens/hw/head_io.py`（`P` → `P,ok,<fw>`、`S,<uptime_ms>,<tof_mm>,<touch_head>,<touch_back>,<seq>` を 10 Hz）。

## できること / できないこと

| | 状態 |
|---|---|
| USB CDC の行プロトコル（P / S）| 骨組みを書いた（実行は未）|
| ToF ×2 の起動順・ID の確認 | 骨組み。**ranging は未実装**（ST の ULD / ライブラリが要る）。S 行の ToF は常に 0 = 測定不能 |
| LED ×2 の PWM（LEDC、25 kHz、起動時は消灯、上限 `LED_DUTY_MAX`）| 骨組み。`L,<左>,<右>` は **PROPOSED**（斜め LED の試験用。Python 側に呼び出しは無い）。`E` / `M`（目の RGB）はこの基板に無いので解釈だけして何もしない |
| タッチ ×2、フードの端のビット（案 H）、SG90 の駆動 | **ピンが無い**（K-0001 の基板に載っていない）= `-1`。フードのビットを PC へ返す行（案: `H,<down>,<up>,<seq>`）は Python 側に未定義。`serpens/floorwatch/hood.py` の 0.6 s 判定は PC 側で、ビットが届かなければ「立っていない」= 止める側に倒れる |

## 要確認（実機で / 一次資料で）

1. **アドレス変更のレジスタ `I2C_SLAVE__DEVICE_ADDRESS` = 0x0001（7 bit 値）**: データシートに載っていない（UM2356 / ST の ULD）。この環境では一次資料を確認していない。変更後に新アドレスで ID を読み直して、成功したときだけ「左 = 使える」にしている。
2. `TOF_ADDR_L = 0x30` は仮置き。`TOF_BOOT_DELAY_MS = 5` は余裕を見た仮置き（データシートの起動時間を確認する）。
3. GPIO3（D2 = LED_L）はストラッピング ピン。eFuse が既定なら無視される（K-0001）。実機で LED 点灯時に起動できるかを確認する。
4. 3V3 駆動では、Wi-Fi 送信・撮影・LED 100 %・ToF のピークを**同時にしない**（K-0002 (b)。採用した運用）。`LED_DUTY_MAX` で絞れる。
5. このスケッチは**頭の XIAO 専用**。駆動リンク（胴の ESP32）とは別（`firmware/serpens_esp32`）。兼用にするかは User の判断（`OQ-ENG-BOUNDARY` EC8）。
