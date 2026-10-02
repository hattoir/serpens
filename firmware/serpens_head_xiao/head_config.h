// 頭の XIAO ESP32S3（Sense）の設定。**ENTRY-K-0001 (1) の GPIO 割り当て（User 承認済み 2026-10-01）の写し**。
// 出典: KiCad Agent の head-sensor-board（commit 85a61fe）。XIAO の D0〜D10 は GPIO1〜9・43・44（Seeed Wiki で確認 = HG-H2 の hardware_test_plan.md (a)）。
// HARDWARE_UNVERIFIED。基板はまだ無い（ガーバー未出力）。
#pragma once

#define HEAD_FW_VERSION "head-0.1-skeleton"

// ---- ピン（K-0001）---------------------------------------------------------------------------------
#define HEAD_PIN_XSHUT_L  D0     // GPIO1  VL53L1X 左の XSHUT（10 kΩ プルアップ実装。L = スタンバイ）
#define HEAD_PIN_XSHUT_R  D1     // GPIO2  VL53L1X 右の XSHUT
#define HEAD_PIN_LED_L    D2     // GPIO3  LED 左の PWM（ストラッピング ピン。100 kΩ で L に固定。eFuse が既定なら無視される）
#define HEAD_PIN_LED_R    D3     // GPIO4  LED 右の PWM
#define HEAD_PIN_SDA      D4     // GPIO5  I2C SDA（基板のプルアップ 3.3 kΩ は既定で未実装 = DNP）
#define HEAD_PIN_SCL      D5     // GPIO6  I2C SCL
// D6・D7（GPIO43・44）= UART 予約（基板では未接続）。D8・D9（GPIO7・8）= microSD と共用、ToF の GPIO1 へのリンクは DNP（既定は非接続）。D10 = 未使用。
// **タッチ ×2・フードの端のビット（案 H のホール素子）・SG90 の駆動のピンは K-0001 の基板に無い**（未割り当て）。-1 = 使わない。
#define HEAD_PIN_TOUCH_HEAD  -1
#define HEAD_PIN_TOUCH_BACK  -1
#define HEAD_PIN_HOOD_DOWN   -1
#define HEAD_PIN_HOOD_UP     -1

// ---- I2C / ToF（VL53L1X ×2）--------------------------------------------------------------------------
#define HEAD_I2C_HZ            400000     // データシートの最大 400 kbit/s
#define TOF_ADDR_DEFAULT       0x29       // 既定 0x52（8 bit 表記）= 7 bit で 0x29（データシート）
#define TOF_ADDR_L             0x30       // 変更後（左）。**値は仮置き（ASSUMED）**。右は既定のまま 0x29
#define TOF_ADDR_R             TOF_ADDR_DEFAULT
#define TOF_REG_MODEL_ID       0x010F     // 0xEA（データシート Table 8）。次の 0x0110 = module type 0xCC。16 bit 指標を MSB から
#define TOF_MODEL_ID           0xEA
#define TOF_MODULE_TYPE        0xCC
// アドレス変更のレジスタ I2C_SLAVE__DEVICE_ADDRESS = 0x0001（7 bit 値を書く）。**データシートには載っていない**（UM2356 / ST の ULD にある）。
// この環境では UM2356 を確認していない = 要確認（HARDWARE_UNVERIFIED。実機で書いたあと、新アドレスで ID を読み直して確認する）。
#define TOF_REG_I2C_ADDR       0x0001
#define TOF_BOOT_DELAY_MS      5          // XSHUT を上げてから I2C に応答するまでの待ち（データシートの起動時間 = 要確認。余裕を持って置く）

// ---- LED（LEDC）---------------------------------------------------------------------------------------
// PWM 周波数は人の耳に聞こえない帯（> 20 kHz）。8 bit 分解能で 20 kHz は ESP32-S3 の LEDC で成り立つ（80 MHz / 2^8 = 312 kHz 上限）。
#define LED_PWM_HZ             25000
#define LED_PWM_BITS           8
// K-0002 (b): 3V3 駆動では「Wi-Fi 送信・撮影・LED 100 %・ToF のピーク」を同時にしない。**LED の上限（デューティ）**。既定は 100 %（制限しない）= 運用で絞る。
#define LED_DUTY_MAX           255

// ---- 通信（head_io.py と同じ）--------------------------------------------------------------------------
#define HEAD_BAUD              115200
#define HEAD_SENSOR_HZ         10        // S 行の周期（config/robot.yaml head_io.sensor_rate_hz）
#define HEAD_SEQ_MODULO        65536
#define HEAD_LINE_MAX          64
