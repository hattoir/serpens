// Serpens EX-1 機体側の定数。**config/robot.yaml の link 節と同じ値を手で写す。**
// ここを変えたら robot.yaml も直すこと（食い違うと PC の表示と機体の挙動がずれる）。
// 出典: docs/link_protocol.md §5、config/robot.yaml の joints / link
#pragma once

#include <stdint.h>

// ---- 時間（docs/link_protocol.md §5。すべて要実測で確定する） ----
static const uint32_t HEARTBEAT_TIMEOUT_MS = 400;   // これだけ heartbeat が来なければ停止
static const uint32_t DRIVE_TTL_MAX_MS     = 1000;  // 受け付ける TTL の上限
static const uint32_t DRIVE_DISARM_MS      = 2000;  // 保持のまま放置したら待機へ
static const uint32_t CONTROL_PERIOD_MS    = 10;    // 100Hz
static const uint32_t TELEMETRY_PERIOD_MS  = 100;   // 10Hz

// ---- 指令値の絶対上限（PC の設定では緩められない） ----
static const float LIMIT_AMPLITUDE_DEG   = 40.0f;
static const float LIMIT_SPATIAL_DEG     = 150.0f;
static const float LIMIT_TEMPORAL_HZ     = 1.0f;
static const float LIMIT_GAMMA_DEG       = 30.0f;
static const float LIMIT_HEAD_SPEED_DPS  = 120.0f;

// ---- 異常で緊急停止（ラッチ）する条件 ----
static const uint8_t FAULT_TEMP_LIMIT_C = 60;       // behavior.safety.overheat_c(55) より上

// ---- 関節（config/robot.yaml の joints と同じ順・同じ値） ----
#define N_AXES 9
#define N_BODY 6                                    // J1..J6 が歩容に使う胴体ヨー

struct JointCfg {
  uint8_t servo_id;
  float   min_deg;
  float   max_deg;
  float   max_speed_dps;
  int8_t  direction;
  float   horn_offset_deg;                          // 実機で測って入れる（既定 0）
};

static const JointCfg JOINTS[N_AXES] = {
  {1, -85.0f, 85.0f, 240.0f, 1, 0.0f},              // J1 尾側
  {2, -85.0f, 85.0f, 240.0f, 1, 0.0f},
  {3, -85.0f, 85.0f, 240.0f, 1, 0.0f},
  {4, -85.0f, 85.0f, 240.0f, 1, 0.0f},
  {5, -85.0f, 85.0f, 240.0f, 1, 0.0f},
  {6, -85.0f, 85.0f, 240.0f, 1, 0.0f},              // J6 頭側の胴体ヨー
  {7,  -8.0f, 90.0f, 120.0f, 1, 0.0f},              // J7 首 pitch
  {8, -80.0f, 80.0f,  90.0f, 1, 0.0f},              // J8 頭 yaw
  {9, -35.0f, 35.0f,  90.0f, 1, 0.0f},              // J9 頭 roll
};

// 起動時の姿勢（config/robot.yaml の poses.home）。J7 = +8° は呼吸が下限で切れないため
static const float HOME_DEG[N_AXES] = {0, 0, 0, 0, 0, 0, 8.0f, 0, 0};

// ---- サーボ（docs/sts3215_registers.md） ----
static const uint32_t SERVO_BAUD      = 1000000;    // TTL バス
static const int      STEPS_PER_REV   = 4096;
static const int      CENTER_STEP     = 2047;

// ---- 配線（**未確定。実機を組む人が決めて、ここに書く**） ----
// TODO: 駆動リンク用 ESP32-S3 と サーボバスの結線が未定。以下は仮の名前だけ置いてある。
//   - 頭部センサ用の XIAO ESP32S3 と同じ基板にするのか、別基板にするのか
//   - サーボの半二重 TTL をどう作るのか（Waveshare Bus Servo Adapter を挟むのか、
//     方向制御ピン付きのトランシーバを載せるのか）
// 決まるまでフラッシュしないこと。
#define SERVO_UART_NUM   1
#define SERVO_TX_PIN     -1                         // TODO
#define SERVO_RX_PIN     -1                         // TODO
#define SERVO_DIR_PIN    -1                         // TODO（不要な回路なら -1 のまま）
