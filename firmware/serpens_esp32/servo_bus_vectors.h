// **自動生成**: tools/gen_servo_vectors.py（ベンダー SDK scservo_sdk の送信バイト列が正解）。手で書き換えない。
// 実機・別の C++ 環境で `sbSelfTest()`（servo_bus.h）がこの正解と自分の出力を比べる。HARDWARE_UNVERIFIED。
#pragma once
#include <stdint.h>
#include <stddef.h>

// WRITE1: id=3 addr=40 val=1（トルクスイッチ ON）
#define SBV_W1_ID 3
#define SBV_W1_ADDR 40
#define SBV_W1_VAL 1
static const uint8_t SBV_WRITE1[] = {0xFF, 0xFF, 0x03, 0x04, 0x03, 0x28, 0x01, 0xCC};
static const size_t  SBV_WRITE1_LEN = 8;

// WRITE2: id=7 addr=48 val=167（トルク制限 = 0.167 × 1000）
#define SBV_W2_ID 7
#define SBV_W2_ADDR 48
#define SBV_W2_VAL 167
static const uint8_t SBV_WRITE2[] = {0xFF, 0xFF, 0x07, 0x05, 0x03, 0x30, 0xA7, 0x00, 0x19};
static const size_t  SBV_WRITE2_LEN = 9;

// READ_STATE: id=5 addr=56 len=8
#define SBV_R_ID 5
#define SBV_R_ADDR 56
#define SBV_R_LEN 8
static const uint8_t SBV_READ_STATE[] = {0xFF, 0xFF, 0x05, 0x04, 0x02, 0x38, 0x08, 0xB4};
static const size_t  SBV_READ_STATE_LEN = 8;

// SYNC_WRITE（位置・速度・加速度。3 軸）
#define SBV_SYNC_N 3
#define SBV_SYNC_ACC 254
static const uint8_t  SBV_SYNC_IDS[]   = {1, 2, 3};
static const uint16_t SBV_SYNC_POS[]   = {2047, 1500, 3000};
static const uint16_t SBV_SYNC_SPEED[] = {3000, 1200, 500};
static const uint8_t SBV_SYNC_WRITE[] = {0xFF, 0xFF, 0xFE, 0x1C, 0x83, 0x29, 0x07, 0x01, 0xFE, 0xFF, 0x07, 0x00, 0x00, 0xB8, 0x0B, 0x02, 0xFE, 0xDC, 0x05, 0x00, 0x00, 0xB0, 0x04, 0x03, 0xFE, 0xB8, 0x0B, 0x00, 0x00, 0xF4, 0x01, 0x1C};
static const size_t  SBV_SYNC_WRITE_LEN = 32;

// 応答（status）: id=5、data = 位置 2047・速度 0・負荷 16・電圧 7.4 V・温度 31 ℃（SDK の readTxRx が同じ data を返すことを tests/test_servo_vectors.py で確認）
#define SBV_STATUS_ID 5
static const uint8_t SBV_STATUS[] = {0xFF, 0xFF, 0x05, 0x0A, 0x00, 0xFF, 0x07, 0x00, 0x00, 0x10, 0x00, 0x4A, 0x1F, 0x71};
static const size_t  SBV_STATUS_LEN = 14;

static const uint8_t SBV_STATUS_DATA[] = {0xFF, 0x07, 0x00, 0x00, 0x10, 0x00, 0x4A, 0x1F};
