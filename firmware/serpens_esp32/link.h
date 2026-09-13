// PC ⇄ 機体のフレーム処理。docs/link_protocol.md v1 と serpens/link/protocol.py の写し。
// **どれか一つを直したら三つとも直すこと。** 値が食い違うと通信が成立しない。
#pragma once

#include <Arduino.h>

static const uint8_t  SOF0 = 0xA5;
static const uint8_t  SOF1 = 0x5A;
static const uint8_t  LINK_VERSION = 1;
static const uint16_t MAX_PAYLOAD = 128;         // テレメトリ 9軸 = 93 バイト
static const uint8_t  HEADER_LEN = 5;            // ver + type + seq(2) + len
static const uint16_t SEQ_FORWARD_WINDOW = 4096;

enum Cmd : uint8_t {
  CMD_HEARTBEAT = 0x01, CMD_ARM = 0x02, CMD_DISARM = 0x03,
  CMD_DRIVE = 0x10, CMD_HEAD = 0x11, CMD_POSE = 0x12, CMD_BREATH = 0x13,
  CMD_BODY = 0x14, CMD_TORQUE = 0x15,
  CMD_STOP = 0x20, CMD_EMERGENCY = 0x21, CMD_CLEAR_FAULT = 0x22,
  CMD_LIMITS = 0x30, CMD_PING = 0x40,
};

enum Rep : uint8_t { REP_TELEMETRY = 0x80, REP_ACK = 0x81, REP_NACK = 0x82, REP_EVENT = 0x83 };

enum NackReason : uint8_t {
  NACK_BAD_CRC = 1, NACK_BAD_VERSION = 2, NACK_BAD_LENGTH = 3, NACK_STALE_SEQ = 4,
  NACK_OUT_OF_RANGE = 5, NACK_DISARMED = 6, NACK_LATCHED = 7, NACK_UNKNOWN_CMD = 8,
  NACK_NO_HEARTBEAT = 9, NACK_NONCE_REUSED = 10, NACK_BUSY = 11,
};

enum DeviceState : uint8_t { ST_DISARMED = 0, ST_ARMED = 1, ST_EMERGENCY = 2 };

enum StopReason : uint8_t {
  SR_NONE = 0, SR_OPERATOR_STOP = 1, SR_DRIVE_TTL = 2, SR_HEARTBEAT_LOST = 3,
  SR_EMERGENCY_CMD = 4, SR_OVERHEAT = 5, SR_OVERCURRENT = 6, SR_SERVO_FAULT = 7,
  SR_OUT_OF_RANGE = 8, SR_BOOT = 9,
};

enum Flags : uint8_t {
  FL_DRIVING = 1 << 0, FL_BREATHING = 1 << 1, FL_HEARTBEAT_OK = 1 << 2,
  FL_DRIVE_VALID = 1 << 3, FL_TORQUE_ON = 1 << 4,
};

// payload の長さ（type ごと）。-1 = 検査しない
inline int payloadLenFor(uint8_t type) {
  switch (type) {
    case CMD_HEARTBEAT: case CMD_ARM: case CMD_DISARM: case CMD_PING: return 0;
    case CMD_DRIVE: return 10;      // ttl(u16) + 4 × i16
    case CMD_HEAD:  return 10;      // ttl(u16) + 3 × i16 + speed(u16)
    case CMD_BODY:  return 16;      // ttl(u16) + 6 × i16 + speed(u16)
    case CMD_TORQUE: return 2;      // ratio(u16 0.001)
    case CMD_POSE:  case CMD_BREATH: case CMD_EMERGENCY: return 1;
    case CMD_STOP:  return 2;
    case CMD_CLEAR_FAULT: return 4;
    default: return -1;
  }
}

// CRC-16/CCITT-FALSE（初期値 0xFFFF、多項式 0x1021）。"123456789" → 0x29B1
inline uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

// seq が進んでいるか（重複・巻き戻りを弾く）
inline bool seqIsForward(uint16_t seq, uint16_t last, bool hasLast) {
  if (!hasLast) return true;
  uint16_t d = (uint16_t)(seq - last);
  return d >= 1 && d <= SEQ_FORWARD_WINDOW;
}

// 受信の状態機械。分割・ゴミ混入から SOF 走査で復帰する
class FrameReader {
 public:
  uint8_t  type = 0;
  uint16_t seq = 0;
  uint8_t  version = 0;
  uint8_t  payload[MAX_PAYLOAD];
  uint8_t  len = 0;
  bool     crcOk = false;
  uint32_t droppedBytes = 0;
  uint32_t badCrc = 0;

  // 1バイト入れて、フレームが揃ったら true
  bool feed(uint8_t b) {
    switch (st_) {
      case S_SOF0: if (b == SOF0) st_ = S_SOF1; else droppedBytes++; return false;
      case S_SOF1:
        if (b == SOF1) { st_ = S_VER; return false; }
        droppedBytes++; st_ = (b == SOF0) ? S_SOF1 : S_SOF0; return false;
      case S_VER: version = b; st_ = S_TYPE; return false;
      case S_TYPE: type = b; st_ = S_SEQ0; return false;
      case S_SEQ0: seq = b; st_ = S_SEQ1; return false;
      case S_SEQ1: seq |= (uint16_t)b << 8; st_ = S_LEN; return false;
      case S_LEN:
        len = b; idx_ = 0;
        if (len > MAX_PAYLOAD) { droppedBytes += HEADER_LEN; st_ = S_SOF0; return false; }
        st_ = len ? S_PAYLOAD : S_CRC0; return false;
      case S_PAYLOAD:
        payload[idx_++] = b;
        if (idx_ >= len) st_ = S_CRC0;
        return false;
      case S_CRC0: crcLo_ = b; st_ = S_CRC1; return false;
      case S_CRC1: {
        uint16_t got = (uint16_t)crcLo_ | ((uint16_t)b << 8);
        uint8_t body[HEADER_LEN + MAX_PAYLOAD];
        body[0] = version; body[1] = type; body[2] = (uint8_t)(seq & 0xFF);
        body[3] = (uint8_t)(seq >> 8); body[4] = len;
        memcpy(body + HEADER_LEN, payload, len);
        crcOk = (got == crc16(body, HEADER_LEN + len));
        if (!crcOk) badCrc++;
        st_ = S_SOF0;
        return true;
      }
    }
    return false;
  }

 private:
  enum St { S_SOF0, S_SOF1, S_VER, S_TYPE, S_SEQ0, S_SEQ1, S_LEN, S_PAYLOAD, S_CRC0, S_CRC1 };
  St st_ = S_SOF0;
  uint8_t idx_ = 0;
  uint8_t crcLo_ = 0;
};

// 1フレーム送る（機体 → PC は seq = 0 固定）
inline void sendFrame(Stream& io, uint8_t type, const uint8_t* payload, uint8_t len) {
  uint8_t body[HEADER_LEN + MAX_PAYLOAD];
  body[0] = LINK_VERSION; body[1] = type; body[2] = 0; body[3] = 0; body[4] = len;
  if (len) memcpy(body + HEADER_LEN, payload, len);
  uint16_t crc = crc16(body, HEADER_LEN + len);
  uint8_t sof[2] = {SOF0, SOF1};
  io.write(sof, 2);
  io.write(body, HEADER_LEN + len);
  uint8_t tail[2] = {(uint8_t)(crc & 0xFF), (uint8_t)(crc >> 8)};
  io.write(tail, 2);
}

inline void sendAck(Stream& io, uint16_t seq, uint8_t cmd) {
  uint8_t p[3] = {(uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8), cmd};
  sendFrame(io, REP_ACK, p, 3);
}

inline void sendNack(Stream& io, uint16_t seq, uint8_t cmd, uint8_t reason) {
  uint8_t p[4] = {(uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8), cmd, reason};
  sendFrame(io, REP_NACK, p, 4);
}

inline void sendEvent(Stream& io, uint8_t code, uint8_t detail) {
  uint8_t p[2] = {code, detail};
  sendFrame(io, REP_EVENT, p, 2);
}

inline int16_t rdI16(const uint8_t* p) { return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
inline uint16_t rdU16(const uint8_t* p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
inline uint32_t rdU32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
