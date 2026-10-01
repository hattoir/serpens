// STS3215（Feetech SCS/STS 系）のサーボバス層。**パケットの生成・解析（純粋関数）+ ポートの抽象 + 偽サーボ + 軸ごとの操作。**
//
// バイト列の正解 = ベンダー SDK（scservo_sdk。PC 側 serpens/hw/feetech_bus.py が使う）の送信バイト列。
//   tools/gen_servo_vectors.py が servo_bus_vectors.h に正解を書き出し、sbSelfTest() が自分の出力と比べる。
// レジスタ・単位・符号は docs/sts3215_registers.md（Python の deg_to_step / step_to_deg / decode_* と同じ式）。
//
// **HARDWARE_UNVERIFIED**: 実機が無い。コンパイルは通る（tools/build_firmware.py。書き込みはしない）が、**C++ としては一度も走らせていない**
//   （この PC に C++ コンパイラが無く、ESP32 向けの交差コンパイルだけができる）。実機・基板が来たら、まず `SERPENS_SERVO_SELFTEST` で
//   sbSelfTest() を通してからサーボをつなぐこと（firmware/serpens_esp32/README.md）。
// 安全: このファイルは**通電を始めない**。ポートを渡されて初めて何かを送る。配線（SERVO_*_PIN）が決まるまでポートは作られない。
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "config.h"

namespace sb {

static const uint8_t BROADCAST_ID = 0xFE;
static const uint8_t INST_READ = 0x02, INST_WRITE = 0x03, INST_SYNC_WRITE = 0x83;
static const uint8_t ADDR_TORQUE_ENABLE = 40, ADDR_ACC = 41, ADDR_TORQUE_LIMIT = 48, ADDR_STATE = 56;
static const uint8_t STATE_LEN = 8;                          // 56〜63 = 位置 2・速度 2・負荷 2・電圧 1・温度 1
static const size_t  MAX_AXES = 9;
static const size_t  SYNC_ENTRY = 8;                         // ID + 7 バイト（加速度・位置 2・時間 2・速度 2）
static const size_t  MAX_TX = 8 + MAX_AXES * SYNC_ENTRY;
static const size_t  MAX_RX = 6 + STATE_LEN;

// ---- パケット（FF FF ID LEN INST ... CHK。CHK = ~(ID + LEN + INST + 以降) & 0xFF）----
inline uint8_t checksum(const uint8_t* p, size_t total) {    // p は先頭（FF FF）からチェックサムの位置までを含む total バイト
  uint8_t s = 0;
  for (size_t i = 2; i + 1 < total; i++) s += p[i];
  return (uint8_t)(~s);
}

// 個別の書き込み（応答を確かめたいので broadcast は不可）。戻り値 = パケットの長さ（0 = 作れない）
inline size_t buildWrite(uint8_t* out, size_t cap, uint8_t id, uint8_t addr, const uint8_t* data, uint8_t len) {
  const size_t total = (size_t)len + 7;
  if (id >= BROADCAST_ID || cap < total) return 0;
  out[0] = 0xFF; out[1] = 0xFF; out[2] = id; out[3] = (uint8_t)(len + 3); out[4] = INST_WRITE; out[5] = addr;
  for (uint8_t i = 0; i < len; i++) out[6 + i] = data[i];
  out[6 + len] = checksum(out, total);
  return total;
}

inline size_t buildWrite1(uint8_t* out, size_t cap, uint8_t id, uint8_t addr, uint8_t v) { return buildWrite(out, cap, id, addr, &v, 1); }

inline size_t buildWrite2(uint8_t* out, size_t cap, uint8_t id, uint8_t addr, uint16_t v) {
  uint8_t d[2] = {(uint8_t)(v & 0xFF), (uint8_t)(v >> 8)};
  return buildWrite(out, cap, id, addr, d, 2);
}

inline size_t buildRead(uint8_t* out, size_t cap, uint8_t id, uint8_t addr, uint8_t len) {
  if (id >= BROADCAST_ID || cap < 8) return 0;
  out[0] = 0xFF; out[1] = 0xFF; out[2] = id; out[3] = 4; out[4] = INST_READ; out[5] = addr; out[6] = len;
  out[7] = checksum(out, 8);
  return 8;
}

// 同期書き込み（41 番地から 7 バイト: 加速度・位置・時間（0）・速度）。応答は無い
inline size_t buildSyncWritePos(uint8_t* out, size_t cap, const uint8_t* ids, const uint16_t* pos, const uint16_t* speed, uint8_t acc, size_t n) {
  const size_t total = 8 + n * SYNC_ENTRY;
  if (n == 0 || n > MAX_AXES || cap < total) return 0;
  out[0] = 0xFF; out[1] = 0xFF; out[2] = BROADCAST_ID; out[3] = (uint8_t)(n * SYNC_ENTRY + 4); out[4] = INST_SYNC_WRITE;
  out[5] = ADDR_ACC; out[6] = 7;
  size_t o = 7;
  for (size_t i = 0; i < n; i++) {
    out[o++] = ids[i]; out[o++] = acc;
    out[o++] = (uint8_t)(pos[i] & 0xFF); out[o++] = (uint8_t)(pos[i] >> 8);
    out[o++] = 0; out[o++] = 0;
    out[o++] = (uint8_t)(speed[i] & 0xFF); out[o++] = (uint8_t)(speed[i] >> 8);
  }
  out[o] = checksum(out, total);
  return total;
}

enum ParseResult { PARSE_OK = 0, PARSE_SHORT = -1, PARSE_HEADER = -2, PARSE_ID = -3, PARSE_LEN = -4, PARSE_CHECKSUM = -5 };

// サーボの応答（FF FF ID LEN ERR data... CHK。LEN = data の長さ + 2）を解析する。err にサーボの状態ビット（0 = 異常なし）
inline int parseStatus(const uint8_t* buf, size_t n, uint8_t id, uint8_t* data, size_t dataLen, uint8_t* err) {
  const size_t total = 6 + dataLen;
  if (n < total) return PARSE_SHORT;
  if (buf[0] != 0xFF || buf[1] != 0xFF) return PARSE_HEADER;
  if (buf[2] != id) return PARSE_ID;
  if (buf[3] != dataLen + 2) return PARSE_LEN;
  if (checksum(buf, total) != buf[total - 1]) return PARSE_CHECKSUM;
  *err = buf[4];
  for (size_t i = 0; i < dataLen; i++) data[i] = buf[5 + i];
  return PARSE_OK;
}

// ---- 単位変換（Python の deg_to_step / step_to_deg / dps_to_step_s / decode_* と同じ式）----
inline int clampInt(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

inline uint16_t degToStep(float deg, int8_t direction, float hornOffsetDeg) {
  const float servoDeg = (float)direction * deg + hornOffsetDeg;
  return (uint16_t)clampInt(CENTER_STEP + (int)lroundf(servoDeg * (float)STEPS_PER_REV / 360.0f), SERVO_STEP_MIN, SERVO_STEP_MAX);
}

inline float stepToDeg(int step, int8_t direction, float hornOffsetDeg) {
  const float servoDeg = (float)(step - CENTER_STEP) * 360.0f / (float)STEPS_PER_REV;
  return (servoDeg - hornOffsetDeg) / (float)direction;
}

inline uint16_t dpsToStepS(float dps) {                      // 0 は書かない（資料に意味が無い）
  return (uint16_t)clampInt((int)lroundf(fabsf(dps) * (float)STEPS_PER_REV / 360.0f), SERVO_SPEED_MIN_STEP_S, SERVO_SPEED_MAX_STEP_S);
}

inline int16_t signMagnitude(uint16_t raw, uint8_t signBit) {  // 符号ビット + 絶対値（SDK の scs_tohost と同じ）
  return (raw & (1u << signBit)) ? (int16_t)(-(int)(raw & ~(1u << signBit))) : (int16_t)raw;
}

inline float decodeLoad(uint16_t raw) {                       // 【仮説】bit10 = 方向（資料に記載なし。実機で確認）
  const int mag = raw & ((1u << SERVO_LOAD_SIGN_BIT) - 1);
  return (float)((raw & (1u << SERVO_LOAD_SIGN_BIT)) ? -mag : mag) * 0.001f;
}

// ---- ポートの抽象（実機は HardwareSerial、試験は偽サーボ）----
struct ServoPort {
  virtual ~ServoPort() {}
  virtual size_t write(const uint8_t* p, size_t n) = 0;                            // 送った数
  virtual size_t read(uint8_t* p, size_t want, uint32_t timeoutMs) = 0;            // 読めた数（timeout まで待つ）
  virtual void flush() = 0;                                                        // 受信バッファを捨てる
};

// ---- 偽サーボ（RAM の模型。**模擬であって実機の挙動ではない**）----
// 受け取ったパケットを解析して状態を更新し、応答（status）を溜める。位置は指令どおりに瞬時に動く。
class FakeServoPort : public ServoPort {
 public:
  FakeServoPort() { reset(); }
  void reset() {
    for (size_t i = 0; i <= MAX_AXES; i++) {
      online_[i] = true; pos_[i] = (uint16_t)CENTER_STEP; torque_[i] = 0; limit_[i] = 1000; speed_[i] = 0; acc_[i] = 0;
      temp_[i] = 30; volt_[i] = 74; err_[i] = 0;
    }
    rxLen_ = rxPos_ = 0; writes_ = 0;
  }
  void setOnline(uint8_t id, bool on) { if (id <= MAX_AXES) online_[id] = on; }
  void setTemp(uint8_t id, uint8_t c) { if (id <= MAX_AXES) temp_[id] = c; }
  void setErr(uint8_t id, uint8_t e) { if (id <= MAX_AXES) err_[id] = e; }
  uint16_t pos(uint8_t id) const { return pos_[id]; }
  uint8_t torque(uint8_t id) const { return torque_[id]; }
  uint16_t limit(uint8_t id) const { return limit_[id]; }
  uint16_t speed(uint8_t id) const { return speed_[id]; }
  uint8_t acc(uint8_t id) const { return acc_[id]; }
  uint32_t writes() const { return writes_; }

  size_t write(const uint8_t* p, size_t n) override {
    writes_++;
    if (n < 6 || p[0] != 0xFF || p[1] != 0xFF || (size_t)p[3] + 4 > n || checksum(p, (size_t)p[3] + 4) != p[(size_t)p[3] + 3]) return n;   // 壊れたパケットは無視（サーボと同じ）
    const uint8_t id = p[2], inst = p[4];
    if (inst == INST_SYNC_WRITE && id == BROADCAST_ID) {
      const uint8_t addr = p[5], dlen = p[6];
      if (addr == ADDR_ACC && dlen == 7) {
        for (size_t o = 7; o + 8 <= (size_t)p[3] + 3; o += 8) {
          const uint8_t sid = p[o];
          if (sid >= 1 && sid <= MAX_AXES && online_[sid] && torque_[sid]) {         // トルクが入っている軸だけ動く
            acc_[sid] = p[o + 1]; pos_[sid] = (uint16_t)(p[o + 2] | (p[o + 3] << 8)); speed_[sid] = (uint16_t)(p[o + 6] | (p[o + 7] << 8));
          }
        }
      }
      return n;                                                                      // 応答なし
    }
    if (id < 1 || id > MAX_AXES || !online_[id]) return n;                           // いない軸は黙る（読む側は timeout）
    if (inst == INST_WRITE) {
      const uint8_t addr = p[5], len = (uint8_t)(p[3] - 3);
      if (addr == ADDR_TORQUE_ENABLE && len == 1) torque_[id] = p[6];
      else if (addr == ADDR_TORQUE_LIMIT && len == 2) limit_[id] = (uint16_t)(p[6] | (p[7] << 8));
      reply(id, 0, 0, 0);
    } else if (inst == INST_READ && p[5] == ADDR_STATE && p[6] == STATE_LEN) {
      uint8_t d[STATE_LEN] = {(uint8_t)(pos_[id] & 0xFF), (uint8_t)(pos_[id] >> 8), 0, 0, 0, 0, volt_[id], temp_[id]};
      reply(id, err_[id], d, STATE_LEN);
    }
    return n;
  }

  size_t read(uint8_t* p, size_t want, uint32_t) override {
    size_t k = 0;
    while (k < want && rxPos_ < rxLen_) p[k++] = rx_[rxPos_++];
    return k;
  }
  void flush() override { rxLen_ = rxPos_ = 0; }

 private:
  void reply(uint8_t id, uint8_t err, const uint8_t* d, size_t n) {
    if (rxLen_ + 6 + n > sizeof(rx_)) return;
    uint8_t* o = rx_ + rxLen_;
    o[0] = 0xFF; o[1] = 0xFF; o[2] = id; o[3] = (uint8_t)(n + 2); o[4] = err;
    for (size_t i = 0; i < n; i++) o[5 + i] = d[i];
    o[5 + n] = checksum(o, 6 + n);
    rxLen_ += 6 + n;
  }
  bool online_[MAX_AXES + 1];
  uint16_t pos_[MAX_AXES + 1], limit_[MAX_AXES + 1], speed_[MAX_AXES + 1];
  uint8_t torque_[MAX_AXES + 1], acc_[MAX_AXES + 1], temp_[MAX_AXES + 1], volt_[MAX_AXES + 1], err_[MAX_AXES + 1];
  uint8_t rx_[128];
  size_t rxLen_, rxPos_;
  uint32_t writes_;
};

// ---- 軸ごとの操作 ----
struct AxisCfg { uint8_t id; int8_t direction; float hornOffsetDeg; float maxSpeedDps; };
struct AxisState { float posDeg, velDps, load; uint8_t tempC, voltX10, fault; };

class ServoBus {
 public:
  ServoBus(ServoPort* port, const AxisCfg* axes, size_t n) : port_(port), axes_(axes), n_(n) {}

  // 1 軸の書き込みを送り、応答（status）の ERR が 0 であることまで確かめる。確かめられなければ false
  bool writeChecked(size_t i, uint8_t addr, const uint8_t* data, uint8_t len) {
    uint8_t pkt[MAX_TX];
    const size_t n = buildWrite(pkt, sizeof(pkt), axes_[i].id, addr, data, len);
    if (!n) return false;
    port_->flush();
    if (port_->write(pkt, n) != n) return false;
    uint8_t rx[MAX_RX], err = 0, dummy[1];
    const size_t got = port_->read(rx, 6, SERVO_RX_TIMEOUT_MS);
    return parseStatus(rx, got, axes_[i].id, dummy, 0, &err) == PARSE_OK && err == 0;
  }

  bool setTorqueLimitAll(uint16_t reg) {                      // 全軸の応答を確かめる。1 軸でも確かめられなければ false（**全軸に試みる**）
    const uint8_t d[2] = {(uint8_t)(reg & 0xFF), (uint8_t)(reg >> 8)};
    bool ok = true;
    for (size_t i = 0; i < n_; i++) ok = writeChecked(i, ADDR_TORQUE_LIMIT, d, 2) && ok;
    return ok;
  }

  bool setTorqueAll(bool on) {
    const uint8_t d = on ? 1 : 0;
    bool ok = true;
    for (size_t i = 0; i < n_; i++) ok = writeChecked(i, ADDR_TORQUE_ENABLE, &d, 1) && ok;
    return ok;
  }

  // 全軸の目標を同期書き込み（応答なし）。speedDps は軸ごとの上限（0 以下なら maxSpeedDps）
  bool syncWrite(const float* deg, const float* speedDps) {
    if (n_ == 0 || n_ > MAX_AXES) return false;
    uint8_t ids[MAX_AXES]; uint16_t pos[MAX_AXES], spd[MAX_AXES];
    for (size_t i = 0; i < n_; i++) {
      ids[i] = axes_[i].id;
      pos[i] = degToStep(deg[i], axes_[i].direction, axes_[i].hornOffsetDeg);
      const float v = (speedDps && speedDps[i] > 0.0f) ? speedDps[i] : axes_[i].maxSpeedDps;
      spd[i] = dpsToStepS(v);
    }
    uint8_t pkt[MAX_TX];
    const size_t n = buildSyncWritePos(pkt, sizeof(pkt), ids, pos, spd, (uint8_t)SERVO_ACCEL_REG, n_);
    return n && port_->write(pkt, n) == n;
  }

  // 1 軸の状態（56〜63）を読む。読めなければ false で、out は触らない（**0 で埋めない**）
  bool readAxis(size_t i, AxisState* out) {
    uint8_t pkt[8];
    const size_t n = buildRead(pkt, sizeof(pkt), axes_[i].id, ADDR_STATE, STATE_LEN);
    if (!n) return false;
    port_->flush();
    if (port_->write(pkt, n) != n) return false;
    uint8_t rx[MAX_RX], d[STATE_LEN], err = 0;
    const size_t got = port_->read(rx, MAX_RX, SERVO_RX_TIMEOUT_MS);
    if (parseStatus(rx, got, axes_[i].id, d, STATE_LEN, &err) != PARSE_OK) return false;
    const int pos = signMagnitude((uint16_t)(d[0] | (d[1] << 8)), 15);
    const int vel = signMagnitude((uint16_t)(d[2] | (d[3] << 8)), 15);
    out->posDeg = stepToDeg(pos, axes_[i].direction, axes_[i].hornOffsetDeg);
    out->velDps = (float)vel * 360.0f / (float)STEPS_PER_REV / (float)axes_[i].direction;
    out->load = (float)axes_[i].direction * decodeLoad((uint16_t)(d[4] | (d[5] << 8)));
    out->voltX10 = d[6]; out->tempC = d[7]; out->fault = err;
    return true;
  }

  size_t size() const { return n_; }

 private:
  ServoPort* port_;
  const AxisCfg* axes_;
  size_t n_;
};

}  // namespace sb

#include "servo_bus_vectors.h"

// ---- 自己検査: 自分の出力を SDK の正解と比べる（実機・別の C++ 環境で）。戻り値 = 失敗したビット（0 = すべて一致）----
inline uint16_t sbSelfTest() {
  using namespace sb;
  uint16_t bad = 0;
  uint8_t b[MAX_TX];
  size_t n = buildWrite1(b, sizeof(b), SBV_W1_ID, SBV_W1_ADDR, SBV_W1_VAL);
  if (n != SBV_WRITE1_LEN || memcmp(b, SBV_WRITE1, n) != 0) bad |= 1u << 0;
  n = buildWrite2(b, sizeof(b), SBV_W2_ID, SBV_W2_ADDR, SBV_W2_VAL);
  if (n != SBV_WRITE2_LEN || memcmp(b, SBV_WRITE2, n) != 0) bad |= 1u << 1;
  n = buildRead(b, sizeof(b), SBV_R_ID, SBV_R_ADDR, SBV_R_LEN);
  if (n != SBV_READ_STATE_LEN || memcmp(b, SBV_READ_STATE, n) != 0) bad |= 1u << 2;
  n = buildSyncWritePos(b, sizeof(b), SBV_SYNC_IDS, SBV_SYNC_POS, SBV_SYNC_SPEED, (uint8_t)SBV_SYNC_ACC, SBV_SYNC_N);
  if (n != SBV_SYNC_WRITE_LEN || memcmp(b, SBV_SYNC_WRITE, n) != 0) bad |= 1u << 3;
  uint8_t d[STATE_LEN], err = 0xFF;
  if (parseStatus(SBV_STATUS, SBV_STATUS_LEN, SBV_STATUS_ID, d, STATE_LEN, &err) != PARSE_OK || err != 0 || memcmp(d, SBV_STATUS_DATA, STATE_LEN) != 0) bad |= 1u << 4;
  uint8_t broken[SBV_STATUS_LEN];
  memcpy(broken, SBV_STATUS, SBV_STATUS_LEN);
  broken[SBV_STATUS_LEN - 1] ^= 0x01;                                                // チェックサムを壊したら拒否する
  if (parseStatus(broken, SBV_STATUS_LEN, SBV_STATUS_ID, d, STATE_LEN, &err) != PARSE_CHECKSUM) bad |= 1u << 5;
  if (degToStep(0.0f, 1, 0.0f) != CENTER_STEP || degToStep(1000.0f, 1, 0.0f) != SERVO_STEP_MAX || degToStep(-1000.0f, 1, 0.0f) != SERVO_STEP_MIN) bad |= 1u << 6;
  if (fabsf(stepToDeg((int)degToStep(30.0f, -1, 2.0f), -1, 2.0f) - 30.0f) > 0.1f) bad |= 1u << 7;      // 往復（direction・ホーン角つき）
  if (dpsToStepS(0.0f) != SERVO_SPEED_MIN_STEP_S || dpsToStepS(1e6f) != SERVO_SPEED_MAX_STEP_S) bad |= 1u << 8;
  // 偽サーボとの往復
  static const AxisCfg ax[3] = {{1, 1, 0.0f, 240.0f}, {2, -1, 0.0f, 240.0f}, {3, 1, 0.0f, 120.0f}};
  FakeServoPort fake;
  ServoBus bus(&fake, ax, 3);
  if (!bus.setTorqueLimitAll(SERVO_TORQUE_CEILING_REG) || fake.limit(1) != SERVO_TORQUE_CEILING_REG || fake.limit(3) != SERVO_TORQUE_CEILING_REG) bad |= 1u << 9;
  if (!bus.setTorqueAll(true) || !fake.torque(2)) bad |= 1u << 10;
  const float goal[3] = {10.0f, -20.0f, 5.0f};
  AxisState st;
  if (!bus.syncWrite(goal, 0) || !bus.readAxis(1, &st) || fabsf(st.posDeg - (-20.0f)) > 0.2f) bad |= 1u << 11;
  fake.setOnline(3, false);
  AxisState keep = {123.0f, 0, 0, 0, 0, 0};
  if (bus.readAxis(2, &keep) || keep.posDeg != 123.0f) bad |= 1u << 12;                // いない軸は false で、out を触らない
  return bad;
}
