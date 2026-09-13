// Serpens EX-1 駆動リンク ファームウェア（XIAO ESP32S3 / ESP32-S3）
//
// **PC が死んでも、この基板だけでヘビを止める。** それがこのファームの存在理由。
// 仕様は docs/link_protocol.md v1。判断の順序と時定数は serpens/link/device.py と同じにしてある。
// 参照実装（Python）は偽時計で全条件を検証済み: tests/test_phase2_device.py / test_phase2_safety.py
//
// **未検証**: 実機（ESP32 + STS3215）が無いため、コンパイルも書き込みもしていない。
//   サーボへの書き込み（writeServos）は配線が決まるまで空のまま。firmware/serpens_esp32/README.md 参照。
#include "config.h"
#include "link.h"

// ---- 状態 ----------------------------------------------------------------------------
static DeviceState gState = ST_DISARMED;
static StopReason  gReason = SR_BOOT;
static uint16_t    gBootId = 0;
static bool        gTorqueOn = true;      // 起動時は現在姿勢を保持（勝手に脱力しない）
static bool        gDriving = false;
static bool        gBreathing = false;

static uint16_t gLastSeq = 0;
static bool     gHasLastSeq = false;
static uint32_t gHbAt = 0;
static bool     gHbSeen = false;
static uint32_t gDriveAt = 0, gDriveUntil = 0, gHeadUntil = 0, gBodyUntil = 0;
static float    gTorqueRatio = 1.0f;          // TORQUE 指令（脱力の演出）
static uint32_t gNonces[16];
static uint8_t  gNonceIdx = 0;

static float gGoal[N_AXES];               // いまサーボへ書いている角度
static float gTarget[N_AXES];             // 補間の目標
static float gSpeed[N_AXES];              // [deg/s]
static float gPhase = 0.0f;               // 歩容の時間位相 [rad]

struct DriveCmd { float amp, spatial, freq, gamma; };
static DriveCmd gDrive = {0, 0, 0, 0};

static uint32_t gLastCtrl = 0, gLastTelem = 0;
static FrameReader gRx;

// ---- 小道具 --------------------------------------------------------------------------
static float clampDeg(int i, float deg) {
  if (deg < JOINTS[i].min_deg) return JOINTS[i].min_deg;
  if (deg > JOINTS[i].max_deg) return JOINTS[i].max_deg;
  return deg;
}

static bool hbFresh(uint32_t now) {
  return gHbSeen && (uint32_t)(now - gHbAt) <= HEARTBEAT_TIMEOUT_MS;
}

// サーボへ 9軸同期書き込み（docs/sts3215_registers.md の 41〜47 ブロック）
// TODO: 配線が決まるまで空。SCServo/ftservo 系ライブラリの SyncWritePosEx を使う予定。
static void writeServos(const float deg[N_AXES]) {
  (void)deg;
}

// トルク ON/OFF。**脱力は STOP(mode=disable) のときだけ**
static void setTorque(bool on) {
  gTorqueOn = on;
  // TODO: 実機では全軸のトルクスイッチを書く
}

static void holdHere() {
  for (int i = 0; i < N_AXES; i++) gTarget[i] = gGoal[i];
}

static void stopMotion(StopReason reason, bool latch, bool disarm, bool torqueOff) {
  if (gState == ST_EMERGENCY && !latch) return;    // ラッチ中の理由は上書きしない
  gDriving = false;
  gBreathing = false;
  gTorqueRatio = 1.0f;                             // 演出の脱力は停止で解除する
  holdHere();                                      // ホーム姿勢へは動かさない
  gReason = reason;
  if (latch) gState = ST_EMERGENCY;
  else if (disarm) gState = ST_DISARMED;
  if (torqueOff) setTorque(false);
  sendEvent(Serial, (uint8_t)gReason, (uint8_t)gState);
}

// ---- 受信 ----------------------------------------------------------------------------
static bool driveOk(const DriveCmd& d, uint16_t ttl) {
  float bodyMax = JOINTS[0].max_deg;
  for (int i = 1; i < N_BODY; i++) if (JOINTS[i].max_deg < bodyMax) bodyMax = JOINTS[i].max_deg;
  return ttl >= 1 && ttl <= DRIVE_TTL_MAX_MS
      && d.amp >= 0.0f && d.amp <= LIMIT_AMPLITUDE_DEG
      && d.spatial > 0.0f && d.spatial <= LIMIT_SPATIAL_DEG
      && fabsf(d.freq) <= LIMIT_TEMPORAL_HZ
      && fabsf(d.gamma) <= LIMIT_GAMMA_DEG
      && d.amp + fabsf(d.gamma) <= bodyMax;
}

static void handleFrame(uint32_t now) {
  const uint8_t type = gRx.type;
  const uint16_t seq = gRx.seq;
  if (!gRx.crcOk)                    { sendNack(Serial, seq, type, NACK_BAD_CRC); return; }
  if (gRx.version != LINK_VERSION)   { sendNack(Serial, seq, type, NACK_BAD_VERSION); return; }
  const int want = payloadLenFor(type);
  if (want < 0)                      { sendNack(Serial, seq, type, NACK_UNKNOWN_CMD); return; }
  if (gRx.len != want)               { sendNack(Serial, seq, type, NACK_BAD_LENGTH); return; }
  if (!seqIsForward(seq, gLastSeq, gHasLastSeq)) { sendNack(Serial, seq, type, NACK_STALE_SEQ); return; }
  gLastSeq = seq; gHasLastSeq = true;

  switch (type) {
    case CMD_HEARTBEAT:                              // DRIVE の期限はここでは延びない
      gHbAt = now; gHbSeen = true; sendAck(Serial, seq, type); return;
    case CMD_PING: sendAck(Serial, seq, type); return;
    case CMD_EMERGENCY:
      stopMotion(SR_EMERGENCY_CMD, true, true, false); sendAck(Serial, seq, type); return;
    case CMD_STOP:
      stopMotion(SR_OPERATOR_STOP, false, true, gRx.payload[0] == 1);
      sendAck(Serial, seq, type); return;
    case CMD_CLEAR_FAULT: {
      uint32_t nonce = rdU32(gRx.payload);
      for (int i = 0; i < 16; i++) if (gNonces[i] == nonce) { sendNack(Serial, seq, type, NACK_NONCE_REUSED); return; }
      gNonces[gNonceIdx] = nonce; gNonceIdx = (gNonceIdx + 1) % 16;
      gState = ST_DISARMED; gDriving = false;        // **待機へ戻すだけ。走行は再開しない**
      sendEvent(Serial, (uint8_t)gReason, (uint8_t)gState);
      sendAck(Serial, seq, type); return;
    }
    default: break;
  }
  if (gState == ST_EMERGENCY) { sendNack(Serial, seq, type, NACK_LATCHED); return; }
  if (type == CMD_DISARM) { stopMotion(SR_OPERATOR_STOP, false, true, false); sendAck(Serial, seq, type); return; }
  if (type == CMD_ARM) {
    if (!hbFresh(now)) { sendNack(Serial, seq, type, NACK_NO_HEARTBEAT); return; }
    gState = ST_ARMED; gReason = SR_NONE; setTorque(true); sendAck(Serial, seq, type); return;
  }
  if (type == CMD_LIMITS) { sendNack(Serial, seq, type, NACK_UNKNOWN_CMD); return; }  // Phase 9

  if (gState != ST_ARMED) { sendNack(Serial, seq, type, NACK_DISARMED); return; }
  if (!hbFresh(now))      { sendNack(Serial, seq, type, NACK_NO_HEARTBEAT); return; }

  if (type == CMD_DRIVE) {
    uint16_t ttl = rdU16(gRx.payload);
    DriveCmd d = {rdI16(gRx.payload + 2) / 10.0f, rdI16(gRx.payload + 4) / 10.0f,
                  rdI16(gRx.payload + 6) / 1000.0f, rdI16(gRx.payload + 8) / 10.0f};
    if (!driveOk(d, ttl)) { sendNack(Serial, seq, type, NACK_OUT_OF_RANGE); return; }
    gDrive = d; gDriveAt = now; gDriveUntil = now + ttl; gDriving = true; gReason = SR_NONE;
    sendAck(Serial, seq, type); return;
  }
  if (type == CMD_HEAD) {
    uint16_t ttl = rdU16(gRx.payload);
    float a[3] = {rdI16(gRx.payload + 2) / 10.0f, rdI16(gRx.payload + 4) / 10.0f,
                  rdI16(gRx.payload + 6) / 10.0f};
    float spd = rdU16(gRx.payload + 8) / 10.0f;
    bool ok = ttl >= 1 && ttl <= DRIVE_TTL_MAX_MS && spd > 0.0f && spd <= LIMIT_HEAD_SPEED_DPS;
    for (int k = 0; k < 3 && ok; k++) {
      const JointCfg& j = JOINTS[N_BODY + k];
      if (a[k] < j.min_deg || a[k] > j.max_deg) ok = false;
    }
    if (!ok) { sendNack(Serial, seq, type, NACK_OUT_OF_RANGE); return; }
    for (int k = 0; k < 3; k++) { gTarget[N_BODY + k] = a[k]; gSpeed[N_BODY + k] = spd; }
    gHeadUntil = now + ttl;
    sendAck(Serial, seq, type); return;
  }
  if (type == CMD_BODY) {
    if (gDriving) { sendNack(Serial, seq, type, NACK_BUSY); return; }   // 胴体の持ち主は一つ
    uint16_t ttl = rdU16(gRx.payload);
    float spd = rdU16(gRx.payload + 14) / 10.0f;
    bool ok = ttl >= 1 && ttl <= DRIVE_TTL_MAX_MS && spd > 0.0f && spd <= LIMIT_BODY_SPEED_DPS;
    float a[N_BODY];
    for (int i = 0; i < N_BODY && ok; i++) {
      a[i] = rdI16(gRx.payload + 2 + 2 * i) / 10.0f;
      if (a[i] < JOINTS[i].min_deg || a[i] > JOINTS[i].max_deg) ok = false;
    }
    if (!ok) { sendNack(Serial, seq, type, NACK_OUT_OF_RANGE); return; }
    for (int i = 0; i < N_BODY; i++) { gTarget[i] = a[i]; gSpeed[i] = spd; }
    gBodyUntil = now + ttl;
    sendAck(Serial, seq, type); return;
  }
  if (type == CMD_TORQUE) {
    float ratio = rdU16(gRx.payload) / 1000.0f;
    if (!(ratio > 0.0f && ratio <= 1.0f)) { sendNack(Serial, seq, type, NACK_OUT_OF_RANGE); return; }
    gTorqueRatio = ratio;                        // TODO: 実機ではトルク制限レジスタ（0〜1000）へ書く
    sendAck(Serial, seq, type); return;
  }
  if (type == CMD_POSE) {
    if (gRx.payload[0] != 0 || gDriving) { sendNack(Serial, seq, type, NACK_OUT_OF_RANGE); return; }
    for (int i = 0; i < N_AXES; i++) { gTarget[i] = clampDeg(i, HOME_DEG[i]); gSpeed[i] = JOINTS[i].max_speed_dps; }
    sendAck(Serial, seq, type); return;
  }
  if (type == CMD_BREATH) { gBreathing = gRx.payload[0] != 0; sendAck(Serial, seq, type); return; }
}

// ---- watchdog（PC が死んでも自分で止まる） -----------------------------------------------
static void watchdogs(uint32_t now) {
  uint8_t maxTemp = 0;
  bool anyFault = false;
  // TODO: 実機ではサーボから温度・fault を読む（docs/sts3215_registers.md の 56〜63）
  if (maxTemp > FAULT_TEMP_LIMIT_C)       { stopMotion(SR_OVERHEAT, true, true, false); return; }
  if (anyFault)                           { stopMotion(SR_SERVO_FAULT, true, true, false); return; }
  if (gHbSeen && !hbFresh(now))           { stopMotion(SR_HEARTBEAT_LOST, false, true, false); return; }
  if (gDriving && (int32_t)(now - gDriveUntil) > 0) { stopMotion(SR_DRIVE_TTL, false, false, false); return; }
  if (gState == ST_ARMED && gDriveAt && (uint32_t)(now - gDriveAt) > DRIVE_DISARM_MS) {
    stopMotion(SR_DRIVE_TTL, false, true, false);
  }
  if ((int32_t)(now - gHeadUntil) > 0) for (int k = 0; k < 3; k++) gTarget[N_BODY + k] = gGoal[N_BODY + k];
  if ((int32_t)(now - gBodyUntil) > 0 && !gDriving) for (int i = 0; i < N_BODY; i++) gTarget[i] = gGoal[i];
}

// ---- 制御（100Hz） --------------------------------------------------------------------
static void control(uint32_t now) {
  float dt = (now - gLastCtrl) / 1000.0f;
  gLastCtrl = now;
  if (gDriving) {
    gPhase += 2.0f * PI * gDrive.freq * dt;
    float big = gDrive.spatial * PI / 180.0f;
    for (int n = 0; n < N_BODY; n++) {                       // α(n,t) = A·sin(Ω·n+ω·t) + γ0·n/N
      float a = gDrive.amp * sinf(big * n + gPhase) + gDrive.gamma * n / (float)(N_BODY - 1);
      gGoal[n] = clampDeg(n, a);
    }
  }
  for (int i = 0; i < N_AXES; i++) {
    if (gDriving && i < N_BODY) continue;
    float step = gSpeed[i] * dt;
    float diff = clampDeg(i, gTarget[i]) - gGoal[i];
    if (diff > step) diff = step;
    if (diff < -step) diff = -step;
    gGoal[i] += diff;
  }
  if (gTorqueOn) writeServos(gGoal);                         // 保持中も送り続ける
}

static void sendTelemetry(uint32_t now) {
  uint8_t p[MAX_PAYLOAD];
  uint32_t up = now;
  p[0] = gBootId & 0xFF; p[1] = gBootId >> 8;
  p[2] = up & 0xFF; p[3] = (up >> 8) & 0xFF; p[4] = (up >> 16) & 0xFF; p[5] = (up >> 24) & 0xFF;
  p[6] = (uint8_t)gState; p[7] = (uint8_t)gReason;
  p[8] = gLastSeq & 0xFF; p[9] = gLastSeq >> 8;
  uint8_t fl = 0;
  if (gDriving) fl |= FL_DRIVING;
  if (gBreathing) fl |= FL_BREATHING;
  if (hbFresh(now)) fl |= FL_HEARTBEAT_OK;
  if (gDriving && (int32_t)(now - gDriveUntil) <= 0) fl |= FL_DRIVE_VALID;
  if (gTorqueOn) fl |= FL_TORQUE_ON;
  p[10] = fl; p[11] = N_AXES;
  int o = 12;
  for (int i = 0; i < N_AXES; i++) {
    int16_t pos = (int16_t)lroundf(gGoal[i] * 10.0f);        // TODO: 実機では読み値を入れる
    p[o++] = pos & 0xFF; p[o++] = (pos >> 8) & 0xFF;
    p[o++] = 0; p[o++] = 0;                                  // load
    p[o++] = 0;                                              // temp ℃
    p[o++] = 120;                                            // 12.0V
    p[o++] = 0;                                              // fault
    p[o++] = 0xFF; p[o++] = 0xFF;                            // current 不明
  }
  sendFrame(Serial, REP_TELEMETRY, p, (uint8_t)o);
}

// ---- Arduino --------------------------------------------------------------------------
void setup() {
  Serial.begin(921600);                                      // ネイティブ USB CDC（速度は無視される）
  gBootId = (uint16_t)(esp_random() & 0xFFFF);               // 起動ごとに変える（PC が再起動に気付く）
  for (int i = 0; i < N_AXES; i++) {
    gGoal[i] = HOME_DEG[i];                                  // TODO: 実機では現在角を読んでから入れる
    gTarget[i] = gGoal[i];
    gSpeed[i] = JOINTS[i].max_speed_dps;
  }
  gState = ST_DISARMED; gReason = SR_BOOT;                   // **必ず待機から始まる**
  gLastCtrl = millis(); gLastTelem = gLastCtrl;
}

void loop() {
  uint32_t now = millis();
  while (Serial.available()) {
    if (gRx.feed((uint8_t)Serial.read())) handleFrame(millis());
  }
  watchdogs(now);
  if ((uint32_t)(now - gLastCtrl) >= CONTROL_PERIOD_MS) control(now);
  if ((uint32_t)(now - gLastTelem) >= TELEMETRY_PERIOD_MS) { gLastTelem = now; sendTelemetry(now); }
}
