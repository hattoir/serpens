// Serpens EX-1 駆動リンク ファームウェア（XIAO ESP32S3 / ESP32-S3）
//
// **PC が死んでも、この基板だけでヘビを止める。** それがこのファームの存在理由。
// 仕様は docs/link_protocol.md v2。状態は7つで、判断の順序と時定数は
// serpens/link/device.py（Virtual ESP32）と同じにしてある。
// 参照実装は偽時計で検証済み: tests/test_phase2_*.py / tests/test_phase3_link_robot.py
//
// **HARDWARE_UNVERIFIED**: 実機（ESP32 + STS3215）が無いため、コンパイルも書き込みもしていない。
//   サーボの読み書き（writeServos / readServos）は配線が決まるまで空。README.md 参照。
#include "config.h"
#include "link.h"

// ---- 状態 ----------------------------------------------------------------------------
static DeviceState gState = ST_BOOT;
static StopReason  gReason = SR_BOOT;
static uint16_t    gBootId = 0;
static bool        gTorqueOn = true;      // 起動時は現在姿勢を保持（勝手に脱力しない）
static bool        gBreathing = false;
static float       gTorqueRatio = 1.0f;   // TORQUE 指令（脱力の演出）。安全上限に対する割合

static uint16_t gLastSeq = 0, gLastDriveSeq = 0;
static bool     gHasLastSeq = false;
static uint32_t gHbAt = 0;
static bool     gHbSeen = false;
static uint32_t gDriveAt = 0, gDriveUntil = 0, gHeadUntil = 0, gBodyUntil = 0;
static bool     gDriveSeen = false;
static uint32_t gNonces[16];
static uint8_t  gNonceIdx = 0;
static uint16_t gOverruns = 0, gLoopPeriodUs = CONTROL_PERIOD_MS * 1000;

static float gGoal[N_AXES];               // いまサーボへ書いている角度（指令値）
static float gTarget[N_AXES];             // 補間の目標
static float gSpeed[N_AXES];              // [deg/s]
static float gPhase = 0.0f;               // 歩容の時間位相 [rad]

// サーボから読んだ値（**実測**。指令値と混ぜない）
static float   gMeasPos[N_AXES], gMeasVel[N_AXES], gMeasLoad[N_AXES];
static uint8_t gMeasTemp[N_AXES], gMeasVolt[N_AXES], gMeasFault[N_AXES];
static bool    gMeasOk[N_AXES];

struct DriveCmd { float amp, spatial, freq, gamma; };
static DriveCmd gDrive = {0, 0, 0, 0};

static uint32_t gLastCtrl = 0, gLastTelem = 0;
static FrameReader gRx;

// ---- 小道具 --------------------------------------------------------------------------
static bool driving() { return gState == ST_DRIVING; }

static float clampDeg(int i, float deg) {
  if (deg < JOINTS[i].min_deg) return JOINTS[i].min_deg;
  if (deg > JOINTS[i].max_deg) return JOINTS[i].max_deg;
  return deg;
}

// ---- Floor Watch の頭（J7）の範囲・速さ（config.h の FW_PITCH_*。既定はオフ。serpens/motion/pitch_guard.py と同じ判断）----
static uint16_t gPitchRejects = 0, gPitchClamps = 0, gPitchSpeedCaps = 0;   // 記録（TODO: テレメトリ v3 で PC へ出す。今は機体内のカウンタだけ）

static bool pitchInStop(float d) { return d >= FW_PITCH_STOP_LO_DEG && d <= FW_PITCH_STOP_HI_DEG; }
static bool pitchInBand(float d) { return d > FW_PITCH_SOFT_HI_DEG || d < FW_PITCH_SOFT_LO_DEG; }

static float pitchClamp(float d) {                          // 姿勢プリセット・目標の最終確認
  if (!FW_PITCH_GUARD_ENABLED || pitchInStop(d)) return d;
  gPitchClamps++;
  return d < FW_PITCH_STOP_LO_DEG ? FW_PITCH_STOP_LO_DEG : FW_PITCH_STOP_HI_DEG;
}

// pos から target へ向かうときの速さの上限。窓の外（区間）では上限、窓の内側から区間へ入るときは窓の端で上限に一致する傾きで減速する
static float pitchSpeedLimit(float pos, float target, float speed) {
  if (!FW_PITCH_GUARD_ENABLED || target == pos) return speed;
  if (pitchInBand(pos)) return speed < FW_PITCH_NEAR_SPEED_DPS ? speed : FW_PITCH_NEAR_SPEED_DPS;
  const bool up = target > pos;
  const float edge = up ? FW_PITCH_SOFT_HI_DEG : FW_PITCH_SOFT_LO_DEG;
  if ((up && target > edge) || (!up && target < edge)) {
    const float dist = fabsf(edge - pos);
    const float cap = sqrtf(FW_PITCH_NEAR_SPEED_DPS * FW_PITCH_NEAR_SPEED_DPS + 2.0f * FW_PITCH_DECEL_DPS2 * dist);
    if (cap < speed) { gPitchSpeedCaps++; return cap; }
  }
  return speed;
}

static bool hbFresh(uint32_t now) {
  return gHbSeen && (uint32_t)(now - gHbAt) <= HEARTBEAT_TIMEOUT_MS;
}

static bool anyServoMissing() {
  for (int i = 0; i < N_AXES; i++) if (!gMeasOk[i]) return true;
  return false;
}

// サーボへ 9軸同期書き込み（docs/sts3215_registers.md の 41〜47 ブロック）
// TODO: 配線が決まるまで空。SCServo/ftservo 系ライブラリの SyncWritePosEx を使う予定。
static void writeServos(const float deg[N_AXES]) { (void)deg; }

// サーボから位置・速度・負荷・温度・電圧・fault を読む（56〜63 ブロック）
// TODO: 未実装。読めなかった軸は gMeasOk[i] = false のままにする（**0 で埋めない**）
static void readServos() {
  for (int i = 0; i < N_AXES; i++) gMeasOk[i] = false;
}

static void setTorque(bool on) {
  gTorqueOn = on;
  // TODO: 実機では全軸のトルクスイッチを書く
}

static void setTorqueRatio(float ratio) {
  gTorqueRatio = ratio;
  // TODO: 実機ではトルク制限レジスタ（48番地）へ ratio × 安全上限 を書く
}

static void holdHere() {
  for (int i = 0; i < N_AXES; i++) gTarget[i] = gGoal[i];
}

static void setState(DeviceState to, StopReason reason) {
  gState = to;
  gReason = reason;
  sendEvent(Serial, (uint8_t)gReason, (uint8_t)gState);
}

// 停止。**現在の出力を保持する**（ホーム姿勢へ動かさない）
static void stopMotion(StopReason reason, DeviceState to) {
  if (gState == ST_EMERGENCY_LATCHED && to != ST_EMERGENCY_LATCHED) return;  // ラッチを上書きしない
  gBreathing = false;
  setTorqueRatio(1.0f);                            // 演出の脱力は停止で解除する
  holdHere();
  if (to == ST_TORQUE_DISABLED) setTorque(false);
  setState(to, reason);
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
      && d.amp + fabsf(d.gamma) <= bodyMax;        // 合成しても operational limit 内
}

static void handleMotion(uint8_t type, uint16_t seq, uint32_t now) {
  if (!stateArmed(gState)) { sendNack(Serial, seq, type, NACK_DISARMED); return; }
  if (!hbFresh(now))       { sendNack(Serial, seq, type, NACK_NO_HEARTBEAT); return; }

  if (type == CMD_DRIVE) {
    uint16_t ttl = rdU16(gRx.payload);
    DriveCmd d = {rdI16(gRx.payload + 2) / 10.0f, rdI16(gRx.payload + 4) / 10.0f,
                  rdI16(gRx.payload + 6) / 1000.0f, rdI16(gRx.payload + 8) / 10.0f};
    if (!driveOk(d, ttl)) { sendNack(Serial, seq, type, NACK_OUT_OF_RANGE); return; }
    gDrive = d; gDriveAt = now; gDriveSeen = true; gDriveUntil = now + ttl; gLastDriveSeq = seq;
    if (gState != ST_DRIVING) setState(ST_DRIVING, SR_NONE);
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
    if (FW_PITCH_GUARD_ENABLED && ok && !pitchInStop(a[FW_PITCH_AXIS - N_BODY])) { gPitchRejects++; ok = false; }   // Floor Watch の範囲の外は拒否（状態を変えない）
    if (!ok) { sendNack(Serial, seq, type, NACK_OUT_OF_RANGE); return; }
    for (int k = 0; k < 3; k++) {
      gTarget[N_BODY + k] = a[k];
      gSpeed[N_BODY + k] = (N_BODY + k == FW_PITCH_AXIS) ? pitchSpeedLimit(gGoal[FW_PITCH_AXIS], a[k], spd) : spd;
    }
    gHeadUntil = now + ttl;
    sendAck(Serial, seq, type); return;
  }
  if (type == CMD_BODY) {
    if (driving()) { sendNack(Serial, seq, type, NACK_BUSY); return; }   // 胴体の持ち主は一つ
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
    setTorqueRatio(ratio);
    sendAck(Serial, seq, type); return;
  }
  if (type == CMD_POSE) {
    if (gRx.payload[0] != 0 || driving()) { sendNack(Serial, seq, type, NACK_OUT_OF_RANGE); return; }
    for (int i = 0; i < N_AXES; i++) {
      float t = clampDeg(i, HOME_DEG[i]);
      if (i == FW_PITCH_AXIS) t = pitchClamp(t);             // Floor Watch の範囲へクランプ（既定オフ）
      gTarget[i] = t; gSpeed[i] = JOINTS[i].max_speed_dps;
    }
    sendAck(Serial, seq, type); return;
  }
  if (type == CMD_BREATH) { gBreathing = gRx.payload[0] != 0; sendAck(Serial, seq, type); return; }
  sendNack(Serial, seq, type, NACK_UNKNOWN_CMD);   // 取りこぼしを別の指令として実行しない
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
      gHbAt = now; gHbSeen = true;
      if (gState == ST_BOOT) setState(ST_DISARMED, SR_BOOT);
      sendAck(Serial, seq, type); return;
    case CMD_PING: sendAck(Serial, seq, type); return;
    case CMD_EMERGENCY:
      stopMotion(SR_EMERGENCY_CMD, ST_EMERGENCY_LATCHED); sendAck(Serial, seq, type); return;
    case CMD_STOP:
      stopMotion(SR_OPERATOR_STOP, gRx.payload[0] == 1 ? ST_TORQUE_DISABLED : ST_DISARMED);
      sendAck(Serial, seq, type); return;
    case CMD_CLEAR_FAULT: {
      uint32_t nonce = rdU32(gRx.payload);
      for (int i = 0; i < 16; i++) if (gNonces[i] == nonce) { sendNack(Serial, seq, type, NACK_NONCE_REUSED); return; }
      gNonces[gNonceIdx] = nonce; gNonceIdx = (gNonceIdx + 1) % 16;
      setTorque(true);
      setState(ST_DISARMED, gReason);                // **待機へ戻すだけ。走行は再開しない**
      sendAck(Serial, seq, type); return;
    }
    default: break;
  }
  if (gState == ST_EMERGENCY_LATCHED) { sendNack(Serial, seq, type, NACK_LATCHED); return; }
  if (type == CMD_DISARM) { stopMotion(SR_OPERATOR_STOP, ST_DISARMED); sendAck(Serial, seq, type); return; }
  if (type == CMD_ARM) {
    // **どの起動の機体を ARM するのかを名指しさせる。** 再起動とすれ違った ARM は通らない
    if (rdU16(gRx.payload) != gBootId) { sendNack(Serial, seq, type, NACK_STALE_BOOT); return; }
    if (gState == ST_FAULT_HOLD) { sendNack(Serial, seq, type, NACK_BUSY); return; }
    if (!hbFresh(now))           { sendNack(Serial, seq, type, NACK_NO_HEARTBEAT); return; }
    setTorque(true);
    setState(ST_ARMED_HOLD, SR_NONE);
    sendAck(Serial, seq, type); return;
  }
  if (type == CMD_LIMITS) { sendNack(Serial, seq, type, NACK_UNKNOWN_CMD); return; }  // Phase 9
  handleMotion(type, seq, now);
}

// ---- watchdog（PC が死んでも自分で止まる） -----------------------------------------------
static void watchdogs(uint32_t now) {
  uint8_t maxTemp = 0;
  bool anyFault = false;
  for (int i = 0; i < N_AXES; i++) {
    if (!gMeasOk[i]) continue;
    if (gMeasTemp[i] > maxTemp) maxTemp = gMeasTemp[i];
    if (gMeasFault[i]) anyFault = true;
  }
  if (maxTemp > FAULT_TEMP_LIMIT_C)  { stopMotion(SR_OVERHEAT, ST_EMERGENCY_LATCHED); return; }
  if (anyFault)                      { stopMotion(SR_SERVO_FAULT, ST_EMERGENCY_LATCHED); return; }
  if (stateArmed(gState) && anyServoMissing()) { stopMotion(SR_SERVO_FAULT, ST_FAULT_HOLD); return; }
  if (gHbSeen && !hbFresh(now) && stateArmed(gState)) {
    stopMotion(SR_HEARTBEAT_LOST, ST_FAULT_HOLD); return;     // USB 断・PC 強制終了もここ
  }
  if (gState == ST_FAULT_HOLD && hbFresh(now) && !anyServoMissing()) {
    setState(ST_DISARMED, gReason); return;                   // 復帰は待機まで
  }
  if (driving() && (int32_t)(now - gDriveUntil) > 0) {
    stopMotion(SR_DRIVE_TTL, ST_ARMED_HOLD); return;          // 保持のまま（条件 4）
  }
  if (gState == ST_ARMED_HOLD && gDriveSeen && (uint32_t)(now - gDriveAt) > DRIVE_DISARM_MS) {
    stopMotion(SR_DRIVE_TTL, ST_DISARMED); return;
  }
  if ((int32_t)(now - gHeadUntil) > 0) for (int k = 0; k < 3; k++) gTarget[N_BODY + k] = gGoal[N_BODY + k];
  if ((int32_t)(now - gBodyUntil) > 0 && !driving()) for (int i = 0; i < N_BODY; i++) gTarget[i] = gGoal[i];
}

// ---- 制御（100Hz） --------------------------------------------------------------------
static void control(uint32_t now) {
  uint32_t dtMs = now - gLastCtrl;
  gLastCtrl = now;
  gLoopPeriodUs = (uint16_t)(dtMs * 1000 > AGE_MAX_MS ? AGE_MAX_MS : dtMs * 1000);
  if (dtMs > CONTROL_PERIOD_MS * 3 / 2) gOverruns++;
  float dt = dtMs / 1000.0f;
  if (driving()) {
    gPhase += 2.0f * PI * gDrive.freq * dt;
    float big = gDrive.spatial * PI / 180.0f;
    for (int n = 0; n < N_BODY; n++) {                       // α(n,t) = A·sin(Ω·n+ω·t) + γ0·n/N
      float a = gDrive.amp * sinf(big * n + gPhase) + gDrive.gamma * n / (float)(N_BODY - 1);
      gGoal[n] = clampDeg(n, a);
    }
  }
  for (int i = 0; i < N_AXES; i++) {
    if (driving() && i < N_BODY) continue;
    float tgt = clampDeg(i, gTarget[i]);
    float spd = gSpeed[i];
    if (i == FW_PITCH_AXIS) {                                // Floor Watch の頭: 範囲と、窓の端の手前の速さ（既定オフ）
      tgt = pitchClamp(tgt);
      spd = pitchSpeedLimit(gGoal[i], tgt, spd);
    }
    float step = spd * dt;
    float diff = tgt - gGoal[i];
    if (diff > step) diff = step;
    if (diff < -step) diff = -step;
    gGoal[i] += diff;
  }
  if (gTorqueOn) writeServos(gGoal);                         // 保持中も送り続ける
  readServos();                                              // 実測値を取り込む
}

static uint16_t ageMs(uint32_t at, bool seen, uint32_t now) {
  if (!seen) return AGE_MAX_MS;
  uint32_t d = now - at;
  return (uint16_t)(d > AGE_MAX_MS ? AGE_MAX_MS : d);
}

static void sendTelemetry(uint32_t now) {
  uint8_t p[MAX_PAYLOAD];
  int o = 0;
  putU16(p, o, gBootId);
  uint32_t up = now;
  p[o++] = up & 0xFF; p[o++] = (up >> 8) & 0xFF; p[o++] = (up >> 16) & 0xFF; p[o++] = (up >> 24) & 0xFF;
  p[o++] = (uint8_t)gState;
  p[o++] = (uint8_t)gReason;
  putU16(p, o, gLastSeq);
  putU16(p, o, gLastDriveSeq);
  uint16_t fl = 0;
  if (driving()) fl |= FL_DRIVING;
  if (gBreathing) fl |= FL_BREATHING;
  if (hbFresh(now)) fl |= FL_HEARTBEAT_OK;
  if (driving() && (int32_t)(now - gDriveUntil) <= 0) fl |= FL_DRIVE_VALID;
  if (gTorqueOn) fl |= FL_TORQUE_ON;
  if (stateArmed(gState)) fl |= FL_ARMED;
  if (gState == ST_EMERGENCY_LATCHED) fl |= FL_EMERGENCY_LATCHED;
  if (anyServoMissing()) fl |= FL_SERVO_MISSING;
  if (gOverruns) fl |= FL_OVERRUN;
  putU16(p, o, fl);                                          // **FL_SIMULATED は実機では立てない**
  putU16(p, o, ageMs(gHbAt, gHbSeen, now));
  putU16(p, o, ageMs(gDriveAt, gDriveSeen, now));
  putU16(p, o, (driving() && (int32_t)(gDriveUntil - now) > 0) ? (uint16_t)(gDriveUntil - now) : 0);
  putU16(p, o, gLoopPeriodUs);
  putU16(p, o, gOverruns);
  p[o++] = (uint8_t)SRC_HARDWARE;                            // **実機の値**
  int nAxes = 0;
  for (int i = 0; i < N_AXES; i++) if (gMeasOk[i]) nAxes++;
  p[o++] = (uint8_t)nAxes;
  for (int i = 0; i < N_AXES; i++) {
    if (!gMeasOk[i]) continue;                               // 読めない軸は返さない（0 で埋めない）
    int16_t pos = (int16_t)lroundf(gMeasPos[i] * 10.0f);
    int16_t vel = (int16_t)lroundf(gMeasVel[i] * 10.0f);
    int16_t load = (int16_t)lroundf(gMeasLoad[i] * 1000.0f);
    p[o++] = pos & 0xFF; p[o++] = (pos >> 8) & 0xFF;
    p[o++] = vel & 0xFF; p[o++] = (vel >> 8) & 0xFF;
    p[o++] = load & 0xFF; p[o++] = (load >> 8) & 0xFF;
    p[o++] = gMeasTemp[i];
    p[o++] = gMeasVolt[i];
    p[o++] = gMeasFault[i];
    p[o++] = 0xFF; p[o++] = 0xFF;                            // current 不明（電流センサ無し）
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
    gMeasOk[i] = false;
  }
  gState = ST_BOOT; gReason = SR_BOOT;                       // **必ず BOOT から始まる**
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
