// Serpens 頭の XIAO ESP32S3（Sense）— 骨組み（LB-E-082）。**コンパイルだけ。書き込み・通電はしていない。**
// 仕様: ENTRY-K-0001 (1)(2)（GPIO・ToF の起動順）、`serpens/hw/head_io.py`（行プロトコル）。HARDWARE_UNVERIFIED。
//
// できること（骨組み）:
//   - USB CDC の行プロトコル: P → "P,ok,<fw>"、S 行を 10 Hz で出す（ToF = 0 = 測定不能、タッチ = 0）。
//   - ToF ×2 の起動順（XSHUT を両方 L → 左だけ上げてアドレス変更 → 右を上げる）と、モデル ID の確認。**ranging は未実装**（ST の ULD / ライブラリが要る）。
//   - LED ×2 の PWM（LEDC。`L,<左>,<右>` = 斜め LED の試験用の PROPOSED コマンド。E / M は目の RGB 用で、この基板には無い）。
// できないこと（決まっていない・基板に無い）: タッチ ×2、フードの端のビット（案 H）、SG90 の駆動 — ピンが無い。フードのビットを PC へ返す行（案: "H,<down>,<up>,<seq>"）は Python 側に未定義。
#include <Wire.h>
#include "head_config.h"
#include "head_logic.h"

static uint32_t g_seq = 0;
static uint32_t g_lastSensorMs = 0;
static char g_line[HEAD_LINE_MAX];
static size_t g_len = 0;
static bool g_tofOk[2] = {false, false};

// ---- I2C（16 bit 指標・MSB 先頭。データシート 4.2）----------------------------------------------------------
static bool tofRead8(uint8_t addr, uint16_t reg, uint8_t* out) {
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, 1) != 1) return false;
  *out = (uint8_t)Wire.read();
  return true;
}

static bool tofWrite8(uint8_t addr, uint16_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool tofIsVl53l1x(uint8_t addr) {
  uint8_t id = 0, type = 0;
  return tofRead8(addr, TOF_REG_MODEL_ID, &id) && tofRead8(addr, TOF_REG_MODEL_ID + 1, &type) && id == TOF_MODEL_ID && type == TOF_MODULE_TYPE;
}

// 起動順（K-0001 (2)）: 両方スタンバイ → 左だけ起こして既定アドレスで確認し、アドレスを変える → 右を起こす（右は既定のまま）。
static void tofBegin() {
  pinMode(HEAD_PIN_XSHUT_L, OUTPUT);
  pinMode(HEAD_PIN_XSHUT_R, OUTPUT);
  digitalWrite(HEAD_PIN_XSHUT_L, LOW);
  digitalWrite(HEAD_PIN_XSHUT_R, LOW);
  delay(TOF_BOOT_DELAY_MS);
  digitalWrite(HEAD_PIN_XSHUT_L, HIGH);
  delay(TOF_BOOT_DELAY_MS);
  if (tofIsVl53l1x(TOF_ADDR_DEFAULT) && tofWrite8(TOF_ADDR_DEFAULT, TOF_REG_I2C_ADDR, TOF_ADDR_L)) {
    g_tofOk[0] = tofIsVl53l1x(TOF_ADDR_L);            // 新アドレスで ID を読み直せたときだけ「左 = 使える」
  }
  digitalWrite(HEAD_PIN_XSHUT_R, HIGH);
  delay(TOF_BOOT_DELAY_MS);
  g_tofOk[1] = tofIsVl53l1x(TOF_ADDR_R);
}

// ---- LED（LEDC）------------------------------------------------------------------------------------------
// esp32 core 2.x（tools/build_firmware.py で確認した 2.0.17）は ledcSetup + ledcAttachPin + ledcWrite(チャンネル)、3.x は ledcAttach + ledcWrite(ピン)。
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void ledBegin() {
  ledcAttach(HEAD_PIN_LED_L, LED_PWM_HZ, LED_PWM_BITS);
  ledcAttach(HEAD_PIN_LED_R, LED_PWM_HZ, LED_PWM_BITS);
}
static void ledWrite(int side, uint8_t duty) { ledcWrite(side == 0 ? HEAD_PIN_LED_L : HEAD_PIN_LED_R, duty); }
#else
static void ledBegin() {
  ledcSetup(0, LED_PWM_HZ, LED_PWM_BITS);
  ledcSetup(1, LED_PWM_HZ, LED_PWM_BITS);
  ledcAttachPin(HEAD_PIN_LED_L, 0);
  ledcAttachPin(HEAD_PIN_LED_R, 1);
}
static void ledWrite(int side, uint8_t duty) { ledcWrite(side, duty); }
#endif

static void handleLine(const char* line) {
  HlCommand c = hlParseLine(line);
  char out[48];
  switch (c.kind) {
    case HL_PING:
      hlFormatPing(out, sizeof out, HEAD_FW_VERSION);
      Serial.print(out);
      break;
    case HL_LED:
      ledWrite(0, hlLedDuty(c.a, LED_DUTY_MAX));
      ledWrite(1, hlLedDuty(c.b, LED_DUTY_MAX));
      break;
    case HL_EYE:
    case HL_MODE:
    default:
      break;                                         // この基板に目の RGB は無い。E / M は解釈だけして何もしない（未知の行は黙って捨てる）
  }
}

void setup() {
  Serial.begin(HEAD_BAUD);
  ledBegin();
  ledWrite(0, 0);                                    // 起動時は消灯（点けるのは指令のあとだけ）
  ledWrite(1, 0);
  Wire.begin(HEAD_PIN_SDA, HEAD_PIN_SCL, HEAD_I2C_HZ);
  tofBegin();
}

void loop() {
  while (Serial.available() > 0) {
    int ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      g_line[g_len] = '\0';
      if (g_len > 0) handleLine(g_line);
      g_len = 0;
    } else if (g_len < HEAD_LINE_MAX - 1) {
      g_line[g_len++] = (char)ch;
    } else {
      g_len = 0;                                     // 長すぎる行は捨てる
    }
  }
  uint32_t now = millis();
  if (now - g_lastSensorMs >= 1000UL / HEAD_SENSOR_HZ) {
    g_lastSensorMs = now;
    char buf[64];
    hlFormatSensor(buf, sizeof buf, now, 0 /* ranging 未実装 = 測定不能 */, false, false, g_seq++);
    Serial.print(buf);
  }
}
