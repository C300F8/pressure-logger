/* ============================================================
 * QMC6310 Magnetometer Logger — GY-271  (виробнича версія)
 * ============================================================
 * Chip:     QST QMC6310  (I2C addr 0x2C, chip ID 0x80)
 * Hardware: Arduino Uno R3
 *
 * Wiring:
 *   VCC → 3.3V або 5V  |  GND → GND
 *   SCL → A5            |  SDA → A4
 *
 * Особливість чіпа: CTRL1 приймає записи тільки одразу після
 * soft reset. Тому перед кожним вимірюванням виконується повна
 * реініціалізація (~300 мс). Для 15-хв логера це несуттєво.
 *
 * Вивід: X / Y / Z у мікротеслах (uT) + курс (deg) + модуль |B|
 * Використовується лише ASCII — без µ та ° щоб не було ⸮ у Serial.
 * ============================================================ */

#include <Wire.h>
#include <math.h>

#define ADDR            0x2C
#define QMC6310_CHIP_ID 0x80

#define REG_CHIP_ID 0x00
#define REG_XOUT_L  0x01
#define REG_CTRL1   0x09
#define REG_CTRL2   0x0A
#define REG_CTRL3   0x0B
#define REG_UNLOCK  0x29

#define SENSITIVITY_UT 0.03f   // uT на LSB при ±8 Gauss

// ── Timing ───────────────────────────────────────────────────
#define TEST_MODE false  // true = 10 с для налагодження

#if TEST_MODE
  #define INTERVAL_MS  10000UL
#else
  #define INTERVAL_MS  900000UL   // 15 хвилин
#endif

unsigned long lastMs = 0;

// ════════════════════════════════════════════════════════════
//  I2C
// ════════════════════════════════════════════════════════════

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
  delay(5);
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

// ════════════════════════════════════════════════════════════
//  REINIT + TRIGGER
//  Єдина послідовність що гарантовано приймає CTRL1=0x19.
//  Підтверджено дослідним шляхом: CTRL1 приймається виключно
//  після soft reset (CTRL2=0x01 + 200мс пауза).
// ════════════════════════════════════════════════════════════

bool reinitAndTrigger() {
  if (readReg(REG_CHIP_ID) != QMC6310_CHIP_ID) return false;

  writeReg(REG_CTRL2, 0x01);   // soft reset
  delay(200);
  writeReg(REG_UNLOCK, 0x06);  // unlock
  writeReg(REG_CTRL3,  0x01);  // SET/RESET period
  writeReg(REG_CTRL2,  0x04);  // range ±8 Gauss
  writeReg(REG_CTRL1,  0x19);  // single-shot trigger

  return (readReg(REG_CTRL1) == 0x19);
}

// ════════════════════════════════════════════════════════════
//  PRINT HELPERS  (тільки ASCII — без µ та °)
// ════════════════════════════════════════════════════════════

void printTime() {
  unsigned long s  = millis() / 1000;
  unsigned long h  = s / 3600;
  unsigned long m  = (s % 3600) / 60;
  unsigned long sc = s % 60;
  if (h  < 10) Serial.print('0'); Serial.print(h);  Serial.print(':');
  if (m  < 10) Serial.print('0'); Serial.print(m);  Serial.print(':');
  if (sc < 10) Serial.print('0'); Serial.print(sc);
}

void printUT(float v) {
  if (v >= 0) Serial.print('+');
  Serial.print(v, 2);
  Serial.print(F(" uT"));
}

// ════════════════════════════════════════════════════════════
//  MEASURE
// ════════════════════════════════════════════════════════════

void measure() {
  if (!reinitAndTrigger()) {
    Serial.println(F("ERROR: reinit failed"));
    return;
  }

  delay(100);  // чекаємо завершення вимірювання (STATUS на цьому чіпі = 0x00 завжди)

  Wire.beginTransmission(ADDR);
  Wire.write(REG_XOUT_L);
  Wire.endTransmission(false);
  Wire.requestFrom(ADDR, (uint8_t)6);

  if (Wire.available() < 6) { Serial.println(F("ERROR: read failed")); return; }

  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = Wire.read();

  int16_t rawX = (int16_t)((b[1] << 8) | b[0]);
  int16_t rawY = (int16_t)((b[3] << 8) | b[2]);
  int16_t rawZ = (int16_t)((b[5] << 8) | b[4]);

  float xUT = rawX * SENSITIVITY_UT;
  float yUT = rawY * SENSITIVITY_UT;
  float zUT = rawZ * SENSITIVITY_UT;
  float tot = sqrt(xUT*xUT + yUT*yUT + zUT*zUT);
  float hdg = atan2(yUT, xUT) * 180.0f / PI;
  if (hdg < 0) hdg += 360.0f;

  printTime();
  Serial.print(F("  | ")); printUT(xUT);
  Serial.print(F(" | ")); printUT(yUT);
  Serial.print(F(" | ")); printUT(zUT);
  Serial.print(F(" | ")); Serial.print(hdg, 1); Serial.print(F(" deg"));
  Serial.print(F("  |B|=")); Serial.print(tot, 1); Serial.println(F(" uT"));
}

// ════════════════════════════════════════════════════════════
//  SETUP / LOOP
// ════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(300);

  Serial.println(F("\n== QMC6310 Magnetometer Logger =="));

  if (readReg(REG_CHIP_ID) != QMC6310_CHIP_ID) {
    Serial.println(F("ERROR: chip not found at 0x2C. Check wiring."));
    while (true) delay(1000);
  }

  Serial.println(F("Chip OK (ID=0x80) | +-8 Gauss | 0.03 uT/LSB"));
#if TEST_MODE
  Serial.println(F("Mode: TEST (10 sec interval)"));
#else
  Serial.println(F("Mode: PRODUCTION (15 min interval)"));
#endif
  Serial.println(F("---------------------------------------------------------------------"));
  Serial.println(F("  Time    |     X     |     Y     |     Z     | Hdg       |B|        "));
  Serial.println(F("---------------------------------------------------------------------"));

  measure();
  lastMs = millis();
}

void loop() {
  if (millis() - lastMs >= INTERVAL_MS) {
    lastMs = millis();
    measure();
  }
}
