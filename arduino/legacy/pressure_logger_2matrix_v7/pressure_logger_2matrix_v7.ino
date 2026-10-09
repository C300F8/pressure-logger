/* ============================================================
 * Atmospheric Pressure Logger — 2x 8×8 MAX7219 + I2C EEPROM
 * ============================================================
 * Hardware:
 *   - Arduino Uno / Pro Mini
 *   - BMP580 barometric pressure sensor (I2C)
 *   - 2× 8×8 MAX7219 LED matrix module (FC-16 chain, 16 columns total)
 *   - AT24C32 / AT24C256 I2C EEPROM
 *
 * Features:
 *   - 12-hour pressure history rendered as a line graph (cols 0–11)
 *   - Trend arrow in cols 12–15 (↑ rising / → steady / ↓ falling)
 *   - Fixed ±10 hPa display window — calm days look calm,
 *     real storms look dramatic (honest scale)
 *   - 0.5 hPa rounding to suppress 1-pixel display jitter
 *   - Power-loss protection: full history saved to EEPROM after
 *     every measurement and restored automatically on boot
 *
 * ── Changelog ───────────────────────────────────────────────
 *   v1  — Initial version: 2-matrix display, BMP580, line graph
 *   v2  — Added AT24C EEPROM save/restore for power-loss protection
 *   v3  — Fixed loadFromEEPROM(): storedCount >= 1 guard added
 *   v4  — Fixed px(): device index was "3 - (col/8)" → produced
 *           indices 3 & 2 (non-existent for 2-device chain).
 *           Corrected to "(DEVICES-1) - (col/8)" → 1 & 0 ✓
 *   v5  — Sensor warm-up: 3 × 10 s dummy reads added
 *   v6  — Code cleanup: full comments, stale "4-in-1" removed
 *   v7  — Replaced auto-scale with fixed ±SCALE_HALF_RANGE window
 *           centered on the mean of stored history.
 * ============================================================ */

#include <Wire.h>
#include <Adafruit_BMP5xx.h>
#include <LedControl.h>

// ── SPI Pins for MAX7219 chain ───────────────────────────────
#define DIN_PIN   11
#define CLK_PIN   13
#define CS_PIN    10

#define DEVICES     2

#define EEPROM_ADDR 0x50

#define TEST_MODE false

#if TEST_MODE
  #define INTERVAL_MS   900000UL   // 15 minutes
#else
  #define INTERVAL_MS  3600000UL   // 1 hour  (production)
#endif

#define HISTORY_LEN 12
float hist[HISTORY_LEN];
int   dataCount = 0;

unsigned long lastMs = 0;

const float SCALE_HALF_RANGE = 10.0f;  // hPa

LedControl      lc(DIN_PIN, CLK_PIN, CS_PIN, DEVICES);
Adafruit_BMP5xx bmp;


// ════════════════════════════════════════════════════════════
//  EEPROM LOW-LEVEL HELPERS  (AT24C series, 16-bit addressing)
// ════════════════════════════════════════════════════════════

void writeEEPROM(int address, byte val) {
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write((int)(address >> 8));
  Wire.write((int)(address & 0xFF));
  Wire.write(val);
  Wire.endTransmission();
  delay(5);
}

byte readEEPROM(int address) {
  byte rdata = 0xFF;
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write((int)(address >> 8));
  Wire.write((int)(address & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_ADDR, 1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}


// ════════════════════════════════════════════════════════════
//  DATA PERSISTENCE
// ════════════════════════════════════════════════════════════

void saveToEEPROM() {
  Serial.println(F("Saving history to EEPROM..."));
  writeEEPROM(0, (byte)dataCount);
  byte* p = (byte*)(void*)&hist;
  for (int i = 0; i < (int)sizeof(hist); i++) {
    writeEEPROM(10 + i, p[i]);
  }
}

void loadFromEEPROM() {
  Serial.println(F("Checking EEPROM for stored data..."));
  byte storedCount = readEEPROM(0);

  if (storedCount >= 1 && storedCount <= HISTORY_LEN) {
    dataCount = (int)storedCount;
    byte* p = (byte*)(void*)&hist;
    for (int i = 0; i < (int)sizeof(hist); i++) {
      p[i] = readEEPROM(10 + i);
    }
    Serial.print(F("Restored "));
    Serial.print(dataCount);
    Serial.println(F(" readings from memory."));
  } else {
    Serial.println(F("No valid history found in EEPROM. Starting fresh."));
    dataCount = 0;
  }
}


// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);
  Wire.begin();
  Serial.println(F("\n== Pressure Logger v7 (2-Matrix + EEPROM) =="));

  if (!bmp.begin((uint8_t)0x46)) {
    Serial.println(F("0x46 failed, trying 0x47..."));
    if (!bmp.begin((uint8_t)0x47)) {
      Serial.println(F("ERROR: BMP580 not found! Check wiring."));
      while (true) delay(1000);
    }
  }

  bmp.setTemperatureOversampling(BMP5_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP5_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP5_IIR_FILTER_COEFF_15);
  bmp.setOutputDataRate(BMP5_ODR_01_HZ);
  Serial.println(F("BMP580 initialized."));

  Serial.println(F("Warming up sensor (30 s)..."));
  for (int i = 0; i < 3; i++) {
    bmp.performReading();
    Serial.print(F("Stabilizing... "));
    Serial.print(i + 1);
    Serial.println(F("/3"));
    delay(10000);
  }
  Serial.println(F("Sensor stabilized."));

  for (int d = 0; d < DEVICES; d++) {
    lc.shutdown(d, false);
    lc.setIntensity(d, 2);
    lc.clearDisplay(d);
  }

  loadFromEEPROM();
  if (dataCount > 0) redraw();

  measure();
  lastMs = millis();
}


void loop() {
  if (millis() - lastMs >= INTERVAL_MS) {
    lastMs = millis();
    measure();
  }
}


// ════════════════════════════════════════════════════════════
//  MEASUREMENT
// ════════════════════════════════════════════════════════════
void measure() {
  if (!bmp.performReading()) {
    Serial.println(F("ERROR: Failed to read BMP580!"));
    return;
  }

  float rawP = bmp.pressure;
  float p    = round(rawP * 2.0f) / 2.0f;

  if (p < 300.0f || p > 1250.0f) {
    Serial.print(F("REJECTED — invalid reading: "));
    Serial.println(p);
    return;
  }

  if (dataCount < HISTORY_LEN) {
    hist[dataCount++] = p;
  } else {
    for (int i = 0; i < HISTORY_LEN - 1; i++) hist[i] = hist[i + 1];
    hist[HISTORY_LEN - 1] = p;
  }

  Serial.print(F("P = "));
  Serial.print(p, 1);
  Serial.print(F(" hPa  | readings stored: "));
  Serial.println(dataCount);

  saveToEEPROM();
  redraw();
}


// ════════════════════════════════════════════════════════════
//  DISPLAY HELPERS
// ════════════════════════════════════════════════════════════

void px(int col, int row, bool on) {
  if (col < 0 || col > 15 || row < 0 || row > 7) return;
  int dev = (DEVICES - 1) - (col / 8);  // 1 or 0  ✓
  int mc  = col % 8;
  lc.setLed(dev, row, mc, on);
}

void redraw() {
  for (int d = 0; d < DEVICES; d++) lc.clearDisplay(d);
  if (dataCount == 0) return;

  float sum = 0;
  for (int i = 0; i < dataCount; i++) sum += hist[i];
  float mean  = sum / dataCount;
  float lo    = mean - SCALE_HALF_RANGE;
  float hi    = mean + SCALE_HALF_RANGE;
  float range = hi - lo;

  int startCol = HISTORY_LEN - dataCount;
  for (int i = 0; i < dataCount; i++) {
    int col       = startCol + i;
    int rowOffset = (int)((hist[i] - lo) / range * 7.0f + 0.5f);
    rowOffset     = constrain(rowOffset, 0, 7);
    px(col, 7 - rowOffset, true);
  }

  drawTrend();
}

void drawTrend() {
  if (dataCount < 2) return;

  int   refIdx = max(0, dataCount - 3);
  float delta  = hist[dataCount - 1] - hist[refIdx];

  const float riseThreshold =  1.0f;
  const float fallThreshold = -1.0f;

  if (delta >= riseThreshold) {
    px(14, 1, true);
    px(13, 2, true); px(14, 2, true); px(15, 2, true);
    px(14, 3, true); px(14, 4, true); px(14, 5, true);
  } else if (delta <= fallThreshold) {
    px(14, 2, true); px(14, 3, true); px(14, 4, true);
    px(13, 5, true); px(14, 5, true); px(15, 5, true);
    px(14, 6, true);
  } else {
    px(12, 3, true); px(13, 3, true); px(14, 3, true); px(15, 3, true);
    px(14, 2, true); px(14, 4, true);
  }
}
