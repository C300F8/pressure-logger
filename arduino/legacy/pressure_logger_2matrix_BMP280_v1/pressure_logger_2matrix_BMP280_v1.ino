/* ============================================================
 * Atmospheric Pressure Logger — 2x 8×8 MAX7219 + I2C EEPROM
 *                              BMP280 SENSOR VARIANT
 * ============================================================
 * Hardware:
 *   - Arduino Uno / Pro Mini
 *   - BMP280 barometric pressure sensor (I2C)
 *       Works with bare BMP280 modules AND AHT20+BMP280 combo
 *       boards — only the BMP280 half is used here. The AHT20
 *       (at 0x38) is independent and simply ignored.
 *   - 2× 8×8 MAX7219 LED matrix module (FC-16 chain, 16 columns total)
 *   - AT24C32 / AT24C256 I2C EEPROM
 *
 * Wiring (BMP280 → Arduino Uno):
 *   VDD → 3.3V       (bare BMP280 boards)
 *         3.3V or 5V (AHT20+BMP280 combo boards — onboard regulator)
 *   GND → GND
 *   SDA → A4
 *   SCL → A5
 *
 * Required library (install via Library Manager):
 *   Adafruit BMP280 Library   by Adafruit
 *   (also pulls in Adafruit Unified Sensor as a dependency)
 *
 * Features (all carried over from BMP580 v7):
 *   - 12-hour pressure history rendered as a line graph (cols 0–11)
 *   - Trend arrow in cols 12–15 (↑ rising / → steady / ↓ falling)
 *   - Fixed ±10 hPa display window — calm days look calm,
 *     real storms look dramatic (honest scale)
 *   - 0.5 hPa rounding to suppress 1-pixel display jitter
 *   - Power-loss protection: full history saved to EEPROM after
 *     every measurement and restored automatically on boot
 *
 * ── Changelog ───────────────────────────────────────────────
 *   v1 (BMP280 fork of BMP580 v7)
 *      - Library:  Adafruit_BMP5xx  →  Adafruit_BMP280
 *      - Object:   Adafruit_BMP5xx bmp  →  Adafruit_BMP280 bmp
 *      - I2C addr: 0x46 / 0x47  →  0x76 / 0x77
 *      - Init:     four bmp.setXxx() calls collapsed into a single
 *                  bmp.setSampling(mode, tOS, pOS, filter, standby).
 *      - Read:     bmp.performReading() + bmp.pressure
 *                    →  bmp.takeForcedMeasurement() + bmp.readPressure()
 *                  BMP280.readPressure() returns Pa, so we divide
 *                  by 100.0f to get hPa (BMP5xx returned hPa directly).
 *      - Mode:     MODE_FORCED — sensor wakes for one measurement
 *                  per hour, then sleeps. Matches v7's "one reading
 *                  per cycle" intent and saves power.
 *      - Sanity range, EEPROM layout, 0.5 hPa rounding, fixed-window
 *        scale, FC-16 column mapping, trend-arrow rendering — all
 *        unchanged from BMP580 v7.
 *
 *   Inherited from BMP580 v7 (BMP280 fork keeps all of these):
 *      - Fixed ±SCALE_HALF_RANGE window centered on mean(history),
 *        replacing the misleading auto-scale of earlier versions.
 *      - Sensor warm-up: 3 × 10 s dummy reads on boot.
 *      - EEPROM save/restore with storedCount range guard.
 *      - px() uses dev = (DEVICES - 1) - (col / 8) — correct for any
 *        chain length, not the hard-coded "3 - ..." of older sketches.
 * ============================================================ */

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <LedControl.h>

// ── SPI Pins for MAX7219 chain ───────────────────────────────
#define DIN_PIN   11
#define CLK_PIN   13
#define CS_PIN    10

// ── Device count ─────────────────────────────────────────────
// 2 physical 8×8 modules chained together → 16 display columns
#define DEVICES     2

// ── I2C address for AT24C32 / AT24C256 EEPROM ───────────────
// Default address when all address pins (A0-A2) are tied low
#define EEPROM_ADDR 0x50

// ── BMP280 I2C addresses ─────────────────────────────────────
// 0x76 — SDO pin low or no SDO pin (most modules, including the
//        AHT20+BMP280 combo board you have)
// 0x77 — SDO pin high (some bare BMP280 modules ship like this)
#define BMP280_ADDR_PRIMARY   0x76
#define BMP280_ADDR_FALLBACK  0x77

// ── Measurement interval ─────────────────────────────────────
// Flip TEST_MODE to true for faster 15-minute cycles during development
#define TEST_MODE false

#if TEST_MODE
  #define INTERVAL_MS   900000UL   // 15 minutes
#else
  #define INTERVAL_MS  3600000UL   // 1 hour  (production)
#endif

// ── History buffer ───────────────────────────────────────────
// 12 columns used for the graph; cols 12–15 are reserved for the trend arrow.
// At 1 reading/hour this covers a 12-hour window.
#define HISTORY_LEN 12
float hist[HISTORY_LEN];  // Circular pressure history (hPa)
int   dataCount = 0;      // Number of valid readings currently stored

unsigned long lastMs = 0; // Timestamp of the last measurement

// ── Display scale ─────────────────────────────────────────────
// Fixed window: ±SCALE_HALF_RANGE hPa around the mean of history.
// Total window = 2 × SCALE_HALF_RANGE = 20 hPa.
//
// Why fixed instead of auto-scale?
//   Auto-scale made a calm day (0.5 hPa variation) look as dramatic
//   as a stormy day (8 hPa variation) — completely misleading.
//   With a fixed window, the same visual slope always represents the
//   same rate of pressure change, making readings honest and comparable.
//
// Increase SCALE_HALF_RANGE if you live in a high-variability region
// and dots frequently clamp to the top or bottom row.
const float SCALE_HALF_RANGE = 10.0f;  // hPa

// ── Library objects ──────────────────────────────────────────
LedControl      lc(DIN_PIN, CLK_PIN, CS_PIN, DEVICES);
Adafruit_BMP280 bmp;


// ════════════════════════════════════════════════════════════
//  EEPROM LOW-LEVEL HELPERS  (AT24C series, 16-bit addressing)
// ════════════════════════════════════════════════════════════

// Write a single byte to the given EEPROM address.
// The 5 ms delay is mandatory — the EEPROM needs it to complete
// its internal write cycle before the next I2C transaction.
void writeEEPROM(int address, byte val) {
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write((int)(address >> 8));    // Address high byte (MSB)
  Wire.write((int)(address & 0xFF));  // Address low byte  (LSB)
  Wire.write(val);
  Wire.endTransmission();
  delay(5);
}

// Read a single byte from the given EEPROM address.
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

// Save the current history buffer to EEPROM.
// Layout:
//   Address  0      → dataCount (1 byte)
//   Address 10 …    → hist[] cast to raw bytes (4 bytes × HISTORY_LEN)
// A 10-byte gap at the start leaves room for future metadata.
// Note: 12 floats × 4 bytes = 48 bytes; total footprint = 58 bytes,
// well within AT24C32 (4 KB) or AT24C256 (32 KB) capacity.
void saveToEEPROM() {
  Serial.println(F("Saving history to EEPROM..."));

  writeEEPROM(0, (byte)dataCount);

  // Cast the float array to a byte pointer and write it byte-by-byte
  byte* p = (byte*)(void*)&hist;
  for (int i = 0; i < (int)sizeof(hist); i++) {
    writeEEPROM(10 + i, p[i]);
  }
}

// Restore history from EEPROM on boot.
// A fresh / erased EEPROM returns 0xFF at address 0, which falls
// outside the valid range [1, HISTORY_LEN] and is correctly rejected.
void loadFromEEPROM() {
  Serial.println(F("Checking EEPROM for stored data..."));
  byte storedCount = readEEPROM(0);

  if (storedCount >= 1 && storedCount <= HISTORY_LEN) {
    // Valid count found — restore the float array
    dataCount = (int)storedCount;
    byte* p = (byte*)(void*)&hist;
    for (int i = 0; i < (int)sizeof(hist); i++) {
      p[i] = readEEPROM(10 + i);
    }
    Serial.print(F("Restored "));
    Serial.print(dataCount);
    Serial.println(F(" readings from memory."));
  } else {
    // 0xFF (blank) or any other garbage value → start fresh
    Serial.println(F("No valid history found in EEPROM. Starting fresh."));
    dataCount = 0;
  }
}


// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);
  Wire.begin();  // Start I2C for both BMP280 and EEPROM
  Serial.println(F("\n== Pressure Logger v1 (2-Matrix + EEPROM, BMP280) =="));

  // ── BMP280 init ───────────────────────────────────────────
  // Try 0x76 (default for most modules including AHT20+BMP280 combo)
  // first, fall back to 0x77 (SDO pulled high on some bare modules).
  if (!bmp.begin(BMP280_ADDR_PRIMARY)) {
    Serial.println(F("0x76 failed, trying 0x77..."));
    if (!bmp.begin(BMP280_ADDR_FALLBACK)) {
      Serial.println(F("ERROR: BMP280 not found! Check wiring."));
      Serial.println(F("  - SDA = A4, SCL = A5"));
      Serial.println(F("  - VDD = 3.3V (or 5V on combo boards with regulator)"));
      Serial.println(F("  - Run an I2C scanner to confirm the address"));
      while (true) delay(1000);  // Halt
    }
  }

  // High-quality pressure settings for slow environmental logging.
  // BMP280 setSampling() takes all five parameters in one call:
  //   1) Mode             — FORCED: wake on demand, sleep between reads
  //   2) Temp oversampling  — ×2  (used only for pressure compensation)
  //   3) Pressure oversampling — ×16 (max — best resolution / lowest noise)
  //   4) IIR filter coefficient — 16 (strongest smoothing)
  //   5) Standby (ignored in FORCED mode but required by the API)
  //
  // FORCED mode mirrors the BMP580 v7 behaviour: the sensor takes
  // one measurement per cycle, then sleeps. Lowest power draw and
  // perfectly matched to a once-per-hour logger.
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
                  Adafruit_BMP280::SAMPLING_X2,    // temp oversampling
                  Adafruit_BMP280::SAMPLING_X16,   // pressure oversampling
                  Adafruit_BMP280::FILTER_X16,     // IIR coefficient 16
                  Adafruit_BMP280::STANDBY_MS_1);  // unused in FORCED mode
  Serial.println(F("BMP280 initialized."));

  // ── Sensor warm-up ────────────────────────────────────────
  // Defensive carry-over from the BMP580 sketch where the first
  // readings after power-on could be wildly wrong (e.g. 1305 hPa).
  // The BMP280 is generally well-behaved from the first read, but
  // three dummy measurements with 10 s gaps cost nothing and let
  // any thermal transients settle.
  Serial.println(F("Warming up sensor (30 s)..."));
  for (int i = 0; i < 3; i++) {
    bmp.takeForcedMeasurement();
    (void)bmp.readPressure();   // discard the value
    Serial.print(F("Stabilizing... "));
    Serial.print(i + 1);
    Serial.println(F("/3"));
    delay(10000);
  }
  Serial.println(F("Sensor stabilized."));

  // ── LED matrix init ───────────────────────────────────────
  for (int d = 0; d < DEVICES; d++) {
    lc.shutdown(d, false);   // Wake up from power-save mode
    lc.setIntensity(d, 2);   // Low brightness (0–15); adjust to taste
    lc.clearDisplay(d);
  }

  // ── Restore and display saved history ─────────────────────
  loadFromEEPROM();
  if (dataCount > 0) redraw();  // Show restored graph before first new reading

  // Take the first live reading immediately
  measure();
  lastMs = millis();
}


// ════════════════════════════════════════════════════════════
//  MAIN LOOP  (non-blocking)
// ════════════════════════════════════════════════════════════
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
  // Trigger one forced-mode measurement. Returns false if the I2C
  // transaction fails or the sensor never reports a finished sample.
  if (!bmp.takeForcedMeasurement()) {
    Serial.println(F("ERROR: BMP280 measurement timed out!"));
    return;
  }

  // BMP280.readPressure() returns pascals — divide by 100 for hPa.
  // (BMP5xx's bmp.pressure was already in hPa; this is the only
  //  unit-conversion change vs. the BMP580 v7 sketch.)
  float rawP = bmp.readPressure() / 100.0f;

  // Round to the nearest 0.5 hPa to prevent 1-pixel jitter on the graph
  float p = round(rawP * 2.0f) / 2.0f;

  // Sanity check — reject physically impossible values
  if (p < 300.0f || p > 1250.0f) {
    Serial.print(F("REJECTED — invalid reading: "));
    Serial.println(p);
    return;
  }

  // Append to history, or shift-left and overwrite the last slot when full
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

  saveToEEPROM();  // Persist immediately in case of power loss
  redraw();
}


// ════════════════════════════════════════════════════════════
//  DISPLAY HELPERS
// ════════════════════════════════════════════════════════════

// Light or clear a single LED by logical display column (0–15).
//
// FC-16 chain order (right-to-left from the Arduino's perspective):
//   Columns  0– 7  → device 1  (left  matrix)
//   Columns  8–15  → device 0  (right matrix)
//
// Formula: dev = (DEVICES - 1) - (col / 8)  →  1, 0  ✓
void px(int col, int row, bool on) {
  if (col < 0 || col > 15 || row < 0 || row > 7) return;

  int dev = (DEVICES - 1) - (col / 8);  // 1 or 0  ✓
  int mc  = col % 8;                     // column within that device

  lc.setLed(dev, row, mc, on);
}

// ── Line graph renderer ──────────────────────────────────────
void redraw() {
  for (int d = 0; d < DEVICES; d++) lc.clearDisplay(d);
  if (dataCount == 0) return;

  // ── Fixed-window scale ────────────────────────────────────
  // Center the window on the mean of all stored readings.
  // Using mean (not the latest value) lets the window track slow
  // long-term drift (e.g. a multi-day high-pressure system) while
  // staying stable hour-to-hour on calm days.
  float sum = 0;
  for (int i = 0; i < dataCount; i++) sum += hist[i];
  float mean  = sum / dataCount;
  float lo    = mean - SCALE_HALF_RANGE;   // e.g. 1013 - 10 = 1003 hPa
  float hi    = mean + SCALE_HALF_RANGE;   // e.g. 1013 + 10 = 1023 hPa
  float range = hi - lo;                   // always 20.0 hPa

  // ── Plot points (right-aligned inside cols 0–11) ──────────
  // Older readings are on the left; the newest is always at col 11.
  // Out-of-window readings are clamped to row 0 (top) or row 7 (bottom).
  int startCol = HISTORY_LEN - dataCount;
  for (int i = 0; i < dataCount; i++) {
    int col       = startCol + i;
    int rowOffset = (int)((hist[i] - lo) / range * 7.0f + 0.5f);
    rowOffset     = constrain(rowOffset, 0, 7);

    px(col, 7 - rowOffset, true);  // row 7 = bottom of display
  }

  drawTrend();
}

// ── Trend arrow (cols 12–15, rows 1–6) ───────────────────────
// Compares the latest reading against the one 3 hours ago.
// Thresholds: ±1.0 hPa over 3 hours → rise / fall; otherwise steady.
void drawTrend() {
  if (dataCount < 2) return;

  int   refIdx = max(0, dataCount - 3);         // 3-hour lookback
  float delta  = hist[dataCount - 1] - hist[refIdx];

  const float riseThreshold =  1.0f;  // hPa — tune to taste
  const float fallThreshold = -1.0f;

  if (delta >= riseThreshold) {
    // ↑ UP ARROW — rising pressure (improving weather)
    px(14, 1, true);                                               // Tip
    px(13, 2, true); px(14, 2, true); px(15, 2, true);            // Arrowhead
    px(14, 3, true); px(14, 4, true); px(14, 5, true);            // Stem

  } else if (delta <= fallThreshold) {
    // ↓ DOWN ARROW — falling pressure (deteriorating weather)
    px(14, 2, true); px(14, 3, true); px(14, 4, true);            // Stem
    px(13, 5, true); px(14, 5, true); px(15, 5, true);            // Arrowhead
    px(14, 6, true);                                               // Tip

  } else {
    // → STEADY ARROW — stable pressure
    px(12, 3, true); px(13, 3, true); px(14, 3, true); px(15, 3, true); // Shaft
    px(14, 2, true); px(14, 4, true);    // Arrowhead tips (V-shape on right)
  }
}
