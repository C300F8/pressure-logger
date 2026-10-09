/* ============================================================
 * Atmospheric Pressure Logger — 0.91" SSD1306 OLED + I2C EEPROM
 *                              BMP280 SENSOR VARIANT
 * ============================================================
 * Hardware:
 *   - Arduino Uno / Pro Mini
 *   - BMP280 (I2C) — bare module or AHT20+BMP280 combo
 *   - 0.91" SSD1306 OLED, 128 x 32, I2C
 *   - AT24Cxx EEPROM (any size; see EEPROM_ADDR_BYTES below)
 *
 * Wiring (everything shares the same I2C bus):
 *   BMP280   VDD->3.3V/5V  GND->GND  SDA->A4  SCL->A5
 *   OLED     VCC->3.3V/5V  GND->GND  SDA->A4  SCL->A5
 *   EEPROM   VCC->5V       GND->GND  SDA->A4  SCL->A5
 *
 *   I2C addresses (no conflicts):
 *     BMP280  0x76 (or 0x77)
 *     OLED    0x3C (or 0x3D)
 *     EEPROM  0x50  (AT24C01..C16 also alias 0x51..0x53 to
 *                    select 256-byte pages — we only use page 0)
 *
 * Required libraries (Library Manager):
 *   Adafruit BMP280 Library    by Adafruit
 *   Adafruit SSD1306           by Adafruit
 *   Adafruit GFX Library       by Adafruit  (dependency)
 *   Adafruit BusIO             by Adafruit  (dependency)
 *
 * Display layout (128 x 32 px, monochrome):
 *   y=0..7   header  — "1013.5 hPa" + trend arrow
 *   y=8      1-px horizontal separator
 *   y=10..31 graph — 22 px tall, ±3 hPa adaptive window
 *   y=20     sparse dotted line = LATEST reading (= window center)
 *
 * Changelog:
 *   v1   OLED port of 2-matrix BMP280 v1.
 *   v2   FILTER_OFF, software oversampling, no 0.5 hPa rounding.
 *   v3   Adaptive window centered on latest reading, ±3 hPa span,
 *        out-of-window points skipped instead of clamped.
 *
 *   v4 (EEPROM addressing fix for AT24C08 + boot self-test):
 *     - Added EEPROM_ADDR_BYTES define (1 for AT24C01..C16,
 *       2 for AT24C32 and larger).
 *           v1..v3 hard-coded the 2-byte AT24C32+ form. With an
 *           AT24C08 in the socket, the second "address" byte was
 *           interpreted as data and clobbered the data byte that
 *           followed it. Every save effectively wrote one byte,
 *           always at physical address 1. Every load failed the
 *           range check and reported "No valid history" on boot.
 *
 *     - Added eepromSelfTest() called once during setup:
 *           Writes 0xA5 to a scratch byte (addr 5, between
 *           dataCount@0 and hist[]@10), reads it back, reports
 *           OK / FAIL on Serial. Quick sanity check after any
 *           wiring or chip change — if you see FAIL, double-check
 *           EEPROM_ADDR_BYTES against your chip.
 *
 *     - Added value-range sanity check inside loadFromEEPROM:
 *           If storedCount is in range but the first restored
 *           pressure value is outside [800, 1200] hPa, treat the
 *           whole blob as garbage and start fresh. Catches the
 *           narrow case where stale bytes happen to encode a
 *           plausible-looking count but nonsense floats.
 *
 *     For users with AT24C32 / AT24C64 / AT24C256 etc.:
 *         change EEPROM_ADDR_BYTES from 1 to 2. Nothing else
 *         needs to be touched.
 * ============================================================ */

#include <Wire.h>
#include <math.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── OLED: SSD1306, 128 x 32, I2C ─────────────────────────────
#define SCREEN_WIDTH        128
#define SCREEN_HEIGHT        32
#define OLED_RESET           -1
#define OLED_ADDR_PRIMARY   0x3C
#define OLED_ADDR_FALLBACK  0x3D

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ── EEPROM (AT24Cxx) ─────────────────────────────────────────
#define EEPROM_ADDR 0x50

// EEPROM_ADDR_BYTES — how many bytes to send for the memory address.
//
//   1   AT24C01, C02, C04, C08, C16   (1 KB..2 KB chips,
//                                      256-byte pages)
//   2   AT24C32, C64, C128, C256,     (4 KB..64 KB chips,
//       C512, C1024                    flat 16-bit address space)
//
// Current build: AT24C08  -->  1
#define EEPROM_ADDR_BYTES 1

// ── BMP280 I2C addresses ─────────────────────────────────────
#define BMP280_ADDR_PRIMARY   0x76
#define BMP280_ADDR_FALLBACK  0x77

// ── Measurement interval ─────────────────────────────────────
#define TEST_MODE false

#if TEST_MODE
  #define INTERVAL_MS   900000UL   // 15 min
#else
  #define INTERVAL_MS  3600000UL   // 1 h (production)
#endif

// ── Software oversampling (v2) ───────────────────────────────
#define N_SAMPLES         8
#define SAMPLE_DELAY_MS 100

// ── History buffer ───────────────────────────────────────────
// 24 readings * 1 h = 24-hour window.
// EEPROM footprint: 1 byte (dataCount) + 96 bytes (hist[]) = 97 bytes.
// Fits comfortably in any AT24Cxx, including the smallest C01 (128 B)
// if you ever swap one in — though C01's 128 B is too small.
// Minimum required size: AT24C02 (256 bytes) and up.
#define HISTORY_LEN 24
float hist[HISTORY_LEN];
int   dataCount = 0;
unsigned long lastMs = 0;

// ── Display scale (v3 — adaptive window) ─────────────────────
const float SCALE_HALF_RANGE = 3.0f;

// ── OLED layout constants ────────────────────────────────────
const uint8_t SEPARATOR_Y    = 8;
const uint8_t GRAPH_TOP      = 10;
const uint8_t GRAPH_BOTTOM   = 31;
const uint8_t PX_PER_READING = 5;
const uint8_t ARROW_X        = 115;
const uint8_t ARROW_Y        = 0;

// ── Library object ───────────────────────────────────────────
Adafruit_BMP280 bmp;


// ════════════════════════════════════════════════════════════
//  EEPROM LOW-LEVEL HELPERS  (v4: 1-byte OR 2-byte addressing)
// ════════════════════════════════════════════════════════════

void writeEEPROM(int address, byte val) {
  Wire.beginTransmission(EEPROM_ADDR);
#if EEPROM_ADDR_BYTES == 2
  Wire.write((byte)(address >> 8));     // MSB — AT24C32 and larger only
#endif
  Wire.write((byte)(address & 0xFF));   // LSB (or sole byte for AT24C08)
  Wire.write(val);
  Wire.endTransmission();
  delay(5);                             // mandatory page-write delay
}

byte readEEPROM(int address) {
  byte rdata = 0xFF;
  Wire.beginTransmission(EEPROM_ADDR);
#if EEPROM_ADDR_BYTES == 2
  Wire.write((byte)(address >> 8));
#endif
  Wire.write((byte)(address & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_ADDR, 1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}

// Boot-time sanity check. Writes a magic byte to a scratch address
// (between dataCount@0 and hist[]@10), reads it back, reports OK/FAIL.
// If this fails, EEPROM_ADDR_BYTES is almost certainly wrong for
// the chip in the socket.
bool eepromSelfTest() {
  const int  TEST_ADDR    = 5;
  const byte TEST_PATTERN = 0xA5;

  // Save whatever was there so the test is non-destructive
  byte saved = readEEPROM(TEST_ADDR);

  writeEEPROM(TEST_ADDR, TEST_PATTERN);
  byte readback = readEEPROM(TEST_ADDR);

  // Restore
  writeEEPROM(TEST_ADDR, saved);

  if (readback == TEST_PATTERN) {
    Serial.println(F("EEPROM self-test OK."));
    return true;
  }

  Serial.print(F("EEPROM self-test FAILED: wrote 0x"));
  Serial.print(TEST_PATTERN, HEX);
  Serial.print(F(", read 0x"));
  Serial.println(readback, HEX);
  Serial.println(F("  -> check EEPROM_ADDR_BYTES against your chip"));
  Serial.println(F("     (1 for AT24C01..C16, 2 for AT24C32 and up)"));
  return false;
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

  if (storedCount < 1 || storedCount > HISTORY_LEN) {
    Serial.println(F("No valid history found in EEPROM. Starting fresh."));
    dataCount = 0;
    return;
  }

  // Provisionally restore
  dataCount = (int)storedCount;
  byte* p = (byte*)(void*)&hist;
  for (int i = 0; i < (int)sizeof(hist); i++) {
    p[i] = readEEPROM(10 + i);
  }

  // v4: value-range sanity check on the first stored reading.
  // Catches the case where storedCount happens to look valid but
  // the rest of the blob is garbage.
  if (hist[0] < 800.0f || hist[0] > 1200.0f || isnan(hist[0])) {
    Serial.println(F("EEPROM data failed sanity check (bad float) -- starting fresh."));
    dataCount = 0;
    return;
  }

  Serial.print(F("Restored "));
  Serial.print(dataCount);
  Serial.println(F(" readings from memory."));
}


// ════════════════════════════════════════════════════════════
//  OLED HELPERS
// ════════════════════════════════════════════════════════════

void showStatus(const __FlashStringHelper* line1,
                const __FlashStringHelper* line2) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 6);
  display.print(line1);
  if (line2) {
    display.setCursor(0, 18);
    display.print(line2);
  }
  display.display();
}

float computeDelta() {
  if (dataCount < 2) return 0.0f;
  int refIdx = max(0, dataCount - 3);
  return hist[dataCount - 1] - hist[refIdx];
}

void drawArrow(float delta) {
  const int x = ARROW_X;
  const int y = ARROW_Y;

  if (delta >= 1.0f) {
    display.fillTriangle(x + 4, y,
                         x,     y + 2,
                         x + 8, y + 2,
                         SSD1306_WHITE);
    display.fillRect(x + 3, y + 2, 3, 5, SSD1306_WHITE);

  } else if (delta <= -1.0f) {
    display.fillRect(x + 3, y, 3, 5, SSD1306_WHITE);
    display.fillTriangle(x + 4, y + 6,
                         x,     y + 4,
                         x + 8, y + 4,
                         SSD1306_WHITE);
  } else {
    display.fillRect(x, y + 2, 6, 3, SSD1306_WHITE);
    display.fillTriangle(x + 5, y,
                         x + 5, y + 6,
                         x + 8, y + 3,
                         SSD1306_WHITE);
  }
}

void drawHeader() {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  if (dataCount == 0) {
    display.print(F("Waiting for data..."));
    return;
  }

  display.print(hist[dataCount - 1], 1);
  display.print(F(" hPa"));
  drawArrow(computeDelta());
}

void drawGraph() {
  if (dataCount == 0) return;

  // v3: window centered on the LATEST reading
  float center = hist[dataCount - 1];
  float lo     = center - SCALE_HALF_RANGE;
  float hi     = center + SCALE_HALF_RANGE;
  float range  = hi - lo;

  const int gh      = GRAPH_BOTTOM - GRAPH_TOP;
  const int centerY = GRAPH_BOTTOM - gh / 2;

  // Dotted reference line at "latest reading" level
  for (int x = 0; x < SCREEN_WIDTH; x += 4) {
    display.drawPixel(x, centerY, SSD1306_WHITE);
  }

  // Plot data line; drop points outside the ±3 hPa window
  int prevX = -1, prevY = -1;
  for (int i = 0; i < dataCount; i++) {
    int x = (SCREEN_WIDTH - 1) -
            ((dataCount - 1) - i) * (int)PX_PER_READING;
    if (x < 0) continue;

    float t = (hist[i] - lo) / range;
    if (t < 0.0f || t > 1.0f) {
      prevX = -1;   // break the line at the gap
      continue;
    }
    int yp = GRAPH_BOTTOM - (int)(t * gh + 0.5f);

    if (prevX >= 0) {
      display.drawLine(prevX, prevY, x, yp, SSD1306_WHITE);
    }
    prevX = x;
    prevY = yp;
  }

  display.fillCircle(SCREEN_WIDTH - 1, centerY, 1, SSD1306_WHITE);
}

void redraw() {
  display.clearDisplay();
  drawHeader();
  display.drawFastHLine(0, SEPARATOR_Y, SCREEN_WIDTH, SSD1306_WHITE);
  drawGraph();
  display.display();
}


// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);
  Wire.begin();
  Serial.println(F("\n== Pressure Logger v4 (OLED + EEPROM, BMP280) =="));

  // ── OLED init ─────────────────────────────────────────────
  bool oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR_PRIMARY);
  if (!oledOk) {
    Serial.println(F("OLED 0x3C failed, trying 0x3D..."));
    oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR_FALLBACK);
  }
  if (!oledOk) {
    Serial.println(F("WARNING: SSD1306 OLED not found — continuing without display."));
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(F("Pressure Logger"));
    display.setCursor(0, 12);
    display.print(F("v4  BMP280"));
    display.setCursor(0, 24);
    display.print(F("Initializing..."));
    display.display();
  }

  // ── EEPROM self-test (new in v4) ──────────────────────────
  // Done before BMP280 / warm-up so any failure is reported early.
  eepromSelfTest();

  // ── BMP280 init ───────────────────────────────────────────
  if (!bmp.begin(BMP280_ADDR_PRIMARY)) {
    Serial.println(F("BMP280 0x76 failed, trying 0x77..."));
    if (!bmp.begin(BMP280_ADDR_FALLBACK)) {
      Serial.println(F("ERROR: BMP280 not found! Check wiring."));
      if (oledOk) showStatus(F("ERROR:"), F("BMP280 missing"));
      while (true) delay(1000);
    }
  }

  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_OFF,
                  Adafruit_BMP280::STANDBY_MS_1);
  Serial.println(F("BMP280 initialized (IIR off, OSR x16)."));

  // ── Sensor warm-up ────────────────────────────────────────
  Serial.println(F("Warming up sensor (30 s)..."));
  if (oledOk) showStatus(F("Warming up"), F("sensor (30 s)..."));
  for (int i = 0; i < 3; i++) {
    bmp.takeForcedMeasurement();
    (void)bmp.readPressure();
    Serial.print(F("Stabilizing... "));
    Serial.print(i + 1);
    Serial.println(F("/3"));
    delay(10000);
  }
  Serial.println(F("Sensor stabilized."));

  loadFromEEPROM();
  if (oledOk) redraw();

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
//  MEASUREMENT  (v2: software oversampling, no rounding)
// ════════════════════════════════════════════════════════════
void measure() {
  const float MIN_VALID_PA = 30000.0f;
  const float MAX_VALID_PA = 125000.0f;

  float sum = 0.0f;
  int   validCount = 0;

  for (int i = 0; i < N_SAMPLES; i++) {
    if (bmp.takeForcedMeasurement()) {
      float pa = bmp.readPressure();
      if (pa > MIN_VALID_PA && pa < MAX_VALID_PA) {
        sum += pa;
        validCount++;
      }
    }
    delay(SAMPLE_DELAY_MS);
  }

  if (validCount == 0) {
    Serial.println(F("ERROR: all samples failed!"));
    return;
  }

  float p = (sum / (float)validCount) / 100.0f;

  if (p < 300.0f || p > 1250.0f) {
    Serial.print(F("REJECTED — invalid average: "));
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
  Serial.print(p, 2);
  Serial.print(F(" hPa  | avg of "));
  Serial.print(validCount);
  Serial.print(F("/"));
  Serial.print(N_SAMPLES);
  Serial.print(F(" samples | stored: "));
  Serial.println(dataCount);

  saveToEEPROM();
  redraw();
}
