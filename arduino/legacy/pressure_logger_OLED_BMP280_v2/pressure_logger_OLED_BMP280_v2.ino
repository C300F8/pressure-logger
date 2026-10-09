/* ============================================================
 * Atmospheric Pressure Logger — 0.91" SSD1306 OLED + I2C EEPROM
 *                              BMP280 SENSOR VARIANT
 * ============================================================
 * Hardware:
 *   - Arduino Uno / Pro Mini
 *   - BMP280 (I2C) — works with bare modules and AHT20+BMP280
 *       combo boards. AHT20 (0x38) is ignored here.
 *   - 0.91" SSD1306 OLED, 128 x 32, I2C
 *       (listings sometimes call it "SSD136" — same chip, typo;
 *        the 0.91" 4-pin OLED is always SSD1306.)
 *   - AT24C32 / AT24C256 I2C EEPROM
 *
 * Wiring (everything shares the same I2C bus):
 *   BMP280   VDD->3.3V/5V  GND->GND  SDA->A4  SCL->A5
 *   OLED     VCC->3.3V/5V  GND->GND  SDA->A4  SCL->A5
 *   EEPROM   VCC->5V       GND->GND  SDA->A4  SCL->A5
 *
 *   I2C addresses (no conflicts):
 *     BMP280  0x76 (or 0x77)
 *     OLED    0x3C (or 0x3D)
 *     EEPROM  0x50
 *
 *   Note on pull-ups: each module has its own pull-up resistors.
 *   Three in parallel makes the effective bus pull-up rather low
 *   (~1.5 kOhm). Still works, but if you see flaky I2C transactions
 *   you can lift the pull-ups from one or two modules with a soldering
 *   iron — typically the EEPROM and OLED ones, leaving the BMP280's.
 *
 * Required libraries (Library Manager):
 *   Adafruit BMP280 Library    by Adafruit
 *   Adafruit SSD1306           by Adafruit
 *   Adafruit GFX Library       by Adafruit  (dependency)
 *   Adafruit BusIO             by Adafruit  (dependency)
 *
 * Display layout (128 x 32 px, monochrome):
 *
 *   y=0..7   header  — "1013.5 hPa" + trend arrow (up / right / down)
 *   y=8      1-px horizontal separator
 *   y=10..31 graph — 22 px tall, fixed +/-10 hPa window, latest at right
 *   y=20     sparse dotted line = mean of stored history
 *
 * Features carried over from earlier versions:
 *   - Fixed +/-SCALE_HALF_RANGE window centered on mean(history)
 *   - Sensor warm-up (3 x 10 s) on boot
 *   - EEPROM save/restore with storedCount range guard
 *   - 3-hour delta lookback for trend arrow
 *   - Forced-mode BMP280 sampling (low power, on-demand)
 *   - 24-hour rolling history
 *
 * Changelog:
 *   v1 (OLED port of 2-matrix BMP280 v1)
 *     - LED matrix output replaced with SSD1306 128x32 OLED
 *     - HISTORY_LEN: 12 -> 24
 *     - Added drawHeader / drawGraph / drawArrow + mean reference line
 *
 *   v2 (responsiveness improvements — addresses "BMP280 feels dumb"):
 *     - IIR filter:  FILTER_X16  ->  FILTER_OFF
 *           v7's FILTER_X16 was sized for continuous high-rate
 *           sampling. At 1 reading per hour each new sample contributes
 *           only ~6% to the filtered output, so the filter took
 *           ~2-3 days to fully reflect a real pressure step — the
 *           visible lag you saw in v1. Disabling IIR removes the lag
 *           entirely. OSR x16 already does 16x hardware oversampling
 *           per measurement, so noise stays well-controlled.
 *
 *     - Software oversampling in measure():
 *           Each cycle now takes N_SAMPLES (default 8) forced
 *           measurements ~100 ms apart and averages them. Recovers
 *           the noise benefit of an aggressive IIR filter
 *           (sqrt(N) ≈ 2.8x reduction for N=8) with zero temporal
 *           lag. Total measurement time: ~1.2 s — negligible vs.
 *           the 1-hour interval. Individual bad samples are
 *           skipped via a per-sample sanity check.
 *
 *     - 0.5 hPa rounding removed:
 *           On the 8-row LED matrix each row was 2.5 hPa, so
 *           0.5 hPa rounding was invisible (1/5 of a row). On the
 *           22-px OLED graph each pixel is ~0.9 hPa, so rounding
 *           was visibly throwing away resolution. Full float
 *           precision now stored, plotted, and serial-logged
 *           (2 decimals in serial; header stays at 1 decimal —
 *           that's the granularity that matters for weather).
 *
 *     - Everything else unchanged from v1. EEPROM layout is
 *       compatible: rounded values saved by v1 restore fine.
 * ============================================================ */

#include <Wire.h>
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
// Per-cycle averaging: take N_SAMPLES forced measurements
// SAMPLE_DELAY_MS apart, average them, log the mean.
//
// With FILTER_OFF, single-sample noise on the BMP280 at OSR x16
// is roughly 0.013 hPa RMS. Averaging 8 samples brings that down
// to ~0.005 hPa — comfortably below the natural minute-to-minute
// drift of real atmospheric pressure.
//
// Tuning: bump N_SAMPLES higher for cleaner readings (diminishing
// returns past ~16); raise SAMPLE_DELAY_MS only if you suspect
// I2C bus contention. Total time per cycle ≈ N * (40 + DELAY) ms.
#define N_SAMPLES         8
#define SAMPLE_DELAY_MS 100

// ── History buffer ───────────────────────────────────────────
// 24 readings x 1 h = 24-hour window.
// RAM: 24 floats * 4 = 96 bytes. EEPROM footprint: 97 bytes.
#define HISTORY_LEN 24
float hist[HISTORY_LEN];
int   dataCount = 0;
unsigned long lastMs = 0;

// ── Display scale (unchanged from v7) ────────────────────────
// Fixed +/-10 hPa window centered on mean(history).
// Same slope on screen = same rate of change in the air.
const float SCALE_HALF_RANGE = 10.0f;

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

// 9 x 7 directional arrow at (ARROW_X, ARROW_Y).
void drawArrow(float delta) {
  const int x = ARROW_X;
  const int y = ARROW_Y;

  if (delta >= 1.0f) {
    // Up arrow: triangle head, stem below
    display.fillTriangle(x + 4, y,
                         x,     y + 2,
                         x + 8, y + 2,
                         SSD1306_WHITE);
    display.fillRect(x + 3, y + 2, 3, 5, SSD1306_WHITE);

  } else if (delta <= -1.0f) {
    // Down arrow: stem on top, triangle head below
    display.fillRect(x + 3, y, 3, 5, SSD1306_WHITE);
    display.fillTriangle(x + 4, y + 6,
                         x,     y + 4,
                         x + 8, y + 4,
                         SSD1306_WHITE);
  } else {
    // Steady: horizontal shaft + right-pointing triangle head
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

  // 1 decimal on display — that's the weather-relevant precision.
  // Underlying value is full float; serial log shows 2 decimals.
  display.print(hist[dataCount - 1], 1);
  display.print(F(" hPa"));
  drawArrow(computeDelta());
}

void drawGraph() {
  if (dataCount == 0) return;

  // Fixed window centered on the mean
  float sum = 0;
  for (int i = 0; i < dataCount; i++) sum += hist[i];
  float mean  = sum / dataCount;
  float lo    = mean - SCALE_HALF_RANGE;
  float hi    = mean + SCALE_HALF_RANGE;
  float range = hi - lo;

  const int gh    = GRAPH_BOTTOM - GRAPH_TOP;   // 21
  const int meanY = GRAPH_BOTTOM - gh / 2;      // 21 (visual center)

  // Dotted mean reference line (4-px spacing so it stays subtle)
  for (int x = 0; x < SCREEN_WIDTH; x += 4) {
    display.drawPixel(x, meanY, SSD1306_WHITE);
  }

  // Connect samples as line segments. Newest at x=127, older to the left.
  int prevX = -1, prevY = -1;
  for (int i = 0; i < dataCount; i++) {
    int x = (SCREEN_WIDTH - 1) -
            ((dataCount - 1) - i) * (int)PX_PER_READING;
    if (x < 0) continue;

    float t = (hist[i] - lo) / range;
    if (t < 0.0f) t = 0.0f;
    else if (t > 1.0f) t = 1.0f;
    int yp = GRAPH_BOTTOM - (int)(t * gh + 0.5f);

    if (prevX >= 0) {
      display.drawLine(prevX, prevY, x, yp, SSD1306_WHITE);
    }
    prevX = x;
    prevY = yp;
  }

  // Latest-reading highlight dot
  if (prevX >= 0) {
    display.fillCircle(prevX, prevY, 1, SSD1306_WHITE);
  }
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
  Serial.println(F("\n== Pressure Logger v2 (OLED + EEPROM, BMP280) =="));

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
    display.print(F("v2  BMP280"));
    display.setCursor(0, 24);
    display.print(F("Initializing..."));
    display.display();
  }

  // ── BMP280 init ───────────────────────────────────────────
  if (!bmp.begin(BMP280_ADDR_PRIMARY)) {
    Serial.println(F("BMP280 0x76 failed, trying 0x77..."));
    if (!bmp.begin(BMP280_ADDR_FALLBACK)) {
      Serial.println(F("ERROR: BMP280 not found! Check wiring."));
      if (oledOk) showStatus(F("ERROR:"), F("BMP280 missing"));
      while (true) delay(1000);
    }
  }

  // v2: FILTER_OFF (was FILTER_X16 in v1). See changelog at top
  // of file. Software averaging in measure() now handles noise.
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
                  Adafruit_BMP280::SAMPLING_X2,    // temp oversampling
                  Adafruit_BMP280::SAMPLING_X16,   // pressure oversampling
                  Adafruit_BMP280::FILTER_OFF,     // *** v2: was FILTER_X16
                  Adafruit_BMP280::STANDBY_MS_1);  // unused in FORCED mode
  Serial.println(F("BMP280 initialized (IIR off, OSR x16)."));

  // ── Sensor warm-up ────────────────────────────────────────
  // Less critical now that there's no IIR state to settle, but
  // thermal stabilization of the chip itself still benefits.
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
//
// Take N_SAMPLES forced-mode measurements ~SAMPLE_DELAY_MS apart,
// reject obvious outliers per-sample, average the survivors.
// Result is logged at full float precision (no 0.5 hPa rounding).
//
// Why this works better than v1:
//   v1 used FILTER_X16, which lagged 2-3 days at hourly cadence.
//   v2 turns off the hardware filter entirely and recovers the
//   noise reduction in software, where the smoothing window is
//   bounded to a single ~1-second burst — no temporal lag at all.
//
void measure() {
  const float MIN_VALID_PA = 30000.0f;    // 300 hPa
  const float MAX_VALID_PA = 125000.0f;   // 1250 hPa

  float sum = 0.0f;
  int   validCount = 0;

  for (int i = 0; i < N_SAMPLES; i++) {
    if (bmp.takeForcedMeasurement()) {
      float pa = bmp.readPressure();   // BMP280 returns Pa
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

  // Average and convert Pa -> hPa. Full float precision now —
  // v1's round(rawP * 2) / 2 step is gone.
  float p = (sum / (float)validCount) / 100.0f;

  // Final sanity check on the averaged value
  if (p < 300.0f || p > 1250.0f) {
    Serial.print(F("REJECTED — invalid average: "));
    Serial.println(p);
    return;
  }

  // Append to history (shift-left when full)
  if (dataCount < HISTORY_LEN) {
    hist[dataCount++] = p;
  } else {
    for (int i = 0; i < HISTORY_LEN - 1; i++) hist[i] = hist[i + 1];
    hist[HISTORY_LEN - 1] = p;
  }

  Serial.print(F("P = "));
  Serial.print(p, 2);                  // 2 decimals — visible precision
  Serial.print(F(" hPa  | avg of "));
  Serial.print(validCount);
  Serial.print(F("/"));
  Serial.print(N_SAMPLES);
  Serial.print(F(" samples | stored: "));
  Serial.println(dataCount);

  saveToEEPROM();
  redraw();
}
