/* ============================================================
 * Atmospheric Pressure Logger — 0.91" SSD1306 OLED + I2C EEPROM
 *                              BMP280 SENSOR VARIANT
 * ============================================================
 * Hardware:
 *   - Arduino Uno / Pro Mini
 *   - BMP280 (I2C) — works with bare modules and AHT20+BMP280
 *       combo boards. AHT20 (0x38) is ignored here.
 *   - 0.91" SSD1306 OLED, 128 x 32, I2C
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
 *   y=10..31 graph — 22 px tall, ±3 hPa adaptive window
 *   y=20     sparse dotted line = LATEST reading (= window center)
 *
 * Features carried over from earlier versions:
 *   - Sensor warm-up (3 x 10 s) on boot
 *   - EEPROM save/restore with storedCount range guard
 *   - 3-hour delta lookback for trend arrow
 *   - Forced-mode BMP280 sampling with software oversampling (v2)
 *   - 24-hour rolling history
 *
 * Changelog:
 *   v1 (OLED port of 2-matrix BMP280 v1)
 *     - LED matrix output replaced with SSD1306 128x32 OLED
 *     - HISTORY_LEN: 12 -> 24
 *
 *   v2 (responsiveness improvements)
 *     - IIR filter: FILTER_X16 -> FILTER_OFF
 *     - Software oversampling (N_SAMPLES averaged per cycle)
 *     - 0.5 hPa rounding removed; full float precision stored
 *
 *   v3 (adaptive window — addresses graph clipping during sustained trends):
 *     - SCALE_HALF_RANGE: 10.0  ->  3.0
 *           v2's ±10 hPa window made calm days and stormy days
 *           look equally tall in the graph. With the latest
 *           reading now anchoring the center (see next point),
 *           a ±3 hPa window gives 3.3x better vertical resolution
 *           (0.27 hPa/px vs. 0.91 hPa/px) while still covering all
 *           but exceptional weather events at full visibility.
 *
 *     - Window centered on LATEST reading, not mean(history)
 *           v1/v2 centered on the running mean. When pressure had
 *           been moving in one direction for several hours, the
 *           mean lagged the current value and the newest readings
 *           climbed/sank against the top/bottom of the graph until
 *           they clipped. v3 anchors the center on hist[last], so
 *           the latest reading always sits at the vertical center
 *           (= the dotted line) and never clips. Older readings
 *           drift visually as the center shifts with new data.
 *
 *     - Out-of-window readings skipped, not clamped
 *           v2 clamped points to row 0 or row 7 — they piled up
 *           at the edge, which was misleading. v3 simply doesn't
 *           plot them and breaks the connecting line at the gap.
 *           Readings are still stored at full precision in RAM
 *           and EEPROM; they just don't appear on screen when
 *           they fall outside the ±3 hPa window around "now".
 *
 *     - Visual interpretation:
 *           Rising trend  →  line slopes upward from left to right
 *                            and meets the dotted center line at
 *                            x=127. Older (lower) readings extend
 *                            below the center toward the graph
 *                            floor. If older readings drift more
 *                            than 3 hPa below "now", the line
 *                            simply ends there.
 *           Falling trend →  mirror image: line slopes downward
 *                            to the right; older readings extend
 *                            above the center toward the ceiling.
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
#define N_SAMPLES         8
#define SAMPLE_DELAY_MS 100

// ── History buffer ───────────────────────────────────────────
#define HISTORY_LEN 24
float hist[HISTORY_LEN];
int   dataCount = 0;
unsigned long lastMs = 0;

// ── Display scale (v3 — adaptive window) ─────────────────────
// The window spans ±SCALE_HALF_RANGE hPa around the LATEST
// reading. Total span = 2 × SCALE_HALF_RANGE hPa, mapped across
// the 22-px-tall graph area.
//
// With SCALE_HALF_RANGE = 3.0:
//   - vertical resolution = 6 hPa / 22 px ≈ 0.27 hPa per pixel
//   - latest reading always sits at the vertical center
//     (also where the dotted reference line is drawn)
//   - any older reading more than 3 hPa away from "now" is
//     skipped — see drawGraph() for the gap-rendering logic.
//
// Tuning: raise to 5.0 if you live in a high-variability region
// and frequently see older readings disappear off-screen during
// active weather. Lower to 2.0 for an even more sensitive view
// during calm spells.
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

  display.print(hist[dataCount - 1], 1);
  display.print(F(" hPa"));
  drawArrow(computeDelta());
}

// ── v3 graph renderer ────────────────────────────────────────
// Window: ±SCALE_HALF_RANGE around hist[dataCount - 1] (latest).
// Latest reading is always at the vertical center of the graph.
// Older readings outside the window are skipped, and the
// connecting line is broken at each gap.
void drawGraph() {
  if (dataCount == 0) return;

  // v3: center on the latest reading, NOT the running mean
  float center = hist[dataCount - 1];
  float lo     = center - SCALE_HALF_RANGE;
  float hi     = center + SCALE_HALF_RANGE;
  float range  = hi - lo;

  const int gh      = GRAPH_BOTTOM - GRAPH_TOP;   // 21
  const int centerY = GRAPH_BOTTOM - gh / 2;      // = 21, visual center

  // ── Dotted reference line at "latest reading" level ──────
  // Sparse 4-px spacing so it doesn't compete with the data line.
  for (int x = 0; x < SCREEN_WIDTH; x += 4) {
    display.drawPixel(x, centerY, SSD1306_WHITE);
  }

  // ── Plot data line; drop points outside the window ───────
  // Iterate oldest -> newest. "prev" tracks the previous drawn
  // point; we reset it to -1 whenever a point is dropped so the
  // line breaks cleanly at the gap (no jump-across-empty-space).
  int prevX = -1, prevY = -1;
  for (int i = 0; i < dataCount; i++) {
    int x = (SCREEN_WIDTH - 1) -
            ((dataCount - 1) - i) * (int)PX_PER_READING;
    if (x < 0) continue;

    float t = (hist[i] - lo) / range;
    if (t < 0.0f || t > 1.0f) {
      // Out of window — skip this point AND break the line.
      // The reading is still in hist[] and EEPROM, just not drawn.
      prevX = -1;
      continue;
    }
    int yp = GRAPH_BOTTOM - (int)(t * gh + 0.5f);

    if (prevX >= 0) {
      display.drawLine(prevX, prevY, x, yp, SSD1306_WHITE);
    }
    prevX = x;
    prevY = yp;
  }

  // ── Highlight the latest reading ─────────────────────────
  // By construction the latest is at (x=127, y=centerY) — it's
  // the anchor of the window. Drawing it explicitly makes it
  // stand out from the dotted reference line passing through.
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
  Serial.println(F("\n== Pressure Logger v3 (OLED + EEPROM, BMP280) =="));

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
    display.print(F("v3  BMP280"));
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

  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
                  Adafruit_BMP280::SAMPLING_X2,    // temp oversampling
                  Adafruit_BMP280::SAMPLING_X16,   // pressure oversampling
                  Adafruit_BMP280::FILTER_OFF,     // v2: IIR off
                  Adafruit_BMP280::STANDBY_MS_1);  // unused in FORCED mode
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
  const float MIN_VALID_PA = 30000.0f;    // 300 hPa
  const float MAX_VALID_PA = 125000.0f;   // 1250 hPa

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
