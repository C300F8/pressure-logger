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
 *   x:  0         60      75    115  124       127
 *      +-----------------------------------------+
 *  y= 0|1013.5 hPa                          /\   |  <- header
 *  y= 7|                                   |  |  |
 *      |-----------------------------------------|  <- separator
 *  y=10|                                .--      |
 *      |              ......      ..../          |  <- graph
 *  y=20|. . . . . . ./. . .\. . ./. . . . . . . .|  <- mean line (dotted)
 *      |        ..../       \.../                |
 *  y=31|.------/                                 |
 *      +-----------------------------------------+
 *
 *   y=0..7   header  — "1013.5 hPa" + trend arrow (up / right / down)
 *   y=8      1-px horizontal separator
 *   y=10..31 graph — 22 px tall, fixed +/-10 hPa window, latest at right
 *   y=20     sparse dotted line = mean of stored history (center of window)
 *
 * Features carried over from BMP280 2-matrix v1 (and BMP580 v7):
 *   - Fixed +/-SCALE_HALF_RANGE window centered on mean(history)
 *   - 0.5 hPa rounding to suppress jitter
 *   - Sensor warm-up (3 x 10 s) to discard wild first readings
 *   - EEPROM save/restore with storedCount range guard
 *   - 3-hour delta lookback for trend arrow
 *   - Forced-mode BMP280 sampling for low power
 *
 * Changelog:
 *   v1 (OLED port of 2-matrix BMP280 v1)
 *     - Removed: LedControl, DEVICES, DIN/CLK/CS pins,
 *                px() with FC-16 column mapping,
 *                8x8 trend-arrow pixel art.
 *     - Added:   Adafruit_SSD1306 + Adafruit_GFX,
 *                drawHeader(), drawGraph(), drawArrow(),
 *                splash screen and "warming up" status text,
 *                mean-pressure reference line in graph.
 *     - HISTORY_LEN: 12 -> 24
 *           Matches the TFT Python v6 24-hour visual style; uses
 *           the OLED's 128-px width well (24 x 5 = 120 px graph).
 *     - All sensor + EEPROM + scale logic unchanged.
 *     - Old EEPROM data (12 readings from previous sketch) restores
 *       correctly: dataCount=12 is still within [1, HISTORY_LEN],
 *       graph just starts half-empty and fills in over the next 12 h.
 * ============================================================ */

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── OLED: SSD1306, 128 x 32, I2C ─────────────────────────────
// OLED_RESET = -1 means "no separate reset pin" — the module
// relies on power-on reset (shared with the Arduino reset).
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

// ── History buffer ───────────────────────────────────────────
// 24 readings * 1 h = 24-hour window.
// RAM cost: 24 floats * 4 bytes = 96 bytes (well within Uno's 2 KB).
// EEPROM cost: 96 bytes for the data + 1 byte for count = 97 bytes.
#define HISTORY_LEN 24
float hist[HISTORY_LEN];
int   dataCount = 0;
unsigned long lastMs = 0;

// ── Display scale (unchanged from v7) ────────────────────────
// Fixed +/-10 hPa window centered on the mean of the stored history.
// Same slope on screen = same rate of pressure change in the air.
const float SCALE_HALF_RANGE = 10.0f;

// ── OLED layout constants ────────────────────────────────────
const uint8_t SEPARATOR_Y    = 8;     // 1-px horizontal line
const uint8_t GRAPH_TOP      = 10;
const uint8_t GRAPH_BOTTOM   = 31;
const uint8_t PX_PER_READING = 5;     // 24 readings * 5 px = 120 px
const uint8_t ARROW_X        = 115;   // arrow occupies x = 115..123
const uint8_t ARROW_Y        = 0;     //              and y =   0..  6

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
  delay(5);                       // mandatory internal-write delay
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

// Layout:
//   address  0      -> dataCount (1 byte)
//   address 10 ...  -> hist[] as raw bytes (4 bytes per float)
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

// Show a short two-line status message centered on the screen.
// Used during boot / warm-up. Both lines optional.
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

// 3-hour delta — same definition used by the LED-matrix sketches.
float computeDelta() {
  if (dataCount < 2) return 0.0f;
  int refIdx = max(0, dataCount - 3);
  return hist[dataCount - 1] - hist[refIdx];
}

// Draw a 9-wide x 7-tall directional arrow at (ARROW_X, ARROW_Y).
// All three shapes share the same bounding box so they swap cleanly.
//
//   delta >= +1.0  ->  up arrow
//   delta <= -1.0  ->  down arrow
//   otherwise      ->  right-pointing "steady" arrow
void drawArrow(float delta) {
  const int x = ARROW_X;
  const int y = ARROW_Y;

  if (delta >= 1.0f) {
    // /\   triangle head, stem below
    display.fillTriangle(x + 4, y,
                         x,     y + 2,
                         x + 8, y + 2,
                         SSD1306_WHITE);
    display.fillRect(x + 3, y + 2, 3, 5, SSD1306_WHITE);

  } else if (delta <= -1.0f) {
    // ||   stem on top, triangle head pointing down
    display.fillRect(x + 3, y, 3, 5, SSD1306_WHITE);
    display.fillTriangle(x + 4, y + 6,
                         x,     y + 4,
                         x + 8, y + 4,
                         SSD1306_WHITE);
  } else {
    // ->   horizontal shaft + triangle head on the right
    display.fillRect(x, y + 2, 6, 3, SSD1306_WHITE);
    display.fillTriangle(x + 5, y,
                         x + 5, y + 6,
                         x + 8, y + 3,
                         SSD1306_WHITE);
  }
}

// Header row at y=0..7: pressure value + trend arrow.
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

// Pressure line graph in y=10..31 (22 px tall).
// Right-aligned: newest reading sits at x = 127, older ones extend left
// at PX_PER_READING (5) pixels apart. Out-of-window values clamp.
void drawGraph() {
  if (dataCount == 0) return;

  // ── Fixed window centered on the mean ────────────────────
  float sum = 0;
  for (int i = 0; i < dataCount; i++) sum += hist[i];
  float mean  = sum / dataCount;
  float lo    = mean - SCALE_HALF_RANGE;
  float hi    = mean + SCALE_HALF_RANGE;
  float range = hi - lo;

  const int gh = GRAPH_BOTTOM - GRAPH_TOP;       // 21
  const int meanY = GRAPH_BOTTOM - gh / 2;       // = 21 (visual center)

  // ── Dotted mean reference line ───────────────────────────
  // Sparse 4-px spacing so it doesn't fight the data line for attention.
  for (int x = 0; x < SCREEN_WIDTH; x += 4) {
    display.drawPixel(x, meanY, SSD1306_WHITE);
  }

  // ── Plot data as connected line segments ─────────────────
  // Iterate oldest -> newest; "prev" tracks the previous point.
  int prevX = -1, prevY = -1;
  for (int i = 0; i < dataCount; i++) {
    int x = (SCREEN_WIDTH - 1) -
            ((dataCount - 1) - i) * (int)PX_PER_READING;
    if (x < 0) continue;   // would only matter if HISTORY_LEN bumped much higher

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

  // ── Highlight the latest reading with a small dot ────────
  if (prevX >= 0) {
    display.fillCircle(prevX, prevY, 1, SSD1306_WHITE);
  }
}

// Full screen redraw — called after each measurement and after EEPROM restore.
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
  Serial.println(F("\n== Pressure Logger v1 (OLED + EEPROM, BMP280) =="));

  // ── OLED init ─────────────────────────────────────────────
  bool oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR_PRIMARY);
  if (!oledOk) {
    Serial.println(F("OLED 0x3C failed, trying 0x3D..."));
    oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR_FALLBACK);
  }
  if (!oledOk) {
    Serial.println(F("WARNING: SSD1306 OLED not found — continuing without display."));
    Serial.println(F("  Check SDA=A4, SCL=A5, and the I2C address."));
  } else {
    // Splash screen
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(F("Pressure Logger"));
    display.setCursor(0, 12);
    display.print(F("v1  BMP280"));
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
      while (true) delay(1000);   // halt
    }
  }

  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
                  Adafruit_BMP280::SAMPLING_X2,    // temp oversampling
                  Adafruit_BMP280::SAMPLING_X16,   // pressure oversampling
                  Adafruit_BMP280::FILTER_X16,     // IIR coefficient 16
                  Adafruit_BMP280::STANDBY_MS_1);  // unused in FORCED mode
  Serial.println(F("BMP280 initialized."));

  // ── Sensor warm-up ────────────────────────────────────────
  Serial.println(F("Warming up sensor (30 s)..."));
  if (oledOk) showStatus(F("Warming up"), F("sensor (30 s)..."));
  for (int i = 0; i < 3; i++) {
    bmp.takeForcedMeasurement();
    (void)bmp.readPressure();   // discard
    Serial.print(F("Stabilizing... "));
    Serial.print(i + 1);
    Serial.println(F("/3"));
    delay(10000);
  }
  Serial.println(F("Sensor stabilized."));

  // ── Restore and display saved history ─────────────────────
  loadFromEEPROM();
  if (oledOk) redraw();

  // First live reading
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
  if (!bmp.takeForcedMeasurement()) {
    Serial.println(F("ERROR: BMP280 measurement timed out!"));
    return;
  }

  // BMP280.readPressure() returns Pa -> divide by 100 for hPa.
  float rawP = bmp.readPressure() / 100.0f;

  // Round to nearest 0.5 hPa to suppress 1-pixel display jitter
  float p = round(rawP * 2.0f) / 2.0f;

  // Sanity check
  if (p < 300.0f || p > 1250.0f) {
    Serial.print(F("REJECTED — invalid reading: "));
    Serial.println(p);
    return;
  }

  // Append, or shift-left when full
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

  saveToEEPROM();   // persist before redraw, in case of power loss
  redraw();
}
