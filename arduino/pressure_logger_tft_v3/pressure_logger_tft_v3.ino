/* ============================================================
 * Atmospheric Pressure Logger — 2.4" TFT LCD Shield
 * ============================================================
 * Hardware:
 *   - Arduino Uno R3
 *   - BMP580 barometric pressure sensor  (I2C: A4=SDA, A5=SCL)
 *   - 2.4" TFT Shield  ILI9340 / ILI9341  (mcufriend.com)
 *   - AT24C32 / AT24C256 I2C EEPROM        (same I2C bus)
 *
 * Libraries — install via Arduino Library Manager:
 *   MCUFRIEND_kbv   by David Prentice
 *   Adafruit GFX    by Adafruit
 *   Adafruit BMP5xx by Adafruit
 *
 * ── v2 fix: A4 pin conflict ──────────────────────────────────
 * The standard 2.4" mcufriend shield wires A4 → LCD_RST.
 * Hardware I2C also uses A4 as SDA (BMP580 + EEPROM).
 * During EEPROM save (192 byte-writes × I2C transactions),
 * A4 toggles as SDA → the TFT controller sees it as RST pulses
 * → display resets to white after every EEPROM operation.
 *
 * Fix: store tftID globally, call tft.begin() + setRotation()
 * at the top of redraw() to re-initialise the controller after
 * any I2C activity. The re-init takes ~5 ms — imperceptible.
 * ============================================================
 * ── Changelog ───────────────────────────────────────────────
 *   v1 — TFT port from pressure_logger_4matrix_v1.ino
 *   v2 — Fix white screen: re-init TFT before every redraw()
 *         to survive A4 (LCD_RST = I2C SDA) pin conflict.
 *         Added tftID to Serial output for diagnostics.
 *   v3 — Fix graph clipping during rapid pressure changes.
 *         Scale window starts at mean ±SCALE_HALF_RANGE for
 *         visual comparability on calm days, but auto-expands
 *         (with 2 hPa margin) when any reading falls outside.
 *         Example: 30 hPa rise over 48 h → window grows from
 *         20 hPa to ~34 hPa so the full curve stays visible.
 * ============================================================ */

#include <Wire.h>
#include <Adafruit_BMP5xx.h>
#include <MCUFRIEND_kbv.h>
#include <Adafruit_GFX.h>

// ════════════════════════════════════════════════════════════
//  HARDWARE OBJECTS
// ════════════════════════════════════════════════════════════
MCUFRIEND_kbv   tft;
Adafruit_BMP5xx bmp;

// ── Stored globally so redraw() can re-init after I2C ops ───
uint16_t g_tftID = 0x9341;   // will be overwritten in setup()


// ════════════════════════════════════════════════════════════
//  CONFIGURATION
// ════════════════════════════════════════════════════════════
#define EEPROM_ADDR   0x50

#define TEST_MODE     false
#if TEST_MODE
  #define INTERVAL_MS    900000UL   // 15 min
#else
  #define INTERVAL_MS   3600000UL   // 1 hour
#endif

#define HISTORY_LEN  48
float         hist[HISTORY_LEN];
int           dataCount = 0;
unsigned long lastMs    = 0;

const float SCALE_HALF_RANGE = 10.0f;


// ════════════════════════════════════════════════════════════
//  COLOR PALETTE  (RGB-565)
// ════════════════════════════════════════════════════════════
#define C_BG      0x0000   // Black background
#define C_PANEL   0x0861   // Very dark warm gray (header/footer)
#define C_GRID    0x2104   // Dim gray — faint grid lines
#define C_MGRID   0x39C7   // Slightly brighter — mean-pressure grid line
#define C_AXIS    0x528A   // Medium gray — axis lines
#define C_LINE    0xFD20   // Orange — data line (LED aesthetic)
#define C_DOT     0xFEE0   // Light amber — dot on latest reading
#define C_TEXT    0xFFFF   // White — primary text
#define C_DIM     0xC618   // Light gray — secondary / axis labels
#define C_RISE    0x07E0   // Bright green  — rising trend
#define C_FALL    0xF800   // Bright red    — falling trend
#define C_STEADY  0xFFFF   // White         — stable trend


// ════════════════════════════════════════════════════════════
//  LAYOUT  (landscape 320 × 240)
// ════════════════════════════════════════════════════════════
#define HEADER_H  48
#define FOOTER_Y  202
#define GX1  50
#define GX2  315
#define GY1  HEADER_H
#define GY2  FOOTER_Y
#define GW   (GX2 - GX1)
#define GH   (GY2 - GY1)


// ════════════════════════════════════════════════════════════
//  EEPROM HELPERS
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

void saveToEEPROM() {
  Serial.println(F("Saving to EEPROM..."));
  writeEEPROM(0, (byte)dataCount);
  byte* p = (byte*)(void*)&hist;
  for (int i = 0; i < (int)sizeof(hist); i++)
    writeEEPROM(10 + i, p[i]);
  Serial.println(F("EEPROM saved."));
  // NOTE: After this many I2C transactions, LCD_RST (A4) has been
  // toggled ~200 times as SDA. tftInit() in redraw() will fix it.
}

void loadFromEEPROM() {
  Serial.println(F("Checking EEPROM..."));
  byte stored = readEEPROM(0);
  if (stored >= 1 && stored <= HISTORY_LEN) {
    dataCount = (int)stored;
    byte* p = (byte*)(void*)&hist;
    for (int i = 0; i < (int)sizeof(hist); i++)
      p[i] = readEEPROM(10 + i);
    Serial.print(F("Restored "));
    Serial.print(dataCount);
    Serial.println(F(" readings from EEPROM."));
  } else {
    dataCount = 0;
    Serial.println(F("No valid history in EEPROM — starting fresh."));
  }
}


// ════════════════════════════════════════════════════════════
//  TFT RE-INITIALISER  ← core fix for A4 conflict
// ════════════════════════════════════════════════════════════
// Must be called at the start of every redraw() because I2C
// operations on A4/SDA reset the TFT controller (A4 = LCD_RST).
void tftInit() {
  tft.begin(g_tftID);
  tft.setRotation(1);   // landscape; use 3 if display is upside-down
}


// ════════════════════════════════════════════════════════════
//  COORDINATE MAPPERS
// ════════════════════════════════════════════════════════════
int pressToY(float p, float lo, float hi) {
  float t = (p - lo) / (hi - lo);
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return GY2 - (int)(t * (float)GH + 0.5f);
}

int idxToX(int i, int count) {
  if (count <= 1) return GX2;
  return GX1 + (int)((float)i / (float)(count - 1) * (float)GW + 0.5f);
}


// ════════════════════════════════════════════════════════════
//  TREND
// ════════════════════════════════════════════════════════════
float trendDelta() {
  if (dataCount < 2) return 0.0f;
  int ref = max(0, dataCount - 4);
  return hist[dataCount - 1] - hist[ref];
}


// ════════════════════════════════════════════════════════════
//  TREND ARROWS  (drawn in header, right side)
// ════════════════════════════════════════════════════════════
#define ARX  232
#define ARY   24

void drawArrowUp(uint16_t c) {
  tft.fillTriangle(ARX, 4,     ARX-12, 22, ARX+12, 22, c);
  tft.fillRect    (ARX-5, 22,  11, 18,                  c);
}
void drawArrowDown(uint16_t c) {
  tft.fillRect    (ARX-5, 6,   11, 18,                  c);
  tft.fillTriangle(ARX, 42,    ARX-12, 24, ARX+12, 24,  c);
}
void drawArrowSteady(uint16_t c) {
  tft.fillRect    (ARX-16, 20, 22, 8,                   c);
  tft.fillTriangle(ARX+6, 12,  ARX+20, ARY, ARX+6, 36,  c);
}


// ════════════════════════════════════════════════════════════
//  HEADER SECTION
// ════════════════════════════════════════════════════════════
void drawHeader() {
  tft.fillRect(0, 0, 320, HEADER_H, C_PANEL);

  tft.setTextSize(3);
  tft.setTextColor(C_TEXT);
  tft.setCursor(4, 10);
  if (dataCount > 0) {
    char buf[8];
    dtostrf(hist[dataCount - 1], 6, 1, buf);
    tft.print(buf);
  } else {
    tft.print(F(" ---.--"));
  }
  tft.setTextSize(2);
  tft.setTextColor(C_DIM);
  tft.setCursor(118, 18);
  tft.print(F("hPa"));

  float       delta = trendDelta();
  uint16_t    col;
  const char* label;

  if (delta >= 1.0f) {
    col = C_RISE;   label = "RISE";  drawArrowUp(C_RISE);
  } else if (delta <= -1.0f) {
    col = C_FALL;   label = "FALL";  drawArrowDown(C_FALL);
  } else {
    col = C_STEADY; label = "STBL";  drawArrowSteady(C_STEADY);
  }

  tft.setTextColor(col);
  tft.setTextSize(2);
  tft.setCursor(252, 6);
  tft.print(label);

  tft.setTextSize(1);
  tft.setCursor(252, 28);
  if (delta > 0.0f) tft.print('+');
  tft.print(delta, 1);
  tft.print(F(" hPa"));

  tft.drawFastHLine(0, HEADER_H - 1, 320, C_AXIS);
}


// ════════════════════════════════════════════════════════════
//  FOOTER SECTION
// ════════════════════════════════════════════════════════════
void drawFooter() {
  tft.fillRect(0, FOOTER_Y, 320, 240 - FOOTER_Y, C_PANEL);
  tft.drawFastHLine(0, FOOTER_Y, 320, C_AXIS);
  if (dataCount == 0) return;

  float minP = hist[0], maxP = hist[0];
  for (int i = 1; i < dataCount; i++) {
    if (hist[i] < minP) minP = hist[i];
    if (hist[i] > maxP) maxP = hist[i];
  }

  char buf[8];
  tft.setTextSize(1);

  tft.setTextColor(C_DIM);  tft.setCursor(4, FOOTER_Y + 7);   tft.print(F("MIN"));
  tft.setTextColor(C_TEXT); tft.setCursor(22, FOOTER_Y + 7);
  dtostrf(minP, 6, 1, buf); tft.print(buf);
  tft.setTextColor(C_DIM);  tft.print(F(" hPa"));

  tft.setTextColor(C_DIM);  tft.setCursor(114, FOOTER_Y + 7); tft.print(F("MAX"));
  tft.setTextColor(C_TEXT); tft.setCursor(132, FOOTER_Y + 7);
  dtostrf(maxP, 6, 1, buf); tft.print(buf);
  tft.setTextColor(C_DIM);  tft.print(F(" hPa"));

  tft.setTextColor(C_DIM);  tft.setCursor(230, FOOTER_Y + 7);
  tft.print(dataCount); tft.print(F(" / ")); tft.print(HISTORY_LEN); tft.print(F(" h"));
}


// ════════════════════════════════════════════════════════════
//  GRAPH SECTION
// ════════════════════════════════════════════════════════════
void drawGraph() {
  tft.fillRect(0, GY1, 320, GH, C_BG);

  if (dataCount == 0) {
    tft.setTextColor(C_DIM); tft.setTextSize(1);
    tft.setCursor(GX1 + 8, GY1 + GH / 2 - 4);
    tft.print(F("Waiting for first reading..."));
    return;
  }

  float sum = 0;
  float minP = hist[0], maxP = hist[0];
  for (int i = 0; i < dataCount; i++) {
    sum += hist[i];
    if (hist[i] < minP) minP = hist[i];
    if (hist[i] > maxP) maxP = hist[i];
  }
  float mean = sum / (float)dataCount;

  // Start with the standard fixed window centered on mean.
  // Auto-expand if any reading falls outside — keeps visual
  // comparability on calm days while preventing clipping on
  // storm days or rapid rises/falls (v3 fix).
  // 2 hPa margin keeps the topmost / bottommost dot off the edge.
  float lo = mean - SCALE_HALF_RANGE;
  float hi = mean + SCALE_HALF_RANGE;
  if (minP - 5.0f < lo) lo = minP - 5.0f;  // v3.1: increased margin 2→5 hPa to avoid dot hugging the edge
  if (maxP + 5.0f > hi) hi = maxP + 5.0f;

  // Grid lines every 5 hPa + Y-axis labels
  float gridStart = ceilf(lo / 5.0f) * 5.0f;
  for (float gp = gridStart; gp <= hi + 0.01f; gp += 5.0f) {
    int y        = pressToY(gp, lo, hi);
    bool nearMean = (fabsf(gp - mean) < 2.5f);
    tft.drawFastHLine(GX1, y, GW, nearMean ? C_MGRID : C_GRID);
    tft.setTextSize(1);
    tft.setTextColor(C_DIM);
    int lx = ((int)gp >= 1000) ? 1 : 7;
    tft.setCursor(lx, y - 3);
    tft.print((int)gp);
  }

  tft.drawFastVLine(GX1,      GY1, GH, C_AXIS);
  tft.drawFastHLine(GX1, GY2 - 1, GW, C_AXIS);

  // Data line (2 px thick for readability)
  for (int i = 1; i < dataCount; i++) {
    int x0 = idxToX(i - 1, dataCount),  y0 = pressToY(hist[i-1], lo, hi);
    int x1 = idxToX(i,     dataCount),  y1 = pressToY(hist[i],   lo, hi);
    tft.drawLine(x0, y0,     x1, y1,     C_LINE);
    tft.drawLine(x0, y0 + 1, x1, y1 + 1, C_LINE);
  }

  // Latest-reading dot
  int lx = idxToX(dataCount - 1, dataCount);
  int ly = pressToY(hist[dataCount - 1], lo, hi);
  tft.fillCircle(lx, ly, 5, C_DOT);
  tft.drawCircle(lx, ly, 5, C_LINE);
}


// ════════════════════════════════════════════════════════════
//  FULL REDRAW  ← always re-inits TFT first (v2 fix)
// ════════════════════════════════════════════════════════════
void redraw() {
  tftInit();        // ← re-init after any I2C ops that toggled A4/RST
  drawHeader();
  drawGraph();
  drawFooter();
}


// ════════════════════════════════════════════════════════════
//  MEASUREMENT
// ════════════════════════════════════════════════════════════
void measure() {
  if (!bmp.performReading()) {
    Serial.println(F("ERROR: BMP580 read failed!"));
    return;
  }

  float rawP = bmp.pressure;
  float p    = round(rawP * 2.0f) / 2.0f;

  if (p < 300.0f || p > 1250.0f) {
    Serial.print(F("REJECTED — invalid: ")); Serial.println(p); return;
  }

  if (dataCount < HISTORY_LEN) {
    hist[dataCount++] = p;
  } else {
    for (int i = 0; i < HISTORY_LEN - 1; i++) hist[i] = hist[i + 1];
    hist[HISTORY_LEN - 1] = p;
  }

  Serial.print(F("P = ")); Serial.print(p, 1);
  Serial.print(F(" hPa  | stored: ")); Serial.println(dataCount);

  saveToEEPROM();   // A4/RST gets disturbed here — tftInit() in redraw() fixes it
  redraw();
}


// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);
  Wire.begin();
  Serial.println(F("\n== Pressure Logger TFT v3 =="));

  // ── Detect TFT controller ID and store globally ───────────
  g_tftID = tft.readID();
  Serial.print(F("TFT ID detected: 0x"));
  Serial.println(g_tftID, HEX);

  // Fallback for shields where readID() returns 0x0000 or 0xFFFF
  if (g_tftID == 0x0000 || g_tftID == 0xFFFF) {
    g_tftID = 0x9341;
    Serial.println(F("readID() returned garbage — falling back to 0x9341"));
  }

  tftInit();
  tft.fillScreen(C_BG);

  // ── Boot splash ───────────────────────────────────────────
  tft.setTextColor(C_LINE); tft.setTextSize(2);
  tft.setCursor(50, 70);    tft.print(F("Pressure Logger"));
  tft.setTextColor(C_DIM);  tft.setTextSize(1);
  tft.setCursor(96, 96);    tft.print(F("TFT Edition  v3"));
  tft.setCursor(68, 116);   tft.print(F("Initialising sensor..."));

  // ── BMP580 init ───────────────────────────────────────────
  if (!bmp.begin((uint8_t)0x46)) {
    Serial.println(F("0x46 failed — trying 0x47..."));
    if (!bmp.begin((uint8_t)0x47)) {
      tftInit();
      tft.fillScreen(C_BG);
      tft.setTextColor(C_FALL); tft.setTextSize(2);
      tft.setCursor(20, 90); tft.print(F("BMP580 NOT FOUND!"));
      tft.setTextColor(C_DIM); tft.setTextSize(1);
      tft.setCursor(20, 116); tft.print(F("SDA=A4  SCL=A5  VCC=3.3V"));
      while (true) delay(1000);
    }
  }
  bmp.setTemperatureOversampling(BMP5_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP5_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP5_IIR_FILTER_COEFF_15);
  bmp.setOutputDataRate(BMP5_ODR_01_HZ);
  Serial.println(F("BMP580 OK."));

  // ── Sensor warm-up (3 × 10 s) ────────────────────────────
  Serial.println(F("Warming up (30 s)..."));
  tftInit();   // re-init after BMP580 I2C init
  tft.fillScreen(C_BG);
  tft.setTextColor(C_TEXT); tft.setTextSize(2);
  tft.setCursor(64, 70);    tft.print(F("Warming up..."));
  tft.setTextColor(C_DIM);  tft.setTextSize(1);
  tft.setCursor(68, 96);    tft.print(F("3 readings x 10 s = 30 s"));

  for (int i = 0; i < 3; i++) {
    bmp.performReading();
    Serial.print(F("Stabilising... ")); Serial.print(i+1); Serial.println(F("/3"));
    tftInit();   // re-init after each I2C read
    tft.fillRect(68, 112, 140, 20, C_BG);
    tft.setTextColor(C_LINE); tft.setTextSize(2);
    tft.setCursor(68, 112);
    tft.print(i + 1); tft.print(F(" / 3"));
    delay(10000);
  }
  Serial.println(F("Sensor stable."));

  // ── EEPROM restore ────────────────────────────────────────
  loadFromEEPROM();
  // A4/RST has been toggled by EEPROM I2C reads — tftInit() in
  // redraw() will fix it automatically.

  // ── First draw ────────────────────────────────────────────
  measure();   // redraw() is called inside measure()
  lastMs = millis();
}


// ════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════
void loop() {
  if (millis() - lastMs >= INTERVAL_MS) {
    lastMs = millis();
    measure();
  }
}
