# Changelog

Per-file release notes. Each sketch keeps its own in-file changelog with full
rationale; this is the cross-sketch index.

## Arduino — current line (BMP280 + OLED + AT24C08)

### `pressure_logger_OLED_BMP280_v4` — EEPROM addressing fix (current)
- `EEPROM_ADDR_BYTES` compile-time define selects 1-byte vs 2-byte addressing
  (1 for AT24C01..C16, 2 for AT24C32 and up).
- Boot-time EEPROM self-test with 0xA5 scratch-byte write/read/verify.
- Value-range sanity check on restored history (first reading must be
  within 800–1200 hPa or the blob is treated as garbage).

### `pressure_logger_OLED_BMP280_v3` — adaptive display window
- `SCALE_HALF_RANGE` reduced from 10.0 → 3.0.
- Window now centered on the latest reading, not the running mean.
  Fixes clipping at the graph edges during sustained pressure trends.
- Out-of-window readings are skipped (line breaks at the gap) rather
  than clamped to the top/bottom row.

### `pressure_logger_OLED_BMP280_v2` — responsiveness
- Hardware IIR filter disabled (`FILTER_OFF`, was `FILTER_X16`).
  At 1 reading / hour, `FILTER_X16` added ~2–3 days of lag to real changes.
- Software oversampling: 8 forced measurements averaged per cycle.
- `0.5 hPa` rounding removed. Full float precision now stored, plotted,
  and logged (2 decimals in serial, 1 decimal on the OLED header).

### `pressure_logger_OLED_BMP280_v1` — OLED port
- Initial port from the 2-matrix layout to a 128×32 SSD1306 OLED.
- `HISTORY_LEN` bumped 12 → 24 to match the TFT Python version.
- GFX arrow drawing, dotted mean reference line.

## Arduino — legacy (BMP580 + MAX7219)

### `pressure_logger_2matrix_BMP280_v1` — BMP280 port
- First BMP280 variant. Replaces `Adafruit_BMP5xx` with `Adafruit_BMP280`,
  I²C address 0x76/0x77, `takeForcedMeasurement()` + `readPressure() / 100`.
- All other logic (fixed-window, EEPROM, trend arrow) unchanged from
  BMP580 v7. **Still uses the 2-byte EEPROM addressing.**

### `pressure_logger_4matrix_v1` — 4-matrix 28-hour window
- Port of 2-matrix v7 to a 32-column FC-16 chain.
- `HISTORY_LEN` 12 → 28, arrow moved from cols 12–15 to cols 28–31.

### `pressure_logger_2matrix_v7` — fixed scale
- Replaced auto-scale with a fixed ±10 hPa window centered on mean history.
  Same visual slope now means the same rate of pressure change.
- Previous history: v1 initial, v2 EEPROM, v3 load guard, v4 `px()` chain
  index fix, v5 sensor warm-up, v6 cleanup.

## Raspberry Pi (BMP580 + Telegram)

### `pressure_logger_v6_TFT.py` — TFT-style chart (current)
- 640×400 dark-navy chart image: large pressure value, trend arrow,
  colored RISE / STBL / FALL label, Δ hPa footer with MIN / MAX / count.
- Grid lines labeled in hPa; brighter grid line near the mean.

### `pressure_logger_v5.py` — LED-matrix chart (legacy)
- Previous chart style: 32×8 glowing-dot grid rendered as a PNG.
- Fixed ±10 hPa window centered on mean history (introduced in v5).
- Previous history: v1 initial, v2 register map fix (0x36/0x37 swap),
  v3 soft reset + data-ready poll + forced mode,
  v4 removed 0.5 hPa rounding, wider minimum scale window.

## Side projects

### `qmc6310_magnetometer`
- QMC6310 (GY-271 clone) magnetometer logger. Reinit-and-trigger
  sequence worked around CTRL1-accept-after-soft-reset-only quirk.
- 15-minute measurement interval, ASCII serial output (uT + heading + |B|).
