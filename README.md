# Pressure Logger

A standalone atmospheric pressure logger with an on-device OLED display and
optional Telegram reporting via Raspberry Pi. Multiple hardware variants have
been built over this project's lifetime; the main build is Arduino + BMP280 +
SSD1306 OLED + AT24C08 EEPROM.

## Current main build (Arduino)

**File:** [`arduino/pressure_logger_OLED_BMP280_v4/`](arduino/pressure_logger_OLED_BMP280_v4)

- BMP280 pressure sensor on I²C
- 0.91" SSD1306 OLED (128×32)
- AT24C08 EEPROM for 24-hour history persistence across power cycles
- Software oversampling (8 forced measurements averaged per cycle) with the
  hardware IIR filter disabled — low lag, low noise
- Adaptive display window centered on the latest reading (±3 hPa)
- Boot-time EEPROM self-test
- Full change rationale in the file header; see also
  [`docs/learnings.md`](docs/learnings.md)

See [`docs/hardware.md`](docs/hardware.md) for the parts list and wiring.

## Raspberry Pi variants (BMP580 + Telegram)

**Files:** [`raspberry-pi/`](raspberry-pi)

Two charting styles sharing the same BMP580 driver and Telegram multi-target
dispatcher:

- `pressure_logger_v6_TFT.py` — 640×400 dark TFT-style chart (current)
- `pressure_logger_v5.py` — LED-matrix-style 32×8 dot chart (legacy)

Run under cron hourly; posts the rendered chart image to any mix of personal
chats, groups, forum topics, or public channels. Fill in `TELEGRAM_TOKEN` and
`TELEGRAM_TARGETS` before first run — see the comment block at the top of
each file.

> ⚠️ **Secrets.** The Telegram token lives in the script as a placeholder
> (`YOUR_BOT_TOKEN_HERE`). Do not commit a real token. The `.gitignore`
> here reserves `secrets.py` / `.env` for a future config split; until then,
> fill the token in your local copy and keep local copies off GitHub.

## Legacy builds

[`arduino/legacy/`](arduino/legacy) — kept for reference:

| Sketch | Sensor | Display | History | Status |
| --- | --- | --- | --- | --- |
| `pressure_logger_OLED_BMP280_v3` | BMP280 | OLED 128×32 | 24 h | Clipping fix in v4 |
| `pressure_logger_OLED_BMP280_v2` | BMP280 | OLED 128×32 | 24 h | Fixed-window ±10 hPa |
| `pressure_logger_OLED_BMP280_v1` | BMP280 | OLED 128×32 | 24 h | FILTER_X16 lag |
| `pressure_logger_2matrix_BMP280_v1` | BMP280 | 2× MAX7219 | 12 h | Has AT24C08 2-byte bug ⚠️ |
| `pressure_logger_4matrix_v1` | BMP580 | 4× MAX7219 | 28 h | Has AT24C08 2-byte bug ⚠️ |
| `pressure_logger_2matrix_v7` | BMP580 | 2× MAX7219 | 12 h | Has AT24C08 2-byte bug ⚠️ |

The three ⚠️-marked legacy sketches use hard-coded 2-byte EEPROM addressing,
which silently corrupts every write on an AT24C08. They still work with the
AT24C32 / AT24C256 EEPROMs they were originally wired against. If you flash
one of these onto hardware with an AT24C08, port the `EEPROM_ADDR_BYTES`
define + conditional `writeEEPROM/readEEPROM` from v4 first.

## Other sketches

[`other-sketches/qmc6310_magnetometer/`](other-sketches/qmc6310_magnetometer)
— a QMC6310 magnetometer logger for a GY-271 clone board. Unrelated to the
pressure logger but kept here as a separate side-project.

## Repository layout

```
pressure-logger/
├── README.md
├── LICENSE
├── CHANGELOG.md
├── .gitignore
├── docs/
│   ├── hardware.md    — parts list, wiring, I²C address map
│   └── learnings.md   — hard-won gotchas (AT24C08, BMP280 IIR, Telegram)
├── arduino/
│   ├── pressure_logger_OLED_BMP280_v4/   — current main
│   └── legacy/                            — earlier revisions
├── raspberry-pi/
│   ├── pressure_logger_v6_TFT.py          — current main
│   └── pressure_logger_v5.py              — earlier chart style
└── other-sketches/
    └── qmc6310_magnetometer/              — side project
```

## License

MIT. See [LICENSE](LICENSE).
