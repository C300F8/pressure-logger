# Hardware

## Current main build (Arduino)

### Bill of materials

| Part | Role | Notes |
| --- | --- | --- |
| Arduino Uno or Pro Mini | Controller | Any 5 V ATmega328P board |
| BMP280 pressure sensor | Barometer | Bare module or AHT20+BMP280 combo |
| 0.91" SSD1306 OLED | Display | 128×32, I²C (4-pin: GND / VCC / SCL / SDA) |
| AT24C08 EEPROM | History storage | Any 24Cxx works — set `EEPROM_ADDR_BYTES` to match |
| Breadboard + jumper wires | Prototyping | |

### Wiring

Everything shares the same I²C bus on Arduino pins **A4 (SDA)** and **A5 (SCL)**.
VCC and GND are the only other pins per device.

```
Arduino Uno
   A4 (SDA) ─┬───── BMP280 SDA
             ├───── OLED   SDA
             └───── EEPROM SDA

   A5 (SCL) ─┬───── BMP280 SCL
             ├───── OLED   SCL
             └───── EEPROM SCL

   3.3V ───── BMP280 VDD   (bare BMP280 — strict 3.3 V)
   5V   ─┬── OLED VCC      (SSD1306 modules usually regulate to 3.3 V internally)
         └── EEPROM VCC    (AT24Cxx breakout — 5 V typical)
   GND  ──── all GND
```

Combo boards (AHT20+BMP280) have an onboard LDO and accept either 3.3 V or 5 V.

### I²C address map

| Device | Address | Notes |
| --- | --- | --- |
| BMP280 | 0x76 (0x77) | 0x76 on most modules, 0x77 when SDO is pulled high |
| SSD1306 OLED | 0x3C (0x3D) | 0x3C is far more common on 0.91" modules |
| AT24C08 EEPROM | 0x50..0x53 | Lower 2 bits of address select the 256-byte page; only page 0 is used |

No conflicts between these three devices. The sketch tries each sensor's
primary address first and falls back to the secondary.

### Pull-ups

Each module ships with its own I²C pull-ups. Three in parallel bring the
effective bus pull-up down to roughly 1.5 kΩ — technically out of spec but
usually works. If I²C transactions start getting flaky, lift the SDA/SCL
pull-ups on one or two of the breakouts (typically the OLED or EEPROM) to
return the bus to a healthier impedance.

## Raspberry Pi variants

### Bill of materials

| Part | Role | Notes |
| --- | --- | --- |
| Raspberry Pi 3 or 4 | Host | Running Ubuntu or Raspberry Pi OS |
| BMP580 pressure sensor | Barometer | Bosch's higher-precision successor to BMP280 |
| USB webcam | Optional capture | Driven by `fswebcam` for occasional snapshots |

### Wiring (BMP580 → Pi header)

```
Raspberry Pi 40-pin header
   Pin  1 (3.3V)  ───── BMP580 VCC     ← strict 3.3 V, not 5 V
   Pin  3 (GPIO2) ───── BMP580 SDA
   Pin  5 (GPIO3) ───── BMP580 SCL
   Pin  6 (GND)   ───── BMP580 GND
```

Enable I²C on the Pi: `sudo raspi-config` → **Interface Options** → **I²C**.

### I²C address map

| Device | Address | Notes |
| --- | --- | --- |
| BMP580 | 0x46 (0x47) | 0x46 when SDO is low; 0x47 when SDO is high |

## QMC6310 magnetometer (side project)

### Wiring

```
Arduino Uno
   VCC ────── GY-271 VCC   (3.3 V or 5 V — module has regulator)
   GND ────── GY-271 GND
   A4 (SDA) ── GY-271 SDA
   A5 (SCL) ── GY-271 SCL
```

### I²C address

| Device | Address | Notes |
| --- | --- | --- |
| QMC6310 | 0x2C | Chip ID at register 0x00 reads 0x80 |

Chip quirk: CTRL1 (register 0x09) only accepts writes immediately after a
soft reset. The sketch runs the full reset + unlock + config sequence before
every single measurement — adds ~300 ms, invisible at a 15-minute cadence.
