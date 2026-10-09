# Pressure Logger — Orange Pi Zero 2W Edition

Ported from the Raspberry Pi build. Zero code changes required —
the BMP580 on this hardware happens to land on I²C bus 1 at address `0x47`,
identical to the RPi wiring.

## Hardware

- **Board:** Orange Pi Zero 2W (2 GB)
- **Sensor:** BMP580 (SDO tied high → address `0x47`)
- **I²C bus:** bus 1 (`/dev/i2c-1`)
- **Mounted on:** LEGO baseplate (the right choice)

## Wiring (BMP580 → Zero 2W)

Same physical pins as Raspberry Pi — compatible header layout:

| BMP580 | Zero 2W Pin | Signal |
|--------|-------------|--------|
| VCC    | Pin 1       | 3.3V   |
| GND    | Pin 6       | GND    |
| SDA    | Pin 3       | I²C SDA|
| SCL    | Pin 5       | I²C SCL|

> ⚠️ Use 3.3V only — BMP580 is NOT 5V tolerant.

## Setup

### 1. Flash OS

Download **Debian Bookworm Server** from [orangepi.org](http://www.orangepi.org)
for the Zero 2W. Flash with Balena Etcher.

Default login: `orangepi` / `orangepi`

### 2. Enable I²C

```bash
sudo orangepi-config
# System → Hardware → enable i2c1 → Save → Reboot
```

### 3. Verify sensor

```bash
sudo apt install i2c-tools -y
sudo i2cdetect -y 1
# BMP580 appears at 0x47
```

### 4. Install Python dependencies

```bash
sudo apt install python3-pip python3-venv -y
python3 -m venv ~/press-track
source ~/press-track/bin/activate
pip install smbus2 Pillow python-telegram-bot
```

### 5. Configure and run

Edit `pressure_logger_v5.py` — fill in `TELEGRAM_TOKEN` and `TELEGRAM_TARGETS`,
then:

```bash
source ~/press-track/bin/activate
python pressure_logger_v5.py
```

### 6. Cron (hourly)

```bash
crontab -e
# Add:
0 * * * * /home/orangepi/press-track/bin/python /home/orangepi/press-track/pressure_logger_v5.py >> /home/orangepi/press-track/pressure.log 2>&1
```

## Verified working

Tested 2026-10-09 — sensor detected at `0x47` on bus 1, chart posted to
Telegram successfully. Pressure reading: 995.1 hPa (falling trend,
matching official forecast for Dnipro, Ukraine 🇺🇦).
