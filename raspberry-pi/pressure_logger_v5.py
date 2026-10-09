#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║         Atmospheric Pressure Logger — Raspberry Pi Edition       ║
╠══════════════════════════════════════════════════════════════════╣
║  Hardware : Raspberry Pi 3/4 + BMP580 sensor                    ║
║  Features :                                                      ║
║    • Reads pressure from BMP580 via I2C (smbus2)                ║
║    • Logs timestamp + hPa to CSV file every hour                 ║
║    • Generates LED-matrix style chart image (32x8 dot grid)     ║
║    • Shows 24h line graph + trend arrow (up / right / down)     ║
║    • Posts chart to Telegram via Bot API                         ║
║    • Multi-target: personal chat, groups, topics, channels       ║
╠══════════════════════════════════════════════════════════════════╣
║  Wiring (BMP580 → Raspberry Pi):                                 ║
║    VCC → Pin 1  (3.3V)   ← NOT 5V!                              ║
║    GND → Pin 6  (GND)                                           ║
║    SDA → Pin 3  (GPIO2)                                         ║
║    SCL → Pin 5  (GPIO3)                                         ║
╠══════════════════════════════════════════════════════════════════╣
║  VERIFIED BMP580 Register Map (Bosch BMP5-Sensor-API source):   ║
║    0x01 — CHIP_ID       reads 0x50                              ║
║    0x27 — INT_STATUS    bit 0 = data ready flag                 ║
║    0x1D — TEMP_DATA     3 bytes (XLSB, LSB, MSB)               ║
║    0x20 — PRESS_DATA    3 bytes (XLSB, LSB, MSB)               ║
║    0x31 — DSP_IIR       IIR filter coefficients                 ║
║    0x36 — OSR_CONFIG    oversampling + pressure enable          ║
║    0x37 — ODR_CONFIG    data rate + power mode                  ║
║    0x7E — CMD           soft reset (write 0xB6)                 ║
╠══════════════════════════════════════════════════════════════════╣
║  Changelog:                                                      ║
║    v1 — initial version                                          ║
║    v2 — fixed register map (0x36/0x37 were swapped)             ║
║    v3 — added soft reset, data-ready polling, forced mode        ║
║    v4 — removed 0.5 hPa rounding (caused visible dot jumps),    ║
║         full float precision logged and charted,                 ║
║         wider minimum scale window (12 hPa instead of 10)       ║
║    v5 — replaced auto-scale with fixed ±10 hPa window           ║
║         centered on mean pressure; consecutive charts are now    ║
║         visually comparable — same slope = same pressure change  ║
╠══════════════════════════════════════════════════════════════════╣
║  Setup:                                                          ║
║    sudo raspi-config → Interface Options → I2C → Enable         ║
║    source ~/press-track/bin/activate                             ║
║    pip install smbus2 Pillow python-telegram-bot                 ║
╠══════════════════════════════════════════════════════════════════╣
║  Cron (hourly, using venv python):                               ║
║    0 * * * * /home/orph/press-track/bin/python \\                ║
║              /home/orph/press-track/pressure_logger.py \\        ║
║              >> /home/orph/press-track/pressure.log 2>&1        ║
╚══════════════════════════════════════════════════════════════════╝
"""

import time
import csv
import asyncio
import datetime
from pathlib import Path

import smbus2
from PIL import Image, ImageDraw
from telegram import Bot

# ══════════════════════════════════════════════════════════════════
#  USER CONFIGURATION — edit these before first run
# ══════════════════════════════════════════════════════════════════

TELEGRAM_TOKEN = "YOUR_BOT_TOKEN_HERE"   # From @BotFather

TELEGRAM_TARGETS = [
    {
        "label"            : "My personal chat",
        "chat_id"          : "YOUR_CHAT_ID_HERE",  # positive number
        "message_thread_id": None,
    },
    # ── Additional targets — uncomment and fill in as needed ──────
    # {
    #     "label"            : "Weather group – #pressure topic",
    #     "chat_id"          : "-1001234567890",   # negative = group
    #     "message_thread_id": 12345,              # topic thread ID
    # },
    # {
    #     "label"            : "Public channel",
    #     "chat_id"          : "@my_pressure_log", # @username
    #     "message_thread_id": None,
    # },
]

BASE_DIR      = Path(__file__).parent
DATA_FILE     = BASE_DIR / "pressure_log.csv"
IMAGE_FILE    = BASE_DIR / "pressure_chart.png"
HISTORY_HOURS = 24

# ── Chart scale ───────────────────────────────────────────────────
# Fixed ±10 hPa window (20 hPa total) centered on the mean pressure
# of the current history. This makes all charts visually comparable:
# the same line slope always means the same rate of pressure change,
# regardless of whether it's a calm day or a stormy one.
# Increase SCALE_HALF_RANGE if you live in a high-variability region.
SCALE_HALF_RANGE = 10.0   # hPa — total window = 2 × this value

# ── Debug flag ────────────────────────────────────────────────────
# True  = print raw I2C bytes and register readbacks (troubleshooting)
# False = clean output for production / cron use
DEBUG = False


# ══════════════════════════════════════════════════════════════════
#  BMP580 SENSOR DRIVER
# ══════════════════════════════════════════════════════════════════

class BMP580:
    """
    BMP580 I2C driver using smbus2.

    BMP580 key facts:
      • All temperature/pressure compensation is done in hardware.
        Raw output registers already contain calibrated fixed-point Pa.
        No calibration coefficients or software math needed.
      • Pressure  : raw_value / 64.0 = Pa  →  / 100.0 = hPa
      • Temperature: raw_value / 65536.0 = degrees Celsius
      • BOTH temp and pressure oversampling must be configured,
        even if only pressure is needed — the sensor uses temperature
        internally for pressure compensation.
      • Must poll INT_STATUS (0x27) bit 0 before reading data.
        If you read before drdy=1, you get 0x7F7F7F (sentinel = not ready).
    """

    ADDRESSES        = (0x46, 0x47)   # SDO low=0x46, SDO high=0x47
    EXPECTED_CHIP_ID = 0x50

    # ── Register addresses ──────────────────────────────────────────
    REG_CHIP_ID    = 0x01
    REG_INT_STATUS = 0x27
    REG_TEMP_DATA  = 0x1D
    REG_PRESS_DATA = 0x20
    REG_DSP_IIR    = 0x31
    REG_OSR_CONFIG = 0x36
    REG_ODR_CONFIG = 0x37
    REG_CMD        = 0x7E

    # ── Config values ───────────────────────────────────────────────
    SOFT_RESET = 0xB6
    OSR_VALUE  = 0x52   # temp x4, press x16, press_en=1
    IIR_VALUE  = 0x12   # IIR coefficient 4 for both

    ODR_STANDBY = 0x00
    ODR_FORCED  = 0x01

    DATA_NOT_READY = 0x7F7F7F

    def __init__(self):
        self.bus  = smbus2.SMBus(1)
        self.addr = None

        for addr in self.ADDRESSES:
            try:
                cid = self.bus.read_byte_data(addr, self.REG_CHIP_ID)
                if cid == self.EXPECTED_CHIP_ID:
                    self.addr = addr
                    self._configure()
                    print(f"  BMP580 found at 0x{addr:02X}  "
                          f"(chip_id=0x{cid:02X} ✓)")
                    return
                else:
                    print(f"  0x{addr:02X}: wrong chip_id=0x{cid:02X} "
                          f"(expected 0x{self.EXPECTED_CHIP_ID:02X})")
            except Exception as e:
                print(f"  0x{addr:02X}: not found ({e})")

        raise RuntimeError(
            "BMP580 not found!\n"
            "  → Check wiring (SDA=Pin3, SCL=Pin5, VCC=3.3V)\n"
            "  → Enable I2C: sudo raspi-config\n"
            "  → Scan bus:   sudo i2cdetect -y 1"
        )

    def _configure(self):
        print("  Soft resetting...")
        self.bus.write_byte_data(self.addr, self.REG_CMD, self.SOFT_RESET)
        time.sleep(0.1)
        self.bus.write_byte_data(self.addr, self.REG_OSR_CONFIG, self.OSR_VALUE)
        self.bus.write_byte_data(self.addr, self.REG_DSP_IIR, self.IIR_VALUE)
        self.bus.write_byte_data(self.addr, self.REG_ODR_CONFIG, self.ODR_STANDBY)
        time.sleep(0.1)

    @property
    def pressure(self) -> float:
        self.bus.write_byte_data(self.addr, self.REG_ODR_CONFIG, self.ODR_FORCED)
        drdy = False
        for _ in range(100):
            status = self.bus.read_byte_data(self.addr, self.REG_INT_STATUS)
            if status & 0x01:
                drdy = True
                break
            time.sleep(0.01)

        data = self.bus.read_i2c_block_data(self.addr, self.REG_TEMP_DATA, 6)
        raw_p = (data[5] << 16) | (data[4] << 8) | data[3]
        return (raw_p / 64.0) / 100.0


# ══════════════════════════════════════════════════════════════════
#  DATA LOGGING
# ══════════════════════════════════════════════════════════════════

def log_pressure(p: float):
    ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(DATA_FILE, "a", newline="") as f:
        csv.writer(f).writerow([ts, f"{p:.2f}"])
    print(f"  [{ts}] Logged: {p:.2f} hPa")


def load_history(hours: int = 24) -> list:
    if not DATA_FILE.exists():
        print("  No data file yet — first run.")
        return []

    cutoff = datetime.datetime.now() - datetime.timedelta(hours=hours)
    rows   = []

    with open(DATA_FILE, "r") as f:
        for row in csv.reader(f):
            try:
                ts = datetime.datetime.strptime(row[0], "%Y-%m-%d %H:%M:%S")
                p  = float(row[1])
                if ts >= cutoff and 300.0 < p < 1250.0:
                    rows.append((ts, p))
            except Exception:
                continue

    print(f"  Loaded {len(rows)} readings from last {hours}h.")
    return rows


# ══════════════════════════════════════════════════════════════════
#  TREND DETECTION
# ══════════════════════════════════════════════════════════════════

def get_trend(history: list) -> str:
    if len(history) < 2:
        return "stable"
    delta = history[-1][1] - history[max(0, len(history) - 4)][1]
    if delta >=  1.0: return "rising"
    if delta <= -1.0: return "falling"
    return "stable"


# ══════════════════════════════════════════════════════════════════
#  IMAGE GENERATION — LED matrix visual style
# ══════════════════════════════════════════════════════════════════

LED_COLS   = 32
LED_ROWS   = 8
LED_SIZE   = 22
LED_GAP    = 5
MATRIX_GAP = 12

BG_COLOR      = ( 15,  15,  15)
LED_OFF_COLOR = ( 45,  12,  12)
LED_ON_COLOR  = (255,  65,   0)


def col_x(c: int) -> int:
    return (MATRIX_GAP
            + (c // 8) * (8 * (LED_SIZE + LED_GAP) + MATRIX_GAP)
            + (c %  8) * (LED_SIZE + LED_GAP))


def row_y(r: int) -> int:
    return MATRIX_GAP + r * (LED_SIZE + LED_GAP)


def draw_led(draw: ImageDraw.Draw, col: int, row: int, on: bool):
    x, y  = col_x(col), row_y(row)
    color = LED_ON_COLOR if on else LED_OFF_COLOR
    draw.ellipse([x, y, x + LED_SIZE, y + LED_SIZE], fill=color)
    if on:
        r  = LED_SIZE // 5
        cx = x + LED_SIZE // 2
        cy = y + LED_SIZE // 2
        draw.ellipse([cx - r, cy - r, cx + r, cy + r],
                     fill=(255, 210, 160))


def make_chart(history: list) -> Image.Image:
    pressures = [p for _, p in history]
    img  = Image.new("RGB", (col_x(32), row_y(8)), BG_COLOR)
    draw = ImageDraw.Draw(img)
    lit  = set()

    if pressures:
        mean = sum(pressures) / len(pressures)
        lo   = mean - SCALE_HALF_RANGE
        hi   = mean + SCALE_HALF_RANGE
        rng  = hi - lo

        start_col = 23 - (len(pressures) - 1)
        for i, p in enumerate(pressures):
            col = start_col + i
            if 0 <= col <= 23:
                row_offset = int(((p - lo) / rng) * 7.0 + 0.5)
                lit.add((col, 7 - max(0, min(7, row_offset))))

        trend = get_trend(history)
        if trend == "rising":
            lit |= {(30, 1), (29, 2), (30, 2), (31, 2),
                    (30, 3), (30, 4), (30, 5)}
        elif trend == "falling":
            lit |= {(30, 2), (30, 3), (30, 4),
                    (29, 5), (30, 5), (31, 5), (30, 6)}
        else:
            lit |= {(28, 4), (29, 4), (30, 4), (31, 4),
                    (30, 3), (30, 5)}

    for c in range(LED_COLS):
        for r in range(LED_ROWS):
            draw_led(draw, c, r, (c, r) in lit)

    return img


# ══════════════════════════════════════════════════════════════════
#  TELEGRAM — multi-target posting
# ══════════════════════════════════════════════════════════════════

async def send_to_telegram(history: list):
    if not history:
        print("  No history yet — skipping Telegram.")
        return

    bot         = Bot(token=TELEGRAM_TOKEN)
    trend       = get_trend(history)
    emoji       = {"rising": "📈", "falling": "📉", "stable": "➡️"}[trend]
    trend_label = {"rising": "Rising ↑", "falling": "Falling ↓",
                   "stable": "Stable →"}[trend]

    caption = (
        f"🌡 *Pressure: {history[-1][1]:.1f} hPa* {emoji}\n"
        f"📊 Trend: {trend_label}\n"
        f"🕐 At {history[-1][0].strftime('%H:%M')}"
    )

    success = 0
    for target in TELEGRAM_TARGETS:
        label  = target.get("label", target["chat_id"])
        thread = target.get("message_thread_id")
        try:
            with open(IMAGE_FILE, "rb") as photo:
                kwargs = {
                    "chat_id"   : target["chat_id"],
                    "photo"     : photo,
                    "caption"   : caption,
                    "parse_mode": "Markdown",
                }
                if thread is not None:
                    kwargs["message_thread_id"] = thread
                await bot.send_photo(**kwargs)
            print(f"  ✓ Posted to: {label}")
            success += 1
        except Exception as e:
            print(f"  ✗ Failed:    {label} — {e}")

    print(f"  Telegram: {success}/{len(TELEGRAM_TARGETS)} targets succeeded.")


# ══════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════

def main():
    print("\n" + "=" * 55)
    print("  Atmospheric Pressure Logger — v5 — starting")
    print("=" * 55)

    print("\n[1/5] Initialising BMP580...")
    sensor = BMP580()

    print("\n[2/5] Warm-up (3 readings × 2s)...")
    for i in range(3):
        _ = sensor.pressure
        print(f"      Stabilising... {i+1}/3")
        time.sleep(2)
    print("      Sensor stable.")

    print("\n[3/5] Measuring...")
    p = sensor.pressure
    print(f"      Pressure: {p:.4f} hPa  →  logging as {p:.2f} hPa")

    if not (300.0 < p < 1250.0):
        print(f"  ERROR: Invalid reading ({p:.2f} hPa) — aborting!")
        return

    log_pressure(p)

    print(f"\n[4/5] Loading {HISTORY_HOURS}h history...")
    history = load_history(HISTORY_HOURS)

    print("\n[5/5] Generating chart and sending to Telegram...")
    make_chart(history).save(IMAGE_FILE)
    print(f"      Chart saved → {IMAGE_FILE}")
    asyncio.run(send_to_telegram(history))

    print("\n" + "=" * 55)
    print("  Done.")
    print("=" * 55 + "\n")


if __name__ == "__main__":
    main()
