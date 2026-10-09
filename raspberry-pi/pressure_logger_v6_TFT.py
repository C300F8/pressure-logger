#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║         Atmospheric Pressure Logger — Raspberry Pi Edition       ║
╠══════════════════════════════════════════════════════════════════╣
║  Hardware : Raspberry Pi 3/4 + BMP580 sensor                    ║
║  Features :                                                      ║
║    • Reads pressure from BMP580 via I2C (smbus2)                ║
║    • Logs timestamp + hPa to CSV file every hour                 ║
║    • Generates TFT-style chart image (640×400 px)               ║
║    • Shows 24h line graph + colored trend arrow + numeric info  ║
║    • Posts chart to Telegram via Bot API                         ║
║    • Multi-target: personal chat, groups, topics, channels       ║
╠══════════════════════════════════════════════════════════════════╣
║  Wiring (BMP580 → Raspberry Pi):                                 ║
║    VCC → Pin 1  (3.3V)   ← NOT 5V!                              ║
║    GND → Pin 6  (GND)                                           ║
║    SDA → Pin 3  (GPIO2)                                         ║
║    SCL → Pin 5  (GPIO3)                                         ║
╠══════════════════════════════════════════════════════════════════╣
║  Changelog:                                                      ║
║    v1 — initial version                                          ║
║    v2 — fixed register map (0x36/0x37 were swapped)             ║
║    v3 — added soft reset, data-ready polling, forced mode        ║
║    v4 — removed 0.5 hPa rounding, wider minimum scale window    ║
║    v5 — replaced auto-scale with fixed ±10 hPa window           ║
║    v6 — replaced LED matrix chart with TFT-style colour image:  ║
║         • 640×400 px dark-navy display aesthetic                 ║
║         • orange line graph, grid lines labeled in hPa          ║
║         • colour trend arrow: green ↑ / white → / red ↓         ║
║         • header: large pressure + RISE/FALL/STBL + Δ hPa       ║
║         • footer: MIN · MAX · readings count                     ║
║         • Telegram caption unchanged from v5                     ║
╠══════════════════════════════════════════════════════════════════╣
║  Setup:                                                          ║
║    sudo raspi-config → Interface Options → I2C → Enable         ║
║    source ~/press-track/bin/activate                             ║
║    pip install smbus2 Pillow python-telegram-bot                 ║
╠══════════════════════════════════════════════════════════════════╣
║  Cron (hourly, using venv python):                               ║
║    0 * * * * /home/orph/press-track/bin/python \                ║
║              /home/orph/press-track/pressure_logger.py \        ║
║              >> /home/orph/press-track/pressure.log 2>&1        ║
╚══════════════════════════════════════════════════════════════════╝
"""

import math
import time
import csv
import asyncio
import datetime
from pathlib import Path

import smbus2
from PIL import Image, ImageDraw, ImageFont
from telegram import Bot


# ══════════════════════════════════════════════════════════════════
#  USER CONFIGURATION — edit these before first run
# ══════════════════════════════════════════════════════════════════

TELEGRAM_TOKEN = "YOUR_BOT_TOKEN_HERE"

TELEGRAM_TARGETS = [
    {
        "label"            : "My personal chat",
        "chat_id"          : "YOUR_CHAT_ID_HERE",
        "message_thread_id": None,
    },
    # ── Additional targets — uncomment and fill in as needed ──────
    # {
    #     "label"            : "Weather group – #pressure topic",
    #     "chat_id"          : "-1001234567890",
    #     "message_thread_id": 12345,
    # },
    # {
    #     "label"            : "Public channel",
    #     "chat_id"          : "@my_pressure_log",
    #     "message_thread_id": None,
    # },
]

BASE_DIR      = Path(__file__).parent
DATA_FILE     = BASE_DIR / "pressure_log.csv"
IMAGE_FILE    = BASE_DIR / "pressure_chart.png"
HISTORY_HOURS = 24

# Fixed ±10 hPa window centered on mean pressure of current history.
# Same slope = same rate of pressure change across all charts.
SCALE_HALF_RANGE = 10.0   # hPa

DEBUG = False


# ══════════════════════════════════════════════════════════════════
#  BMP580 SENSOR DRIVER  (unchanged from v5)
# ══════════════════════════════════════════════════════════════════

class BMP580:
    """
    BMP580 I2C driver using smbus2.
    All compensation done in hardware — raw output is calibrated Pa.
    Pressure  : raw / 64.0 / 100.0  = hPa
    Temperature: raw / 65536.0       = °C
    """

    ADDRESSES        = (0x46, 0x47)
    EXPECTED_CHIP_ID = 0x50

    REG_CHIP_ID    = 0x01
    REG_INT_STATUS = 0x27
    REG_TEMP_DATA  = 0x1D
    REG_PRESS_DATA = 0x20
    REG_DSP_IIR    = 0x31
    REG_OSR_CONFIG = 0x36
    REG_ODR_CONFIG = 0x37
    REG_CMD        = 0x7E

    SOFT_RESET  = 0xB6
    OSR_VALUE   = 0x52   # temp×4, press×16, press_en=1
    IIR_VALUE   = 0x12   # IIR coefficient 4 for both channels
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
                    print(f"  BMP580 found at 0x{addr:02X}  (chip_id=0x{cid:02X} ✓)")
                    return
                else:
                    print(f"  0x{addr:02X}: wrong chip_id=0x{cid:02X}")
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
        self.bus.write_byte_data(self.addr, self.REG_DSP_IIR,    self.IIR_VALUE)
        self.bus.write_byte_data(self.addr, self.REG_ODR_CONFIG, self.ODR_STANDBY)
        time.sleep(0.1)

        if DEBUG:
            osr = self.bus.read_byte_data(self.addr, self.REG_OSR_CONFIG)
            iir = self.bus.read_byte_data(self.addr, self.REG_DSP_IIR)
            odr = self.bus.read_byte_data(self.addr, self.REG_ODR_CONFIG)
            print(f"  DEBUG OSR=0x{osr:02X} IIR=0x{iir:02X} ODR=0x{odr:02X}")

    @property
    def pressure(self) -> float:
        self.bus.write_byte_data(self.addr, self.REG_ODR_CONFIG, self.ODR_FORCED)
        drdy = False
        for _ in range(100):
            if self.bus.read_byte_data(self.addr, self.REG_INT_STATUS) & 0x01:
                drdy = True
                break
            time.sleep(0.01)

        data  = self.bus.read_i2c_block_data(self.addr, self.REG_TEMP_DATA, 6)
        raw_p = (data[5] << 16) | (data[4] << 8) | data[3]

        if DEBUG:
            raw_t  = (data[2] << 16) | (data[1] << 8) | data[0]
            hpa    = (raw_p / 64.0) / 100.0
            print(f"  DEBUG drdy={drdy}  temp={(raw_t/65536.0):.1f}°C  "
                  f"press={hpa:.4f} hPa  "
                  f"{'✓' if 800 < hpa < 1200 else '✗ WRONG'}")

        return (raw_p / 64.0) / 100.0


# ══════════════════════════════════════════════════════════════════
#  DATA LOGGING  (unchanged from v5)
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
#  TREND DETECTION  (unchanged from v5)
# ══════════════════════════════════════════════════════════════════

def get_trend(history: list) -> str:
    if len(history) < 2:
        return "stable"
    delta = history[-1][1] - history[max(0, len(history) - 4)][1]
    if delta >=  1.0: return "rising"
    if delta <= -1.0: return "falling"
    return "stable"


def trend_delta(history: list) -> float:
    """Raw Δ hPa vs ~3 hours ago (used for the numeric label in header)."""
    if len(history) < 2:
        return 0.0
    return history[-1][1] - history[max(0, len(history) - 4)][1]


# ══════════════════════════════════════════════════════════════════
#  IMAGE GENERATION — TFT display style  (new in v6)
# ══════════════════════════════════════════════════════════════════

# ── Image dimensions ─────────────────────────────────────────────
IMG_W    = 640
IMG_H    = 400
HEADER_H = 80     # header band height (y = 0 … HEADER_H-1)
FOOTER_Y = 362    # footer starts here (y = FOOTER_Y … IMG_H-1)

# Graph area
GX1 = 82          # left edge  (room for Y-axis labels)
GX2 = 628         # right edge (small margin from border)
GY1 = HEADER_H    # top    = 80
GY2 = FOOTER_Y    # bottom = 362
GW  = GX2 - GX1  # 546 px wide
GH  = GY2 - GY1  # 282 px tall

# ── Color palette  (matches the TFT display photo) ───────────────
C_BG     = ( 10,  14,  28)   # dark navy background
C_PANEL  = ( 18,  24,  50)   # slightly lighter — header / footer
C_BORDER = ( 35,  50, 100)   # thin border between panel and graph
C_GRID   = ( 28,  48, 102)   # dim blue grid lines
C_MGRID  = ( 50,  82, 162)   # brighter — grid line nearest mean
C_AXIS   = ( 72, 100, 162)   # axis lines
C_LINE   = (255, 162,  20)   # orange data line
C_DOT    = (255, 212,  90)   # amber dot — latest reading
C_TEXT   = (215, 228, 255)   # white-ish primary text
C_DIM    = (120, 148, 200)   # dimmer secondary text / axis labels
C_RISE   = ( 38, 210,  58)   # green — rising trend
C_FALL   = (222,  44,  44)   # red   — falling trend
C_STEADY = (215, 228, 255)   # white — stable trend


def _load_font(size: int) -> ImageFont.ImageFont:
    """
    Try common TTF paths on Raspberry Pi OS / Debian / Ubuntu.
    Falls back to PIL's built-in bitmap font if none are found.
    The built-in font is tiny (10px) — a warning is printed.
    """
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf",
        "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
    ]
    for path in candidates:
        try:
            return ImageFont.truetype(path, size)
        except Exception:
            continue
    print(f"  WARNING: No TTF font found — falling back to bitmap (size {size} ignored).")
    print("  Install fonts:  sudo apt install fonts-dejavu-core")
    return ImageFont.load_default()


# Pre-load font sizes used in the chart
_F_LARGE  = None   # pressure value  ~52px
_F_MED    = None   # labels          ~20px
_F_SMALL  = None   # axis / footer   ~14px

def _init_fonts():
    global _F_LARGE, _F_MED, _F_SMALL
    _F_LARGE = _load_font(52)
    _F_MED   = _load_font(20)
    _F_SMALL = _load_font(14)


def _press_to_y(p: float, lo: float, hi: float) -> int:
    """Map a pressure value to a Y pixel inside the graph (clamped)."""
    t = max(0.0, min(1.0, (p - lo) / (hi - lo)))
    return int(GY2 - t * GH + 0.5)


def _idx_to_x(i: int, count: int) -> int:
    """Map reading index to X pixel. Newest reading is always at GX2."""
    if count <= 1:
        return GX2
    return int(GX1 + i / (count - 1) * GW + 0.5)


def _draw_arrow_up(draw: ImageDraw.Draw, cx: int, cy: int, color: tuple):
    """↑ Up arrow centred on (cx, cy). Head points up, stem below."""
    hw = 20   # head half-width
    hh = 22   # head height
    sw = 9    # stem half-width
    sh = 22   # stem height
    # Triangle head
    draw.polygon([(cx, cy - hh - sh // 2),
                  (cx - hw, cy - sh // 2),
                  (cx + hw, cy - sh // 2)], fill=color)
    # Rectangle stem
    draw.rectangle([cx - sw, cy - sh // 2,
                    cx + sw, cy + sh], fill=color)


def _draw_arrow_down(draw: ImageDraw.Draw, cx: int, cy: int, color: tuple):
    """↓ Down arrow centred on (cx, cy). Head points down, stem above."""
    hw = 20
    hh = 22
    sw = 9
    sh = 22
    draw.rectangle([cx - sw, cy - sh,
                    cx + sw, cy + sh // 2], fill=color)
    draw.polygon([(cx, cy + hh + sh // 2),
                  (cx - hw, cy + sh // 2),
                  (cx + hw, cy + sh // 2)], fill=color)


def _draw_arrow_steady(draw: ImageDraw.Draw, cx: int, cy: int, color: tuple):
    """→ Steady arrow centred on (cx, cy). Horizontal, head on right."""
    sh = 8    # shaft half-height
    sl = 30   # shaft length (left of head)
    hw = 20   # head half-height
    hl = 18   # head length
    draw.rectangle([cx - sl, cy - sh,
                    cx + 4,  cy + sh], fill=color)
    draw.polygon([(cx + 4,  cy - hw),
                  (cx + hl, cy),
                  (cx + 4,  cy + hw)], fill=color)


def _draw_header(draw: ImageDraw.Draw, history: list):
    """Header band: large pressure + coloured trend arrow + delta."""
    draw.rectangle([0, 0, IMG_W, HEADER_H], fill=C_PANEL)
    draw.line([(0, HEADER_H - 1), (IMG_W, HEADER_H - 1)], fill=C_BORDER, width=2)

    if not history:
        draw.text((10, 16), "--- hPa", font=_F_LARGE, fill=C_TEXT)
        return

    # ── Large pressure value ──────────────────────────────────────
    p_now = history[-1][1]
    draw.text((10, 14), f"{p_now:.1f}", font=_F_LARGE, fill=C_TEXT)
    draw.text((200, 46), "hPa",         font=_F_MED,   fill=C_DIM)

    # ── Trend colour + labels ─────────────────────────────────────
    trend = get_trend(history)
    delta = trend_delta(history)

    if trend == "rising":
        color, label = C_RISE,   "RISE"
        _draw_arrow_up    (draw, 448, 40, C_RISE)
    elif trend == "falling":
        color, label = C_FALL,   "FALL"
        _draw_arrow_down  (draw, 448, 40, C_FALL)
    else:
        color, label = C_STEADY, "STBL"
        _draw_arrow_steady(draw, 448, 40, C_STEADY)

    # Label "RISE / FALL / STBL"
    draw.text((488, 10), label, font=_F_MED, fill=color)

    # Δ value  "+2.5 hPa" or "-3.0 hPa"
    sign = "+" if delta >= 0 else ""
    draw.text((488, 40), f"{sign}{delta:.1f} hPa", font=_F_SMALL, fill=color)


def _draw_footer(draw: ImageDraw.Draw, history: list):
    """Footer band: MIN · MAX · count."""
    draw.rectangle([0, FOOTER_Y, IMG_W, IMG_H], fill=C_PANEL)
    draw.line([(0, FOOTER_Y), (IMG_W, FOOTER_Y)], fill=C_BORDER, width=2)

    if not history:
        return

    pressures = [p for _, p in history]
    p_min, p_max = min(pressures), max(pressures)
    count = len(pressures)

    y = FOOTER_Y + 12

    draw.text((10,  y), "MIN", font=_F_SMALL, fill=C_DIM)
    draw.text((42,  y), f"{p_min:.1f} hPa", font=_F_SMALL, fill=C_TEXT)

    draw.text((200, y), "MAX", font=_F_SMALL, fill=C_DIM)
    draw.text((232, y), f"{p_max:.1f} hPa", font=_F_SMALL, fill=C_TEXT)

    draw.text((470, y), f"{count} / {HISTORY_HOURS} h",
              font=_F_SMALL, fill=C_DIM)


def _draw_graph(draw: ImageDraw.Draw, history: list):
    """Graph area: grid, axis, orange line, amber dot on latest point."""
    # Clear graph area
    draw.rectangle([0, GY1, IMG_W, GY2], fill=C_BG)

    if not history:
        draw.text((GX1 + 10, GY1 + GH // 2 - 7),
                  "Waiting for first reading…",
                  font=_F_SMALL, fill=C_DIM)
        return

    pressures = [p for _, p in history]
    mean = sum(pressures) / len(pressures)
    lo   = mean - SCALE_HALF_RANGE
    hi   = mean + SCALE_HALF_RANGE

    # ── Horizontal grid lines every 5 hPa + Y-axis labels ────────
    grid_start = math.ceil(lo / 5.0) * 5.0
    gp = grid_start
    while gp <= hi + 0.01:
        y        = _press_to_y(gp, lo, hi)
        near_mean = abs(gp - mean) < 2.5
        draw.line([(GX1, y), (GX2, y)],
                  fill=C_MGRID if near_mean else C_GRID, width=1)
        # Y-axis label — right-aligned before GX1
        label = str(int(gp))
        bbox  = draw.textbbox((0, 0), label, font=_F_SMALL)
        lw    = bbox[2] - bbox[0]
        draw.text((GX1 - lw - 4, y - 7), label, font=_F_SMALL, fill=C_DIM)
        gp += 5.0

    # ── Axis lines ────────────────────────────────────────────────
    draw.line([(GX1, GY1), (GX1, GY2)], fill=C_AXIS, width=1)   # Y axis
    draw.line([(GX1, GY2), (GX2, GY2)], fill=C_AXIS, width=1)   # X axis

    # ── Data line (width=3 for visibility) ────────────────────────
    count = len(pressures)
    pts = [(_idx_to_x(i, count), _press_to_y(pressures[i], lo, hi))
           for i in range(count)]

    if count >= 2:
        draw.line(pts, fill=C_LINE, width=3)

    # ── Latest-reading dot ────────────────────────────────────────
    lx, ly = pts[-1]
    r = 7
    draw.ellipse([lx - r, ly - r, lx + r, ly + r], fill=C_DOT)
    draw.ellipse([lx - r, ly - r, lx + r, ly + r], outline=C_LINE, width=2)


def make_chart(history: list) -> Image.Image:
    """
    Build the TFT-style chart image (640×400 px).

    Layout:
        y =   0 …  79  — header: pressure value + trend arrow + Δ
        y =  80 … 361  — graph:  line chart with grid and axis labels
        y = 362 … 399  — footer: MIN · MAX · count / hours

    Scale (inherited from v5):
        Fixed ±SCALE_HALF_RANGE hPa window centered on mean of history.
        Same slope always represents the same rate of pressure change.
    """
    _init_fonts()
    img  = Image.new("RGB", (IMG_W, IMG_H), C_BG)
    draw = ImageDraw.Draw(img)

    _draw_header(draw, history)
    _draw_graph (draw, history)
    _draw_footer(draw, history)

    return img


# ══════════════════════════════════════════════════════════════════
#  TELEGRAM — multi-target posting  (unchanged from v5)
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
#  MAIN  (unchanged from v5)
# ══════════════════════════════════════════════════════════════════

def main():
    print("\n" + "=" * 55)
    print("  Atmospheric Pressure Logger — v6 — starting")
    if DEBUG:
        print("  *** DEBUG MODE ON — set DEBUG=False for production ***")
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
