# Learnings

Hard-won lessons from the pressure logger's evolution. Each entry describes
something that cost real debugging time and is worth remembering before the
next hardware or protocol change.

## AT24C08 EEPROM addressing is not AT24C32 addressing

The 24Cxx EEPROM family splits in half on how memory is addressed:

- **AT24C01 .. AT24C16** use an **8-bit byte address** sent in **one byte**.
  Memory is organized as 256-byte pages, and the page index is encoded into
  bits 0–2 of the I²C device address (so an AT24C08 responds at 0x50, 0x51,
  0x52, and 0x53 — one address per 256-byte page).
- **AT24C32 and larger** use a flat **16-bit byte address** sent as two
  bytes (MSB first, then LSB) and respond at a single I²C device address.

The bug: our Arduino sketches hard-coded the two-byte form. On an AT24C08,
the second "address" byte was accepted as a data byte and clobbered the
real data that followed. Every `writeEEPROM(N, val)` effectively wrote one
byte — always at physical address 1 — overwriting the previous call. On
boot, `storedCount` read back nonsense, failed the range check, and the
sketch dutifully reported *"No valid history found in EEPROM. Starting
fresh."* every single time.

Fix in `pressure_logger_OLED_BMP280_v4`:

```c
#define EEPROM_ADDR_BYTES 1   // 1 for AT24C01..C16, 2 for AT24C32+
```

```c
void writeEEPROM(int address, byte val) {
  Wire.beginTransmission(EEPROM_ADDR);
#if EEPROM_ADDR_BYTES == 2
  Wire.write((byte)(address >> 8));
#endif
  Wire.write((byte)(address & 0xFF));
  Wire.write(val);
  Wire.endTransmission();
  delay(5);
}
```

Plus a boot-time self-test: write `0xA5` to a scratch byte, read it back,
report OK or FAIL on serial. If the readback is wrong, `EEPROM_ADDR_BYTES`
is almost certainly set for the wrong chip.

## BMP280 IIR filter at hourly intervals makes the sensor feel "dumb"

The BMP280's hardware IIR filter is designed for continuous high-rate
sampling. At `FILTER_X16`, each new measurement contributes only ~6% to the
filtered output: `new = (old × 15 + raw) / 16`.

At one reading per hour, that means a real pressure step takes roughly
**2–3 days** to fully reflect in the filter output. The symptom was that
rising pressure looked flat; the arrow stayed "steady" when it should have
been trending up; and the recorded history visibly lagged real weather.

Fix in `v2`:
- `FILTER_OFF` instead of `FILTER_X16` — no hardware filter at all.
- OSR ×16 pressure oversampling is kept (that's done per measurement, no
  temporal lag).
- Noise reduction is now done in software: 8 forced measurements averaged
  per cycle. √8 ≈ 2.8× noise reduction, bounded to a 1-second burst — no
  multi-day lag.

The same analysis applies to the BMP580 variant: `FILTER_COEFF_15` on a
once-per-hour cron job will visibly lag reality. Software averaging the
reads is cleaner.

## Telegram `getUpdates` has two silent failure modes

1. **No prior messages → empty response.** The bot has to receive at least
   one message before `getUpdates` has anything to return. New bots look
   broken until you send them a `/start` first.
2. **Active webhook blocks polling entirely.** If the bot has a webhook
   configured (even one pointed at a long-dead URL), `getUpdates` returns
   `409 Conflict` and never produces anything. Delete the webhook first:
   `curl "https://api.telegram.org/bot<TOKEN>/deleteWebhook"`.

Related, from one real debugging session: an URL typo `api.telegram.org/<TOKEN>`
instead of `api.telegram.org/bot<TOKEN>` returns a plausible-looking error
("Not Found") that looks like a token problem rather than a URL problem.
Double-check the `bot` prefix is there.

## Telegram webhooks pointed at `webhook.site` are a token-exposure vector

A `webhook.site` URL used for inspection is public and persistent. Anyone
with that inspector URL can see every update Telegram forwards — including
the bot's own `message_id` and chat IDs. More importantly, until the
webhook is deleted, Telegram will keep posting to it. Treat any token that
has ever had its webhook pointed at `webhook.site` as potentially
compromised: revoke it in BotFather and reissue.

## Traditional cron beats workflow-automation platforms for simple pushes

For "take a reading, render a chart, post to Telegram, run hourly," the
Python-plus-cron combination is more reliable, lower latency, and far
simpler to debug than any workflow-automation platform (n8n, Zapier,
Make.com, etc.). AI-agent runners add a whole new category of failure modes
without earning their keep for a 20-line Python script. Save the agent
tools for jobs that genuinely need branching logic or external
integrations.
