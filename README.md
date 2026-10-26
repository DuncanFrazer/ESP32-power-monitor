# Power Monitor

A power-profiling tool for battery-powered devices, built around an ESP32
and an INA219 current/voltage sensor. It continuously samples voltage and
current, automatically detects and isolates periods of activity using a
configurable current threshold, and gives you a browser-based viewer for
inspecting the resulting traces.

## Features

- **Continuous voltage/current/power sampling** via an INA219 sensor on the ESP32
- **Automatic activity detection** using a hysteresis current threshold with debouncing, so brief noise spikes don't cause false triggers
- **Two connection options**, running the same sampling/detection logic:
  - **USB** — streams data over serial
  - **Wi-Fi** — streams data over a WebSocket, discoverable via mDNS (`.local` hostname) so the device's IP doesn't need to be looked up manually
- **On-device capture logging (Wi-Fi variant)** — activity detection and file logging run entirely on the ESP32 itself, so captures are recorded to the device's internal flash whenever it's powered, independent of whether a PC or browser is connected. A rolling-buffer strategy automatically deletes the oldest saved capture if flash space runs low, so logging never simply stops. See the storage note below for capacity limits.
- **Real timestamps (Wi-Fi variant)** — capture filenames and logged data use actual date/time (synced via NTP), not just time-since-boot, with an automatic fallback if sync isn't available. See the time sync note below.
- **Live trace view** in the browser, with detected activity periods shaded on the chart, when connected
- **Voltage and current plotted together**, each on its own axis, for both the live view and any loaded capture — not just current alone
- **Device file browser (Wi-Fi variant)** — list captures saved on the ESP32's flash and click one to load it straight into the trace view — with markers, point inspection, and export — independent of the live connection; download or delete from the same panel
- **Browser-side capture logging** — completed captures are also written to disk on the PC as individual CSV files as soon as they end while the browser is connected (via the File System Access API, with an automatic-download fallback in browsers that don't support it)
- **Timer markers** — set start/end markers (via buttons or by clicking the chart) to measure duration, average current, average power, and charge/energy consumed between them
- **Point inspection** — move the mouse anywhere across the plot (not just onto a line) and both the current and voltage traces highlight at that position, with a popup showing the time and both values anchored at the cursor; the Instant panel shows the same sample's voltage, current, power and state. Works on the live view, which keeps updating under a stationary cursor, and on loaded captures
- **Export** — save the currently viewed trace as a PNG image or CSV, automatically named after the capture being viewed (or a timestamp, for a live-session export) so exports don't collide or overwrite each other

## Repository structure

```
├── README.md
├── power_monitor.html          # Browser-based viewer/logger app
├── .gitignore
├── USB/
│   └── esp32_power_logger_usb/
│       └── esp32_power_logger_usb.ino
├── Wi-Fi/
│   └── esp_power_logger_wifi/
│       ├── esp32_power_logger_wifi.ino
│       ├── secrets_example.h    # Template for Wi-Fi credentials
│       └── secrets.h            # Your credentials (git-ignored)
└── ESP32-C3-INA226/             # Wi-Fi variant for ESP32-C3 Supermini + INA226
    └── esp32c3_power_logger_wifi_ina226/
        ├── esp32c3_power_logger_wifi_ina226.ino
        ├── secrets_example.h    # Template for Wi-Fi credentials
        └── secrets.h            # Your credentials (git-ignored)
```

Flash whichever sketch matches how you want to connect, then open
`power_monitor.html` in a browser and select the matching connection mode.

## Requirements

### Hardware
- ESP32 development board
- INA219 current/voltage sensor, wired to the ESP32's default I2C pins: SDA → GPIO21, SCL → GPIO22 (both sketches call `Wire.begin()` with no arguments, which uses these defaults — if you wire to different GPIOs, update to `Wire.begin(SDA_PIN, SCL_PIN)` in the sketch)

### ESP32-C3 Supermini + INA226 variant

`ESP32-C3-INA226/` is the Wi-Fi variant ported to an ESP32-C3 Supermini board with an INA226 sensor module instead of an ESP32 dev board with an INA219. It behaves identically — same on-device logging, wake/sleep state machine, pre/post-roll, HTTP/WebSocket protocol and timestamp format — so `power_monitor.html` works with it unchanged.

| INA226 module | Supermini |
|---|---|
| VCC | 3V3 |
| GND | GND (also common with the monitored circuit's ground) |
| SDA | GPIO4 |
| SCL | GPIO3 |
| ALERT | not connected |
| VIN+ / VIN− | in series with the monitored device's positive supply (VIN+ toward the source, VIN− toward the load); swap them if current reads negative |

The module's A0/A1 address pins are expected tied to GND (I2C address `0x40`). GPIO3/4 avoid the Supermini's strapping pins (GPIO2, 8, 9), on-board LED (GPIO8), BOOT button (GPIO9) and UART pins (GPIO20/21); change `I2C_SDA_PIN` / `I2C_SCL_PIN` to use others. Clone boards vary, so check your board's silkscreen.

Things to check on your hardware:
- **Shunt value.** `SHUNT_OHMS` / `MAX_EXPECTED_A` must match your module's shunt resistor, which varies between modules: R100 = 0.1 Ω, R010 = 0.01 Ω, R002 = 0.002 Ω. The defaults are set for an **R010 (0.01 Ω)** module; a wrong value scales every current reading by the ratio between the two (assuming 0.1 Ω on a 0.01 Ω shunt reads 10× too low, which can keep current below the wake threshold so nothing triggers). At 0.01 Ω the shunt reaches ~8 A with ~0.25 mA per count — ample here. `MAX_EXPECTED_A` must be at least ~0.26 A for 0.01 Ω or the calibration value overflows.
- **Arduino IDE settings.** Board "ESP32C3 Dev Module"; "USB CDC On Boot: Enabled" (otherwise `Serial` doesn't appear on the USB-C port); 4MB flash with a partition scheme that includes a SPIFFS/LittleFS partition (the default has one). If the port doesn't appear, hold BOOT while plugging in.
- **Hardware averaging.** The INA226 averages readings internally (`INA226_AVG_SETTING` × `INA226_CONV_SETTING`, ~35 ms per reading by default). Keep that below `SAMPLE_INTERVAL_MS`.
- **Single core.** Unlike the original ESP32, the C3 has one CPU core, so Wi-Fi/network handling shares it with the sampling loop. The flash-write stall behaviour described below applies equally here, and network activity may add some jitter of its own — `flush_log.csv` will show what it looks like on this hardware.
- **Current reads 0 / nothing triggers.** The web app's current axis starts at 0, so a *negative* current (VIN+/VIN− reversed) is clipped off the bottom of the chart and also never crosses the wake threshold — it looks exactly like 0. Hover the trace to see the real value in the Instant panel. `SENSOR_DEBUG_SERIAL` prints the raw bus voltage, shunt voltage and current to Serial once a second to tell the causes apart: ~0 mV shunt means it isn't in series with the load; a negative shunt voltage means reversed polarity; a small positive current usually means `SHUNT_OHMS` doesn't match the module (several common INA226 modules use a 2 mΩ shunt, not 0.1 Ω).
- **Wi-Fi flakiness.** Some Supermini clones connect unreliably at full TX power; `WIFI_REDUCE_TX_POWER` caps it.
- **Hostname.** It advertises `powerloggerc3.local` (the original ESP32/INA219 build keeps `powerlogger.local`) so both can run side by side — enter the matching name in the web app's address field. Change `MDNS_HOSTNAME` to rename it.

### Arduino IDE / board support
- [ESP32 board package](https://github.com/espressif/arduino-esp32) ("esp32 by Espressif Systems"), installed via Boards Manager

### Libraries

Install via the Arduino Library Manager (Sketch → Include Library → Manage Libraries).

**USB and original Wi-Fi variants:**
- `Adafruit INA219` (pulls in `Adafruit BusIO` as a dependency — accept the dependency install prompt if shown)

**Wi-Fi variant only** (`esp32_power_logger_wifi.ino`):
- `WebSockets` by Markus Sattler (repo: Links2004/arduinoWebSockets) — search "WebSockets" and match on the author, as there are similarly named alternatives
- `WiFi` and `ESPmDNS` are bundled with the ESP32 board package and don't need separate installation

**ESP32-C3 + INA226 variant** (`esp32c3_power_logger_wifi_ina226.ino`):
- `INA226_WE` by Wolfgang Ewald (instead of `Adafruit INA219`). The sketch uses the `INA226_`-prefixed constants (`INA226_CONTINUOUS`, `INA226_AVERAGE_16`, …) of recent versions; older versions use unprefixed names.
- `WebSockets` by Markus Sattler, as for the Wi-Fi variant above

**USB variant** (`esp32_power_logger_usb.ino`):
- No additional libraries beyond `Adafruit INA219` — serial communication uses the built-in `Serial` interface

### Browser
- Chrome or Edge recommended. USB mode requires the Web Serial API; Wi-Fi mode's optional auto-save-to-folder feature requires the File System Access API. Both are Chromium-only at present; other browsers can still use Wi-Fi mode for live viewing, with captures falling back to individual downloads instead of folder auto-save.

### On-device storage (Wi-Fi variant)

Captures are saved to the ESP32's internal flash filesystem (LittleFS), not an SD card. This is convenient (no extra hardware) but limited in capacity — a typical ESP32's default partition scheme leaves roughly 1.3–1.5MB free after the firmware itself. The default sample rate (`SAMPLE_INTERVAL_MS`, 20Hz) is tuned for observing gross wake/sleep behaviour rather than fine ms-level detail, both for this reason and because a lower rate directly reduces flash-write frequency (see the jitter note below) — raise it if you need finer resolution and are prepared to trade off storage headroom and jitter for it. Once the flash fills, the oldest captures are deleted automatically so writes keep succeeding rather than failing outright (how, exactly, is described below) — download captures regularly via the device file browser if you want to keep more history than the flash can hold.

**Pre-roll and post-roll context.** Each saved capture also includes `PREROLL_MS` (2s) of samples from before the wake was confirmed — kept in a small always-running ring buffer — and `POSTROLL_MS` (2s) after current drops below threshold, so the ramp up from idle and the ramp back down are visible either side of the active span rather than the trace starting and stopping abruptly. These context rows are written with `active=0`, so the viewer's shaded region still marks only the true above-threshold span. The capture's filename timestamp is the file's true first row (the oldest pre-roll sample), not the moment wake was confirmed. Both windows are specified in milliseconds and converted to sample counts via `SAMPLE_INTERVAL_MS`, so they stay correct if the rate changes. One minor side effect: if a new capture begins within a couple of seconds of the previous one ending, its pre-roll can overlap the previous file's post-roll in time.

**Capture finalization has a long-tail grace period.** Current dropping below `SLEEP_THRESHOLD_MA` doesn't immediately close the file — it pauses logging and starts a `SLEEP_GRACE_MS` clock (60s by default). If current rises back above `WAKE_THRESHOLD_MA` before that elapses, logging just resumes into the same still-open file; no new file, no new wake event. Only if the low-current state persists for the full grace window does the capture actually get finalized. This exists because a single real session can otherwise fragment into many separate files whenever the monitored device briefly reboots or dips to near-zero current without actually finishing its activity — one test produced 21 separate files from what should have been far fewer, with gaps between them of up to 33s; 60s gives headroom above that. This only helps when the ESP32 itself keeps running and sampling through the dip — if the ESP32 running this firmware loses power or resets, all RAM state (including the open file handle) is lost regardless of this setting, and a new file starts on the next wake no matter what.

**Retention and cleanup (ESP32-C3 + INA226 variant).** Cleanup happens at the *end* of each capture, while the device is idle, so the next capture finds room already waiting and nothing needs deleting while it is being recorded. The target is whichever is larger of: space for the next capture (`CAPTURE_BUDGET_FACTOR` × the largest of the last `RECENT_CAPTURES_TRACKED` captures, so it adapts if captures get longer), and a floor of `FREE_FLOOR_PERCENT` (20%) of the flash. The floor is there for speed, not capacity: LittleFS finds free blocks by traversing the whole filesystem each time its allocator's look-ahead window runs dry, which happens far more often when the flash is nearly full. Measured over ~100 captures with only ~20 KB kept free, every flush, write and file-open got steadily slower (mean flush 8 → 40 ms, capture-open 40 → 190 ms), which at 20 Hz costs samples. Set `FREE_FLOOR_PERCENT` to 0 for a purely next-capture-sized reserve — more retained history, slower writes as the flash fills.

Other details: cleanup deletes at most `MAX_DELETES_PER_CALL` files per capture so no single stall is long (it converges over a few captures); at capture *start* it only cleans up if space is genuinely short (`EMERGENCY_FREE_BYTES`); oldest-first order comes from an index held in RAM and built once at boot, so no directory scan is needed at capture time; and used space is tracked incrementally rather than asking LittleFS, which traverses the entire filesystem each time (that call alone had grown to ~170 ms per capture). The exact figure is re-read every `FS_RESYNC_EVERY_N_CLOSES` captures so the estimate can't drift.

**Flushing — deliberately almost never (ESP32-C3 + INA226 variant).** Each `flush()` stalls sampling (a flash erase + program), so captures are written straight through and committed when they close. The trade-off: until a capture closes, its data is only in RAM and not-yet-committed flash blocks, so **if power is lost or the chip resets mid-capture, that in-progress capture is likely lost** (LittleFS publishes a file's contents on sync/close); previously closed captures are unaffected. Two "something extreme" exceptions:
- a single capture that stays open for `SAFETY_FLUSH_MS` (2 minutes) gets a flush, bounding the data at risk in a very long session;
- if an open capture eats into the last of the free space (`EXTREME_LOW_FREE_BYTES`), the oldest captures are purged immediately — a one-off stall, accepted because the alternative is writes failing.

LittleFS also auto-commits its own internal write cache about every 4 KB of written data (the `slow_write` stalls, ~16–60 ms), independent of any of this; the main lever against that one is total data volume, which is why `SAMPLE_INTERVAL_MS` defaults to a conservative 20 Hz.

**Diagnostic log.** `DIAG_FLUSH_TIMING` (on by default) records timing to Serial and to `/flush_log.csv` on the device (rotating to `/flush_log_prev.csv` once it passes `DIAG_LOG_MAX_BYTES`; both are fetchable via `/download?file=…` even though they don't appear in the Device files list). Rows are held in RAM and written to flash only when a capture has just ended and the device is idle — writing them immediately had become the biggest single source of sampling gaps (150–440 ms each once ~100 captures were stored, landing in the middle of the capture being measured). Row types:
- `boot` — one per boot, with the chip's reset reason (`poweron`, `software`, `PANIC`, `TASK_WDT`, `BROWNOUT`, …), file count and flash usage, so unexpected reboots show up here.
- `open` — a capture starting: `ensureFreeSpace` time, `LittleFS.open()` time, header write time, files held, estimated free space.
- `close` — a capture finishing: bytes saved, files deleted by the cleanup, exact-resync time (when it ran), total cleanup time, files held, estimated free space and the cleanup target.
- `flush` — a safety flush (rare now).
- `slow_write` — an ordinary per-sample write that took longer than `SLOW_WRITE_THRESHOLD_MS` (40 ms, just under one sample interval).
- `extreme` — a mid-capture purge because the open capture was running out of space.

Watch for any duration approaching a few seconds — ESP32's watchdog timer will reset the whole device (losing the in-progress capture) if something blocks the CPU without yielding for too long, commonly around 5s by default.

*Note: the original ESP32 + INA219 Wi-Fi sketch (`esp32_power_logger_wifi.ino`) has not been given these changes. It still flushes every 5 s, deletes the oldest capture only once ~20 KB remain, and writes diagnostic rows to flash immediately — so on a busy device it will show the progressive slowdown described above.*

**Startup stall after a long idle period.** A separate, still only partially-understood ~1-1.3s stall has been observed specifically on the first sample after the device has been idle for hours (e.g. overnight). As a trial fix, `WiFi.setSleep(false)` disables Wi-Fi modem-sleep power saving in `setup()`, on the theory that the radio waking from a power-save state might be the cause — at the cost of a bit more idle power draw. Still being verified against real captures.

### Time sync (Wi-Fi variant)

The Wi-Fi sketch syncs the current time over NTP once connected, and uses it for both capture filenames (e.g. `cap_20260916_143205_123.csv`, in local time) and the `timestamp` column inside each CSV (e.g. `2026-09-16T14:32:05.123+01:00` — local time with an explicit UTC offset, so it's unambiguous regardless of what timezone anything reading it is in). This requires outbound UDP access to NTP servers (`pool.ntp.org` / `time.google.com` by default) on port 123, which most networks allow — check your network settings if sync fails. The timezone defaults to Europe/London (with automatic GMT/BST handling); change the `TIMEZONE_STRING` constant near the top of the sketch if the device runs elsewhere. If NTP sync fails or times out, the firmware falls back to a boot-relative timestamp instead (e.g. `T+16851ms`), so logging still works — check the Serial console on boot to see which mode is active. The web app's own CSV exports (Export CSV, and the PC-side auto-save) follow the same `timestamp` column convention, written as UTC ISO 8601 strings.

## Setup

1. Wire the INA219 to the ESP32's I2C pins (SDA → GPIO21, SCL → GPIO22 by default) and power/VBus lines to the circuit being measured.
2. Choose USB or Wi-Fi and open the corresponding sketch in the Arduino IDE.
3. For the Wi-Fi variant, copy `secrets_example.h` to `secrets.h` in the same folder and fill in your Wi-Fi credentials.
4. Install the required libraries listed above, select your ESP32 board, and flash the sketch.
5. Open `power_monitor.html` in a browser, choose the matching connection mode (USB or Wi-Fi), and connect.
