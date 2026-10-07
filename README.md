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
- **Point inspection** — hover any point on a trace for its exact voltage/current/power values
- **Export** — save the currently viewed trace as a PNG image or CSV, automatically named after the capture being viewed (or a timestamp, for a live-session export) so exports don't collide or overwrite each other

## Repository structure

```
├── README.md
├── power_monitor.html          # Browser-based viewer/logger app
├── .gitignore
├── USB/
│   └── esp32_power_logger_usb/
│       └── esp32_power_logger_usb.ino
└── Wi-Fi/
    └── esp_power_logger_wifi/
        ├── esp32_power_logger_wifi.ino
        ├── secrets_example.h    # Template for Wi-Fi credentials
        └── secrets.h            # Your credentials (git-ignored)
```

Flash whichever sketch matches how you want to connect, then open
`power_monitor.html` in a browser and select the matching connection mode.

## Requirements

### Hardware
- ESP32 development board
- INA219 current/voltage sensor, wired to the ESP32's default I2C pins: SDA → GPIO21, SCL → GPIO22 (both sketches call `Wire.begin()` with no arguments, which uses these defaults — if you wire to different GPIOs, update to `Wire.begin(SDA_PIN, SCL_PIN)` in the sketch)

### Arduino IDE / board support
- [ESP32 board package](https://github.com/espressif/arduino-esp32) ("esp32 by Espressif Systems"), installed via Boards Manager

### Libraries

Install via the Arduino Library Manager (Sketch → Include Library → Manage Libraries).

**Both variants:**
- `Adafruit INA219` (pulls in `Adafruit BusIO` as a dependency — accept the dependency install prompt if shown)

**Wi-Fi variant only** (`esp32_power_logger_wifi.ino`):
- `WebSockets` by Markus Sattler (repo: Links2004/arduinoWebSockets) — search "WebSockets" and match on the author, as there are similarly named alternatives
- `WiFi` and `ESPmDNS` are bundled with the ESP32 board package and don't need separate installation

**USB variant** (`esp32_power_logger_usb.ino`):
- No additional libraries beyond `Adafruit INA219` — serial communication uses the built-in `Serial` interface

### Browser
- Chrome or Edge recommended. USB mode requires the Web Serial API; Wi-Fi mode's optional auto-save-to-folder feature requires the File System Access API. Both are Chromium-only at present; other browsers can still use Wi-Fi mode for live viewing, with captures falling back to individual downloads instead of folder auto-save.

### On-device storage (Wi-Fi variant)

Captures are saved to the ESP32's internal flash filesystem (LittleFS), not an SD card. This is convenient (no extra hardware) but limited in capacity — a typical ESP32's default partition scheme leaves roughly 1.3–1.5MB free after the firmware itself. The default sample rate (`SAMPLE_INTERVAL_MS`, 20Hz) is tuned for observing gross wake/sleep behaviour rather than fine ms-level detail, both for this reason and because a lower rate directly reduces flash-write frequency (see the jitter note below) — raise it if you need finer resolution and are prepared to trade off storage headroom and jitter for it. When free space runs low, the firmware automatically deletes the oldest saved capture to make room, so writes keep succeeding rather than failing outright — download captures regularly via the device file browser if you want to keep more history than that.

**Capture finalization has a long-tail grace period.** Current dropping below `SLEEP_THRESHOLD_MA` doesn't immediately close the file — it pauses logging and starts a `SLEEP_GRACE_MS` clock (60s by default). If current rises back above `WAKE_THRESHOLD_MA` before that elapses, logging just resumes into the same still-open file; no new file, no new wake event. Only if the low-current state persists for the full grace window does the capture actually get finalized. This exists because a single real session can otherwise fragment into many separate files whenever the monitored device briefly reboots or dips to near-zero current without actually finishing its activity — one test produced 21 separate files from what should have been far fewer, with gaps between them of up to 33s; 60s gives headroom above that. This only helps when the ESP32 itself keeps running and sampling through the dip — if the ESP32 running this firmware loses power or resets, all RAM state (including the open file handle) is lost regardless of this setting, and a new file starts on the next wake no matter what.

**Sampling jitter during a capture.** The open capture file is flushed to flash periodically (`FLUSH_INTERVAL_MS`, every ~5s of wall-clock time by default, converted to a sample count via `SAMPLE_INTERVAL_MS` so it stays correct if the sample rate changes) rather than every sample. Each flush briefly stalls sampling — this is mostly a NOR flash erase, which on ESP32 disables interrupts system-wide for its duration, a hardware constraint rather than something software buffering can fully hide. Raising `FLUSH_INTERVAL_MS` trades more at-risk data on a power cut for fewer, less frequent stalls — don't raise it further than the current default without evidence it's safe to (see below).

There's a second, separate source of jitter: LittleFS's own internal write cache auto-commits to flash on its own schedule — observed as a stall roughly every ~4KB of written data — regardless of `FLUSH_INTERVAL_MS`, since that constant only controls this firmware's own explicit `flush()` calls, not LittleFS's internal ones. The main lever against this one is total data volume, which is why `SAMPLE_INTERVAL_MS` defaults to a conservative 10Hz rather than something faster.

`DIAG_FLUSH_TIMING` (on by default) logs timing to both Serial and a small persistent file on the device, `/flush_log.csv` (fetchable any time via `/download?file=flush_log.csv`, even though it won't appear in the Device files list since it isn't a capture). The persistent copy matters because you won't usually have Serial open when a capture happens unattended. Three row types appear:
- `open` — one per capture, breaking down `startCaptureFile()`'s `ensureFreeSpace()` time, the `LittleFS.open()` call itself, and the header row's write time separately (a brand-new file's first write can trigger its own block allocation/erase beyond what `open()` alone shows).
- `flush` — one per periodic flush (`FLUSH_INTERVAL_MS`).
- `slow_write` — logged only when an ordinary per-sample write unexpectedly takes longer than `SLOW_WRITE_THRESHOLD_MS` (15ms by default); this should be rare and mainly catches stalls that don't fall on a flush boundary.

Watch all three for durations creeping up toward a few seconds — ESP32's watchdog timer will reset the whole device (losing the in-progress capture) if something blocks the CPU without yielding for too long, commonly around 5s by default, and flash garbage-collection on a filling filesystem is a known way to occasionally spike that high. That's the real ceiling on how far `FLUSH_INTERVAL_MS` can safely go, not a fixed number.

**Startup stall after a long idle period.** A separate, still only partially-understood ~1-1.3s stall has been observed specifically on the first sample after the device has been idle for hours (e.g. overnight). As a trial fix, `WiFi.setSleep(false)` disables Wi-Fi modem-sleep power saving in `setup()`, on the theory that the radio waking from a power-save state might be the cause — at the cost of a bit more idle power draw. Still being verified against real captures.

### Time sync (Wi-Fi variant)

The Wi-Fi sketch syncs the current time over NTP once connected, and uses it for both capture filenames (e.g. `cap_20260916_143205_123.csv`, in local time) and the `timestamp` column inside each CSV (e.g. `2026-09-16T14:32:05.123+01:00` — local time with an explicit UTC offset, so it's unambiguous regardless of what timezone anything reading it is in). This requires outbound UDP access to NTP servers (`pool.ntp.org` / `time.google.com` by default) on port 123, which most networks allow — check your network settings if sync fails. The timezone defaults to Europe/London (with automatic GMT/BST handling); change the `TIMEZONE_STRING` constant near the top of the sketch if the device runs elsewhere. If NTP sync fails or times out, the firmware falls back to a boot-relative timestamp instead (e.g. `T+16851ms`), so logging still works — check the Serial console on boot to see which mode is active. The web app's own CSV exports (Export CSV, and the PC-side auto-save) follow the same `timestamp` column convention, written as UTC ISO 8601 strings.

## Setup

1. Wire the INA219 to the ESP32's I2C pins (SDA → GPIO21, SCL → GPIO22 by default) and power/VBus lines to the circuit being measured.
2. Choose USB or Wi-Fi and open the corresponding sketch in the Arduino IDE.
3. For the Wi-Fi variant, copy `secrets_example.h` to `secrets.h` in the same folder and fill in your Wi-Fi credentials.
4. Install the required libraries listed above, select your ESP32 board, and flash the sketch.
5. Open `power_monitor.html` in a browser, choose the matching connection mode (USB or Wi-Fi), and connect.
