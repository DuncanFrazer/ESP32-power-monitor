/*
  ESP32 + INA219 Power Logger (Wi-Fi / on-device logging version)
  -----------------------------------------------------------------
  Samples voltage/current continuously. Detects device wake/sleep using
  a hysteresis current threshold and, independently of whether anything
  is connected, writes each completed capture to the ESP32's internal
  flash (LittleFS) as its own CSV file — so logging continues as long
  as the board is powered, regardless of PC/browser state.

  When a browser IS connected it also gets a live view: every sample is
  broadcast over a WebSocket in real time, same as before. A small HTTP
  server lets the browser list, download, and delete the files saved on
  the device.

  Internal flash is limited (see README) — a rolling-buffer strategy
  automatically deletes the oldest saved capture if free space runs low,
  so writes never fail outright. If you need to retain more history than
  that allows, an SD card is the better fit; ask if you'd like that
  variant instead.

  WebSocket protocol (port 81, path "/"), one text frame per sample/event:
    SAMPLE,<timestamp>,<voltage_mV>,<current_mA>,<power_mW>,<active 0|1>
    EVENT,WAKE,<timestamp>
    EVENT,SLEEP,<timestamp>,<duration_ms>
  <timestamp> is a local date/time string with UTC offset, e.g.
  "2026-09-16T14:32:05.123+01:00", once NTP sync succeeds. If sync fails,
  it falls back to a boot-relative form instead, e.g. "T+16851ms", so the
  device still logs something usable — see NTP_SYNC_TIMEOUT_MS below.

  A capture isn't finalized (file closed, EVENT,SLEEP sent) the instant
  current drops below SLEEP_THRESHOLD_MA — see SLEEP_GRACE_MS below. A
  brief dip (the monitored device rebooting, say) just pauses logging
  into the same still-open file; if current comes back up within the
  grace window, logging resumes with no new file and no new EVENT,WAKE.
  SAMPLE's <active> flag reflects whether that exact sample was written
  to the file, so it will read 0 during a sleep-pending gap even before
  the capture is actually finalized.

  HTTP API (port 80), all responses include CORS headers so
  power_monitor.html can call them when opened as a local file:
    GET    /files            -> JSON array of {"name":..,"size":..}
    GET    /download?file=X  -> returns the file's CSV content
    DELETE /delete?file=X    -> removes the file from flash
  /flush_log.csv (see DIAG_FLUSH_TIMING below) isn't a capture file, so it
  won't appear in /files, but it's still fetchable directly:
  /download?file=flush_log.csv

  Libraries (Library Manager):
    - "Adafruit INA219"
    - "WebSockets" by Markus Sattler (Links2004/arduinoWebSockets)
    - WiFi, ESPmDNS, WebServer, LittleFS, and NTP/time support are all
      bundled with the ESP32 board package — no separate install needed.

  Setup: copy secrets_example.h to secrets.h (same folder) and fill in
  your Wi-Fi SSID/password there before compiling.

  --- TUNE THESE FOR YOUR DEVICE ---
*/

#include <Wire.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebSocketsServer.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <time.h>
#include <sys/time.h>
#include <Adafruit_INA219.h>
#include "secrets.h"   // copy secrets_example.h -> secrets.h and fill in your credentials

Adafruit_INA219 ina219;

// ---- Wi-Fi ----
const char* WIFI_SSID     = SECRET_WIFI_SSID;
const char* WIFI_PASSWORD = SECRET_WIFI_PASSWORD;
const char* MDNS_HOSTNAME = "powerlogger"; // reachable at powerlogger.local
const uint16_t WS_PORT    = 81;
const uint16_t HTTP_PORT  = 80;

WebSocketsServer webSocket(WS_PORT);
WebServer httpServer(HTTP_PORT);

// ---- Time sync ----
// POSIX TZ string for Europe/London — handles the GMT/BST switch
// automatically. Change this if the device runs elsewhere.
const char* TIMEZONE_STRING = "GMT0BST,M3.5.0/1,M10.5.0";
const char* NTP_SERVER_1 = "pool.ntp.org";
const char* NTP_SERVER_2 = "time.google.com";
const uint32_t NTP_SYNC_TIMEOUT_MS = 15000; // give up and fall back to millis() after this long
bool timeSynced = false;

// ---- Detection thresholds (mA) ----
// Wake threshold: current must exceed this to be considered "active"
const float WAKE_THRESHOLD_MA  = 15.0;
// Sleep threshold: current must drop below this to be considered "idle"
// (kept lower than wake threshold = hysteresis, avoids chatter on noisy edges)
const float SLEEP_THRESHOLD_MA = 10.0;
// Number of consecutive samples that must agree before flipping state,
// to reject single-sample glitches. At SAMPLE_INTERVAL_MS below, this is
// the wake/sleep detection latency (currently 3 * 50ms = 150ms) — fine
// for gross behaviour, but worth remembering if SAMPLE_INTERVAL_MS changes.
const uint8_t DEBOUNCE_SAMPLES = 3;
// How long a confirmed drop below SLEEP_THRESHOLD_MA must persist before a
// capture is actually finalized (file closed, EVENT,SLEEP emitted). Below
// this, dropping below threshold just pauses logging — if current rises
// back above WAKE_THRESHOLD_MA before this window elapses, logging simply
// resumes into the SAME still-open file, no new file, no new EVENT,WAKE.
// This exists because a single capture can otherwise fragment into many
// separate files whenever the monitored device briefly reboots or dips to
// near-zero current without actually finishing its activity — observed
// directly: one test produced 21 separate files, with gaps between them
// of up to 33s. 60s gives headroom above that observed worst case.
// NOTE: this only helps when the ESP32 itself keeps running and sampling
// through the dip (i.e. the *monitored* device reboots/sleeps briefly). If
// the ESP32 running this firmware itself loses power or resets, all RAM
// state (including the open file handle) is lost regardless of this
// setting, and a new file will start on the next wake no matter what.
const uint32_t SLEEP_GRACE_MS = 60000;

// ---- Sampling ----
// 20 Hz — this project is for observing gross wake/sleep behaviour, not
// fine ms-level detail, so a coarser rate than the original 100 Hz is the
// right tradeoff: lower data volume means less frequent flash writes (see
// FLUSH_EVERY_N_SAMPLES below and the periodic "slow_write" stalls in
// DIAG_FLUSH_TIMING/flush_log.csv), and 20 Hz specifically also halves
// wake/sleep debounce latency versus 10 Hz (see DEBOUNCE_SAMPLES above).
const uint32_t SAMPLE_INTERVAL_MS = 50;

// ---- On-device storage ----
// Keep at least this many bytes free at all times; if a capture close
// would leave less than this, the oldest saved capture is deleted first.
const size_t MIN_FREE_BYTES = 20000;       // ~20KB headroom
// How often to flush the open capture file to flash while it's being
// written, expressed as a target wall-clock interval (not a raw sample
// count) so it stays correct if SAMPLE_INTERVAL_MS is ever tuned again.
// Flushing every sample is safest against power loss but wears flash
// faster and can stall the sample loop; this balances the two.
//
// Each flush briefly stalls sampling (measured 40-350ms+ in practice,
// apparently growing as more capture history accumulates on the device —
// see the DIAG_FLUSH_TIMING log below). That stall is largely a NOR flash
// erase, which on ESP32 disables interrupts system-wide for its duration
// -- a hardware constraint no amount of software buffering/task splitting
// can fully hide. The practical lever is simply flushing less often:
// fewer, larger writes means fewer erases overall, at the cost of more
// data at risk if power is cut mid-capture (up to roughly this many
// milliseconds of data).
//
// Note this is a separate mechanism from LittleFS's own internal write
// cache, which auto-commits to flash on its own schedule (observed as
// "slow_write" entries roughly every ~4KB of data) regardless of this
// setting — this constant only controls writeSampleToCapture()'s own
// explicit flush() calls, not LittleFS's internal ones.
const uint32_t FLUSH_INTERVAL_MS = 5000; // target: flush roughly every 5s of wall-clock time
const uint16_t FLUSH_EVERY_N_SAMPLES = FLUSH_INTERVAL_MS / SAMPLE_INTERVAL_MS; // Don't raise
                                             // FLUSH_INTERVAL_MS further until /flush_log.csv
                                             // shows headroom below the watchdog reset risk
                                             // noted below — see the Time sync / storage notes
                                             // in the README.

// Logs how long each flush (and startCaptureFile()'s ensureFreeSpace() +
// file-open) takes — both to Serial (for when it's being watched live) and
// to a small persistent file, /flush_log.csv, on the device itself (for
// when it isn't). The file survives power cycles and is downloadable the
// same way a capture is: /download?file=flush_log.csv — it isn't a
// capture file so it won't show up in the Device files list, but it's
// fetchable directly. Negligible overhead; safe to leave on.
//
// IMPORTANT ceiling to watch for in that log: ESP32's watchdog timer will
// reset the whole device if something blocks the CPU without yielding for
// too long (commonly ~5s by default, though this varies by board-package
// version) — a flash garbage-collection stall is a known way to
// occasionally spike into multi-second territory on a filling filesystem.
// If any duration in the log starts approaching a few seconds, that's the
// signal to stop raising FLUSH_EVERY_N_SAMPLES further, not a Serial
// message you'd otherwise never see.
const bool DIAG_FLUSH_TIMING = true;
const char* DIAG_LOG_PATH = "/flush_log.csv";
const size_t DIAG_LOG_MAX_BYTES = 50000; // cap so diagnostics can't eat meaningfully into capture-data space

void diagLog(const char* line) {
  if (!DIAG_FLUSH_TIMING) return;
  File check = LittleFS.open(DIAG_LOG_PATH, "r");
  bool tooBig = check && check.size() > DIAG_LOG_MAX_BYTES;
  if (check) check.close();
  File f = LittleFS.open(DIAG_LOG_PATH, tooBig ? "w" : "a"); // "w" truncates and starts fresh once the cap is hit
  if (f) {
    f.println(line);
    f.close();
  }
}

enum CaptureState { CAP_IDLE, CAP_ACTIVE, CAP_SLEEP_PENDING };
CaptureState captureState = CAP_IDLE;
uint8_t debounceCount = 0;
uint32_t wakeStartMs = 0;         // when the current capture file's session began
uint32_t sleepPendingStartMs = 0; // when current first dropped below threshold, this session
uint32_t lastSampleMs = 0;

File captureFile;
uint16_t samplesSinceFlush = 0;
char captureFilename[48];

void webSocketEvent(uint8_t clientId, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    webSocket.sendTXT(clientId, "READY,esp32_power_logger_wifi");
  }
}

// Current time as Unix epoch milliseconds if NTP sync succeeded, otherwise
// falls back to millis() (time-since-boot) so the device still logs
// something usable.
uint64_t epochMillis() {
  if (!timeSynced) return millis();
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (uint64_t)tv.tv_sec * 1000ULL + tv.tv_usec / 1000;
}

// Formats wallMs as a human-readable local timestamp with an explicit UTC
// offset, e.g. "2026-09-16T14:32:05.123+01:00" — unambiguous regardless of
// what timezone the browser reading it is in. If NTP never synced, falls
// back to a clearly-labelled boot-relative form, e.g. "T+16851ms".
void formatTimestamp(uint64_t wallMs, char* buf, size_t bufSize) {
  if (timeSynced) {
    time_t seconds = wallMs / 1000;
    int milliseconds = wallMs % 1000;
    struct tm timeinfo;
    localtime_r(&seconds, &timeinfo);

    // UK-specific shortcut for the offset: both tm_gmtoff and timegm()
    // turned out to be unavailable on this ESP32 toolchain, so rather than
    // keep chasing libc extensions, use tm_isdst instead — it's a standard
    // struct tm field guaranteed by C, and with TIMEZONE_STRING fixed to
    // Europe/London the only two possible offsets are GMT (+0) and BST
    // (+1h), which tm_isdst tells us directly. If you change
    // TIMEZONE_STRING to a region with a different/non-whole-hour DST
    // offset, this line needs updating to match.
    long offsetSec = (timeinfo.tm_isdst > 0) ? 3600 : 0;

    snprintf(buf, bufSize, "%04d-%02d-%02dT%02d:%02d:%02d.%03d+%02ld:%02ld",
             timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, milliseconds,
             offsetSec / 3600, (offsetSec % 3600) / 60);
  } else {
    snprintf(buf, bufSize, "T+%llums", (unsigned long long)wallMs);
  }
}

// ---- Storage helpers ----

void addCorsHeaders() {
  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.sendHeader("Access-Control-Allow-Methods", "GET, DELETE, OPTIONS");
}

// Deletes the oldest /cap_*.csv file. Returns true if a file was deleted.
bool deleteOldestCapture() {
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  String oldestName;
  time_t oldestTime = 0;
  bool found = false;
  while (file) {
    String name = String(file.name());
    if (name.startsWith("cap_") && name.endsWith(".csv")) {
      if (!found || file.getLastWrite() < oldestTime) {
        oldestTime = file.getLastWrite();
        oldestName = name;
        found = true;
      }
    }
    file = root.openNextFile();
  }
  if (found) {
    String path = "/" + oldestName;
    LittleFS.remove(path);
    Serial.print("Freed space by deleting oldest capture: ");
    Serial.println(path);
    return true;
  }
  return false;
}

// Ensures at least MIN_FREE_BYTES is free, deleting oldest captures as needed.
// Returns how many files were deleted (0 is the common case and is cheap —
// the expensive directory scan inside deleteOldestCapture() only runs when
// this actually has work to do).
int ensureFreeSpace() {
  size_t total = LittleFS.totalBytes();
  size_t used  = LittleFS.usedBytes();
  size_t free  = total - used;
  int guard = 0; // avoid infinite loop if something's wrong
  int deleted = 0;
  while (free < MIN_FREE_BYTES && guard < 50) {
    if (!deleteOldestCapture()) break; // nothing left to delete
    used = LittleFS.usedBytes();
    free = total - used;
    guard++;
    deleted++;
  }
  return deleted;
}

void startCaptureFile(uint64_t wallMs) {
  uint32_t t0 = DIAG_FLUSH_TIMING ? millis() : 0;
  int filesDeleted = ensureFreeSpace();
  uint32_t t1 = DIAG_FLUSH_TIMING ? millis() : 0;

  if (timeSynced) {
    // Human-readable local-time filename, e.g. /cap_20260916_143205_123.csv
    time_t seconds = wallMs / 1000;
    int milliseconds = wallMs % 1000;
    struct tm timeinfo;
    localtime_r(&seconds, &timeinfo);
    snprintf(captureFilename, sizeof(captureFilename), "/cap_%04d%02d%02d_%02d%02d%02d_%03d.csv",
             timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, milliseconds);
  } else {
    // Fallback: no wall-clock available, name by boot-relative milliseconds as before
    snprintf(captureFilename, sizeof(captureFilename), "/cap_%llu.csv", (unsigned long long)wallMs);
  }
  captureFile = LittleFS.open(captureFilename, "w");
  uint32_t t2 = DIAG_FLUSH_TIMING ? millis() : 0;
  uint32_t t3 = t2;
  if (captureFile) {
    captureFile.println("timestamp,voltage_mV,current_mA,power_mW,active");
    t3 = DIAG_FLUSH_TIMING ? millis() : 0; // a brand-new file's first write can itself trigger a block
                                            // allocation/erase that open() alone doesn't show — see below
    samplesSinceFlush = 0;
  } else {
    Serial.print("ERROR: failed to open capture file ");
    Serial.println(captureFilename);
  }
  if (DIAG_FLUSH_TIMING) {
    char tsBuf[32];
    formatTimestamp(wallMs, tsBuf, sizeof(tsBuf));
    char diagLine[180];
    snprintf(diagLine, sizeof(diagLine),
             "%s,open,ensureFreeSpace_ms=%lu,deleted=%d,open_ms=%lu,header_write_ms=%lu,total_ms=%lu",
             tsBuf, (unsigned long)(t1 - t0), filesDeleted, (unsigned long)(t2 - t1),
             (unsigned long)(t3 - t2), (unsigned long)(t3 - t0));
    Serial.println(diagLine);
    diagLog(diagLine);
  }
}

// Samples write quickly almost always; this only logs the rare cases where
// a plain write (not the periodic flush below) takes noticeably longer
// than one sample interval, which previously went unaccounted for — e.g.
// a brand-new file's very first write can trigger its own block
// allocation/erase that LittleFS.open() alone doesn't incur.
const uint32_t SLOW_WRITE_THRESHOLD_MS = 15;

void writeSampleToCapture(const char* timestamp, float voltage_mV, float current_mA, float power_mW) {
  if (!captureFile) return;
  uint32_t tw0 = DIAG_FLUSH_TIMING ? millis() : 0;
  captureFile.printf("%s,%.1f,%.3f,%.3f,1\n", timestamp, voltage_mV, current_mA, power_mW);
  if (DIAG_FLUSH_TIMING) {
    uint32_t writeMs = millis() - tw0;
    if (writeMs > SLOW_WRITE_THRESHOLD_MS) {
      char diagLine[160];
      snprintf(diagLine, sizeof(diagLine), "%s,slow_write,duration_ms=%lu", timestamp, (unsigned long)writeMs);
      Serial.println(diagLine);
      diagLog(diagLine);
    }
  }
  samplesSinceFlush++;
  if (samplesSinceFlush >= FLUSH_EVERY_N_SAMPLES) {
    uint32_t t0 = DIAG_FLUSH_TIMING ? millis() : 0;
    captureFile.flush();
    if (DIAG_FLUSH_TIMING) {
      uint32_t durationMs = millis() - t0;
      char diagLine[160];
      snprintf(diagLine, sizeof(diagLine), "%s,flush,duration_ms=%lu", timestamp, (unsigned long)durationMs);
      Serial.println(diagLine);
      diagLog(diagLine);
    }
    samplesSinceFlush = 0;
  }
}

void closeCaptureFile() {
  if (captureFile) {
    captureFile.flush();
    captureFile.close();
    Serial.print("Capture saved: ");
    Serial.println(captureFilename);
  }
  ensureFreeSpace();
}

// ---- HTTP handlers ----

void handleListFiles() {
  addCorsHeaders();
  String json = "[";
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  bool first = true;
  while (file) {
    String name = String(file.name());
    if (name.startsWith("cap_") && name.endsWith(".csv")) {
      if (!first) json += ",";
      json += "{\"name\":\"" + name + "\",\"size\":" + String(file.size()) + "}";
      first = false;
    }
    file = root.openNextFile();
  }
  json += "]";
  httpServer.send(200, "application/json", json);
}

void handleDownload() {
  addCorsHeaders();
  if (!httpServer.hasArg("file")) { httpServer.send(400, "text/plain", "missing file param"); return; }
  String path = "/" + httpServer.arg("file");
  if (!LittleFS.exists(path)) { httpServer.send(404, "text/plain", "not found"); return; }
  File f = LittleFS.open(path, "r");
  httpServer.sendHeader("Content-Disposition", "attachment; filename=\"" + httpServer.arg("file") + "\"");
  httpServer.streamFile(f, "text/csv");
  f.close();
}

void handleDelete() {
  addCorsHeaders();
  if (!httpServer.hasArg("file")) { httpServer.send(400, "text/plain", "missing file param"); return; }
  String path = "/" + httpServer.arg("file");
  if (!LittleFS.exists(path)) { httpServer.send(404, "text/plain", "not found"); return; }
  LittleFS.remove(path);
  httpServer.send(200, "text/plain", "deleted");
}

void handleOptions() {
  addCorsHeaders();
  httpServer.send(204);
}

void setup() {
  Serial.begin(115200); // kept for local debugging only; not required by the app

  if (!ina219.begin()) {
    Serial.println("ERROR: INA219 not found - check wiring");
    while (1) { delay(1000); }
  }

  // 16V / 400mA range gives better resolution at low currents than the
  // default 32V/2A range. Adjust if your device's active-state current
  // exceeds ~400mA.
  ina219.setCalibration_16V_400mA();

  if (!LittleFS.begin(true)) { // true = format on mount failure
    Serial.println("ERROR: LittleFS mount failed");
  } else {
    Serial.printf("LittleFS ready: %u / %u bytes used\n", LittleFS.usedBytes(), LittleFS.totalBytes());
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected. Device IP: ");
  Serial.println(WiFi.localIP());

  // Disable Wi-Fi modem-sleep power saving. This is a trial fix for an
  // unexplained ~1-1.3s stall seen on the first broadcast of a capture
  // after a long idle gap (overnight/multi-hour) — a plausible cause is
  // the radio needing to wake from a power-save state before it can send.
  // Costs a bit more idle power draw in exchange; worth it to find out
  // whether it fixes the stall before looking elsewhere.
  WiFi.setSleep(false);

  // ---- NTP time sync ----
  configTzTime(TIMEZONE_STRING, NTP_SERVER_1, NTP_SERVER_2);
  Serial.print("Waiting for NTP time sync");
  uint32_t syncStart = millis();
  time_t nowSecs = time(nullptr);
  // A freshly-booted, unsynced clock reads as a small number of seconds
  // since 1970; wait until it looks like a real recent date instead.
  while (nowSecs < 8 * 3600 * 2 && millis() - syncStart < NTP_SYNC_TIMEOUT_MS) {
    delay(250);
    Serial.print(".");
    nowSecs = time(nullptr);
  }
  Serial.println();
  if (nowSecs >= 8 * 3600 * 2) {
    timeSynced = true;
    struct tm timeinfo;
    localtime_r(&nowSecs, &timeinfo);
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S %Z", &timeinfo);
    Serial.print("Time synced: ");
    Serial.println(buf);
  } else {
    Serial.println("WARNING: NTP sync failed — capture filenames and timestamps will fall back to milliseconds-since-boot.");
  }

  if (MDNS.begin(MDNS_HOSTNAME)) {
    Serial.print("mDNS responder started — reachable at ");
    Serial.print(MDNS_HOSTNAME);
    Serial.println(".local");
    MDNS.addService("ws", "tcp", WS_PORT);
    MDNS.addService("http", "tcp", HTTP_PORT);
  } else {
    Serial.println("mDNS setup failed — fall back to the IP address above.");
  }

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  httpServer.on("/files", HTTP_GET, handleListFiles);
  httpServer.on("/download", HTTP_GET, handleDownload);
  httpServer.on("/delete", HTTP_DELETE, handleDelete);
  httpServer.on("/files", HTTP_OPTIONS, handleOptions);
  httpServer.on("/download", HTTP_OPTIONS, handleOptions);
  httpServer.on("/delete", HTTP_OPTIONS, handleOptions);
  httpServer.begin();
}

void loop() {
  webSocket.loop();
  httpServer.handleClient();

  uint32_t now = millis(); // monotonic — used only for sample-interval gating and wake/sleep durations
  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  lastSampleMs = now;

  uint64_t wallMs = epochMillis(); // wall-clock (or millis() fallback) — used for anything logged or displayed
  char tsBuf[32];
  formatTimestamp(wallMs, tsBuf, sizeof(tsBuf));

  float busVoltage_V  = ina219.getBusVoltage_V();
  float current_mA    = ina219.getCurrent_mA();
  float power_mW      = busVoltage_V * current_mA; // more stable than getPower_mW() derived value at low currents
  float voltage_mV    = busVoltage_V * 1000.0;

  // ---- Hysteresis + debounce wake/sleep detection, with a long-tail grace
  // period before a capture is actually finalized (see SLEEP_GRACE_MS) ----
  bool instantAboveWake  = current_mA > WAKE_THRESHOLD_MA;
  bool instantBelowSleep = current_mA < SLEEP_THRESHOLD_MA;

  switch (captureState) {
    case CAP_IDLE:
      if (instantAboveWake) {
        debounceCount++;
        if (debounceCount >= DEBOUNCE_SAMPLES) {
          captureState = CAP_ACTIVE;
          wakeStartMs = now;
          debounceCount = 0;
          startCaptureFile(wallMs);
          char buf[48];
          snprintf(buf, sizeof(buf), "EVENT,WAKE,%s", tsBuf);
          webSocket.broadcastTXT(buf);
        }
      } else {
        debounceCount = 0;
      }
      break;

    case CAP_ACTIVE:
      if (instantBelowSleep) {
        debounceCount++;
        if (debounceCount >= DEBOUNCE_SAMPLES) {
          captureState = CAP_SLEEP_PENDING;
          sleepPendingStartMs = now;
          debounceCount = 0;
          // File stays open, nothing finalized yet — just pause logging
          // and start the grace-period clock.
        }
      } else {
        debounceCount = 0;
      }
      break;

    case CAP_SLEEP_PENDING:
      if (instantAboveWake) {
        debounceCount++;
        if (debounceCount >= DEBOUNCE_SAMPLES) {
          // Resumed within the grace window — same file, no new EVENT,WAKE.
          captureState = CAP_ACTIVE;
          debounceCount = 0;
        }
      } else {
        debounceCount = 0; // a brief flicker toward wake shouldn't eat into the debounce needed to actually resume
        if (now - sleepPendingStartMs >= SLEEP_GRACE_MS) {
          captureState = CAP_IDLE;
          // Duration reflects the actual logged active span, not the grace
          // tail spent waiting to see if it would resume.
          uint32_t duration = sleepPendingStartMs - wakeStartMs;
          closeCaptureFile();
          char buf[64];
          snprintf(buf, sizeof(buf), "EVENT,SLEEP,%s,%lu", tsBuf, (unsigned long)duration);
          webSocket.broadcastTXT(buf);
        }
      }
      break;
  }

  // ---- Save to flash only while actively logging (not during a sleep-pending gap) ----
  bool loggingNow = (captureState == CAP_ACTIVE);
  if (loggingNow) {
    writeSampleToCapture(tsBuf, voltage_mV, current_mA, power_mW);
  }

  // ---- Always stream the sample over Wi-Fi too, regardless of logging state ----
  char line[96];
  snprintf(line, sizeof(line), "SAMPLE,%s,%.1f,%.3f,%.3f,%d",
           tsBuf, voltage_mV, current_mA, power_mW, loggingNow ? 1 : 0);
  webSocket.broadcastTXT(line);
}
