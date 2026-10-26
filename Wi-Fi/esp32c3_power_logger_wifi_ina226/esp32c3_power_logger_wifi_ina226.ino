/*
  ESP32-C3 Supermini + INA226 Power Logger (Wi-Fi / on-device logging version)
  ------------------------------------------------------------------------------
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
  SAMPLE's <active> flag is the true active state (above-threshold, not
  paused in a sleep-pending gap), so it reads 0 during a sleep-pending gap
  even before the capture is actually finalized.

  Saved capture files also include PREROLL_MS of context before the wake
  and POSTROLL_MS after activity drops off, written with active=0, so the
  ramp up and down are visible either side of the true active span.

  HTTP API (port 80), all responses include CORS headers so
  power_monitor.html can call them when opened as a local file:
    GET    /files            -> JSON array of {"name":..,"size":..}
    GET    /download?file=X  -> returns the file's CSV content
    DELETE /delete?file=X    -> removes the file from flash
  /flush_log.csv and /flush_log_prev.csv (the log rotates when it grows
  past DIAG_LOG_MAX_BYTES) aren't capture files, so they won't appear in
  /files, but they're still fetchable directly:
  /download?file=flush_log.csv

  --- HARDWARE (this variant) ---
  Board:  ESP32-C3 Supermini (Arduino: "ESP32C3 Dev Module")
  Sensor: INA226 breakout module, I2C address 0x40 (A0/A1 tied to GND)

    INA226 VCC  -> Supermini 3V3
    INA226 GND  -> Supermini GND   (also common with the monitored circuit's ground)
    INA226 SDA  -> Supermini GPIO4
    INA226 SCL  -> Supermini GPIO3
    INA226 ALERT-> not connected
    VIN+ / VIN- -> in series with the monitored device's positive supply,
                   exactly as with the previous INA219 setup (VIN+ toward
                   the battery/source, VIN- toward the load). If current
                   reads negative, swap VIN+ and VIN-.

  GPIO3/4 were chosen to avoid the Supermini's strapping pins (GPIO2, 8, 9),
  the on-board LED (GPIO8), the BOOT button (GPIO9), and the UART pins
  (GPIO20/21), and they sit next to the 3V3/GND header pins for easy wiring.
  Clone boards vary — check the silkscreen on yours. To use other pins,
  change I2C_SDA_PIN / I2C_SCL_PIN below.

  Arduino IDE board settings: Board "ESP32C3 Dev Module", "USB CDC On Boot:
  Enabled" (needed for Serial over the USB-C port), Flash Size 4MB, and a
  partition scheme with a SPIFFS/LittleFS partition (the default 4MB
  scheme has one).

  Libraries (Library Manager):
    - "INA226_WE" by Wolfgang Ewald
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
#include <esp_system.h>
#include <sys/time.h>
#include <INA226_WE.h>
#include "secrets.h"   // copy secrets_example.h -> secrets.h and fill in your credentials

// ---- Board / sensor wiring (ESP32-C3 Supermini + INA226) ----
const int      I2C_SDA_PIN  = 4;
const int      I2C_SCL_PIN  = 3;
const uint32_t I2C_CLOCK_HZ = 400000;
const uint8_t  INA226_I2C_ADDR = 0x40; // A0/A1 both to GND

// The C3's USB-CDC serial port disappears and re-enumerates on every reset,
// so anything printed in the first second or two never reaches a serial
// monitor that was already open. Wait up to this long at boot for one to
// attach — bounded, so a headless boot isn't held up.
const uint32_t SERIAL_WAIT_MS = 3000;

// Prints the INA226's raw readings to Serial about once a second, so a
// wiring/shunt/calibration problem can be told apart at a glance:
//   shunt ~0 mV            -> shunt isn't actually in series with the load
//   shunt negative         -> VIN+ / VIN- are the wrong way round
//   shunt non-zero, current small -> SHUNT_OHMS doesn't match the module
//   shunt non-zero, current exactly 0 -> calibration register not applied
// Cheap (one extra register read per second); turn off once it's working.
const bool SENSOR_DEBUG_SERIAL = true;
uint32_t lastSensorDebugMs = 0;
const bool CLIP_NEGATIVE_CURRENT = true;

// Shunt resistor on your INA226 module. Yours is marked R010 = 0.010 ohm
// (10 milliohm). Other common values: R100 = 0.1 ohm, R002 = 0.002 ohm —
// if you ever swap modules, change SHUNT_OHMS to match, or every current
// reading will be scaled wrongly by the ratio between the two (a 0.1 ohm
// assumption on a 0.01 ohm shunt reads 10x too LOW).
//
// With 0.01 ohm the shunt input (+/-81.92 mV full scale) reaches ~8 A and
// each count of the shunt voltage register (2.5 uV) is ~0.25 mA, which is
// ample for this project's tens-to-hundreds of mA.
//
// MAX_EXPECTED_A sets the current register's scale (it saturates at this
// value) and must be at least ~0.26 A for a 0.01 ohm shunt, or the
// calibration value overflows its 16-bit register. 1.0 A gives ~3x headroom
// over the largest peak seen so far (~306 mA).
const float SHUNT_OHMS     = 0.01;
const float MAX_EXPECTED_A = 1.0;

// Hardware averaging done inside the INA226 itself, which smooths noise
// at low currents for free. Total time per reading = 2 x conversion time
// x averages (2 x 1.1ms x 16 = ~35ms here) — keep that BELOW
// SAMPLE_INTERVAL_MS (50ms) so every loop read sees a freshly completed
// result. Options: INA226_AVERAGE_1/4/16/64/128/256/512/1024 and
// INA226_CONV_TIME_140/204/332/588/1100/2116/4156/8244 (microseconds).
// (These INA226_-prefixed names match recent INA226_WE versions; older
// versions use the unprefixed AVERAGE_16 / CONV_TIME_1100 / CONTINUOUS.)
#define INA226_AVG_SETTING   INA226_AVERAGE_16
#define INA226_CONV_SETTING  INA226_CONV_TIME_1100

INA226_WE ina226 = INA226_WE(INA226_I2C_ADDR);

// Some ESP32-C3 Supermini clones have antenna/regulator quirks that make
// Wi-Fi connection flaky at full TX power. If yours won't connect reliably
// or drops out, set this true to cap TX power.
const bool WIFI_REDUCE_TX_POWER = true;

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
const float WAKE_THRESHOLD_MA  = 2.0;
// Sleep threshold: current must drop below this to be considered "idle"
// (kept lower than wake threshold = hysteresis, avoids chatter on noisy edges)
const float SLEEP_THRESHOLD_MA = 1.0;
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
const uint32_t SLEEP_GRACE_MS = 30000;

// ---- Sampling ----
// 20 Hz — this project is for observing gross wake/sleep behaviour, not
// fine ms-level detail, so a coarser rate than the original 100 Hz is the
// right tradeoff: lower data volume means less frequent flash writes (see
// FLUSH_EVERY_N_SAMPLES below and the periodic "slow_write" stalls in
// DIAG_FLUSH_TIMING/flush_log.csv), and 20 Hz specifically also halves
// wake/sleep debounce latency versus 10 Hz (see DEBOUNCE_SAMPLES above).
const uint32_t SAMPLE_INTERVAL_MS = 50;

// ---- Pre-roll / post-roll context around each capture ----
// Each saved capture also includes a little context either side of the
// actual above-threshold activity, to help visualise the full capture
// (the ramp up from idle, and the ramp back down) rather than a trace
// that starts and stops abruptly at the threshold crossings.
//   - Pre-roll: the last PREROLL_MS of samples from *before* wake was
//     confirmed, kept in a small always-running ring buffer.
//   - Post-roll: the first POSTROLL_MS of samples after current drops
//     below threshold, written before logging pauses into the sleep-
//     pending gap (see SLEEP_GRACE_MS).
// Context rows are written with active=0 (they weren't above threshold),
// so the viewer's shaded "active" region still marks only the true active
// span, with this context visible just outside it. Both are expressed in
// wall-clock time and converted to sample counts via SAMPLE_INTERVAL_MS,
// so they stay correct if the sample rate is tuned again.
const uint32_t PREROLL_MS  = 2000;
const uint32_t POSTROLL_MS = 2000;
const uint16_t PREROLL_SAMPLES = PREROLL_MS / SAMPLE_INTERVAL_MS;

struct RecentSample {
  uint64_t wallMs;
  float voltage_mV;
  float current_mA;
  float power_mW;
};
RecentSample prerollBuf[PREROLL_SAMPLES];
uint16_t prerollWriteIdx = 0;
uint16_t prerollCount = 0; // valid entries so far (less than PREROLL_SAMPLES until the buffer first fills)

// ---- On-device storage ----
// Rolling retention. The goal: when a capture finishes, clean out old data
// right then, while the device is idle, so there is room for the whole NEXT
// capture — and nothing needs to be deleted (or flushed) while that next
// capture is being recorded. Space reserved for the next capture is
// CAPTURE_BUDGET_FACTOR x the largest of the last RECENT_CAPTURES_TRACKED
// captures (so it adapts if captures get longer).
//
// On top of that, a FLOOR of free space (a percentage of the flash) is kept
// for speed: LittleFS finds free blocks by traversing the whole filesystem
// each time its allocator's look-ahead window runs dry, which happens far
// more often when the flash is nearly full. Observed directly: keeping only
// ~20KB free, every flush/write/open got steadily slower as the flash filled
// (flush mean 8 -> 40 ms, capture-open 40 -> 190 ms over ~100 captures),
// costing samples at 20 Hz. The cleanup target is whichever of the two is
// larger. Set FREE_FLOOR_PERCENT to 0 for a purely next-capture-sized
// reserve (more retained history, slower writes as it fills).
const uint8_t  FREE_FLOOR_PERCENT      = 40;
const uint8_t  CAPTURE_BUDGET_FACTOR   = 2;
const uint8_t  RECENT_CAPTURES_TRACKED = 8;
const size_t   MIN_CAPTURE_BUDGET_BYTES = 64000; // reserve at least this much, however small recent captures were
const size_t   EMERGENCY_FREE_BYTES    = 64000;  // at capture START, only clean up if free space is below this
const size_t   EXTREME_LOW_FREE_BYTES  = 32000;  // MID-capture: purge only if free space (net of this capture) falls below this
const uint8_t  MAX_DELETES_PER_CALL    = 4;      // bounds any one stall; remaining cleanup carries on after later captures
const uint8_t  FS_RESYNC_EVERY_N_CLOSES = 20;    // how often to re-read the exact (slow) used-bytes figure
const size_t   FS_BLOCK_BYTES          = 4096;   // LittleFS block size on ESP32
// FLUSHING: deliberately almost never. Each flush() stalls sampling (a flash
// erase+program; 40-350 ms+ observed, and growing as the flash fills), so
// captures are written straight through and only committed when they close.
// The trade-off to be aware of: until a capture closes (or a flush happens),
// its data is only in RAM and in not-yet-committed flash blocks, so if power
// is lost or the chip resets mid-capture, that in-progress capture is
// likely lost (LittleFS publishes a file's contents on sync/close). The
// capture's start and the unfinished tail are simply gone; previously
// closed captures are unaffected.
//
// The one exception — "something extreme": a single capture that stays open
// for SAFETY_FLUSH_MS (a very long session) gets a flush, so the data at
// risk is bounded to about that long rather than the whole thing. Expressed
// in wall-clock time and converted to a sample count via SAMPLE_INTERVAL_MS
// so it stays correct if the sample rate is tuned. (The other "extreme" case
// is running short of space mid-capture; see EXTREME_LOW_FREE_BYTES.)
//
// Separately from all this, LittleFS auto-commits its own internal write
// cache every ~4KB (the periodic "slow_write" entries) — that isn't
// controllable from here.
const uint32_t SAFETY_FLUSH_MS = 120000;
const uint16_t FLUSH_EVERY_N_SAMPLES = SAFETY_FLUSH_MS / SAMPLE_INTERVAL_MS;

// Diagnostic log: capture open/close timing (including what the cleanup
// deleted and how much space is estimated free), flushes, unusually slow
// writes, mid-capture "extreme" purges, and one row per boot including the
// chip's reset reason. Rows are held in RAM and written to /flush_log.csv
// only when the device is idle (end of a capture), never mid-capture. The
// file survives power cycles and rotates to /flush_log_prev.csv when it
// grows large. Both are downloadable like a capture —
// /download?file=flush_log.csv — though they don't appear in the Device
// files list since they aren't captures.
//
// Watch for any duration approaching a few seconds: ESP32's watchdog timer
// will reset the whole device if something blocks the CPU without yielding
// for too long (commonly ~5s by default, varying by board-package version),
// and a flash garbage-collection stall is a known way to occasionally spike
// that high on a filling filesystem.
const bool DIAG_FLUSH_TIMING = true;
const char* DIAG_LOG_PATH = "/flush_log.csv";
const char* DIAG_LOG_PREV_PATH = "/flush_log_prev.csv";
const size_t DIAG_LOG_MAX_BYTES = 40000; // once exceeded, the log rotates to flush_log_prev.csv (so ~2x this is kept)
const size_t DIAG_BUF_BYTES = 3072;

// ---- Capture index & cleanup (begin) ----
// Diagnostic rows are collected in RAM and only written to flash once the
// device is idle (end of a capture) — never mid-capture. Writing each one
// immediately used to be the single biggest source of sampling gaps: every
// append is itself a flash erase+program plus filesystem metadata work that
// grows with the number of stored files (150-440 ms once ~100 captures were
// stored), and it landed in the middle of the very capture being measured,
// on top of whatever stall was being logged.
char diagBuf[DIAG_BUF_BYTES];
size_t diagBufLen = 0;
uint16_t diagDropped = 0;

void diagLog(const char* line) {
  if (!DIAG_FLUSH_TIMING) return;
  size_t n = strlen(line);
  if (diagBufLen + n + 1 > DIAG_BUF_BYTES) { diagDropped++; return; } // full: drop, and say so when flushed
  memcpy(diagBuf + diagBufLen, line, n);
  diagBufLen += n;
  diagBuf[diagBufLen++] = '\n';
}

// Writes the buffered rows to flash (rotating the log if it has grown past
// its cap). Only call this when sampling stalls don't matter.
void diagFlushToFlash() {
  if (!DIAG_FLUSH_TIMING || diagBufLen == 0) return;
  File f = LittleFS.open(DIAG_LOG_PATH, "a");
  if (f && f.size() > DIAG_LOG_MAX_BYTES) {
    f.close();
    LittleFS.remove(DIAG_LOG_PREV_PATH);
    LittleFS.rename(DIAG_LOG_PATH, DIAG_LOG_PREV_PATH);
    f = LittleFS.open(DIAG_LOG_PATH, "w");
  }
  if (f) {
    f.write((const uint8_t*)diagBuf, diagBufLen);
    if (diagDropped) f.printf("diag,dropped_lines=%u\n", (unsigned)diagDropped);
    f.close();
  }
  diagBufLen = 0;
  diagDropped = 0;
}

// An oldest-first list of capture files held in RAM, so cleanup never has to
// walk the directory (an O(files) scan) at capture time. Built once at boot,
// then maintained incrementally. It tolerates stale entries — e.g. a file
// deleted from the web page: removing a missing file just pops the entry.
const uint16_t MAX_TRACKED_CAPTURES = 256;
struct CaptureEntry { char name[36]; uint32_t sizeBytes; uint32_t lastWrite; };
CaptureEntry capIndex[MAX_TRACKED_CAPTURES];
uint16_t capHead = 0;           // position of the oldest entry
uint16_t capCount = 0;
size_t fsTotalBytes = 0;
size_t fsUsedEstimate = 0;      // tracked incrementally — LittleFS.usedBytes() is a slow full traversal
uint8_t closesSinceResync = 0;
uint32_t bootIndexMs = 0;
uint32_t recentCapBytes[RECENT_CAPTURES_TRACKED]; // sizes of the last few closed captures (ring)
uint8_t recentCapPos = 0;
size_t capBytesThisCapture = 0; // bytes written to the capture currently open
bool extremeGaveUp = false;     // mid-capture purge found nothing left to delete

size_t fsBlockRound(size_t bytes) { return ((bytes + FS_BLOCK_BYTES - 1) / FS_BLOCK_BYTES) * FS_BLOCK_BYTES; }
size_t fsFreeEstimate() { return (fsTotalBytes > fsUsedEstimate) ? fsTotalBytes - fsUsedEstimate : 0; }

void recentCaptureNote(uint32_t bytes) {
  recentCapBytes[recentCapPos] = bytes;
  recentCapPos = (recentCapPos + 1) % RECENT_CAPTURES_TRACKED;
}
size_t largestRecentCapture() {
  uint32_t m = 0;
  for (uint8_t i = 0; i < RECENT_CAPTURES_TRACKED; i++) if (recentCapBytes[i] > m) m = recentCapBytes[i];
  return m;
}
// Space to reserve for the next capture: a multiple of the largest recent one.
size_t nextCaptureBudgetBytes() {
  size_t b = fsBlockRound(largestRecentCapture()) * CAPTURE_BUDGET_FACTOR;
  if (b < MIN_CAPTURE_BUDGET_BYTES) b = MIN_CAPTURE_BUDGET_BYTES;
  size_t cap = fsTotalBytes / 2; // never reserve more than half the flash
  if (b > cap) b = cap;
  return b;
}
// How much free space the end-of-capture cleanup aims for.
size_t cleanupTargetBytes() {
  size_t floorBytes = (fsTotalBytes / 100) * FREE_FLOOR_PERCENT;
  size_t budget = nextCaptureBudgetBytes();
  return budget > floorBytes ? budget : floorBytes;
}

// Registers a capture file that has just been created (name without leading slash).
void capIndexPushBack(const char* nameNoSlash) {
  if (capCount >= MAX_TRACKED_CAPTURES) return; // can't track it; cleanup still works on the entries it does hold
  CaptureEntry& e = capIndex[(capHead + capCount) % MAX_TRACKED_CAPTURES];
  strncpy(e.name, nameNoSlash, sizeof(e.name) - 1);
  e.name[sizeof(e.name) - 1] = 0;
  e.sizeBytes = 0;
  e.lastWrite = 0;
  capCount++;
}

// Builds the index from what's on flash (once, at boot), oldest first, and
// seeds the "recent capture sizes" from the newest few.
void loadCaptureIndex() {
  capHead = 0;
  capCount = 0;
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  while (file) {
    String name = String(file.name());
    if (name.startsWith("/")) name = name.substring(1);
    if (name.startsWith("cap_") && name.endsWith(".csv") &&
        name.length() < sizeof(capIndex[0].name) && capCount < MAX_TRACKED_CAPTURES) {
      CaptureEntry& e = capIndex[capCount++];
      strcpy(e.name, name.c_str());
      e.sizeBytes = file.size();
      e.lastWrite = (uint32_t)file.getLastWrite();
    }
    file = root.openNextFile();
  }
  // Insertion sort, oldest first (by last-write time, then name).
  for (uint16_t i = 1; i < capCount; i++) {
    CaptureEntry key = capIndex[i];
    int j = (int)i - 1;
    while (j >= 0 && (capIndex[j].lastWrite > key.lastWrite ||
           (capIndex[j].lastWrite == key.lastWrite && strcmp(capIndex[j].name, key.name) > 0))) {
      capIndex[j + 1] = capIndex[j];
      j--;
    }
    capIndex[j + 1] = key;
  }
  for (uint8_t i = 0; i < RECENT_CAPTURES_TRACKED; i++) recentCapBytes[i] = 0;
  recentCapPos = 0;
  uint16_t seed = (capCount < RECENT_CAPTURES_TRACKED) ? capCount : RECENT_CAPTURES_TRACKED;
  for (uint16_t i = capCount - seed; i < capCount; i++) recentCaptureNote(capIndex[(capHead + i) % MAX_TRACKED_CAPTURES].sizeBytes);
}

// Deletes the oldest tracked capture. Returns true if a file was removed.
bool deleteOldestCapture() {
  while (capCount > 0) {
    CaptureEntry& e = capIndex[capHead];
    String path = String("/") + e.name;
    uint32_t sz = e.sizeBytes;
    bool ok = LittleFS.remove(path);
    capHead = (capHead + 1) % MAX_TRACKED_CAPTURES;
    capCount--;
    if (ok) {
      size_t freed = fsBlockRound(sz);
      fsUsedEstimate = (fsUsedEstimate > freed) ? fsUsedEstimate - freed : 0;
      Serial.print("Freed space by deleting oldest capture: ");
      Serial.println(path);
      return true;
    }
    // Stale entry (the file was already gone) — carry on to the next oldest.
  }
  return false;
}

// Deletes oldest captures until enough is free (bounded by MAX_DELETES_PER_CALL
// so one call can't stall sampling for long). atOpen=true is for the moment a
// capture is about to start: the time right after a wake is the one place a
// stall costs real data, so only clean up there if space is genuinely short
// for the capture about to be written; otherwise (a capture just finished and
// the device is idle) aim for the full target. Returns the number deleted.
int ensureFreeSpace(bool atOpen) {
  size_t goal;
  if (atOpen) {
    goal = fsBlockRound(largestRecentCapture());
    if (goal < EMERGENCY_FREE_BYTES) goal = EMERGENCY_FREE_BYTES;
  } else {
    goal = cleanupTargetBytes();
  }
  int deleted = 0;
  while (deleted < MAX_DELETES_PER_CALL && fsFreeEstimate() < goal) {
    if (!deleteOldestCapture()) break; // nothing tracked left to delete
    deleted++;
  }
  return deleted;
}

// Records a capture that has just been closed with its final size.
void registerClosedCapture(const char* nameNoSlash, size_t sizeBytes) {
  if (capCount > 0) {
    CaptureEntry& last = capIndex[(capHead + capCount - 1) % MAX_TRACKED_CAPTURES];
    if (strcmp(last.name, nameNoSlash) == 0) last.sizeBytes = sizeBytes;
  }
  fsUsedEstimate += fsBlockRound(sizeBytes);
  recentCaptureNote((uint32_t)sizeBytes);
}

// Re-reads the exact used-bytes figure every FS_RESYNC_EVERY_N_CLOSES
// captures, so drift in the running estimate (filesystem metadata overhead,
// files deleted from the web page, the diagnostic log growing) can't
// accumulate. It's slow, so it only runs at the end of a capture when the
// device is idle. Returns how long it took in ms, or 0 if it wasn't due.
uint32_t storageResyncIfDue() {
  if (++closesSinceResync < FS_RESYNC_EVERY_N_CLOSES) return 0;
  closesSinceResync = 0;
  uint32_t r0 = millis();
  fsUsedEstimate = LittleFS.usedBytes();
  uint32_t ms = millis() - r0;
  return ms ? ms : 1;
}

// ---- Mid-capture space guard ("something extreme") ----
void resetCaptureSpaceTracking() {
  capBytesThisCapture = 0;
  extremeGaveUp = false;
}
// Free space as it is right now: the running estimate doesn't include the
// capture that's still open, so subtract what it has written so far.
size_t fsFreeNetOfOpenCapture() {
  size_t open = fsBlockRound(capBytesThisCapture);
  size_t f = fsFreeEstimate();
  return f > open ? f - open : 0;
}
// Called when the open capture has eaten into the last of the free space (it
// is far bigger than anything recent). Purges the oldest captures NOW — a
// one-off stall, accepted because the alternative is writes failing — and
// logs it. Returns the number deleted.
int extremeSpaceGuard(const char* tsForLog) {
  if (extremeGaveUp) return 0;
  int deleted = 0;
  while (deleted < MAX_DELETES_PER_CALL && fsFreeNetOfOpenCapture() < EXTREME_LOW_FREE_BYTES * 2) {
    if (!deleteOldestCapture()) { extremeGaveUp = true; break; }
    deleted++;
  }
  char line[200];
  snprintf(line, sizeof(line), "%s,extreme,reason=low_free,deleted=%d,free_net_kb=%u,capture_kb=%u,gave_up=%d",
           tsForLog, deleted, (unsigned)(fsFreeNetOfOpenCapture() / 1024), (unsigned)(capBytesThisCapture / 1024), extremeGaveUp ? 1 : 0);
  Serial.println(line);
  diagLog(line);
  return deleted;
}
// ---- Capture index & cleanup (end) ----

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
    webSocket.sendTXT(clientId, "READY,esp32c3_power_logger_wifi_ina226");
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

void startCaptureFile(uint64_t wallMs) {
  uint32_t t0 = DIAG_FLUSH_TIMING ? millis() : 0;
  int filesDeleted = ensureFreeSpace(true);
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
    resetCaptureSpaceTracking();
    capIndexPushBack(captureFilename + 1); // +1 skips the leading slash
    capBytesThisCapture += captureFile.println("timestamp,voltage_mV,current_mA,power_mW,active");
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
    char diagLine[220];
    snprintf(diagLine, sizeof(diagLine),
             "%s,open,ensureFreeSpace_ms=%lu,deleted=%d,open_ms=%lu,header_write_ms=%lu,total_ms=%lu,files=%u,fs_free_est_kb=%u",
             tsBuf, (unsigned long)(t1 - t0), filesDeleted, (unsigned long)(t2 - t1),
             (unsigned long)(t3 - t2), (unsigned long)(t3 - t0),
             (unsigned)capCount, (unsigned)(fsFreeEstimate() / 1024));
    Serial.println(diagLine);
    diagLog(diagLine);
  }
}

// Samples write quickly almost always; this only logs the rare cases where
// a plain write (not the periodic flush below) takes noticeably longer
// than one sample interval, which previously went unaccounted for — e.g.
// a brand-new file's very first write can trigger its own block
// allocation/erase that LittleFS.open() alone doesn't incur.
// Raised from 15 to 40 ms: at 20 Hz a stall only costs a sample once it
// approaches the 50 ms interval, and the normal LittleFS cache commit
// (~16-20 ms, every few seconds) is no longer worth a log row.
const uint32_t SLOW_WRITE_THRESHOLD_MS = 40;

// activeFlag goes in the CSV's "active" column: 1 for a sample taken while
// genuinely in the active state, 0 for pre-roll/post-roll context rows
// written just outside it (see PREROLL_MS / POSTROLL_MS).
void writeSampleToCapture(const char* timestamp, float voltage_mV, float current_mA, float power_mW, bool activeFlag) {
  if (!captureFile) return;
  uint32_t tw0 = DIAG_FLUSH_TIMING ? millis() : 0;
  size_t wrote = captureFile.printf("%s,%.1f,%.3f,%.3f,%d\n", timestamp, voltage_mV, current_mA, power_mW, activeFlag ? 1 : 0);
  capBytesThisCapture += wrote;
  if (DIAG_FLUSH_TIMING) {
    uint32_t writeMs = millis() - tw0;
    if (writeMs > SLOW_WRITE_THRESHOLD_MS) {
      char diagLine[160];
      snprintf(diagLine, sizeof(diagLine), "%s,slow_write,duration_ms=%lu", timestamp, (unsigned long)writeMs);
      Serial.println(diagLine);
      diagLog(diagLine);
    }
  }
  // "Something extreme" #1: this capture is eating into the last of the free
  // space (it's far bigger than any recent one) — purge old captures now
  // rather than let writes start failing.
  if (fsFreeNetOfOpenCapture() < EXTREME_LOW_FREE_BYTES) extremeSpaceGuard(timestamp);

  // "Something extreme" #2: a single capture has been open a very long time —
  // flush so the data at risk from a power loss stays bounded. (Normal
  // captures close long before this and are committed by the close.)
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

// Records every sample into the always-running pre-roll ring buffer,
// regardless of capture state, so the last PREROLL_MS of history is
// always available the moment a wake is confirmed.
void prerollPush(uint64_t wallMs, float voltage_mV, float current_mA, float power_mW) {
  prerollBuf[prerollWriteIdx] = { wallMs, voltage_mV, current_mA, power_mW };
  prerollWriteIdx = (prerollWriteIdx + 1) % PREROLL_SAMPLES;
  if (prerollCount < PREROLL_SAMPLES) prerollCount++;
}

// Writes the buffered pre-wake context into the just-opened capture file,
// oldest first. Skips the buffer's newest entry: that's the sample which
// just confirmed the wake, and the normal active-write path in loop()
// writes it separately right afterward — dumping it here too would
// duplicate that row.
void writePrerollToCapture() {
  if (prerollCount <= 1) return;
  uint16_t entriesToWrite = prerollCount - 1;
  // Oldest entry sits at prerollWriteIdx once the buffer has wrapped, or
  // at index 0 if it hasn't filled yet.
  uint16_t oldestIdx = (prerollCount < PREROLL_SAMPLES) ? 0 : prerollWriteIdx;
  for (uint16_t k = 0; k < entriesToWrite; k++) {
    const RecentSample& s = prerollBuf[(oldestIdx + k) % PREROLL_SAMPLES];
    char tsBuf[32];
    formatTimestamp(s.wallMs, tsBuf, sizeof(tsBuf));
    writeSampleToCapture(tsBuf, s.voltage_mV, s.current_mA, s.power_mW, false);
  }
}

void closeCaptureFile() {
  uint32_t c0 = DIAG_FLUSH_TIMING ? millis() : 0;
  size_t savedBytes = 0;
  if (captureFile) {
    captureFile.flush();
    savedBytes = captureFile.size();
    captureFile.close();
    Serial.print("Capture saved: ");
    Serial.println(captureFilename);
    registerClosedCapture(captureFilename + 1, savedBytes);
  }
  // The device is idle now (the 60s sleep grace has just run out): this is
  // where old data gets cleaned out, so the NEXT capture finds room waiting.
  uint32_t resyncMs = storageResyncIfDue();
  int filesDeleted = ensureFreeSpace(false);
  if (DIAG_FLUSH_TIMING) {
    char tsBuf[32];
    formatTimestamp(epochMillis(), tsBuf, sizeof(tsBuf));
    char diagLine[240];
    snprintf(diagLine, sizeof(diagLine),
             "%s,close,saved_bytes=%u,deleted=%d,resync_ms=%lu,cleanup_total_ms=%lu,files=%u,fs_free_est_kb=%u,free_target_kb=%u",
             tsBuf, (unsigned)savedBytes, filesDeleted, (unsigned long)resyncMs,
             (unsigned long)(millis() - c0), (unsigned)capCount, (unsigned)(fsFreeEstimate() / 1024),
             (unsigned)(cleanupTargetBytes() / 1024));
    Serial.println(diagLine);
    diagLog(diagLine);
    diagFlushToFlash(); // idle, so this is the safe moment to touch flash for diagnostics
  }
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
  size_t sz = 0;
  File f = LittleFS.open(path, "r");
  if (f) { sz = f.size(); f.close(); }
  if (LittleFS.remove(path)) {
    size_t freed = fsBlockRound(sz);
    fsUsedEstimate = (fsUsedEstimate > freed) ? fsUsedEstimate - freed : 0;
  }
  httpServer.send(200, "text/plain", "deleted");
}

void handleOptions() {
  addCorsHeaders();
  httpServer.send(204);
}

const char* resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "poweron";
    case ESP_RST_EXT:       return "external";
    case ESP_RST_SW:        return "software";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    default:                return "other";
  }
}

void setup() {
  Serial.begin(115200); // kept for local debugging only; not required by the app
  { uint32_t waitStart = millis(); while (!Serial && millis() - waitStart < SERIAL_WAIT_MS) delay(10); }
  Serial.println("\nESP32-C3 power logger booting...");
#if ARDUINO_USB_CDC_ON_BOOT
  // On the C3 the USB-C port is a native USB-CDC device. With nothing
  // listening, a full Serial TX buffer would otherwise make every print
  // block (up to the default timeout) — which would show up as sampling
  // stalls when running headless. 0 = never block, just drop the output.
  // (If this line fails to compile on your core version, delete it.)
  Serial.setTxTimeoutMs(0);
#endif

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  // Ping the address directly first: a plain I2C probe doesn't depend on how
  // a given INA226_WE version's init() behaves with no device attached.
  Wire.beginTransmission(INA226_I2C_ADDR);
  bool inaPresent = (Wire.endTransmission() == 0);
  if (!inaPresent || !ina226.init()) {
    // Repeat the message rather than printing it once, so it's visible
    // whenever a serial monitor is opened — not just at the instant of boot.
    while (1) {
      Serial.printf("ERROR: INA226 not found at 0x%02X (SDA=GPIO%d, SCL=GPIO%d) - check wiring and I2C address\n",
                    INA226_I2C_ADDR, I2C_SDA_PIN, I2C_SCL_PIN);
      delay(2000);
    }
  }
  ina226.setResistorRange(SHUNT_OHMS, MAX_EXPECTED_A);
  ina226.setAverage(INA226_AVG_SETTING);
  ina226.setConversionTime(INA226_CONV_SETTING);
  ina226.setMeasureMode(INA226_CONTINUOUS);

  if (!LittleFS.begin(true)) { // true = format on mount failure
    Serial.println("ERROR: LittleFS mount failed");
  } else {
    uint32_t idxStart = millis();
    fsTotalBytes = LittleFS.totalBytes();
    fsUsedEstimate = LittleFS.usedBytes(); // the one exact read; from here on it's tracked incrementally
    loadCaptureIndex();
    bootIndexMs = millis() - idxStart;
    Serial.printf("LittleFS ready: %u / %u bytes used, %u captures indexed in %lu ms\n",
                  (unsigned)fsUsedEstimate, (unsigned)fsTotalBytes, (unsigned)capCount, (unsigned long)bootIndexMs);
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  if (WIFI_REDUCE_TX_POWER) WiFi.setTxPower(WIFI_POWER_8_5dBm);
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

  if (DIAG_FLUSH_TIMING) {
    // One row per boot, including why the chip reset — so unexpected reboots
    // (watchdog, brownout, panic) show up in the log instead of only as
    // mysteriously fragmented captures.
    char bootTs[32];
    formatTimestamp(epochMillis(), bootTs, sizeof(bootTs));
    char bootLine[220];
    snprintf(bootLine, sizeof(bootLine), "%s,boot,reset=%s,files=%u,fs_used_kb=%u,fs_total_kb=%u,index_ms=%lu",
             bootTs, resetReasonName(esp_reset_reason()), (unsigned)capCount,
             (unsigned)(fsUsedEstimate / 1024), (unsigned)(fsTotalBytes / 1024), (unsigned long)bootIndexMs);
    Serial.println(bootLine);
    diagLog(bootLine);
    diagFlushToFlash();
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

  float busVoltage_V  = ina226.getBusVoltage_V();
  float rawCurrent_mA = ina226.getCurrent_mA();
  float current_mA    = (CLIP_NEGATIVE_CURRENT && rawCurrent_mA < 0) ? 0.0f : rawCurrent_mA;
  float power_mW      = busVoltage_V * current_mA; // more stable than getPower_mW() derived value at low currents
  float voltage_mV    = busVoltage_V * 1000.0;

  if (SENSOR_DEBUG_SERIAL && (now - lastSensorDebugMs >= 1000)) {
    lastSensorDebugMs = now;
    Serial.printf("INA226 bus=%.3fV shunt=%.4fmV current=%.3fmA\n",
                  busVoltage_V, ina226.getShuntVoltage_mV(), rawCurrent_mA);
  }

  // Always record into the pre-roll ring buffer first, whatever the capture
  // state — by the time a wake is confirmed below, it already holds the
  // preceding PREROLL_MS of history (and this sample as its newest entry).
  prerollPush(wallMs, voltage_mV, current_mA, power_mW);

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
          // Name the file after its true first row — the oldest pre-roll
          // sample — so the filename's timestamp still matches the file's
          // first data row now that pre-roll starts it ahead of the wake.
          uint64_t fileStartMs = wallMs;
          if (prerollCount > 1) {
            uint16_t oldestIdx = (prerollCount < PREROLL_SAMPLES) ? 0 : prerollWriteIdx;
            fileStartMs = prerollBuf[oldestIdx].wallMs;
          }
          startCaptureFile(fileStartMs);
          writePrerollToCapture();
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

  // ---- Save to flash while actively logging, plus the first POSTROLL_MS
  // after dropping below threshold (post-roll context), but not the rest of
  // a sleep-pending gap ----
  bool isActive = (captureState == CAP_ACTIVE);
  bool inPostroll = (captureState == CAP_SLEEP_PENDING) && ((now - sleepPendingStartMs) < POSTROLL_MS);
  if (isActive || inPostroll) {
    // Post-roll rows go in with active=0: they're context, not part of the
    // true above-threshold span.
    writeSampleToCapture(tsBuf, voltage_mV, current_mA, power_mW, isActive);
  }

  // ---- Always stream the sample over Wi-Fi too, regardless of logging state.
  // The active flag here is the true active state, not "was written to file",
  // so live shading marks only the genuine above-threshold span. ----
  char line[96];
  snprintf(line, sizeof(line), "SAMPLE,%s,%.1f,%.3f,%.3f,%d",
           tsBuf, voltage_mV, current_mA, power_mW, isActive ? 1 : 0);
  webSocket.broadcastTXT(line);
}
