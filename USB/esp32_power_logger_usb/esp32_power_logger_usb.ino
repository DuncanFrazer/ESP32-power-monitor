/*
  ESP32 + INA219 Power Logger
  ---------------------------
  Samples voltage/current continuously and streams CSV over serial.
  Detects device wake/sleep using a hysteresis current threshold and
  tags each sample with an "active" flag, plus emits explicit
  EVENT,WAKE / EVENT,SLEEP lines so a host app doesn't need to
  re-derive detection logic.

  Serial protocol (115200 baud, one line per sample):
    SAMPLE,<millis>,<voltage_mV>,<current_mA>,<power_mW>,<active 0|1>
    EVENT,WAKE,<millis>
    EVENT,SLEEP,<millis>,<duration_ms>

  Library: Adafruit_INA219 (Library Manager -> "Adafruit INA219")

  --- TUNE THESE FOR YOUR DEVICE ---
*/

#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

// ---- Detection thresholds (mA) ----
// Wake threshold: current must exceed this to be considered "active"
const float WAKE_THRESHOLD_MA  = 15.0;
// Sleep threshold: current must drop below this to be considered "idle"
// (kept lower than wake threshold = hysteresis, avoids chatter on noisy edges)
const float SLEEP_THRESHOLD_MA = 10.0;
// Number of consecutive samples that must agree before flipping state,
// to reject single-sample glitches
const uint8_t DEBOUNCE_SAMPLES = 3;

// ---- Sampling ----
const uint32_t SAMPLE_INTERVAL_MS = 10;   // 100 Hz. INA219 continuous mode
                                           // conversion time + I2C overhead
                                           // comfortably supports this;
                                           // push lower (e.g. 5ms) if you
                                           // need finer resolution and your
                                           // I2C bus can keep up.

bool active = false;
uint8_t debounceCount = 0;
uint32_t wakeStartMs = 0;
uint32_t lastSampleMs = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  if (!ina219.begin()) {
    Serial.println("ERROR,INA219 not found - check wiring");
    while (1) { delay(1000); }
  }

  // 16V / 400mA range gives better resolution at low currents than the
  // default 32V/2A range. Adjust if your device's active-state current
  // exceeds ~400mA.
  ina219.setCalibration_16V_400mA();

  Serial.println("READY,esp32_power_logger");
}

void loop() {
  uint32_t now = millis();
  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  lastSampleMs = now;

  float busVoltage_V  = ina219.getBusVoltage_V();
  float current_mA    = ina219.getCurrent_mA();
  float power_mW      = busVoltage_V * current_mA; // more stable than getPower_mW() derived value at low currents
  float voltage_mV    = busVoltage_V * 1000.0;

  // ---- Hysteresis + debounce wake/sleep detection ----
  bool instantAboveWake  = current_mA > WAKE_THRESHOLD_MA;
  bool instantBelowSleep = current_mA < SLEEP_THRESHOLD_MA;

  if (!active && instantAboveWake) {
    debounceCount++;
    if (debounceCount >= DEBOUNCE_SAMPLES) {
      active = true;
      wakeStartMs = now;
      debounceCount = 0;
      Serial.print("EVENT,WAKE,");
      Serial.println(now);
    }
  } else if (active && instantBelowSleep) {
    debounceCount++;
    if (debounceCount >= DEBOUNCE_SAMPLES) {
      active = false;
      uint32_t duration = now - wakeStartMs;
      debounceCount = 0;
      Serial.print("EVENT,SLEEP,");
      Serial.print(now);
      Serial.print(",");
      Serial.println(duration);
    }
  } else {
    debounceCount = 0; // reset on any sample that doesn't match the pending transition
  }

  // ---- Always stream the sample, regardless of active state ----
  Serial.print("SAMPLE,");
  Serial.print(now);
  Serial.print(",");
  Serial.print(voltage_mV, 1);
  Serial.print(",");
  Serial.print(current_mA, 3);
  Serial.print(",");
  Serial.print(power_mW, 3);
  Serial.print(",");
  Serial.println(active ? 1 : 0);
}
