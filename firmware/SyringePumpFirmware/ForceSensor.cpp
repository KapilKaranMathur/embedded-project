#include <Arduino.h>
#include <HX711.h>
#include <stdlib.h>
#include <string.h>
#include <util/atomic.h>
#include "ForceSensor.h"

namespace {
constexpr byte DOUT_PIN = 40;
constexpr byte SCK_PIN = 42;
constexpr long ADC_MIN = -8388608L;
constexpr long ADC_MAX = 8388607L;
constexpr float RATED_KG = 20.0f;
constexpr float STANDARD_GRAVITY = 9.80665f;
constexpr unsigned long SAMPLE_TIMEOUT_MS = 1000UL;

HX711 sensor;
volatile uint8_t* dataPort;
volatile uint8_t* clockPort;
uint8_t dataMask;
uint8_t clockMask;
bool discardNext = true;
bool waitingForHigh = false;
bool received = false;
bool transferOk = true;
bool expired = false;
unsigned long sampleMs;
long raw;
long zeroCounts;
float countsPerNewton;
float forceN;
float peakN;
bool peakRecorded = false;
float limitN;

bool inRange(long count) {
  return count > ADC_MIN && count < ADC_MAX;
}

bool sampleUsable() {
  return received && !expired && transferOk && inRange(raw) &&
    millis() - sampleMs < SAMPLE_TIMEOUT_MS;
}

// The library handles setup and readiness. This fixed-length transfer avoids
// its internal wait-for-ready loop. Interrupts stay masked only for each pulse.
bool clockBit() {
  bool high;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    *clockPort |= clockMask;
    delayMicroseconds(2);
    high = (*dataPort & dataMask) != 0;
    *clockPort &= static_cast<uint8_t>(~clockMask);
  }
  delayMicroseconds(2);
  return high;
}

long readReadySample() {
  uint32_t bits = 0;
  for (byte i = 0; i < 24; ++i) {
    bits = (bits << 1) | clockBit();
  }
  clockBit();  // Pulse 25 selects channel A, gain 128 for the next conversion.
  const long count = static_cast<long>(bits);
  return (bits & 0x800000UL) ? count - 0x1000000L : count;
}

bool parseCounts(const char* value, long& count) {
  if (!value) return false;
  char* end;
  count = strtol(value, &end, 10);
  return end != value && *end == '\0' && inRange(count);
}

float parsePositive(const char* value) {
  if (!value) return 0.0f;
  char* end;
  const float number = strtod(value, &end);
  return end != value && *end == '\0' && number > 0.0f ? number : 0.0f;
}

void applyCalibration(long zero, float scale) {
  zeroCounts = zero;
  countsPerNewton = scale;
  forceN = (raw - zeroCounts) / countsPerNewton;
  limitN = 0.0f;
  resetForcePeak();
}
}

void beginForceSensor() {
  digitalWrite(SCK_PIN, LOW);
  sensor.begin(DOUT_PIN, SCK_PIN, 128);
  dataPort = portInputRegister(digitalPinToPort(DOUT_PIN));
  clockPort = portOutputRegister(digitalPinToPort(SCK_PIN));
  dataMask = digitalPinToBitMask(DOUT_PIN);
  clockMask = digitalPinToBitMask(SCK_PIN);
  sampleMs = millis();
}

void updateForceSensor(bool trackPeak) {
  if (millis() - sampleMs >= SAMPLE_TIMEOUT_MS) expired = true;

  if (!sensor.is_ready()) {
    waitingForHigh = false;
    return;
  }
  if (waitingForHigh) return;

  const long value = readReadySample();
  transferOk = !sensor.is_ready();
  waitingForHigh = !transferOk;
  if (!transferOk) return;

  if (discardNext) {
    discardNext = false;
    return;
  }

  raw = value;
  sampleMs = millis();
  received = true;
  expired = false;
  if (countsPerNewton != 0.0f && inRange(raw)) {
    forceN = (raw - zeroCounts) / countsPerNewton;
    if (trackPeak) {
      peakRecorded = true;
      if (forceN > peakN) peakN = forceN;
    }
  }
}

bool forceLimitConfigured() {
  return limitN > 0.0f;
}

const char* forceFault() {
  if (!sampleUsable()) return "FORCE SENSOR FAULT";
  if (forceN >= limitN) return "FORCE OVERLOAD";
  return nullptr;
}

void resetForcePeak() {
  peakN = 0.0f;
  peakRecorded = false;
}

void printForceStatus() {
  const unsigned long age = millis() - sampleMs;
  const bool stale = expired || age >= SAMPLE_TIMEOUT_MS;
  const __FlashStringHelper* health = F("OK");
  if (!transferOk) {
    health = F("INVALID");
  } else if (stale) {
    health = received ? F("STALE") : F("NO DATA");
  } else if (!received) {
    health = F("WAITING");
  } else if (!inRange(raw)) {
    health = F("SATURATED");
  }

  Serial.print(F("FORCE SENSOR: "));
  Serial.println(health);
  Serial.print(F("FORCE RAW: "));
  if (received && !stale && transferOk) {
    Serial.println(raw);
    Serial.print(F("FORCE SAMPLE AGE MS: "));
    Serial.println(age);
  } else {
    Serial.println(F("NA"));
  }

  const bool calibrated = countsPerNewton != 0.0f;
  Serial.print(F("FORCE CALIBRATED: "));
  Serial.println(calibrated ? F("YES") : F("NO"));
  Serial.print(F("FORCE LIMIT N: "));
  if (forceLimitConfigured()) Serial.println(limitN, 3);
  else Serial.println(F("NA"));
  Serial.print(F("FORCE N: "));
  if (calibrated && sampleUsable()) Serial.println(forceN, 2);
  else Serial.println(F("NA"));
  if (peakRecorded) {
    Serial.print(F("FORCE PEAK N: "));
    Serial.println(peakN, 2);
  }
}

void processForceCommand(char* argument, bool canConfigure) {
  constexpr char separators[] = " \t";
  char* cursor;
  const char* command = argument ? strtok_r(argument, separators, &cursor) : nullptr;
  if (!command) {
    Serial.println(F("ERROR: FORCE COMMAND REQUIRED"));
    return;
  }

  if (strcmp(command, "RAW") == 0 || strcmp(command, "STATUS") == 0) {
    printForceStatus();
    return;
  }
  if (!canConfigure) {
    Serial.println(F("ERROR: FORCE SETTINGS REQUIRE IDLE OR COMPLETE"));
    return;
  }
  if (!sampleUsable()) {
    Serial.println(F("ERROR: FRESH HX711 SAMPLE REQUIRED"));
    return;
  }

  if (strcmp(command, "CAL") == 0) {
    long zero, loaded;
    const char* zeroText = strtok_r(nullptr, separators, &cursor);
    const char* loadedText = strtok_r(nullptr, separators, &cursor);
    const float kg = parsePositive(strtok_r(nullptr, separators, &cursor));
    if (!parseCounts(zeroText, zero) || !parseCounts(loadedText, loaded) ||
        loaded == zero || !(kg >= 0.000001f && kg <= RATED_KG) ||
        strtok_r(nullptr, separators, &cursor)) {
      Serial.println(F("ERROR: INVALID FORCE CALIBRATION"));
      return;
    }
    applyCalibration(zero, (loaded - zero) / (kg * STANDARD_GRAVITY));
    Serial.println(F("FORCE CALIBRATED"));
  } else if (strcmp(command, "TARE") == 0) {
    if (countsPerNewton == 0.0f || strtok_r(nullptr, separators, &cursor)) {
      Serial.println(F("ERROR: CALIBRATE FORCE BEFORE TARE"));
      return;
    }
    applyCalibration(raw, countsPerNewton);
    Serial.println(F("FORCE TARED"));
  } else if (strcmp(command, "LIMIT") == 0) {
    const float limit = parsePositive(strtok_r(nullptr, separators, &cursor));
    if (countsPerNewton == 0.0f || !(limit >= 0.001f && limit <= RATED_KG * STANDARD_GRAVITY) ||
        strtok_r(nullptr, separators, &cursor)) {
      Serial.println(F("ERROR: INVALID FORCE LIMIT OR CALIBRATION MISSING"));
      return;
    }
    limitN = limit;
    Serial.println(F("FORCE LIMIT UPDATED"));
  } else {
    Serial.println(F("ERROR: UNKNOWN FORCE COMMAND"));
  }
}
