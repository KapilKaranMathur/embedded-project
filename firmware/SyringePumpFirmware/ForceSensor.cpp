#include <Arduino.h>
#include <HX711.h>
#include "ForceSensor.h"

namespace {
constexpr byte DOUT_PIN = 40;
constexpr byte SCK_PIN = 42;
constexpr long ADC_MIN = -8388608L;
constexpr long ADC_MAX = 8388607L;
constexpr unsigned long SAMPLE_TIMEOUT_MS = 1000UL;

HX711 sensor;
bool sampling = false;
bool received = false;
bool discardNext = true;
bool waitingForHigh = false;
unsigned long sampleMs = 0;
long raw = 0;
}

void beginForceSensor() {
  digitalWrite(SCK_PIN, LOW);
  sensor.begin(DOUT_PIN, SCK_PIN, 128);
}

void updateForceSensor(bool samplingAllowed) {
  if (!samplingAllowed) {
    sampling = false;
    return;
  }

  if (!sampling) {
    sampling = true;
    received = false;
    discardNext = true;
    waitingForHigh = false;
    sampleMs = millis();
  }

  if (!sensor.is_ready()) {
    waitingForHigh = false;
  } else if (!waitingForHigh) {
    // DOUT holds low until clocked. Skip the library's wait during conversion.
    const long value = sensor.read();
    waitingForHigh = true;
    if (discardNext) {
      // The first conversion may have finished while sampling was suspended.
      discardNext = false;
      return;
    }

    raw = value;
    sampleMs = millis();
    received = true;
  }
}

void printForceStatus() {
  const unsigned long age = millis() - sampleMs;
  const __FlashStringHelper* health;
  if (!sampling) {
    health = F("SUSPENDED");
  } else if (age >= SAMPLE_TIMEOUT_MS) {
    health = received ? F("STALE") : F("NO DATA");
  } else if (!received) {
    health = F("WAITING");
  } else if (raw <= ADC_MIN || raw >= ADC_MAX) {
    health = F("SATURATED");
  } else {
    health = F("OK");
  }

  Serial.print(F("FORCE SENSOR: "));
  Serial.println(health);
  if (!sampling) {
    return;
  }

  Serial.print(F("FORCE RAW: "));
  if (received && age < SAMPLE_TIMEOUT_MS) {
    Serial.println(raw);
    Serial.print(F("FORCE SAMPLE AGE MS: "));
    Serial.println(age);
  } else {
    Serial.println(F("NA"));
  }
}
