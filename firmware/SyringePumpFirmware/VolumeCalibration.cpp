#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <util/crc16.h>
#include "VolumeCalibration.h"

namespace {
constexpr int RECORD_VERSION_ADDR = 16;
constexpr int RECORD_DATA_ADDR = RECORD_VERSION_ADDR + 1;
constexpr uint8_t RECORD_VERSION = 1;
constexpr uint8_t RECORD_UNCOMMITTED = 0;
constexpr uint8_t VALID_20 = 0x01;
constexpr uint8_t VALID_50 = 0x02;

struct __attribute__((packed)) VolumeRecord {
  float stepsPerML20;
  float stepsPerML50;
  uint8_t valid;
};

static_assert(sizeof(VolumeRecord) == 9, "Unexpected volume EEPROM layout");
constexpr int RECORD_CRC_ADDR = RECORD_DATA_ADDR + sizeof(VolumeRecord);

VolumeRecord record{};

bool validSteps(float value) {
  return isfinite(value) && value >= 1.0f && value <= 1000000.0f;
}

uint8_t syringeMask(int syringeSize) {
  return syringeSize == 20 ? VALID_20 : VALID_50;
}

float& storedSteps(int syringeSize) {
  return syringeSize == 20 ? record.stepsPerML20 : record.stepsPerML50;
}

uint16_t recordCRC(const VolumeRecord& value) {
  uint16_t crc = _crc16_update(0xFFFF, RECORD_VERSION);
  const auto* bytes = reinterpret_cast<const uint8_t*>(&value);
  for (size_t i = 0; i < sizeof(value); ++i) {
    crc = _crc16_update(crc, bytes[i]);
  }
  return crc;
}

bool validRecord(const VolumeRecord& value) {
  if (value.valid & ~(VALID_20 | VALID_50)) return false;
  if ((value.valid & VALID_20) && !validSteps(value.stepsPerML20)) return false;
  if ((value.valid & VALID_50) && !validSteps(value.stepsPerML50)) return false;
  return true;
}

bool readRecord(VolumeRecord& saved) {
  if (EEPROM.read(RECORD_VERSION_ADDR) != RECORD_VERSION) return false;
  uint16_t savedCRC;
  EEPROM.get(RECORD_DATA_ADDR, saved);
  EEPROM.get(RECORD_CRC_ADDR, savedCRC);
  return savedCRC == recordCRC(saved) && validRecord(saved);
}

bool saveRecord() {
  VolumeRecord saved;
  if (readRecord(saved) && memcmp(&saved, &record, sizeof(record)) == 0) return true;

  EEPROM.update(RECORD_VERSION_ADDR, RECORD_UNCOMMITTED);
  if (EEPROM.read(RECORD_VERSION_ADDR) != RECORD_UNCOMMITTED) return false;
  EEPROM.put(RECORD_DATA_ADDR, record);
  EEPROM.put(RECORD_CRC_ADDR, recordCRC(record));
  EEPROM.update(RECORD_VERSION_ADDR, RECORD_VERSION);
  VolumeRecord verified;
  return readRecord(verified) && memcmp(&verified, &record, sizeof(record)) == 0;
}

bool parsePositive(const char* text, float& value) {
  if (!text) return false;
  char* end;
  const float parsed = strtod(text, &end);
  if (end == text || *end || !(parsed > 0.0f) || !isfinite(parsed)) return false;
  value = parsed;
  return true;
}
}

void beginVolumeCalibration() {
  VolumeRecord saved;
  if (readRecord(saved)) record = saved;
}

float applySavedVolumeCalibration(int syringeSize, float provisionalStepsPerML) {
  const uint8_t mask = syringeMask(syringeSize);
  return record.valid & mask ? storedSteps(syringeSize) : provisionalStepsPerML;
}

bool processVolumeCalibrationCommand(
  char* argument,
  int syringeSize,
  float& stepsPerML,
  float provisionalStepsPerML,
  bool canConfigure
) {
  if (!canConfigure) {
    Serial.println(F("ERROR: VOLUME CALIBRATION REQUIRES IDLE OR COMPLETE"));
    return true;
  }

  constexpr char separators[] = " 	";
  char* cursor;
  const char* action = argument ? strtok_r(argument, separators, &cursor) : nullptr;
  if (!action) return false;

  const uint8_t mask = syringeMask(syringeSize);
  if (strcmp(action, "CLEAR") == 0) {
    if (strtok_r(nullptr, separators, &cursor)) {
      Serial.println(F("ERROR: INVALID VOLUME CALIBRATION COMMAND"));
      return true;
    }
    const VolumeRecord previous = record;
    record.valid &= ~mask;
    storedSteps(syringeSize) = 0.0f;
    if (!saveRecord()) {
      record = previous;
      Serial.println(F("ERROR: COULD NOT SAVE VOLUME CALIBRATION"));
      return true;
    }
    stepsPerML = provisionalStepsPerML;
    Serial.println(F("VOLUME CALIBRATION CLEARED"));
    return true;
  }

  if (strcmp(action, "CAL") != 0) return false;
  float commanded;
  float measured;
  const char* commandedText = strtok_r(nullptr, separators, &cursor);
  const char* measuredText = strtok_r(nullptr, separators, &cursor);
  if (!parsePositive(commandedText, commanded) ||
      !parsePositive(measuredText, measured) ||
      strtok_r(nullptr, separators, &cursor) ||
      commanded > syringeSize || measured > syringeSize * 1.5f ||
      commanded / measured < 0.5f || commanded / measured > 2.0f) {
    Serial.println(F("ERROR: INVALID VOLUME CALIBRATION"));
    return true;
  }

  const float corrected = stepsPerML * commanded / measured;
  if (!validSteps(corrected)) {
    Serial.println(F("ERROR: INVALID VOLUME CALIBRATION"));
    return true;
  }

  const VolumeRecord previous = record;
  storedSteps(syringeSize) = corrected;
  record.valid |= mask;
  if (!saveRecord()) {
    record = previous;
    Serial.println(F("ERROR: COULD NOT SAVE VOLUME CALIBRATION"));
    return true;
  }

  stepsPerML = corrected;
  Serial.println(F("VOLUME CALIBRATED"));
  return true;
}

void printVolumeCalibrationStatus(int syringeSize, float stepsPerML) {
  Serial.print(F("VOLUME CALIBRATED: "));
  Serial.println(record.valid & syringeMask(syringeSize) ? F("YES") : F("NO"));
  Serial.println(F("VOLUME CAL STORAGE: EEPROM"));
  Serial.print(F("STEPS PER ML: "));
  Serial.println(stepsPerML, 4);
}
