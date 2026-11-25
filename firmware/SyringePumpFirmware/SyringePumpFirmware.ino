#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <limits.h>
#include "ForceSensor.h"
#include "VolumeCalibration.h"

// =====================================================
// SYRINGE PUMP V4.4
// Arduino Mega 2560 + RAMPS 1.4 + A4988
// FORWARD / BACKWARD MOTOR CONTROL
// Non-medical benchtop prototype
// =====================================================

// RAMPS X-axis pins
const byte STEP_PIN = 54;
const byte DIR_PIN = 55;
const byte EN_PIN = 38;

const byte HOME_PIN = 3;
const byte MAX_PIN = 2;

// Motor: 1.8 degrees, A4988 at 1/16 microstepping
const float STEPS_PER_REV = 3200.0f;

// T8 lead screw with 8 mm lead
const float STEPS_PER_MM = 400.0f;

// Provisional operating limits
const float MAX_STEP_HZ = 1000.0f;
const float MIN_STEP_HZ = 0.01f;
const float ACCEL = 800.0f;
const float JOG_STEP_HZ = 400.0f;
const float COMMISSION_STEP_HZ = STEPS_PER_MM * 0.5f;
const unsigned long COMMISSION_STEPS = STEPS_PER_MM;
const unsigned long COMMISSION_DURATION_MS = 3000UL;
const float REVERSE_CRUISE_HZ = 1.5f * STEPS_PER_MM;
const float REVERSE_UNREFERENCED_HZ = 1.0f * STEPS_PER_MM;
const float HOME_APPROACH_HZ = 0.75f * STEPS_PER_MM;
const unsigned long HOME_APPROACH_STEPS = 2.0f * STEPS_PER_MM;
const unsigned long HOME_RELEASE_STEPS = 2.0f * STEPS_PER_MM;
const unsigned long HOME_RELEASE_CONFIRM_MS = 20UL;

const unsigned long MIN_STEP_US = 1000UL;
const unsigned long LINK_TIMEOUT_MS = 15000UL;
const unsigned long JOG_TIMEOUT_MS = 400UL;

// Syringe settings — provisional dimensions
int syringeSize = 20;
float boreMM = 20.05f;
float stepsPerML = 0.0f;

// Independent user settings
float flowRate = 1.0f;       // mL/min
float targetVolume = 1.0f;   // mL-equivalent travel
float retractSpeedHz = REVERSE_CRUISE_HZ;

enum PumpState {
  IDLE,
  RUNNING,
  PAUSING,
  PAUSED,
  JOGGING,
  COMPLETE,
  ALARM
};

enum Direction {
  FORWARD,
  BACKWARD
};

PumpState state = IDLE;
Direction direction = FORWARD;
Direction runDirection = FORWARD;
bool returnRun = false;
Direction jogDirection = FORWARD;
bool jogMotorEnabled = false;
unsigned long commissionStepsLeft = 0;
unsigned long commissionStartedMs = 0;
unsigned long lastJogKeepMs = 0;
const char* alarmReason = nullptr;

// Carriage position is estimated from STEP pulses since the last HOME hit.
bool positionKnown = false;
unsigned long positionSteps = 0;
unsigned long measuredMaxSteps = 0;
unsigned long forwardLimitSteps = 0;
bool maxReachedByJog = false;
bool departingHome = false;
unsigned long homeDepartureSteps = 0;
bool homeReleasePending = false;
unsigned long homeReleasedMs = 0;

// Motion variables
float currentHz = 0.0f;
float stepCredit = 0.0f;

unsigned long deliveredSteps = 0;
unsigned long targetSteps = 0;

unsigned long lastUpdateUs = 0;
unsigned long lastStepUs = 0;
unsigned long lastContactMs = 0;

// Serial input
char commandBuffer[64];
byte commandIndex = 0;
bool commandOverflow = false;

// =====================================================
// CALCULATIONS
// =====================================================

void calculateSyringe() {
  float areaMM2 = PI * boreMM * boreMM / 4.0f;
  const float provisionalStepsPerML = STEPS_PER_MM * 1000.0f / areaMM2;
  stepsPerML = applySavedVolumeCalibration(syringeSize, provisionalStepsPerML);
}

float rateHz(float rate) {
  return rate * stepsPerML / 60.0f;
}

float maxRate() {
  return 60.0f * MAX_STEP_HZ / stepsPerML;
}

bool validRate(float rate) {
  if (!(rate > 0.0f && rate <= 1000000.0f)) {
    return false;
  }

  float hz = rateHz(rate);

  return hz >= MIN_STEP_HZ && hz <= MAX_STEP_HZ;
}

bool positiveNumber(const char* text, float &out) {
  if (!text || !*text) {
    return false;
  }

  char* end;
  double number = strtod(text, &end);

  if (end == text) {
    return false;
  }

  while (*end && isspace((unsigned char)*end)) {
    ++end;
  }

  if (*end || !(number > 0.0 && number <= 1000000.0)) {
    return false;
  }

  out = (float)number;
  return true;
}

// =====================================================
// LIMIT SWITCHES
// =====================================================

// Verified for this rig:
// Released = LOW
// Pressed  = HIGH

bool homePressed() { return digitalRead(HOME_PIN) == HIGH; }
bool maxPressed() { return digitalRead(MAX_PIN) == HIGH; }
bool bothLimitsPressed() { return homePressed() && maxPressed(); }
bool runActive() { return state == RUNNING || state == PAUSING || state == PAUSED; }

void referenceHome() {
  positionSteps = 0;
  positionKnown = true;
  maxReachedByJog = false;
  homeDepartureSteps = 0;
  homeReleasePending = false;
}

void updateHomeRelease() {
  // Keep the remaining allowance across stops until HOME has reliably opened.
  if (!departingHome && homeDepartureSteps == 0) return;
  if (homePressed()) {
    homeReleasePending = false;
  } else if (!homeReleasePending) {
    homeReleasedMs = millis();
    homeReleasePending = true;
  } else if (millis() - homeReleasedMs >= HOME_RELEASE_CONFIRM_MS) {
    departingHome = false;
    homeDepartureSteps = 0;
    homeReleasePending = false;
  }
}

void trackTravelStep(Direction moved) {
  if (!positionKnown) return;
  if (moved == BACKWARD) {
    if (positionSteps > 0) --positionSteps;
    else positionKnown = false;
  } else if (positionSteps < ULONG_MAX) {
    ++positionSteps;
  } else {
    positionKnown = false;
  }
}

unsigned long remainingForwardSteps() {
  return positionSteps < forwardLimitSteps ? forwardLimitSteps - positionSteps : 0;
}

float limitReverseSpeed(float requestedHz) {
  if (!positionKnown) {
    return min(requestedHz, REVERSE_UNREFERENCED_HZ);
  }
  if (positionSteps <= HOME_APPROACH_STEPS || requestedHz <= HOME_APPROACH_HZ) {
    return min(requestedHz, HOME_APPROACH_HZ);
  }

  const unsigned long brakingSteps = positionSteps - HOME_APPROACH_STEPS;
  const float approachLimitSquared = HOME_APPROACH_HZ * HOME_APPROACH_HZ +
    2.0f * ACCEL * brakingSteps;
  return approachLimitSquared < requestedHz * requestedHz
    ? sqrtf(approachLimitSquared) : requestedHz;
}

bool checkForwardTravel(Direction requested, unsigned long steps) {
  if (requested != FORWARD || measuredMaxSteps == 0) return true;
  if (!positionKnown) {
    Serial.println(F("ERROR: RETURN TO HOME TO RESTORE POSITION"));
    return false;
  }
  if (steps > remainingForwardSteps()) {
    Serial.println(F("ERROR: TARGET EXCEEDS REMAINING FORWARD TRAVEL"));
    return false;
  }
  return true;
}

bool maxCaptureReady() {
  return maxReachedByJog && positionKnown && positionSteps > 0 &&
    maxPressed() && !homePressed();
}

void processTravelCommand(const char* argument) {
  if (state != IDLE && state != COMPLETE) {
    Serial.println(F("ERROR: TRAVEL SETTINGS REQUIRE IDLE OR COMPLETE"));
    return;
  }

  if (argument && strcmp(argument, "CLEAR") == 0) {
    measuredMaxSteps = 0;
    forwardLimitSteps = 0;
    maxReachedByJog = false;
    Serial.println(F("MAX TRAVEL CLEARED"));
    return;
  }

  if (!argument || strncmp(argument, "MAX ", 4) != 0) {
    Serial.println(F("ERROR: USE TRAVEL MAX <CLEARANCE MM> OR TRAVEL CLEAR"));
    return;
  }
  if (!maxCaptureReady()) {
    Serial.println(F("ERROR: RETURN TO HOME THEN JOG TO MAX BEFORE RECORDING"));
    return;
  }
  float clearanceMm;
  if (!positiveNumber(argument + 4, clearanceMm)) {
    Serial.println(F("ERROR: ENTER POSITIVE MAX CLEARANCE IN MM"));
    return;
  }
  const float clearanceSteps = ceilf(clearanceMm * STEPS_PER_MM);
  if (!(clearanceSteps >= 1.0f && clearanceSteps < positionSteps)) {
    Serial.println(F("ERROR: CLEARANCE MUST BE LESS THAN MEASURED TRAVEL"));
    return;
  }

  measuredMaxSteps = positionSteps;
  forwardLimitSteps = measuredMaxSteps - static_cast<unsigned long>(clearanceSteps);
  Serial.println(F("MAX TRAVEL RECORDED"));
}

void printTravelValue(const __FlashStringHelper* label, bool available, unsigned long steps) {
  Serial.print(label);
  if (available) Serial.println(steps / STEPS_PER_MM, 3);
  else Serial.println(F("NA"));
}

void printTravelStatus() {
  Serial.print(F("HOME SWITCH: "));
  Serial.println(homePressed() ? F("PRESSED") : F("RELEASED"));
  Serial.print(F("MAX SWITCH: "));
  Serial.println(maxPressed() ? F("PRESSED") : F("RELEASED"));
  printTravelValue(F("POSITION MM: "), positionKnown, positionSteps);
  const bool measured = measuredMaxSteps > 0;
  printTravelValue(F("MAX TRAVEL MM: "), measured, measuredMaxSteps);
  printTravelValue(F("FORWARD LIMIT MM: "), measured, forwardLimitSteps);
  printTravelValue(F("FORWARD ROOM MM: "), measured && positionKnown, remainingForwardSteps());
  Serial.print(F("MAX CAPTURE READY: "));
  Serial.println(maxCaptureReady() ? F("YES") : F("NO"));
}

// =====================================================
// MOTOR SAFETY
// =====================================================

void disableMotor() {
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(EN_PIN, HIGH);

  jogMotorEnabled = false;
  commissionStepsLeft = 0;
  departingHome = false;
  currentHz = 0.0f;
  stepCredit = 0.0f;
}

void triggerAlarm(const char* reason) {
  if (state == ALARM) {
    return;
  }

  disableMotor();
  state = ALARM;
  alarmReason = reason;
  positionKnown = false;
  maxReachedByJog = false;

  Serial.print(F("ALARM: "));
  Serial.println(reason);

  Serial.println(F("MOTOR DISABLED"));
  Serial.println(F("RESET REQUIRED"));
}

void stopPump() {
  disableMotor();

  // A STOP command cannot clear a latched alarm.
  if (state != ALARM) {
    state = IDLE;
  }

  Serial.println(F("PUMP STOPPED"));
}

// =====================================================
// START
// =====================================================

bool motionAvailable() {
  if (state == ALARM) {
    Serial.println(F("ERROR: RESET REQUIRED"));
    return false;
  }

  if (state != IDLE && state != COMPLETE) {
    Serial.println(F("ERROR: STOP FIRST"));
    return false;
  }

  return true;
}

bool checkForwardForce(Direction requested) {
  if (requested != FORWARD) return true;

  if (!forceLimitConfigured()) {
    Serial.println(F("ERROR: CALIBRATE FORCE AND SET A LIMIT BEFORE FORWARD START"));
    return false;
  }
  if (const char* fault = forceFault()) {
    triggerAlarm(fault);
    return false;
  }
  return true;
}

void enableMotion(Direction requested) {
  maxReachedByJog = false;
  departingHome = requested == FORWARD && (homePressed() || homeDepartureSteps > 0);
  currentHz = 0.0f;
  stepCredit = 0.0f;

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, requested == FORWARD ? LOW : HIGH);
  delayMicroseconds(5);
  digitalWrite(EN_PIN, LOW);

  lastUpdateUs = micros();
  lastStepUs = lastUpdateUs;
}

bool forwardHomeClear() {
  if (departingHome && homeDepartureSteps < HOME_RELEASE_STEPS) return true;
  if (!departingHome && !homePressed()) return true;

  triggerAlarm(departingHome ? "HOME SWITCH DID NOT RELEASE" :
    "HOME SWITCH ACTIVE DURING FORWARD MOTION");
  return false;
}

bool forwardDosePathClear() {
  if (bothLimitsPressed()) {
    triggerAlarm("BOTH LIMIT SWITCHES ACTIVE");
    return false;
  }
  if (maxPressed()) {
    if (deliveredSteps < targetSteps) {
      triggerAlarm("MAX SWITCH BEFORE TARGET VOLUME");
      return false;
    }
  }
  return forwardHomeClear();
}

bool returnPathClear() {
  if (bothLimitsPressed()) {
    triggerAlarm("BOTH LIMIT SWITCHES ACTIVE");
    return false;
  }
  if (!homePressed()) return true;

  if (runActive()) {
    disableMotor();
    referenceHome();
    state = COMPLETE;
    Serial.println(F("HOME REACHED"));
  } else {
    Serial.println(F("ERROR: HOME LIMIT ACTIVE"));
  }
  return false;
}

void startPump() {
  if (!motionAvailable()) return;

  const bool returning = direction == BACKWARD;
  if (returning) {
    if (!returnPathClear()) return;
  } else {
    if (bothLimitsPressed()) {
      triggerAlarm("BOTH LIMIT SWITCHES ACTIVE");
      return;
    }
    if (maxPressed()) {
      Serial.println(F("ERROR: MAX LIMIT ACTIVE"));
      return;
    }
    if (homePressed() && homeDepartureSteps >= HOME_RELEASE_STEPS) {
      triggerAlarm("HOME SWITCH DID NOT RELEASE");
      return;
    }
  }

  if (!returning && !validRate(flowRate)) {
    Serial.println(F("ERROR: INVALID FLOW RATE"));
    return;
  }
  if (!checkForwardForce(direction)) return;

  unsigned long moveTarget = 0;
  if (!returning) {
    const float requestedSteps = targetVolume * stepsPerML;
    if (!(requestedSteps >= 1.0f && requestedSteps <= 4000000000.0f)) {
      Serial.println(F("ERROR: INVALID TARGET"));
      return;
    }
    moveTarget = (unsigned long)(requestedSteps + 0.5f);
  }
  if (!checkForwardTravel(direction, moveTarget)) return;

  targetSteps = moveTarget;
  deliveredSteps = 0;
  runDirection = direction;
  returnRun = returning;
  resetForcePeak();

  enableMotion(direction);
  lastContactMs = millis();

  state = RUNNING;

  Serial.println(F("PUMP STARTED"));
}

void finishJog(const __FlashStringHelper* message) {
  if (state != JOGGING) return;
  disableMotor();
  state = IDLE;
  Serial.println(message);
}

void timeoutJog() {
  if (jogMotorEnabled) positionKnown = false;
  finishJog(F("JOG TIMEOUT"));
}

bool jogPathClear(Direction requested) {
  const bool home = homePressed();
  const bool max = maxPressed();
  if (home && max) {
    triggerAlarm("BOTH LIMIT SWITCHES ACTIVE");
    return false;
  }

  const bool towardHome = requested == BACKWARD;
  if (!(towardHome ? home : max)) {
    if (towardHome) return true;
    if (jogMotorEnabled) return forwardHomeClear();
    if (home && homeDepartureSteps >= HOME_RELEASE_STEPS) {
      triggerAlarm("HOME SWITCH DID NOT RELEASE");
      return false;
    }
    return true;
  }

  if (state == JOGGING) {
    if (jogMotorEnabled) {
      if (towardHome) referenceHome();
      else maxReachedByJog = positionKnown && positionSteps > 0;
    }
    finishJog(towardHome ? F("JOG HOME REACHED") : F("JOG MAX REACHED"));
  } else {
    Serial.println(towardHome ? F("ERROR: HOME LIMIT ACTIVE") : F("ERROR: MAX LIMIT ACTIVE"));
  }
  return false;
}

bool checkJogForce(Direction requested, bool commissioning) {
  if (!commissioning) return checkForwardForce(requested);
  if (const char* fault = forceFault(false)) {
    if (strcmp(fault, "FORCE OVERLOAD") == 0) triggerAlarm(fault);
    else Serial.println(F("ERROR: COMMISSIONING REQUIRES FRESH CALIBRATED FORCE DATA"));
    return false;
  }
  return true;
}

void prepareJog(Direction requested, bool commissioning) {
  if (!motionAvailable() || !jogPathClear(requested) ||
      !checkForwardTravel(requested, 1) || !checkJogForce(requested, commissioning)) return;

  jogDirection = requested;
  commissionStepsLeft = commissioning ? COMMISSION_STEPS : 0;
  lastJogKeepMs = millis();
  state = JOGGING;
  // An acknowledged direction alone does not enable the driver.
  Serial.println(F("JOG READY"));
}

void keepJog(bool commissioning) {
  if (state != JOGGING || commissioning != (commissionStepsLeft > 0)) return;
  const unsigned long now = millis();
  if (now - lastJogKeepMs >= JOG_TIMEOUT_MS) {
    timeoutJog();
    return;
  }

  if (!jogMotorEnabled) {
    if (!jogPathClear(jogDirection) || !checkForwardTravel(jogDirection, 1) ||
        !checkJogForce(jogDirection, commissioning)) return;
    if (commissioning) {
      resetForcePeak();
      commissionStartedMs = now;
    }
    enableMotion(jogDirection);
    jogMotorEnabled = true;
    Serial.println(F("JOG STARTED"));
  }
  lastJogKeepMs = now;
}

// =====================================================
// PAUSE / RESUME
// =====================================================

void pausePump() {
  if (state != RUNNING) {
    Serial.println(F("ERROR: NOT RUNNING"));
    return;
  }

  state = PAUSING;
  Serial.println(F("DECELERATING TO PAUSE"));
}

void resumePump() {
  if (state != PAUSED) {
    Serial.println(F("ERROR: NOT PAUSED"));
    return;
  }

  if (returnRun) {
    if (!returnPathClear()) return;
  } else if (!forwardDosePathClear()) {
    return;
  }

  if (!returnRun && !validRate(flowRate)) {
    Serial.println(F("ERROR: INVALID FLOW RATE"));
    return;
  }

  lastContactMs = millis();
  lastUpdateUs = micros();

  state = RUNNING;

  Serial.println(F("PUMP RESUMED"));
}

void completeDose() {
  disableMotor();
  state = COMPLETE;

  Serial.println(F("TARGET STEPS COMPLETE"));
}

// =====================================================
// NONBLOCKING MOTOR MOTION
// =====================================================

void updateMotion() {
  const bool jogging = state == JOGGING && jogMotorEnabled;
  const bool volumeLimited = !jogging && !returnRun;
  if (!jogging && state != RUNNING && state != PAUSING) {
    return;
  }

  unsigned long now = micros();
  unsigned long elapsedUs = now - lastUpdateUs;

  if (elapsedUs < 150UL) {
    return;
  }

  lastUpdateUs = now;

  // Avoid catch-up bursts following a long delay.
  if (elapsedUs > 20000UL) {
    elapsedUs = 20000UL;
  }

  float dt = elapsedUs / 1000000.0f;

  if (volumeLimited && deliveredSteps >= targetSteps) {
    completeDose();
    return;
  }

  float desiredHz = 0.0f;
  if (jogging) {
    desiredHz = jogDirection == BACKWARD ? limitReverseSpeed(REVERSE_CRUISE_HZ) : JOG_STEP_HZ;
    if (commissionStepsLeft > 0) desiredHz = COMMISSION_STEP_HZ;
  } else if (state == RUNNING) {
    desiredHz = returnRun ? limitReverseSpeed(retractSpeedHz) : rateHz(flowRate);
  }
  if (departingHome) desiredHz = min(desiredHz, HOME_APPROACH_HZ);
  if (volumeLimited) {
    const unsigned long remaining = targetSteps - deliveredSteps;
    const float stoppingHz = sqrtf(2.0f * ACCEL * remaining);
    if (desiredHz > stoppingHz) desiredHz = stoppingHz;
  } else if (jogging && jogDirection == FORWARD && measuredMaxSteps > 0) {
    const unsigned long remaining = remainingForwardSteps();
    if (!positionKnown || remaining == 0) {
      finishJog(F("JOG TRAVEL LIMIT"));
      return;
    }
    const float stoppingHz = sqrtf(2.0f * ACCEL * remaining);
    if (desiredHz > stoppingHz) desiredHz = stoppingHz;
  }

  // Acceleration / deceleration
  float previousHz = currentHz;
  float speedChange = ACCEL * dt;

  if (currentHz < desiredHz) {
    currentHz += speedChange;

    if (currentHz > desiredHz) {
      currentHz = desiredHz;
    }
  }
  else if (currentHz > desiredHz) {
    currentHz -= speedChange;

    if (currentHz < desiredHz) {
      currentHz = desiredHz;
    }
  }

  // Accumulate step demand.
  stepCredit +=
    0.5f * (previousHz + currentHz) * dt;

  if (stepCredit > 1.0f) {
    stepCredit = 1.0f;
  }

  if (
    stepCredit >= 1.0f &&
    (unsigned long)(now - lastStepUs) >= MIN_STEP_US
  ) {
    if (jogging) {
      if (!jogPathClear(jogDirection)) return;
    } else if (returnRun) {
      if (!returnPathClear()) return;
    } else if (!forwardDosePathClear()) {
      return;
    }

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP_PIN, LOW);

    lastStepUs = micros();

    stepCredit -= 1.0f;
    if (departingHome) ++homeDepartureSteps;
    trackTravelStep(jogging ? jogDirection : runDirection);

    if (jogging && commissionStepsLeft > 0 && --commissionStepsLeft == 0) {
      finishJog(F("JOG COMMISSION COMPLETE"));
      return;
    }
    if (!jogging) {
      ++deliveredSteps;
      if (volumeLimited && deliveredSteps >= targetSteps) {
        completeDose();
        return;
      }
    }
  }

  if (
    state == PAUSING &&
    currentHz <= 0.001f
  ) {
    currentHz = 0.0f;
    stepCredit = 0.0f;

    // Driver remains enabled for holding torque.
    state = PAUSED;

    Serial.println(F("PUMP PAUSED"));
  }
}

// =====================================================
// COMMUNICATION WATCHDOG
// =====================================================

void checkWatchdog() {
  if (state == JOGGING) {
    if (millis() - lastJogKeepMs >= JOG_TIMEOUT_MS) timeoutJog();
    else if (jogMotorEnabled && commissionStepsLeft > 0 &&
        millis() - commissionStartedMs >= COMMISSION_DURATION_MS) {
      finishJog(F("JOG COMMISSION COMPLETE"));
    }
    return;
  }

  if (!runActive()) {
    return;
  }

  if (
    (unsigned long)(millis() - lastContactMs)
      > LINK_TIMEOUT_MS
  ) {
    triggerAlarm("COMMUNICATION TIMEOUT");
  }
}

// =====================================================
// STATUS
// =====================================================

const char* stateName() {
  switch (state) {
    case IDLE: return "IDLE";
    case RUNNING: return "RUNNING";
    case PAUSING: return "PAUSING";
    case PAUSED: return "PAUSED";
    case JOGGING: return "JOGGING";
    case COMPLETE: return "COMPLETE";
    case ALARM: return "ALARM";
  }

  return "UNKNOWN";
}

void printStatus() {
  Serial.println(F("------- PUMP STATUS -------"));

  Serial.print(F("STATE: "));
  Serial.println(stateName());
  if (alarmReason) {
    Serial.print(F("ALARM REASON: "));
    Serial.println(alarmReason);
  }

  Serial.print(F("SYRINGE: "));
  Serial.println(syringeSize);

  Serial.print(F("DIRECTION: "));
  Serial.println(
    direction == FORWARD ? F("FORWARD") : F("BACKWARD")
  );

  Serial.println(F("BACKWARD STOP: HOME"));
  if (targetSteps > 0 || returnRun) {
    Serial.print(F("RUN DIRECTION: "));
    Serial.println(runDirection == FORWARD ? F("FORWARD") : F("BACKWARD"));
  }

  Serial.print(F("FLOW RATE: "));
  Serial.println(flowRate, 6);

  Serial.print(F("TARGET VOLUME: "));
  Serial.println(targetVolume, 3);

  // In BACKWARD mode, this represents
  // equivalent carriage travel, not delivered fluid.
  Serial.print(F("ESTIMATED DELIVERED: "));
  Serial.println(deliveredSteps / stepsPerML, 4);

  Serial.print(F("TRAVEL MM: "));
  Serial.println(deliveredSteps / STEPS_PER_MM, 3);

  Serial.print(F("STEPS: "));
  Serial.print(deliveredSteps);
  Serial.print(F(" / "));
  Serial.println(targetSteps);

  Serial.print(F("CURRENT RPM: "));
  Serial.println(
    currentHz * 60.0f / STEPS_PER_REV, 3
  );

  Serial.print(F("MAX FLOW: "));
  Serial.println(maxRate(), 3);
  printVolumeCalibrationStatus(syringeSize, stepsPerML);

  Serial.print(F("JOG SPEED MM S: "));
  Serial.println(JOG_STEP_HZ / STEPS_PER_MM, 2);
  Serial.println(F("COMMISSION JOG: YES"));
  Serial.print(F("RETRACT SPEED MM S: "));
  Serial.println(retractSpeedHz / STEPS_PER_MM, 6);
  Serial.print(F("RETRACT MAX MM S: "));
  Serial.println(REVERSE_CRUISE_HZ / STEPS_PER_MM, 2);
  if (state == JOGGING) {
    Serial.print(F("JOG DIRECTION: "));
    Serial.println(jogDirection == FORWARD ? F("FORWARD") : F("BACKWARD"));
  }

  printTravelStatus();
  printForceStatus();
  Serial.println(F("---------------------------"));
}

// =====================================================
// SERIAL COMMAND PROCESSOR
// =====================================================

void processCommand(char* command) {
  // Trim leading spaces.
  while (
    *command &&
    isspace((unsigned char)*command)
  ) {
    ++command;
  }

  // Uppercase for case-insensitive commands.
  for (char* p = command; *p; ++p) {
    *p = toupper((unsigned char)*p);
  }

  // Separate the command from its argument.
  char* argument = command;

  while (
    *argument &&
    !isspace((unsigned char)*argument)
  ) {
    ++argument;
  }

  if (*argument) {
    *argument++ = '\0';

    while (
      *argument &&
      isspace((unsigned char)*argument)
    ) {
      ++argument;
    }
  }
  else {
    argument = NULL;
  }

  // ---------------- STATUS ----------------

  if (strcmp(command, "STATUS") == 0) {
    lastContactMs = millis();
    printStatus();
  }

  else if (strcmp(command, "FORCE") == 0) {
    processForceCommand(argument, state == IDLE || state == COMPLETE);
  }

  else if (strcmp(command, "TRAVEL") == 0) {
    processTravelCommand(argument);
  }

  else if (strcmp(command, "JOG") == 0) {
    if (argument && strcmp(argument, "FORWARD") == 0) {
      prepareJog(FORWARD, false);
    } else if (argument && strcmp(argument, "BACKWARD") == 0) {
      prepareJog(BACKWARD, false);
    } else if (argument && strcmp(argument, "COMMISSION") == 0) {
      prepareJog(FORWARD, true);
    } else if (argument && strcmp(argument, "COMMISSION KEEP") == 0) {
      keepJog(true);
    } else if (argument && strcmp(argument, "KEEP") == 0) {
      keepJog(false);
    } else if (argument && strcmp(argument, "STOP") == 0) {
      finishJog(F("JOG STOPPED"));
    } else {
      Serial.println(F("ERROR: INVALID JOG COMMAND"));
    }
  }

  // ---------------- PING ----------------

  else if (strcmp(command, "PING") == 0) {
    lastContactMs = millis();
    Serial.println(F("PONG"));
  }

  // ---------------- STOP ----------------

  else if (strcmp(command, "STOP") == 0) {
    stopPump();
  }

  // ---------------- START ----------------

  else if (strcmp(command, "START") == 0) {
    startPump();
  }

  // ---------------- PAUSE ----------------

  else if (strcmp(command, "PAUSE") == 0) {
    lastContactMs = millis();
    pausePump();
  }

  // ---------------- RESUME ----------------

  else if (strcmp(command, "RESUME") == 0) {
    resumePump();
  }

  // ---------------- DIRECTION ----------------

  else if (strcmp(command, "DIRECTION") == 0) {
    if (state != IDLE && state != COMPLETE) {
      Serial.println(F("ERROR: STOP FIRST"));
      return;
    }

    if (
      argument &&
      strcmp(argument, "FORWARD") == 0
    ) {
      direction = FORWARD;
    }

    else if (
      argument &&
      strcmp(argument, "BACKWARD") == 0
    ) {
      direction = BACKWARD;
    }

    else {
      Serial.println(
        F("ERROR: USE FORWARD OR BACKWARD")
      );
      return;
    }

    Serial.println(F("DIRECTION UPDATED"));
  }

  // ---------------- SYRINGE ----------------

  else if (strcmp(command, "SYRINGE") == 0) {
    if (state != IDLE && state != COMPLETE) {
      Serial.println(F("ERROR: STOP FIRST"));
      return;
    }

    if (
      argument &&
      strcmp(argument, "20") == 0
    ) {
      syringeSize = 20;
      boreMM = 20.05f;
    }

    else if (
      argument &&
      strcmp(argument, "50") == 0
    ) {
      syringeSize = 50;
      boreMM = 29.2f;
    }

    else {
      Serial.println(F("ERROR: USE 20 OR 50"));
      return;
    }

    calculateSyringe();

    if (targetVolume > syringeSize) {
      targetVolume = syringeSize;
    }

    deliveredSteps = 0;
    targetSteps = 0;
    returnRun = false;

    Serial.println(F("SYRINGE UPDATED"));
  }

  // ---------------- FLOW RATE ----------------

  else if (strcmp(command, "RATE") == 0) {
    float newRate;

    if (
      !positiveNumber(argument, newRate) ||
      !validRate(newRate)
    ) {
      Serial.print(F("ERROR: MAX FLOW = "));
      Serial.println(maxRate(), 3);
      return;
    }

    // Independent of target volume.
    // May be changed while the motor is running.
    flowRate = newRate;
    lastContactMs = millis();

    Serial.println(F("FLOW RATE UPDATED"));
  }

  else if (strcmp(command, "RETRACT") == 0) {
    if (state != IDLE && state != COMPLETE) {
      Serial.println(F("ERROR: STOP FIRST"));
      return;
    }

    float speedMmPerSecond;
    if (!positiveNumber(argument, speedMmPerSecond)) {
      Serial.println(F("ERROR: INVALID RETRACT SPEED"));
      return;
    }
    const float speedHz = speedMmPerSecond * STEPS_PER_MM;
    if (speedHz < MIN_STEP_HZ || speedHz > REVERSE_CRUISE_HZ) {
      Serial.println(F("ERROR: INVALID RETRACT SPEED"));
      return;
    }

    retractSpeedHz = speedHz;
    Serial.println(F("RETRACT SPEED UPDATED"));
  }

  // ---------------- TARGET VOLUME ----------------

  else if (strcmp(command, "VOLUME") == 0) {
    if (state != IDLE && state != COMPLETE) {
      Serial.println(F("ERROR: STOP FIRST"));
      return;
    }

    const float areaMM2 = PI * boreMM * boreMM / 4.0f;
    const float provisionalStepsPerML = STEPS_PER_MM * 1000.0f / areaMM2;
    if (processVolumeCalibrationCommand(
          argument,
          syringeSize,
          stepsPerML,
          provisionalStepsPerML,
          true
        )) {
      deliveredSteps = 0;
      targetSteps = 0;
      returnRun = false;
      return;
    }

    float newVolume;

    if (
      !positiveNumber(argument, newVolume) ||
      newVolume < 0.01f ||
      newVolume > syringeSize
    ) {
      Serial.println(F("ERROR: INVALID VOLUME"));
      return;
    }

    // Independent of flow rate.
    targetVolume = newVolume;

    Serial.println(F("TARGET VOLUME UPDATED"));
  }

  else {
    Serial.println(F("ERROR: UNKNOWN COMMAND"));
  }
}

// =====================================================
// SERIAL INPUT READER
// =====================================================

void readSerialCommands() {
  byte processed = 0;

  while (
    Serial.available() > 0 &&
    processed++ < 16
  ) {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      if (commandOverflow) {
        commandIndex = 0;
        commandOverflow = false;

        Serial.println(
          F("ERROR: COMMAND TOO LONG")
        );
      }

      else if (commandIndex > 0) {
        commandBuffer[commandIndex] = '\0';
        processCommand(commandBuffer);
        commandIndex = 0;
      }
    }

    else if (!commandOverflow) {
      if (
        commandIndex <
        sizeof(commandBuffer) - 1
      ) {
        commandBuffer[commandIndex++] = c;
      }

      else {
        commandOverflow = true;
      }
    }
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  // Disable driver before configuring outputs.
  digitalWrite(EN_PIN, HIGH);
  pinMode(EN_PIN, OUTPUT);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);

  pinMode(HOME_PIN, INPUT_PULLUP);
  pinMode(MAX_PIN, INPUT_PULLUP);

  Serial.begin(115200);

  beginVolumeCalibration();
  calculateSyringe();
  beginForceSensor();

  lastContactMs = millis();
  lastUpdateUs = micros();

  Serial.println(F("SYRINGE PUMP V4.4 READY"));
  Serial.println(
    F("DIRECTION: FORWARD / BACKWARD")
  );
  Serial.println(F("WATCHDOG: 15 SECONDS"));

  if (bothLimitsPressed()) {
    triggerAlarm("BOTH LIMIT SWITCHES ACTIVE");
  }
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  updateHomeRelease();
  if (state == JOGGING) {
    jogPathClear(jogDirection);
  } else if (runActive()) {
    if (returnRun) returnPathClear();
    else forwardDosePathClear();
  } else if (state != ALARM && bothLimitsPressed()) {
    triggerAlarm("BOTH LIMIT SWITCHES ACTIVE");
  }

  checkWatchdog();
  const bool active = runActive();
  const bool commissioning = state == JOGGING && jogMotorEnabled && commissionStepsLeft > 0;
  updateForceSensor(active || commissioning);
  const bool forwardJog = state == JOGGING && jogMotorEnabled && jogDirection == FORWARD;
  if ((active && runDirection == FORWARD) || forwardJog) {
    if (const char* fault = forceFault(!commissioning)) {
      triggerAlarm(fault);
    }
  }
  updateMotion();
  readSerialCommands();
}
