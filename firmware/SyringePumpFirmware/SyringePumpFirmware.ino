#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "ForceSensor.h"

// =====================================================
// SYRINGE PUMP V3.4
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

const unsigned long MIN_STEP_US = 1000UL;
const unsigned long LINK_TIMEOUT_MS = 15000UL;

// Syringe settings — provisional dimensions
int syringeSize = 20;
float boreMM = 20.05f;
float stepsPerML = 0.0f;

// Independent user settings
float flowRate = 1.0f;       // mL/min
float targetVolume = 1.0f;   // mL-equivalent travel

enum PumpState {
  IDLE,
  RUNNING,
  PAUSING,
  PAUSED,
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
  stepsPerML = STEPS_PER_MM * 1000.0f / areaMM2;
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

bool limitPressed() {
  return digitalRead(HOME_PIN) == HIGH ||
         digitalRead(MAX_PIN) == HIGH;
}

// =====================================================
// MOTOR SAFETY
// =====================================================

void disableMotor() {
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(EN_PIN, HIGH);

  currentHz = 0.0f;
  stepCredit = 0.0f;
}

void triggerAlarm(const char* reason) {
  if (state == ALARM) {
    return;
  }

  disableMotor();
  state = ALARM;

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

void startPump() {
  if (state == ALARM) {
    Serial.println(F("ERROR: RESET REQUIRED"));
    return;
  }

  if (state != IDLE && state != COMPLETE) {
    Serial.println(F("ERROR: STOP FIRST"));
    return;
  }

  if (limitPressed()) {
    triggerAlarm("LIMIT SWITCH");
    return;
  }

  if (!validRate(flowRate)) {
    Serial.println(F("ERROR: INVALID FLOW RATE"));
    return;
  }

  float requestedSteps = targetVolume * stepsPerML;

  if (!(requestedSteps >= 1.0f &&
        requestedSteps <= 4000000000.0f)) {
    Serial.println(F("ERROR: INVALID TARGET"));
    return;
  }

  targetSteps = (unsigned long)(requestedSteps + 0.5f);
  deliveredSteps = 0;
  runDirection = direction;

  currentHz = 0.0f;
  stepCredit = 0.0f;

  digitalWrite(STEP_PIN, LOW);

  // Direction LOW selects forward and HIGH selects backward.
  digitalWrite(
  DIR_PIN,
  direction == FORWARD ? LOW : HIGH
);

  delayMicroseconds(5);

  // Enable A4988 (active LOW)
  updateForceSensor(false);
  digitalWrite(EN_PIN, LOW);

  lastUpdateUs = micros();
  lastStepUs = lastUpdateUs;
  lastContactMs = millis();

  state = RUNNING;

  Serial.println(F("PUMP STARTED"));
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

  if (limitPressed()) {
    triggerAlarm("LIMIT SWITCH");
    return;
  }

  if (!validRate(flowRate)) {
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
  if (state != RUNNING && state != PAUSING) {
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

  if (deliveredSteps >= targetSteps) {
    completeDose();
    return;
  }

  unsigned long remaining =
    targetSteps - deliveredSteps;

  float desiredHz =
    state == RUNNING ? rateHz(flowRate) : 0.0f;

  // Deceleration near the target.
  float stoppingHz =
    sqrtf(2.0f * ACCEL * remaining);

  if (desiredHz > stoppingHz) {
    desiredHz = stoppingHz;
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
    if (limitPressed()) {
      triggerAlarm("LIMIT SWITCH");
      return;
    }

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP_PIN, LOW);

    lastStepUs = micros();

    ++deliveredSteps;
    stepCredit -= 1.0f;

    if (deliveredSteps >= targetSteps) {
      completeDose();
      return;
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
  bool active =
    state == RUNNING ||
    state == PAUSING ||
    state == PAUSED;

  if (!active) {
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
    case COMPLETE: return "COMPLETE";
    case ALARM: return "ALARM";
  }

  return "UNKNOWN";
}

void printStatus() {
  Serial.println(F("------- PUMP STATUS -------"));

  Serial.print(F("STATE: "));
  Serial.println(stateName());

  Serial.print(F("SYRINGE: "));
  Serial.println(syringeSize);

  Serial.print(F("DIRECTION: "));
  Serial.println(
    direction == FORWARD ? F("FORWARD") : F("BACKWARD")
  );

  if (targetSteps > 0) {
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
    if (argument && (strcmp(argument, "RAW") == 0 ||
                     strcmp(argument, "STATUS") == 0)) {
      printForceStatus();
    } else {
      Serial.println(F("ERROR: USE FORCE RAW OR FORCE STATUS"));
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

  // ---------------- TARGET VOLUME ----------------

  else if (strcmp(command, "VOLUME") == 0) {
    if (state != IDLE && state != COMPLETE) {
      Serial.println(F("ERROR: STOP FIRST"));
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

  calculateSyringe();
  beginForceSensor();

  lastContactMs = millis();
  lastUpdateUs = micros();

  Serial.println(F("SYRINGE PUMP V3.4 READY"));
  Serial.println(
    F("DIRECTION: FORWARD / BACKWARD")
  );
  Serial.println(F("WATCHDOG: 15 SECONDS"));

  if (limitPressed()) {
    triggerAlarm("LIMIT SWITCH");
  }
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  // Both endstops stop either direction.
  if (state != ALARM && limitPressed()) {
    triggerAlarm("LIMIT SWITCH");
  }

  checkWatchdog();
  updateMotion();
  // The HX711 transfer masks interrupts on AVR. Sample with the driver disabled.
  updateForceSensor(state == IDLE || state == COMPLETE || state == ALARM);
  readSerialCommands();
}
