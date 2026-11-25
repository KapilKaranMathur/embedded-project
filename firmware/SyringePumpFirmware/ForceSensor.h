#pragma once

void beginForceSensor();
void updateForceSensor(bool trackPeak);
void processForceCommand(char* argument, bool canConfigure);
bool forceLimitConfigured();
const char* forceFault();
void resetForcePeak();
void printForceStatus();
