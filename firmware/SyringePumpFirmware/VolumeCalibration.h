#pragma once

void beginVolumeCalibration();
float applySavedVolumeCalibration(int syringeSize, float provisionalStepsPerML);
bool processVolumeCalibrationCommand(
  char* argument,
  int syringeSize,
  float& stepsPerML,
  float provisionalStepsPerML,
  bool canConfigure
);
void printVolumeCalibrationStatus(int syringeSize, float stepsPerML);
