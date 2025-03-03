#include "include_all.h"
void init_PID() {
  //initialize the variables we're linked to
  PIDInput = -2000;
  PIDSetpoint = 0;

  myPID.SetMode(AUTOMATIC);
}

bool PID_dco_calibration() {

  PIDInput = (double)constrain(DCO_calibration_difference, -1500, 1500);

  double PIDgap = abs(PIDSetpoint - PIDInput);  //distance away from setpoint

  if (micros() - currentNoteCalibrationStart > 15000000 && ampCompCalibrationVal > (DIV_COUNTER * 0.98)) {
    Serial.println("Find highest freq");
    Serial.println((String) "Total time: " + (millis() - DCOCalibrationStart));
    calibrationFlag = false;
    DCO_calibration_difference = 2;
    return true;

  } else if (micros() - currentNoteCalibrationStart > 20000000 && micros() - PIDComputeTimer > sampleTime) {
    PIDMinGap = PIDMinGap * 1.02;
  } else if (micros() - currentNoteCalibrationStart > 10000000 && micros() - PIDComputeTimer > sampleTime) {
    PIDMinGap = PIDMinGap * 1.01;
  }

  if (PIDgap < bestGap) {
    bestGap = PIDgap;
    bestCandidate = ampCompCalibrationVal;
  }

  bool calibrationSwing = false;
  if ((DCO_calibration_difference < 0 && lastDCODifference > 0) || (DCO_calibration_difference > 0 && lastDCODifference < 0)) {
    lastGapFlipCount++;
    if (lastGapFlipCount >= 4) {
      if (DCO_calibration_current_note > 50) {
        PIDMinGap = PIDMinGap * 1.01;
      } else {
        PIDMinGap = PIDMinGap * 1.05;
      }
    }
    if (lastGapFlipCount >= 8) {
      Serial.println("*********************/*/*/*/*/*/  FLIP !!! *********************/*/*/*/*/*/*********");

      calibrationSwing = true;
    }
  } else {
    lastGapFlipCount = 0;
  }

  if (PIDgap < PIDMinGap || calibrationSwing == true) {

    PIDMinGapCounter++;

    if (PIDMinGapCounter >= 2) {
      if (autotuneDebug >= 1) {
        Serial.println((String)(String) " - Gap = " + PIDgap + " - MIN Gap: " + (PIDMinGap) + (String) " - " + DCO_calibration_current_note);
      }

      calibrationData[arrayPos] = (uint32_t)(sNotePitches[DCO_calibration_current_note - 12] * 100);
      calibrationData[arrayPos + 1] = (uint32_t)bestCandidate;  //bestCandidate;
      arrayPos += 2;

      Serial.println((uint32_t)(sNotePitches[DCO_calibration_current_note - 12] * 100) + (String) ", " + ampCompCalibrationVal + (String) ",");
      Serial.println((String) "Final gap = " + PIDgap + (String) " |||| NOTE: " + DCO_calibration_current_note + (String) " |||| Note calibration time(s): " + ((micros() - currentNoteCalibrationStart) / 1000000));


      DCO_calibration_current_note = DCO_calibration_current_note + calibration_note_interval;
      VOICE_NOTES[0] = DCO_calibration_current_note;

      currentNoteCalibrationStart = micros();
      //PIDMinGap = (1240.6114554 * pow(0.9924189, (double)ampCompCalibrationVal)) * 0.05;

      PIDMinGap = (37701.182837 * pow(0.855327, (double)DCO_calibration_current_note));

      PIDLimitsFormula = ((1.364 * (double)(ampCompCalibrationVal)) - 12) * 1.05;
      PIDOutputLowerLimit = PIDLimitsFormula * 0.8;
      PIDOutputHigherLimit = PIDLimitsFormula * 1.05;

      Serial.println((String) "Next MinGap: " + PIDMinGap);
      Serial.println((String) "PIDOutputLowerLimit: " + PIDOutputLowerLimit + (String) " PIDOutputHigherLimit: " + PIDOutputHigherLimit);
      Serial.println("  ----------------------------------------------------------------- ");

      if (PIDOutputHigherLimit >= (DIV_COUNTER * 0.98)) {
        Serial.println("Find highest freq");
        Serial.println((String) "Total time: " + ((millis() - DCOCalibrationStart) / 1000));
        calibrationFlag = false;
        DCO_calibration_difference = 2;
        return true;
      }

      ampCompCalibrationVal = PIDLimitsFormula;

      sampleTime = (1000000 / sNotePitches[DCO_calibration_current_note - 12]) * ((samplesNumber - 1) / 2);
      if (sampleTime < 8000) sampleTime = 8000;

      voice_task_autotune(0, ampCompCalibrationVal);
      delay(50);

      DCO_calibration_lastTime = micros();
      DCO_calibration_difference = 4000;
      lastDCODifference = 50000;
      lastGapFlipCount = 0;
      lastPIDgap = 50000;
      bestGap = 50000;
      bestCandidate = 50000;
      lastampCompCalibrationVal = 0;
      DCO_calibration_lastTime = 0;
      PIDMinGapCounter = 0;

      return false;
    }
  }

  if (autotuneDebug >= 1) {
    Serial.println((String) " - GAP = " + PIDgap + " - MIN GAP: " + (PIDMinGap) + (String) " -  NOTE: " + DCO_calibration_current_note);
  }

  lastDCODifference = DCO_calibration_difference;
  lastPIDgap = PIDgap;
  lastampCompCalibrationVal = ampCompCalibrationVal;

  if (DCO_calibration_difference > 0.00) {
    if (DCO_calibration_difference > PIDMinGap * 20) {
      ampCompCalibrationVal += 2;
    } else {
      ampCompCalibrationVal++;
    }
  } else if (DCO_calibration_difference < 0.00) {
    if (abs(DCO_calibration_difference) > PIDMinGap * 20) {
      ampCompCalibrationVal -= 2;
    } else {
      ampCompCalibrationVal--;
    }
  }

  //ampCompCalibrationVal = constrain(ampCompCalibrationVal, PIDOutputLowerLimit, PIDOutputHigherLimit);

  if (autotuneDebug >= 1) {
    Serial.println((String) "ampCompCalibrationVal: " + ampCompCalibrationVal + (String) " -- PIDOutputLowerLimit: " + PIDOutputLowerLimit);
    Serial.print((String) " PIDOutputHigherLimit: " + PIDOutputHigherLimit + (String) " - DCO: " + currentDCO);
  }
  return false;
}

void PID_find_highest_freq() {

  ampCompCalibrationVal = DIV_COUNTER;
  PIDTuningMultiplier = 0.28752775 * pow(1.00408722, 1779);
  PIDTuningMultiplierKi = 0.33936558 * pow(1.00702176, 1779);
  PIDInput = 100;
  myPID.SetOutputLimits(sNotePitches[DCO_calibration_current_note - 12 - calibration_note_interval], sNotePitches[DCO_calibration_current_note - 12 + calibration_note_interval]);
  myPID.SetTunings(0.01, 1, 0.0005);
  myPID.SetSampleTime(5);
  while (abs(DCO_calibration_difference) > 0.5) {
    voice_task_autotune(1, DIV_COUNTER);
    delay(4);
    find_gap(0);
    PIDInput = 0 - (double)DCO_calibration_difference;

    myPID.Compute();

    if (autotuneDebug >= 1) {
      Serial.println((String) "Pid output: " + PIDOutput + (String) " Pid gap: " + DCO_calibration_difference);
    }
  }
  Serial.println((String) "Highest freq found: " + PIDOutput);

  //find highest note
  for (int i = 0; i < sizeof(sNotePitches); i++) {
    if (PIDOutput > sNotePitches[i] && PIDOutput < sNotePitches[i + 1]) {
      highestNoteOSC[currentDCO] = i;
      Serial.println((String) "Highest note found: " + i + (String) " - Note freq: " + sNotePitches[i]);
      break;
    }
  }
}

float find_highest_freq() {

  ampCompCalibrationVal = DIV_COUNTER;
  PIDTuningMultiplier = 0.28752775 * pow(1.00408722, 1779);
  PIDTuningMultiplierKi = 0.33936558 * pow(1.00702176, 1779);
  PIDInput = 100;
  myPID.SetOutputLimits(sNotePitches[DCO_calibration_current_note - 12 - calibration_note_interval], sNotePitches[DCO_calibration_current_note - 12 + calibration_note_interval]);
  myPID.SetTunings(0.01, 1.2, 0.002);
  myPID.SetSampleTime(5);

  while (abs(DCO_calibration_difference) > 0.5) {
    voice_task_autotune(4, DIV_COUNTER);
    delay(4);
    find_gap(0);
    PIDInput = 0 - (double)DCO_calibration_difference;

    myPID.Compute();

    if (autotuneDebug >= 1) {
      Serial.println((String) "Pid output: " + PIDOutput + (String) " Pid gap: " + DCO_calibration_difference);
    }
  }
  Serial.println((String) "Highest freq found: " + PIDOutput);

  //find highest note
  for (int i = 0; i < sizeof(sNotePitches); i++) {
    if (PIDOutput > sNotePitches[i] && PIDOutput < sNotePitches[i + 1]) {
      highestNoteOSC[currentDCO] = i;
      Serial.println((String) "Highest note found: " + i + (String) " - Note freq: " + sNotePitches[i]);
      break;
    }
  }

  return PIDOutput * 100;
}

float find_lowest_freq() {

  DCO_calibration_difference = 1000;

  ampCompCalibrationVal = ampCompLowestFreqVal;
  PIDTuningMultiplier = 0.28752775 * pow(1.00408722, 1779);
  PIDTuningMultiplierKi = 0.33936558 * pow(1.00702176, 1779);
  PIDInput = 100;

  double PIDLowerLimit = 2;
  double PIDUpperLimit = sNotePitches[manual_DCO_calibration_start_note ];
  uint16_t targetGap = 100;

  myPID.SetOutputLimits(PIDLowerLimit, PIDLowerLimit);
  myPID.SetTunings(0.000004, 0.000007, 0.0000001);
  myPID.SetSampleTime(300);

  int validValuesArraySize = 50;
  double gapArray[validValuesArraySize];
  double validValuesArray[validValuesArraySize];

  double newLowerLimit = PIDLowerLimit;
  double newUpperLimit = PIDUpperLimit;

  double closestToZeroNegativeGap = -1000000;
  double closestToZeroPositiveGap = 1000000;
  double closestToZero = 1000000;
  double bestGap = 1000000;

  bool lowestFreqFound = false;

  voice_task_autotune(4, PIDLowerLimit);
  delay(300);

  while (abs(bestGap) > targetGap) {
    if (lowestFreqFound) {
      break;
    }
    for (int i = 0; i < validValuesArraySize; i++) {

      PIDOutput = (double)newLowerLimit + ((double)i * (double)((newUpperLimit - newLowerLimit) / (double)validValuesArraySize));
      validValuesArray[i] = PIDOutput;

      voice_task_autotune(4, PIDOutput);
      delay(150);
      gapArray[i] = (double)find_gap(2);

      Serial.println((String) "validValuesArray: " + validValuesArray[i]);
      Serial.println((String) "gapArray: " + gapArray[i]);
      Serial.println((String) "PIDOutput: " + PIDOutput);
      Serial.println((String) "i: " + i);



      if (gapArray[i] != 1.16999f) {
        if (abs(gapArray[i]) < targetGap) {
          closestToZero = PIDOutput;
          lowestFreqFound = true;
          break;
        }
        if (gapArray[i] < 0 && closestToZeroNegativeGap < gapArray[i]) {
          closestToZeroNegativeGap = gapArray[i];
          newLowerLimit = validValuesArray[i] - (validValuesArray[i] * 0.02);
        }
        if (gapArray[i] > 0 && gapArray[i] < closestToZeroPositiveGap) {
          closestToZeroPositiveGap = gapArray[i];
          newUpperLimit = validValuesArray[i] + (validValuesArray[i] * 0.02);
        }

        if (abs(newUpperLimit) < abs(newLowerLimit)) {
          closestToZero = newUpperLimit;
        } else {
          closestToZero = newLowerLimit;
        }

        if (abs(closestToZeroNegativeGap) < abs(closestToZeroPositiveGap)) {
          bestGap = closestToZeroNegativeGap;
        } else {
          bestGap = closestToZeroPositiveGap;
        }
      }
    }

    Serial.println((String) "newLowerLimit: " + newLowerLimit);
    Serial.println((String) "newUpperLimit: " + newUpperLimit);

    myPID.SetOutputLimits(newLowerLimit, newUpperLimit);
    PIDInput = (double)-100;
    PIDOutput = (double)newUpperLimit - (double)newLowerLimit;
  }
  Serial.println((String) "Lowest freq found: " + PIDOutput);
  return closestToZero * 100;
}

void calibrateOscillator() {

  double tolerance;      // minGap
  uint16_t minAmpComp;   // Lower Limit
  uint16_t maxAmpComp;   //  Higher Limit
  int rangeSamples = 2;  // Number of measurements to store around the sign change
  const int numPresetVoltages = chanLevelVoiceDataSize;

  for (int j = 4; j < numPresetVoltages; j += 2) {  // Start from the 3rd preset voltage
    uint16_t currentAmpCompCalibrationVal;

    DCO_calibration_current_note = DCO_calibration_start_note + (calibration_note_interval * (j - 4) / 2);
    VOICE_NOTES[0] = DCO_calibration_current_note;
    if (j == 4) {
      currentAmpCompCalibrationVal = (initManualAmpCompCalibrationVal[currentDCO] + manualCalibrationOffset[currentDCO])* 1.35;
    } else if (j == 6) {
      currentAmpCompCalibrationVal = logarithmicInterpolation(calibrationData[2], calibrationData[3], calibrationData[4], calibrationData[5], sNotePitches[DCO_calibration_current_note - 12] * 100);
    } else {
      currentAmpCompCalibrationVal = quadraticInterpolation(calibrationData[j - 6], calibrationData[j - 5], calibrationData[j - 4], calibrationData[j - 3], calibrationData[j - 2], calibrationData[j - 1], sNotePitches[DCO_calibration_current_note - 12] * 100);
    }

    if (currentAmpCompCalibrationVal > DIV_COUNTER * 0.98) {
      float highestFreqFound = find_highest_freq();
      calibrationData[j] = highestFreqFound;
      calibrationData[j + 1] = DIV_COUNTER;

      for (int i = j + 2; i < numPresetVoltages; i += 2) {
        calibrationData[i] = 20000000;
        calibrationData[i + 1] = DIV_COUNTER;
      }
      break;
    }

    uint16_t minAmpComp = currentAmpCompCalibrationVal * 0.8;  // Lower Limit
    uint16_t maxAmpComp = currentAmpCompCalibrationVal * 1.3;  //  Higher Limit

    //tolerance = (double)(37701.182837 * pow(0.855327, (double)DCO_calibration_current_note));
    tolerance = (double)1000000.00 / (double)sNotePitches[VOICE_NOTES[0] - 12] /  (double)sNotePitches[VOICE_NOTES[0] - 12] / 4.00d;

    Serial.println((String) "Current DCO: " + currentDCO);
    Serial.println((String) "Calibration note: " + VOICE_NOTES[0]);
    Serial.println((String) "Calibration note freq: " + sNotePitches[VOICE_NOTES[0] - 12]);
    Serial.println((String) "Calibration note amplitude: " + currentAmpCompCalibrationVal);
    Serial.println((String) "Tolerance: " + tolerance);
    Serial.println((String) "MinAmpComp: " + minAmpComp);
    Serial.println((String) "MaxAmpComp: " + maxAmpComp);

    voice_task_autotune(0, currentAmpCompCalibrationVal);  // Send the preset voltage
    delay(10);

    uint16_t bestAmpComp = currentAmpCompCalibrationVal;
    float closestToZero = 50000;  // Initialize with a large value
    float previousAvgValue = 0.0;

    float lowerMeasurements[rangeSamples];
    float higherMeasurements[rangeSamples];
    uint16_t lowerVoltages[rangeSamples];
    uint16_t higherVoltages[rangeSamples];

    int flipCounter = 0;

    while (true) {
      voice_task_autotune(0, currentAmpCompCalibrationVal);
      delay(10);
      float avgValue = find_gap(0);

      if (abs(avgValue) < abs(closestToZero && avgValue != 1.16999f)) {
        closestToZero = avgValue;
        bestAmpComp = currentAmpCompCalibrationVal;
      } else {
        avgValue == 0;
      }

      // Detect sign change
      if ((previousAvgValue > 0 && avgValue < 0) || (previousAvgValue < 0 && avgValue > 0) ) {
        // Store measurements around the current voltage
        for (int i = 0; i < rangeSamples; i++) {
          float lowerVoltage = currentAmpCompCalibrationVal - (i + 1);
          float higherVoltage = currentAmpCompCalibrationVal + (i + 1);

          voice_task_autotune(0, lowerVoltage);
          lowerMeasurements[i] = find_gap(0);
          lowerVoltages[i] = lowerVoltage;

          voice_task_autotune(0, higherVoltage);
          higherMeasurements[i] = find_gap(0);
          higherVoltages[i] = higherVoltage;
        }

        // Evaluate stored measurements including the current voltage
        for (int i = 0; i < rangeSamples; i++) {
          if (abs(lowerMeasurements[i]) < abs(closestToZero)) {
            closestToZero = lowerMeasurements[i];
            bestAmpComp = lowerVoltages[i];
          }
          if (abs(higherMeasurements[i]) < abs(closestToZero)) {
            closestToZero = higherMeasurements[i];
            bestAmpComp = higherVoltages[i];
          }
        }

        // Check the current voltage again
        if (abs(avgValue) < abs(closestToZero)) {
          closestToZero = avgValue;
          bestAmpComp = currentAmpCompCalibrationVal;
        }

        // Break the loop if the closest value is within tolerance
        if (abs(closestToZero) <= tolerance) {
          break;
        } else {
          tolerance = tolerance * 1.2;
        }
        flipCounter++;
        if (flipCounter >= 3 && abs(closestToZero) <= tolerance * 2) {
          break;
        } else {
          tolerance = tolerance * 1.5;
        }
      }

      // Adjust the voltage based on the measurement
      if (abs(avgValue) < tolerance * 20) {
        if (avgValue > 0) {
          currentAmpCompCalibrationVal += 1;
        } else {
          currentAmpCompCalibrationVal -= 1;
        }
      } else {
        if (avgValue > 0) {
          currentAmpCompCalibrationVal += 2;
        } else {
          currentAmpCompCalibrationVal -= 2;
        }
      }

      // Ensure the voltage stays within the allowed range
      if (currentAmpCompCalibrationVal < minAmpComp || currentAmpCompCalibrationVal > maxAmpComp) {
        Serial.println((String) "Calibration voltage out of range: " + currentAmpCompCalibrationVal);
      }

      previousAvgValue = avgValue;
    }

    calibrationData[j] = sNotePitches[DCO_calibration_current_note - 12] * 100;
    calibrationData[j + 1] = bestAmpComp;  // Store the best voltage for the current preset voltage

    Serial.print("DCO_calibration_current_note ");
    Serial.println(DCO_calibration_current_note);
    Serial.print("Best calibration voltage: ");
    Serial.println(bestAmpComp);
    Serial.print("Closest measurement to zero: ");
    Serial.println(closestToZero);
  }
}


float quadraticInterpolation(float x0, float y0, float x1, float y1, float x2, float y2, float x) {
  // Calculate the coefficients of the quadratic polynomial
  float a = ((y2 - (x2 * (y1 - y0) + x1 * y0 - x0 * y1) / (x1 - x0)) / (x2 * (x2 - x0 - x1) + x0 * x1));
  float b = ((y1 - y0) / (x1 - x0) - a * (x0 + x1));
  float c = y0 - x0 * (b + a * x0);

  // Use the polynomial to estimate the next value
  return a * x * x + b * x + c;
}

uint16_t exponentialInterpolation(float x0, float y0, float x1, float y1, float x) {
  // Ensure y0 and y1 are not zero to avoid log(0)
  if (y0 <= 0 || y1 <= 0) {
    return 0;  // or handle the error as needed
  }

  // Calculate the constants a and b
  float b = log(y1 / y0) / (x1 - x0);
  float a = y0 / exp(b * x0);

  // Calculate the y value at the given x
  float y = a * exp(b * x);

  return round(y);
}

uint16_t logarithmicInterpolation(float x0, float y0, float x1, float y1, float x) {
  // Ensure x0 and x1 are not zero or negative to avoid log(0) or log of negative number
  if (x0 <= 0 || x1 <= 0) {
    return 0;  // or handle the error as needed
  }

  // Calculate the constants a and b
  float a = (y1 - y0) / (log(x1) - log(x0));
  float b = y0 - a * log(x0);

  // Calculate the y value at the given x
  float y = a * log(x) + b;

  return (uint16_t)round(y);
}

float logarithmicInterpolationFloat(float x0, float y0, float x1, float y1, float x) {
  // Ensure x0 and x1 are not zero or negative to avoid log(0) or log of negative number
  if (x0 <= 0 || x1 <= 0) {
    return 0;  // or handle the error as needed
  }

  // Calculate the constants a and b
  float a = (y1 - y0) / (logf(x1) - logf(x0));
  float b = y0 - a * logf(x0);

  // Calculate the y value at the given x
  float y = a * logf(x) + b;

  return y;
}

double logarithmicInterpolationDouble(double x0, double y0, double x1, double y1, double x) {
  // Ensure x0 and x1 are not zero or negative to avoid log(0) or log of negative number
  if (x0 <= 0 || x1 <= 0) {
    return 0;  // or handle the error as needed
  }

  // Calculate the constants a and b
  double a = (y1 - y0) / (logf(x1) - logf(x0));
  double b = y0 - a * logf(x0);

  // Calculate the y value at the given x
  double y = a * logf(x) + b;

  return y;
}

float linearInterpolation(float x0, float y0, float x1, float y1, float x) {
  // Ensure x0 and x1 are not the same to avoid division by zero
  if (x0 == x1) {
    return 0;  // or handle the error as needed
  }

  // Calculate the slope (m) of the line
  float m = (y1 - y0) / (x1 - x0);

  // Calculate the y-intercept (b) of the line
  float b = y0 - m * x0;

  // Calculate the y value at the given x
  float y = m * x + b;

  return y;
}

double expInterpolationSolveY(double x, double x0, double x1, double y0, double y1) {
    if (x0 <= 0 || x1 <= 0) {
        // Handle error: x0 and x1 must be greater than 0 for exponential interpolation
        return NAN;
    }

    double log_y0 = log(y0);
    double log_y1 = log(y1);

    double log_y = log_y0 + (log_y1 - log_y0) * (x - x0) / (x1 - x0);

    return exp(log_y);
}