#include "include_all.h"

// Compute allowed |gap| (in microseconds) for a given frequency (Hz) and
// duty-cycle error fraction (e.g. 0.005 = 0.5% duty error).
static double compute_gap_tolerance_for_freq(double freqHz, double dutyErrorFraction) {
  if (freqHz <= 0.0) {
    return 1e6;  // Very loose tolerance if frequency is invalid.
  }
  double periodUs = 1e6 / freqHz;                         // Wave period in microseconds.
  double toleranceUs = 2.0 * dutyErrorFraction * periodUs;  // From |gap| <= 2 * ε * T.
  return toleranceUs;
}

// Return true if the two values have opposite signs (simple sign change test).
// Used by calibrate_DCO() to detect when the duty-cycle error has crossed
// through zero between successive measurements (indicating we've passed the
// ideal PWM point and should probe neighbours more carefully).
static bool did_sign_change(float previous, float current) {
  return (previous > 0.0f && current < 0.0f) ||
         (previous < 0.0f && current > 0.0f);
}

// Helper: set the current DCO amplitude, wait for the waveform to settle,
// and return the measured duty-cycle gap (or timeout sentinel value).
// This centralizes the "write PWM, delay, measure gap" pattern so that
// calibrate_DCO() stays focused on the search logic instead of timing details.
static float measure_gap_for_amp(uint16_t ampPwm) {
  voice_task_autotune(0, ampPwm);
  delay(10);
  GapMeasurement gm = measure_gap(0);
  return gm.value;
}

// Helper: evaluate neighbour measurements (lower/higher) around the current
// voltage and update closestToZero / bestAmpComp if any of them are better.
// The caller passes in the measurements taken one step below and above the
// current PWM value; this routine picks the best candidate among those and
// the current PWM, based purely on closeness of the duty error to zero.
static void update_best_from_neighbours(
  int rangeSamples,
  const float* lowerMeasurements,
  const uint16_t* lowerVoltages,
  const float* higherMeasurements,
  const uint16_t* higherVoltages,
  float avgValue,
  float& closestToZero,
  uint16_t& bestAmpComp,
  uint16_t currentAmpCompCalibrationVal
) {
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
}

// Helper: update the calibration PWM based on the current error and tolerance.
// For large errors we step the PWM in units of 2; once we get close to the
// target (within tolerance * 20) we only step by 1 to avoid overshooting.
static void step_amp_from_error(float avgValue, double tolerance, uint16_t& currentAmpCompCalibrationVal) {
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
}

// Helper: compute the initial amplitude (range PWM) guess for a given table
// index j and note, using the same interpolation strategy as the original code:
//  - j == 4: manual preset scaled by 1.35,
//  - j == 6: logarithmic interpolation between the first two entries,
//  - else : quadratic interpolation based on the previous three calibration points.
static uint16_t compute_initial_amp_for_note(
  const DCOCalibrationContext& ctx,
  int j
) {
  if (j == 4) {
    return (ctx.initManualAmpByOsc[ctx.dcoIndex] + ctx.manualOffsetByOsc[ctx.dcoIndex]) * 1.35;
  } else if (j == 6) {
    return logarithmicInterpolation(
      ctx.calibrationData[2],
      ctx.calibrationData[3],
      ctx.calibrationData[4],
      ctx.calibrationData[5],
      sNotePitches[ctx.currentNote - 12] * 100
    );
  } else {
    return quadraticInterpolation(
      ctx.calibrationData[j - 6],
      ctx.calibrationData[j - 5],
      ctx.calibrationData[j - 4],
      ctx.calibrationData[j - 3],
      ctx.calibrationData[j - 2],
      ctx.calibrationData[j - 1],
      sNotePitches[ctx.currentNote - 12] * 100
    );
  }
}

// Helper: store the final calibration pair for the current note into the
// calibration table and print a short summary to Serial.
static void store_note_result(
  DCOCalibrationContext& ctx,
  int j,
  uint16_t bestAmpComp,
  float closestToZero
) {
  ctx.calibrationData[j]     = sNotePitches[ctx.currentNote - 12] * 100;
  ctx.calibrationData[j + 1] = bestAmpComp;

  Serial.print("DCO_calibration_current_note ");
  Serial.println(ctx.currentNote);
  Serial.print("Best calibration voltage: ");
  Serial.println(bestAmpComp);
  Serial.print("Closest measurement to zero: ");
  Serial.println(closestToZero);
}

// Initialize the global PID controller used by some legacy calibration
// routines. Newer code relies more on explicit search than on PID_v1.
void init_PID() {
  //initialize the variables we're linked to
  PIDInput = -2000;
  PIDSetpoint = 0;

  myPID.SetMode(AUTOMATIC);
}

// LEGACY: Note-by-note DCO calibration loop driven by PID_v1.
// Superseded by calibrate_DCO() and currently not used in the main flow.
// Left here for reference and potential future experiments.
#if 0  // LEGACY_PID_DCO_CALIBRATION
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

      DCO_calibration_difference = 4000;
      if (DCO_calibration_current_note == 12) DCO_calibration_difference = 3000;
      


      lastDCODifference = 50000;
      lastGapFlipCount = 0;
      lastPIDgap = 50000;
      bestGap = 50000;
      bestCandidate = 50000;
      lastampCompCalibrationVal = 0;
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

#if 0  // LEGACY_PID_FIND_HIGHEST_FREQ (unused helper)
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
#endif  // LEGACY_PID_FIND_HIGHEST_FREQ
#endif  // LEGACY_PID_DCO_CALIBRATION

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

#if 0  // LEGACY_FIND_LOWEST_FREQ (unused helper)
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
      // Use structured gap measurement to detect timeouts more clearly.
      GapMeasurement gm = measure_gap(2);
      gapArray[i] = (double)gm.value;

      Serial.println((String) "validValuesArray: " + validValuesArray[i]);
      Serial.println((String) "gapArray: " + gapArray[i]);
      Serial.println((String) "PIDOutput: " + PIDOutput);
      Serial.println((String) "i: " + i);

      if (!gm.timedOut) {
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
#endif  // LEGACY_FIND_LOWEST_FREQ

// Build the [frequency -> amplitude PWM] calibration table for the DCO in ctx.
// For each calibration note it:
//  - Picks an initial PWM guess (via interpolation),
//  - Searches locally for the PWM that makes the duty error closest to zero,
//  - Stores the best PWM together with the note frequency in ctx.calibrationData.
// dutyErrorFraction controls how much duty-cycle error (e.g. 0.005 = 0.5%)
// is tolerated before the search stops for each note.
void calibrate_DCO(DCOCalibrationContext& ctx, double dutyErrorFraction) {

  double tolerance;      // Allowed absolute duty error (in microseconds) for a given note.
  uint16_t minAmpComp;   // Lower bound for the PWM search around the initial guess.
  uint16_t maxAmpComp;   // Upper bound for the PWM search around the initial guess.
  int rangeSamples = 2;  // Number of neighbour voltages to probe around a sign change.
  const int numPresetVoltages = chanLevelVoiceDataSize;  // Size of the [freq, pwm] table.

  for (int j = 4; j < numPresetVoltages; j += 2) {  // Start from the 3rd preset voltage
    uint16_t currentAmpCompCalibrationVal;

    ctx.currentNote = DCO_calibration_start_note + (calibration_note_interval * (j - 4) / 2);
    VOICE_NOTES[0] = ctx.currentNote;
    currentAmpCompCalibrationVal = compute_initial_amp_for_note(ctx, j);

    if (currentAmpCompCalibrationVal > DIV_COUNTER * 0.98) {
      float highestFreqFound = find_highest_freq();
      ctx.calibrationData[j] = highestFreqFound;
      ctx.calibrationData[j + 1] = DIV_COUNTER;

      for (int i = j + 2; i < numPresetVoltages; i += 2) {
        ctx.calibrationData[i] = 20000000;
        ctx.calibrationData[i + 1] = DIV_COUNTER;
      }
      break;
    }

    uint16_t minAmpComp = currentAmpCompCalibrationVal * 0.8;  // Lower Limit for this note.
    uint16_t maxAmpComp = currentAmpCompCalibrationVal * 1.3;  // Upper Limit for this note.

    double freqHz = sNotePitches[VOICE_NOTES[0] - 12];
    tolerance = compute_gap_tolerance_for_freq(freqHz, dutyErrorFraction);

    // For debugging, report the effective duty-cycle tolerance in percent.
    double periodUs = (freqHz > 0.0) ? (1000000.0 / freqHz) : 0.0;
    double toleranceDutyPercent = 0.0;
    if (periodUs > 0.0) {
      double tolDutyFrac = tolerance / (2.0 * periodUs);
      toleranceDutyPercent = tolDutyFrac * 100.0;
    }

    Serial.println((String) "Current DCO: " + ctx.dcoIndex);
    Serial.println((String) "Calibration note: " + VOICE_NOTES[0]);
    Serial.println((String) "Calibration note freq: " + sNotePitches[VOICE_NOTES[0] - 12]);
    Serial.println((String) "Calibration note amplitude: " + currentAmpCompCalibrationVal);
    Serial.println((String) "Tolerance (us): " + tolerance);
    Serial.println((String) "Tolerance duty approx (%): " + toleranceDutyPercent);
    Serial.println((String) "MinAmpComp: " + minAmpComp);
    Serial.println((String) "MaxAmpComp: " + maxAmpComp);

    voice_task_autotune(0, currentAmpCompCalibrationVal);  // Send the preset voltage
    delay(10);

    uint16_t bestAmpComp = currentAmpCompCalibrationVal;  // Best PWM found so far for this note.
    float closestToZero = 50000;  // Smallest absolute duty error seen so far.
    float previousAvgValue = 0.0; // Duty error from the previous iteration (for sign-change detection).

    float lowerMeasurements[rangeSamples];   // Duty errors measured at lower neighbour PWMs.
    float higherMeasurements[rangeSamples];  // Duty errors measured at higher neighbour PWMs.
    uint16_t lowerVoltages[rangeSamples];    // PWM values used for lowerMeasurements[].
    uint16_t higherVoltages[rangeSamples];   // PWM values used for higherMeasurements[].

    int flipCounter = 0;  // Count of successive sign changes; used to relax tolerance if the search oscillates.

    while (true) {
      float avgValue = measure_gap_for_amp(currentAmpCompCalibrationVal);

      // Optional debug: report current duty and tolerance when enabled.
      // Treat timeout sentinel specially so we don't fake a 50% duty reading.
      if (autotuneDebug >= 2 && periodUs > 0.0) {
        if (avgValue == kGapTimeoutSentinel) {
          Serial.println((String)"[DCO_AMP_SCAN] note=" + ctx.currentNote +
                         (String)" DCO=" + ctx.dcoIndex +
                         (String)" AMP=" + currentAmpCompCalibrationVal +
                         (String)" gap=TIMEOUT" +
                         (String)" duty=NA target=50% tol≈" + toleranceDutyPercent + "%");
        } else {
          // avgValue is the same DCO_calibration_difference used elsewhere:
          // positive => low segment longer (duty < 50%), negative => high longer.
          double dutyErrorFrac = (double)avgValue / (2.0 * periodUs);
          double dutyPercent   = (0.5 + dutyErrorFrac) * 100.0;
        Serial.println((String)"[DCO_AMP_SCAN] note=" + ctx.currentNote +
                       (String)" DCO=" + ctx.dcoIndex +
                       (String)" AMP=" + currentAmpCompCalibrationVal +
                       (String)" gap=" + avgValue +
                       (String)"us duty=" + dutyPercent +
                       (String)"% target=50% tol≈" + toleranceDutyPercent + "%");
        }
      }

      // Update best candidate only if the measurement is valid (not a timeout)
      // and closer to zero than what we've seen before.
      if (avgValue != kGapTimeoutSentinel && abs(avgValue) < abs(closestToZero)) {
        closestToZero = avgValue;
        bestAmpComp = currentAmpCompCalibrationVal;
      } else {
        avgValue == 0;
      }

      // Detect sign change
      if (did_sign_change(previousAvgValue, avgValue)) {
        // Store measurements around the current voltage
        for (int i = 0; i < rangeSamples; i++) {
          float lowerVoltage = currentAmpCompCalibrationVal - (i + 1);
          float higherVoltage = currentAmpCompCalibrationVal + (i + 1);

          lowerMeasurements[i] = measure_gap_for_amp(lowerVoltage);
          lowerVoltages[i] = lowerVoltage;

          higherMeasurements[i] = measure_gap_for_amp(higherVoltage);
          higherVoltages[i] = higherVoltage;
        }

        update_best_from_neighbours(
          rangeSamples,
          lowerMeasurements,
          lowerVoltages,
          higherMeasurements,
          higherVoltages,
          avgValue,
          closestToZero,
          bestAmpComp,
          currentAmpCompCalibrationVal
        );

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

      step_amp_from_error(avgValue, tolerance, currentAmpCompCalibrationVal);

      // Ensure the voltage stays within the allowed range
      if (currentAmpCompCalibrationVal < minAmpComp || currentAmpCompCalibrationVal > maxAmpComp) {
        Serial.println((String) "Calibration voltage out of range: " + currentAmpCompCalibrationVal);
      }

      previousAvgValue = avgValue;
    }

    store_note_result(ctx, j, bestAmpComp, closestToZero);
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