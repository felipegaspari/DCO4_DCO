
#include "include_all.h"


// Helper: turn off all oscillators and set their range PWMs to 0.
static void disable_all_oscillators_and_range_pwm() {
  for (int i = 0; i < NUM_OSCILLATORS; i++) {
    uint8_t pioNumber = VOICE_TO_PIO[i];
    PIO pioN = pio[VOICE_TO_PIO[i]];
    uint8_t sm1N = VOICE_TO_SM[i];

    uint32_t clk_div1 = 200;

    pio_sm_put(pioN, sm1N, clk_div1);
    pio_sm_exec(pioN, sm1N, pio_encode_pull(false, false));
    pwm_set_chan_level(RANGE_PWM_SLICES[i], pwm_gpio_to_channel(RANGE_PINS[i]), 0);
  }
}

// Helper: set PW for all even-indexed voices to the center value.
static void reset_even_pw_to_center() {
  for (int i = 0; i < NUM_VOICES_TOTAL; i += 2) {
    PW[i] = DIV_COUNTER_PW / 2;
    pwm_set_chan_level(PW_PWM_SLICES[i], pwm_gpio_to_channel(PW_PINS[i]), PW[i]);
  }
}

// Initialize legacy PID-based DCO calibration state for oscillator 0.
// Note: the main calibration now uses calibrate_DCO(); this is kept
// for compatibility and reference.
void init_DCO_calibration() {

  currentDCO = 0;

  VOICE_NOTES[0] = DCO_calibration_start_note;
  DCO_calibration_current_note = DCO_calibration_start_note;

  arrayPos = 0;
  calibrationData[arrayPos] = 0;
  calibrationData[arrayPos + 1] = ampCompLowestFreqVal;
  arrayPos += 2;

  calibrationData[arrayPos] = (uint32_t)(sNotePitches[manual_DCO_calibration_start_note - 12] * 100);
  calibrationData[arrayPos + 1] = initManualAmpCompCalibrationVal[currentDCO];

  arrayPos += 2;

  currentNoteCalibrationStart = micros();
  DCOCalibrationStart = millis();
  DCO_calibration_difference = 10000;
  PIDMinGap = 300;

  samplesNumber = 52;

  sampleTime = (1000000 / sNotePitches[DCO_calibration_current_note - 12]) * ((samplesNumber - 1) / 2);

  PIDLimitsFormula = 78;
  PIDOutputLowerLimit = 70;
  PIDOutputHigherLimit = 100;

  // TURN OFF ALL OSCILLATORS and reset PW for even voices.
  disable_all_oscillators_and_range_pwm();
  reset_even_pw_to_center();

  // DISABLE PW PWM
  // for (int i = 0; i < NUM_VOICES_TOTAL; i++) {
  //   PW[i] = PW_CENTER[i];
  //   pwm_set_chan_level(PW_PWM_SLICES[i], pwm_gpio_to_channel(PW_PINS[i]), PW[i]);
  //   pwm_set_enabled(PW_PWM_SLICES[i], false);
  // }

  voice_task_autotune(0, PIDLimitsFormula);  //what value goes here?

  delay(100);

  DCO_calibration_difference = 4000;
  lastDCODifference = 50000;
  lastGapFlipCount = 0;
  lastPIDgap = 50000;
  bestGap = 50000;
  bestCandidate = 50000;
  lastampCompCalibrationVal = 0;
  edgeDetectionLastTime = 0;
  PIDMinGapCounter = 0;
  pulseCounter = 0;
}

/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/
// Main DCO amplitude-compensation calibration entry point.
// For each DCO:
//  - Optionally find PW center (for even-indexed oscillators),
//  - Run calibrate_DCO() to build a [freq -> range PWM] table,
//  - Persist that table via update_FS_voice().
void DCO_calibration() {

  // TURN OFF ALL OSCILLATORS and reset PW for even voices.
  disable_all_oscillators_and_range_pwm();
  reset_even_pw_to_center();

  for (int i = 0; i < NUM_OSCILLATORS; i++) {
    currentDCO = i;

    restart_DCO_calibration();

    ampCompCalibrationVal = initManualAmpCompCalibrationVal[currentDCO] + manualCalibrationOffset[currentDCO];
    pwm_set_chan_level(RANGE_PWM_SLICES[i], pwm_gpio_to_channel(RANGE_PINS[i]), ampCompCalibrationVal);

    if ((currentDCO % 2) == 0) {
      // For each voice (even-indexed DCO), calibrate PW center, low limit,
      // and high limit using the shared PW search routines.
      // Mode 0 = low note PW center (used for both limits as well).
      find_PW_center(0);
      find_PW_low_limit();
      find_PW_high_limit();

      // After PW calibration, set PW to the calibrated center for this voice
      // so that subsequent DCO amplitude calibration runs from a good PW.
      pwm_set_chan_level(PW_PWM_SLICES[currentDCO / 2],
                         pwm_gpio_to_channel(PW_PINS[currentDCO / 2]),
                         PW_CENTER[currentDCO / 2]);

    } else {
      DCO_calibration_current_note = DCO_calibration_start_note;
      VOICE_NOTES[0] = DCO_calibration_current_note;
    }

    // uint16_t lowestFrequency = find_lowest_freq();
    // calibrationData[0] = lowestFrequency;

    bool oscAmpCompCalibrationComplete = false;

    // Build a small context for this DCO and run the calibration routine.
    DCOCalibrationContext ctx(
      currentDCO,
      DCO_calibration_current_note,
      calibrationData,
      manualCalibrationOffset,
      initManualAmpCompCalibrationVal
    );
    // Desired duty-cycle error tolerance as a fraction (e.g. 0.005 = 0.5%).
    double dutyErrorFraction = 0.002;
    calibrate_DCO(ctx, dutyErrorFraction);

    for (int i = 0; i < chanLevelVoiceDataSize; i++) {
      Serial.println(calibrationData[i]);
    }

    update_FS_voice(currentDCO);

    Serial.println((String) "DCO " + currentDCO + (String) " calibration finished.");

    // if ((currentDCO % 2) == 0) {
    //   //Falta agregar que use los nuevos datos antes de encontrar el centro
    //   find_PW_center(1);
    //   //find_PW_high_limit();
    //   //find_PW_low_limit();
    // }

    restart_DCO_calibration();
  }
  calibrationFlag = false;
  init_FS();

  // Rebuild amp-comp tables for the active engine.
  precompute_amp_comp_for_engine();
}
/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/

// Reset per-DCO calibration state and header entries in calibrationData.
// This is called before calibrating each DCO (and reused by VCO calibration).
void restart_DCO_calibration() {

  VOICE_NOTES[0] = DCO_calibration_start_note;
  DCO_calibration_current_note = DCO_calibration_start_note;

  arrayPos = 0;
  calibrationData[arrayPos] = 0;
  calibrationData[arrayPos + 1] = ampCompLowestFreqVal;
  arrayPos += 2;

  calibrationData[arrayPos] = (uint32_t)(sNotePitches[DCO_calibration_current_note - calibration_note_interval - 12] * 100);
  calibrationData[arrayPos + 1] = initManualAmpCompCalibrationVal[currentDCO] + manualCalibrationOffset[currentDCO];

  arrayPos += 2;

  currentNoteCalibrationStart = micros();
  DCOCalibrationStart = millis();
  DCO_calibration_difference = 10000;
  PIDMinGap = 300;


  // TURN OFF ALL OSCILLATORS for a clean restart.
  disable_all_oscillators_and_range_pwm();

  delay(100);

  DCO_calibration_difference = 4000;
  lastDCODifference = 50000;
  lastGapFlipCount = 0;
  lastPIDgap = 50000;
  bestGap = 50000;
  bestCandidate = 50000;
  lastampCompCalibrationVal = 0;
  edgeDetectionLastTime = 0;
  PIDMinGapCounter = 0;
  pulseCounter = 0;
}

/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/

// Shared search routine used by PW calibration functions (center/low/high).
// It searches for the PW value that yields a duty cycle closest to
// targetDutyFraction at the current calibration note. targetGap is the
// allowed absolute gap (in microseconds) from the ideal duty at that note.
static uint16_t find_PW_for_target_duty(double targetDutyFraction, uint16_t targetGap) {

  DCO_calibration_difference = 4000;
  lastDCODifference = 50000;
  lastGapFlipCount = 0;
  lastPIDgap = 50000;
  bestGap = 50000;
  bestCandidate = 50000;
  lastampCompCalibrationVal = 0;
  edgeDetectionLastTime = 0;
  PIDMinGapCounter = 0;
  pulseCounter = 0;

  // Precompute period and duty-cycle tolerance (in %) for debug reporting.
  double freqHz = (double)sNotePitches[DCO_calibration_current_note - 12];
  double periodUs = (freqHz > 0.0) ? (1000000.0 / freqHz) : 0.0;
  double toleranceDutyPercent = 0.0;
  double gapTarget = 0.0;
  if (periodUs > 0.0) {
    // Ideal gap for target duty: gap = T*(1 - 2p)
    gapTarget = periodUs * (1.0 - 2.0 * targetDutyFraction);
    double tolDutyFrac = (double)targetGap / (2.0 * periodUs);
    toleranceDutyPercent = tolDutyFrac * 100.0;
  }

  // ---- Phase 1: Coarse scan over PW range to find a sign-change bracket ----
  uint16_t pwMin = 0;
  uint16_t pwMax = DIV_COUNTER_PW;
  uint16_t coarseStep = DIV_COUNTER_PW / 16;
  if (coarseStep == 0) coarseStep = 1;

  bool havePrev = false;
  double prevGap = 0.0;
  uint16_t prevPW = 0;

  bool haveBracket = false;
  uint16_t pwLow = 0, pwHigh = 0;
  double gapLow = 0.0, gapHigh = 0.0;

  for (uint16_t pw = pwMin; pw <= pwMax; pw = (uint16_t)(pw + coarseStep)) {

    if (millis() - DCOCalibrationStart > 60000) {
      Serial.println("PW center coarse scan timeout (60s)");
      break;
    }

    pwm_set_chan_level(PW_PWM_SLICES[currentDCO / 2],
                       pwm_gpio_to_channel(PW_PINS[currentDCO / 2]),
                       pw);
    delay(30);

    GapMeasurement gm = measure_gap(2);
    if (gm.timedOut) {
      continue;  // no usable signal at this PW
    }

    double gap = (double)gm.value;
    double gapDiff = gap - gapTarget;
    double absGapDiff = abs(gapDiff);

    if (autotuneDebug >= 2 && periodUs > 0.0) {
      double dutyErrorFrac = -gap / (2.0 * periodUs);
      double dutyPercent = (0.5 + dutyErrorFrac) * 100.0;
      Serial.println((String)"PW coarse: PW=" + pw +
                     (String)" gap=" + gap +
                     (String)"us duty=" + dutyPercent +
                     (String)"% tolDuty≈" + toleranceDutyPercent + "%");
    }

    if (absGapDiff < bestGap) {
      bestGap = absGapDiff;
      bestCandidate = pw;
    }

    if (havePrev) {
      // Check for sign change between prevGap and gap (relative to target duty)
      if ((gapDiff > 0.0 && prevGap < 0.0) || (gapDiff < 0.0 && prevGap > 0.0)) {
        haveBracket = true;
        pwLow = prevPW;
        gapLow = prevGap;
        pwHigh = pw;
        gapHigh = gap;
        break;
      }
    }

    havePrev = true;
    // For the bracket we keep the raw gap value; we subtract gapTarget only
    // when computing gapDiff.
    prevGap = gap;
    prevPW = pw;
  }

  // If we didn't find a bracket, fall back to the best coarse candidate.
  if (!haveBracket) {
    Serial.println("PW center: no sign-change bracket found, using best coarse candidate.");
  } else {
    // ---- Phase 2: Bisection search within the bracket ----
    for (int iter = 0; iter < 14; ++iter) {
      if (millis() - DCOCalibrationStart > 60000) {
        Serial.println("PW center bisection timeout (60s)");
        break;
      }

      uint16_t pwMid = (uint16_t)((pwLow + pwHigh) / 2);
      pwm_set_chan_level(PW_PWM_SLICES[currentDCO / 2],
                         pwm_gpio_to_channel(PW_PINS[currentDCO / 2]),
                         pwMid);
      delay(30);

      GapMeasurement gm = measure_gap(2);
      if (gm.timedOut) {
        // No valid data at this midpoint; skip this iteration and try again
        // on the next loop. Global time/iteration guards will still ensure
        // we eventually stop if there is no usable region.
        if (autotuneDebug >= 2) {
          Serial.println("PW center: timeout during bisection, skipping midpoint.");
        }
        continue;
      }

      double gapMid = (double)gm.value;
      double gapDiffMid = gapMid - gapTarget;
      double absGapDiffMid = abs(gapDiffMid);
      if (absGapDiffMid < bestGap) {
        bestGap = absGapDiffMid;
        bestCandidate = pwMid;
      }

      if (autotuneDebug >= 2 && periodUs > 0.0) {
        double dutyErrorFrac = -gapMid / (2.0 * periodUs);
        double dutyPercent = (0.5 + dutyErrorFrac) * 100.0;
        Serial.println((String)"PW bisect: PW=" + pwMid +
                       (String)" gap=" + gapMid +
                       (String)"us duty=" + dutyPercent +
                       (String)"% tolDuty≈" + toleranceDutyPercent + "%");
      }

      // Early exit if we're within the original target gap.
      if (absGapDiffMid <= (double)targetGap) {
        break;
      }

      // Maintain the sign-change bracket.
      if ((gapDiffMid > 0.0 && (gapLow - gapTarget) > 0.0) ||
          (gapDiffMid < 0.0 && (gapLow - gapTarget) < 0.0)) {
        pwLow = pwMid;
        gapLow = gapMid;
      } else {
        pwHigh = pwMid;
        gapHigh = gapMid;
      }

      if (pwHigh - pwLow <= 1) {
        // Can't refine further in integer PW space.
        break;
      }
    }
  }

  if (autotuneDebug >= 1) {
    Serial.println((String)"PW search best gap: " + bestGap +
                   (String)" at PW: " + bestCandidate);
  }

  return bestCandidate;
}

// Locate PW center for the current DCO's voice by minimizing duty-cycle error
// at a reference note. Mode 0 = low note, mode 1 = higher note refinement.
void find_PW_center(uint8_t mode) {

  uint16_t targetGap;
  uint8_t voiceTaskMode;

  if (mode == 0) {
    DCO_calibration_current_note = manual_DCO_calibration_start_note;
    VOICE_NOTES[0] = DCO_calibration_current_note;
    targetGap = 20;
    voiceTaskMode = 2;
  } else {
    DCO_calibration_current_note = 76;
    VOICE_NOTES[0] = DCO_calibration_current_note;
    targetGap = 5;
    voiceTaskMode = 3;
  }

  currentNoteCalibrationStart = micros();
  DCOCalibrationStart = millis();

  PIDOutputLowerLimit = 0;
  PIDOutputHigherLimit = DIV_COUNTER_PW;

  ampCompCalibrationVal = initManualAmpCompCalibrationVal[currentDCO] + manualCalibrationOffset[currentDCO];

  if (firstTuneFlag == true) {
    PW[currentDCO / 2] = DIV_COUNTER_PW / 2;
    PWCalibrationVal = DIV_COUNTER_PW / 2;
    PW_CENTER[currentDCO / 2] = DIV_COUNTER_PW / 2;
  } else {

    PW[currentDCO / 2] = PW_CENTER[currentDCO / 2];
    PWCalibrationVal = PW_CENTER[currentDCO / 2];
  }

  voice_task_autotune(voiceTaskMode, ampCompCalibrationVal);

  uint16_t centerPW = find_PW_for_target_duty(kPWCenterDutyFraction, targetGap);
  Serial.println("PW center found !!!");
  update_FS_PWCenter(currentDCO / 2, centerPW);
  PW_CENTER[currentDCO / 2] = centerPW;
}

/////////////////////////////////
////////////////////////////////
////////////////////////////////

// Find a lower PW limit for the current DCO's voice at which duty/frequency
// behaviour remains acceptable. Uses the shared PW search with a low duty
// fraction target (e.g. 10% duty).
void find_PW_low_limit() {
  DCO_calibration_current_note = DCO_calibration_start_note;
  VOICE_NOTES[0] = DCO_calibration_current_note;

  currentNoteCalibrationStart = micros();
  DCOCalibrationStart = millis();

  PIDOutputLowerLimit = 0;
  PIDOutputHigherLimit = DIV_COUNTER_PW;

  // Use a gap tolerance proportional to the period (1% of period as before).
  uint16_t targetGap = (uint16_t)(1000000.00f / sNotePitches[DCO_calibration_current_note - 12] * 0.01f);

  // Initialize PW somewhere in the lower half of the range as a starting point.
  PW[currentDCO / 2] = PW_CENTER[currentDCO / 2] / 2;
  PWCalibrationVal = PW[currentDCO / 2];
  pwm_set_chan_level(PW_PWM_SLICES[currentDCO / 2],
                     pwm_gpio_to_channel(PW_PINS[currentDCO / 2]),
                     PW[currentDCO / 2]);

  // Configure the DCO for PW calibration mode.
  voice_task_autotune(3, 0);

  delay(100);

  uint16_t lowPW = find_PW_for_target_duty(kPWLowDutyFraction, targetGap);
  Serial.println("PW low limit found !!!");
  update_FS_PW_Low_Limit(currentDCO / 2, lowPW);
  PW_LOW_LIMIT[currentDCO / 2] = lowPW;
}

// Find a higher PW limit for the current DCO's voice at which duty/frequency
// behaviour remains acceptable. Uses the shared PW search with a high duty
// fraction target (e.g. 90% duty).
void find_PW_high_limit() {
  DCO_calibration_current_note = DCO_calibration_start_note;
  VOICE_NOTES[0] = DCO_calibration_current_note;

  currentNoteCalibrationStart = micros();
  DCOCalibrationStart = millis();

  PIDOutputLowerLimit = 0;
  PIDOutputHigherLimit = DIV_COUNTER_PW;

  // Use a gap tolerance proportional to the period (1% of period as before).
  uint16_t targetGap = (uint16_t)(1000000.00f / sNotePitches[DCO_calibration_current_note - 12] * 0.01f);

  // Initialize PW somewhere in the upper half of the range as a starting point.
  PW[currentDCO / 2] = (PW_CENTER[currentDCO / 2] + DIV_COUNTER_PW) / 2;
  PWCalibrationVal = PW[currentDCO / 2];
  pwm_set_chan_level(PW_PWM_SLICES[currentDCO / 2],
                     pwm_gpio_to_channel(PW_PINS[currentDCO / 2]),
                     PW[currentDCO / 2]);

  // Configure the DCO for PW calibration mode.
  voice_task_autotune(3, 0);

  delay(100);

  uint16_t highPW = find_PW_for_target_duty(kPWHighDutyFraction, targetGap);
  Serial.println("PW high limit found !!!");
  update_FS_PW_High_Limit(currentDCO / 2, highPW);
  PW_HIGH_LIMIT[currentDCO / 2] = highPW;
}

// Measure duty-cycle error on DCO_calibration_pin by timing rising/falling
// edges. Returns 0 when duty is ≈50%, or kGapTimeoutSentinel on timeout.
float find_gap(byte specialMode) {
  if (specialMode == 2) {  // find lowest freq mode
    samplesNumber = 14;
  } else {
    samplesNumber = 10;
  }

  edgeDetectionLastTime = micros();

  while (samplesCounter < samplesNumber) {

    bool val = digitalRead(DCO_calibration_pin);
    microsNow = micros();
    if ((microsNow - edgeDetectionLastTime) > kGapTimeoutUs) {

      pulseCounter = 0;
      samplesCounter = 0;
      DCO_calibration_difference = kGapTimeoutSentinel;
      val = 0;
      edgeDetectionLastVal = 0;

      if (autotuneDebug >= 3) {
        Serial.println("Timeout loop");
        Serial.println((String) "ampCompCalibrationVal: " + ampCompCalibrationVal);
      }

      microsNow = micros();
      edgeDetectionLastTime = microsNow;

      return kGapTimeoutSentinel;
    }
    if (val != edgeDetectionLastVal) {
      if ((microsNow - edgeDetectionLastTime) >= kEdgeDebounceMinUs) {

        edgeDetectionLastVal = val;

        if (pulseCounter == 1 && val == 0) {
          pulseCounter == 0;
        }
        if (pulseCounter > 2) {
          if (val == 0) {
            fallingEdgeTimeSum += microsNow - edgeDetectionLastTime;
          } else {
            risingEdgeTimeSum += microsNow - edgeDetectionLastTime;
          }
          samplesCounter++;
        }
        edgeDetectionLastTime = microsNow;
        pulseCounter++;
      }
    }
  }

  if (samplesCounter == samplesNumber) {

    DCO_calibration_difference = float((float)fallingEdgeTimeSum / float((float)samplesNumber / 4.00f) 
    - float((float)risingEdgeTimeSum / float((float)samplesNumber / 4.00f))); // this is the difference between 50% duty cyclce and the actual duty cycle, divided by 4 to allow more coarse calibration.

    if (autotuneDebug >= 1) {
      Serial.println((String) "DCO_calibration_difference: " + DCO_calibration_difference);
      Serial.println((String) "NOTE: " + DCO_calibration_current_note);
    }

    
    pulseCounter = 0;
    samplesCounter = 0;
    risingEdgeTimeSum = 0;
    fallingEdgeTimeSum = 0;
    edgeDetectionLastVal = 0;

  } else {
    return kGapTimeoutSentinel;
  }
  return (float)DCO_calibration_difference;
}

/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/

// LEGACY: PID-based search for highest achievable DCO frequency.
// Currently unused by the main calibration flow (calibrate_DCO()).
#if 0  // LEGACY_DCO_CALIBRATION_FIND_HIGHEST_FREQ
void DCO_calibration_find_highest_freq() {

  samplesNumber = 28;

  while (samplesCounter < samplesNumber) {
    bool val = digitalRead(DCO_calibration_pin);
    microsNow = micros();
    // if ((microsNow - edgeDetectionLastTime) > 10000) {
    //   if (autotuneDebug >= 1) {
    //     Serial.println("return");
    //   }
    //   edgeDetectionLastTime = 0;
    //   return;
    // }
    if (val != edgeDetectionLastVal) {
      if ((microsNow - edgeDetectionLastTime) >= 30) {

        edgeDetectionLastVal = val;

        if (pulseCounter == 1 && val == 0) {
          pulseCounter == 0;
        }
        if (pulseCounter != 0) {
          if (val == 0) {
            fallingEdgeTimeSum += microsNow - edgeDetectionLastTime;
          } else {
            risingEdgeTimeSum += microsNow - edgeDetectionLastTime;
          }
          samplesCounter++;
        }
        if (autotuneDebug >= 4) {
          Serial.println((String) "pulseCounter: " + pulseCounter);
        }
        edgeDetectionLastTime = microsNow;
        pulseCounter++;
      }
    }
  }

  if (samplesCounter == samplesNumber) {
    //  Serial.print((String) "ON= " + (1000000 / (risingEdgeTimeSum / 45)) + (String) " - ");
    //  Serial.println((String) "OFF= " + (1000000 / (fallingEdgeTimeSum / 45)));

    // Serial.println(1000000 / ((risingEdgeTimeSum + fallingEdgeTimeSum) / 90));
    DCO_calibration_difference = ((float)fallingEdgeTimeSum / ((float)samplesNumber / 4) - ((float)risingEdgeTimeSum / ((float)samplesNumber / 4)));

    if (autotuneDebug >= 1) {
      Serial.println((String) "NOTE: " + DCO_calibration_current_note);
    }

    pulseCounter = 0;
    samplesCounter = 0;
    risingEdgeTimeSum = 0;
    fallingEdgeTimeSum = 0;
    edgeDetectionLastVal = 0;
  }
}
#endif  // LEGACY_DCO_CALIBRATION_FIND_HIGHEST_FREQ

/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/

// Debug helper: repeatedly measure and print duty-cycle difference for
// the current note. Not used in normal calibration flow.
void DCO_calibration_debug() {
  // if (DCO_calibration_current_note > 31) {
  //   samplesNumber = 25 * 2;
  // } else if (DCO_calibration_current_note > 51) {
  //   samplesNumber = 25 * 2;
  // } else if (DCO_calibration_current_note > 71) {
  //   samplesNumber = 19 * 2;
  // } else if (DCO_calibration_current_note > 91) {
  //   samplesNumber = 15 * 2;
  // } else {
  //   samplesNumber = 21 * 2;
  // }

  samplesNumber = 12;

  edgeDetectionLastTime = micros();

  while (samplesCounter < samplesNumber) {
    bool val = digitalRead(DCO_calibration_pin);
    microsNow = micros();

    if ((microsNow - edgeDetectionLastTime) > kGapTimeoutUs) {

      pulseCounter = 0;
      samplesCounter = 0;
      DCO_calibration_difference = kGapTimeoutSentinel;
      val = 0;
      edgeDetectionLastVal = 0;

      if (autotuneDebug >= 3) {
        Serial.println("Timeout loop");
        Serial.println((String) "ampCompCalibrationVal: " + ampCompCalibrationVal);
      }

      microsNow = micros();
      edgeDetectionLastTime = microsNow;
      break;
    }

    if (val != edgeDetectionLastVal) {
      if ((microsNow - edgeDetectionLastTime) >= kEdgeDebounceMinUs) {

        edgeDetectionLastVal = val;

        if (pulseCounter == 1 && val == 0) {
          pulseCounter == 0;
        }
        if (pulseCounter > 2) {
          if (val == 0) {
            fallingEdgeTimeSum += microsNow - edgeDetectionLastTime;
//            fallingEdgeTimeSum_counter++;
          } else {
            risingEdgeTimeSum += microsNow - edgeDetectionLastTime;
//            risingEdgeTimeSum_counter++;
          }
          samplesCounter++;
        }
        edgeDetectionLastTime = microsNow;
        pulseCounter++;
      }
    }
  }

  if (samplesCounter == samplesNumber) {
    //  Serial.print((String) "ON= " + (1000000 / (risingEdgeTimeSum / 45)) + (String) " - ");
    //  Serial.println((String) "OFF= " + (1000000 / (fallingEdgeTimeSum / 45)));

    // Serial.println(1000000 / ((risingEdgeTimeSum + fallingEdgeTimeSum) / 90));
    DCO_calibration_difference = ((float)fallingEdgeTimeSum / (samplesNumber / 4) - ((float)risingEdgeTimeSum / (samplesNumber / 4)));

    if (autotuneDebug >= 1) {
      Serial.println(DCO_calibration_difference);
      Serial.println((String) "NOTE: " + DCO_calibration_current_note);
    }

    serialSendParam32(154,(int32_t)DCO_calibration_difference);

    pulseCounter = 0;
    samplesCounter = 0 ;
    risingEdgeTimeSum = 0;
    fallingEdgeTimeSum = 0;
    edgeDetectionLastVal = 0;
  }
}


/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/

// LEGACY: VCO-style calibration entry point mirroring DCO_calibration().
// This is currently unused in the main firmware but kept for reference.
#if 0  // LEGACY_VCO_CALIBRATION
void VCO_calibration() {

  // TURN OFF ALL OSCILLATORS and reset PW for even voices.
  disable_all_oscillators_and_range_pwm();
  reset_even_pw_to_center();

  for (int i = 0; i < NUM_OSCILLATORS; i++) {
    currentDCO = i;

    restart_DCO_calibration();

    ampCompCalibrationVal = initManualAmpCompCalibrationVal[currentDCO] + manualCalibrationOffset[currentDCO];
    pwm_set_chan_level(RANGE_PWM_SLICES[i], pwm_gpio_to_channel(RANGE_PINS[i]), ampCompCalibrationVal);

    // if ((currentDCO % 2) == 0) {
    //   if (firstTuneFlag == true) {
    //     find_PW_center(0);
    //     pwm_set_chan_level(PW_PWM_SLICES[currentDCO / 2], pwm_gpio_to_channel(PW_PINS[currentDCO / 2]), PW_CENTER[currentDCO / 2]);

    //   } else {
    //     find_PW_center(0);  // Should be on. off for testing!!!!!!
    //     //find_PW_low_limit();
    //   }
    // } else {
      DCO_calibration_current_note = DCO_calibration_start_note;
      VOICE_NOTES[0] = DCO_calibration_current_note;
    // }

    // uint16_t lowestFrequency = find_lowest_freq();
    // calibrationData[0] = lowestFrequency;

    bool oscAmpCompCalibrationComplete = false;

    calibrate_DCO();

    for (int i = 0; i < chanLevelVoiceDataSize; i++) {
      Serial.println(calibrationData[i]);
    }

    update_FS_voice(currentDCO);

    Serial.println((String) "DCO " + currentDCO + (String) " calibration finished.");

    // if ((currentDCO % 2) == 0) {
    //   //Falta agregar que use los nuevos datos antes de encontrar el centro
    //   find_PW_center(1);
    //   //find_PW_high_limit();
    //   //find_PW_low_limit();
    // }

    restart_DCO_calibration();
  }
  calibrationFlag = false;
  init_FS();
  
  // Rebuild amp-comp tables for the active engine.
  precompute_amp_comp_for_engine();}
#endif  // LEGACY_VCO_CALIBRATION

/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/

// Measure VCO/DCO frequency by timing edges and computing the period.
// Sets DCO_calibration_difference to (targetFreq - measuredFreq).
// Currently unused in the main calibration flow.
#if 0  // LEGACY_VCO_MEASURE_FREQUENCY
void VCO_measure_frequency() {
  samplesNumber = 12;
  edgeDetectionLastTime = micros();

  while (samplesCounter < samplesNumber) {
    bool val = digitalRead(DCO_calibration_pin);
    microsNow = micros();

    if ((microsNow - edgeDetectionLastTime) > kGapTimeoutUs) {
      pulseCounter = 0;
      samplesCounter = 0;
      DCO_calibration_difference = kGapTimeoutSentinel;
      val = 0;
      edgeDetectionLastVal = 0;

      if (autotuneDebug >= 3) {
        Serial.println("Timeout loop");
        Serial.println((String) "ampCompCalibrationVal: " + ampCompCalibrationVal);
      }

      microsNow = micros();
      edgeDetectionLastTime = microsNow;
      break;
    }

    if (val != edgeDetectionLastVal) {
      if ((microsNow - edgeDetectionLastTime) >= kEdgeDebounceMinUs) {
        edgeDetectionLastVal = val;

        if (pulseCounter == 1 && val == 0) {
          pulseCounter == 0;
        }
        if (pulseCounter > 2) {
          if (val == 0) {
            fallingEdgeTimeSum += microsNow - edgeDetectionLastTime;
          } else {
            risingEdgeTimeSum += microsNow - edgeDetectionLastTime;
          }
          samplesCounter++;
        }
        edgeDetectionLastTime = microsNow;
        pulseCounter++;
      }
    }
  }

  if (samplesCounter == samplesNumber) {
    float freqReading = 1000000.00f / float((risingEdgeTimeSum + fallingEdgeTimeSum) / (float)samplesNumber * 2);
    DCO_calibration_difference = sNotePitches[DCO_calibration_current_note - 12] - freqReading;

    if (autotuneDebug >= 1) {
      Serial.println((String) "DCO_calibration_difference: " + DCO_calibration_difference + (String) " - Freq reading: " + freqReading);
      Serial.println((String) "NOTE: " + DCO_calibration_current_note + (String) " - NOTE Frequency: " + sNotePitches[DCO_calibration_current_note - 12]);
    }

    pulseCounter = 0;
    samplesCounter = 0;
    risingEdgeTimeSum = 0;
    fallingEdgeTimeSum = 0;
    edgeDetectionLastVal = 0;

  }
}
#endif  // LEGACY_VCO_MEASURE_FREQUENCY

/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/

// void init_tuning_tables() {

//   for (int i = 0; i < pwm_to_vco_array_size; i++) {
//     pwm_to_vco_euler_array[i] = pow(pwm_to_vco_array[i], 2.71828);
//   }
//   float freq;

//   // for (int i = 0; i < pwm_to_vco_array_size; i++) {
//   //   ((uint8_t *)&freq)[0] = EEPROM.read(0 + (4 * i));
//   //   ((uint8_t *)&freq)[1] = EEPROM.read(1 + (4 * i));
//   //   ((uint8_t *)&freq)[2] = EEPROM.read(2 + (4 * i));
//   //   ((uint8_t *)&freq)[3] = EEPROM.read(3 + (4 * i));
//   //   freq_to_vco_array[i] = freq;
//   // }
// }

// void autotune() {
//   autotuneOnFlag = true;
//   Serial.println("autotune routine start");
//   delay(10);

//   for (int i = 0; i < pwm_to_vco_array_size; i++) {
//     uint32_t startTime = millis();
//     uint16_t send_ok = 666;
//     // Serial.println((String) "Iteration: " + i + (String) " - VCO pwm level: " + pwm_to_vco_array[i]);

//     serial_send_generaldata(send_ok);
//     // Serial.println("Sent OK");

//     // delay(50);

//     char commandCharacter = 'z';

//     while (commandCharacter != 'k') {
//       // serial_send_generaldata(send_ok);
//       if (uart_is_readable(uart1)) {
//         commandCharacter = uart_getc(uart1);
//       }
//       // Serial.println("Waiting for k");
//     }
//     // Serial.println("received loop OK");
//     pwm_set_chan_level(VCO_PWM_SLICES[0], pwm_gpio_to_channel(22), (uint16_t)(pwm_to_vco_array[i]));
//     while (commandCharacter != 't') {
//       if (uart_is_readable(uart1)) {
//         commandCharacter = uart_getc(uart1);
//       }
//       // Serial.println("Waiting for t");
//       delayMicroseconds(100);
//       if ((millis() - startTime) > 10000) {
//         if (autotuneDebug >= 1) {
//           Serial.println("Timeout waiting for t");
//         }
//         return;
//       }
//     }
//     uint8_t udata = 0;
//     while (udata < 4) {
//       // delay(1);
//       if (uart_is_readable(uart1)) {
//         dataArray[udata] = uart_getc(uart1);
//         udata++;
//       }
//       ((uint8_t *)&dato_serial_float)[0] = dataArray[0];
//       ((uint8_t *)&dato_serial_float)[1] = dataArray[1];
//       ((uint8_t *)&dato_serial_float)[2] = dataArray[2];
//       ((uint8_t *)&dato_serial_float)[3] = dataArray[3];
//     }
//     // Serial.print((String) "Received freq: ");
//     // Serial.print(dato_serial_float);
//     // Serial.println((String) " for value: " + pwm_to_vco_array[i]);
//     freq_to_vco_array[i] = dato_serial_float;
//   }
//   uint16_t send_autotune_end = 777;
//   serial_send_generaldata(send_autotune_end);
//   Serial.println("Sent autotune finish");
//   for (int i = 0; i < pwm_to_vco_array_size; i++) {
//     // Serial.println(pow(pwm_to_vco_array[i], 2.71828) + (String) " " + freq_to_vco_array[i]);
//     Serial.println(freq_to_vco_array[i]);

//     byte *b = (byte *)&freq_to_vco_array[i];
//     freq_to_vco_array_memcopy[0 + (i * 4)] = (b[0]);
//     freq_to_vco_array_memcopy[1 + (i * 4)] = (b[1]);
//     freq_to_vco_array_memcopy[2 + (i * 4)] = (b[2]);
//     freq_to_vco_array_memcopy[3 + (i * 4)] = (b[3]);
//   }
//   // for (int i = 0; i < (pwm_to_vco_array_size * 4); i++) {
//   //   EEPROM.write(i, freq_to_vco_array_memcopy[i]);

//   // }
//   // EEPROM.commit();
//   autotuneOnFlag = false;
// }