
#include "include_all.h"

// For debug logging in gap measurement: track the last PW raw value we
// explicitly programmed for the current DCO, and the duty target/period
// assumed by the current PW search routine. This avoids relying on PW[]
// being untouched by other parts of the system.
static uint16_t g_lastPWMeasurementRaw = 0;
static double   g_gapLogCurrentPeriodUs = 0.0;
static double   g_gapLogTargetDutyFraction = 0.5;  // default 50%

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

    DCO_calibration_current_note = manual_DCO_calibration_start_note;
    VOICE_NOTES[0] = DCO_calibration_current_note;

    if ((currentDCO % 2) == 0) {
      // For each voice (even-indexed DCO), calibrate PW center, low limit,
      // and high limit using the shared PW search routines.
      // Mode 0 = low note PW center (used for both limits as well).
        find_PW_center(0);
      find_PW_limit(PW_LIMIT_LOW);
      find_PW_limit(PW_LIMIT_HIGH);

      // After PW calibration, set PW to the calibrated center for this voice
      // so that subsequent DCO amplitude calibration runs from a good PW,
      // and keep PW[] in sync for debug logging.
      pwm_set_chan_level(PW_PWM_SLICES[currentDCO / 2],
                         pwm_gpio_to_channel(PW_PINS[currentDCO / 2]),
                         PW_CENTER[currentDCO / 2]);
      PW[currentDCO / 2] = PW_CENTER[currentDCO / 2];
    } else {
      DCO_calibration_current_note = DCO_calibration_start_note;
      VOICE_NOTES[0] = DCO_calibration_current_note;
    }

    DCO_calibration_current_note = DCO_calibration_start_note;
    VOICE_NOTES[0] = DCO_calibration_current_note;

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
    double dutyErrorFraction = 0.001;
    calibrate_DCO(ctx, dutyErrorFraction);

    for (int i = 0; i < chanLevelVoiceDataSize; i++) {
      Serial.println(calibrationData[i]);
    }

    update_FS_voice(currentDCO);

    Serial.println((String) "DCO " + currentDCO + (String) " calibration finished.");

    // if ((currentDCO % 2) == 0) {
    //   //Falta agregar que use los nuevos datos antes de encontrar el centro
    //   find_PW_center(1);
    // find_PW_limit(PW_LIMIT_LOW);
    // find_PW_limit(PW_LIMIT_HIGH);
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
static uint16_t find_PW_for_target_duty(double targetDutyFraction,
                                        uint16_t targetGap,
                                        uint16_t pwMin,
                                        uint16_t pwMax) {

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

  // Fixed-size tables for valid and invalid samples.
  // Valid samples: store (PW, gapDiff = gap - gapTarget).
  // Invalid samples: store (PW, distance in PW units to the nearest valid sample).
  const int kMaxSamples = 40;
  uint16_t validPW[kMaxSamples];
  double   validGapDiff[kMaxSamples];
  int      validCount = 0;
  int      inToleranceCount = 0;  // Number of valid samples within target gap

  uint16_t invalidPW[kMaxSamples];
  uint16_t invalidDistToValid[kMaxSamples];
  int      invalidCount = 0;

  // Precompute period and duty-cycle tolerance (in %) for debug reporting.
  double freqHz = (double)sNotePitches[DCO_calibration_current_note - 12];
  double periodUs = (freqHz > 0.0) ? (1000000.0 / freqHz) : 0.0;

  // Update global logging context for gap measurements during this search.
  g_gapLogCurrentPeriodUs     = periodUs;
  g_gapLogTargetDutyFraction  = targetDutyFraction;
  double toleranceDutyPercent = 0.0;
  double gapTarget = 0.0;
  if (periodUs > 0.0) {
    // Ideal gap for target duty: gap = T*(1 - 2p)
    gapTarget = periodUs * (1.0 - 2.0 * targetDutyFraction);
    double tolDutyFrac = (double)targetGap / (2.0 * periodUs);
    toleranceDutyPercent = tolDutyFrac * 100.0;
  }

  // ---- Phase 1: Coarse scan over PW range to find a sign-change bracket ----
  // Use smaller coarse steps for low/high limit searches (target duty far from 50%)
  // and larger steps for center search (targetDutyFraction ~ 0.5).
  uint16_t coarseDiv = (fabs(targetDutyFraction - 0.5) < 0.05) ? 16 : 32;
  uint16_t coarseStep = (pwMax > pwMin) ? ((pwMax - pwMin) / coarseDiv) : 1;
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
    // Keep PW[] and debug tracker in sync so GAP logs show the actual PW tested.
    PW[currentDCO / 2]        = pw;
    g_lastPWMeasurementRaw    = pw;
    delay(30);

    GapMeasurement gm = measure_gap(2);
    if (gm.timedOut) {
      // No usable signal at this PW; if we have at least one valid sample,
      // track this as an invalid entry near some valid PW for diagnostic use.
      if (validCount > 0 && invalidCount < kMaxSamples) {
        // Compute distance to nearest valid PW.
        uint16_t bestDist = 0xFFFF;
        for (int vi = 0; vi < validCount; ++vi) {
          uint16_t dist = (validPW[vi] > pw) ? (validPW[vi] - pw) : (pw - validPW[vi]);
          if (dist < bestDist) bestDist = dist;
        }
        invalidPW[invalidCount] = pw;
        invalidDistToValid[invalidCount] = bestDist;
        invalidCount++;
      } else if (validCount > 0 && invalidCount >= kMaxSamples) {
        // Table full: only keep invalids that are closer to valids than the current worst.
        uint16_t bestDist = 0xFFFF;
        for (int vi = 0; vi < validCount; ++vi) {
          uint16_t dist = (validPW[vi] > pw) ? (validPW[vi] - pw) : (pw - validPW[vi]);
          if (dist < bestDist) bestDist = dist;
        }
        // Find worst (largest distance) invalid entry.
        int worstIdx = 0;
        uint16_t worstDist = invalidDistToValid[0];
        for (int ii = 1; ii < invalidCount; ++ii) {
          if (invalidDistToValid[ii] > worstDist) {
            worstDist = invalidDistToValid[ii];
            worstIdx = ii;
          }
        }
        if (bestDist < worstDist) {
          invalidPW[worstIdx] = pw;
          invalidDistToValid[worstIdx] = bestDist;
        }
      }
      continue;  // skip invalid sample
    }

    double gap = (double)gm.value;
    double gapDiff = gap - gapTarget;
    double absGapDiff = abs(gapDiff);

    if (autotuneDebug >= 2 && periodUs > 0.0) {
      double dutyErrorFrac = -gap / (2.0 * periodUs);
      double dutyPercent = (0.5 + dutyErrorFrac) * 100.0;
      Serial.println((String)"[PW_CENTER_COARSE] note=" + DCO_calibration_current_note +
                     (String)" DCO=" + currentDCO +
                     (String)" PW_raw=" + pw +
                     (String)" gap=" + gap +
                     (String)"us duty=" + dutyPercent +
                     (String)"% target=50% tol≈" + toleranceDutyPercent + "%");
    }

    if (absGapDiff <= (double)targetGap) {
      inToleranceCount++;
    }

    if (absGapDiff < bestGap) {
      bestGap = absGapDiff;
      bestCandidate = pw;
    }

    // Maintain table of best valid samples.
    if (validCount < kMaxSamples) {
      validPW[validCount] = pw;
      validGapDiff[validCount] = gapDiff;
      validCount++;
    } else {
      // Table full: replace the worst entry if this one is closer to the target.
      int worstIdx = 0;
      double worstAbs = fabs(validGapDiff[0]);
      for (int vi = 1; vi < validCount; ++vi) {
        double curAbs = fabs(validGapDiff[vi]);
        if (curAbs > worstAbs) {
          worstAbs = curAbs;
          worstIdx = vi;
        }
      }
      if (absGapDiff < worstAbs) {
        validPW[worstIdx] = pw;
        validGapDiff[worstIdx] = gapDiff;
      }
    }

    if (havePrev) {
      // Check for sign change between prevGap and gap (relative to target duty)
      if ((gapDiff > 0.0 && prevGap < 0.0) || (gapDiff < 0.0 && prevGap > 0.0)) {
        haveBracket = true;
        pwLow = prevPW;
        gapLow = prevGap;
        pwHigh = pw;
        gapHigh = gap;

        // With two valid samples straddling the target, estimate the crossing
        // point via linear interpolation between prevGap and gap.
        double prevGapDiff = prevGap - gapTarget;
        double curGapDiff  = gap - gapTarget;
        double denom = fabs(prevGapDiff) + fabs(curGapDiff);
        if (denom > 0.0) {
          double t = fabs(prevGapDiff) / denom;  // weight towards the closer side
          uint16_t pwEst = (uint16_t)((double)prevPW + ((double)(pw - prevPW) * t));
          if (pwEst >= pwMin && pwEst <= pwMax) {
            pwm_set_chan_level(PW_PWM_SLICES[currentDCO / 2],
                               pwm_gpio_to_channel(PW_PINS[currentDCO / 2]),
                               pwEst);
            PW[currentDCO / 2]     = pwEst;
            g_lastPWMeasurementRaw = pwEst;
            delay(30);
            GapMeasurement gmEst = measure_gap(2);
            if (!gmEst.timedOut) {
              double gapEst = (double)gmEst.value;
              double gapDiffEst = gapEst - gapTarget;
              double absGapDiffEst = fabs(gapDiffEst);

              if (absGapDiffEst <= (double)targetGap) {
                inToleranceCount++;
              }

              if (absGapDiffEst < bestGap) {
                bestGap = absGapDiffEst;
                bestCandidate = pwEst;
              }
              // Insert estimated point into valid table if it's good enough.
              if (validCount < kMaxSamples) {
                validPW[validCount] = pwEst;
                validGapDiff[validCount] = gapDiffEst;
                validCount++;
              }
            }
          }
        }

        break;
      }
    }

    havePrev = true;
    // For the bracket we keep the raw gap value; we subtract gapTarget only
    // when computing gapDiff.
    prevGap = gap;
    prevPW = pw;
  }

  // If we didn't find a bracket, we still want a fine search around the best
  // coarse candidate so that we gather multiple near-target samples before
  // deciding on a final PW.
  if (!haveBracket) {
    if (autotuneDebug >= 1) {
      Serial.println("PW center: no sign-change bracket found, running local fine scan.");
    }
    uint16_t startPW = (bestCandidate >= pwMin && bestCandidate <= pwMax)
                         ? bestCandidate
                         : (uint16_t)((pwMin + pwMax) / 2);
    uint16_t span = (coarseStep > 0) ? coarseStep * 2 : 4;
    uint16_t fineMin = (startPW > span) ? (startPW - span) : pwMin;
    uint16_t fineMax = (startPW + span < pwMax) ? (startPW + span) : pwMax;
    if (fineMax < fineMin) {
      uint16_t tmp = fineMin;
      fineMin = fineMax;
      fineMax = tmp;
    }
    uint16_t fineStep = (fineMax > fineMin) ? ((fineMax - fineMin) / 16) : 1;
    if (fineStep == 0) fineStep = 1;

    for (uint16_t pw = fineMin; pw <= fineMax; pw = (uint16_t)(pw + fineStep)) {
      if (millis() - DCOCalibrationStart > 60000) {
        Serial.println("PW center local fine scan timeout (60s)");
        break;
      }

      pwm_set_chan_level(PW_PWM_SLICES[currentDCO / 2],
                         pwm_gpio_to_channel(PW_PINS[currentDCO / 2]),
                         pw);
      PW[currentDCO / 2]     = pw;
      g_lastPWMeasurementRaw = pw;
      delay(30);

      GapMeasurement gm = measure_gap(2);
      if (gm.timedOut) {
        continue;
      }

      double gap = (double)gm.value;
      double gapDiff = gap - gapTarget;
      double absGapDiff = fabs(gapDiff);

      if (absGapDiff <= (double)targetGap) {
        inToleranceCount++;
      }

      if (absGapDiff < bestGap) {
        bestGap = absGapDiff;
        bestCandidate = pw;
      }

      if (validCount < kMaxSamples) {
        validPW[validCount] = pw;
        validGapDiff[validCount] = gapDiff;
        validCount++;
      }
    }
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
      PW[currentDCO / 2]     = pwMid;
      g_lastPWMeasurementRaw = pwMid;
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
      if (absGapDiffMid <= (double)targetGap) {
        inToleranceCount++;
      }

      if (absGapDiffMid < bestGap) {
        bestGap = absGapDiffMid;
        bestCandidate = pwMid;
      }

      if (autotuneDebug >= 2 && periodUs > 0.0) {
        double dutyErrorFrac = -gapMid / (2.0 * periodUs);
        double dutyPercent = (0.5 + dutyErrorFrac) * 100.0;
        Serial.println((String)"[PW_CENTER_BISECT] note=" + DCO_calibration_current_note +
                       (String)" DCO=" + currentDCO +
                       (String)" PW_raw=" + pwMid +
                       (String)" gap=" + gapMid +
                       (String)"us duty=" + dutyPercent +
                       (String)"% target=50% tol≈" + toleranceDutyPercent + "%");
      }

      // Do not early-exit on first in-tolerance sample; we want at least a
      // couple of near-target measurements before deciding, or until the
      // bracket can no longer be refined.

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

  // Choose the best PW from the valid samples table. We now:
  //  1) Use the smallest gap to the target as the primary ranking.
  //  2) For each candidate (best to worse), run a lock-in phase that demands
  //     3 consecutive in-band readings at that PW.
  //  3) If all candidates fail lock-in, keep the previous PW_CENTER.
  if (validCount > 0) {
    // Try candidates from best gap to worse, without keeping an explicit
    // rejected list: after each failed lock-in, we mark that candidate by
    // inflating its gap difference so it won't be chosen again.
    for (int attempt = 0; attempt < validCount; ++attempt) {
      int   bestIdx = -1;
      double bestAbs = 1e12;
      int   inTolForThisPass = 0;

      // Find current best candidate and count in-band samples.
      for (int vi = 0; vi < validCount; ++vi) {
        double curAbs = fabs(validGapDiff[vi]);
        if (curAbs <= (double)targetGap) {
          inTolForThisPass++;
        }
        if (curAbs < bestAbs) {
          bestAbs = curAbs;
          bestIdx = vi;
        }
      }

      if (bestIdx < 0) {
        break;
      }

      // If the best gap is still extremely large compared to the allowed gap
      // (e.g. > 10x), abort early and keep the previous PW center. We no
      // longer require a minimum number of in-band coarse samples here,
      // because the lock-in phase will enforce stability.
      if (bestAbs > (double)targetGap * 10.0) {
        if (autotuneDebug >= 1) {
          Serial.println((String)"[PW_CENTER_ABORT] note=" + DCO_calibration_current_note +
                         (String)" DCO=" + currentDCO +
                         (String)" bestGap=" + bestAbs +
                         (String)"us (> " + (double)targetGap * 10.0 +
                         (String)"us); keeping PW_center=" + PW_CENTER[currentDCO / 2]);
        }
        return PWCalibrationVal;
      }

      uint16_t chosenPW = validPW[bestIdx];
      // Reconstruct the gap for the chosen sample so we can report its duty.
      double chosenGap = gapTarget + validGapDiff[bestIdx];
      double chosenDutyPercent = 0.0;
      if (periodUs > 0.0) {
        double dutyErrorFrac = -chosenGap / (2.0 * periodUs);
        chosenDutyPercent = (0.5 + dutyErrorFrac) * 100.0;
      }

      // Lock-in phase for this candidate PW:
      bool lockedIn = false;
      int consecutiveOk = 0;
      const int kMaxLockInTries = 8;

      for (int li = 0; li < kMaxLockInTries && !lockedIn; ++li) {
        pwm_set_chan_level(PW_PWM_SLICES[currentDCO / 2],
                           pwm_gpio_to_channel(PW_PINS[currentDCO / 2]),
                           chosenPW);
        PW[currentDCO / 2]     = chosenPW;
        g_lastPWMeasurementRaw = chosenPW;
        delay(30);

        GapMeasurement gmLock = measure_gap(2);
        if (gmLock.timedOut || periodUs <= 0.0) {
          consecutiveOk = 0;
          continue;
        }

        double gapLock = (double)gmLock.value;
        double gapDiffLock = gapLock - gapTarget;
        double absGapDiffLock = fabs(gapDiffLock);

        if (absGapDiffLock <= (double)targetGap) {
          consecutiveOk++;
          if (consecutiveOk >= 3) {
            lockedIn = true;
            chosenGap = gapLock;
            if (periodUs > 0.0) {
              double dutyErrorFrac = -chosenGap / (2.0 * periodUs);
              chosenDutyPercent = (0.5 + dutyErrorFrac) * 100.0;
            }
            break;
          }
        } else {
          consecutiveOk = 0;
        }
      }

      if (lockedIn) {
        // Local refinement: probe a small neighbourhood around the locked-in PW
        // (PW-2..PW+2). For each candidate in this window, we also require a
        // mini lock-in: 3 consecutive measurements within the target gap band
        // at that PW before we consider it.
        uint16_t bestLocalPW = chosenPW;
        double   bestLocalGapAbs = bestAbs;

        int16_t startOffset = -2;
        int16_t endOffset   =  2;
        for (int16_t off = startOffset; off <= endOffset; ++off) {
          int32_t testPW32 = (int32_t)chosenPW + off;
          if (testPW32 < (int32_t)pwMin || testPW32 > (int32_t)pwMax) continue;
          uint16_t testPW = (uint16_t)testPW32;

          bool   localLocked = false;
          int    localConsecutiveOk = 0;
          double gapLocal = 0.0;
          const int kMaxLocalLockInTries = 8;

          for (int lli = 0; lli < kMaxLocalLockInTries && !localLocked; ++lli) {
            pwm_set_chan_level(PW_PWM_SLICES[currentDCO / 2],
                               pwm_gpio_to_channel(PW_PINS[currentDCO / 2]),
                               testPW);
            PW[currentDCO / 2]     = testPW;
            g_lastPWMeasurementRaw = testPW;
            delay(30);

            GapMeasurement gmLocal = measure_gap(2);
            if (gmLocal.timedOut || periodUs <= 0.0) {
              localConsecutiveOk = 0;
              continue;
            }

            gapLocal = (double)gmLocal.value;
            double gapDiffLocal = gapLocal - gapTarget;
            double absGapDiffLocal = fabs(gapDiffLocal);

            if (absGapDiffLocal <= (double)targetGap) {
              localConsecutiveOk++;
              if (localConsecutiveOk >= 3) {
                localLocked = true;
                double dutyErrorFracLocal = -gapLocal / (2.0 * periodUs);
                double dutyPercentLocal = (0.5 + dutyErrorFracLocal) * 100.0;
                (void)dutyPercentLocal; // only used implicitly via bestLocalGapAbs

                if (absGapDiffLocal < bestLocalGapAbs) {
                  bestLocalGapAbs = absGapDiffLocal;
                  bestLocalPW     = testPW;
                  chosenGap       = gapLocal;
                }
                break;
              }
            } else {
              localConsecutiveOk = 0;
            }
          }
        }

        chosenPW = bestLocalPW;
        if (periodUs > 0.0) {
          double dutyErrorFrac = -chosenGap / (2.0 * periodUs);
          chosenDutyPercent = (0.5 + dutyErrorFrac) * 100.0;
        }

        if (autotuneDebug >= 1) {
          Serial.println((String)"[PW_CENTER_RESULT] note=" + DCO_calibration_current_note +
                         (String)" DCO=" + currentDCO +
                         (String)" PW_center=" + chosenPW +
                         (String)" duty≈" + chosenDutyPercent +
                         (String)"% bestGap=" + bestLocalGapAbs +
                         (String)"us inTolSamples=" + inTolForThisPass +
                         (String)" totalValid=" + validCount);
        }
        return chosenPW;
      }

      // This candidate failed lock-in; inflate its gap diff so we try the next
      // best one on the following attempt.
      validGapDiff[bestIdx] = (double)targetGap * 20.0;
      if (autotuneDebug >= 1) {
        Serial.println((String)"[PW_CENTER_LOCKIN_REJECT] note=" + DCO_calibration_current_note +
                       (String)" DCO=" + currentDCO +
                       (String)" PW=" + chosenPW +
                       (String)" could not get 3 consecutive in-band readings; trying next candidate.");
      }
    }

    // If we reach here, no candidate passed lock-in.
    if (autotuneDebug >= 1) {
      Serial.println((String)"[PW_CENTER_ABORT] note=" + DCO_calibration_current_note +
                     (String)" DCO=" + currentDCO +
                     (String)" all candidates failed lock-in; keeping PW_center=" +
                     PW_CENTER[currentDCO / 2]);
    }
    return PWCalibrationVal;
  } else {
    // No valid samples at all in the searched range: keep the existing PWCalibrationVal
    // and log the situation so the user can investigate.
    if (autotuneDebug >= 1) {
      Serial.println("PW search: no valid samples found; keeping current PWCalibrationVal.");
    }
    return PWCalibrationVal;
  }
}

// Locate PW center for the current DCO's voice by minimizing duty-cycle error
// at a reference note. Mode 0 = low note, mode 1 = higher note refinement.
void find_PW_center(uint8_t mode) {

  DCO_calibration_current_note = manual_DCO_calibration_start_note;
  VOICE_NOTES[0] = DCO_calibration_current_note;
  ampCompCalibrationVal = initManualAmpCompCalibrationVal[currentDCO] + manualCalibrationOffset[currentDCO];

  uint16_t targetGap;
  uint8_t voiceTaskMode;
  

  if (mode == 0) {
    targetGap = compute_gap_tolerance_for_freq(sNotePitches[DCO_calibration_current_note - 12], 0.005);
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

  

  if (firstTuneFlag == true) {
    PW[currentDCO / 2] = DIV_COUNTER_PW / 2;
    PWCalibrationVal = DIV_COUNTER_PW / 2;
    PW_CENTER[currentDCO / 2] = DIV_COUNTER_PW / 2;
  } else {

    PW[currentDCO / 2] = PW_CENTER[currentDCO / 2];
    PWCalibrationVal = PW_CENTER[currentDCO / 2];
  }

  voice_task_autotune(voiceTaskMode, ampCompCalibrationVal);

  uint16_t centerPW = find_PW_for_target_duty(
    kPWCenterDutyFraction,
    targetGap,
    0,
    DIV_COUNTER_PW
  );
  Serial.println("PW center found !!!");
  update_FS_PWCenter(currentDCO / 2, centerPW);
  PW_CENTER[currentDCO / 2] = centerPW;
}

/////////////////////////////////
////////////////////////////////
////////////////////////////////


// Unified PW limit finder. Based on the original low-limit logic, extended so
// that it can search either toward 0 (LOW) or toward DIV_COUNTER_PW (HIGH)
// depending on 'dir', and update the corresponding PW_*_LIMIT and FS entry.
void find_PW_limit(PWLimitDir dir) {
  uint8_t voiceTaskMode = 2;

  DCO_calibration_current_note = manual_DCO_calibration_start_note;
  VOICE_NOTES[0] = DCO_calibration_current_note;
  ampCompCalibrationVal = initManualAmpCompCalibrationVal[currentDCO] + manualCalibrationOffset[currentDCO];

  currentNoteCalibrationStart = micros();
  DCOCalibrationStart = millis();

  PIDOutputLowerLimit = 0;
  PIDOutputHigherLimit = DIV_COUNTER_PW;

  double freqHz   = (double)sNotePitches[DCO_calibration_current_note - 12];
  double periodUs = (freqHz > 0.0) ? (1000000.0 / freqHz) : 0.0;

  uint8_t  voiceIdx  = currentDCO / 2;
  uint16_t centerPW  = PW_CENTER[voiceIdx];

  // Hard bounds for this search so we never scan on both sides of center
  // in a single pass.
  uint16_t minPW = (dir == PW_LIMIT_LOW)  ? 0           : centerPW;
  uint16_t maxPW = (dir == PW_LIMIT_LOW)  ? centerPW    : DIV_COUNTER_PW;

  // Direction-dependent target duty and log labels.
  double targetDuty = (dir == PW_LIMIT_LOW) ? kPWLowDutyFraction : kPWHighDutyFraction;

  // Update global logging context for gap measurements during PW-limit search.
  g_gapLogCurrentPeriodUs    = periodUs;
  g_gapLogTargetDutyFraction = targetDuty;

  double finalDutyPercent = 666.66;

  // Configure the DCO for PW calibration mode.
  voice_task_autotune(voiceTaskMode, ampCompCalibrationVal);
  delay(100);

  const char *scanTag       = (dir == PW_LIMIT_LOW) ? "[PW_LOW_SCAN]"          : "[PW_HIGH_SCAN]";
  const char *refCrossTag   = (dir == PW_LIMIT_LOW) ? "[PW_LOW_REFINE_CROSS]"  : "[PW_HIGH_REFINE_CROSS]";
  const char *refTimeoutTag = (dir == PW_LIMIT_LOW) ? "[PW_LOW_REFINE_TIMEOUT]": "[PW_HIGH_REFINE_TIMEOUT]";
  const char *scanFineTag   = (dir == PW_LIMIT_LOW) ? "[PW_LOW_SCAN_FINE]"     : "[PW_HIGH_SCAN_FINE]";
  const char *rescanFineTag = (dir == PW_LIMIT_LOW) ? "[PW_LOW_RESCAN_FINE]"   : "[PW_HIGH_RESCAN_FINE]";
  const char *abortTag      = (dir == PW_LIMIT_LOW) ? "[PW_LOW_ABORT_NO_SIGNAL]": "[PW_HIGH_ABORT_NO_SIGNAL]";
  const char *resultTag     = (dir == PW_LIMIT_LOW) ? "[PW_LOW_RESULT]"        : "[PW_HIGH_RESULT]";

  // Step size for scanning from center.
  uint16_t step = DIV_COUNTER_PW / 64;
  if (step == 0) step = 1;

  bool     havePrevValid   = false;
  double   prevDuty        = 0.0;
  uint16_t prevPW          = centerPW;

  uint16_t limitPW         = centerPW;  // Final chosen low/high limit.

  bool   found             = false;
  int    totalValidCount   = 0;   // Count of all valid (non-timeout) samples across scans.
  int    inToleranceCount  = 0;   // Valid samples whose duty is near the target duty.
  int    inToleranceRequired = 3; // Minimum number of in-tolerance samples required to accept the result.

  // Track the best coarse sample (closest to target) seen so far so that
  // on signal loss we can refine starting from the globally best-known PW.
  bool     haveBestCoarse  = false;
  double   bestCoarseDelta = 1e12;
  uint16_t bestCoarsePW    = centerPW;

  // Coarse scan from center toward the selected direction.
  for (uint16_t pw = centerPW; ; ) {
    if (millis() - DCOCalibrationStart > 60000) {
      Serial.println((dir == PW_LIMIT_LOW)
                     ? "PW low limit search timeout (60s)"
                     : "PW high limit search timeout (60s)");
      break;
    }

    // Clamp pw to the allowed side of center before applying it.
    if (pw < minPW) pw = minPW;
    if (pw > maxPW) pw = maxPW;

    pwm_set_chan_level(PW_PWM_SLICES[voiceIdx],
                       pwm_gpio_to_channel(PW_PINS[voiceIdx]),
                       pw);
    // Keep PW[] and debug tracker in sync so [GAP_MEASURE]/[GAP_TIMEOUT] log
    // the actual PW value we are testing.
    PW[voiceIdx]           = pw;
    g_lastPWMeasurementRaw = pw;
    delay(30);

    GapMeasurement gm = measure_gap(2);
    if (!gm.timedOut && periodUs > 0.0) {
      totalValidCount++;
      double gap = (double)gm.value;
      double dutyErrorFrac = -gap / (2.0 * periodUs);
      double duty = 0.5 + dutyErrorFrac;

      if (autotuneDebug >= 2) {
        Serial.println((String)scanTag +
                       (String)" note=" + DCO_calibration_current_note +
                       (String)" DCO=" + currentDCO +
                       (String)" PW_raw=" + pw +
                       (String)" duty=" + (duty * 100.0) + "%" +
                       (String)" targetDuty=" + (targetDuty * 100.0) + "%");
      }

      // Update global best coarse sample w.r.t. target.
      double deltaCoarse = fabs(duty - targetDuty);
      if (!haveBestCoarse || deltaCoarse < bestCoarseDelta) {
        haveBestCoarse  = true;
        bestCoarseDelta = deltaCoarse;
        bestCoarsePW    = pw;
      }

      // Count samples that are close to the target duty as "in tolerance".
      if (fabs(duty - targetDuty) <= kPWLimitDutyTolerance) {
        inToleranceCount++;
    }

      bool crossed = (dir == PW_LIMIT_LOW) ? (duty <= targetDuty)
                                           : (duty >= targetDuty);
      bool prevOnOtherSide = (dir == PW_LIMIT_LOW)
                             ? (havePrevValid && prevDuty > targetDuty)
                             : (havePrevValid && prevDuty < targetDuty);

      if (crossed) {
        // We have reached or passed the target duty.
        if (prevOnOtherSide) {
          // We have a coarse "bracket" [prevPW .. pw] where duty crosses
          // the target. Refine with step=1 to find the PW that gets as
          // close as possible to the target.
          uint16_t bestPWLocal   = prevPW;
          double   bestDeltaLocal= fabs(prevDuty - targetDuty);

          if (autotuneDebug >= 2) {
            Serial.println((String)refCrossTag +
                           (String)" note=" + DCO_calibration_current_note +
                           (String)" DCO=" + currentDCO +
                           (String)" prevPW=" + prevPW + " dutyPrev=" + (prevDuty * 100.0) +
                           (String)"% pwHit=" + pw + " dutyHit=" + (duty * 100.0) + "%");
          }

          if (dir == PW_LIMIT_LOW) {
            for (uint16_t pwFine = prevPW; pwFine >= pw; --pwFine) {
              if (pwFine < minPW) break;
              pwm_set_chan_level(PW_PWM_SLICES[voiceIdx],
                                 pwm_gpio_to_channel(PW_PINS[voiceIdx]),
                                 pwFine);
              PW[voiceIdx]           = pwFine;
              g_lastPWMeasurementRaw = pwFine;
              delay(30);

              GapMeasurement gmFine = measure_gap(2);
              if (gmFine.timedOut || periodUs <= 0.0) {
                if (autotuneDebug >= 3) {
                  Serial.println((String)"[PW_REFINE_GAP_TIMEOUT] PW_raw=" + pwFine);
                }
                continue;
              }

              double gapFine = (double)gmFine.value;
              double dutyErrorFracFine = -gapFine / (2.0 * periodUs);
              double dutyFine = 0.5 + dutyErrorFracFine;
              double deltaFine = fabs(dutyFine - targetDuty);

              if (autotuneDebug >= 3) {
                Serial.println((String)"[PW_REFINE_SAMPLE] PW_raw=" + pwFine +
                               (String)" duty=" + (dutyFine * 100.0) + "%" +
                               (String)" |Δ|=" + (deltaFine * 100.0) + "%");
              }

              if (deltaFine < bestDeltaLocal) {
                bestDeltaLocal = deltaFine;
                bestPWLocal    = pwFine;
              }

              if (pwFine == 0) {
                break;
              }
            }
          } else { // PW_LIMIT_HIGH
            for (uint16_t pwFine = prevPW; pwFine <= pw; ++pwFine) {
              if (pwFine > maxPW) break;
              pwm_set_chan_level(PW_PWM_SLICES[voiceIdx],
                                 pwm_gpio_to_channel(PW_PINS[voiceIdx]),
                                 pwFine);
              PW[voiceIdx]           = pwFine;
              g_lastPWMeasurementRaw = pwFine;
              delay(30);

              GapMeasurement gmFine = measure_gap(2);
              if (gmFine.timedOut || periodUs <= 0.0) {
                if (autotuneDebug >= 3) {
                  Serial.println((String)"[PW_REFINE_GAP_TIMEOUT] PW_raw=" + pwFine);
                }
                continue;
              }

              double gapFine = (double)gmFine.value;
              double dutyErrorFracFine = -gapFine / (2.0 * periodUs);
              double dutyFine = 0.5 + dutyErrorFracFine;
              double deltaFine = fabs(dutyFine - targetDuty);

              if (autotuneDebug >= 3) {
                Serial.println((String)"[PW_REFINE_SAMPLE] PW_raw=" + pwFine +
                               (String)" duty=" + (dutyFine * 100.0) + "%" +
                               (String)" |Δ|=" + (deltaFine * 100.0) + "%");
              }

              if (deltaFine < bestDeltaLocal) {
                bestDeltaLocal = deltaFine;
                bestPWLocal    = pwFine;
              }

              if (pwFine >= DIV_COUNTER_PW - 1) {
                break;
              }
            }
          }

          limitPW = bestPWLocal;
      } else {
          // No previous valid sample on the other side of target; use current PW.
          limitPW = pw;
            }
            found = true;
            break;
          }

      // Still on the same side of the target.
          havePrevValid = true;
          prevDuty = duty;
          prevPW = pw;
    } else if (gm.timedOut && havePrevValid && haveBestCoarse) {
      // We stepped from a valid PW into a region with no measurable signal.
      // Go back to the best-known coarse PW (closest to the target so far)
      // and refine with step=1 toward the search boundary, optimising
      // |duty - targetDuty|.
      uint16_t bestPWLocal    = bestCoarsePW;
      double   bestDeltaLocal = bestCoarseDelta;

    if (autotuneDebug >= 1) {
        Serial.println((String)refTimeoutTag +
                       (String)" note=" + DCO_calibration_current_note +
                     (String)" DCO=" + currentDCO +
                       (String)" startPW=" + bestCoarsePW);
      }

      if (dir == PW_LIMIT_LOW) {
        for (uint16_t pwFine = bestCoarsePW; pwFine > 0; --pwFine) {
          if (pwFine < minPW) break;
          pwm_set_chan_level(PW_PWM_SLICES[voiceIdx],
                             pwm_gpio_to_channel(PW_PINS[voiceIdx]),
                             pwFine);
          PW[voiceIdx]           = pwFine;
          g_lastPWMeasurementRaw = pwFine;
          delay(30);

          GapMeasurement gmFine = measure_gap(2);
          if (gmFine.timedOut || periodUs <= 0.0) {
            // Once we start consistently losing signal we can stop refining.
            continue;
          }

          double gapFine = (double)gmFine.value;
          double dutyErrorFracFine = -gapFine / (2.0 * periodUs);
          double dutyFine = 0.5 + dutyErrorFracFine;
          double deltaFine = fabs(dutyFine - targetDuty);

          if (deltaFine < bestDeltaLocal) {
            bestDeltaLocal = deltaFine;
            bestPWLocal    = pwFine;
          }

          if (pwFine == 0) {
            break;
          }
        }
      } else { // PW_LIMIT_HIGH
        for (uint16_t pwFine = bestCoarsePW; pwFine < DIV_COUNTER_PW; ++pwFine) {
          if (pwFine > maxPW) break;
          pwm_set_chan_level(PW_PWM_SLICES[voiceIdx],
                             pwm_gpio_to_channel(PW_PINS[voiceIdx]),
                             pwFine);
          PW[voiceIdx]           = pwFine;
          g_lastPWMeasurementRaw = pwFine;
          delay(30);

          GapMeasurement gmFine = measure_gap(2);
          if (gmFine.timedOut || periodUs <= 0.0) {
            continue;
          }

          double gapFine = (double)gmFine.value;
          double dutyErrorFracFine = -gapFine / (2.0 * periodUs);
          double dutyFine = 0.5 + dutyErrorFracFine;
          double deltaFine = fabs(dutyFine - targetDuty);

          if (deltaFine < bestDeltaLocal) {
            bestDeltaLocal = deltaFine;
            bestPWLocal    = pwFine;
          }

          if (pwFine >= DIV_COUNTER_PW - 1) {
            break;
          }
        }
      }

      limitPW = bestPWLocal;
      found = true;
      break;
    }

    // Step toward the selected boundary.
    if (dir == PW_LIMIT_LOW) {
      if (pw <= minPW + step) {
        // Reached or about to reach 0; stop.
        break;
      }
      pw = (uint16_t)(pw - step);
    } else {
    if (pw >= DIV_COUNTER_PW - step) {
      // Reached or about to reach maximum; stop.
      break;
    }
    pw = (uint16_t)(pw + step);
    }
  }

  // If coarse scan from center failed to find a threshold and we only
  // ever saw a single valid sample (typically at the center), try again with a
  // finer step size before giving up.
  if (!found && totalValidCount <= 1 && step > 1) {
    uint16_t fineStep = step / 4;
    if (fineStep == 0) fineStep = 1;
    if (fineStep != step) {
    if (autotuneDebug >= 1) {
        Serial.println((String)rescanFineTag +
                       (String)" centerPW=" + centerPW +
                       (String)" coarseStep=" + step +
                       (String)" fineStep=" + fineStep);
    }
      // Reset per-scan state for finer scan. We keep totalValidCount so that
      // any valid center sample from the coarse pass is still counted.
      havePrevValid = false;
      prevDuty = 0.0;
      prevPW = centerPW;

      for (uint16_t pw = centerPW; ; ) {
        if (millis() - DCOCalibrationStart > 60000) {
          Serial.println((dir == PW_LIMIT_LOW)
                         ? "PW low limit search timeout (60s, fine scan)"
                         : "PW high limit search timeout (60s, fine scan)");
          break;
        }

        // Clamp pw to the allowed side of center before applying it.
        if (pw < minPW) pw = minPW;
        if (pw > maxPW) pw = maxPW;

        pwm_set_chan_level(PW_PWM_SLICES[voiceIdx],
                           pwm_gpio_to_channel(PW_PINS[voiceIdx]),
                           pw);
        PW[voiceIdx] = pw;
        delay(30);

        GapMeasurement gmFine = measure_gap(2);
        if (!gmFine.timedOut && periodUs > 0.0) {
          totalValidCount++;
          double gap = (double)gmFine.value;
          double dutyErrorFrac = -gap / (2.0 * periodUs);
          double duty = 0.5 + dutyErrorFrac;

          if (autotuneDebug >= 2) {
            Serial.println((String)scanFineTag +
                           (String)" PW=" + pw +
                           (String)" gap=" + gap +
                           (String)"us duty=" + (duty * 100.0) + "%");
          }

          if (fabs(duty - targetDuty) <= kPWLimitDutyTolerance) {
            inToleranceCount++;
          }

          bool crossed = (dir == PW_LIMIT_LOW) ? (duty <= targetDuty)
                                               : (duty >= targetDuty);
          bool prevOnOtherSide = (dir == PW_LIMIT_LOW)
                                 ? (havePrevValid && prevDuty > targetDuty)
                                 : (havePrevValid && prevDuty < targetDuty);

          if (crossed) {
            if (prevOnOtherSide) {
              limitPW = (uint16_t)((prevPW + pw) / 2);
      } else {
              limitPW = pw;
            }
            found = true;
            break;
          }

          havePrevValid = true;
          prevDuty = duty;
          prevPW = pw;
        } else if (gmFine.timedOut && havePrevValid) {
          // Stepping from a valid PW into a timeout region means we've
          // crossed the practical edge. Use the last valid PW as the limit.
          limitPW = prevPW;
          found = true;
          break;
        }

        // Step toward the selected boundary using fineStep.
        if (dir == PW_LIMIT_LOW) {
          if (pw <= minPW + fineStep) {
            break;
          }
          pw = (uint16_t)(pw - fineStep);
        } else {
        if (pw >= DIV_COUNTER_PW - fineStep) {
          break;
        }
        pw = (uint16_t)(pw + fineStep);
        }
      }
    }
  }

  // After coarse + optional fine scan, decide whether to update the limit.
  // If we never saw any valid sample at all, keep the previous limit.
  if (totalValidCount == 0) {
    if (autotuneDebug >= 1) {
      Serial.println((String)abortTag +
                     (String)" note=" + DCO_calibration_current_note +
                     (String)" DCO=" + currentDCO +
                     (String)" totalValid=0 keeping " +
                     (dir == PW_LIMIT_LOW ? "PW_LOW_LIMIT=" : "PW_HIGH_LIMIT=") +
                     (dir == PW_LIMIT_LOW ? PW_LOW_LIMIT[voiceIdx] : PW_HIGH_LIMIT[voiceIdx]));
    }
    return;
  }

  // If we never formally "found" a crossing or timeout edge, fall back to the
  // last valid PW we saw while scanning.
  if (!found && havePrevValid) {
    limitPW = prevPW;
  }

  if (autotuneDebug >= 1) {
    // Recompute duty at the chosen limitPW for logging, if possible.
    finalDutyPercent = -1.0;
    if (periodUs > 0.0) {
      pwm_set_chan_level(PW_PWM_SLICES[voiceIdx],
                         pwm_gpio_to_channel(PW_PINS[voiceIdx]),
                         limitPW);
      PW[voiceIdx] = limitPW;
      delay(30);
      GapMeasurement gmFinal = measure_gap(2);
      if (!gmFinal.timedOut) {
        double gap = (double)gmFinal.value;
        double dutyErrorFrac = -gap / (2.0 * periodUs);
        finalDutyPercent = (0.5 + dutyErrorFrac) * 100.0;
      }
    }
    Serial.println((String)resultTag +
                   (String)" note=" + DCO_calibration_current_note +
                   (String)" DCO=" + currentDCO +
                   (String)" PW_LIMIT=" + limitPW +
                   (String)" duty≈" + finalDutyPercent + "%" +
                   (String)" targetDuty=" + (targetDuty * 100.0) + "%");
  }

  if (dir == PW_LIMIT_LOW) {
    Serial.println("--------------------------------");
    Serial.println("PW low limit found !!!");
    Serial.println(
                   (String)" PW_LIMIT=" + limitPW +
                   (String)" duty≈" + finalDutyPercent + "%" +
                   (String)" targetDuty=" + (targetDuty * 100.0) + "%");    
    Serial.println("--------------------------------");
    update_FS_PW_Low_Limit(voiceIdx, limitPW);
    PW_LOW_LIMIT[voiceIdx] = limitPW;
  } else {
    Serial.println("--------------------------------");
  Serial.println("PW high limit found !!!");
    Serial.println(
                   (String)" PW_LIMIT=" + limitPW +
                   (String)" duty≈" + finalDutyPercent + "%" +
                   (String)" targetDuty=" + (targetDuty * 100.0) + "%");    
    Serial.println("--------------------------------");
    update_FS_PW_High_Limit(voiceIdx, limitPW);
    PW_HIGH_LIMIT[voiceIdx] = limitPW;
  }
}

// Measure duty-cycle error on DCO_calibration_pin by timing rising/falling
// edges. Returns 0 when duty is ≈50%, or kGapTimeoutSentinel on timeout.
float find_gap(byte specialMode) {
  if (specialMode == 2) {  // find lowest freq mode
    samplesNumber = 14;
  } else {
    samplesNumber = 10;
  }

  // Reset edge-timing accumulators and counters at the start of each
  // measurement to avoid leaking partial sums from previous calls.
  pulseCounter         = 0;
  samplesCounter       = 0;
  risingEdgeTimeSum    = 0;
  fallingEdgeTimeSum   = 0;
  edgeDetectionLastVal = 0;

  // Local counters for how many rising/falling segments we actually measured.
  uint16_t risingCount  = 0;
  uint16_t fallingCount = 0;

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
        Serial.println((String)"[GAP_TIMEOUT] note=" + DCO_calibration_current_note +
                       (String)" DCO=" + currentDCO +
                       (String)" PW_raw=" + PW[currentDCO / 2] +
                       (String)" ampComp=" + ampCompCalibrationVal);
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
            fallingCount++;
          } else {
            risingEdgeTimeSum += microsNow - edgeDetectionLastTime;
            risingCount++;
          }
          samplesCounter++;
        }
        edgeDetectionLastTime = microsNow;
        pulseCounter++;
      }
    }
  }

  if (samplesCounter == samplesNumber) {

    // Compute average low and high segment durations directly from the number
    // of segments we actually accumulated, instead of dividing by an assumed
    // fraction of samplesNumber. This removes the legacy scaling factor that
    // distorted gap measurements, especially near 0% / 100% duty.
    float avgLowUs  = (fallingCount  > 0) ? (float)fallingEdgeTimeSum  / (float)fallingCount  : 0.0f;
    float avgHighUs = (risingCount   > 0) ? (float)risingEdgeTimeSum   / (float)risingCount   : 0.0f;

    // Positive DCO_calibration_difference means low segment longer than high
    // (duty < 50%), negative means high segment longer (duty > 50%).
    DCO_calibration_difference = avgLowUs - avgHighUs;

    if (autotuneDebug >= 2) {
      // Log raw gap measurement with context: which mode, note/DCO, the
      // current amplitude compensation value, the last PW we explicitly set,
      // and the inferred duty/target duty if a period is available.
      uint16_t pwRaw = g_lastPWMeasurementRaw;
      double dutyPercent = 0.0;
      double targetDutyPercent = g_gapLogTargetDutyFraction * 100.0;
      if (g_gapLogCurrentPeriodUs > 0.0) {
        double dutyErrorFrac = -(double)DCO_calibration_difference / (2.0 * g_gapLogCurrentPeriodUs);
        dutyPercent = (0.5 + dutyErrorFrac) * 100.0;
      }
      Serial.println((String)"[GAP_MEASURE] mode=" + specialMode +
                     (String)" note=" + DCO_calibration_current_note +
                     (String)" DCO=" + currentDCO +
                     (String)" AMP=" + ampCompCalibrationVal +
                     (String)" PW_raw=" + pwRaw +
                     (String)" diff=" + DCO_calibration_difference +
                     (String)" duty≈" + dutyPercent + "%" +
                     (String)" targetDuty=" + targetDutyPercent + "%");
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