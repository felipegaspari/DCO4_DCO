
#include "include_all.h"
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

  // TURN OFF ALL OSCILLATORS:
  for (int i = 0; i < NUM_OSCILLATORS; i++) {

    uint8_t pioNumber = VOICE_TO_PIO[i];
    PIO pioN = pio[VOICE_TO_PIO[i]];
    uint8_t sm1N = VOICE_TO_SM[i];

    uint32_t clk_div1 = 200;

    pio_sm_put(pioN, sm1N, clk_div1);
    pio_sm_exec(pioN, sm1N, pio_encode_pull(false, false));
    pwm_set_chan_level(RANGE_PWM_SLICES[i], pwm_gpio_to_channel(RANGE_PINS[i]), 0);
  }

  for (int i = 0; i < NUM_VOICES_TOTAL; i += 2) {
    PW[i] = DIV_COUNTER_PW / 2;
    pwm_set_chan_level(PW_PWM_SLICES[i], pwm_gpio_to_channel(PW_PINS[i]), PW[i]);
  }

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
  DCO_calibration_lastTime = 0;
  PIDMinGapCounter = 0;
  pulseCounter = 0;
}

/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/
void DCO_calibration() {

  // TURN OFF ALL OSCILLATORS:
  for (int i = 0; i < NUM_OSCILLATORS; i++) {

    uint8_t pioNumber = VOICE_TO_PIO[i];
    PIO pioN = pio[VOICE_TO_PIO[i]];
    uint8_t sm1N = VOICE_TO_SM[i];

    uint32_t clk_div1 = 200;

    pio_sm_put(pioN, sm1N, clk_div1);
    pio_sm_exec(pioN, sm1N, pio_encode_pull(false, false));
    pwm_set_chan_level(RANGE_PWM_SLICES[i], pwm_gpio_to_channel(RANGE_PINS[i]), 0);
  }

  for (int i = 0; i < NUM_VOICES_TOTAL; i += 2) {
    PW[i] = DIV_COUNTER_PW / 2;
    pwm_set_chan_level(PW_PWM_SLICES[i], pwm_gpio_to_channel(PW_PINS[i]), PW[i]);
  }

  for (int i = 0; i < NUM_OSCILLATORS; i++) {
    currentDCO = i;

    restart_DCO_calibration();

    ampCompCalibrationVal = initManualAmpCompCalibrationVal[currentDCO] + manualCalibrationOffset[currentDCO];
    pwm_set_chan_level(RANGE_PWM_SLICES[i], pwm_gpio_to_channel(RANGE_PINS[i]), ampCompCalibrationVal);

    if ((currentDCO % 2) == 0) {
      if (firstTuneFlag == true) {
        find_PW_center(0);
        pwm_set_chan_level(PW_PWM_SLICES[currentDCO / 2], pwm_gpio_to_channel(PW_PINS[currentDCO / 2]), PW_CENTER[currentDCO / 2]);

      } else {
        find_PW_center(0);  // Should be on. off for testing!!!!!!
        //find_PW_low_limit();
      }
    } else {
      DCO_calibration_current_note = DCO_calibration_start_note;
      VOICE_NOTES[0] = DCO_calibration_current_note;
    }

    // uint16_t lowestFrequency = find_lowest_freq();
    // calibrationData[0] = lowestFrequency;

    bool oscAmpCompCalibrationComplete = false;

    calibrateOscillator();

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
}
/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/

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


  // TURN OFF ALL OSCILLATORS:
  for (int i = 0; i < NUM_OSCILLATORS; i++) {

    uint8_t pioNumber = VOICE_TO_PIO[i];
    PIO pioN = pio[VOICE_TO_PIO[i]];
    uint8_t sm1N = VOICE_TO_SM[i];

    uint32_t clk_div1 = 200;

    pio_sm_put(pioN, sm1N, clk_div1);
    pio_sm_exec(pioN, sm1N, pio_encode_pull(false, false));
    pwm_set_chan_level(RANGE_PWM_SLICES[i], pwm_gpio_to_channel(RANGE_PINS[i]), 0);
  }

  delay(100);

  DCO_calibration_difference = 4000;
  lastDCODifference = 50000;
  lastGapFlipCount = 0;
  lastPIDgap = 50000;
  bestGap = 50000;
  bestCandidate = 50000;
  lastampCompCalibrationVal = 0;
  DCO_calibration_lastTime = 0;
  PIDMinGapCounter = 0;
  pulseCounter = 0;
}

/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/

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

  DCO_calibration_difference = 4000;
  lastDCODifference = 50000;
  lastGapFlipCount = 0;
  lastPIDgap = 50000;
  bestGap = 50000;
  bestCandidate = 50000;
  lastampCompCalibrationVal = 0;
  DCO_calibration_lastTime = 0;
  PIDMinGapCounter = 0;
  pulseCounter = 0;

  while (bestGap > targetGap) {

    pwm_set_chan_level(PW_PWM_SLICES[currentDCO / 2], pwm_gpio_to_channel(PW_PINS[currentDCO / 2]), PWCalibrationVal);

    delay(30);

    double PIDgap;
    double difference;
    double currentGap = find_gap(2);  // distance away from setpoint


    if (currentGap != 1.16999f) {
      PIDgap = abs(currentGap);
      difference = currentGap;
    } else {
      difference = lastDCODifference;
      PIDgap = lastPIDgap;
    }

    if (PIDgap < bestGap) {
      bestGap = PIDgap;
      bestCandidate = PWCalibrationVal;
    }

    bool calibrationSwing = false;

    if ((difference < 0 && lastDCODifference > 0) || (difference > 0 && lastDCODifference < 0)) {
      lastGapFlipCount++;
      if (lastGapFlipCount >= 4) {
        Serial.println("*********************/*/*/*/*/*/  FLIP !!! *********************/*/*/*/*/*/*********");

        calibrationSwing = true;
      }
    } else {
      lastGapFlipCount = 0;
    }

    if (bestGap < targetGap || calibrationSwing == true) {

      PIDMinGapCounter++;

      if (PIDMinGapCounter >= 2) {
        if (autotuneDebug >= 1) {
          Serial.println((String)(String) " - Gap = " + PIDgap + " - MIN Gap: " + (targetGap) + (String) " - " + DCO_calibration_current_note);
        }
        Serial.println((String) "Final gap = " + PIDgap);
        break;
      }
    }

    if (autotuneDebug >= 1) {
      Serial.println((String) " - GAP = " + PIDgap + " - MIN GAP: " + (targetGap) + (String) " -  NOTE: " + DCO_calibration_current_note);
    }

    lastDCODifference = difference;
    lastPIDgap = PIDgap;

    if (difference > 0.00) {
      if (PIDgap >= 4000) {
        PWCalibrationVal += 5;
      } else if (PIDgap > 2000) {
        PWCalibrationVal += 5;
      } else if (PIDgap > 1200) {
        PWCalibrationVal += 2;
      } else {
        PWCalibrationVal++;
      }
    } else if (DCO_calibration_difference < 0.00) {
      if (PIDgap >= 4000) {
        PWCalibrationVal -= 5;
      } else if (PIDgap > 2000) {
        PWCalibrationVal -= 3;
      } else if (PIDgap > 1200) {
        PWCalibrationVal -= 2;
      } else {
        PWCalibrationVal--;
      }
    }
    if (PWCalibrationVal > DIV_COUNTER_PW) { PWCalibrationVal = 0; }

    if (autotuneDebug >= 1) {
      Serial.println((String) "PWCalibrationVal: " + PWCalibrationVal);
    }
  }
  Serial.println("PW center found !!!");
  update_FS_PWCenter(currentDCO / 2, bestCandidate);  //bestCandidate;
  PW_CENTER[currentDCO / 2] = bestCandidate;
}

/////////////////////////////////
////////////////////////////////
////////////////////////////////

void find_PW_low_limit() {
  DCO_calibration_current_note = DCO_calibration_start_note;
  VOICE_NOTES[0] = DCO_calibration_current_note;
  uint16_t targetGap = 1000000.00f / sNotePitches[DCO_calibration_current_note - 12] * 0.01;


  currentNoteCalibrationStart = micros();
  DCOCalibrationStart = millis();
  DCO_calibration_difference = 10000;
  PIDMinGap = 100;

  samplesNumber = 52;

  sampleTime = (1000000 / sNotePitches[DCO_calibration_current_note - 12]) * ((samplesNumber - 1) / 2);

  PW[currentDCO / 2] = PW_CENTER[currentDCO / 2] / 2;
  PWCalibrationVal = PW[currentDCO / 2];

  PIDOutputLowerLimit = 0;
  PIDOutputHigherLimit = DIV_COUNTER_PW;

  PWCalibrationVal = PW[currentDCO / 2];
  pwm_set_chan_level(PW_PWM_SLICES[currentDCO / 2], pwm_gpio_to_channel(PW_PINS[currentDCO / 2]), PW[currentDCO / 2]);

  voice_task_autotune(3, 0);

  delay(100);

  DCO_calibration_difference = 4000;
  lastDCODifference = 50000;
  lastGapFlipCount = 0;
  lastPIDgap = 50000;
  bestGap = 50000;
  bestCandidate = 50000;
  lastampCompCalibrationVal = 0;
  DCO_calibration_lastTime = 0;
  PIDMinGapCounter = 0;
  pulseCounter = 0;

  while (abs(targetGap - DCO_calibration_difference)) {

    pwm_set_chan_level(PW_PWM_SLICES[currentDCO / 2], pwm_gpio_to_channel(PW_PINS[currentDCO / 2]), PWCalibrationVal);

    delay(30);

    find_gap(0);

    double PIDgap = abs(targetGap - DCO_calibration_difference);  // distance away from setpoint

    bool calibrationSwing = false;

    if ((DCO_calibration_difference < 0 && lastDCODifference > 0) || (DCO_calibration_difference > 0 && lastDCODifference < 0)) {
      lastGapFlipCount++;
      if (lastGapFlipCount >= 16) {
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
        Serial.println((String) "Final gap = " + PIDgap);
      }
    }

    if (autotuneDebug >= 1) {
      Serial.println((String) " - GAP = " + PIDgap + " - MIN GAP: " + (PIDMinGap) + (String) " -  NOTE: " + DCO_calibration_current_note);
    }

    lastDCODifference = DCO_calibration_difference;
    lastPIDgap = PIDgap;

    if (DCO_calibration_difference < 0.00) {
      if (DCO_calibration_difference >= 1500) {
        PWCalibrationVal += 5;
      } else if (abs(DCO_calibration_difference) > 1000) {
        PWCalibrationVal += 5;
      } else if (abs(DCO_calibration_difference) > 400) {
        PWCalibrationVal += 2;
      } else {
        PWCalibrationVal++;
      }
    } else if (DCO_calibration_difference > 0.00) {
      if (abs(DCO_calibration_difference) >= 1500) {
        PWCalibrationVal -= 5;
      } else if (abs(DCO_calibration_difference) > 1000) {
        PWCalibrationVal -= 3;
      } else if (abs(DCO_calibration_difference) > 400) {
        PWCalibrationVal -= 2;
      } else {
        PWCalibrationVal--;
      }
    }
    if (PWCalibrationVal > DIV_COUNTER_PW) { PWCalibrationVal = 0; }

    if (autotuneDebug >= 1) {
      Serial.println((String) "PWCalibrationVal: " + PWCalibrationVal);
    }
  }
  update_FS_PW_Low_Limit(currentDCO / 2, PWCalibrationVal);  //bestCandidate?;
  PW_LOW_LIMIT[currentDCO / 2] = PWCalibrationVal;
}

float find_gap(byte specialMode) {
  if (specialMode == 2) {  // find lowest freq mode
    samplesNumber = 15;
  } else {
    samplesNumber = 11;
  }

  // else  if (DCO_calibration_current_note > 31) {
  //   samplesNumber = 51;
  // } else if (DCO_calibration_current_note > 51) {
  //   samplesNumber = 47;
  // } else if (DCO_calibration_current_note > 71) {
  //   samplesNumber = 37;
  // } else if (DCO_calibration_current_note > 91) {
  //   samplesNumber = 21;
  // } else {
  //   samplesNumber = 49;
  // }

  DCO_calibration_lastTime = micros();

  while (pulseCounter < samplesNumber) {

    bool val = digitalRead(DCO_calibration_pin);
    microsNow = micros();
    if ((microsNow - DCO_calibration_lastTime) > 100000) {
      // ampCompCalibrationVal = ampCompCalibrationVal + 1;
      // if (ampCompCalibrationVal > PIDOutputHigherLimit) {

      //ampCompCalibrationVal = PIDLimitsFormula;
      //}
      //voice_task_autotune(0);
      //delay(10);

      pulseCounter = 0;
      DCO_calibration_difference = 1.16999f;
      val = 0;
      DCO_calibration_lastVal = 0;

      if (autotuneDebug >= 3) {
        Serial.println("Timeout loop");
        Serial.println((String) "ampCompCalibrationVal: " + ampCompCalibrationVal);
      }
      // val = digitalRead(DCO_calibration_pin);
      microsNow = micros();
      DCO_calibration_lastTime = microsNow;
      // } else {
      //   PID_dco_calibration();
      // }
      return 1.16999f;
    }
    if (val != DCO_calibration_lastVal) {
      if ((microsNow - DCO_calibration_lastTime) >= 30) {

        DCO_calibration_lastVal = val;

        if (pulseCounter == 1 && val == 0) {
          pulseCounter == 0;
        }
        if (pulseCounter > 2) {
          if (val == 0) {
            DCO_calibration_avg2 += microsNow - DCO_calibration_lastTime;
            DCO_calibration_avg2_counter++;
          } else {
            DCO_calibration_avg1 += microsNow - DCO_calibration_lastTime;
            DCO_calibration_avg1_counter++;
          }
        }
        DCO_calibration_lastTime = microsNow;
        pulseCounter++;
      }
    }
  }

  if (pulseCounter == samplesNumber) {
    //  Serial.print((String) "ON= " + (1000000 / (DCO_calibration_avg1 / 45)) + (String) " - ");
    //  Serial.println((String) "OFF= " + (1000000 / (DCO_calibration_avg2 / 45)));

    // Serial.println(1000000 / ((DCO_calibration_avg1 + DCO_calibration_avg2) / 90));
    DCO_calibration_difference = (DCO_calibration_avg2 / ((samplesNumber - 1) / 4) - (DCO_calibration_avg1 / ((samplesNumber - 1) / 4)));

    if (autotuneDebug >= 1) {
      Serial.println((String) "DCO_calibration_difference: " + DCO_calibration_difference);
      Serial.println((String) "NOTE: " + DCO_calibration_current_note);
    }

    pulseCounter = 0;
    DCO_calibration_avg1 = 0;
    DCO_calibration_avg2 = 0;
    DCO_calibration_lastVal = 0;
    DCO_calibration_avg1_counter = 0;
    DCO_calibration_avg2_counter = 0;

  } else {
    return 1.16999f;
  }
  return (float)DCO_calibration_difference;
}

/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/

void DCO_calibration_find_highest_freq() {

  samplesNumber = 30;

  while (pulseCounter < samplesNumber) {
    bool val = digitalRead(DCO_calibration_pin);
    microsNow = micros();
    // if ((microsNow - DCO_calibration_lastTime) > 10000) {
    //   if (autotuneDebug >= 1) {
    //     Serial.println("return");
    //   }
    //   DCO_calibration_lastTime = 0;
    //   return;
    // }
    if (val != DCO_calibration_lastVal) {
      if ((microsNow - DCO_calibration_lastTime) >= 30) {

        DCO_calibration_lastVal = val;

        if (pulseCounter == 1 && val == 0) {
          pulseCounter == 0;
        }
        if (pulseCounter != 0) {
          if (val == 0) {
            DCO_calibration_avg2 += microsNow - DCO_calibration_lastTime;
            DCO_calibration_avg2_counter++;
          } else {
            DCO_calibration_avg1 += microsNow - DCO_calibration_lastTime;
            DCO_calibration_avg1_counter++;
          }
        }
        if (autotuneDebug >= 4) {
          Serial.println((String) "pulseCounter: " + pulseCounter);
        }
        DCO_calibration_lastTime = microsNow;
        pulseCounter++;
      }
    }
  }

  if (pulseCounter == samplesNumber) {
    //  Serial.print((String) "ON= " + (1000000 / (DCO_calibration_avg1 / 45)) + (String) " - ");
    //  Serial.println((String) "OFF= " + (1000000 / (DCO_calibration_avg2 / 45)));

    // Serial.println(1000000 / ((DCO_calibration_avg1 + DCO_calibration_avg2) / 90));
    DCO_calibration_difference = (DCO_calibration_avg2 / ((float)(samplesNumber - 1) / 4) - (DCO_calibration_avg1 / ((float)(samplesNumber - 1) / 4)));

    if (autotuneDebug >= 1) {
      Serial.println((String) "NOTE: " + DCO_calibration_current_note);
    }

    pulseCounter = 0;
    DCO_calibration_avg1 = 0;
    DCO_calibration_avg2 = 0;
    DCO_calibration_lastVal = 0;
    DCO_calibration_avg1_counter = 0;
    DCO_calibration_avg2_counter = 0;
  }
}

/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/

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

  samplesNumber = 15;

  DCO_calibration_lastTime = micros();

  while (pulseCounter < samplesNumber) {
    bool val = digitalRead(DCO_calibration_pin);
    microsNow = micros();

    if ((microsNow - DCO_calibration_lastTime) > 100000) {

      pulseCounter = 0;
      DCO_calibration_difference = 1.16999f;
      val = 0;
      DCO_calibration_lastVal = 0;

      if (autotuneDebug >= 3) {
        Serial.println("Timeout loop");
        Serial.println((String) "ampCompCalibrationVal: " + ampCompCalibrationVal);
      }

      microsNow = micros();
      DCO_calibration_lastTime = microsNow;
      break;
    }

    if (val != DCO_calibration_lastVal) {
      if ((microsNow - DCO_calibration_lastTime) >= 30) {

        DCO_calibration_lastVal = val;

        if (pulseCounter == 1 && val == 0) {
          pulseCounter == 0;
        }
        if (pulseCounter > 2) {
          if (val == 0) {
            DCO_calibration_avg2 += microsNow - DCO_calibration_lastTime;
            DCO_calibration_avg2_counter++;
          } else {
            DCO_calibration_avg1 += microsNow - DCO_calibration_lastTime;
            DCO_calibration_avg1_counter++;
          }
        }
        DCO_calibration_lastTime = microsNow;
        pulseCounter++;
      }
    }
  }

  if (pulseCounter == samplesNumber) {
    //  Serial.print((String) "ON= " + (1000000 / (DCO_calibration_avg1 / 45)) + (String) " - ");
    //  Serial.println((String) "OFF= " + (1000000 / (DCO_calibration_avg2 / 45)));

    // Serial.println(1000000 / ((DCO_calibration_avg1 + DCO_calibration_avg2) / 90));
    DCO_calibration_difference = (DCO_calibration_avg2 / ((samplesNumber - 1) / 4) - (DCO_calibration_avg1 / ((samplesNumber - 1) / 4)));

    if (autotuneDebug >= 1) {
      Serial.println(DCO_calibration_difference);
      Serial.println((String) "NOTE: " + DCO_calibration_current_note);
    }

    serialSendParam32(154,(int32_t)DCO_calibration_difference);

    pulseCounter = 0;
    DCO_calibration_avg1 = 0;
    DCO_calibration_avg2 = 0;
    DCO_calibration_lastVal = 0;
    DCO_calibration_avg1_counter = 0;
    DCO_calibration_avg2_counter = 0;
  }
}

/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/

void find_vco_frequency() {
  if (DCO_calibration_current_note > 31) {
    samplesNumber = 52;
  } else if (DCO_calibration_current_note > 51) {
    samplesNumber = 48;
  } else if (DCO_calibration_current_note > 71) {
    samplesNumber = 38;
  } else if (DCO_calibration_current_note > 91) {
    samplesNumber = 22;
  } else {
    samplesNumber = 50;
  }

  while (pulseCounter < samplesNumber) {

    bool val = digitalRead(DCO_calibration_pin);
    microsNow = micros();
    if ((microsNow - DCO_calibration_lastTime) > 600000) {
      // ampCompCalibrationVal = ampCompCalibrationVal + 1;
      // if (ampCompCalibrationVal > PIDOutputHigherLimit) {
      ampCompCalibrationVal = PIDLimitsFormula;
      //}
      //voice_task_autotune(0);
      delay(10);

      pulseCounter = 0;
      DCO_calibration_difference = 1500;
      val = 0;
      DCO_calibration_lastVal = 0;

      if (autotuneDebug >= 3) {
        Serial.println("Timeout loop");
        Serial.println((String) "ampCompCalibrationVal: " + ampCompCalibrationVal);
      }
      // val = digitalRead(DCO_calibration_pin);
      microsNow = micros();
      DCO_calibration_lastTime = microsNow;
      // } else {
      //   PID_dco_calibration();
      // }
    }
    if (val != DCO_calibration_lastVal) {
      if ((microsNow - DCO_calibration_lastTime) >= 30) {

        DCO_calibration_lastVal = val;
        // if (pulseCounter == 0) {
        //   DCO_calibration_lastTime = microsNow;
        // }
        if (pulseCounter == 1 && val == 0) {
          pulseCounter == 0;
        }
        if (pulseCounter > 0) {
          if (val == 0) {
            DCO_calibration_avg2 += microsNow - DCO_calibration_lastTime;
            DCO_calibration_avg2_counter++;
          } else {
            DCO_calibration_avg1 += microsNow - DCO_calibration_lastTime;
            DCO_calibration_avg1_counter++;
          }
        }
        DCO_calibration_lastTime = microsNow;
        pulseCounter++;
      }
    }
  }

  if (pulseCounter == samplesNumber) {
    //  Serial.print((String) "ON= " + (1000000 / (DCO_calibration_avg1 / 45)) + (String) " - ");
    //  Serial.println((String) "OFF= " + (1000000 / (DCO_calibration_avg2 / 45)));

    // Serial.println(1000000 / ((DCO_calibration_avg1 + DCO_calibration_avg2) / 90));
    double freqReading = (double)1000000 / double((DCO_calibration_avg1 + DCO_calibration_avg2) / (double)(samplesNumber - 2) * 2);

    DCO_calibration_difference = sNotePitches[DCO_calibration_current_note - 12] - freqReading;

    if (autotuneDebug >= 1) {
      Serial.println((String) "DCO_calibration_difference: " + DCO_calibration_difference + (String) " - Freq reading: " + freqReading);
      Serial.println((String) "NOTE: " + DCO_calibration_current_note + (String) " - NOTE Frequency: " + sNotePitches[DCO_calibration_current_note - 12]);
    }

    pulseCounter = 0;
    DCO_calibration_avg1 = 0;
    DCO_calibration_avg2 = 0;
    DCO_calibration_lastVal = 0;
    DCO_calibration_avg1_counter = 0;
    DCO_calibration_avg2_counter = 0;
  }
}
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