void init_PID() {
  //initialize the variables we're linked to
  PIDInput = -2000;
  PIDSetpoint = 0;

  myPID.SetMode(AUTOMATIC);
}

void PID_dco_calibration() {

  PIDInput = (double)constrain(DCO_calibration_difference, -1500, 1500);

  double PIDgap = abs(PIDSetpoint - PIDInput);  //distance away from setpoint

  if (micros() - currentNoteCalibrationStart > 15000000 && ampCompCalibrationVal > 8000) {
    Serial.println("Find highest freq");
    Serial.println((String) "Total time: " + (millis() - DCOCalibrationStart));
    autotuneOnFlag = false;
    DCO_calibration_difference = 2;
    PID_find_highest_freq();
    return;

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
    if (lastGapFlipCount >= 8) {
      if (DCO_calibration_current_note > 50) {
        PIDMinGap = PIDMinGap * 1.01;
      } else {
        PIDMinGap = PIDMinGap * 1.05;
      }
    }
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

      calibrationData[arrayPos] = (uint32_t)(sNotePitches[DCO_calibration_current_note - 12] * 100);
      calibrationData[arrayPos + 1] = (uint32_t)ampCompCalibrationVal; //bestCandidate;
      arrayPos += 2;

      Serial.println((uint32_t)(sNotePitches[DCO_calibration_current_note - 12] * 100) + (String) ", " + ampCompCalibrationVal + (String) ",");
      Serial.println((String) "Final gap = " + PIDgap + (String) " |||| NOTE: " + DCO_calibration_current_note + (String) " |||| Note calibration time(s): " + ((micros() - currentNoteCalibrationStart) / 1000000));


      DCO_calibration_current_note = DCO_calibration_current_note + 5;
      VOICE_NOTES[0] = DCO_calibration_current_note;


      if (DCO_calibration_current_note > 31) {
        samplesNumber = 21;
      } else if (DCO_calibration_current_note > 51) {
        samplesNumber = 17;
      } else if (DCO_calibration_current_note > 71) {
        samplesNumber = 15;
      } else if (DCO_calibration_current_note > 91) {
        samplesNumber = 13;
      } else {
        samplesNumber = 21;
      }
      currentNoteCalibrationStart = micros();
      //PIDMinGap = (1240.6114554 * pow(0.9924189, (double)ampCompCalibrationVal)) * 0.05;

      PIDMinGap = (37701.182837 * pow(0.855327, (double)DCO_calibration_current_note));

      PIDLimitsFormula = ((1.364 * (double)(ampCompCalibrationVal)) - 12) * 1.05;
      PIDOutputLowerLimit = PIDLimitsFormula * 0.8;
      PIDOutputHigherLimit = PIDLimitsFormula * 1.05;

      Serial.println((String) "Next MinGap: " + PIDMinGap);
      Serial.println((String) "PIDOutputLowerLimit: " + PIDOutputLowerLimit + (String) " PIDOutputHigherLimit: " + PIDOutputHigherLimit);
      Serial.println("  ----------------------------------------------------------------- ");

      if (PIDOutputHigherLimit >= 9800) {
        Serial.println("Find highest freq");
        Serial.println((String) "Total time: " + ((millis() - DCOCalibrationStart) / 1000));
        autotuneOnFlag = false;
        DCO_calibration_difference = 2;
        PID_find_highest_freq();
        return;
      }

      ampCompCalibrationVal = PIDLimitsFormula;

      sampleTime = (1000000 / sNotePitches[DCO_calibration_current_note - 12]) * ((samplesNumber - 1) / 2);
      if (sampleTime < 8000) sampleTime = 8000;

      voice_task_autotune();
      delay(50);

      //delay(5000); //FIX THIS

      DCO_calibration_lastTime = micros();
      DCO_calibration_difference = 4000;
      lastDCODifference = 50000;
      lastGapFlipCount = 0;
      lastPIDgap = 50000;
      bestGap = 50000;
      bestCandidate= 50000;
      lastampCompCalibrationVal = 0;
      DCO_calibration_lastTime = 0;
      PIDMinGapCounter = 0;

      return;
    }
  }

  if (autotuneDebug >= 1) {
    Serial.println((String) " - GAP = " + PIDgap + " - MIN GAP: " + (PIDMinGap) + (String) " -  NOTE: " + DCO_calibration_current_note);
  }

  lastDCODifference = DCO_calibration_difference;
  lastPIDgap = PIDgap;
  lastampCompCalibrationVal = ampCompCalibrationVal;

  if (DCO_calibration_difference > 0.00) {
    ampCompCalibrationVal++;
  } else if (DCO_calibration_difference < 0.00) {
    ampCompCalibrationVal--;
  }

  //ampCompCalibrationVal = constrain(ampCompCalibrationVal, PIDOutputLowerLimit, PIDOutputHigherLimit);

  if (autotuneDebug >= 1) {
    Serial.println((String) "ampCompCalibrationVal: " + ampCompCalibrationVal + (String) " -- PIDOutputLowerLimit: " + PIDOutputLowerLimit);
    Serial.print((String) " PIDOutputHigherLimit: " + PIDOutputHigherLimit + (String) " - DCO: " + currentDCO);
  }
}

void PID_find_highest_freq() {

  ampCompCalibrationVal = DIV_COUNTER;
  PIDTuningMultiplier = 0.28752775 * pow(1.00408722, 1779);
  PIDTuningMultiplierKi = 0.33936558 * pow(1.00702176, 1779);
  PIDInput = 100;
  myPID.SetOutputLimits(sNotePitches[DCO_calibration_current_note - 15], sNotePitches[DCO_calibration_current_note - 5]);
  myPID.SetTunings(0.035, 6, 0.003);
  myPID.SetSampleTime(5);
  while (abs(DCO_calibration_difference) > 0.5) {
    //Serial.println("a");
    voice_task_autotune();
    delay(4);
    //Serial.println("b");
    DCO_calibration_find_highest_freq();
    //Serial.println("c");
    PIDInput = 0 - (double)DCO_calibration_difference;

    myPID.Compute();

    if (autotuneDebug >= 1) {
      Serial.println((String) "Pid output: " + PIDOutput + (String) " Pid gap: " + DCO_calibration_difference);
    }
  }
  Serial.println((String) "Highest freq found: " + PIDOutput);
  calibrationData[arrayPos] = (uint32_t)(PIDOutput * 100);
  calibrationData[arrayPos + 1] = (uint32_t)DIV_COUNTER;
  arrayPos += 2;

  while (arrayPos < chanLevelVoiceDataSize) {
    calibrationData[arrayPos] = 20000000;
    calibrationData[arrayPos + 1] = DIV_COUNTER;
    arrayPos += 2;
  }

  for (int i = 0; i < chanLevelVoiceDataSize; i++) {
    Serial.println(calibrationData[i]);
  }
  update_FS_voice(currentDCO);

  //find highest note

  for (int i = 0; i < sizeof(sNotePitches); i++) {
    if (PIDOutput > sNotePitches[i] && PIDOutput < sNotePitches[i + 1]) {
      highestNoteOSC[currentDCO] = i;
      Serial.println((String) "Highest note found: " + i + (String) " - Note freq: " + sNotePitches[i]);
      break;
    }
  }

  autotuneOnFlag = true;

  Serial.println((String) "DCO " + currentDCO + (String) " calibration finished.");

  restart_DCO_calibration();
}