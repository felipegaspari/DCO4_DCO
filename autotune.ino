void init_DCO_calibration() {

  currentDCO = 0;

  VOICE_NOTES[0] = DCO_calibration_start_note;
  DCO_calibration_current_note = DCO_calibration_start_note;

  arrayPos = 0;
  calibrationData[arrayPos] = 0;
  calibrationData[arrayPos + 1] = 0;
  arrayPos += 2;

  calibrationData[arrayPos] = (uint32_t)(sNotePitches[DCO_calibration_current_note - 17] * 100);
  calibrationData[arrayPos + 1] = 60;

  arrayPos += 2;

  currentNoteCalibrationStart = micros();
  DCOCalibrationStart = millis();
  DCO_calibration_difference = 10000;
  PIDMinGap = 300;

  samplesNumber = 21;

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

  voice_task_autotune();

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

void restart_DCO_calibration() {

  currentDCO++;

  if (currentDCO >= NUM_OSCILLATORS) {
    autotuneOnFlag = false;
    init_FS();
    return;
  }

  VOICE_NOTES[0] = DCO_calibration_start_note;
  DCO_calibration_current_note = DCO_calibration_start_note;

  arrayPos = 0;
  calibrationData[arrayPos] = 0;
  calibrationData[arrayPos + 1] = 0;
  arrayPos += 2;

  calibrationData[arrayPos] = (uint32_t)(sNotePitches[DCO_calibration_current_note - 17] * 100);
  calibrationData[arrayPos + 1] = 60;

  arrayPos += 2;

  currentNoteCalibrationStart = micros();
  DCOCalibrationStart = millis();
  DCO_calibration_difference = 10000;
  PIDMinGap = 300;

  samplesNumber = 21;

  sampleTime = (1000000 / sNotePitches[DCO_calibration_current_note - 12]) * ((samplesNumber - 1) / 2);

  PIDLimitsFormula = 75;
  PIDOutputLowerLimit = 70;
  PIDOutputHigherLimit = 100;

  ampCompCalibrationVal = PIDLimitsFormula;

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

  PIDLimitsFormula = 75;
  PIDOutputLowerLimit = 70;
  PIDOutputHigherLimit = 100;

  ampCompCalibrationVal = PIDLimitsFormula;

  voice_task_autotune();

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
  if (DCO_calibration_current_note > 31) {
    samplesNumber = 25 * 2;
  } else if (DCO_calibration_current_note > 51) {
    samplesNumber = 25 * 2;
  } else if (DCO_calibration_current_note > 71) {
    samplesNumber = 19 * 2;
  } else if (DCO_calibration_current_note > 91) {
    samplesNumber = 15 * 2;
  } else {
    samplesNumber = 21 * 2;
  }

  while (pulseCounter < samplesNumber) {

    bool val = digitalRead(DCO_calibration_pin);
    microsNow = micros();
    if ((microsNow - DCO_calibration_lastTime) > 600000) {
      //ampCompCalibrationVal = ampCompCalibrationVal + 1;
      //if (ampCompCalibrationVal > PIDOutputHigherLimit) {
      ampCompCalibrationVal = PIDLimitsFormula;
      //}
      voice_task_autotune();
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

        if (pulseCounter == 1 && val == 0) {
          pulseCounter == 0;
        }
        if (pulseCounter > 3) {
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
    PID_dco_calibration();
  }
}

/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/

void DCO_calibration_find_highest_freq() {

  samplesNumber = 33;

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
  if (DCO_calibration_current_note > 31) {
    samplesNumber = 21;
  } else if (DCO_calibration_current_note > 51) {
    samplesNumber = 17;
  } else if (DCO_calibration_current_note > 71) {
    samplesNumber = 13;
  } else if (DCO_calibration_current_note > 91) {
    samplesNumber = 13;
  } else {
    samplesNumber = 21;
  }

  while (pulseCounter < samplesNumber) {
    bool val = digitalRead(DCO_calibration_pin);
    microsNow = micros();

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

    pulseCounter = 0;
    DCO_calibration_avg1 = 0;
    DCO_calibration_avg2 = 0;
    DCO_calibration_lastVal = 0;
    DCO_calibration_avg1_counter = 0;
    DCO_calibration_avg2_counter = 0;

    // PID_dco_calibration();
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