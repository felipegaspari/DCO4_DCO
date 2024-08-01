// #include "include_all.h"
// void init_PID() {
//   //initialize the variables we're linked to
//   PIDInput = -2000;
//   PIDSetpoint = 0;

//   myPID.SetMode(AUTOMATIC);
// }

// bool PID_dco_calibration() {

//   PIDInput = (double)constrain(DCO_calibration_difference, -1500, 1500);

//   double PIDgap = abs(PIDSetpoint - PIDInput);  //distance away from setpoint

//   if (micros() - currentNoteCalibrationStart > 15000000 && ampCompCalibrationVal > (DIV_COUNTER * 0.98)) {
//     Serial.println("Find highest freq");
//     Serial.println((String) "Total time: " + (millis() - DCOCalibrationStart));
//     autotuneOnFlag = false;
//     DCO_calibration_difference = 2;
//     return true;

//   } else if (micros() - currentNoteCalibrationStart > 20000000 && micros() - PIDComputeTimer > sampleTime) {
//     PIDMinGap = PIDMinGap * 1.02;
//   } else if (micros() - currentNoteCalibrationStart > 10000000 && micros() - PIDComputeTimer > sampleTime) {
//     PIDMinGap = PIDMinGap * 1.01;
//   }

//   if (PIDgap < bestGap) {
//     bestGap = PIDgap;
//     bestCandidate = ampCompCalibrationVal;
//   }

//   bool calibrationSwing = false;
//   if ((DCO_calibration_difference < 0 && lastDCODifference > 0) || (DCO_calibration_difference > 0 && lastDCODifference < 0)) {
//     lastGapFlipCount++;
//     if (lastGapFlipCount >= 4) {
//       if (DCO_calibration_current_note > 50) {
//         PIDMinGap = PIDMinGap * 1.01;
//       } else {
//         PIDMinGap = PIDMinGap * 1.05;
//       }
//     }
//     if (lastGapFlipCount >= 8) {
//       Serial.println("*********************/*/*/*/*/*/  FLIP !!! *********************/*/*/*/*/*/*********");

//       calibrationSwing = true;
//     }
//   } else {
//     lastGapFlipCount = 0;
//   }

//   if (PIDgap < PIDMinGap || calibrationSwing == true) {

//     PIDMinGapCounter++;

//     if (PIDMinGapCounter >= 2) {
//       if (autotuneDebug >= 1) {
//         Serial.println((String)(String) " - Gap = " + PIDgap + " - MIN Gap: " + (PIDMinGap) + (String) " - " + DCO_calibration_current_note);
//       }

//       calibrationData[arrayPos] = (uint32_t)(sNotePitches[DCO_calibration_current_note - 12] * 100);
//       calibrationData[arrayPos + 1] = (uint32_t)bestCandidate;  //bestCandidate;
//       arrayPos += 2;

//       Serial.println((uint32_t)(sNotePitches[DCO_calibration_current_note - 12] * 100) + (String) ", " + ampCompCalibrationVal + (String) ",");
//       Serial.println((String) "Final gap = " + PIDgap + (String) " |||| NOTE: " + DCO_calibration_current_note + (String) " |||| Note calibration time(s): " + ((micros() - currentNoteCalibrationStart) / 1000000));


//       DCO_calibration_current_note = DCO_calibration_current_note + calibration_note_interval;
//       VOICE_NOTES[0] = DCO_calibration_current_note;

//       currentNoteCalibrationStart = micros();
//       //PIDMinGap = (1240.6114554 * pow(0.9924189, (double)ampCompCalibrationVal)) * 0.05;

//       PIDMinGap = (37701.182837 * pow(0.855327, (double)DCO_calibration_current_note));

//       PIDLimitsFormula = ((1.364 * (double)(ampCompCalibrationVal)) - 12) * 1.05;
//       PIDOutputLowerLimit = PIDLimitsFormula * 0.8;
//       PIDOutputHigherLimit = PIDLimitsFormula * 1.05;

//       Serial.println((String) "Next MinGap: " + PIDMinGap);
//       Serial.println((String) "PIDOutputLowerLimit: " + PIDOutputLowerLimit + (String) " PIDOutputHigherLimit: " + PIDOutputHigherLimit);
//       Serial.println("  ----------------------------------------------------------------- ");

//       if (PIDOutputHigherLimit >= (DIV_COUNTER * 0.98)) {
//         Serial.println("Find highest freq");
//         Serial.println((String) "Total time: " + ((millis() - DCOCalibrationStart) / 1000));
//         autotuneOnFlag = false;
//         DCO_calibration_difference = 2;
//         return true;
//       }

//       ampCompCalibrationVal = PIDLimitsFormula;

//       sampleTime = (1000000 / sNotePitches[DCO_calibration_current_note - 12]) * ((samplesNumber - 1) / 2);
//       if (sampleTime < 8000) sampleTime = 8000;

//       voice_task_autotune(0);
//       delay(50);

//       DCO_calibration_lastTime = micros();
//       DCO_calibration_difference = 4000;
//       lastDCODifference = 50000;
//       lastGapFlipCount = 0;
//       lastPIDgap = 50000;
//       bestGap = 50000;
//       bestCandidate = 50000;
//       lastampCompCalibrationVal = 0;
//       DCO_calibration_lastTime = 0;
//       PIDMinGapCounter = 0;

//       return false;
//     }
//   }

//   if (autotuneDebug >= 1) {
//     Serial.println((String) " - GAP = " + PIDgap + " - MIN GAP: " + (PIDMinGap) + (String) " -  NOTE: " + DCO_calibration_current_note);
//   }

//   lastDCODifference = DCO_calibration_difference;
//   lastPIDgap = PIDgap;
//   lastampCompCalibrationVal = ampCompCalibrationVal;

//   if (DCO_calibration_difference > 0.00) {
//     if (DCO_calibration_difference > PIDMinGap * 20) {
//       ampCompCalibrationVal += 2;
//     } else {
//       ampCompCalibrationVal++;
//     }
//   } else if (DCO_calibration_difference < 0.00) {
//     if (abs(DCO_calibration_difference) > PIDMinGap * 20) {
//       ampCompCalibrationVal -= 2;
//     } else {
//       ampCompCalibrationVal--;
//     }
//   }

//   //ampCompCalibrationVal = constrain(ampCompCalibrationVal, PIDOutputLowerLimit, PIDOutputHigherLimit);

//   if (autotuneDebug >= 1) {
//     Serial.println((String) "ampCompCalibrationVal: " + ampCompCalibrationVal + (String) " -- PIDOutputLowerLimit: " + PIDOutputLowerLimit);
//     Serial.print((String) " PIDOutputHigherLimit: " + PIDOutputHigherLimit + (String) " - DCO: " + currentDCO);
//   }
//   return false;
// }

// void PID_find_highest_freq() {

//   ampCompCalibrationVal = DIV_COUNTER;
//   PIDTuningMultiplier = 0.28752775 * pow(1.00408722, 1779);
//   PIDTuningMultiplierKi = 0.33936558 * pow(1.00702176, 1779);
//   PIDInput = 100;
//   myPID.SetOutputLimits(sNotePitches[DCO_calibration_current_note - 12 - calibration_note_interval ], sNotePitches[DCO_calibration_current_note - 12 + calibration_note_interval]);
//   myPID.SetTunings(0.01, 1, 0.0005);
//   myPID.SetSampleTime(5);
//   while (abs(DCO_calibration_difference) > 0.5) {
//     voice_task_autotune(1);
//     delay(4);
//     find_gap();
//     PIDInput = 0 - (double)DCO_calibration_difference;

//     myPID.Compute();

//     if (autotuneDebug >= 1) {
//       Serial.println((String) "Pid output: " + PIDOutput + (String) " Pid gap: " + DCO_calibration_difference);
//     }
//   }
//   Serial.println((String) "Highest freq found: " + PIDOutput);

//   //find highest note
//   for (int i = 0; i < sizeof(sNotePitches); i++) {
//     if (PIDOutput > sNotePitches[i] && PIDOutput < sNotePitches[i + 1]) {
//       highestNoteOSC[currentDCO] = i;
//       Serial.println((String) "Highest note found: " + i + (String) " - Note freq: " + sNotePitches[i]);
//       break;
//     }
//   }
// }

// const int analogPin = A0; // Analog pin connected to the device
// const int numSamples = 10; // Number of samples to take for each voltage
// const float tolerance = 0.01; // Tolerance for stopping the binary search
// const float minVoltage = 0.0; // Minimum voltage to start calibration
// const float maxVoltage = 5.0; // Maximum voltage to end calibration
// const int rangeSamples = 3; // Number of measurements to store around the sign change
// const int numPresetVoltages = 5; // Number of preset voltages
// const float presetVoltages[numPresetVoltages] = {1.0, 2.0, 3.0, 4.0, 5.0}; // Array of preset voltages
// float bestVoltages[numPresetVoltages] = {0.5, 1.5}; // Array to store the best voltages, with first two values manually set

// float sendVoltage(float voltage);
// float readAnalogValue();
// float averageAnalogValue(int numSamples);
// float quadraticInterpolation(float x0, float y0, float x1, float y1, float x2, float y2, float x);


// void loop() {
//   for (int j = 2; j < numPresetVoltages; j++) { // Start from the 3rd preset voltage
//     float presetVoltage = presetVoltages[j];
//     sendVoltage(presetVoltage); // Send the preset voltage

//     float low = minVoltage;
//     float high = maxVoltage;
//     float bestVoltage = low;
//     float closestToZero = 1023.0; // Initialize with a large value
//     float previousAvgValue = 0.0;

//     float lowerMeasurements[rangeSamples];
//     float higherMeasurements[rangeSamples];
//     float lowerVoltages[rangeSamples];
//     float higherVoltages[rangeSamples];

//     while ((high - low) > tolerance) {
//       float mid = (low + high) / 2.0;
//       sendVoltage(mid);
//       float avgValue = averageAnalogValue(numSamples);
      
//       if (abs(avgValue) < abs(closestToZero)) {
//         closestToZero = avgValue;
//         bestVoltage = mid;
//       }

//       // Detect sign change
//       if ((previousAvgValue > 0 && avgValue < 0) || (previousAvgValue < 0 && avgValue > 0)) {
//         // Store measurements around the midpoint
//         for (int i = 0; i < rangeSamples; i++) {
//           float lowerVoltage = mid - (i + 1) * tolerance;
//           float higherVoltage = mid + (i + 1) * tolerance;

//           sendVoltage(lowerVoltage);
//           lowerMeasurements[i] = averageAnalogValue(numSamples);
//           lowerVoltages[i] = lowerVoltage;

//           sendVoltage(higherVoltage);
//           higherMeasurements[i] = averageAnalogValue(numSamples);
//           higherVoltages[i] = higherVoltage;
//         }

//         // Evaluate stored measurements
//         for (int i = 0; i < rangeSamples; i++) {
//           if (abs(lowerMeasurements[i]) < abs(closestToZero)) {
//             closestToZero = lowerMeasurements[i];
//             bestVoltage = lowerVoltages[i];
//           }
//           if (abs(higherMeasurements[i]) < abs(closestToZero)) {
//             closestToZero = higherMeasurements[i];
//             bestVoltage = higherVoltages[i];
//           }
//         }

//         // Narrow the range around the best voltage found
//         low = bestVoltage - tolerance;
//         high = bestVoltage + tolerance;
//       } else {
//         if (avgValue > 0) {
//           high = mid;
//         } else {
//           low = mid;
//         }
//       }

//       previousAvgValue = avgValue;
//     }

//     bestVoltages[j] = bestVoltage; // Store the best voltage for the current preset voltage

//     Serial.print("Preset voltage: ");
//     Serial.println(presetVoltage);
//     Serial.print("Best calibration voltage: ");
//     Serial.println(bestVoltage);
//     Serial.print("Closest measurement to zero: ");
//     Serial.println(closestToZero);

//     // Add a delay to avoid flooding the serial monitor
//     delay(5000);
//   }

//   // Interpolate the next voltage using the last three best voltages
//   float nextVoltage = quadraticInterpolation(
//     presetVoltages[numPresetVoltages - 3], bestVoltages[numPresetVoltages - 3],
//     presetVoltages[numPresetVoltages - 2], bestVoltages[numPresetVoltages - 2],
//     presetVoltages[numPresetVoltages - 1], bestVoltages[numPresetVoltages - 1],
//     presetVoltages[numPresetVoltages - 1] + 1 // Example: next preset voltage
//   );

//   Serial.print("Interpolated next voltage: ");
//   Serial.println(nextVoltage);

//   // Print all best voltages
//   Serial.println("All best calibration voltages:");
//   for (int i = 0; i < numPresetVoltages; i++) {
//     Serial.print("Preset voltage ");
//     Serial.print(presetVoltages[i]);
//     Serial.print(": ");
//     Serial.println(bestVoltages[i]);
//   }

//   // Add a delay to avoid flooding the serial monitor
//   delay(10000);
// }

// float sendVoltage(float voltage) {
//   // Implement the function to send the specific voltage to your device
//   // This will depend on your specific hardware setup
//   // Example: Using a DAC or PWM to set the voltage
// }

// float readAnalogValue() {
//   int sensorValue = analogRead(analogPin);
//   float voltage = sensorValue * (5.0 / 1023.0);
//   return voltage;
// }

// float averageAnalogValue(int numSamples) {
//   float total = 0.0;
//   for (int i = 0; i < numSamples; i++) {
//     total += readAnalogValue();
//     delay(10); // Small delay between samples
//   }
//   return total / numSamples;
// }

// float quadraticInterpolation(float x0, float y0, float x1, float y1, float x2, float y2, float x) {
//   // Calculate the coefficients of the quadratic polynomial
//   float a = ((y2 - (x2 * (y1 - y0) + x1 * y0 - x0 * y1) / (x1 - x0)) / (x2 * (x2 - x0 - x1) + x0 * x1));
//   float b = ((y1 - y0) / (x1 - x0) - a * (x0 + x1));
//   float c = y0 - x0 * (b + a * x0);

//   // Use the polynomial to estimate the next value
//   return a * x * x + b * x + c;
// }






















// const int analogPin = A0; // Analog pin connected to the device
// const int numSamples = 10; // Number of samples to take for each voltage
// const float tolerance = 0.01; // Tolerance for stopping the binary search
// const float minVoltage = 0.0; // Minimum voltage to start calibration
// const float maxVoltage = 5.0; // Maximum voltage to end calibration
// const int rangeSamples = 3; // Number of measurements to store around the sign change
// const int numPresetVoltages = 5; // Number of preset voltages
// const float presetVoltages[numPresetVoltages] = {1.0, 2.0, 3.0, 4.0, 5.0}; // Array of preset voltages
// float bestVoltages[numPresetVoltages] = {0.5, 1.5}; // Array to store the best voltages, with first two values manually set

// float sendVoltage(float voltage);
// float readAnalogValue();
// float averageAnalogValue(int numSamples);
// float quadraticInterpolation(float x0, float y0, float x1, float y1, float x2, float y2, float x);


// void loop() {
//   for (int j = 2; j < numPresetVoltages; j++) { // Start from the 3rd preset voltage
//     float presetVoltage = presetVoltages[j];
//     sendVoltage(presetVoltage); // Send the preset voltage

//     float low = minVoltage;
//     float high = maxVoltage;
//     float bestVoltage = low;
//     float closestToZero = 1023.0; // Initialize with a large value
//     float previousAvgValue = 0.0;

//     float lowerMeasurements[rangeSamples];
//     float higherMeasurements[rangeSamples];
//     float lowerVoltages[rangeSamples];
//     float higherVoltages[rangeSamples];

//     while ((high - low) > tolerance) {
//       float mid = (low + high) / 2.0;
//       sendVoltage(mid);
//       float avgValue = averageAnalogValue(numSamples);
      
//       if (abs(avgValue) < abs(closestToZero)) {
//         closestToZero = avgValue;
//         bestVoltage = mid;
//       }

//       // Detect sign change
//       if ((previousAvgValue > 0 && avgValue < 0) || (previousAvgValue < 0 && avgValue > 0)) {
//         // Store measurements around the midpoint
//         for (int i = 0; i < rangeSamples; i++) {
//           float lowerVoltage = mid - (i + 1) * tolerance;
//           float higherVoltage = mid + (i + 1) * tolerance;

//           sendVoltage(lowerVoltage);
//           lowerMeasurements[i] = averageAnalogValue(numSamples);
//           lowerVoltages[i] = lowerVoltage;

//           sendVoltage(higherVoltage);
//           higherMeasurements[i] = averageAnalogValue(numSamples);
//           higherVoltages[i] = higherVoltage;
//         }

//         // Evaluate stored measurements
//         for (int i = 0; i < rangeSamples; i++) {
//           if (abs(lowerMeasurements[i]) < abs(closestToZero)) {
//             closestToZero = lowerMeasurements[i];
//             bestVoltage = lowerVoltages[i];
//           }
//           if (abs(higherMeasurements[i]) < abs(closestToZero)) {
//             closestToZero = higherMeasurements[i];
//             bestVoltage = higherVoltages[i];
//           }
//         }

//         // Narrow the range around the best voltage found
//         low = bestVoltage - tolerance;
//         high = bestVoltage + tolerance;
//       } else {
//         if (avgValue > 0) {
//           high = mid;
//         } else {
//           low = mid;
//         }
//       }

//       previousAvgValue = avgValue;
//     }

//     bestVoltages[j] = bestVoltage; // Store the best voltage for the current preset voltage

//     Serial.print("Preset voltage: ");
//     Serial.println(presetVoltage);
//     Serial.print("Best calibration voltage: ");
//     Serial.println(bestVoltage);
//     Serial.print("Closest measurement to zero: ");
//     Serial.println(closestToZero);

//     // Add a delay to avoid flooding the serial monitor
//     delay(5000);
//   }

//   // Interpolate the next voltage using the last three best voltages
//   float nextVoltage = quadraticInterpolation(
//     presetVoltages[numPresetVoltages - 3], bestVoltages[numPresetVoltages - 3],
//     presetVoltages[numPresetVoltages - 2], bestVoltages[numPresetVoltages - 2],
//     presetVoltages[numPresetVoltages - 1], bestVoltages[numPresetVoltages - 1],
//     presetVoltages[numPresetVoltages - 1] + 1 // Example: next preset voltage
//   );

//   Serial.print("Interpolated next voltage: ");
//   Serial.println(nextVoltage);

//   // Print all best voltages
//   Serial.println("All best calibration voltages:");
//   for (int i = 0; i < numPresetVoltages; i++) {
//     Serial.print("Preset voltage ");
//     Serial.print(presetVoltages[i]);
//     Serial.print(": ");
//     Serial.println(bestVoltages[i]);
//   }

//   // Add a delay to avoid flooding the serial monitor
//   delay(10000);
// }

// float sendVoltage(float voltage) {
//   // Implement the function to send the specific voltage to your device
//   // This will depend on your specific hardware setup
//   // Example: Using a DAC or PWM to set the voltage
// }

// float readAnalogValue() {
//   int sensorValue = analogRead(analogPin);
//   float voltage = sensorValue * (5.0 / 1023.0);
//   return voltage;
// }

// float averageAnalogValue(int numSamples) {
//   float total = 0.0;
//   for (int i = 0; i < numSamples; i++) {
//     total += readAnalogValue();
//     delay(10); // Small delay between samples
//   }
//   return total / numSamples;
// }

// float quadraticInterpolation(float x0, float y0, float x1, float y1, float x2, float y2, float x) {
//   // Calculate the coefficients of the quadratic polynomial
//   float a = ((y2 - (x2 * (y1 - y0) + x1 * y0 - x0 * y1) / (x1 - x0)) / (x2 * (x2 - x0 - x1) + x0 * x1));
//   float b = ((y1 - y0) / (x1 - x0) - a * (x0 + x1));
//   float c = y0 - x0 * (b + a * x0);

//   // Use the polynomial to estimate the next value
//   return a * x * x + b * x + c;
// }















// const int analogPin = A0; // Analog pin connected to the device
// const int numSamples = 10; // Number of samples to take for each voltage
// const float tolerance = 0.01; // Tolerance for stopping the adjustment
// const float minVoltage = 0.0; // Minimum voltage to start calibration
// const float maxVoltage = 5.0; // Maximum voltage to end calibration
// const int rangeSamples = 3; // Number of measurements to store around the sign change
// const int numPresetVoltages = 5; // Number of preset voltages
// const float presetVoltages[numPresetVoltages] = {1.0, 2.0, 3.0, 4.0, 5.0}; // Array of preset voltages
// float bestVoltages[numPresetVoltages] = {0.5, 1.5}; // Array to store the best voltages, with first two values manually set

// float sendVoltage(float voltage);
// float readAnalogValue();
// float averageAnalogValue(int numSamples);

// void loop() {
//   for (int j = 2; j < numPresetVoltages; j++) { // Start from the 3rd preset voltage
//     float presetVoltage = presetVoltages[j];
//     sendVoltage(presetVoltage); // Send the preset voltage

//     float currentVoltage = (minVoltage + maxVoltage) / 2.0;
//     float bestVoltage = currentVoltage;
//     float closestToZero = 1023.0; // Initialize with a large value
//     float previousAvgValue = 0.0;

//     float lowerMeasurements[rangeSamples];
//     float higherMeasurements[rangeSamples];
//     float lowerVoltages[rangeSamples];
//     float higherVoltages[rangeSamples];

//     while (true) {
//       sendVoltage(currentVoltage);
//       float avgValue = averageAnalogValue(numSamples);
      
//       if (abs(avgValue) < abs(closestToZero)) {
//         closestToZero = avgValue;
//         bestVoltage = currentVoltage;
//       }

//       // Detect sign change
//       if ((previousAvgValue > 0 && avgValue < 0) || (previousAvgValue < 0 && avgValue > 0)) {
//         // Store measurements around the current voltage
//         for (int i = 0; i < rangeSamples; i++) {
//           float lowerVoltage = currentVoltage - (i + 1);
//           float higherVoltage = currentVoltage + (i + 1);

//           sendVoltage(lowerVoltage);
//           lowerMeasurements[i] = averageAnalogValue(numSamples);
//           lowerVoltages[i] = lowerVoltage;

//           sendVoltage(higherVoltage);
//           higherMeasurements[i] = averageAnalogValue(numSamples);
//           higherVoltages[i] = higherVoltage;
//         }

//         // Evaluate stored measurements including the current voltage
//         for (int i = 0; i < rangeSamples; i++) {
//           if (abs(lowerMeasurements[i]) < abs(closestToZero)) {
//             closestToZero = lowerMeasurements[i];
//             bestVoltage = lowerVoltages[i];
//           }
//           if (abs(higherMeasurements[i]) < abs(closestToZero)) {
//             closestToZero = higherMeasurements[i];
//             bestVoltage = higherVoltages[i];
//           }
//         }

//         // Check the current voltage again
//         if (abs(avgValue) < abs(closestToZero)) {
//           closestToZero = avgValue;
//           bestVoltage = currentVoltage;
//         }

//         // Break the loop if the closest value is within tolerance
//         if (abs(closestToZero) <= tolerance) {
//           break;
//         } else {
//           tolerance = tolerance * 1.2;
//         }
//       }

//       // Adjust the voltage based on the measurement
//       if (abs(avgValue) < tolerance * 10) {
//         if (avgValue > 0) {
//           currentVoltage += 1;
//         } else {
//           currentVoltage -= 1;
//         }
//       } else {
//         if (avgValue > 0) {
//           currentVoltage += 2;
//         } else {
//           currentVoltage -= 2;
//         }
//       }

//       // Ensure the voltage stays within the allowed range
//       if (currentVoltage < minVoltage || currentVoltage > maxVoltage) {
//         break;
//       }

//       previousAvgValue = avgValue;
//     }

//     bestVoltages[j] = bestVoltage; // Store the best voltage for the current preset voltage

//     Serial.print("Preset voltage: ");
//     Serial.println(presetVoltage);
//     Serial.print("Best calibration voltage: ");
//     Serial.println(bestVoltage);
//     Serial.print("Closest measurement to zero: ");
//     Serial.println(closestToZero);

//     // Add a delay to avoid flooding the serial monitor
//     delay(5000);
//   }

//   // Print all best voltages
//   Serial.println("All best calibration voltages:");
//   for (int i = 0; i < numPresetVoltages; i++) {
//     Serial.print("Preset voltage ");
//     Serial.print(presetVoltages[i]);
//     Serial.print(": ");
//     Serial.println(bestVoltages[i]);
//   }

//   // Add a delay to avoid flooding the serial monitor
//   delay(10000);
// }

// float sendVoltage(float voltage) {
//   // Implement the function to send the specific voltage to your device
//   // This will depend on your specific hardware setup
//   // Example: Using a DAC or PWM to set the voltage
// }

// float readAnalogValue() {
//   int sensorValue = analogRead(analogPin);
//   float voltage = sensorValue * (5.0 / 1023.0);
//   return voltage;
// }

// float averageAnalogValue(int numSamples) {
//   float total = 0.0;
//   for (int i = 0; i < numSamples; i++) {
//     total += readAnalogValue();
//     delay(10); // Small delay between samples
//   }
//   return total / numSamples;
// }



























// const float tolerance = 0.01; // minGap
// const float minVoltage = 0.0; // Lower Limit
// const float maxVoltage = 5.0; //  Higher Limit
// const int rangeSamples = 3; // Number of measurements to store around the sign change
// const int numPresetVoltages = chanLevelVoiceDataSize / 2;

// const float presetVoltages[numPresetVoltages] = {1.0, 2.0, 3.0, 4.0, 5.0}; // Array of preset voltages
// float bestVoltages[numPresetVoltages] = {0.5, 1.5}; // Array to store the best voltages, with first two values manually set

// float sendVoltage(float voltage);
// float readAnalogValue();
// float averageAnalogValue(int numSamples);

// void loop() {
//   for (int j = 2; j < numPresetVoltages; j++) { // Start from the 3rd preset voltage
//     float presetVoltage = presetVoltages[j];
//     sendVoltage(presetVoltage); // Send the preset voltage

//     float currentVoltage = (minVoltage + maxVoltage) / 2.0;
//     float bestVoltage = currentVoltage;
//     float closestToZero = 1023.0; // Initialize with a large value
//     float previousAvgValue = 0.0;

//     float lowerMeasurements[rangeSamples];
//     float higherMeasurements[rangeSamples];
//     float lowerVoltages[rangeSamples];
//     float higherVoltages[rangeSamples];

//     while (true) {
//       sendVoltage(currentVoltage);
//       float avgValue = averageAnalogValue(numSamples);
      
//       if (abs(avgValue) < abs(closestToZero)) {
//         closestToZero = avgValue;
//         bestVoltage = currentVoltage;
//       }

//       // Detect sign change
//       if ((previousAvgValue > 0 && avgValue < 0) || (previousAvgValue < 0 && avgValue > 0)) {
//         // Store measurements around the current voltage
//         for (int i = 0; i < rangeSamples; i++) {
//           float lowerVoltage = currentVoltage - (i + 1);
//           float higherVoltage = currentVoltage + (i + 1);

//           sendVoltage(lowerVoltage);
//           lowerMeasurements[i] = averageAnalogValue(numSamples);
//           lowerVoltages[i] = lowerVoltage;

//           sendVoltage(higherVoltage);
//           higherMeasurements[i] = averageAnalogValue(numSamples);
//           higherVoltages[i] = higherVoltage;
//         }

//         // Evaluate stored measurements including the current voltage
//         for (int i = 0; i < rangeSamples; i++) {
//           if (abs(lowerMeasurements[i]) < abs(closestToZero)) {
//             closestToZero = lowerMeasurements[i];
//             bestVoltage = lowerVoltages[i];
//           }
//           if (abs(higherMeasurements[i]) < abs(closestToZero)) {
//             closestToZero = higherMeasurements[i];
//             bestVoltage = higherVoltages[i];
//           }
//         }

//         // Check the current voltage again
//         if (abs(avgValue) < abs(closestToZero)) {
//           closestToZero = avgValue;
//           bestVoltage = currentVoltage;
//         }

//         // Break the loop if the closest value is within tolerance
//         if (abs(closestToZero) <= tolerance) {
//           break;
//         } else {
//           tolerance = tolerance * 1.2;
//         }
//       }

//       // Adjust the voltage based on the measurement
//       if (abs(avgValue) < tolerance * 10) {
//         if (avgValue > 0) {
//           currentVoltage += 1;
//         } else {
//           currentVoltage -= 1;
//         }
//       } else {
//         if (avgValue > 0) {
//           currentVoltage += 2;
//         } else {
//           currentVoltage -= 2;
//         }
//       }

//       // Ensure the voltage stays within the allowed range
//       if (currentVoltage < minVoltage || currentVoltage > maxVoltage) {
//         break;
//       }

//       previousAvgValue = avgValue;
//     }

//     bestVoltages[j] = bestVoltage; // Store the best voltage for the current preset voltage

//     Serial.print("Preset voltage: ");
//     Serial.println(presetVoltage);
//     Serial.print("Best calibration voltage: ");
//     Serial.println(bestVoltage);
//     Serial.print("Closest measurement to zero: ");
//     Serial.println(closestToZero);

//     // Add a delay to avoid flooding the serial monitor
//     delay(5000);
//   }

//   // Print all best voltages
//   Serial.println("All best calibration voltages:");
//   for (int i = 0; i < numPresetVoltages; i++) {
//     Serial.print("Preset voltage ");
//     Serial.print(presetVoltages[i]);
//     Serial.print(": ");
//     Serial.println(bestVoltages[i]);
//   }

//   // Add a delay to avoid flooding the serial monitor
//   delay(10000);
// }

// float sendVoltage(float voltage) {
//   // Implement the function to send the specific voltage to your device
//   // This will depend on your specific hardware setup
//   // Example: Using a DAC or PWM to set the voltage
// }

// float readAnalogValue() {
//   int sensorValue = analogRead(analogPin);
//   float voltage = sensorValue * (5.0 / 1023.0);
//   return voltage;
// }

// float averageAnalogValue(int numSamples) {
//   float total = 0.0;
//   for (int i = 0; i < numSamples; i++) {
//     total += readAnalogValue();
//     delay(10); // Small delay between samples
//   }
//   return total / numSamples;
// }
