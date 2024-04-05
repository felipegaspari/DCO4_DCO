void init_voices() {

  for (int i = 0; i < NUM_VOICES_TOTAL; i++) {
    VOICE_NOTES[i] = DCO_calibration_start_note;
  }

  setVoiceMode();
  voice_task();
}

void voice_task() {
  // unsigned long voice_task_1_time;
  // unsigned long voice_task_2_time;
  // unsigned long voice_task_3_time;
  // unsigned long voice_task_4_time;
  // unsigned long voice_task_total_time;
  // unsigned long voice_task_start_time = micros();

  // Serial.println("VOICE TASK 1");

  for (int i = 0; i < NUM_VOICES; i++) {
    if (note_on_flag[i] == 1) {
      note_on_flag_flag[i] = true;
      note_on_flag[i] = 0;
    }
  }

  last_midi_pitch_bend = midi_pitch_bend;
  LAST_DETUNE = DETUNE;
  for (int i = 0; i < NUM_VOICES; i++) {
    if (VOICE_NOTES[i] >= 0) {
      uint8_t note1 = VOICE_NOTES[i] - 36 + OSC1_interval;
      if (note1 > highestNote) {
        note1 -= ((uint8_t(note1 - highestNote) / 12) * 12);
      }
      uint8_t note2 = note1 - 24 + OSC2_interval;
      if (note2 > highestNote) {
        note2 -= ((uint8_t(note2 - highestNote) / 12) * 12);
      }

      if (OSC2_serial_detune == 127) {
        OSC2_detune = 1;
      } else {
        OSC2_detune = 1.00f + (0.0002f * ((int)127 - OSC2_serial_detune));
      }

      volatile register float freq;
      volatile register float freq2;

      // Serial.println("VOICE TASK 2");
      ////***********************    PORTAMENTO CODE   ****************************************/////

      if (portamento_time > 0 /*&& portamento_start != 0 && portamento_stop != 0*/) {
        portamentoTimer[i] = millis() - portamentoStartMillis[i];

        if (note_on_flag_flag[i]) {
          //Serial.println("NOTE ON");
          portamentoStartMillis[i] = millis();

          portamentoTimer[i] = 0;

          portamento_start[i] = portamento_cur_freq[i];
          portamento_start[i + NUM_VOICES_TOTAL] = portamento_cur_freq[i + NUM_VOICES_TOTAL];


          portamento_stop[i] = sNotePitches[note1];
          portamento_stop[i + NUM_VOICES_TOTAL] = sNotePitches[note2];

          freqPortaInterval[i] = (float)(portamento_stop[i] - (float)portamento_start[i]) / portamento_time;
          freqPortaInterval[i + NUM_VOICES_TOTAL] = (float)(portamento_stop[i + NUM_VOICES_TOTAL] - (float)portamento_start[i + NUM_VOICES_TOTAL]) / portamento_time;
        }

        if (/*portamento_cur_freq[i] == 0 ||*/ portamentoTimer[i] >= portamento_time) {

          portamento_cur_freq[i] = sNotePitches[note1];
          portamento_cur_freq[i + NUM_VOICES_TOTAL] = sNotePitches[note2];

        } else {
          portamento_cur_freq[i] = (float)portamento_start[i] + (float)(freqPortaInterval[i] * (float)portamentoTimer[i]);
          portamento_cur_freq[i + NUM_VOICES_TOTAL] = (float)portamento_start[i + NUM_VOICES_TOTAL] + (float)(freqPortaInterval[i + NUM_VOICES_TOTAL] * (float)portamentoTimer[i]);
        }

        freq = portamento_cur_freq[i];
        freq2 = portamento_cur_freq[i + NUM_VOICES_TOTAL];

      } else {
        freq = (float)sNotePitches[note1];
        portamento_cur_freq[i] = freq;
        portamento_start[i] = freq;
        portamento_stop[i] = freq;

        freq2 = (float)sNotePitches[note2];
        portamento_cur_freq[i + NUM_VOICES_TOTAL] = freq2;
        portamento_start[i + NUM_VOICES_TOTAL] = freq2;
        portamento_stop[i + NUM_VOICES_TOTAL] = freq2;
      }

      // Serial.println("VOICE TASK 3");

      //voice_task_1_time = micros() - voice_task_start_time;

      float ADSRModifier;
      float ADSRModifierOSC1;
      float ADSRModifierOSC2;

      if (ADSR1toDETUNE1 != 0) {
        ADSRModifier = (float)ADSR1Level[i] * ADSR1toDETUNE1_formula;
        switch (ADSR3ToOscSelect) {
          case 0:
            ADSRModifierOSC1 = ADSRModifier;
            ADSRModifierOSC2 = 0;
            break;
          case 1:
            ADSRModifierOSC1 = 0;
            ADSRModifierOSC2 = ADSRModifier;
            break;
          case 2:
            ADSRModifierOSC1 = ADSRModifier;
            ADSRModifierOSC2 = ADSRModifier;
            break;
        }
      } else {
        ADSRModifier = 0;
        ADSRModifierOSC1 = 0;
        ADSRModifierOSC2 = 0;
      }

      float unisonMODIFIER = 0;
      if (voiceMode == 2) {
        unisonMODIFIER = (0.0001f * unisonDetune);
        if ((i & 0x01) == 0) {
          unisonMODIFIER = 0 - (unisonMODIFIER * (i - 1));
        } else {
          unisonMODIFIER = 0 - (unisonMODIFIER * i);
        }
      }

      /*   Este bloque tarda 10 microsegundos */

      freq = (freq * (DETUNE_INTERNAL_FIFO_float + ADSRModifierOSC1 + unisonMODIFIER + 1)) - (freq * ((0x2000 - midi_pitch_bend) / 67000));

      freq2 = (freq2 * (DETUNE_INTERNAL_FIFO_float + ADSRModifierOSC2 + unisonMODIFIER + 1) * OSC2_detune) - (freq2 * ((0x2000 - midi_pitch_bend) / 67000));

      /* FIN */

      //voice_task_2_time = micros() - voice_task_start_time;

      // serial_send_freq(freq);

      // // pwm_set_chan_level(RANGE_PWM_SLICES[i], pwm_gpio_to_channel(RANGE_PINS[i]), dato_serial);

      // chanLevel = dato_serial;
      // uint16_t chanLevel = get_chan_level(freq);
      // uint16_t chanLevel2 = get_chan_level(freq2);
      //int chanLevel = get_chan_level_lookup(freq);
      //int chanLevel2 = get_chan_level_lookup(freq2);
      //  float chanLevel = dato_serial;
      //  float chanLevel2 = dato_serial;
      // pio_clkdiv_restart_sm_mask(pio1, (1<<0)+(1<<1));
      uint8_t pioNumber = VOICE_TO_PIO[i * 2];
      PIO pioN = pio[VOICE_TO_PIO[i * 2]];
      uint8_t sm1N = VOICE_TO_SM[i * 2];
      uint8_t sm2N = sm1N + 1;

      //voice_task_3_time = micros() - voice_task_start_time;

      //  set_frequency(pio1, sm1N, freq);
      //  set_frequency(pio1, sm2N, freq2);
      volatile register uint32_t clk_div1 = (int)((float)(eightSysClock_Hz + pioPulseLengthTimesEight - eightPioPulseLength * freq) / freq);
      // clk_div1 = clk_div1 - pioPulseLength; //(clk_div1 * 0.0028);
      if (freq == 0)
        clk_div1 = 0;
      volatile register uint32_t clk_div2 = (int)((float)(eightSysClock_Hz + pioPulseLengthTimesEight - eightPioPulseLength * freq2) / freq2);
      // clk_div2 = clk_div2 - pioPulseLength; //(clk_div2 * 0.0028);

      //voice_task_4_time = micros() - voice_task_start_time;

      // if (freq2 == 0)   ??
      //   clk_div2 = 0;   ??

      // Serial.println("VOICE TASK 4");
      uint16_t chanLevel = get_chan_level_lookup((int_fast32_t)(freq * 100), (i * 2));
      uint16_t chanLevel2 = get_chan_level_lookup((int_fast32_t)(freq2 * 100), (i * 2) + 1);
      // Serial.println("VOICE TASK 5");
      // Serial.println("VOICE TASK a");
      // Serial.println("VOICE TASK a");
      // Serial.println("VOICE TASK a");

      //VCO LEVEL //uint16_t vcoLevel = get_vco_level(freq);
      if (oscSync == 0) {
        // Serial.println("VOICE TASK b");
        uint_fast32_t clockdivFraction = clk_div1 / 300;
        uint_fast32_t clockdiv2Fraction = clk_div2 / 300;
        pio_sm_put(pioN, sm1N, clk_div1 - clockdivFraction / 3 + random(0, clockdivFraction));
        pio_sm_put(pioN, sm2N, clk_div2 - clockdiv2Fraction / 2 + random(0, clockdiv2Fraction));
        pio_sm_exec(pioN, sm1N, pio_encode_pull(false, false));
        pio_sm_exec(pioN, sm2N, pio_encode_pull(false, false));
      } else {
        // Serial.println("VOICE TASK 5a");
        pio_sm_put(pioN, sm1N, clk_div1);
        pio_sm_put(pioN, sm2N, clk_div2);
        pio_sm_exec(pioN, sm1N, pio_encode_pull(false, false));
        pio_sm_exec(pioN, sm2N, pio_encode_pull(false, false));
        if (note_on_flag_flag[i]) {
          // pio_sm_exec(pioN, sm1N, pio_encode_out(pio_osr, 31));
          // pio_sm_exec(pioN, sm2N, pio_encode_out(pio_osr, 31));
          unsigned long periodA = (float)1000000 / freq / 64;
          // Serial.println("VOICE TASK 5b");
          switch (oscSync) {

            case 1:
              // pwm_set_chan_level(RANGE_PWM_SLICES[i * 2], pwm_gpio_to_channel(RANGE_PINS[i * 2]), 1);
              // pwm_set_chan_level(RANGE_PWM_SLICES[(i * 2) + 1], pwm_gpio_to_channel(RANGE_PINS[(i * 2) + 1]), 1);
              // delayMicroseconds(6);
              // Serial.println("VOICE TASK 5c");
              pio_sm_exec(pioN, sm1N, pio_encode_jmp(10 + offset[pioNumber]));  // OSC Sync MODE
              pio_sm_exec(pioN, sm2N, pio_encode_jmp(10 + offset[pioNumber]));
              // Serial.println("VOICE TASK 5d");
              //delayMicroseconds(periodA);
              break;
            case 2:
              // Serial.println("VOICE TASK 5e");
              pio_sm_exec(pioN, sm1N, pio_encode_jmp(4 + offset[pioNumber]));  // OSC Half Sync MODE
              pio_sm_exec(pioN, sm2N, pio_encode_jmp(12 + offset[pioNumber]));
              break;
            case 3:
              // Serial.println("VOICE TASK 5f");
              pio_sm_exec(pioN, sm1N, pio_encode_jmp(4 + offset[pioNumber]));  // OSC 3rd-quarter Sync MODE
              pio_sm_exec(pioN, sm2N, pio_encode_jmp(10 + offset[pioNumber]));
              break;
            default:
              break;
          }
          // Serial.println("VOICE TASK 6");
          pwm_set_chan_level(RANGE_PWM_SLICES[i * 2], pwm_gpio_to_channel(RANGE_PINS[i * 2]), chanLevel);
          pwm_set_chan_level(RANGE_PWM_SLICES[(i * 2) + 1], pwm_gpio_to_channel(RANGE_PINS[(i * 2) + 1]), chanLevel2);
          // Serial.println("VOICE TASK 7");
        }
      }
      // Serial.println("VOICE TASK 8");
      if (timer99microsFlag) {
        // Serial.println("VOICE TASK 9");

        pwm_set_chan_level(RANGE_PWM_SLICES[i * 2], pwm_gpio_to_channel(RANGE_PINS[i * 2]), chanLevel);
        // Serial.println("VOICE TASK 12");
        //pwm_set_chan_level(RANGE_PWM_SLICES[i], pwm_gpio_to_channel(RANGE_PINS[i]), dato_serial);
        pwm_set_chan_level(RANGE_PWM_SLICES[(i * 2) + 1], pwm_gpio_to_channel(RANGE_PINS[(i * 2) + 1]), chanLevel2);

        // if (i == 0) {
        //   Serial.println((String) " chanlevel " + chanLevel + (String) " - chanlevel2 " + chanLevel2);
        // }
        // Serial.println("VOICE TASK 13");
        // pwm_set_chan_level(VCO_PWM_SLICES[0], pwm_gpio_to_channel(22), (uint16_t)(vcoLevel)); // VCO control
      }
      //pwm_set_chan_level(RANGE_PWM_SLICES[i], pwm_gpio_to_channel(RANGE_PINS[i]), dato_serial);
      // pwm_set_chan_level(RANGE_PWM_SLICES[i + 1], pwm_gpio_to_channel(RANGE_PINS[i + 1]), dato_serial);

      // voiceFreq[i] = freq;
    }
    note_on_flag_flag[i] = false;
  }

  // Serial.println("VOICE TASK 14");
  //   voice_task_total_time = micros() - voice_task_start_time;
  //   if (voice_task_total_time >= 20) {
  //     Serial.println((String) " - T1: " + voice_task_1_time + (String) " - T2: " + voice_task_2_time + (String) " - T3: " + voice_task_3_time + (String) " - T4: " + voice_task_4_time + (String) " - TT: " + voice_task_total_time);
  //   }
}

// PER OSCILLATOR AUTOTUNE FUNCTION
void voice_task_autotune() {

  float freq;

  uint8_t note1;  // = 57;


  if (VOICE_NOTES[0] >= 0) {
    note1 = VOICE_NOTES[0] - 12;
  }

  if (autotuneOnFlag == true) {
    freq = (float)sNotePitches[note1];
  } else {
    freq = PIDOutput;
  }

  int chanLevel = ampCompCalibrationVal;

  if (manualTuneOnFlag == true) {

    // ALL AT ONCE
    for (int i = 0; i < NUM_OSCILLATORS; i++) {
      uint8_t pioNumber = VOICE_TO_PIO[i];
      PIO pioN = pio[VOICE_TO_PIO[i]];
      uint8_t sm1N = VOICE_TO_SM[i];

     
      volatile register uint32_t clk_div1 = (int)((float)(eightSysClock_Hz + pioPulseLengthTimesEight - eightPioPulseLength * freq) / freq);

      if (freq == 0)
        clk_div1 = 0;

      pio_sm_put(pioN, sm1N, clk_div1);

      pio_sm_exec(pioN, sm1N, pio_encode_pull(false, false));

      pwm_set_chan_level(RANGE_PWM_SLICES[i], pwm_gpio_to_channel(RANGE_PINS[i]), chanLevel);
    }

    //SINGLE VOICE
    // currentDCO = 1;
    // uint8_t pioNumber = VOICE_TO_PIO[currentDCO];
    // PIO pioN = pio[VOICE_TO_PIO[currentDCO]];
    // uint8_t sm1N = VOICE_TO_SM[currentDCO];

    // volatile register uint32_t clk_div1 = (int)((float)(eightSysClock_Hz + pioPulseLengthTimesEight - eightPioPulseLength * freq) / freq);

    // if (freq == 0)
    //   clk_div1 = 0;

    // pio_sm_put(pioN, sm1N, clk_div1);

    // pio_sm_exec(pioN, sm1N, pio_encode_pull(false, false));

    // pwm_set_chan_level(RANGE_PWM_SLICES[currentDCO], pwm_gpio_to_channel(RANGE_PINS[currentDCO]), chanLevel);


  } else {

    uint8_t pioNumber = VOICE_TO_PIO[currentDCO];
    PIO pioN = pio[VOICE_TO_PIO[currentDCO]];
    uint8_t sm1N = VOICE_TO_SM[currentDCO];

    uint32_t clk_div1 = (int)((float)(eightSysClock_Hz - eightPioPulseLength - (freq * 500)) / freq);

    if (freq == 0)
      clk_div1 = 0;

    pio_sm_put(pioN, sm1N, clk_div1);

    pio_sm_exec(pioN, sm1N, pio_encode_pull(false, false));

    pwm_set_chan_level(RANGE_PWM_SLICES[currentDCO], pwm_gpio_to_channel(RANGE_PINS[currentDCO]), chanLevel);

    voiceFreq[currentDCO] = freq;
  }
  //Serial.println((String) "| currentDCO: " + currentDCO + (String) " | freq: " + freq + (String) " | clk_div1: " + clk_div1 + (String) " | ampCompCalibrationVal: " + ampCompCalibrationVal);
}

// uint16_t get_vco_level(float freq) {

//   float vcoLevel;
//   if (freq > 8500.00) {
//     vcoLevel = 9999;
//   } else {
//     // for (int i = 0; i < 75; i = i + 2)
//     for (int i = 0; i < sizeof(freq_to_vco_array); i++) {
//       if (freq == freq_to_vco_array[i]) {
//         vcoLevel = pwm_to_vco_euler_array[i];
//         return vcoLevel;
//         break;
//       } else if ((freq >= freq_to_vco_array[i]) && (freq <= freq_to_vco_array[i + 1])) {
//         vcoLevel = pwm_to_vco_euler_array[i] - ((pwm_to_vco_euler_array[i] - pwm_to_vco_euler_array[i + 1]) * (freq - freq_to_vco_array[i])) / (freq_to_vco_array[i + 1] - freq_to_vco_array[i]);
//         vcoLevel = pow(vcoLevel, 0.36788);
//         return (uint16_t)vcoLevel;
//         break;
//       }
//     }
//   }
//   return (uint16_t)vcoLevel;
// }

float get_chan_level(float freq_to_amp_comp) {

  ////73,41  130,80
  float chanLevelFloat;
  // if (freq_to_amp_comp <= 111)
  // {
  //     chanLevelFloat = 0.297 * freq_to_amp_comp - 10.163;
  // }

  if (freq_to_amp_comp <= 87) {
    chanLevelFloat = 0.297 * freq_to_amp_comp - 10.163;
  } else if (freq_to_amp_comp <= 132) {
    chanLevelFloat = 0.29 * freq_to_amp_comp - 9.7;
  } else if (freq_to_amp_comp <= 480) {
    chanLevelFloat = 0.31 * freq_to_amp_comp - 12.17;
  } else {
    chanLevelFloat = 0.32 * freq_to_amp_comp - 19.15;
  }
  return chanLevelFloat;
}

uint16_t get_chan_level_lookup(int_fast32_t intFreq, uint8_t voiceN) {

#ifndef ENABLE_FS_CALIBRATION
  voiceN = 0;
#endif

  uint16_t chanLevel;
  uint16_t startArrayPosition = chanLevelVoiceDataSize * voiceN;
  uint16_t endArrayPosition = startArrayPosition + chanLevelVoiceDataSize;

  if (intFreq > 520000) {
    chanLevel = DIV_COUNTER - 1;
    return chanLevel;
  } else {

    for (int i = startArrayPosition; i < endArrayPosition; i = i + 2) {
      if (intFreq == freq_to_amp_comp_array[i]) {
        chanLevel = freq_to_amp_comp_array[i + 1];
        return chanLevel;
        break;

      } else if ((intFreq > freq_to_amp_comp_array[i]) && (intFreq < freq_to_amp_comp_array[i + 2])) {
        //if (voiceN == 1) {
          //Serial.println((String) "INTFREQ: " + intFreq +(String) " - POS 0: " + freq_to_amp_comp_array[i] + (String)" - POS B: " + freq_to_amp_comp_array[i + 1] + (String)" - POS C: " + freq_to_amp_comp_array[i + 2] + (String)" - POS D: " + freq_to_amp_comp_array[i + 3]);
          //delay(1000);
        //}
        //int_fast32_t chanLevel32 = freq_to_amp_comp_array[i + 1];
        int_fast32_t chanLevel32_2 = ((freq_to_amp_comp_array[i + 1]) - (freq_to_amp_comp_array[i + 3])) * (intFreq - freq_to_amp_comp_array[i]);
        int_fast32_t chanLevel32_3 = freq_to_amp_comp_array[i + 2] - freq_to_amp_comp_array[i];
        // chanLevel32 = chanLevel32 - (int_fast32_t)((float)chanLevel32_2 / chanLevel32_3);
        // chanLevel = (uint16_t)chanLevel32;

        return (double)freq_to_amp_comp_array[i + 1] - (double)((double)chanLevel32_2 / (double)chanLevel32_3);
        break;
      }
    }
  }
  return chanLevel;
}

// uint16_t get_chan_level_lookup(float freq) {
//   uint16_t chanLevel;
//   if (freq > 3323.00) {
//     chanLevel = 2499;
//   } else {
//     // for (int i = 0; i < 75; i = i + 2)
//     for (int i = 0; i < 52; i = i + 2) {
//       if (freq == freq_to_amp_comp_array[i]) {
//         chanLevel = freq_to_amp_comp_array[i + 1];
//         return chanLevel;
//         break;
//       } else if ((freq >= freq_to_amp_comp_array[i]) && (freq <= freq_to_amp_comp_array[i + 2])) {
//         chanLevel = freq_to_amp_comp_array[i + 1] - ((freq_to_amp_comp_array[i + 1] - freq_to_amp_comp_array[i + 3]) * (freq - freq_to_amp_comp_array[i])) / (freq_to_amp_comp_array[i + 2] - freq_to_amp_comp_array[i]);
//                 Serial.print(" - L16 = ");
//         Serial.println(chanLevel);
//         return chanLevel;
//         break;
//       }
//     }
//   }
//   return chanLevel;
// }

uint8_t get_free_voice_sequential() {

  uint8_t nextVoice;
  uint8_t freeVoices;
  uint8_t freeVoiceNum;

  if (VOICES[VOICES_LAST_SEQUENCE[NUM_VOICES_TOTAL - 1]] == 1) {

    for (int i = NUM_VOICES_TOTAL - 2; i > 0; i--) {
      if (VOICES[VOICES_LAST_SEQUENCE[i]] == 0) {
        nextVoice = VOICES_LAST_SEQUENCE[i];
        freeVoices = 1;

        for (int j = i; j > 0; j--) {
          VOICES_LAST_SEQUENCE[j] = VOICES_LAST_SEQUENCE[j - 1];
        }

        VOICES_LAST_SEQUENCE[0] = nextVoice;
        return nextVoice;
      }
    }
  } else {
    if (VOICES[VOICES_LAST_SEQUENCE[NUM_VOICES_TOTAL - 1]] == 0) {
      nextVoice = VOICES_LAST_SEQUENCE[NUM_VOICES_TOTAL - 1];

      for (int i = NUM_VOICES_TOTAL - 1; i > 0; i--) {
        VOICES_LAST_SEQUENCE[i] = VOICES_LAST_SEQUENCE[i - 1];
      }

      VOICES_LAST_SEQUENCE[0] = nextVoice;

      return nextVoice;
    }
  }
  if (freeVoices == 0) {
    nextVoice = VOICES_LAST_SEQUENCE[NUM_VOICES_TOTAL - 1];

    for (int i = NUM_VOICES_TOTAL - 1; i > 0; i--) {
      VOICES_LAST_SEQUENCE[i] = VOICES_LAST_SEQUENCE[i - 1];
    }

    VOICES_LAST_SEQUENCE[0] = nextVoice;
  }
  return nextVoice;
}


uint8_t get_free_voice() {
  uint32_t oldest_time = millis();
  uint8_t oldest_voice = 0;

  for (int i = 0; i < NUM_VOICES_TOTAL; i++)  // REVISAR!!
  {
    uint8_t n = (NEXT_VOICE + i) % NUM_VOICES_TOTAL;

    if (VOICES[n] == 0) {
      NEXT_VOICE = (n + 1) % NUM_VOICES_TOTAL;
      return n;
    }

    if (VOICES[i] < oldest_time) {
      oldest_time = VOICES[i];
      oldest_voice = i;
    }
  }

  NEXT_VOICE = (oldest_voice + 1) % NUM_VOICES_TOTAL;
  return oldest_voice;
}

void setVoiceMode() {
  switch (voiceMode) {
    case 0:
      NUM_VOICES = 1;
      STACK_VOICES = 1;
      break;
    case 1:
      NUM_VOICES = NUM_VOICES_TOTAL;
      STACK_VOICES = 1;
      break;
    case 2:
      NUM_VOICES = NUM_VOICES_TOTAL;
      STACK_VOICES = NUM_VOICES_TOTAL;
      break;
  }
}

void voice_task_debug() {

  for (int i = 0; i < NUM_VOICES; i++) {
    if (note_on_flag[i] == 1) {
      note_on_flag_flag[i] = true;
      note_on_flag[i] = 0;
    }
  }

  last_midi_pitch_bend = midi_pitch_bend;
  LAST_DETUNE = DETUNE;
  for (int i = 0; i < NUM_VOICES; i++) {
    if (VOICE_NOTES[i] >= 0) {
      uint8_t note1 = VOICE_NOTES[i] - 36 + OSC1_interval;
      if (note1 > highestNote) {
        note1 -= ((uint8_t(note1 - highestNote) / 12) * 12);
      }
      uint8_t note2 = note1 - 24 + OSC2_interval;
      if (note2 > highestNote) {
        note2 -= ((uint8_t(note2 - highestNote) / 12) * 12);
      }

      volatile register float freq;
      volatile register float freq2;

      freq = (float)sNotePitches[note1];
      freq2 = (float)sNotePitches[note2];

      uint8_t pioNumber = VOICE_TO_PIO[i * 2];
      PIO pioN = pio[VOICE_TO_PIO[i * 2]];
      uint8_t sm1N = VOICE_TO_SM[i * 2];
      uint8_t sm2N = sm1N + 1;

      volatile register uint32_t clk_div1 = (int)((float)(eightSysClock_Hz + pioPulseLengthTimesEight - eightPioPulseLength * freq) / freq);
      // clk_div1 = clk_div1 - pioPulseLength; //(clk_div1 * 0.0028);
      if (freq == 0)
        clk_div1 = 0;
      volatile register uint32_t clk_div2 = (int)((float)(eightSysClock_Hz + pioPulseLengthTimesEight - eightPioPulseLength * freq2) / freq2);
      // clk_div2 = clk_div2 - pioPulseLength; //(clk_div2 * 0.0028);

      //voice_task_4_time = micros() - voice_task_start_time;

      // if (freq2 == 0)   ??
      //   clk_div2 = 0;   ??
      // Serial.println("VOICE TASK 4");
      uint16_t chanLevel = get_chan_level_lookup((int_fast32_t)(freq * 100), (i * 2));
      uint16_t chanLevel2 = get_chan_level_lookup((int_fast32_t)(freq2 * 100), (i * 2) + 1);
      // Serial.println("VOICE TASK 5");
      // Serial.println("VOICE TASK a");
      // Serial.println("VOICE TASK a");
      // Serial.println("VOICE TASK a");

      //VCO LEVEL //uint16_t vcoLevel = get_vco_level(freq);
      if (oscSync == 0) {
        // Serial.println("VOICE TASK b");
        uint_fast32_t clockdivFraction = clk_div1 / 300;
        uint_fast32_t clockdiv2Fraction = clk_div2 / 300;
        pio_sm_put(pioN, sm1N, clk_div1 - clockdivFraction / 3 + random(0, clockdivFraction));
        pio_sm_put(pioN, sm2N, clk_div2 - clockdiv2Fraction / 2 + random(0, clockdiv2Fraction));
        pio_sm_exec(pioN, sm1N, pio_encode_pull(false, false));
        pio_sm_exec(pioN, sm2N, pio_encode_pull(false, false));
      } else {
        // Serial.println("VOICE TASK 5a");
        pio_sm_put(pioN, sm1N, clk_div1);
        pio_sm_put(pioN, sm2N, clk_div2);
        pio_sm_exec(pioN, sm1N, pio_encode_pull(false, false));
        pio_sm_exec(pioN, sm2N, pio_encode_pull(false, false));
        if (note_on_flag_flag[i]) {
          // pio_sm_exec(pioN, sm1N, pio_encode_out(pio_osr, 31));
          // pio_sm_exec(pioN, sm2N, pio_encode_out(pio_osr, 31));
          unsigned long periodA = (float)1000000 / freq / 64;
          // Serial.println("VOICE TASK 5b");
          switch (oscSync) {

            case 1:
              // pwm_set_chan_level(RANGE_PWM_SLICES[i * 2], pwm_gpio_to_channel(RANGE_PINS[i * 2]), 1);
              // pwm_set_chan_level(RANGE_PWM_SLICES[(i * 2) + 1], pwm_gpio_to_channel(RANGE_PINS[(i * 2) + 1]), 1);
              // delayMicroseconds(6);
              // Serial.println("VOICE TASK 5c");
              pio_sm_exec(pioN, sm1N, pio_encode_jmp(10 + offset[pioNumber]));  // OSC Sync MODE
              pio_sm_exec(pioN, sm2N, pio_encode_jmp(10 + offset[pioNumber]));
              // Serial.println("VOICE TASK 5d");
              //delayMicroseconds(periodA);
              break;
            case 2:
              // Serial.println("VOICE TASK 5e");
              pio_sm_exec(pioN, sm1N, pio_encode_jmp(4 + offset[pioNumber]));  // OSC Half Sync MODE
              pio_sm_exec(pioN, sm2N, pio_encode_jmp(12 + offset[pioNumber]));
              break;
            case 3:
              // Serial.println("VOICE TASK 5f");
              pio_sm_exec(pioN, sm1N, pio_encode_jmp(4 + offset[pioNumber]));  // OSC 3rd-quarter Sync MODE
              pio_sm_exec(pioN, sm2N, pio_encode_jmp(10 + offset[pioNumber]));
              break;
            default:
              break;
          }
          // Serial.println("VOICE TASK 6");
          uint16_t chanLevel = get_chan_level_lookup((int_fast32_t)(freq * 100), (i * 2));
          uint16_t chanLevel2 = get_chan_level_lookup((int_fast32_t)(freq2 * 100), (i * 2) + 1);
          pwm_set_chan_level(RANGE_PWM_SLICES[i * 2], pwm_gpio_to_channel(RANGE_PINS[i * 2]), chanLevel);
          pwm_set_chan_level(RANGE_PWM_SLICES[(i * 2) + 1], pwm_gpio_to_channel(RANGE_PINS[(i * 2) + 1]), chanLevel2);
          // Serial.println("VOICE TASK 7");
        }
      }
      // Serial.println("VOICE TASK 8");
      if (timer99microsFlag) {
        // Serial.println("VOICE TASK 9");

        uint16_t chanLevel = get_chan_level_lookup((int_fast32_t)(freq * 100), (i * 2));
        // Serial.println("VOICE TASK 10");
        uint16_t chanLevel2 = get_chan_level_lookup((int_fast32_t)(freq2 * 100), (i * 2) + 1);
        // Serial.println("VOICE TASK 11");
        pwm_set_chan_level(RANGE_PWM_SLICES[i * 2], pwm_gpio_to_channel(RANGE_PINS[i * 2]), chanLevel);
        // Serial.println("VOICE TASK 12");
        pwm_set_chan_level(RANGE_PWM_SLICES[(i * 2) + 1], pwm_gpio_to_channel(RANGE_PINS[(i * 2) + 1]), chanLevel2);

        // if (i == 0) {
        //   Serial.println((String) " chanlevel " + chanLevel + (String) " - chanlevel2 " + chanLevel2);
        // }
      }
    }
    note_on_flag_flag[i] = false;
  }
}