#include "include_all.h"

#ifdef RUNNING_AVERAGE
// RunningAverage object definitions for timing measurements
RunningAverage ra_pitchbend(2000);
RunningAverage ra_osc2_detune(2000);
RunningAverage ra_portamento(2000);
RunningAverage ra_adsr_modifier(2000);
RunningAverage ra_unison_modifier(2000);
RunningAverage ra_drift_modifier(2000);
RunningAverage ra_modifiers_combination(2000);
RunningAverage ra_freq_scaling(2000);
RunningAverage ra_interpolate_pitch(2000);
RunningAverage ra_get_chan_level(2000);
RunningAverage ra_pwm_calculations(2000);
RunningAverage ra_voice_task_total(2000);
RunningAverage ra_clk_div_calc(2000);

unsigned long last_timing_print = 0;
unsigned long voice_task_max_time = 0;
const unsigned long TIMING_PRINT_INTERVAL = 1000; // Print every 5 seconds
#endif

void init_voices() {

  for (int i = 0; i < NUM_VOICES_TOTAL; i++) {
    VOICE_NOTES[i] = DCO_calibration_start_note;
  }

  initMultiplierTables();
  setVoiceMode();
  voice_task();

#ifdef RUNNING_AVERAGE
  // Clear all running averages
  ra_pitchbend.clear();
  ra_osc2_detune.clear();
  ra_portamento.clear();
  ra_adsr_modifier.clear();
  ra_unison_modifier.clear();
  ra_drift_modifier.clear();
  ra_modifiers_combination.clear();
  ra_freq_scaling.clear();
  ra_interpolate_pitch.clear();
  ra_get_chan_level.clear();
  ra_pwm_calculations.clear();
  ra_voice_task_total.clear();
  ra_clk_div_calc.clear();
#endif
}

inline void voice_task() {
#ifdef RUNNING_AVERAGE
  unsigned long voice_task_start_time = micros();
#endif

  float calcPitchbend;

#ifdef RUNNING_AVERAGE
  unsigned long t_start = micros();
#endif
  if (midi_pitch_bend == 8192) {
    calcPitchbend = 0.0f;
  } else {
    if (midi_pitch_bend < 8192) {
      calcPitchbend = (((float)midi_pitch_bend / 8190.99f) - 1.0f) * pitchBendMultiplier;
    } else {
      calcPitchbend = (((float)midi_pitch_bend / 8192.99f) - 1.0f) * pitchBendMultiplier;
    }
  }
#ifdef RUNNING_AVERAGE
  ra_pitchbend.addValue((float)(micros() - t_start));
#endif

  last_midi_pitch_bend = midi_pitch_bend;
  LAST_DETUNE = DETUNE;

  for (int i = 0; i < NUM_VOICES_TOTAL; i++) {

    if (note_on_flag[i] == 1) {
      note_on_flag_flag[i] = true;
      note_on_flag[i] = 0;
    }

    if (VOICE_NOTES[i] >= 0) {
      uint8_t note1 = VOICE_NOTES[i] - 36 + OSC1_interval;
      if (note1 > highestNote) {
        note1 -= ((uint8_t(note1 - highestNote) / 12) * 12);
      }
      uint8_t note2 = note1 - 36 + OSC2_interval;
      if (note2 > highestNote) {
        note2 -= ((uint8_t(note2 - highestNote) / 12) * 12);
      }

#ifdef RUNNING_AVERAGE
      unsigned long t_osc2 = micros();
#endif
      if (OSC2DetuneVal == 256) {
        OSC2_detune = 1;
      } else {
        OSC2_detune = 1.00f + (0.0002f * ((int)256 - OSC2DetuneVal));
      }
#ifdef RUNNING_AVERAGE
      ra_osc2_detune.addValue((float)(micros() - t_osc2));
#endif

      register float freq;
      register float freq2;

      uint8_t DCO_A = i * 2;
      uint8_t DCO_B = (i * 2) + 1;

      // Serial.println("VOICE TASK 2");
      ////***********************    PORTAMENTO CODE   ****************************************/////
#ifdef RUNNING_AVERAGE
      unsigned long t_portamento = micros();
#endif
      if (portamento_time > 0 /*&& portamento_start != 0 && portamento_stop != 0*/) {
        portamentoTimer[i] = micros() - portamentoStartMicros[i];

        if (note_on_flag_flag[i]) {
          // Serial.println("NOTE ON");
          portamentoStartMicros[i] = micros();

          portamentoTimer[i] = 0;

          portamento_start[DCO_A] = portamento_cur_freq[DCO_A];
          portamento_start[DCO_B] = portamento_cur_freq[DCO_B];

          portamento_stop[DCO_A] = sNotePitches[note1];
          portamento_stop[DCO_B] = sNotePitches[note2];

          freqPortaInterval[DCO_A] = (float)(portamento_stop[DCO_A] - (float)portamento_start[DCO_A]) / portamento_time;
          freqPortaInterval[DCO_B] = (float)(portamento_stop[DCO_B] - (float)portamento_start[DCO_B]) / portamento_time;
        }

        if (portamentoTimer[i] > portamento_time) {
          portamento_cur_freq[DCO_A] = sNotePitches[note1];
          portamento_start[DCO_A] = portamento_cur_freq[DCO_A];
          portamento_stop[DCO_A] = portamento_cur_freq[DCO_A];
          portamento_cur_freq[DCO_B] = sNotePitches[note2];
          portamento_start[DCO_B] = portamento_cur_freq[DCO_B];
          portamento_stop[DCO_B] = portamento_cur_freq[DCO_B];
        } else {
          portamento_cur_freq[DCO_A] = (float)portamento_start[DCO_A] + (float)(freqPortaInterval[DCO_A] * (float)portamentoTimer[i]);
          portamento_cur_freq[DCO_B] = (float)portamento_start[DCO_B] + (float)(freqPortaInterval[DCO_B] * (float)portamentoTimer[i]);
        }

        freq = portamento_cur_freq[DCO_A];
        freq2 = portamento_cur_freq[DCO_B];
      } else {
        freq = (float)sNotePitches[note1];
        portamento_cur_freq[DCO_A] = freq;
        portamento_start[DCO_A] = freq;
        portamento_stop[DCO_A] = freq;

        freq2 = (float)sNotePitches[note2];
        portamento_cur_freq[DCO_B] = freq2;
        portamento_start[DCO_B] = freq2;
        portamento_stop[DCO_B] = freq2;
      }
#ifdef RUNNING_AVERAGE
      ra_portamento.addValue((float)(micros() - t_portamento));
#endif
      ////***********************    PORTAMENTO CODE  END    ****************************************/////

      // Serial.println("VOICE TASK 3");
      // voice_task_1_time = micros() - voice_task_start_time;

      /* OLD CODE
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
      if (unisonDetune != 0) {
        unisonMODIFIER = (0.00006f * unisonDetune);
        if ((i & 0x01) == 0) {
          unisonMODIFIER = 0 - (unisonMODIFIER * (i - 1));
        } else {
          unisonMODIFIER = 0 - (unisonMODIFIER * i);
        }
      }

      float DETUNE_DRIFT_OSC1 = 0;
      float DETUNE_DRIFT_OSC2 = 0;

      if (analogDrift != 0) {
        DETUNE_DRIFT_OSC1 = (float)((float)LFO_DRIFT_LEVEL[DCO_A] * 0.0000005f * analogDrift);
        DETUNE_DRIFT_OSC2 = (float)((float)LFO_DRIFT_LEVEL[DCO_B] * 0.0000005f * analogDrift);
      }
*/
#ifdef RUNNING_AVERAGE
      unsigned long t_adsr = micros();
#endif
      float ADSRModifier = (ADSR1toDETUNE1 != 0) ? ((float)linToLogLookup[ADSR1Level[i]] * ADSR1toDETUNE1_formula) : 0;
      float ADSRModifierOSC1 = (ADSR3ToOscSelect == 0 || ADSR3ToOscSelect == 2) ? ADSRModifier : 0;
      float ADSRModifierOSC2 = (ADSR3ToOscSelect == 1 || ADSR3ToOscSelect == 2) ? ADSRModifier : 0;
#ifdef RUNNING_AVERAGE
      ra_adsr_modifier.addValue((float)(micros() - t_adsr));
      unsigned long t_unison = micros();
#endif

      float unisonMODIFIER = (unisonDetune != 0) ? (0.00006f * unisonDetune * ((i & 0x01) == 0 ? -(i - 1) : -i)) : 0;
#ifdef RUNNING_AVERAGE
      ra_unison_modifier.addValue((float)(micros() - t_unison));
      unsigned long t_drift = micros();
#endif

      float DETUNE_DRIFT_OSC1 = (analogDrift != 0) ? (LFO_DRIFT_LEVEL[DCO_A] * 0.0000005f * analogDrift) : 0;
      float DETUNE_DRIFT_OSC2 = (analogDrift != 0) ? (LFO_DRIFT_LEVEL[DCO_B] * 0.0000005f * analogDrift) : 0;
#ifdef RUNNING_AVERAGE
      ra_drift_modifier.addValue((float)(micros() - t_drift));
#endif

      /*   Este bloque tardaba 10 microsegundos */

#ifdef RUNNING_AVERAGE
      unsigned long t_modifiers = micros();
#endif
      float modifiersAll = DETUNE_INTERNAL_FIFO_float + unisonMODIFIER + calcPitchbend + 1.00001f;
      float freqModifiers = ADSRModifierOSC1 + DETUNE_DRIFT_OSC1 + modifiersAll;
      float freq2Modifiers = (ADSRModifierOSC2 + DETUNE_DRIFT_OSC2 + modifiersAll);
#ifdef RUNNING_AVERAGE
      ra_modifiers_combination.addValue((float)(micros() - t_modifiers));
      unsigned long t_freq_scaling = micros();
#endif

      freq = freq * (float)((float)interpolatePitchMultiplier(freqModifiers) / (float)multiplierTableScale);
      freq2 = freq2 * OSC2_detune * (float)((float)interpolatePitchMultiplier(freq2Modifiers) / (float)multiplierTableScale);
#ifdef RUNNING_AVERAGE
      ra_freq_scaling.addValue((float)(micros() - t_freq_scaling));
#endif

      if ((uint16_t)freq > maxFrequency) {
        freq = maxFrequency;
      } else if ((uint16_t)freq < 6) {
        freq = 6;
      }
      if ((uint16_t)freq2 >= maxFrequency) {
        freq2 = maxFrequency;
      } else if ((uint16_t)freq2 < 6) {
        freq2 = 6;
      }

      /* FIN */

      // voice_task_2_time = micros() - voice_task_start_time;

      uint8_t pioNumberA = VOICE_TO_PIO[DCO_A];
      uint8_t pioNumberB = VOICE_TO_PIO[DCO_B];
      PIO pioN_A = pio[VOICE_TO_PIO[DCO_A]];
      PIO pioN_B = pio[VOICE_TO_PIO[DCO_B]];
      uint8_t sm1N = VOICE_TO_SM[DCO_A];
      uint8_t sm2N = VOICE_TO_SM[DCO_B];

      // voice_task_3_time = micros() - voice_task_start_time;

#ifdef RUNNING_AVERAGE
      unsigned long t_clk_div = micros();
#endif
      register uint32_t clk_div1 = (uint32_t)((eightSysClock_Hz / freq) - eightPioPulseLength);
      if (freq == 0)
        clk_div1 = 0;

      register uint32_t clk_div2;
      uint32_t phaseDelay;

      if (oscSync > 1) {

        clk_div2 = (uint32_t)(sysClock_Hz / freq2);
        phaseDelay = (clk_div2 - pioPulseLength) / 180 * phaseAlignOSC2;
        clk_div2 = (uint32_t)((clk_div2 - pioPulseLength - phaseDelay) / 8);
      } else {
        clk_div2 = (uint32_t)((eightSysClock_Hz / freq2) - eightPioPulseLength);
      }
      if (freq2 == 0)
        clk_div2 = 0;
#ifdef RUNNING_AVERAGE
      ra_clk_div_calc.addValue((float)(micros() - t_clk_div));
#endif

      // voice_task_4_time = micros() - voice_task_start_time;

      uint16_t chanLevel, chanLevel2;

#ifdef RUNNING_AVERAGE
      unsigned long t_chan_level = micros();
#endif
      switch (syncMode) {
        case 0:
          chanLevel = get_chan_level_lookup((int32_t)(freq * 100), DCO_A);
          chanLevel2 = get_chan_level_lookup((int32_t)(freq2 * 100), DCO_B);
          break;
        case 1:
          chanLevel = get_chan_level_lookup((int32_t)(max(freq, freq2) * 100), DCO_A);
          chanLevel2 = get_chan_level_lookup((int32_t)(freq2 * 100), DCO_B);
          break;
        case 2:
          chanLevel = get_chan_level_lookup((int32_t)(freq * 100), DCO_A);
          chanLevel2 = get_chan_level_lookup((int32_t)(max(freq, freq2) * 100), DCO_B);
          break;
      }
#ifdef RUNNING_AVERAGE
      ra_get_chan_level.addValue((float)(micros() - t_chan_level));
#endif

      // VCO LEVEL //uint16_t vcoLevel = get_vco_level(freq);

      pio_sm_put(pioN_A, sm1N, clk_div1);
      pio_sm_put(pioN_B, sm2N, clk_div2);
      pio_sm_exec(pioN_A, sm1N, pio_encode_pull(false, false));
      pio_sm_exec(pioN_B, sm2N, pio_encode_pull(false, false));

      // Serial.println("VOICE TASK 5a");

      if (note_on_flag_flag[i]) {
        if (oscSync > 0) {
          pio_sm_exec(pioN_A, sm1N, pio_encode_jmp(10 + offset[pioNumberA]));  // OSC Sync MODE
          pio_sm_exec(pioN_B, sm2N, pio_encode_jmp(10 + offset[pioNumberB]));

          if (oscSync > 1) {
            pio_sm_put(pioN_B, sm2N, pioPulseLength + phaseDelay - correctionPioPulseLength);
            pio_sm_exec(pioN_B, sm2N, pio_encode_pull(false, false));
            pio_sm_exec(pioN_B, sm2N, pio_encode_out(pio_y, 31));
          }
        }
        pwm_set_chan_level(RANGE_PWM_SLICES[DCO_A], pwm_gpio_to_channel(RANGE_PINS[DCO_A]), chanLevel);
        pwm_set_chan_level(RANGE_PWM_SLICES[DCO_B], pwm_gpio_to_channel(RANGE_PINS[DCO_B]), chanLevel2);
      }

      if (timer99microsFlag) {
        pwm_set_chan_level(RANGE_PWM_SLICES[DCO_A], pwm_gpio_to_channel(RANGE_PINS[DCO_A]), chanLevel);
        pwm_set_chan_level(RANGE_PWM_SLICES[DCO_B], pwm_gpio_to_channel(RANGE_PINS[DCO_B]), chanLevel2);

        if (sqr1Status) {
#ifdef RUNNING_AVERAGE
          unsigned long t_pwm = micros();
#endif
          float ADSR1toPW_calculated = (ADSR1toPWM != 0) ? ((float)ADSR1Level[i] / 4.00f * ADSR1toPWM_formula) : 0;
          float LFO2toPW_calculated = (LFO2toPW != 0) ? ((float)LFO2Level * LFO2toPWM_formula) : 0;
          PW_PWM[i] = (uint16_t)constrain((DIV_COUNTER_PW - 1 - LFO2toPW_calculated - PW[0] + ADSR1toPW_calculated), 0, DIV_COUNTER_PW - 1);
          // PW_PWM[i] = (uint16_t)constrain(DIV_COUNTER_PW - 1 - /*((float)ADSR3Level[i] * ADSR3toPWM_formula)*/ - ((float)LFO2Level * LFO2toPWM_formula) - PW /*+ RANDOMNESS1 + RANDOMNESS2*/, 0, DIV_COUNTER_PW-1);
          pwm_set_chan_level(PW_PWM_SLICES[i], pwm_gpio_to_channel(PW_PINS[i]), get_PW_level_interpolated(PW_PWM[i], i));
#ifdef RUNNING_AVERAGE
          ra_pwm_calculations.addValue((float)(micros() - t_pwm));
#endif
        } else {
          pwm_set_chan_level(PW_PWM_SLICES[i], pwm_gpio_to_channel(PW_PINS[i]), 0);
        }
        // Serial.println("VOICE TASK 13");
        // pwm_set_chan_level(VCO_PWM_SLICES[0], pwm_gpio_to_channel(22), (uint16_t)(vcoLevel)); // VCO control
      }
    }
    note_on_flag_flag[i] = false;
  }

#ifdef RUNNING_AVERAGE
  unsigned long voice_task_duration = micros() - voice_task_start_time;
  ra_voice_task_total.addValue((float)voice_task_duration);
  if (voice_task_duration > voice_task_max_time) {
    voice_task_max_time = voice_task_duration;
  }

  // Print timing statistics periodically
  unsigned long current_time = millis();
  if (current_time - last_timing_print >= TIMING_PRINT_INTERVAL) {
    print_voice_task_timings();
    last_timing_print = current_time;
  }
#endif

  // Serial.println("VOICE TASK 14");
  //   voice_task_total_time = micros() - voice_task_start_time;
  //   if (voice_task_total_time >= 20) {
  //     Serial.println((String) " - T1: " + voice_task_1_time + (String) " - T2: " + voice_task_2_time + (String) " - T3: " + voice_task_3_time + (String) " - T4: " + voice_task_4_time + (String) " - TT: " + voice_task_total_time);
  //   }
}

inline void voice_task_simple() {
  for (int i = 0; i < NUM_VOICES; i++) {

    if (note_on_flag[i] == 1) {
      note_on_flag_flag[i] = true;
      note_on_flag[i] = 0;
    }

    if (VOICE_NOTES[i] >= 0) {
      uint8_t note1 = VOICE_NOTES[i] - 36 + OSC1_interval;
      if (note1 > highestNote) {
        note1 -= ((uint8_t(note1 - highestNote) / 12) * 12);
      }
      uint8_t note2 = note1 - 36 + OSC2_interval;
      if (note2 > highestNote) {
        note2 -= ((uint8_t(note2 - highestNote) / 12) * 12);
      }

      if (OSC2DetuneVal == 256) {
        OSC2_detune = 1;
      } else {
        OSC2_detune = 1.00f + (0.0002f * ((int)256 - OSC2DetuneVal));
      }

      float freq;
      float freq2;

      uint8_t DCO_A = i * 2;
      uint8_t DCO_B = (i * 2) + 1;

      freq = sNotePitches[note1];
      freq2 = sNotePitches[note2];

      // Serial.println("VOICE TASK 2");

      if ((uint16_t)freq > maxFrequency) {
        freq = maxFrequency;
      } else if ((uint16_t)freq < 6) {
        freq = 6;
      }
      if ((uint16_t)freq2 >= maxFrequency) {
        freq2 = maxFrequency;
      } else if ((uint16_t)freq2 < 6) {
        freq2 = 6;
      }

      // voice_task_2_time = micros() - voice_task_start_time;

      uint8_t pioNumberA = VOICE_TO_PIO[DCO_A];
      uint8_t pioNumberB = VOICE_TO_PIO[DCO_B];
      PIO pioN_A = pio[VOICE_TO_PIO[DCO_A]];
      PIO pioN_B = pio[VOICE_TO_PIO[DCO_B]];
      uint8_t sm1N = VOICE_TO_SM[DCO_A];
      uint8_t sm2N = VOICE_TO_SM[DCO_B];

      // voice_task_3_time = micros() - voice_task_start_time;

      uint32_t clk_div1 = (uint32_t)((eightSysClock_Hz / freq) - eightPioPulseLength);
      if (freq == 0)
        clk_div1 = 0;

      uint32_t clk_div2;
      uint32_t phaseDelay;

      if (oscSync > 1) {

        clk_div2 = (uint32_t)(sysClock_Hz / freq2);
        phaseDelay = (clk_div2 - pioPulseLength) / 180 * phaseAlignOSC2;
        clk_div2 = (uint32_t)((clk_div2 - pioPulseLength - phaseDelay) / 8);
      } else {
        clk_div2 = (uint32_t)((eightSysClock_Hz / freq2) - eightPioPulseLength);
      }
      if (freq2 == 0)
        clk_div2 = 0;

      // voice_task_4_time = micros() - voice_task_start_time;

      uint16_t chanLevel, chanLevel2;

      switch (syncMode) {
        case 0:
          chanLevel = get_chan_level_lookup((int32_t)(freq * 100), DCO_A);
          chanLevel2 = get_chan_level_lookup((int32_t)(freq2 * 100), DCO_B);
          break;
        case 1:
          chanLevel = get_chan_level_lookup((int32_t)(max(freq, freq2) * 100), DCO_A);
          chanLevel2 = get_chan_level_lookup((int32_t)(freq2 * 100), DCO_B);
          break;
        case 2:
          chanLevel = get_chan_level_lookup((int32_t)(freq * 100), DCO_A);
          chanLevel2 = get_chan_level_lookup((int32_t)(max(freq, freq2) * 100), DCO_B);
          break;
      }

      // VCO LEVEL //uint16_t vcoLevel = get_vco_level(freq);

      pio_sm_put(pioN_A, sm1N, clk_div1);
      pio_sm_put(pioN_B, sm2N, clk_div2);
      pio_sm_exec(pioN_A, sm1N, pio_encode_pull(false, false));
      pio_sm_exec(pioN_B, sm2N, pio_encode_pull(false, false));

      // Serial.println("VOICE TASK 5a");

      if (note_on_flag_flag[i]) {
        if (oscSync > 0) {
          pio_sm_exec(pioN_A, sm1N, pio_encode_jmp(10 + offset[pioNumberA]));  // OSC Sync MODE
          pio_sm_exec(pioN_B, sm2N, pio_encode_jmp(10 + offset[pioNumberB]));

          if (oscSync > 1) {
            pio_sm_put(pioN_B, sm2N, pioPulseLength + phaseDelay - correctionPioPulseLength);
            pio_sm_exec(pioN_B, sm2N, pio_encode_pull(false, false));
            pio_sm_exec(pioN_B, sm2N, pio_encode_out(pio_y, 31));
            pio_sm_exec(pioN_B, sm2N, pio_encode_out(pio_x, 31));
          }
        }

        pwm_set_chan_level(RANGE_PWM_SLICES[DCO_A], pwm_gpio_to_channel(RANGE_PINS[DCO_A]), chanLevel);
        pwm_set_chan_level(RANGE_PWM_SLICES[DCO_B], pwm_gpio_to_channel(RANGE_PINS[DCO_B]), chanLevel2);
      }

      if (timer99microsFlag) {
        pwm_set_chan_level(RANGE_PWM_SLICES[DCO_A], pwm_gpio_to_channel(RANGE_PINS[DCO_A]), chanLevel);
        pwm_set_chan_level(RANGE_PWM_SLICES[DCO_B], pwm_gpio_to_channel(RANGE_PINS[DCO_B]), chanLevel2);
      }

      if (sqr1Status) {
        pwm_set_chan_level(PW_PWM_SLICES[i], pwm_gpio_to_channel(PW_PINS[i]), PW_CENTER[i]);
      } else {
        pwm_set_chan_level(PW_PWM_SLICES[i], pwm_gpio_to_channel(PW_PINS[i]), 0);
      }
    }
    note_on_flag_flag[i] = false;
  }
}

inline uint8_t get_free_voice_sequential() {
  uint8_t nextVoice;
  uint8_t freeVoices = 0;

  if (VOICES[VOICES_LAST_SEQUENCE[NUM_VOICES_TOTAL - 1]] == 1 || VOICES[VOICES_LAST_SEQUENCE[NUM_VOICES_TOTAL - 1]] == 0) {
    for (int voiceIndex = NUM_VOICES_TOTAL - 1; voiceIndex > 0; voiceIndex--) {
      if (VOICES[VOICES_LAST_SEQUENCE[voiceIndex]] == 0) {
        nextVoice = VOICES_LAST_SEQUENCE[voiceIndex];
        freeVoices = 1;
        for (int freeIndex = voiceIndex; freeIndex > 0; freeIndex--) {
          VOICES_LAST_SEQUENCE[freeIndex] = VOICES_LAST_SEQUENCE[freeIndex - 1];
        }
        VOICES_LAST_SEQUENCE[0] = nextVoice;
        return nextVoice;
      }
    }
  } else {
    if (VOICES[VOICES_LAST_SEQUENCE[NUM_VOICES_TOTAL - 1]] == 0) {
      nextVoice = VOICES_LAST_SEQUENCE[NUM_VOICES_TOTAL - 1];

      for (int voiceIndex = NUM_VOICES_TOTAL - 1; voiceIndex > 0; voiceIndex--) {
        VOICES_LAST_SEQUENCE[voiceIndex] = VOICES_LAST_SEQUENCE[voiceIndex - 1];
      }

      VOICES_LAST_SEQUENCE[0] = nextVoice;

      return nextVoice;
    }
  }
  if (freeVoices == 0) {
    nextVoice = VOICES_LAST_SEQUENCE[NUM_VOICES_TOTAL - 1];

    for (int voiceIndex = NUM_VOICES_TOTAL - 1; voiceIndex > 0; voiceIndex--) {
      VOICES_LAST_SEQUENCE[voiceIndex] = VOICES_LAST_SEQUENCE[voiceIndex - 1];
    }

    VOICES_LAST_SEQUENCE[0] = nextVoice;
  }
  return nextVoice;
}

inline uint8_t get_free_voice() {
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

inline void setVoiceMode() {
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

// Uses non linear interpolation and coefficients
inline uint16_t get_chan_level_lookup(int32_t x, uint8_t voiceN) {
  // Note: Timing is measured at the call site in voice_task() for total lookup time
  // This includes both OSC1 and OSC2 lookups per voice iteration

  // Check if x is out of bounds
  if (x <= ampCompFrequencyArray[voiceN][0]) {
    return ampCompArray[voiceN][0];
  }
  if (x >= ampCompFrequencyArray[voiceN][ampCompTableSize - 1]) {
    return ampCompArray[voiceN][ampCompTableSize - 1];
  }

  // Find the interval x is in and use the precomputed coefficients
  for (int i = 0; i < ampCompTableSize - 2; i++) {
    if (ampCompFrequencyArray[voiceN][i] <= x && x < ampCompFrequencyArray[voiceN][i + 2]) {
      // Calculate the interpolated value using the precomputed coefficients
      float interpolatedValue = aCoeff[voiceN][i] * x * x + bCoeff[voiceN][i] * x + cCoeff[voiceN][i];

      // Round the result to the nearest integer
      return round(interpolatedValue);
    }
  }
  // If no interval is found (should not happen), return 0
  return 0;
}

inline uint16_t get_PW_level_interpolated(uint16_t PWval, uint8_t voiceN) {

  uint16_t chanLevel;

  if (PWval >= DIV_COUNTER) {
    chanLevel = PW_HIGH_LIMIT[voiceN];
    return chanLevel;
  } else if (PWval <= 0) {
    chanLevel = PW_LOW_LIMIT[voiceN];
    return chanLevel;
  } else {

    if (PWval >= PW_LOOKUP[1]) {
      chanLevel = map(PWval, PW_LOOKUP[1], PW_LOOKUP[2], PW_CENTER[voiceN], PW_HIGH_LIMIT[voiceN]);
      return chanLevel;
    } else {
      chanLevel = map(PWval, PW_LOOKUP[0], PW_LOOKUP[1], PW_LOW_LIMIT[voiceN], PW_CENTER[voiceN]);
      return chanLevel;   
    }

    return chanLevel;
  }
}

// PER OSCILLATOR AUTOTUNE FUNCTION
void voice_task_autotune(uint8_t taskAutotuneVoiceMode, uint16_t calibrationValue) {

  float freq;
  uint8_t note1;  // = 57;
  int chanLevel = ampCompCalibrationVal;

  if (VOICE_NOTES[0] > 0) {
    note1 = VOICE_NOTES[0] - 12;
  }

  if (taskAutotuneVoiceMode == 1 || taskAutotuneVoiceMode == 4) {
    freq = PIDOutput;
  } else {
    freq = (float)sNotePitches[note1];
  }

  // if (manualCalibrationFlag == true) {

  //   // ALL AT ONCE
  //   for (int i = 0; i < NUM_OSCILLATORS; i++) {
  //     uint8_t pioNumber = VOICE_TO_PIO[i];
  //     PIO pioN = pio[VOICE_TO_PIO[i]];
  //     uint8_t sm1N = VOICE_TO_SM[i];

  //     register uint32_t clk_div1 = (int)((float)(eightSysClock_Hz + pioPulseLengthTimesEight - eightPioPulseLength * freq) / freq);

  //     if (freq == 0)
  //       clk_div1 = 0;

  //     pio_sm_put(pioN, sm1N, clk_div1);

  //     pio_sm_exec(pioN, sm1N, pio_encode_pull(false, false));

  //     uint16_t chanLevelManualCalibration = (uint16_t)initManualAmpCompCalibrationVal[i];
  //     pwm_set_chan_level(RANGE_PWM_SLICES[i], pwm_gpio_to_channel(RANGE_PINS[i]), chanLevelManualCalibration);

  //     pwm_set_chan_level(PW_PWM_SLICES[i / 2], pwm_gpio_to_channel(PW_PINS[i / 2]), 0);

  //   }
  //
  if (manualCalibrationFlag == true) {  // One Ocillator at a time to get correct gap

    int8_t currentCalibrationOscillator = manualCalibrationStage / 2;

    // ALL AT ONCE
    for (int i = 0; i < NUM_OSCILLATORS; i++) {
      uint8_t pioNumber = VOICE_TO_PIO[i];
      PIO pioN = pio[VOICE_TO_PIO[i]];
      uint8_t sm1N = VOICE_TO_SM[i];

      if (i != currentCalibrationOscillator) {
        uint32_t clk_div1 = 200;

        pio_sm_put(pioN, sm1N, clk_div1);
        pio_sm_exec(pioN, sm1N, pio_encode_pull(false, false));
        pwm_set_chan_level(RANGE_PWM_SLICES[i], pwm_gpio_to_channel(RANGE_PINS[i]), 0);
      } else {
        register uint32_t clk_div1 = (int)((float)(eightSysClock_Hz + pioPulseLengthTimesEight - eightPioPulseLength * freq) / freq);

        if (freq == 0)
          clk_div1 = 0;

        pio_sm_put(pioN, sm1N, clk_div1);

        pio_sm_exec(pioN, sm1N, pio_encode_pull(false, false));

        pwm_set_chan_level(RANGE_PWM_SLICES[i], pwm_gpio_to_channel(RANGE_PINS[i]), calibrationValue);

        pwm_set_chan_level(PW_PWM_SLICES[i / 2], pwm_gpio_to_channel(PW_PINS[i / 2]), 0);

        Serial.println((String) "currentCalibrationOscillator: " + (int)currentCalibrationOscillator + (String) "        calibrationValue: " + (int)calibrationValue);
      }
    }
  } else {

    uint8_t pioNumber = VOICE_TO_PIO[currentDCO];
    PIO pioN = pio[VOICE_TO_PIO[currentDCO]];
    uint8_t sm1N = VOICE_TO_SM[currentDCO];

    uint32_t clk_div1 = (int)((float)(eightSysClock_Hz - eightPioPulseLength - (freq * 500)) / freq);

    if (freq == 0)
      clk_div1 = 0;

    pio_sm_put(pioN, sm1N, clk_div1);
    pio_sm_exec(pioN, sm1N, pio_encode_pull(false, false));

    switch (taskAutotuneVoiceMode) {
      case 0:
        pwm_set_chan_level(RANGE_PWM_SLICES[currentDCO], pwm_gpio_to_channel(RANGE_PINS[currentDCO]), calibrationValue);
        break;
      case 1:
        pwm_set_chan_level(RANGE_PWM_SLICES[currentDCO], pwm_gpio_to_channel(RANGE_PINS[currentDCO]), calibrationValue);
        pio_sm_exec(pioN, sm1N, pio_encode_jmp(11 + offset[pioNumber]));
        break;
      case 2:
        pwm_set_chan_level(RANGE_PWM_SLICES[currentDCO], pwm_gpio_to_channel(RANGE_PINS[currentDCO]), chanLevel);
        break;
      case 3:
        chanLevel = get_chan_level_lookup((int32_t)(freq * 100), currentDCO);
        pwm_set_chan_level(RANGE_PWM_SLICES[currentDCO], pwm_gpio_to_channel(RANGE_PINS[currentDCO]), chanLevel);
      case 4:
        pwm_set_chan_level(RANGE_PWM_SLICES[currentDCO], pwm_gpio_to_channel(RANGE_PINS[currentDCO]), calibrationValue);
        break;
    }

    voiceFreq[currentDCO] = freq;
    //}
    // Serial.println((String) "| currentDCO: " + currentDCO + (String) " | freq: " + freq + (String) " | clk_div1: " + clk_div1 + (String) " | ampCompCalibrationVal: " + ampCompCalibrationVal);
  }
}

//  uint16_t get_chan_level_lookup(int32_t intFreq, uint8_t voiceN) {
// #ifndef ENABLE_FS_CALIBRATION
//   voiceN = 0;
// #endif

//   uint16_t chanLevel;
//   uint16_t startArrayPosition = chanLevelVoiceDataSize * voiceN;
//   uint16_t endArrayPosition = startArrayPosition + chanLevelVoiceDataSize;

//   if (intFreq > 520000) {
//     return DIV_COUNTER - 1;
//   }

//   for (uint16_t i = startArrayPosition; i < endArrayPosition; i += 2) {
//     if (intFreq > freq_to_amp_comp_array[i] && intFreq < freq_to_amp_comp_array[i + 2]) {
//       int32_t chanLevel32_2 = (freq_to_amp_comp_array[i + 1] - freq_to_amp_comp_array[i + 3]) * (intFreq - freq_to_amp_comp_array[i]);
//       int32_t chanLevel32_3 = freq_to_amp_comp_array[i + 2] - freq_to_amp_comp_array[i];
//       return round((float)freq_to_amp_comp_array[i + 1] - ((float)chanLevel32_2 / chanLevel32_3));
//     } else if (intFreq == freq_to_amp_comp_array[i]) {
//       return freq_to_amp_comp_array[i + 1];
//     }
//   }
//   return 0;
// }

// Uses linear interpolation  // separated x and y tables
// uint16_t get_chan_level_lookup(int32_t x, uint8_t voiceN) {
//   // Check if x is out of bounds
//   if (x <= ampCompFrequencyArray[voiceN][0]) {
//     return ampCompArray[voiceN][0];
//   }
//   if (x >= ampCompFrequencyArray[voiceN][ampCompTableSize - 1]) {
//     return ampCompArray[voiceN][ampCompTableSize - 1];
//   }

//   // Find the interval x is in
//   for (int i = 0; i < ampCompTableSize - 1; i++) {
//     if (ampCompFrequencyArray[voiceN][i] <= x && x < ampCompFrequencyArray[voiceN][i + 1]) {
//       // Perform linear interpolation using floating-point arithmetic
//       int32_t x0 = ampCompFrequencyArray[voiceN][i];
//       int32_t x1 = ampCompFrequencyArray[voiceN][i + 1];
//       int32_t y0 = ampCompArray[voiceN][i];
//       int32_t y1 = ampCompArray[voiceN][i + 1];

//       // Calculate the interpolated value and round it to the nearest integer
//       return round(y0 + (float)(y1 - y0) * (x - x0) / (x1 - x0));

//     }
//   }

//   // If no interval is found (should not happen), return 0
//   return 0;
// }

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

void voice_task_debug() {

  for (int i = 0; i < NUM_VOICES_TOTAL; i++) {
    if (note_on_flag[i] == 1) {
      note_on_flag_flag[i] = true;
      note_on_flag[i] = 0;
    }
  }

  last_midi_pitch_bend = midi_pitch_bend;
  LAST_DETUNE = DETUNE;
  for (int i = 0; i < NUM_VOICES_TOTAL; i++) {
    if (VOICE_NOTES[i] >= 0) {
      uint8_t note1 = VOICE_NOTES[i] - 36 + OSC1_interval;
      if (note1 > highestNote) {
        note1 -= ((uint8_t(note1 - highestNote) / 12) * 12);
      }
      uint8_t note2 = note1 - 36 + OSC2_interval;
      if (note2 > highestNote) {
        note2 -= ((uint8_t(note2 - highestNote) / 12) * 12);
      }

      uint8_t DCO_A = i * 2;
      uint8_t DCO_B = (i * 2) + 1;

      register float freq;
      register float freq2;

      freq = (float)sNotePitches[note1];
      freq2 = (float)sNotePitches[note2];

      uint8_t pioNumberA = VOICE_TO_PIO[DCO_A];
      uint8_t pioNumberB = VOICE_TO_PIO[DCO_B];
      PIO pioN_A = pio[VOICE_TO_PIO[DCO_A]];
      PIO pioN_B = pio[VOICE_TO_PIO[DCO_B]];
      uint8_t sm1N = VOICE_TO_SM[DCO_A];
      uint8_t sm2N = VOICE_TO_SM[DCO_B];

      register uint32_t clk_div1 = (uint32_t)((eightSysClock_Hz / freq) - eightPioPulseLength - 1);

      if (freq == 0)
        clk_div1 = 0;

      register uint32_t clk_div2 = (uint32_t)((eightSysClock_Hz / freq2) - eightPioPulseLength - 1);

      uint16_t chanLevel = get_chan_level_lookup((int32_t)(freq * 100), (i * 2));
      uint16_t chanLevel2 = get_chan_level_lookup((int32_t)(freq2 * 100), (i * 2) + 1);

      if (oscSync == 0) {

        pio_sm_put(pioN_A, sm1N, clk_div1);
        pio_sm_put(pioN_B, sm2N, clk_div2);
        pio_sm_exec(pioN_A, sm1N, pio_encode_pull(false, false));
        pio_sm_exec(pioN_B, sm2N, pio_encode_pull(false, false));
      } else {
        pio_sm_put(pioN_A, sm1N, clk_div1);
        pio_sm_put(pioN_B, sm2N, clk_div2);
        pio_sm_exec(pioN_A, sm1N, pio_encode_pull(false, false));
        pio_sm_exec(pioN_B, sm2N, pio_encode_pull(false, false));
        if (note_on_flag_flag[i]) {
          switch (oscSync) {
            case 1:
              pio_sm_exec(pioN_A, sm1N, pio_encode_jmp(10 + offset[pioNumberA]));  // OSC Sync MODE
              pio_sm_exec(pioN_B, sm2N, pio_encode_jmp(10 + offset[pioNumberB]));
              break;
            case 2:
              pio_sm_exec(pioN_A, sm1N, pio_encode_jmp(4 + offset[pioNumberA]));  // OSC Half Sync MODE
              pio_sm_exec(pioN_B, sm2N, pio_encode_jmp(12 + offset[pioNumberB]));
              break;
            case 3:
              pio_sm_exec(pioN_A, sm1N, pio_encode_jmp(4 + offset[pioNumberA]));  // OSC 3rd-quarter Sync MODE
              pio_sm_exec(pioN_B, sm2N, pio_encode_jmp(10 + offset[pioNumberB]));
              break;
            default:
              break;
          }
          uint16_t chanLevel = get_chan_level_lookup((int32_t)(freq * 100), (i * 2));
          uint16_t chanLevel2 = get_chan_level_lookup((int32_t)(freq2 * 100), (i * 2) + 1);
          pwm_set_chan_level(RANGE_PWM_SLICES[DCO_A], pwm_gpio_to_channel(RANGE_PINS[DCO_A]), chanLevel);
          pwm_set_chan_level(RANGE_PWM_SLICES[DCO_B], pwm_gpio_to_channel(RANGE_PINS[DCO_B]), chanLevel2);
        }
      }
      if (timer99microsFlag) {
        uint16_t chanLevel = get_chan_level_lookup((int32_t)(freq * 100), (i * 2));
        uint16_t chanLevel2 = get_chan_level_lookup((int32_t)(freq2 * 100), (i * 2) + 1);
        pwm_set_chan_level(RANGE_PWM_SLICES[DCO_A], pwm_gpio_to_channel(RANGE_PINS[DCO_A]), chanLevel);
        pwm_set_chan_level(RANGE_PWM_SLICES[DCO_B], pwm_gpio_to_channel(RANGE_PINS[DCO_B]), chanLevel2);
      }
    }
    note_on_flag_flag[i] = false;
  }
}

inline int32_t interpolatePitchMultiplier(float x_float) {
#ifdef RUNNING_AVERAGE
  unsigned long t_interp = micros();
#endif
  int32_t x = (int32_t)(x_float * (float)multiplierTableScale);
  // Check if x is out of bounds
  if (x <= xMultiplierTable[0]) {
#ifdef RUNNING_AVERAGE
    ra_interpolate_pitch.addValue((float)(micros() - t_interp));
#endif
    return yMultiplierTable[0];
  }
  if (x >= xMultiplierTable[multiplierTableSize - 1]) {
#ifdef RUNNING_AVERAGE
    ra_interpolate_pitch.addValue((float)(micros() - t_interp));
#endif
    return yMultiplierTable[multiplierTableSize - 1];
  }

  // Binary search to find the interval
  int low = 0;
  int high = multiplierTableSize - 1;
  while (low <= high) {
    int mid = (low + high) / 2;
    if (xMultiplierTable[mid] <= x && x < xMultiplierTable[mid + 1]) {
      low = mid;
      break;
    } else if (x < xMultiplierTable[mid]) {
      high = mid - 1;
    } else {
      low = mid + 1;
    }
  }

  // Perform linear interpolation using fixed-point arithmetic
  int32_t x0 = xMultiplierTable[low];
  int32_t x1 = xMultiplierTable[low + 1];
  int32_t y0 = yMultiplierTable[low];
  int32_t y1 = yMultiplierTable[low + 1];

  int32_t y = y0 + ((y1 - y0) * (x - x0) / (x1 - x0));
#ifdef RUNNING_AVERAGE
  ra_interpolate_pitch.addValue((float)(micros() - t_interp));
#endif
  return y;
}

void initMultiplierTables() {

  float y_value;
  double divisor = multiplierTableSize;
  double fraction = 4.00d / divisor;

  for (int i = 0; i < multiplierTableSize; i++) {
    double x;

    if (i == 0) {
      x = -1.00d;
      y_value = 0.25d;
    } else if (i == multiplierTableSize - 1) {
      x = 3;
      y_value = 4;
    } else {
      x = (-1.00d + (fraction * (double)i));

      y_value = (expInterpolationSolveY(x + 1.00d, 1.00d, 3.00d, 0.50d, 2.00d));
    }

    xMultiplierTable[i] = (int32_t)(x * (double)multiplierTableScale);
    yMultiplierTable[i] = (int32_t)(y_value * (double)multiplierTableScale);
  }
}

#ifdef RUNNING_AVERAGE
void print_voice_task_timings() {
  Serial.println("\n=== VOICE_TASK TIMING STATISTICS (microseconds) ===");
  Serial.print("Pitch Bend Calc:      "); 
  if (ra_pitchbend.getCount() > 0) Serial.println(ra_pitchbend.getFastAverage(), 2); else Serial.println("N/A");
  
  Serial.print("OSC2 Detune:          "); 
  if (ra_osc2_detune.getCount() > 0) Serial.println(ra_osc2_detune.getFastAverage(), 2); else Serial.println("N/A");
  
  Serial.print("Portamento:           "); 
  if (ra_portamento.getCount() > 0) Serial.println(ra_portamento.getFastAverage(), 2); else Serial.println("N/A");
  
  Serial.print("ADSR Modifier:        "); 
  if (ra_adsr_modifier.getCount() > 0) Serial.println(ra_adsr_modifier.getFastAverage(), 2); else Serial.println("N/A");
  
  Serial.print("Unison Modifier:      "); 
  if (ra_unison_modifier.getCount() > 0) Serial.println(ra_unison_modifier.getFastAverage(), 2); else Serial.println("N/A");
  
  Serial.print("Drift Modifier:       "); 
  if (ra_drift_modifier.getCount() > 0) Serial.println(ra_drift_modifier.getFastAverage(), 2); else Serial.println("N/A");
  
  Serial.print("Modifiers Combination:"); 
  if (ra_modifiers_combination.getCount() > 0) Serial.println(ra_modifiers_combination.getFastAverage(), 2); else Serial.println("N/A");
  
  Serial.print("Freq Scaling:         "); 
  if (ra_freq_scaling.getCount() > 0) Serial.println(ra_freq_scaling.getFastAverage(), 2); else Serial.println("N/A");
  
  Serial.print("Interpolate Pitch:    "); 
  if (ra_interpolate_pitch.getCount() > 0) Serial.println(ra_interpolate_pitch.getFastAverage(), 2); else Serial.println("N/A");
  
  Serial.print("Get Chan Level:       "); 
  if (ra_get_chan_level.getCount() > 0) Serial.println(ra_get_chan_level.getFastAverage(), 2); else Serial.println("N/A");
  
  Serial.print("Clock Div Calc:       "); 
  if (ra_clk_div_calc.getCount() > 0) Serial.println(ra_clk_div_calc.getFastAverage(), 2); else Serial.println("N/A");
  
  Serial.print("PWM Calculations:     "); 
  if (ra_pwm_calculations.getCount() > 0) Serial.println(ra_pwm_calculations.getFastAverage(), 2); else Serial.println("N/A");
  
  Serial.print("Voice Task Total:     "); 
  Serial.println(ra_voice_task_total.getFastAverage(), 2); Serial.print(" avg, max "); Serial.println(voice_task_max_time);
  
  Serial.println("===================================================\n");
}
#endif