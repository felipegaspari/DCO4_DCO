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
  // Fixed-point (Q24) normalization of pitch bend; convert to float once
  // ((bend / 8192.0) - 1.0) ≈ ((bend << 11) - (1<<24)) / (1<<24)
  int32_t bend_q24 = ((int32_t)midi_pitch_bend << 11) - (1 << 24);
  calcPitchbend = ((float)bend_q24 / (float)(1 << 24)) * pitchBendMultiplier;
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
      // Clamp note indexes to table (defensive)
      const size_t NOTE_TABLE_LEN = sizeof(sNotePitches_q24) / sizeof(sNotePitches_q24[0]);
      if (note1 >= NOTE_TABLE_LEN) note1 = (uint8_t)(NOTE_TABLE_LEN - 1);
      if (note2 >= NOTE_TABLE_LEN) note2 = (uint8_t)(NOTE_TABLE_LEN - 1);

#ifdef RUNNING_AVERAGE
      unsigned long t_osc2 = micros();
#endif
      // Fixed-point (Q24) intermediate for OSC2 detune; convert to float once
      // detune = 1.0 + 0.0002 * (256 - val)
      static constexpr int32_t DETUNE_SCALE_Q24 = (int32_t)(0.0002f * (float)(1 << 24) + 0.5f);
      int32_t detune_steps = ((int)256 - OSC2DetuneVal);
      int32_t detune_q24 = (1 << 24) + (detune_steps * DETUNE_SCALE_Q24);
      OSC2_detune = (float)detune_q24 / (float)(1 << 24);
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
      ////***********************    PORTAMENTO CODE   ****************************************/////
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
      ////***********************    PORTAMENTO CODE  END    ****************************************/////
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
      // Fixed-point ADSR modifier in Q24: ((linToLog * ADSR1toDETUNE1) / 1080000)
      int64_t ADSRModifier_q24 = 0;
      if (ADSR1toDETUNE1 != 0) {
        int64_t prod = (int64_t)linToLogLookup[ADSR1Level[i]] * (int16_t)ADSR1toDETUNE1;
        ADSRModifier_q24 = (prod << 24) / 1080000;
      }
      int64_t ADSRModifierOSC1_q24 = (ADSR3ToOscSelect == 0 || ADSR3ToOscSelect == 2) ? ADSRModifier_q24 : 0;
      int64_t ADSRModifierOSC2_q24 = (ADSR3ToOscSelect == 1 || ADSR3ToOscSelect == 2) ? ADSRModifier_q24 : 0;
#ifdef RUNNING_AVERAGE
      ra_adsr_modifier.addValue((float)(micros() - t_adsr));
      unsigned long t_unison = micros();
#endif

      // Fixed-point unison modifier in Q24: 0.00006 * unisonDetune * step
      static constexpr int32_t UNISON_SCALE_Q24 = (int32_t)(0.00006f * (float)(1 << 24) + 0.5f);
      int32_t unisonStep = ((i & 0x01) == 0 ? -(i - 1) : -i);
      int64_t unisonMODIFIER_q24 = (int64_t)unisonDetune * (int64_t)UNISON_SCALE_Q24 * (int64_t)unisonStep;
#ifdef RUNNING_AVERAGE
      ra_unison_modifier.addValue((float)(micros() - t_unison));
      unsigned long t_drift = micros();
#endif

      // Fixed-point drift modifiers in Q24: LFO_LEVEL * (0.0000005 * analogDrift)
      static constexpr int32_t DRIFT_UNIT_Q24 = (int32_t)(0.0000005f * (float)(1 << 24) + 0.5f);
      int32_t driftScale_q24 = (int32_t)((int32_t)analogDrift * DRIFT_UNIT_Q24);
      int64_t DETUNE_DRIFT_OSC1_q24 = (analogDrift != 0) ? ((int64_t)LFO_DRIFT_LEVEL[DCO_A] * (int64_t)driftScale_q24) : 0;
      int64_t DETUNE_DRIFT_OSC2_q24 = (analogDrift != 0) ? ((int64_t)LFO_DRIFT_LEVEL[DCO_B] * (int64_t)driftScale_q24) : 0;
#ifdef RUNNING_AVERAGE
      ra_drift_modifier.addValue((float)(micros() - t_drift));
#endif

      /*   Este bloque tardaba 10 microsegundos */

#ifdef RUNNING_AVERAGE
      unsigned long t_modifiers = micros();
#endif
      // Combine modifiers in Q24 and convert to float once for downstream
      int32_t detune_fifo_q24 = DETUNE_INTERNAL_FIFO_q24;
      int32_t calcPitchbend_q24 = (int32_t)(calcPitchbend * (float)(1 << 24));
      int32_t one_q24 = (1 << 24);
      int64_t modifiersAll_q24 = (int64_t)detune_fifo_q24 + unisonMODIFIER_q24 + (int64_t)calcPitchbend_q24 + (int64_t)one_q24;
      int64_t freqModifiers_q24 = ADSRModifierOSC1_q24 + DETUNE_DRIFT_OSC1_q24 + modifiersAll_q24;
      int64_t freq2Modifiers_q24 = ADSRModifierOSC2_q24 + DETUNE_DRIFT_OSC2_q24 + modifiersAll_q24;
      float freqModifiers = (float)((double)freqModifiers_q24 / (double)(1 << 24));
      float freq2Modifiers = (float)((double)freq2Modifiers_q24 / (double)(1 << 24));
#ifdef RUNNING_AVERAGE
      ra_modifiers_combination.addValue((float)(micros() - t_modifiers));
      unsigned long t_freq_scaling = micros();
#endif

      // Use fixed input to interpolation: x_scaled = (q24 * tableScale) >> 24
      int32_t xScaled1 = (int32_t)((freqModifiers_q24 * (int64_t)multiplierTableScale) >> 24);
      int32_t xScaled2 = (int32_t)((freq2Modifiers_q24 * (int64_t)multiplierTableScale) >> 24);
      int32_t yScaled1 = interpolatePitchMultiplier(xScaled1);
      int32_t yScaled2 = interpolatePitchMultiplier(xScaled2);
      // Build Q24 frequencies from base Q24 and ratios; apply OSC2 detune in Q24
      int64_t freq_q24_A = (portamento_cur_freq_q24[DCO_A] * (int64_t)yScaled1) / (int64_t)multiplierTableScale;
      int64_t freq_q24_B = (portamento_cur_freq_q24[DCO_B] * (int64_t)yScaled2) / (int64_t)multiplierTableScale;
      freq_q24_B = (freq_q24_B * (int64_t)detune_q24) >> 24;
      // Keep float copies for any remaining legacy paths
      freq = (float)((double)freq_q24_A / (double)(1 << 24));
      freq2 = (float)((double)freq_q24_B / (double)(1 << 24));

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
      // Integer clock divider using Q24 frequency: clk_div1 = (eightSysClock_Hz_u << 24)/freq_q24 - eightPioPulseLength
      uint32_t clk_div1;
      if (freq == 0.0f) {
        clk_div1 = 0;
      } else {
        if (freq_q24_A <= 0) {
          clk_div1 = 0;
        } else {
          clk_div1 = (uint32_t)(((uint64_t)eightSysClock_Hz_u << 24) / (uint64_t)freq_q24_A);
          if (clk_div1 > eightPioPulseLength) {
            clk_div1 -= eightPioPulseLength;
          } else {
            clk_div1 = 0;
          }
        }
      }

      register uint32_t clk_div2;
      uint32_t phaseDelay;

      if (oscSync > 1) {

        // clk_div2_raw = (sysClock_Hz << 24)/freq2_q24
        uint32_t clk_div2_raw = (uint32_t)(((uint64_t)sysClock_Hz << 24) / (uint64_t)freq_q24_B);
        uint32_t base = (clk_div2_raw > pioPulseLength) ? (clk_div2_raw - pioPulseLength) : 0;
        phaseDelay = (uint32_t)(((uint64_t)base * (uint64_t)phaseAlignOSC2) / 180ULL);
        uint32_t adj = (base > phaseDelay) ? (base - phaseDelay) : 0;
        clk_div2 = (uint32_t)(adj / 8U);
      } else {
        if (freq2 == 0.0f) {
          clk_div2 = 0;
        } else {
          clk_div2 = (uint32_t)(((uint64_t)eightSysClock_Hz_u << 24) / (uint64_t)freq_q24_B);
          if (clk_div2 > eightPioPulseLength) {
            clk_div2 -= eightPioPulseLength;
          } else {
            clk_div2 = 0;
          }
        }
      }
      if (freq2 == 0.0f)
        clk_div2 = 0;
#ifdef RUNNING_AVERAGE
      ra_clk_div_calc.addValue((float)(micros() - t_clk_div));
#endif

      // voice_task_4_time = micros() - voice_task_start_time;

      uint16_t chanLevel, chanLevel2;

#ifdef RUNNING_AVERAGE
      unsigned long t_chan_level = micros();
#endif
      // Derive fixed-point Hz (Hz * 2^FREQ_FRAC_BITS) from Q24 frequencies for amp-comp lookup
      int32_t freqFx_A = (int32_t)((freq_q24_A + (1LL << (24 - FREQ_FRAC_BITS - 1))) >> (24 - FREQ_FRAC_BITS));
      int32_t freqFx_B = (int32_t)((freq_q24_B + (1LL << (24 - FREQ_FRAC_BITS - 1))) >> (24 - FREQ_FRAC_BITS));
      switch (syncMode) {
        case 0:
          chanLevel = get_chan_level(freqFx_A, DCO_A);
          chanLevel2 = get_chan_level(freqFx_B, DCO_B);
          break;
        case 1:
          chanLevel = get_chan_level((freqFx_A > freqFx_B ? freqFx_A : freqFx_B), DCO_A);
          chanLevel2 = get_chan_level(freqFx_B, DCO_B);
          break;
        case 2:
          chanLevel = get_chan_level(freqFx_A, DCO_A);
          chanLevel2 = get_chan_level((freqFx_A > freqFx_B ? freqFx_A : freqFx_B), DCO_B);
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
          int32_t ADSR1toPW_delta = (ADSR1toPWM != 0) ? (int32_t)(((int64_t)ADSR1Level[i] * (int64_t)ADSR1toPWM_formula_q24) >> 24) : 0;
          int32_t LFO2toPW_delta = (LFO2toPW != 0) ? (int32_t)(((int64_t)LFO2Level * (int64_t)LFO2toPWM_formula_q24) >> 24) : 0;
          int32_t pw_calc = (int32_t)DIV_COUNTER_PW - 1 - LFO2toPW_delta - PW[0] + ADSR1toPW_delta;
          if (pw_calc < 0) pw_calc = 0;
          if (pw_calc > (int32_t)DIV_COUNTER_PW - 1) pw_calc = (int32_t)DIV_COUNTER_PW - 1;
          PW_PWM[i] = (uint16_t)pw_calc;
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

      uint32_t clk_div1 = (uint32_t)(((float)eightSysClock_Hz_u / freq) - eightPioPulseLength);
      if (freq == 0)
        clk_div1 = 0;

      uint32_t clk_div2;
      uint32_t phaseDelay;

      if (oscSync > 1) {

        clk_div2 = (uint32_t)(sysClock_Hz / freq2);
        phaseDelay = (clk_div2 - pioPulseLength) / 180 * phaseAlignOSC2;
        clk_div2 = (uint32_t)((clk_div2 - pioPulseLength - phaseDelay) / 8);
      } else {
        clk_div2 = (uint32_t)(((float)eightSysClock_Hz_u / freq2) - eightPioPulseLength);
      }
      if (freq2 == 0)
        clk_div2 = 0;

      // voice_task_4_time = micros() - voice_task_start_time;

      uint16_t chanLevel, chanLevel2;

      // Compute fixed-point Hz for amp-comp from float frequencies (fallback path)
      int32_t fxA = (int32_t)(freq  > 0.0f ? (freq  * (float)(1u << FREQ_FRAC_BITS) + 0.5f) : 0.0f);
      int32_t fxB = (int32_t)(freq2 > 0.0f ? (freq2 * (float)(1u << FREQ_FRAC_BITS) + 0.5f) : 0.0f);

      switch (syncMode) {
        case 0:
          chanLevel = get_chan_level(fxA, DCO_A);
          chanLevel2 = get_chan_level(fxB, DCO_B);
          break;
        case 1:
          chanLevel = get_chan_level((fxA > fxB ? fxA : fxB), DCO_A);
          chanLevel2 = get_chan_level(fxB, DCO_B);
          break;
        case 2:
          chanLevel = get_chan_level(fxA, DCO_A);
          chanLevel2 = get_chan_level((fxA > fxB ? fxA : fxB), DCO_B);
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

// Uses non linear interpolation and coefficients (fast fixed-point; minimal branching)
inline uint16_t get_chan_level_lookup(int32_t x, uint8_t voiceN) {
  static int lastSegIdx[NUM_OSCILLATORS] = {0};
  // If frequency is at/above max comp band, drive full-scale level
  if (x >= AMP_COMP_MAX_HZ_Q) {
    return (uint16_t)DIV_COUNTER;
  }
  // Clamp bounds
  if (x <= ampCompFrequencyArray[voiceN][0]) return ampCompArray[voiceN][0];
  if (x >= ampCompFrequencyArray[voiceN][ampCompTableSize - 1]) return ampCompArray[voiceN][ampCompTableSize - 1];

  // Use cached window first
  int i = lastSegIdx[voiceN];
  if (i < 0) i = 0;
  if (i > ampCompTableSize - 3) i = ampCompTableSize - 3;
  if (ampCompFrequencyArray[voiceN][i] <= x && x <= ampCompFrequencyArray[voiceN][i + 2]) {
    // ok
  } else if ((i + 1) <= (ampCompTableSize - 3) && ampCompFrequencyArray[voiceN][i + 1] <= x && x <= ampCompFrequencyArray[voiceN][i + 3]) {
    i = i + 1;
  } else if ((i - 1) >= 0 && ampCompFrequencyArray[voiceN][i - 1] <= x && x <= ampCompFrequencyArray[voiceN][i + 1]) {
    i = i - 1;
  } else {
    // Short linear scan (rare)
    for (int k = 0; k < ampCompTableSize - 2; k++) {
      if (ampCompFrequencyArray[voiceN][k] <= x && x <= ampCompFrequencyArray[voiceN][k + 2]) {
        i = k;
        break;
      }
    }
  }
  lastSegIdx[voiceN] = i;

  // Fixed-point eval in window i..i+2
  int32_t x0 = xBaseWIN[voiceN][i];
  int32_t dx02 = dxWIN[voiceN][i];
  int32_t dx = x - x0;
  if (dx < 0) dx = 0;
  if (dx > dx02) dx = dx02;
  // 32-bit fast path: t_q(T_FRAC) = ((dx >> tShift) * tScale_qT)
  uint32_t dx_s = ((uint32_t)dx) >> tShiftWIN[voiceN][i];
  uint32_t t_q = (uint32_t)(dx_s * tScaleWIN_qT[voiceN][i]); // Q(T_FRAC)
  if (t_q > (uint32_t)(1u << T_FRAC)) t_q = (1u << T_FRAC);

  // 32-bit polynomial using aQ/bQ (Q(T_FRAC)) and t in Q(T_FRAC):
  int32_t aQ = (int32_t)aQWIN[voiceN][i];
  int32_t bQ = (int32_t)bQWIN[voiceN][i];
  int32_t cQ = (int32_t)cQWIN[voiceN][i];
  // quad = ((aQ * t_q) >> T_FRAC) * t_q >> T_FRAC ; lin = (bQ * t_q) >> T_FRAC
  int32_t tmp = (int32_t)((aQ * (int32_t)t_q) >> T_FRAC);
  int32_t quad = (int32_t)((tmp * (int32_t)t_q) >> T_FRAC);
  int32_t lin  = (int32_t)((bQ * (int32_t)t_q) >> T_FRAC);
  int32_t y = quad + lin + cQ;
  if (y < 0) y = 0;
  if (y > (int32_t)DIV_COUNTER) y = DIV_COUNTER;
  return (uint16_t)y;
}

// Float reference version (kept for fallback/testing)
inline uint16_t get_chan_level_lookup_float(int32_t x, uint8_t voiceN) {
  if (x <= ampCompFrequencyArray[voiceN][0]) return ampCompArray[voiceN][0];
  if (x >= ampCompFrequencyArray[voiceN][ampCompTableSize - 1]) return ampCompArray[voiceN][ampCompTableSize - 1];
  int low = 0, high = ampCompTableSize - 1;
  while (low <= high) {
    int mid = (low + high) / 2;
    if (ampCompFrequencyArray[voiceN][mid] < x) low = mid + 1;
    else high = mid - 1;
  }
  int k = low - 1;
  if (k < 0) k = 0;
  if (k > ampCompTableSize - 3) k = ampCompTableSize - 3;
  float xf = (float)x / (float)(1u << FREQ_FRAC_BITS);
    float yf = (aCoeff[voiceN][k] * xf + bCoeff[voiceN][k]) * xf + cCoeff[voiceN][k];
  int32_t y = (int32_t)lrintf(yf);
  if (y < 0) y = 0;
  if (y > (int32_t)DIV_COUNTER) y = DIV_COUNTER;
  return (uint16_t)y;
}

// Unified wrapper to switch implementation at compile time
inline uint16_t get_chan_level(int32_t x, uint8_t voiceN) {
#ifdef AMP_COMP_USE_FLOAT
  return get_chan_level_lookup_float(x, voiceN);
#else
  return get_chan_level_lookup(x, voiceN);
#endif
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
        register uint32_t clk_div1 = (int)((float)(eightSysClock_Hz_u + pioPulseLengthTimesEight - eightPioPulseLength * freq) / freq);

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

    uint32_t clk_div1 = (int)((float)(eightSysClock_Hz_u - eightPioPulseLength - (freq * 500)) / freq);

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
        chanLevel = get_chan_level((int32_t)(freq * 100), currentDCO);
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

      register uint32_t clk_div1 = (uint32_t)(((float)eightSysClock_Hz_u / freq) - eightPioPulseLength - 1);

      if (freq == 0)
        clk_div1 = 0;

      register uint32_t clk_div2 = (uint32_t)(((float)eightSysClock_Hz_u / freq2) - eightPioPulseLength - 1);

      uint16_t chanLevel = get_chan_level((int32_t)(freq * 100), (i * 2));
      uint16_t chanLevel2 = get_chan_level((int32_t)(freq2 * 100), (i * 2) + 1);

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
          uint16_t chanLevel = get_chan_level((int32_t)(freq * 100), (i * 2));
          uint16_t chanLevel2 = get_chan_level((int32_t)(freq2 * 100), (i * 2) + 1);
          pwm_set_chan_level(RANGE_PWM_SLICES[DCO_A], pwm_gpio_to_channel(RANGE_PINS[DCO_A]), chanLevel);
          pwm_set_chan_level(RANGE_PWM_SLICES[DCO_B], pwm_gpio_to_channel(RANGE_PINS[DCO_B]), chanLevel2);
        }
      }
      if (timer99microsFlag) {
        uint16_t chanLevel = get_chan_level((int32_t)(freq * 100), (i * 2));
        uint16_t chanLevel2 = get_chan_level((int32_t)(freq2 * 100), (i * 2) + 1);
        pwm_set_chan_level(RANGE_PWM_SLICES[DCO_A], pwm_gpio_to_channel(RANGE_PINS[DCO_A]), chanLevel);
        pwm_set_chan_level(RANGE_PWM_SLICES[DCO_B], pwm_gpio_to_channel(RANGE_PINS[DCO_B]), chanLevel2);
      }
    }
    note_on_flag_flag[i] = false;
  }
}

inline int32_t interpolatePitchMultiplier(int32_t x) {
#ifdef RUNNING_AVERAGE
  unsigned long t_interp = micros();
#endif
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
  Serial.print(ra_voice_task_total.getFastAverage(), 2); Serial.print(" avg, max "); Serial.println(voice_task_max_time);
  
  Serial.println("===================================================\n");
}
#endif