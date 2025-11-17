#include "include_all.h"

// Enable/disable detailed DCO debug report (including OSC1 frequency stages)
#define DCO_DEBUG_REPORT 0


#ifdef RUNNING_AVERAGE
// RunningAverage object definitions for timing measurements
RunningAverage ra_pitchbend(2000);
RunningAverage ra_osc2_detune(2000);
RunningAverage ra_portamento(2000);
RunningAverage ra_adsr_modifier(2000);
RunningAverage ra_unison_modifier(2000);
RunningAverage ra_drift_multiplier(2000);
RunningAverage ra_modifiers_combination(2000);
RunningAverage ra_freq_scaling_x(2000);
RunningAverage ra_freq_scaling_ratio(2000);
RunningAverage ra_freq_scaling_post(2000);
RunningAverage ra_get_chan_level(2000);
RunningAverage ra_pwm_calculations(2000);
RunningAverage ra_voice_task_total(2000);
RunningAverage ra_clk_div_calc(2000);

unsigned long last_timing_print = 0;
unsigned long voice_task_max_time = 0;
const unsigned long TIMING_PRINT_INTERVAL = 1000;  // Print every 5 seconds
#endif

void init_voices() {

  for (int i = 0; i < NUM_VOICES_TOTAL; i++) {
    VOICE_NOTES[i] = DCO_calibration_start_note;
  }

  initMultiplierTables();
  setVoiceMode();
  voice_task_main();
}

// Fast helper: convert a Q16 note (semitones) to Q24 frequency using linear
// interpolation on the sNotePitches_q24 table. Used in slew-rate mode.
static inline int64_t noteQ16_to_freqQ24(int32_t note_q16) {
  const size_t NOTE_TABLE_LEN = sizeof(sNotePitches_q24) / sizeof(sNotePitches_q24[0]);
  if (NOTE_TABLE_LEN == 0) return 0;

  int32_t noteInt = note_q16 >> 16;
  uint32_t frac = (uint32_t)note_q16 & 0xFFFF;

  if (noteInt <= 0) {
    if (NOTE_TABLE_LEN == 1) return sNotePitches_q24[0];
    if (frac == 0) return sNotePitches_q24[0];
    int64_t f0 = sNotePitches_q24[0];
    int64_t f1 = sNotePitches_q24[1];
    int64_t df = f1 - f0;
    return f0 + ((df * (int64_t)frac) >> 16);
  }
  if ((size_t)noteInt >= NOTE_TABLE_LEN - 1) {
    // Clamp to top of table
    return sNotePitches_q24[NOTE_TABLE_LEN - 1];
  }

  if (frac == 0) {
    // Exact semitone, just return table entry (common case).
    return sNotePitches_q24[noteInt];
  }

  int64_t f0 = sNotePitches_q24[noteInt];
  int64_t f1 = sNotePitches_q24[noteInt + 1];
  int64_t df = f1 - f0;
  return f0 + ((df * (int64_t)frac) >> 16);
}

// Helper: convert float Hz to Q24 fixed-point (Hz * 2^24)
static inline int64_t float_to_q24(float f) {
  return (int64_t)lrintf(f * (float)(1 << 24));
}

#ifdef USE_FLOAT_VOICE_TASK
// Helper: convert a semitone index (float) to Hz using sNotePitches[] with linear interpolation.
static inline float noteIndex_to_freqFloat(float noteIndex) {
  const size_t LEN = sizeof(sNotePitches) / sizeof(sNotePitches[0]);
  if (LEN == 0) return 0.0f;
  if (noteIndex <= 0.0f) return sNotePitches[0];
  if (noteIndex >= (float)(LEN - 1)) return sNotePitches[LEN - 1];

  int n0 = (int)floorf(noteIndex);
  int n1 = n0 + 1;
  float t = noteIndex - (float)n0;
  float f0 = sNotePitches[n0];
  float f1 = sNotePitches[n1];
  return f0 + (f1 - f0) * t;
}
#endif

#ifndef USE_FLOAT_VOICE_TASK
inline void voice_task() {
#ifdef RUNNING_AVERAGE
  unsigned long voice_task_start_time = micros();
#endif

  // Track portamento-time and mode changes between calls so we can smoothly
  // retime the glide without introducing pitch discontinuities.
  static uint32_t last_portamento_time = 0;
  static uint8_t last_portamento_mode = PORTA_MODE_TIME;
  uint32_t portaTime = portamento_time;
  uint8_t portaMode = portamento_mode;
  bool portaTimeChanged = (portaTime != last_portamento_time);
  bool portaModeChanged = (portaMode != last_portamento_mode);

  // Pre-calculate pitch bend as a Q24 value. This is done once per voice_task call.
  int32_t calcPitchbend_q24;

#ifdef RUNNING_AVERAGE
  unsigned long t_start = micros();
#endif
  // Optimized: Perform pitch bend calculation entirely in fixed-point Q24.
  // ((bend / 8192.0) - 1.0) * pitchBendMultiplier
  // This avoids float conversions and multiplications in the hot path.
  int32_t bend_normalized_q24 = ((int32_t)midi_pitch_bend << 11) - (1 << 24);
  calcPitchbend_q24 = (int32_t)(((int64_t)bend_normalized_q24 * pitchBendMultiplier_q24) >> 24);
#ifdef RUNNING_AVERAGE
  ra_pitchbend.addValue((float)(micros() - t_start));
#endif

  last_midi_pitch_bend = midi_pitch_bend;
  LAST_DETUNE = DETUNE;

  // Hoist PWM parameters out of the loop. This is critical for performance,
  // as it reads the volatile LFO2toPW variable only once per task run.
  const int16_t local_ADSR1toPWM = ADSR1toPWM;
  const int16_t local_LFO2toPW = LFO2toPW;

  for (int i = 0; i < NUM_VOICES_TOTAL; i++) {

#if DCO_DEBUG_REPORT
    // Debug: track OSC1 frequency at key stages of the pipeline for DCO report.
    float dbg_freq_base_Hz = 0.0f;       // After portamento, before modifiers
    float dbg_freq_after_mod_Hz = 0.0f;  // After all modifiers applied (freq_q24_A)
#endif

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
      // Optimized: Calculate OSC2 detune in Q24 and keep it there.
      // The float conversion has been removed as it is no longer needed.
      // detune = 1.0 + 0.0002 * (256 - val)
      static constexpr int32_t DETUNE_SCALE_Q24 = (int32_t)(0.0002f * (float)(1 << 24) + 0.5f);
      int32_t detune_steps = ((int)256 - OSC2DetuneVal);
      int32_t detune_q24 = (1 << 24) + (detune_steps * DETUNE_SCALE_Q24);
#ifdef RUNNING_AVERAGE
      ra_osc2_detune.addValue((float)(micros() - t_osc2));
#endif

      int64_t freq_q24_A;
      int64_t freq_q24_B;

      uint8_t DCO_A = i * 2;
      uint8_t DCO_B = (i * 2) + 1;

      // Serial.println("VOICE TASK 2");
      ////***********************    PORTAMENTO CODE   ****************************************/////
#ifdef RUNNING_AVERAGE
      unsigned long t_portamento = micros();
#endif
      if (portaTime > 0 /*&& portamento_start != 0 && portamento_stop != 0*/) {
        uint32_t now_us = micros();
        portamentoTimer[i] = now_us - portamentoStartMicros[i];

        if (note_on_flag_flag[i]) {
          // Serial.println("NOTE ON");
          portamentoStartMicros[i] = now_us;

          portamentoTimer[i] = 0;

          // Derive endpoints for portamento
          int64_t stopA_q24 = sNotePitches_q24[note1];
          int64_t stopB_q24 = sNotePitches_q24[note2];
          portamento_stop_q24[DCO_A] = stopA_q24;
          portamento_stop_q24[DCO_B] = stopB_q24;

          int32_t T = (portaTime == 0) ? 1 : (int32_t)portaTime;

          if (portaMode == PORTA_MODE_TIME) {
            // Time-based mode: glide linearly in frequency.
            int64_t startA_q24 = portamento_cur_freq_q24[DCO_A];
            int64_t startB_q24 = portamento_cur_freq_q24[DCO_B];
            portamento_start_q24[DCO_A] = startA_q24;
            portamento_start_q24[DCO_B] = startB_q24;
            portamento_cur_freq_q24[DCO_A] = startA_q24;
            portamento_cur_freq_q24[DCO_B] = startB_q24;

            int64_t dA = stopA_q24 - startA_q24;
            int64_t dB = stopB_q24 - startB_q24;

            // Fixed-time glide: span is covered in approximately portaTime microseconds.
            int64_t halfT = (int64_t)T >> 1;
            int64_t numA = (dA >= 0) ? (dA + halfT) : (dA - halfT);
            int64_t numB = (dB >= 0) ? (dB + halfT) : (dB - halfT);
            freqPortaStep_q24[DCO_A] = (numA / (int64_t)T);
            freqPortaStep_q24[DCO_B] = (numB / (int64_t)T);
          } else {
            // Slew-rate (musical) mode: glide linearly in note-space (semitones).
            // Use current note position as start; if uninitialized, fall back to target note.
            int32_t startNoteA_q16 = porta_note_cur_q16[DCO_A];
            int32_t startNoteB_q16 = porta_note_cur_q16[DCO_B];
            int32_t targetNoteA_q16 = ((int32_t)note1) << 16;
            int32_t targetNoteB_q16 = ((int32_t)note2) << 16;

            if (startNoteA_q16 == 0) startNoteA_q16 = targetNoteA_q16;
            if (startNoteB_q16 == 0) startNoteB_q16 = targetNoteB_q16;

            porta_note_start_q16[DCO_A] = startNoteA_q16;
            porta_note_start_q16[DCO_B] = startNoteB_q16;
            porta_note_stop_q16[DCO_A] = targetNoteA_q16;
            porta_note_stop_q16[DCO_B] = targetNoteB_q16;

            int32_t dNoteA_q16 = porta_note_stop_q16[DCO_A] - porta_note_start_q16[DCO_A];
            int32_t dNoteB_q16 = porta_note_stop_q16[DCO_B] - porta_note_start_q16[DCO_B];

            // Per-microsecond step in Q16 notes.
            // Use symmetric rounding for step magnitude.
            int64_t halfT = (int64_t)T >> 1;
            int64_t numA = (dNoteA_q16 >= 0) ? ((int64_t)dNoteA_q16 + halfT) : ((int64_t)dNoteA_q16 - halfT);
            int64_t numB = (dNoteB_q16 >= 0) ? ((int64_t)dNoteB_q16 + halfT) : ((int64_t)dNoteB_q16 - halfT);
            porta_note_step_q16[DCO_A] = (int32_t)(numA / (int64_t)T);
            porta_note_step_q16[DCO_B] = (int32_t)(numB / (int64_t)T);

            // Ensure we always move for non-zero intervals; otherwise tiny intervals
            // with long times could quantize to zero step and "stick".
            if (dNoteA_q16 != 0 && porta_note_step_q16[DCO_A] == 0) {
              porta_note_step_q16[DCO_A] = (dNoteA_q16 > 0) ? 1 : -1;
            }
            if (dNoteB_q16 != 0 && porta_note_step_q16[DCO_B] == 0) {
              porta_note_step_q16[DCO_B] = (dNoteB_q16 > 0) ? 1 : -1;
            }

            // Initialize current note and frequency at start of glide
            porta_note_cur_q16[DCO_A] = startNoteA_q16;
            porta_note_cur_q16[DCO_B] = startNoteB_q16;
            portamento_cur_freq_q24[DCO_A] = noteQ16_to_freqQ24(startNoteA_q16);
            portamento_cur_freq_q24[DCO_B] = noteQ16_to_freqQ24(startNoteB_q16);
          }
        }

        // Compute current glide position using existing timing/slope
        int32_t elapsed_us = (int32_t)portamentoTimer[i];
        int64_t curA;
        int64_t curB;

        if (portaMode == PORTA_MODE_TIME) {
          if ((uint32_t)elapsed_us > portaTime) {
            // Snap to target once we have exceeded the (current) portamento time
            curA = portamento_stop_q24[DCO_A];
            curB = portamento_stop_q24[DCO_B];
          } else {
            // Absolute-time base in Q24
            curA = portamento_start_q24[DCO_A] + freqPortaStep_q24[DCO_A] * (int64_t)elapsed_us;
            curB = portamento_start_q24[DCO_B] + freqPortaStep_q24[DCO_B] * (int64_t)elapsed_us;
          }
        } else {
          // Slew-rate (musical) mode: step is constant in note-space; stop when we reach the target.
          int32_t dNoteA_q16 = porta_note_stop_q16[DCO_A] - porta_note_start_q16[DCO_A];
          int32_t dNoteB_q16 = porta_note_stop_q16[DCO_B] - porta_note_start_q16[DCO_B];

          int64_t curNoteA_q16 = (int64_t)porta_note_start_q16[DCO_A] + (int64_t)porta_note_step_q16[DCO_A] * (int64_t)elapsed_us;
          int64_t curNoteB_q16 = (int64_t)porta_note_start_q16[DCO_B] + (int64_t)porta_note_step_q16[DCO_B] * (int64_t)elapsed_us;

          // Clamp when passing the target
          if ((dNoteA_q16 >= 0 && curNoteA_q16 >= (int64_t)porta_note_stop_q16[DCO_A]) ||
              (dNoteA_q16 < 0 && curNoteA_q16 <= (int64_t)porta_note_stop_q16[DCO_A])) {
            curNoteA_q16 = porta_note_stop_q16[DCO_A];
          }
          if ((dNoteB_q16 >= 0 && curNoteB_q16 >= (int64_t)porta_note_stop_q16[DCO_B]) ||
              (dNoteB_q16 < 0 && curNoteB_q16 <= (int64_t)porta_note_stop_q16[DCO_B])) {
            curNoteB_q16 = porta_note_stop_q16[DCO_B];
          }

          porta_note_cur_q16[DCO_A] = (int32_t)curNoteA_q16;
          porta_note_cur_q16[DCO_B] = (int32_t)curNoteB_q16;

          curA = noteQ16_to_freqQ24(porta_note_cur_q16[DCO_A]);
          curB = noteQ16_to_freqQ24(porta_note_cur_q16[DCO_B]);
        }

        portamento_cur_freq_q24[DCO_A] = curA;
        portamento_cur_freq_q24[DCO_B] = curB;

        // If the portamento time or mode control changed while gliding, retime the glide
        // from the *current* position so there is no pitch jump, only a change
        // in glide speed / curve.
        if (portaTimeChanged || portaModeChanged) {
          int32_t T = (portaTime == 0) ? 1 : (int32_t)portaTime;

          portamentoStartMicros[i] = now_us;
          portamentoTimer[i] = 0;

          if (portaMode == PORTA_MODE_TIME) {
            // Recompute time-based glide from current frequency.
            int64_t targetA = sNotePitches_q24[note1];
            int64_t targetB = sNotePitches_q24[note2];

            portamento_start_q24[DCO_A] = curA;
            portamento_start_q24[DCO_B] = curB;
            portamento_stop_q24[DCO_A] = targetA;
            portamento_stop_q24[DCO_B] = targetB;

            int64_t dA = targetA - curA;
            int64_t dB = targetB - curB;
            int64_t halfT = (int64_t)T >> 1;
            int64_t numA = (dA >= 0) ? (dA + halfT) : (dA - halfT);
            int64_t numB = (dB >= 0) ? (dB + halfT) : (dB - halfT);
            freqPortaStep_q24[DCO_A] = (numA / (int64_t)T);
            freqPortaStep_q24[DCO_B] = (numB / (int64_t)T);
          } else {
            // Recompute slew-rate glide from current note position.
            int32_t currentNoteA_q16 = porta_note_cur_q16[DCO_A];
            int32_t currentNoteB_q16 = porta_note_cur_q16[DCO_B];
            int32_t targetNoteA_q16 = ((int32_t)note1) << 16;
            int32_t targetNoteB_q16 = ((int32_t)note2) << 16;

            porta_note_start_q16[DCO_A] = currentNoteA_q16;
            porta_note_start_q16[DCO_B] = currentNoteB_q16;
            porta_note_stop_q16[DCO_A] = targetNoteA_q16;
            porta_note_stop_q16[DCO_B] = targetNoteB_q16;

            int32_t dNoteA_q16 = porta_note_stop_q16[DCO_A] - porta_note_start_q16[DCO_A];
            int32_t dNoteB_q16 = porta_note_stop_q16[DCO_B] - porta_note_start_q16[DCO_B];

            int64_t halfT = (int64_t)T >> 1;
            int64_t numA = (dNoteA_q16 >= 0) ? ((int64_t)dNoteA_q16 + halfT) : ((int64_t)dNoteA_q16 - halfT);
            int64_t numB = (dNoteB_q16 >= 0) ? ((int64_t)dNoteB_q16 + halfT) : ((int64_t)dNoteB_q16 - halfT);
            porta_note_step_q16[DCO_A] = (int32_t)(numA / (int64_t)T);
            porta_note_step_q16[DCO_B] = (int32_t)(numB / (int64_t)T);

            if (dNoteA_q16 != 0 && porta_note_step_q16[DCO_A] == 0) {
              porta_note_step_q16[DCO_A] = (dNoteA_q16 > 0) ? 1 : -1;
            }
            if (dNoteB_q16 != 0 && porta_note_step_q16[DCO_B] == 0) {
              porta_note_step_q16[DCO_B] = (dNoteB_q16 > 0) ? 1 : -1;
            }
          }
        }
      } else {
        portamento_cur_freq_q24[DCO_A] = sNotePitches_q24[note1];
        portamento_start_q24[DCO_A] = portamento_cur_freq_q24[DCO_A];
        portamento_stop_q24[DCO_A] = portamento_cur_freq_q24[DCO_A];

        portamento_cur_freq_q24[DCO_B] = sNotePitches_q24[note2];
        portamento_start_q24[DCO_B] = portamento_cur_freq_q24[DCO_B];
        portamento_stop_q24[DCO_B] = portamento_cur_freq_q24[DCO_B];
      }

#if DCO_DEBUG_REPORT
      // Debug: OSC1 base frequency after portamento, before modifiers (in Hz)
      dbg_freq_base_Hz = (float)portamento_cur_freq_q24[DCO_A] / (float)(1 << 24);
#endif
#ifdef RUNNING_AVERAGE
      ra_portamento.addValue((float)(micros() - t_portamento));
#endif
      ////***********************    PORTAMENTO CODE  END    ****************************************/////

#ifdef RUNNING_AVERAGE
      unsigned long t_adsr = micros();
#endif
      // Fixed-point ADSR modifier in Q24: ((linToLog * ADSR1toDETUNE1) / 1080000)
      int64_t ADSRModifier_q24 = 0;
      if (ADSR1toDETUNE1 != 0) {
        // Use precomputed Q24 scale: ADSR1toDETUNE1_scale_q24 = round(ADSR1toDETUNE1 * 2^24 / 1080000)
        ADSRModifier_q24 = (int64_t)linToLogLookup[ADSR1Level[i]] * (int32_t)ADSR1toDETUNE1_scale_q24;
      }
      int64_t ADSRModifierOSC1_q24 = (ADSR3ToOscSelect == 0 || ADSR3ToOscSelect == 2) ? ADSRModifier_q24 : 0;
      int64_t ADSRModifierOSC2_q24 = (ADSR3ToOscSelect == 1 || ADSR3ToOscSelect == 2) ? ADSRModifier_q24 : 0;
#ifdef RUNNING_AVERAGE
      ra_adsr_modifier.addValue((float)(micros() - t_adsr));
      unsigned long t_unison = micros();
#endif

      // Fixed-point unison modifier in Q24: 0.00006 * unisonDetune * step
      static constexpr int32_t UNISON_SCALE_Q24 = (int32_t)(0.0001f * (float)(1 << 24) + 0.5f);
      // Voice-indexed alternating pattern: +1, -1, +2, -2, +3, -3, ...
      int32_t mag = (i >> 1) + 1;
      int32_t sign = ((i & 0x01) == 0) ? 1 : -1;
      int32_t unisonStep = sign * mag;
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
      ra_drift_multiplier.addValue((float)(micros() - t_drift));
#endif

#ifdef RUNNING_AVERAGE
      unsigned long t_modifiers = micros();
#endif
      // Combine modifiers in Q24 (faithful to original float path)
      int32_t detune_fifo_q24 = DETUNE_INTERNAL_FIFO_q24;

      // 1.00001f in Q24 (epsilon ≈ 168 LSBs)
      // Fixed-point equivalent of:
      //   modifiersAll = DETUNE_INTERNAL_FIFO_float + unisonMODIFIER + calcPitchbend + 1.00001f;
      int64_t modifiersAll_q24 =
        (int64_t)detune_fifo_q24 + unisonMODIFIER_q24 + (int64_t)calcPitchbend_q24 + (int64_t)Q24_ONE_EPS;
      int64_t freqModifiers_q24 = ADSRModifierOSC1_q24 + DETUNE_DRIFT_OSC1_q24 + modifiersAll_q24;
      int64_t freq2Modifiers_q24 = ADSRModifierOSC2_q24 + DETUNE_DRIFT_OSC2_q24 + modifiersAll_q24 + (int64_t)DETUNE_INTERNAL2_q24;
#ifdef RUNNING_AVERAGE
      ra_modifiers_combination.addValue((float)(micros() - t_modifiers));
      unsigned long t_freq_scaling_x = micros();
#endif



      // Fast fixed-point equivalent of:
      //   freq  *= interpolatePitchMultiplier(freqModifiers)/multiplierTableScale;
      //   freq2 *= OSC2_detune * interpolatePitchMultiplier(freq2Modifiers)/multiplierTableScale;
      // High-resolution fixed-point x with truncation toward zero (matches original float cast):
      // xQ16 = trunc((q24 * scale) / 2^8) to carry 16 fractional bits of table-units
      int64_t x1_q24s = (freqModifiers_q24 * (int64_t)multiplierTableScale);   // Q24 * int -> Q24
      int64_t x2_q24s = (freq2Modifiers_q24 * (int64_t)multiplierTableScale);  // Q24 * int -> Q24
      int32_t xScaled1_Q16 = (x1_q24s >= 0) ? (int32_t)(x1_q24s >> 8) : (int32_t)(-((-x1_q24s) >> 8));
      int32_t xScaled2_Q16 = (x2_q24s >= 0) ? (int32_t)(x2_q24s >> 8) : (int32_t)(-((-x2_q24s) >> 8));

#ifdef RUNNING_AVERAGE
      ra_freq_scaling_x.addValue((float)(micros() - t_freq_scaling_x));
      unsigned long t_freq_scaling_ratio = micros();
#endif

#if PITCH_USE_RATIO_Q16
      int32_t ratio1_Q16 = interpolateRatioQ16_cached(xScaled1_Q16, DCO_A);
      int32_t ratio2_Q16 = interpolateRatioQ16_cached(xScaled2_Q16, DCO_B);
#ifdef RUNNING_AVERAGE
      ra_freq_scaling_ratio.addValue((float)(micros() - t_freq_scaling_ratio));
      unsigned long t_freq_scaling_post = micros();
#endif

      freq_q24_A = (portamento_cur_freq_q24[DCO_A] * (int64_t)ratio1_Q16) >> 16;
      // Combine OSC2 ratio with detune into one Q16 factor
      // detune_Q16 = round(detune_q24 / 2^8)
      int32_t detune_Q16 = (int32_t)((((int64_t)detune_q24) + 128) >> 8);
      // combined_Q16 = round((ratio2_Q16 * detune_Q16) / 2^16)
      int32_t combined_Q16 = (int32_t)((((int64_t)ratio2_Q16 * (int64_t)detune_Q16) + (1LL << 15)) >> 16);
      freq_q24_B = (portamento_cur_freq_q24[DCO_B] * (int64_t)combined_Q16) >> 16;
#else
#ifdef RUNNING_AVERAGE
      ra_freq_scaling_ratio.addValue((float)(micros() - t_freq_scaling_ratio));
      unsigned long t_freq_scaling_post = micros();
#endif

      int32_t yTab1 = interpolatePitchMultiplierIntQ16_cached(xScaled1_Q16, DCO_A);
      int32_t yTab2 = interpolatePitchMultiplierIntQ16_cached(xScaled2_Q16, DCO_B);
      // Convert yTab -> ratioQ16 using reciprocal-multiply (round((yTab<<16)/10000))
      uint64_t numA = ((uint64_t)(uint32_t)yTab1 << 16) + 5000u;
      int32_t ratio1_Q16_fallback = (int32_t)((numA * 0xD1B71759ULL) >> 45);
      uint64_t numB = ((uint64_t)(uint32_t)yTab2 << 16) + 5000u;
      int32_t ratio2_Q16_fallback = (int32_t)((numB * 0xD1B71759ULL) >> 45);
      // Scale A with ratioQ16
      freq_q24_A = (portamento_cur_freq_q24[DCO_A] * (int64_t)ratio1_Q16_fallback) >> 16;
      // Combine OSC2 ratio with detune into one Q16 factor
      int32_t detune_Q16_fb = (int32_t)((((int64_t)detune_q24) + 128) >> 8);
      int32_t combined_Q16_fb = (int32_t)((((int64_t)ratio2_Q16_fallback * (int64_t)detune_Q16_fb) + (1LL << 15)) >> 16);
      freq_q24_B = (portamento_cur_freq_q24[DCO_B] * (int64_t)combined_Q16_fb) >> 16;
#endif

#if DCO_DEBUG_REPORT
      // Debug: OSC1 frequency after all modifiers applied (in Hz)
      dbg_freq_after_mod_Hz = (float)freq_q24_A / (float)(1 << 24);
#endif


      // Per-cycle: no caching; compute dividers directly from current Q18 frequency

#ifdef RUNNING_AVERAGE
      ra_freq_scaling_post.addValue((float)(micros() - t_freq_scaling_post));
#endif
      // Convert from Q24 fixed-point to a compact Q4 (Hz * 2^4) representation.
      // freq_q24_X is Hz * 2^24, so shifting right by 20 yields Hz * 2^4.
      uint32_t freqA_Q4 = (uint32_t)((freq_q24_A + (1LL << 19)) >> 20);  // round to nearest
      uint32_t freqB_Q4 = (uint32_t)((freq_q24_B + (1LL << 19)) >> 20);  // round to nearest
      if (freqA_Q4 == 0) freqA_Q4 = 1;
      if (freqB_Q4 == 0) freqB_Q4 = 1;

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

      register uint32_t clk_div2, clk_div1;

      uint8_t arbitrary_measured_correction_value = 0; // 60 is a measured correction for the PIO
      
      uint32_t phaseDelay = 0;

      uint32_t total_cycles1, total_cycles2;

#if HIGH_PRECISION_CLKDIV
      // High-precision path: use full Q24 frequency with 64-bit intermediate divide.
      if (freq_q24_A > 0) {
        uint64_t num1 = ((uint64_t)sysClock_Hz << 24) + (uint64_t)(freq_q24_A / 2);
        total_cycles1 = (uint32_t)(num1 / (uint64_t)freq_q24_A);
      } else {
        total_cycles1 = 0;
      }

      if (freq_q24_B > 0) {
        uint64_t num2 = ((uint64_t)sysClock_Hz << 24) + (uint64_t)(freq_q24_B / 2);
        total_cycles2 = (uint32_t)(num2 / (uint64_t)freq_q24_B);
      } else {
        total_cycles2 = 0;
      }
#else
      // --- Oscillator 1: Fixed-point Calculation (no float / 64-bit divide) ---
      // freqA_Q4 represents Hz * 2^4, so multiply sysClock_Hz by 2^4 and divide.
      total_cycles1 = (sysClock_Hz * 16u + (freqA_Q4 / 2u)) / freqA_Q4;  // rounded

      // --- Oscillator 2: Fixed-point Calculation (no float / 64-bit divide) ---
      total_cycles2 = (sysClock_Hz * 16u + (freqB_Q4 / 2u)) / freqB_Q4;  // rounded
#endif

      // Use rounded division when computing clk_div to minimise bias.
      uint32_t total_osr_val1 = total_cycles1 - T_HIGH_TOTAL_CYCLES - T_LOW_OVERHEAD_CYCLES + arbitrary_measured_correction_value;  
      clk_div1 = (total_osr_val1 + (NUM_OSR_CHUNKS / 2u)) / NUM_OSR_CHUNKS;

      // 1. Calculate the dynamic phase and high period on EVERY call.
      //    Use a single high-precision multiply/divide to avoid compounding
      //    rounding error from per-degree quantisation.
      if (oscSync > 1 && phaseAlignOSC2 != 0) {
        // phaseDelay ~= total_cycles2 * phaseAlignOSC2 / 360
        uint64_t phase_num = (uint64_t)total_cycles2 * (uint64_t)phaseAlignOSC2;
        phaseDelay = (uint32_t)((phase_num + 180u) / 360u);
      } else {
        phaseDelay = 0;
      }
      uint32_t y_val2 = pioPulseLength + phaseDelay;
      uint32_t high_total_cycles2 = y_val2 + T_HIGH_OVERHEAD_CYCLES;

      // 2. Calculate the low period using the CORRECT, potentially phase-delayed high period.
      //    This is the critical fix.
      uint32_t total_osr_val2 = total_cycles2 - high_total_cycles2 - T_LOW_OVERHEAD_CYCLES + arbitrary_measured_correction_value;
      clk_div2 = (total_osr_val2 + (NUM_OSR_CHUNKS / 2u)) / NUM_OSR_CHUNKS;

#ifdef RUNNING_AVERAGE
      ra_clk_div_calc.addValue((float)(micros() - t_clk_div));
#endif


#ifdef RUNNING_AVERAGE
      unsigned long t_chan_level = micros();
#endif

      uint16_t chanLevel, chanLevel2;

      // Derive Q16 from Q24 for amp-comp, then to Hz*2^FREQ_FRAC_BITS (get_chan_level does not need higher precision)
      int32_t freq_q16_A = (int32_t)((freq_q24_A + (1LL << 7)) >> 8);
      int32_t freq_q16_B = (int32_t)((freq_q24_B + (1LL << 7)) >> 8);
      const int Q16_TO_FREQ_SHIFT = (16 - FREQ_FRAC_BITS);
      int32_t freqFx_A = (freq_q16_A >= 0) ? (freq_q16_A >> Q16_TO_FREQ_SHIFT)
                                           : -((-freq_q16_A) >> Q16_TO_FREQ_SHIFT);
      int32_t freqFx_B = (freq_q16_B >= 0) ? (freq_q16_B >> Q16_TO_FREQ_SHIFT)
                                           : -((-freq_q16_B) >> Q16_TO_FREQ_SHIFT);
      switch (syncMode) {
        case 0:
          chanLevel = get_chan_level_lookup_fast(freqFx_A, DCO_A);
          chanLevel2 = get_chan_level_lookup_fast(freqFx_B, DCO_B);
          break;
        case 1:
          chanLevel = get_chan_level_lookup_fast((freqFx_A > freqFx_B ? freqFx_A : freqFx_B), DCO_A);
          chanLevel2 = get_chan_level_lookup_fast(freqFx_B, DCO_B);
          break;
        case 2:
          chanLevel = get_chan_level_lookup_fast(freqFx_A, DCO_A);
          chanLevel2 = get_chan_level_lookup_fast((freqFx_A > freqFx_B ? freqFx_A : freqFx_B), DCO_B);
          break;
      }
#ifdef RUNNING_AVERAGE
      ra_get_chan_level.addValue((float)(micros() - t_chan_level));
#endif

      pio_sm_put(pioN_A, sm1N, clk_div1);
      pio_sm_put(pioN_B, sm2N, clk_div2);
      pio_sm_exec(pioN_A, sm1N, pio_encode_pull(false, false));
      pio_sm_exec(pioN_B, sm2N, pio_encode_pull(false, false));

      if (note_on_flag_flag[i]) {
        // --- Reverse Calculation to find the expected output frequency ---
        uint32_t actual_total_osr_val = (clk_div1 * NUM_OSR_CHUNKS);  // This is what the PIO actually gets
        uint32_t actual_total_period = T_HIGH_TOTAL_CYCLES + actual_total_osr_val + T_LOW_OVERHEAD_CYCLES;
        float expected_freq = (double)sysClock_Hz / (double)actual_total_period;

#if DCO_DEBUG_REPORT
        // --- Print Diagnostic Report ---
        Serial.println("----------------[ DCO DEBUG REPORT ]----------------");
        Serial.printf("Target Freq In:   %.2f Hz\n", (float)freq_q24_A / (float)(1 << 24));
        Serial.printf("Total Cycles Calc:  %lu (Target for the whole period)\n", total_cycles1);
        Serial.printf("High Period Fixed:  %lu cycles (From constants)\n", T_HIGH_TOTAL_CYCLES);
        Serial.printf("Low Overhead Fixed: %lu cycles (From constants)\n", T_LOW_OVERHEAD_CYCLES);
        Serial.printf("Total OSR Delay:    %lu cycles (Remaining for loops)\n", total_osr_val1);
        Serial.printf("clk_div (Average):  %lu (Value sent to PIO)\n", clk_div1);
        Serial.println("---");
        Serial.printf("Actual Period Gen:  %lu cycles (High + (clk_div*%u) + Low)\n",
                      actual_total_period, (unsigned)NUM_OSR_CHUNKS);
        Serial.printf("==> Expected Freq Out: %.2f Hz\n", expected_freq);
        Serial.println("---");

        Serial.println("OSC1 Frequency Stages:");
        Serial.printf("  Base after portamento:     %.4f Hz\n", dbg_freq_base_Hz);
        Serial.printf("  After modifiers (Q24):     %.4f Hz\n", dbg_freq_after_mod_Hz);
        Serial.printf("  Quantized by PIO (clkdiv): %.4f Hz\n", expected_freq);
        Serial.println("---");

        Serial.println("OSC1 Modifier Breakdown (Q24/Q16):");
        Serial.printf("  ADSRModifierOSC1_q24:      %.6f\n", (double)ADSRModifierOSC1_q24 / (double)(1 << 24));
        Serial.printf("  DETUNE_DRIFT_OSC1_q24:     %.6f\n", (double)DETUNE_DRIFT_OSC1_q24 / (double)(1 << 24));
        Serial.printf("  detune_fifo_q24:           %.6f\n", (double)detune_fifo_q24 / (double)(1 << 24));
        Serial.printf("  unisonMODIFIER_q24:        %.6f\n", (double)unisonMODIFIER_q24 / (double)(1 << 24));
        Serial.printf("  pitchbend_q24:             %.6f\n", (double)calcPitchbend_q24 / (double)(1 << 24));
        Serial.printf("  Q24_ONE_EPS:               %.6f\n", (double)Q24_ONE_EPS / (double)(1 << 24));
        Serial.printf("  modifiersAll_q24:          %.6f\n", (double)modifiersAll_q24 / (double)(1 << 24));
        Serial.printf("  freqModifiers_q24:         %.6f\n", (double)freqModifiers_q24 / (double)(1 << 24));
        Serial.println("---");

        Serial.println("OSC1 Multiplier Table Inputs:");
        Serial.printf("  x1_q24s (table-units*Q24): %.6f\n", (double)x1_q24s / (double)(1 << 24));
        Serial.printf("  xScaled1_Q16:              %ld (int)\n", (long)xScaled1_Q16);
        Serial.printf("  ratio1_Q16:                %.6f\n", (double)ratio1_Q16 / (double)(1 << 16));
        Serial.println("----------------------------------------------------\n");

#endif

        if (oscSync == 1) {
          pio_sm_exec(pioN_A, sm1N, pio_encode_jmp(10 + offset[pioNumberA]));  // OSC Sync MODE
          pio_sm_exec(pioN_B, sm2N, pio_encode_jmp(10 + offset[pioNumberB]));
        }

        if (oscSync > 1) {
          const uint32_t sm_mask = (1u << sm1N) | (1u << sm2N);

          pio_set_sm_mask_enabled(pioN_A, sm_mask, false);

          pio_sm_clear_fifos(pioN_B, sm2N);
          pio_sm_clear_fifos(pioN_A, sm1N);

          pio_sm_put(pioN_B, sm2N, y_val2);
          pio_sm_exec(pioN_B, sm2N, pio_encode_pull(false, false));
          pio_sm_exec(pioN_B, sm2N, pio_encode_out(pio_y, 31));

          pio_sm_put(pioN_A, sm1N, clk_div1);
          pio_sm_put(pioN_B, sm2N, clk_div2);
          pio_sm_exec(pioN_A, sm1N, pio_encode_pull(false, true));
          pio_sm_exec(pioN_B, sm2N, pio_encode_pull(false, true));

          pio_sm_exec(pioN_A, sm1N, pio_encode_jmp(10 + offset[pioNumberA]));  // OSC Sync MODE
          pio_sm_exec(pioN_B, sm2N, pio_encode_jmp(10 + offset[pioNumberB]));


          pio_set_sm_mask_enabled(pioN_A, sm_mask, true);
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
          // Optimized: This version avoids storing large intermediate products.
          // The multiplication and shift are combined into one expression per modulator,
          // allowing the compiler to make better use of registers.
          int32_t adsr1_delta = ((int32_t)ADSR1Level[i] * local_ADSR1toPWM) >> 11;
          int32_t lfo2_delta = ((int32_t)LFO2Level * local_LFO2toPW) >> 9;
          int32_t pw_calc = (int32_t)DIV_COUNTER_PW - 1 - lfo2_delta - PW[0] + adsr1_delta;

          if (pw_calc < 0) pw_calc = 0;
          if (pw_calc > (int32_t)DIV_COUNTER_PW - 1) pw_calc = (int32_t)DIV_COUNTER_PW - 1;
          PW_PWM[i] = (uint16_t)pw_calc;
#ifdef RUNNING_AVERAGE
          ra_pwm_calculations.addValue((float)(micros() - t_pwm));
#endif
          // PW_PWM[i] = (uint16_t)constrain(DIV_COUNTER_PW - 1 - /*((float)ADSR3Level[i] * ADSR3toPWM_formula)*/ - ((float)LFO2Level * LFO2toPWM_formula) - PW /*+ RANDOMNESS1 + RANDOMNESS2*/, 0, DIV_COUNTER_PW-1);
          pwm_set_chan_level(PW_PWM_SLICES[i], pwm_gpio_to_channel(PW_PINS[i]), get_PW_level_interpolated(PW_PWM[i], i));

        } else {
          pwm_set_chan_level(PW_PWM_SLICES[i], pwm_gpio_to_channel(PW_PINS[i]), 0);
        }
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
#endif

  // Update cached portamento parameters for next call
  last_portamento_time = portaTime;
  last_portamento_mode = portaMode;
}
#endif  // !USE_FLOAT_VOICE_TASK

// Dispatch entry point: select float vs fixed-point implementation at compile time.
inline void voice_task_main() {
#ifdef USE_FLOAT_VOICE_TASK
  voice_task_float();
#else
  voice_task();
#endif
}

#ifdef USE_FLOAT_VOICE_TASK
inline void voice_task_float() {
  #ifdef RUNNING_AVERAGE
    unsigned long voice_task_start_time = micros();
  #endif
  
    // --- Track portamento control changes exactly as in original ---
    static uint32_t last_portamento_time = 0;
    static uint8_t  last_portamento_mode = PORTA_MODE_TIME;
    uint32_t portaTime = portamento_time;
    uint8_t  portaMode = portamento_mode;
    bool portaTimeChanged = (portaTime != last_portamento_time);
    bool portaModeChanged = (portaMode != last_portamento_mode);
  
    // --- 1. Pitch bend as float, equivalent to Q24 math ---
  #ifdef RUNNING_AVERAGE
    unsigned long t_start = micros();
  #endif
    // Use original float pitch bend behaviour, but derive multiplier from Q24
    float pitchBendMultiplier = (float)pitchBendMultiplier_q24 / (float)(1 << 24);
    float calcPitchbend;

    if (midi_pitch_bend == 8192) {
      calcPitchbend = 0.0f;
    } else if (midi_pitch_bend < 8192) {
      calcPitchbend = (((float)midi_pitch_bend / 8190.99f) - 1.0f) * pitchBendMultiplier;
    } else {  // midi_pitch_bend > 8192
      calcPitchbend = (((float)midi_pitch_bend / 8192.99f) - 1.0f) * pitchBendMultiplier;
    }
  
  #ifdef RUNNING_AVERAGE
    ra_pitchbend.addValue((float)(micros() - t_start));
  #endif
  
    last_midi_pitch_bend = midi_pitch_bend;
    LAST_DETUNE          = DETUNE;
  
    // Cache PWM sources like original
    const int16_t local_ADSR1toPWM = ADSR1toPWM;
    const int16_t local_LFO2toPW   = LFO2toPW;
  
    // --- 2. Per-voice loop (mirror original structure) ---
    for (int i = 0; i < NUM_VOICES_TOTAL; ++i) {
  
  #if DCO_DEBUG_REPORT
      float dbg_freq_base_Hz      = 0.0f;
      float dbg_freq_after_mod_Hz = 0.0f;
  #endif
  
      if (note_on_flag[i] == 1) {
        note_on_flag_flag[i] = true;
        note_on_flag[i]      = 0;
      }
  
      if (VOICE_NOTES[i] < 0) {
        note_on_flag_flag[i] = false;
        continue;
      }
  
      // --- 2.1 Note indices (unchanged logic) ---
      uint8_t note1 = VOICE_NOTES[i] - 36 + OSC1_interval;
      if (note1 > highestNote) {
        note1 -= ((uint8_t(note1 - highestNote) / 12) * 12);
      }
      uint8_t note2 = note1 - 36 + OSC2_interval;
      if (note2 > highestNote) {
        note2 -= ((uint8_t(note2 - highestNote) / 12) * 12);
      }
  
      const size_t NOTE_TABLE_LEN = sizeof(sNotePitches) / sizeof(sNotePitches[0]);
      if (note1 >= NOTE_TABLE_LEN) note1 = (uint8_t)(NOTE_TABLE_LEN - 1);
      if (note2 >= NOTE_TABLE_LEN) note2 = (uint8_t)(NOTE_TABLE_LEN - 1);
  
  #ifdef RUNNING_AVERAGE
      unsigned long t_osc2 = micros();
  #endif
      // --- 2.2 OSC2 detune (float equivalent of Q24) ---
      float detuneSteps = (float)((int)256 - OSC2DetuneVal);
      float osc2DetuneRatio = 1.0f + 0.0002f * detuneSteps;
  #ifdef RUNNING_AVERAGE
      ra_osc2_detune.addValue((float)(micros() - t_osc2));
  #endif
  
      // base note frequencies from float table
      float noteFreq1 = sNotePitches[note1];
      float noteFreq2 = sNotePitches[note2];
  
      float freqA, freqB;      // will hold the portamento-processed base freqs
  
      uint8_t DCO_A = i * 2;
      uint8_t DCO_B = i * 2 + 1;
  
      // --- 2.3 Portamento (time mode & slew mode), float-only implementation ---
  #ifdef RUNNING_AVERAGE
      unsigned long t_portamento = micros();
  #endif
  
      if (portaTime > 0) {
        uint32_t now_us = micros();
        portamentoTimer[i] = now_us - portamentoStartMicros[i];
  
        if (note_on_flag_flag[i]) {
          portamentoStartMicros[i] = now_us;
          portamentoTimer[i]       = 0;
  
          float T = (portaTime == 0) ? 1.0f : (float)portaTime;

          if (portaMode == PORTA_MODE_TIME) {
            // TIME-BASED: glide linearly in frequency (Hz) over portaTime microseconds.
            float stopA  = noteFreq1;
            float stopB  = noteFreq2;
            float startA = porta_freq_cur_f[DCO_A];
            float startB = porta_freq_cur_f[DCO_B];

            porta_freq_start_f[DCO_A] = startA;
            porta_freq_start_f[DCO_B] = startB;
            porta_freq_stop_f [DCO_A] = stopA;
            porta_freq_stop_f [DCO_B] = stopB;

            float dA = stopA - startA;
            float dB = stopB - startB;

            float stepA = dA / T;
            float stepB = dB / T;
            if (dA != 0.0f && stepA == 0.0f) stepA = (dA > 0.0f) ? (1.0f / T) : (-1.0f / T);
            if (dB != 0.0f && stepB == 0.0f) stepB = (dB > 0.0f) ? (1.0f / T) : (-1.0f / T);

            porta_freq_step_f[DCO_A] = stepA;  // Hz per microsecond
            porta_freq_step_f[DCO_B] = stepB;

          } else {
            // SLEW-RATE: glide linearly in note-space (semitones).
            float startNoteA = porta_note_cur_f[DCO_A];
            float startNoteB = porta_note_cur_f[DCO_B];
            float targetNoteA = (float)note1;
            float targetNoteB = (float)note2;

            if (startNoteA == 0.0f) startNoteA = targetNoteA;
            if (startNoteB == 0.0f) startNoteB = targetNoteB;

            porta_note_start_f[DCO_A] = startNoteA;
            porta_note_start_f[DCO_B] = startNoteB;
            porta_note_stop_f [DCO_A] = targetNoteA;
            porta_note_stop_f [DCO_B] = targetNoteB;

            float dNoteA = targetNoteA - startNoteA;
            float dNoteB = targetNoteB - startNoteB;

            float stepNoteA = dNoteA / T;
            float stepNoteB = dNoteB / T;
            if (dNoteA != 0.0f && stepNoteA == 0.0f) stepNoteA = (dNoteA > 0.0f) ? (1.0f / T) : (-1.0f / T);
            if (dNoteB != 0.0f && stepNoteB == 0.0f) stepNoteB = (dNoteB > 0.0f) ? (1.0f / T) : (-1.0f / T);

            porta_note_step_f[DCO_A] = stepNoteA;  // semitones per microsecond
            porta_note_step_f[DCO_B] = stepNoteB;

            porta_note_cur_f[DCO_A] = startNoteA;
            porta_note_cur_f[DCO_B] = startNoteB;

            porta_freq_cur_f[DCO_A] = noteIndex_to_freqFloat(startNoteA);
            porta_freq_cur_f[DCO_B] = noteIndex_to_freqFloat(startNoteB);
          }
        }
  
        int32_t elapsed_us = (int32_t)portamentoTimer[i];
  
        float curA, curB;
        if (portaMode == PORTA_MODE_TIME) {
          float startA = porta_freq_start_f[DCO_A];
          float startB = porta_freq_start_f[DCO_B];
          float stopA  = porta_freq_stop_f [DCO_A];
          float stopB  = porta_freq_stop_f [DCO_B];

          if ((uint32_t)elapsed_us > portaTime || portaTime == 0) {
            curA = stopA;
            curB = stopB;
          } else {
            curA = startA + porta_freq_step_f[DCO_A] * (float)elapsed_us;
            curB = startB + porta_freq_step_f[DCO_B] * (float)elapsed_us;
          }

          porta_freq_cur_f[DCO_A] = curA;
          porta_freq_cur_f[DCO_B] = curB;
        } else {
          float startNoteA = porta_note_start_f[DCO_A];
          float startNoteB = porta_note_start_f[DCO_B];
          float stopNoteA  = porta_note_stop_f [DCO_A];
          float stopNoteB  = porta_note_stop_f [DCO_B];

          float dNoteA = stopNoteA - startNoteA;
          float dNoteB = stopNoteB - startNoteB;

          float curNoteA = startNoteA + porta_note_step_f[DCO_A] * (float)elapsed_us;
          float curNoteB = startNoteB + porta_note_step_f[DCO_B] * (float)elapsed_us;

          // Clamp to stop as original
          if ((dNoteA >= 0.0f && curNoteA >= stopNoteA) ||
              (dNoteA <  0.0f && curNoteA <= stopNoteA)) {
            curNoteA = stopNoteA;
          }
          if ((dNoteB >= 0.0f && curNoteB >= stopNoteB) ||
              (dNoteB <  0.0f && curNoteB <= stopNoteB)) {
            curNoteB = stopNoteB;
          }

          porta_note_cur_f[DCO_A] = curNoteA;
          porta_note_cur_f[DCO_B] = curNoteB;

          curA = noteIndex_to_freqFloat(curNoteA);
          curB = noteIndex_to_freqFloat(curNoteB);

          porta_freq_cur_f[DCO_A] = curA;
          porta_freq_cur_f[DCO_B] = curB;
        }
  
        freqA = curA;
        freqB = curB;
  
      } else {
        // No portamento
        freqA = noteFreq1;
        freqB = noteFreq2;

        // Keep float portamento state coherent when portamento is off.
        porta_freq_cur_f[DCO_A] = freqA;
        porta_freq_cur_f[DCO_B] = freqB;
        porta_note_cur_f[DCO_A] = (float)note1;
        porta_note_cur_f[DCO_B] = (float)note2;
      }
  
  #if DCO_DEBUG_REPORT
      dbg_freq_base_Hz = freqA;
  #endif
  
  #ifdef RUNNING_AVERAGE
      ra_portamento.addValue((float)(micros() - t_portamento));
  #endif
  
      // --- 2.4 ADSR detune (float equivalent of Q24) ---
  #ifdef RUNNING_AVERAGE
      unsigned long t_adsr = micros();
  #endif
      float ADSRModifier = 0.0f;
      if (ADSR1toDETUNE1 != 0) {
        float env = (float)linToLogLookup[ADSR1Level[i]];  // original int table
        float scale = (float)ADSR1toDETUNE1_scale_q24 / (float)(1 << 24);
        ADSRModifier = env * scale;
      }
      float ADSRModifierOSC1 = (ADSR3ToOscSelect == 0 || ADSR3ToOscSelect == 2) ? ADSRModifier : 0.0f;
      float ADSRModifierOSC2 = (ADSR3ToOscSelect == 1 || ADSR3ToOscSelect == 2) ? ADSRModifier : 0.0f;
  #ifdef RUNNING_AVERAGE
      ra_adsr_modifier.addValue((float)(micros() - t_adsr));
      unsigned long t_unison = micros();
  #endif
  
      // --- 2.5 Unison modifier (float equivalent) ---
      static constexpr float UNISON_SCALE = 0.0001f; // from original Q24 constant
      int32_t mag  = (i >> 1) + 1;
      int32_t sign = ((i & 0x01) == 0) ? 1 : -1;
      int32_t unisonStep = sign * mag;
      float unisonMODIFIER = (float)unisonDetune * UNISON_SCALE * (float)unisonStep;
  #ifdef RUNNING_AVERAGE
      ra_unison_modifier.addValue((float)(micros() - t_unison));
      unsigned long t_drift = micros();
  #endif
  
      // --- 2.6 Drift modifiers (float) ---
      static constexpr float DRIFT_UNIT = 0.0000005f; // from original
      float driftScale = (float)analogDrift * DRIFT_UNIT;
      float DETUNE_DRIFT_OSC1 = (analogDrift != 0) ? (float)LFO_DRIFT_LEVEL[DCO_A] * driftScale : 0.0f;
      float DETUNE_DRIFT_OSC2 = (analogDrift != 0) ? (float)LFO_DRIFT_LEVEL[DCO_B] * driftScale : 0.0f;
  #ifdef RUNNING_AVERAGE
      ra_drift_multiplier.addValue((float)(micros() - t_drift));
  #endif
  
  #ifdef RUNNING_AVERAGE
      unsigned long t_modifiers = micros();
  #endif
  
      float detune_fifo = (float)DETUNE_INTERNAL_FIFO_q24 / (float)(1 << 24);
      float detune2      = (float)DETUNE_INTERNAL2_q24     / (float)(1 << 24);
      float eps          = (float)Q24_ONE_EPS              / (float)(1 << 24);
      float pitchBendF   = calcPitchbend;
  
      float modifiersAll = detune_fifo + unisonMODIFIER + pitchBendF + eps;
      float freqModifiers1 = ADSRModifierOSC1 + DETUNE_DRIFT_OSC1 + modifiersAll;
      float freqModifiers2 = ADSRModifierOSC2 + DETUNE_DRIFT_OSC2 + modifiersAll + detune2;
  
  #ifdef RUNNING_AVERAGE
      ra_modifiers_combination.addValue((float)(micros() - t_modifiers));
      unsigned long t_freq_scaling_x = micros();
  #endif
  
      // --- 2.7 Multiplier table x scaling & ratio interpolation (float version) ---
      float x1 = freqModifiers1 * (float)multiplierTableScale;
      float x2 = freqModifiers2 * (float)multiplierTableScale;
  
  #ifdef RUNNING_AVERAGE
      ra_freq_scaling_x.addValue((float)(micros() - t_freq_scaling_x));
      unsigned long t_freq_scaling_ratio = micros();
  #endif
  
  #if PITCH_USE_RATIO_Q16
      float ratio1 = interpolateRatioFloat_cached(x1, DCO_A);
      float ratio2 = interpolateRatioFloat_cached(x2, DCO_B);
  #ifdef RUNNING_AVERAGE
      ra_freq_scaling_ratio.addValue((float)(micros() - t_freq_scaling_ratio));
      unsigned long t_freq_scaling_post = micros();
  #endif
      // Apply ratios to portamento frequencies, with osc2 detune (Hz domain).
      float freqA_Hz = freqA * ratio1;
      float freqB_Hz = freqB * (ratio2 * osc2DetuneRatio);
  #else
      // If you keep the IntQ16 path, you can still derive ratio as float from that
      ...
  #endif
  
  #if DCO_DEBUG_REPORT
      dbg_freq_after_mod_Hz = freqA_Hz;
  #endif
  
  #ifdef RUNNING_AVERAGE
      ra_freq_scaling_post.addValue((float)(micros() - t_freq_scaling_post));
  #endif
  
      // --- 2.8 Clock divider calculation (float equivalent) ---
  #ifdef RUNNING_AVERAGE
      unsigned long t_clk_div = micros();
  #endif

      float totalCycles1_f = (float)sysClock_Hz / freqA_Hz;
      float totalCycles2_f = (float)sysClock_Hz / freqB_Hz;

      float correction = 0.0f;   // keep your measured correction if needed
      float total_osr_val1_f = totalCycles1_f - (float)T_HIGH_TOTAL_CYCLES
                             - (float)T_LOW_OVERHEAD_CYCLES + correction;
      uint32_t clk_div1 = (uint32_t)((total_osr_val1_f / (float)NUM_OSR_CHUNKS) + 0.5f);

      float phaseDelay_f = 0.0f;
      if (oscSync > 1 && phaseAlignOSC2 != 0) {
        phaseDelay_f = totalCycles2_f * ((float)phaseAlignOSC2 / 360.0f);
      }
      float y_val2_f = (float)pioPulseLength + phaseDelay_f;
      float high_total_cycles2_f = y_val2_f + (float)T_HIGH_OVERHEAD_CYCLES;
      float total_osr_val2_f = totalCycles2_f - high_total_cycles2_f
                             - (float)T_LOW_OVERHEAD_CYCLES + correction;
      uint32_t clk_div2 = (uint32_t)((total_osr_val2_f / (float)NUM_OSR_CHUNKS) + 0.5f);

  #ifdef RUNNING_AVERAGE
      ra_clk_div_calc.addValue((float)(micros() - t_clk_div));
  #endif
  
      // --- 2.9 Amplitude compensation using engine-agnostic helper ---
  #ifdef RUNNING_AVERAGE
      unsigned long t_chan_level = micros();
  #endif

      uint16_t chanLevel, chanLevel2;
      switch (syncMode) {
        case 0:
          chanLevel  = get_chan_level_for_engine(freqA_Hz, DCO_A);
          chanLevel2 = get_chan_level_for_engine(freqB_Hz, DCO_B);
          break;
        case 1: {
          float maxFreq = (freqA_Hz > freqB_Hz) ? freqA_Hz : freqB_Hz;
          chanLevel  = get_chan_level_for_engine(maxFreq, DCO_A);
          chanLevel2 = get_chan_level_for_engine(freqB_Hz, DCO_B);
          break;
        }
        case 2: {
          float maxFreq = (freqA_Hz > freqB_Hz) ? freqA_Hz : freqB_Hz;
          chanLevel  = get_chan_level_for_engine(freqA_Hz, DCO_A);
          chanLevel2 = get_chan_level_for_engine(maxFreq, DCO_B);
          break;
        }
      }
  #ifdef RUNNING_AVERAGE
      ra_get_chan_level.addValue((float)(micros() - t_chan_level));
  #endif
  
      // --- 2.10 PIO + PWM + PW math (very close to original, but float inside PW calc) ---
      uint8_t pioNumberA = VOICE_TO_PIO[DCO_A];
      uint8_t pioNumberB = VOICE_TO_PIO[DCO_B];
      PIO pioN_A = pio[VOICE_TO_PIO[DCO_A]];
      PIO pioN_B = pio[VOICE_TO_PIO[DCO_B]];
      uint8_t sm1N = VOICE_TO_SM[DCO_A];
      uint8_t sm2N = VOICE_TO_SM[DCO_B];
  
      pio_sm_put(pioN_A, sm1N, clk_div1);
      pio_sm_put(pioN_B, sm2N, clk_div2);
      pio_sm_exec(pioN_A, sm1N, pio_encode_pull(false, false));
      pio_sm_exec(pioN_B, sm2N, pio_encode_pull(false, false));
  
      if (note_on_flag_flag[i]) {
        // Sync logic mirrored from fixed-point voice_task, using float-derived clk_div and phase.
        if (oscSync == 1) {
          pio_sm_exec(pioN_A, sm1N, pio_encode_jmp(10 + offset[pioNumberA]));  // OSC Sync MODE
          pio_sm_exec(pioN_B, sm2N, pio_encode_jmp(10 + offset[pioNumberB]));
        }

        if (oscSync > 1) {
          const uint32_t sm_mask = (1u << sm1N) | (1u << sm2N);

          pio_set_sm_mask_enabled(pioN_A, sm_mask, false);

          pio_sm_clear_fifos(pioN_B, sm2N);
          pio_sm_clear_fifos(pioN_A, sm1N);

          uint32_t y_val2_u = (uint32_t)(y_val2_f + 0.5f);
          pio_sm_put(pioN_B, sm2N, y_val2_u);
          pio_sm_exec(pioN_B, sm2N, pio_encode_pull(false, false));
          pio_sm_exec(pioN_B, sm2N, pio_encode_out(pio_y, 31));

          pio_sm_put(pioN_A, sm1N, clk_div1);
          pio_sm_put(pioN_B, sm2N, clk_div2);
          pio_sm_exec(pioN_A, sm1N, pio_encode_pull(false, true));
          pio_sm_exec(pioN_B, sm2N, pio_encode_pull(false, true));

          pio_sm_exec(pioN_A, sm1N, pio_encode_jmp(10 + offset[pioNumberA]));  // OSC Sync MODE
          pio_sm_exec(pioN_B, sm2N, pio_encode_jmp(10 + offset[pioNumberB]));

          pio_set_sm_mask_enabled(pioN_A, sm_mask, true);
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
          float adsr1_delta = ((float)ADSR1Level[i] * (float)local_ADSR1toPWM) / 2048.0f; // 2^11
          float lfo2_delta  = ((float)LFO2Level    * (float)local_LFO2toPW)   / 512.0f;   // 2^9
          float pw_calc =
              (float)DIV_COUNTER_PW - 1.0f
            - (float)PW[0]
            - lfo2_delta
            + adsr1_delta;
  
          if (pw_calc < 0.0f) pw_calc = 0.0f;
          if (pw_calc > (float)(DIV_COUNTER_PW - 1)) pw_calc = (float)(DIV_COUNTER_PW - 1);
  
          PW_PWM[i] = (uint16_t)lrintf(pw_calc);
  #ifdef RUNNING_AVERAGE
          ra_pwm_calculations.addValue((float)(micros() - t_pwm));
  #endif
          pwm_set_chan_level(PW_PWM_SLICES[i],
                             pwm_gpio_to_channel(PW_PINS[i]),
                             get_PW_level_interpolated(PW_PWM[i], i));
        } else {
          pwm_set_chan_level(PW_PWM_SLICES[i], pwm_gpio_to_channel(PW_PINS[i]), 0);
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
  #endif

  last_portamento_time = portaTime;
  last_portamento_mode = portaMode;
}
#endif  // USE_FLOAT_VOICE_TASK

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

      uint32_t clk_div1 = (uint32_t)((sysClock_Hz / freq) - pioPulseLength) / NUM_OSR_CHUNKS;
      if (freq == 0)
        clk_div1 = 0;

      uint32_t clk_div2;
      uint32_t phaseDelay;

      if (oscSync > 1) {

        clk_div2 = (uint32_t)(sysClock_Hz / freq2);
        phaseDelay = (clk_div2 - pioPulseLength) / 180 * phaseAlignOSC2;
        clk_div2 = (uint32_t)((clk_div2 - phaseDelay) / NUM_OSR_CHUNKS);
      } else {
        clk_div2 = (uint32_t)((sysClock_Hz / freq2) - pioPulseLength) / NUM_OSR_CHUNKS;
      }
      if (freq2 == 0)
        clk_div2 = 0;

      // voice_task_4_time = micros() - voice_task_start_time;

      uint16_t chanLevel, chanLevel2;

      switch (syncMode) {
        case 0:
          chanLevel = get_chan_level_fast_from_float(freq, DCO_A);
          chanLevel2 = get_chan_level_fast_from_float(freq2, DCO_B);
          break;
        case 1:
          chanLevel = get_chan_level_fast_from_float(max(freq, freq2), DCO_A);
          chanLevel2 = get_chan_level_fast_from_float(freq2, DCO_B);
          break;
        case 2:
          chanLevel = get_chan_level_fast_from_float(freq, DCO_A);
          chanLevel2 = get_chan_level_fast_from_float(max(freq, freq2), DCO_B);
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
            pio_sm_put(pioN_B, sm2N, pioPulseLength + phaseDelay);
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

void setSyncMode() {
  for (int i = 0; i < NUM_OSCILLATORS; i++) {
    uint8_t sidesetPin;
    switch (syncMode) {
      case 0:
        sidesetPin = RESET_PINS[i];
        break;
      case 1:
        if (i == 0 || i == 2 || i == 4 || i == 6) {
          sidesetPin = RESET_PINS[i];
        } else {
          sidesetPin = RESET_PINS[i - 1];
        }
        break;
      case 2:
        if (i == 0 || i == 2 || i == 4 || i == 6) {
          sidesetPin = RESET_PINS[i + 1];
        } else {
          sidesetPin = RESET_PINS[i];
        }
        break;
    }

    pio_sm_set_sideset_pins(pio[VOICE_TO_PIO[i]], VOICE_TO_SM[i], sidesetPin);
    pio_gpio_init(pio[VOICE_TO_PIO[i]], sidesetPin);
    pio_sm_restart(pio[VOICE_TO_PIO[i]], VOICE_TO_SM[i]);  // IS THIS NEEDED ?
  }

  for (int i = 0; i < NUM_VOICES_TOTAL; i++) {
    note_on_flag[i] = 1;
  }
}

/**
 * @brief Fast amplitude compensation lookup tuned for the RP2040.
 *
 * This function is optimized for the simple, in-order pipeline of the ARM Cortex-M0+ core.
 * It replaces complex, unpredictable branching with a simple, predictable linear scan
 * to find the correct interpolation window. For small tables, this approach is often
 * faster as it avoids processor pipeline stalls. The core calculation uses the proven,
 * numerically stable fixed-point quadratic method.
 */
inline uint16_t get_chan_level_lookup_fast(int32_t x, uint8_t voiceN) {
  static uint8_t lastWindow[NUM_OSCILLATORS] = { 0 };

  // --- 1. Load pointers to this oscillator's table rows (cache friendly) ---
  const int32_t* freqRow = ampCompFrequencyArray[voiceN];
  const int32_t* ampRow = ampCompArray[voiceN];
  const int32_t* xBaseRow = xBaseWIN[voiceN];
  const int32_t* spanRow = dxWIN[voiceN];
  const uint32_t* invRow_q28 = invDxWIN_q28[voiceN];
  const int32_t* aRow = aQWIN_fast[voiceN];
  const int32_t* bRow = bQWIN_fast[voiceN];
  const uint16_t* cRow = cQWIN[voiceN];
  const bool* plateauRow = plateauWindow[voiceN];

  // --- 2. Handle boundary conditions ---
  if (x <= freqRow[0]) return (uint16_t)ampRow[0];
  const int lastIdx = ampCompTableSize;
  if (x >= freqRow[lastIdx]) return (uint16_t)ampRow[lastIdx];

  // --- 3. Find the correct window using a cached search ---
  int window = lastWindow[voiceN];
  const int maxWindow = ampCompTableSize - 2;
  if (window > maxWindow) window = maxWindow;

  // These small loops are very fast for gliding frequencies (common case).
  while (window > 0 && x < freqRow[window]) {
    --window;
  }
  while (window < maxWindow && x > freqRow[window + 2]) {
    ++window;
  }
  lastWindow[voiceN] = (uint8_t)window;

  // --- 4. Check for and handle plateaus ---
  if (plateauRow[window] && x >= freqRow[window + 1]) {
    return (uint16_t)DIV_COUNTER;
  }

  // --- 5. Core quadratic calculation ---
  // This path is now fully branchless for maximum speed.
  int32_t dx = x - xBaseRow[window];
  const int32_t span = spanRow[window];
  if (dx < 0) dx = 0;
  if (dx > span) dx = span;

  // Calculate t in Q(T_FRAC) using a clean 64-bit multiply-shift.
  // T_FRAC is 14, matching the legacy high-precision path.
  const uint32_t inv_q28 = invRow_q28[window];
  uint32_t t_q = (uint32_t)(((uint64_t)dx * inv_q28) >> (28 - T_FRAC));

  // y(t) = a*t^2 + b*t + c
  // All intermediate math uses 64-bit to prevent overflow.
  int64_t a = aRow[window];
  int64_t b = bRow[window];
  int32_t c = cRow[window];

  // Perform the quadratic evaluation with correct scaling at each step.
  uint32_t t2 = (uint32_t)(((uint32_t)t_q * t_q) >> T_FRAC);
  int32_t term_a = (int32_t)((a * t2) >> T_FRAC);
  int32_t term_b = (int32_t)((b * t_q) >> T_FRAC);

  // Sum the terms (all are Q(T_FRAC)) and then scale back to Q0 with rounding.
  int32_t y_q = term_a + term_b + (c << T_FRAC);
  int32_t y = (y_q + (1 << (T_FRAC - 1))) >> T_FRAC;

  // --- 6. Clamp and return final value ---
  if (y < 0) y = 0;
  if (y > (int32_t)DIV_COUNTER) y = (int32_t)DIV_COUNTER;

  return (uint16_t)y;
}


inline uint16_t get_chan_level_lookup(int32_t x, uint8_t voiceN) {
  static int lastSegIdx[NUM_OSCILLATORS] = { 0 };
  // If frequency is at/above max comp band, drive full-scale level
  if (x >= AMP_COMP_MAX_HZ_Q) {
    return (uint16_t)DIV_COUNTER;
  }
  // Clamp bounds
  if (x <= ampCompFrequencyArray[voiceN][0]) return ampCompArray[voiceN][0];
  if (x >= ampCompFrequencyArray[voiceN][ampCompTableSize]) return ampCompArray[voiceN][ampCompTableSize];

  // Use cached window first
  int i = lastSegIdx[voiceN];
  if (i < 0) i = 0;
  if (i > ampCompTableSize - 2) i = ampCompTableSize - 2;
  if (ampCompFrequencyArray[voiceN][i] <= x && x <= ampCompFrequencyArray[voiceN][i + 2]) {
    // ok
  } else if ((i + 1) <= (ampCompTableSize - 2) && ampCompFrequencyArray[voiceN][i + 1] <= x && x <= ampCompFrequencyArray[voiceN][i + 3]) {
    i = i + 1;
  } else if ((i - 1) >= 0 && ampCompFrequencyArray[voiceN][i - 1] <= x && x <= ampCompFrequencyArray[voiceN][i + 1]) {
    i = i - 1;
  } else {
    // Short linear scan (rare)
    for (int k = 0; k < ampCompTableSize - 1; k++) {
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
  // Instrument high window data for voice 0 (diff handler calls amp_comp_debug_window)
  const uint16_t y1_top = ampCompArray[voiceN][i + 1];
  const uint16_t y2_top = ampCompArray[voiceN][i + 2];
  const int32_t x1 = ampCompFrequencyArray[voiceN][i + 1];
  if ((y1_top >= DIV_COUNTER && y2_top >= DIV_COUNTER) && x >= x1) {
    return (uint16_t)DIV_COUNTER;
  }
  const uint16_t y0 = ampCompArray[voiceN][i];
  int32_t dx = x - x0;
  if (dx < 0) dx = 0;
  if (dx > dx02) dx = dx02;
  // Q28 reciprocal path: t_q = round((dx / dx02) * 2^T_FRAC)
  // invDxWIN_q28 = round(2^28 / dx02) precomputed per window
  const uint32_t invDx_q28 = invDxWIN_q28[voiceN][i];
  uint32_t t_q = (uint32_t)(((uint64_t)(uint32_t)dx * (uint64_t)invDx_q28) >> (28 - T_FRAC));
  if (t_q > (uint32_t)(1u << T_FRAC)) t_q = (uint32_t)(1u << T_FRAC);

  // 32-bit polynomial using aQ/bQ (Q(T_FRAC)) and t in Q(T_FRAC):
  int64_t aQ = (int64_t)aQWIN[voiceN][i];
  int64_t bQ = (int64_t)bQWIN[voiceN][i];
  int32_t cQ = (int32_t)cQWIN[voiceN][i];
  // Fast Q(T_FRAC) polynomial: compute t2 once, do one rounding at the end
  uint32_t t2 = (uint32_t)(((uint64_t)t_q * (uint64_t)t_q) >> T_FRAC);       // Q(T_FRAC)
  int32_t quadQ = (int32_t)(((aQ * (int64_t)t2) >> T_FRAC));                 // Q(T_FRAC)
  int32_t linQ = (int32_t)(((bQ * (int64_t)t_q) >> T_FRAC));                 // Q(T_FRAC)
  int32_t yQ = quadQ + linQ + (cQ << T_FRAC);                                // Q(T_FRAC)
  int32_t y = (int32_t)((((int64_t)yQ) + (1LL << (T_FRAC - 1))) >> T_FRAC);  // Q0
  if (y < 0) y = 0;
  if (y > (int32_t)DIV_COUNTER) y = DIV_COUNTER;
  return (uint16_t)y;
}


// Float reference version (kept for fallback/testing)
inline uint16_t get_chan_level_lookup_float(int32_t x, uint8_t voiceN) {
  if (x <= ampCompFrequencyArray[voiceN][0]) return ampCompArray[voiceN][0];
  if (x >= ampCompFrequencyArray[voiceN][ampCompTableSize]) return ampCompArray[voiceN][ampCompTableSize];
  int low = 0, high = ampCompTableSize;
  while (low <= high) {
    int mid = (low + high) / 2;
    if (ampCompFrequencyArray[voiceN][mid] < x) low = mid + 1;
    else high = mid - 1;
  }
  int k = low - 1;
  if (k < 0) k = 0;
  if (k > ampCompTableSize - 2) k = ampCompTableSize - 2;
  float xf = (float)x / (float)(1u << FREQ_FRAC_BITS);
  float yf = (aCoeff[voiceN][k] * xf + bCoeff[voiceN][k]) * xf + cCoeff[voiceN][k];
  int32_t y = (int32_t)lrintf(yf);
  if (y < 0) y = 0;
  if (y > (int32_t)DIV_COUNTER) y = DIV_COUNTER;
  return (uint16_t)y;
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
        register uint32_t clk_div1 = (uint32_t)(((float)sysClock_Hz / freq) - pioPulseLength) / NUM_OSR_CHUNKS;

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
        pio_sm_exec(pioN, sm1N, pio_encode_jmp(10 + offset[pioNumber]));
        break;
      case 2:
        pwm_set_chan_level(RANGE_PWM_SLICES[currentDCO], pwm_gpio_to_channel(RANGE_PINS[currentDCO]), chanLevel);
        break;
      case 3:
        chanLevel = get_chan_level_lookup_float(freq, currentDCO);
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

      register uint32_t clk_div1 = (uint32_t)(((float)sysClock_Hz / freq) - pioPulseLength - 1) / NUM_OSR_CHUNKS;

      if (freq == 0)
        clk_div1 = 0;

      register uint32_t clk_div2 = (uint32_t)(((float)sysClock_Hz / freq2) - pioPulseLength - 1) / NUM_OSR_CHUNKS;

      uint16_t chanLevel = get_chan_level_lookup_float((int32_t)(freq * 100), (i * 2));
      uint16_t chanLevel2 = get_chan_level_lookup_float((int32_t)(freq2 * 100), (i * 2) + 1);

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
          uint16_t chanLevel = get_chan_level_lookup_float((int32_t)(freq * 100), (i * 2));
          uint16_t chanLevel2 = get_chan_level_lookup_float((int32_t)(freq2 * 100), (i * 2) + 1);
          pwm_set_chan_level(RANGE_PWM_SLICES[DCO_A], pwm_gpio_to_channel(RANGE_PINS[DCO_A]), chanLevel);
          pwm_set_chan_level(RANGE_PWM_SLICES[DCO_B], pwm_gpio_to_channel(RANGE_PINS[DCO_B]), chanLevel2);
        }
      }
      if (timer99microsFlag) {
        uint16_t chanLevel = get_chan_level_lookup_float((int32_t)(freq * 100), (i * 2));
        uint16_t chanLevel2 = get_chan_level_lookup_float((int32_t)(freq2 * 100), (i * 2) + 1);
        pwm_set_chan_level(RANGE_PWM_SLICES[DCO_A], pwm_gpio_to_channel(RANGE_PINS[DCO_A]), chanLevel);
        pwm_set_chan_level(RANGE_PWM_SLICES[DCO_B], pwm_gpio_to_channel(RANGE_PINS[DCO_B]), chanLevel2);
      }
    }
    note_on_flag_flag[i] = false;
  }
}

// Cached variant: pass DCO index to reuse last segment and avoid binary search
inline int32_t interpolatePitchMultiplierIntQ16_cached(int32_t xQ16, int dcoIndex) {
  int32_t xInt = xQ16 >> 16;
  // Clamp to bounds using integer part
  if (xInt <= xMultiplierTable[0]) {
    return yMultiplierTable[0];
  }
  if (xInt >= xMultiplierTable[multiplierTableSize - 1]) {
    return yMultiplierTable[multiplierTableSize - 1];
  }
  int low = interpSegCache[dcoIndex];
  // Validate cache; adjust locally if possible
  if (low < 0 || low > multiplierTableSize - 2 || !(xMultiplierTable[low] <= xInt && xInt < xMultiplierTable[low + 1])) {
    // Try step toward correct segment
    if (low >= 0 && low < multiplierTableSize - 1) {
      if (xInt >= xMultiplierTable[low + 1]) {
        while (low < multiplierTableSize - 2 && xInt >= xMultiplierTable[low + 1]) low++;
      } else if (xInt < xMultiplierTable[low]) {
        while (low > 0 && xInt < xMultiplierTable[low]) low--;
      }
    }
    // If still wrong, do binary search
    if (!(low >= 0 && low < multiplierTableSize - 1 && xMultiplierTable[low] <= xInt && xInt < xMultiplierTable[low + 1])) {
      int l = 0, h = multiplierTableSize - 1;
      while (l <= h) {
        int m = (l + h) >> 1;
        if (xMultiplierTable[m] <= xInt && xInt < xMultiplierTable[m + 1]) {
          low = m;
          break;
        } else if (xInt < xMultiplierTable[m]) {
          h = m - 1;
        } else {
          l = m + 1;
        }
      }
      if (low < 0) low = 0;
      if (low > multiplierTableSize - 2) low = multiplierTableSize - 2;
    }
    interpSegCache[dcoIndex] = (int16_t)low;
  }
  int32_t x0 = xMultiplierTable[low];
  int32_t y0 = yMultiplierTable[low];
  int32_t slope = slopeQ20[low];
#ifdef PITCH_INTERP_USE_Q8
  // 32-bit friendly path: slope in Q8, delta in Q8; total 16 frac bits
  int32_t deltaQ8 = (xQ16 - (x0 << 16)) >> 8;
  int32_t slope8 = slopeQ8[low];
  // Product is Q16; shift by 16 to return table units (with rounding)
  int32_t y = y0 + (int32_t)((((int64_t)deltaQ8 * (int64_t)slope8) + (1LL << 15)) >> 16);
#elif defined(PITCH_INTERP_USE_Q12)
  // Medium-precision path: slope in Q12, delta in Q12; total 24 frac bits
  int32_t deltaQ12 = (xQ16 - (x0 << 16)) >> 4;
  int32_t slope12 = slopeQ12[low];
  int32_t y = y0 + (int32_t)((((int64_t)deltaQ12 * (int64_t)slope12) + (1LL << 23)) >> 24);
#else
  // High-precision path: slope in Q20, delta in Q16
  int32_t deltaQ16 = xQ16 - (x0 << 16);
  int32_t y = y0 + (int32_t)((((int64_t)deltaQ16 * (int64_t)slope) + (1LL << 35)) >> 36);
#endif
  return y;
}

// Cached Q16 ratio interpolator: returns multiplier ratio in Q16 without divide
inline int32_t interpolateRatioQ16_cached(int32_t xQ16, int dcoIndex) {
  int32_t xInt = xQ16 >> 16;
  // Clamp to bounds using integer part
  if (xInt <= xMultiplierTable[0]) {
    // Convert table y->Q16 ratio with rounding using reciprocal-multiply (n/10000 ≈ (n * M) >> 45)
    uint64_t num0 = ((uint64_t)(uint32_t)yMultiplierTable[0] << 16) + 5000u;
    return (int32_t)((num0 * 0xD1B71759ULL) >> 45);
  }
  if (xInt >= xMultiplierTable[multiplierTableSize - 1]) {
    uint64_t numN = ((uint64_t)(uint32_t)yMultiplierTable[multiplierTableSize - 1] << 16) + 5000u;
    return (int32_t)((numN * 0xD1B71759ULL) >> 45);
  }
  int low = interpSegCache[dcoIndex];
  // Validate cache; adjust locally if possible
  if (low < 0 || low > multiplierTableSize - 2 || !(xMultiplierTable[low] <= xInt && xInt < xMultiplierTable[low + 1])) {
    // Try step toward correct segment
    if (low >= 0 && low < multiplierTableSize - 1) {
      if (xInt >= xMultiplierTable[low + 1]) {
        while (low < multiplierTableSize - 2 && xInt >= xMultiplierTable[low + 1]) low++;
      } else if (xInt < xMultiplierTable[low]) {
        while (low > 0 && xInt < xMultiplierTable[low]) low--;
      }
    }
    // If still wrong, do binary search
    if (!(low >= 0 && low < multiplierTableSize - 1 && xMultiplierTable[low] <= xInt && xInt < xMultiplierTable[low + 1])) {
      int l = 0, h = multiplierTableSize - 1;
      while (l <= h) {
        int m = (l + h) >> 1;
        if (xMultiplierTable[m] <= xInt && xInt < xMultiplierTable[m + 1]) {
          low = m;
          break;
        } else if (xInt < xMultiplierTable[m]) {
          h = m - 1;
        } else {
          l = m + 1;
        }
      }
      if (low < 0) low = 0;
      if (low > multiplierTableSize - 2) low = multiplierTableSize - 2;
    }
    interpSegCache[dcoIndex] = (int16_t)low;
  }
  // Interpolate y in table units using high-precision slope (same as IntQ16 path)
  int32_t x0 = xMultiplierTable[low];
  int32_t y0 = yMultiplierTable[low];
  int32_t slope = slopeQ20[low];
  int32_t deltaQ16 = xQ16 - (x0 << 16);
  int32_t yTab = y0 + (int32_t)((((int64_t)deltaQ16 * (int64_t)slope) + (1LL << 35)) >> 36);
  // Convert y (table units) to ratio Q16 with rounding using reciprocal-multiply
  uint64_t num = ((uint64_t)(uint32_t)yTab << 16) + 5000u;  // scale/2
  int32_t ratioQ16 = (int32_t)((num * 0xD1B71759ULL) >> 45);
  return ratioQ16;
}

// Float ratio interpolator: same table/segment logic as interpolateRatioQ16_cached,
// but operates directly in float "table units" and returns a float ratio.
inline float interpolateRatioFloat_cached(float x, int dcoIndex) {
  // Interpret x in same "table units" domain as xMultiplierTableF
  // Clamp to bounds using float table values (mirrors integer path behaviour)
  if (x <= xMultiplierTableF[0]) {
    return yMultiplierTableF[0] / (float)multiplierTableScale;
  }
  if (x >= xMultiplierTableF[multiplierTableSize - 1]) {
    return yMultiplierTableF[multiplierTableSize - 1] / (float)multiplierTableScale;
  }

  int low = interpSegCache[dcoIndex];

  // Validate cache; adjust locally if possible
  if (low < 0 || low > multiplierTableSize - 2 ||
      !(xMultiplierTableF[low] <= x && x < xMultiplierTableF[low + 1])) {
    // Try stepping toward correct segment
    if (low >= 0 && low < multiplierTableSize - 1) {
      if (x >= xMultiplierTableF[low + 1]) {
        while (low < multiplierTableSize - 2 && x >= xMultiplierTableF[low + 1]) {
          ++low;
        }
      } else if (x < xMultiplierTableF[low]) {
        while (low > 0 && x < xMultiplierTableF[low]) {
          --low;
        }
      }
    }
    // If still wrong, fall back to binary search
    if (!(low >= 0 && low < multiplierTableSize - 1 &&
          xMultiplierTableF[low] <= x && x < xMultiplierTableF[low + 1])) {
      int l = 0;
      int h = multiplierTableSize - 1;
      while (l <= h) {
        int m = (l + h) >> 1;
        if (xMultiplierTableF[m] <= x && x < xMultiplierTableF[m + 1]) {
          low = m;
          break;
        } else if (x < xMultiplierTableF[m]) {
          h = m - 1;
        } else {
          l = m + 1;
        }
      }
      if (low < 0) low = 0;
      if (low > multiplierTableSize - 2) low = multiplierTableSize - 2;
    }
    interpSegCache[dcoIndex] = (int16_t)low;
  }

  // Linear interpolation in table units using precomputed float slopes
  float x0 = xMultiplierTableF[low];
  float y0 = yMultiplierTableF[low];
  float m  = slopeF[low];                 // dy/dx as float
  float yTab = y0 + m * (x - x0);         // table units

  // Convert table units to ratio (same as yTab / multiplierTableScale)
  float ratio = yTab / (float)multiplierTableScale;
  return ratio;
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
    x0Q16_tbl[i] = xMultiplierTable[i] << 16;
  }
  // Precompute slopes for fast integer interpolation
  for (int i = 0; i < (multiplierTableSize - 1); ++i) {
    int32_t dx = xMultiplierTable[i + 1] - xMultiplierTable[i];
    if (dx == 0) dx = 1;
    // Precompute slope in Q20 for fast multiply-only interpolation
    int32_t dy = yMultiplierTable[i + 1] - yMultiplierTable[i];
    int64_t numSlope = ((int64_t)dy << 20) + (dx > 0 ? dx / 2 : -dx / 2);
    slopeQ20[i] = (int32_t)(numSlope / (int64_t)dx);
#ifdef PITCH_INTERP_USE_Q8
    // Optional lower-precision slope for 32-bit fast path
    int64_t numSlope8 = ((int64_t)dy << 8) + (dx > 0 ? dx / 2 : -dx / 2);
    slopeQ8[i] = (int32_t)(numSlope8 / (int64_t)dx);
#endif
#ifdef PITCH_INTERP_USE_Q12
    // Medium-precision slope for balanced speed/accuracy
    int64_t numSlope12 = ((int64_t)dy << 12) + (dx > 0 ? dx / 2 : -dx / 2);
    slopeQ12[i] = (int32_t)(numSlope12 / (int64_t)dx);
#endif
  }
  // Initialize per-DCO cache to invalid
  for (int d = 0; d < NUM_VOICES_TOTAL * 2; ++d) interpSegCache[d] = -1;

  // Build float mirrors of multiplier tables and slopes for the float path.
  for (int i = 0; i < multiplierTableSize; ++i) {
    xMultiplierTableF[i] = (float)xMultiplierTable[i];
    yMultiplierTableF[i] = (float)yMultiplierTable[i];
  }
  for (int i = 0; i < multiplierTableSize - 1; ++i) {
    slopeF[i] = (float)slopeQ20[i] / (float)(1 << 20);  // Q20 -> float once at init
  }
}

#ifdef RUNNING_AVERAGE
void print_voice_task_timings() {
  Serial.println("\n=== VOICE_TASK TIMING STATISTICS (microseconds) ===");
  Serial.print("Pitch Bend Calc:      ");
  if (ra_pitchbend.getCount() > 0) Serial.println(ra_pitchbend.getFastAverage(), 2);
  else Serial.println("N/A");

  Serial.print("OSC2 Detune:          ");
  if (ra_osc2_detune.getCount() > 0) Serial.println(ra_osc2_detune.getFastAverage(), 2);
  else Serial.println("N/A");

  Serial.print("Portamento:           ");
  if (ra_portamento.getCount() > 0) Serial.println(ra_portamento.getFastAverage(), 2);
  else Serial.println("N/A");

  Serial.print("ADSR Modifier:        ");
  if (ra_adsr_modifier.getCount() > 0) Serial.println(ra_adsr_modifier.getFastAverage(), 2);
  else Serial.println("N/A");

  Serial.print("Unison Modifier:      ");
  if (ra_unison_modifier.getCount() > 0) Serial.println(ra_unison_modifier.getFastAverage(), 2);
  else Serial.println("N/A");

  Serial.print("Drift Modifier:       ");
  if (ra_drift_multiplier.getCount() > 0) Serial.println(ra_drift_multiplier.getFastAverage(), 2);
  else Serial.println("N/A");

  Serial.print("Modifiers Combination:");
  if (ra_modifiers_combination.getCount() > 0) Serial.println(ra_modifiers_combination.getFastAverage(), 2);
  else Serial.println("N/A");

  Serial.print("Freq Scaling x:       ");
  if (ra_freq_scaling_x.getCount() > 0) Serial.println(ra_freq_scaling_x.getFastAverage(), 2);
  else Serial.println("N/A");

  Serial.print("Freq Scaling ratio:   ");
  if (ra_freq_scaling_ratio.getCount() > 0) Serial.println(ra_freq_scaling_ratio.getFastAverage(), 2);
  else Serial.println("N/A");

  Serial.print("Freq Scaling post:    ");
  if (ra_freq_scaling_post.getCount() > 0) Serial.println(ra_freq_scaling_post.getFastAverage(), 2);
  else Serial.println("N/A");

  Serial.print("Get Chan Level:       ");
  if (ra_get_chan_level.getCount() > 0) Serial.println(ra_get_chan_level.getFastAverage(), 2);
  else Serial.println("N/A");

  Serial.print("Clock Div Calc:       ");
  if (ra_clk_div_calc.getCount() > 0) Serial.println(ra_clk_div_calc.getFastAverage(), 2);
  else Serial.println("N/A");

  Serial.print("PWM Calculations:     ");
  if (ra_pwm_calculations.getCount() > 0) Serial.println(ra_pwm_calculations.getFastAverage(), 2);
  else Serial.println("N/A");

  Serial.print("Voice Task Total:     ");
  Serial.print(ra_voice_task_total.getFastAverage(), 2);
  Serial.print(" avg, max ");
  Serial.println(voice_task_max_time);

#ifdef CLKDIV_BENCHMARK
  // Print clock-divider float vs double comparison stats
  if (clkdiv_bench_count > 0) {
    double avgFloatUs  = clkdiv_time_float_sum_us  / (double)clkdiv_bench_count;
    double avgDoubleUs = clkdiv_time_double_sum_us / (double)clkdiv_bench_count;
    double avgDelta1   = (double)clkdiv_delta1_sum / (double)clkdiv_bench_count;
    double avgDelta2   = (double)clkdiv_delta2_sum / (double)clkdiv_bench_count;
    double avgFreqDiff1 = clkdiv_freq1_diff_sum / (double)clkdiv_bench_count;
    double avgFreqDiff2 = clkdiv_freq2_diff_sum / (double)clkdiv_bench_count;

    Serial.println("=== CLKDIV BENCH ===");
    Serial.print("count=");
    Serial.print(clkdiv_bench_count);
    Serial.print(" avgFloatUs=");
    Serial.print(avgFloatUs, 3);
    Serial.print(" avgDoubleUs=");
    Serial.print(avgDoubleUs, 3);
    Serial.println();

    Serial.print("clk_div1 delta avg=");
    Serial.print(avgDelta1, 3);
    Serial.print(" maxAbs=");
    Serial.print(clkdiv_delta1_max);
    Serial.println();

    Serial.print("clk_div2 delta avg=");
    Serial.print(avgDelta2, 3);
    Serial.print(" maxAbs=");
    Serial.print(clkdiv_delta2_max);
    Serial.println();

    Serial.print("freq1 diff avg=");
    Serial.print(avgFreqDiff1, 6);
    Serial.print(" Hz maxAbs=");
    Serial.print(clkdiv_freq1_diff_max_abs, 6);
    Serial.println();

    Serial.print("freq2 diff avg=");
    Serial.print(avgFreqDiff2, 6);
    Serial.print(" Hz maxAbs=");
    Serial.print(clkdiv_freq2_diff_max_abs, 6);
    Serial.println();

    // Print example of last measured target vs output frequencies
    Serial.print("OSC1 last: target=");
    Serial.print(clkdiv_last_target1_Hz, 6);
    Serial.print(" Hz float=");
    Serial.print(clkdiv_last_out1_float_Hz, 6);
    Serial.print(" Hz double=");
    Serial.print(clkdiv_last_out1_double_Hz, 6);
    Serial.println(" Hz");

    Serial.print("OSC2 last: target=");
    Serial.print(clkdiv_last_target2_Hz, 6);
    Serial.print(" Hz float=");
    Serial.print(clkdiv_last_out2_float_Hz, 6);
    Serial.print(" Hz double=");
    Serial.print(clkdiv_last_out2_double_Hz, 6);
    Serial.println(" Hz");
    Serial.println("====================");

    // Reset accumulators
    clkdiv_bench_count         = 0;
    clkdiv_time_float_sum_us  = 0.0;
    clkdiv_time_double_sum_us = 0.0;
    clkdiv_delta1_sum = clkdiv_delta2_sum = 0;
    clkdiv_delta1_max = clkdiv_delta2_max = 0;
    clkdiv_freq1_diff_sum = clkdiv_freq2_diff_sum = 0.0;
    clkdiv_freq1_diff_max_abs = clkdiv_freq2_diff_max_abs = 0.0;
  }
#endif

  Serial.println("===================================================\n");
}
#endif

inline void voice_task_gold_reference() {
  for (int i = 0; i < NUM_VOICES; i++) {
    uint32_t phaseDelay;

    if (note_on_flag[i] == 1) {
      note_on_flag_flag[i] = true;
      note_on_flag[i] = 0;
    }

    uint8_t DCO_A = i * 2;
    uint8_t DCO_B = (i * 2) + 1;

    // --- High Precision Frequency Calculation (compatible with voice_task_simple) ---
    uint8_t note1 = VOICE_NOTES[i] - 36 + OSC1_interval;
    if (note1 > highestNote) {
      note1 -= ((uint8_t(note1 - highestNote) / 12) * 12);
    }
    long double freq_ld = sNotePitches[note1];

    // Base frequency for OSC2 is the original note, which unison will modify.
    long double freq2_ld = sNotePitches[VOICE_NOTES[i] - 36];

    // High-precision unison detune calculation from first principles.
    if (unisonDetune != 0) {
      // Unison detune steps are 0.25 semitones = 25 cents.
      long double cents_offset = (long double)unisonDetune * 0.12L;
      long double unison_ratio_ld = powl(2.0L, cents_offset / 1200.0L);
      freq2_ld *= unison_ratio_ld;
      freq_ld *= unison_ratio_ld;
    }

    uint8_t pioNumberA = VOICE_TO_PIO[DCO_A];
    uint8_t pioNumberB = VOICE_TO_PIO[DCO_B];
    PIO pioN_A = pio[VOICE_TO_PIO[DCO_A]];
    PIO pioN_B = pio[VOICE_TO_PIO[DCO_B]];
    uint8_t sm1N = VOICE_TO_SM[DCO_A];
    uint8_t sm2N = VOICE_TO_SM[DCO_B];

    // --- High Precision Clock Divider Calculation (Inlined) ---
    uint32_t clk_div1;
    long double raw_div1_ld = (long double)eightSysClock_Hz_u / freq_ld;
    clk_div1 = (raw_div1_ld > eightPioPulseLength) ? (uint32_t)(raw_div1_ld - eightPioPulseLength) : 0;

    uint32_t clk_div2;

    if (oscSync > 1) {
      // This path uses sysClock_Hz
      long double raw_div2_ld = (long double)sysClock_Hz / freq2_ld;
      if (raw_div2_ld > pioPulseLength) {
        long double phaseDelay_ld = (raw_div2_ld - (long double)pioPulseLength) / 180.0L * (long double)phaseAlignOSC2;
        long double clk_div2_ld = (raw_div2_ld - (long double)pioPulseLength - phaseDelay_ld) / 8.0L;
        clk_div2 = (uint32_t)clk_div2_ld;
      } else {
        clk_div2 = 0;
      }
    } else {
      // This path uses eightSysClock_Hz_u
      long double raw_div2_ld = (long double)eightSysClock_Hz_u / freq2_ld;
      clk_div2 = (raw_div2_ld > eightPioPulseLength) ? (uint32_t)(raw_div2_ld - eightPioPulseLength) : 0;
    }

    // voice_task_4_time = micros() - voice_task_start_time;

    uint16_t chanLevel, chanLevel2;

    // Convert back to Q8 for amp comp lookup
    int32_t fxA = (int32_t)(freq_ld > 0.0L ? (freq_ld * (long double)(1u << FREQ_FRAC_BITS) + 0.5L) : 0.0L);
    int32_t fxB = (int32_t)(freq2_ld > 0.0L ? (freq2_ld * (long double)(1u << FREQ_FRAC_BITS) + 0.5L) : 0.0L);

    switch (syncMode) {
      case 0:
        chanLevel = get_chan_level_lookup_fast(fxA, DCO_A);
        chanLevel2 = get_chan_level_lookup_fast(fxB, DCO_B);
        break;
      case 1:
        chanLevel = get_chan_level_lookup_fast((fxA > fxB ? fxA : fxB), DCO_A);
        chanLevel2 = get_chan_level_lookup_fast(fxB, DCO_B);
        break;
      case 2:
        chanLevel = get_chan_level_lookup_fast(fxA, DCO_A);
        chanLevel2 = get_chan_level_lookup_fast((fxA > fxB ? fxA : fxB), DCO_B);
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
          pio_sm_put(pioN_B, sm2N, pioPulseLength + phaseDelay);
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

    note_on_flag_flag[i] = false;
  }
}


// Uses non linear interpolation and coefficients
inline uint16_t get_chan_level_lookup_original(int32_t x, uint8_t voiceN) {
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