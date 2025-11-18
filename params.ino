// Central parameter router for shared parameters.
//
// This module maps numeric parameter IDs (used over Serial / between MCUs)
// to concrete synth state changes on this MCU. It is the single place where
// "what does parameter X actually do?" is implemented.
//
// The stable parameter ID definitions live in params_def.h so all MCUs and
// tools can share the same mapping.
//
// High-level flow:
//   1) Some control source (front panel, STM32, MIDI, editor) decides that
//      parameter P should change to value V.
//   2) It sends P and V over the link (e.g. via Serial 'p'/'w'/'x' commands).
//   3) The receiver ends up calling:
//         update_parameters(paramNumber, paramValue);
//   4) update_parameters() looks up paramNumber in paramTable[] and calls
//      the corresponding apply_param_*() function.
//   5) That function updates internal state and performs any required
//      precomputations (fixed-point scales, LFO frequencies, etc.).
//
// How to add or modify a parameter on this MCU:
//   1) Define or reuse a ParamId in params_def.h.
//   2) Implement a new apply_param_*() function below that:
//        - Accepts int16_t (the raw transport value).
//        - Updates the appropriate globals / DSP structures.
//        - Computes any derived values (e.g. Q24 scales, Hz values).
//   3) Add an entry to paramTable[] that maps your ParamId to the new
//      apply_param_*() function.
//   4) Make sure the sending side (other MCU / UI) uses the same ParamId,
//      and sends values in the range/format your apply function expects.
//
// Notes:
//   - The transport is 16-bit (int16_t) for this router. For values derived
//     from larger types (e.g. 'x' 32-bit commands), the conversion happens
//     before calling update_parameters(), preserving the old behavior.
//   - If an unknown paramNumber is received, update_parameters() simply
//     ignores it (same as the old default: case).


// ---- Apply functions for each parameter -----------------------------

static void apply_param_sqr1_status(int16_t v) {
  sqr1Status = v;
  // update_waveSelector(4);  // keep commented behavior as in original code
}

static void apply_param_adsr3_to_osc_select(int16_t v) {
  ADSR3ToOscSelect = v;
}

static void apply_param_lfo1_waveform(int16_t v) {
  LFO1Waveform = v;
  LFO1_class.setWaveForm(LFO1Waveform);
  LFO1_class.setMode0Freq((float)LFO1Speed, micros());
}

static void apply_param_lfo2_waveform(int16_t v) {
  LFO2Waveform = v;
  LFO2_class.setWaveForm(LFO2Waveform);
  LFO2_class.setMode0Freq((float)LFO2Speed, micros());
}

static void apply_param_osc1_interval(int16_t v) {
  OSC1_interval = v;
}

static void apply_param_osc2_interval(int16_t v) {
  OSC2_interval = v;
}

static void apply_param_osc2_detune_val(int16_t v) {
  OSC2DetuneVal = 512 - v;
}

static void apply_param_lfo2_to_detune2(int16_t v) {
  float lfo2_amt = (float)expConverterFloat((uint8_t)v, 500) / 275000.0f;
  LFO2toDETUNE2_q24 = (int32_t)(lfo2_amt * (float)(1 << 24) + 0.5f);
}

static void apply_param_osc_sync_mode(int16_t v) {
  oscSync = v;
  if (oscSync < 2) {
    for (int i = 0; i < NUM_OSCILLATORS; i++) {
      pio_sm_put(pio[VOICE_TO_PIO[i]], VOICE_TO_SM[i], pioPulseLength);
      pio_sm_exec(pio[VOICE_TO_PIO[i]], VOICE_TO_SM[i], pio_encode_pull(false, false));
      pio_sm_exec(pio[VOICE_TO_PIO[i]], VOICE_TO_SM[i], pio_encode_out(pio_y, 31));
    }
  } else {
    if (oscSync > 8) {
      phaseAlignOSC2 = oscSync * 2;
    } else {
      switch (oscSync) {
        case 2:
          phaseAlignOSC2 = 45;
          break;
        case 3:
          phaseAlignOSC2 = 90;
          break;
        case 4:
          phaseAlignOSC2 = 135;
          break;
        case 5:
          phaseAlignOSC2 = 180;
          break;
        case 6:
          phaseAlignOSC2 = 225;
          break;
        case 7:
          phaseAlignOSC2 = 270;
          break;
        case 8:
          phaseAlignOSC2 = 315;
          break;
        default:
          break;
      }
    }
  }
  for (int i = 0; i < NUM_VOICES_TOTAL; i++) {
    note_on_flag[i] = 1;
  }
}

static void apply_param_portamento_time(int16_t v) {
  uint8_t portaSerial = (uint8_t)v;
  if (portaSerial == 0) {
    portamento_time = 0;
  } else if (portaSerial < 200) {
    portamento_time = (expConverter(portaSerial + 15, 100) * 2000);
  } else {
    portamento_time = map(portaSerial, 200, 255, 1000000, 10000000);
  }
}

static void apply_param_portamento_mode(int16_t v) {
  // Portamento mode: 0 = fixed-time glide, 1 = analog-style slew-rate
  portamento_mode = (v == 0) ? PORTA_MODE_TIME : PORTA_MODE_SLEW;
}

static void apply_param_calibration_value(int16_t /*v*/) {
  // Placeholder: original code did nothing but kept the ID reserved.
}

static void apply_param_voice_mode(int16_t v) {
  voiceMode = v;
  setVoiceMode();
}

static void apply_param_unison_detune(int16_t v) {
  unisonDetune = v;
}

static void apply_param_analog_drift_amount(int16_t v) {
  analogDrift = v;
}

static void apply_param_analog_drift_speed(int16_t v) {
  analogDriftSpeed = v;
  for (int i = 0; i < NUM_OSCILLATORS; i++) {
    LFO_DRIFT_SPEED_OFFSET[i] =
      (float)(1.00f - (float)((float)analogDriftSpread * 0.005f) +
              (float)((float)analogDriftSpread * 0.00125f * (float)i)) *
      (float)expConverterFloat((float)analogDriftSpeed, 5000);
    LFO_DRIFT_CLASS[i].setMode0Freq(LFO_DRIFT_SPEED_OFFSET[i], micros());
  }
}

static void apply_param_analog_drift_spread(int16_t v) {
  analogDriftSpread = v;
  for (int i = 0; i < NUM_OSCILLATORS; i++) {
    LFO_DRIFT_SPEED_OFFSET[i] =
      (float)(1.00f - (float)((float)analogDriftSpread * 0.005f) +
              (float)((float)analogDriftSpread * 0.00125f * (float)i)) *
      (float)expConverterFloat((float)analogDriftSpeed, 5000);
    LFO_DRIFT_CLASS[i].setMode0Freq(LFO_DRIFT_SPEED_OFFSET[i], micros());
  }
}

static void apply_param_sync_mode(int16_t v) {
  syncMode = v;
  setSyncMode();
}

static void apply_param_lfo1_to_dco(int16_t v) {
  LFO1toDCOVal = v;
  // Compute LFO1->DCO modulation depth both in float (for any legacy use)
  // and in Q24 fixed-point for the fast detune path.
  float lfo1_amt = (float)expConverterFloat(LFO1toDCOVal, 500) / 275000.0f;
  LFO1toDCO = lfo1_amt;
  LFO1toDCO_q24 = (int32_t)(lfo1_amt * (float)(1 << 24) + 0.5f);
}

static void apply_param_lfo1_speed(int16_t v) {
  LFO1SpeedVal = v;
  LFO1Speed = expConverterFloat(LFO1SpeedVal, 5000);
  LFO1_class.setMode0Freq((float)LFO1Speed, micros());
}

static void apply_param_lfo2_speed(int16_t v) {
  LFO2SpeedVal = v;
  LFO2Speed = expConverterFloat(LFO2SpeedVal, 5000);
  LFO2_class.setMode0Freq((float)LFO2Speed, micros());
}

static void apply_param_lfo2_to_pw(int16_t v) {
  LFO2toPW = (int16_t)v;
}

static void apply_param_adsr1_to_pwm(int16_t v) {
  ADSR1toPWM = (int16_t)v - 512;
}

static void apply_param_adsr1_to_detune1(int16_t v) {
  // ADSR1toDETUNE1 controls how much ADSR1 modulates pitch (detune).
  // Original float formula was:
  //   ADSRModifier = linToLogLookup[level] * (ADSR1toDETUNE1 / 1080000.0f)
  // We now precompute a Q24 scale factor so the per-voice path stays fixed-point:
  //   ADSRModifier_q24 = linToLogLookup[level] * ADSR1toDETUNE1_scale_q24
  ADSR1toDETUNE1 = (int16_t)v;
  if (ADSR1toDETUNE1 == 0) {
    ADSR1toDETUNE1_scale_q24 = 0;
  } else {
    int64_t num = ((int64_t)ADSR1toDETUNE1 << 24);
    // Symmetric rounding toward nearest for positive/negative values
    const int32_t denom = 1080000;
    if (num >= 0) {
      num += (denom / 2);
    } else {
      num -= (denom / 2);
    }
    ADSR1toDETUNE1_scale_q24 = (int32_t)(num / denom);
  }
}

static void apply_param_adsr1_curve(int16_t /*v*/) {
  // = v " ADSR1 Curve" (reserved, no behavior yet)
}

static void apply_param_adsr2_curve(int16_t /*v*/) {
  // = v " ADSR2 Curve" (reserved, no behavior yet)
}

static void apply_param_pwm_pots_manual(int16_t v) {
  PWMPotsControlManual = v;
}

static void apply_param_function_key(int16_t /*v*/) {
  // = v " FUNCTION KEY" (handled elsewhere or reserved)
}

static void apply_param_calibration_flag(int16_t v) {
  calibrationFlag = v;
}

static void apply_param_manual_calibration_flag(int16_t v) {
  // When manual calibration is active, both flags follow this param.
  // When it is turned off (v == 0), persist the final per‑oscillator
  // manualCalibrationOffset[] values to the filesystem so they survive reboot.
  if (v == 0) {
    // Manual stage ended – commit all offsets in one batch.
    for (uint8_t osc = 0; osc < NUM_OSCILLATORS; ++osc) {
      update_FS_ManualCalibrationOffset(osc, manualCalibrationOffset[osc]);
    }
  }
  manualCalibrationFlag = v;
  calibrationFlag = v;
}

static void apply_param_manual_calibration_stage(int16_t v) {
  manualCalibrationStage = (int8_t)v;
}

static void apply_param_manual_calibration_offset(int16_t v) {
  manualCalibrationOffset[(uint8_t)manualCalibrationStage / 2] = (int8_t)v;
  // initManualAmpCompCalibrationVal[manualCalibrationStage / 2] =
  //   initManualAmpCompCalibrationValPreset +
  //   manualCalibrationOffset[manualCalibrationStage / 2]; // WAS WRONG ?
}

// ---- Parameter table ------------------------------------------------

static const ParamDescriptorT<int16_t> paramTable[] = {
  { PARAM_SQR1_STATUS,               apply_param_sqr1_status },
  { PARAM_ADSR3_TO_OSC_SELECT,       apply_param_adsr3_to_osc_select },
  { PARAM_LFO1_WAVEFORM,             apply_param_lfo1_waveform },
  { PARAM_LFO2_WAVEFORM,             apply_param_lfo2_waveform },
  { PARAM_OSC1_INTERVAL,             apply_param_osc1_interval },
  { PARAM_OSC2_INTERVAL,             apply_param_osc2_interval },
  { PARAM_OSC2_DETUNE_VAL,           apply_param_osc2_detune_val },
  { PARAM_LFO2_TO_DETUNE2,           apply_param_lfo2_to_detune2 },
  { PARAM_OSC_SYNC_MODE,             apply_param_osc_sync_mode },
  { PARAM_PORTAMENTO_TIME,           apply_param_portamento_time },
  { PARAM_PORTAMENTO_MODE,           apply_param_portamento_mode },
  { PARAM_CALIBRATION_VALUE,         apply_param_calibration_value },
  { PARAM_VOICE_MODE,                apply_param_voice_mode },
  { PARAM_UNISON_DETUNE,             apply_param_unison_detune },
  { PARAM_ANALOG_DRIFT_AMOUNT,       apply_param_analog_drift_amount },
  { PARAM_ANALOG_DRIFT_SPEED,        apply_param_analog_drift_speed },
  { PARAM_ANALOG_DRIFT_SPREAD,       apply_param_analog_drift_spread },
  { PARAM_SYNC_MODE,                 apply_param_sync_mode },
  { PARAM_LFO1_TO_DCO,               apply_param_lfo1_to_dco },
  { PARAM_LFO1_SPEED,                apply_param_lfo1_speed },
  { PARAM_LFO2_SPEED,                apply_param_lfo2_speed },
  { PARAM_LFO2_TO_PW,                apply_param_lfo2_to_pw },
  { PARAM_ADSR3_TO_PWM,              apply_param_adsr1_to_pwm },
  { PARAM_ADSR3_TO_DETUNE1,          apply_param_adsr1_to_detune1 },
  { PARAM_ADSR1_ATTACK_CURVE,        apply_param_adsr1_curve },
  { PARAM_ADSR1_DECAY_CURVE,         apply_param_adsr2_curve },
  { PARAM_PWM_POTS_CONTROL_MANUAL,   apply_param_pwm_pots_manual },
  { PARAM_FUNCTION_KEY,              apply_param_function_key },
  { PARAM_CALIBRATION_FLAG,          apply_param_calibration_flag },
  { PARAM_MANUAL_CALIBRATION_FLAG,   apply_param_manual_calibration_flag },
  { PARAM_MANUAL_CALIBRATION_STAGE,  apply_param_manual_calibration_stage },
  { PARAM_MANUAL_CALIBRATION_OFFSET, apply_param_manual_calibration_offset }
};

static const size_t paramTableSize =
  sizeof(paramTable) / sizeof(paramTable[0]);

// Public entry point: called from Serial/MIDI/UI code.
inline void update_parameters(byte paramNumber, int16_t paramValue) {
  param_router_apply<int16_t>(paramTable, paramTableSize,
                              static_cast<uint16_t>(paramNumber),
                              paramValue);
}


