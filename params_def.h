#ifndef PARAMS_DEF_H
#define PARAMS_DEF_H

#include <stdint.h>

// Central definition of all parameter IDs used across MCUs.
// This DCO header mirrors the mainboard's params_def.h numeric values
// so that parameters sent over Serial are interpreted consistently.
//
// IMPORTANT:
//   - Do not change numeric values of existing IDs.
//   - New parameters should get new, unused numbers.
//   - The meaning of each ID (name + number) should be stable across MCUs.

enum ParamId : uint16_t {
  // --- Oscillator wave enable (mainboard-local, not used on DCO) ------
  PARAM_SAW_STATUS               = 1,
  PARAM_SAW2_STATUS              = 2,
  PARAM_TRI_STATUS               = 3,
  PARAM_SINE_STATUS              = 4,

  // Shared with DCO: 5, 10.. etc
  PARAM_SQR1_STATUS              = 5,
  PARAM_SQR2_STATUS              = 6,   // mainboard-local

  PARAM_RESONANCE_COMPENSATION   = 7,   // mainboard-local
  PARAM_VCA_ADSR_RESTART         = 8,   // mainboard-local
  PARAM_VCF_ADSR_RESTART         = 9,   // mainboard-local

  // --- Shared routing / oscillator parameters -------------------------
  PARAM_ADSR3_TO_OSC_SELECT      = 10,

  PARAM_LFO1_WAVEFORM            = 11,
  PARAM_LFO2_WAVEFORM            = 12,

  PARAM_OSC1_INTERVAL            = 13,
  PARAM_OSC2_INTERVAL            = 14,

  PARAM_OSC2_DETUNE_VAL          = 15,
  PARAM_LFO2_TO_DETUNE2          = 16,

  PARAM_OSC_SYNC_MODE            = 17,

  PARAM_PORTAMENTO_TIME          = 18,

  // --- Mainboard-local filter/velocity routing ------------------------
  PARAM_VCF_KEYTRACK             = 19,
  PARAM_VELOCITY_TO_VCF          = 20,
  PARAM_VELOCITY_TO_VCA          = 21,
  PARAM_SQR1_LEVEL               = 22,
  PARAM_SQR2_LEVEL               = 23,
  PARAM_SUB_LEVEL                = 24,

  // --- Shared calibration / voice mode --------------------------------
  PARAM_CALIBRATION_VALUE        = 25,

  PARAM_VOICE_MODE               = 26,
  PARAM_UNISON_DETUNE            = 27,

  PARAM_ANALOG_DRIFT_AMOUNT      = 28,
  PARAM_ANALOG_DRIFT_SPEED       = 29,
  PARAM_ANALOG_DRIFT_SPREAD      = 30,

  PARAM_SYNC_MODE                = 31,

  // 32: DCO-only portamento mode selector (currently local to DCO)
  PARAM_PORTAMENTO_MODE          = 32,

  // --- LFO routing (shared) -------------------------------------------
  PARAM_LFO1_TO_DCO              = 40,
  PARAM_LFO1_SPEED               = 41,
  PARAM_LFO2_SPEED               = 42,

  // --- Mainboard-only VCA routing ------------------------------------
  PARAM_VCA_LEVEL                = 43,
  PARAM_LFO1_TO_VCA              = 44,

  // --- PWM / ADSR to PWM / detune (shared with DCO) -------------------
  PARAM_LFO2_TO_PW               = 45,
  PARAM_ADSR3_TO_PWM             = 46,
  PARAM_ADSR3_TO_DETUNE1         = 47,

  // ADSR curve shaping (mainboard-local only)
  PARAM_ADSR1_ATTACK_CURVE       = 48,
  PARAM_ADSR1_DECAY_CURVE        = 49,
  PARAM_ADSR2_ATTACK_CURVE       = 50,
  PARAM_ADSR2_DECAY_CURVE        = 51,

  // --- Misc / control / UI flags ------------------------------------
  // Calibration mode selector (screen/UI only for now)
  PARAM_CALIBRATION_MODE         = 101,

  // Global/manual control flags (input+screen; DCO may ignore)
  PARAM_FADERS_CONTROL_MANUAL    = 120,
  PARAM_FADER_ROW1_CONTROL_MANUAL= 121,
  PARAM_FADER_ROW2_CONTROL_MANUAL= 122,
  PARAM_VCF_POTS_CONTROL_MANUAL  = 123,
  PARAM_PWM_POTS_CONTROL_MANUAL  = 124,
  PARAM_ALL_CONTROLS_MANUAL      = 125,

  PARAM_ADSR3_ENABLED            = 126,
  PARAM_FUNCTION_KEY             = 127,

  PARAM_VCA_POTS_CONTROL_MANUAL  = 128,
  PARAM_POTS_CONTROL_MANUAL      = 129,

  // UI navigation / calibration helper parameters (screen-focused)
  PARAM_UI_MENU_POSITION         = 190,
  PARAM_UI_CALIBRATION_DISMISS   = 199,
  PARAM_UI_CALIBRATION_MENU_MODE = 200,

  // Reserved / screen-only extras (future expansion)
  PARAM_PW_VALUE                 = 210,
  PARAM_LFO3_SPEED               = 211,
  PARAM_LFO3_WAVEFORM            = 212,
  PARAM_ADSR3_RESTART            = 214,
  PARAM_VCA_LEVEL_ALT            = 215,

  // --- Calibration flags (shared) ------------------------------------
  PARAM_CALIBRATION_FLAG         = 150,
  PARAM_MANUAL_CALIBRATION_FLAG  = 151,
  PARAM_MANUAL_CALIBRATION_STAGE = 152,
  PARAM_MANUAL_CALIBRATION_OFFSET= 153,

  // 154: mainboard-only gap-from-DCO parameter (screen reporting)
  PARAM_GAP_FROM_DCO             = 154
};

#endif  // PARAMS_DEF_H



