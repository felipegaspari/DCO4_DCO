#ifndef __AUTOTUNE_H__
#define __AUTOTUNE_H__

#include "include_all.h"
#include "autotune_constants.h"
#include "autotune_measurement.h"
#include "autotune_context.h"

// Global flags controlling calibration routines.
//  - calibrationFlag: a calibration process is currently running.
//  - manualCalibrationFlag: manual calibration mode is active.
//  - firstTuneFlag: true on the very first calibration run after boot/flash.
bool calibrationFlag = false;
bool manualCalibrationFlag = false;
bool firstTuneFlag = false;

// Manual DCO calibration workflow state and per-oscillator manual offsets
// that are added on top of automatic amp compensation.
uint8_t manualCalibrationStage;
int8_t manualCalibrationOffset[NUM_OSCILLATORS] = {0,0,0,0,0,0,0,0};
/************************************************/
/****************** DCO calibration ******************/

// Temporary buffer used during calibration to build [frequency, range-PWM]
// pairs for a single DCO. Persisted via update_FS_voice() when an osc is done.
uint32_t calibrationData[chanLevelVoiceDataSize];

// Index of the DCO currently being calibrated.
uint8_t currentDCO;

// Timing helpers for edge measurement and per-note calibration.
unsigned long edgeDetectionLastTime;
unsigned long microsNow;
unsigned long currentNoteCalibrationStart;
unsigned long DCOCalibrationStart;

// Last digital level read on the calibration pin (for edge detection).
bool edgeDetectionLastVal = 0;

// Current range-PWM value used during calibration for the active DCO.
volatile uint16_t ampCompCalibrationVal;



// Baseline manual amp-comp starting value for all oscillators.
int8_t initManualAmpCompCalibrationValPreset = 35;
// weact rp2040 dco //volatile int8_t initManualAmpCompCalibrationVal[NUM_OSCILLATORS] = {24,26,25,25,25,18,20,25};
// Per-oscillator baseline manual amp-comp starting values.
int8_t initManualAmpCompCalibrationVal[NUM_OSCILLATORS] = {initManualAmpCompCalibrationValPreset,initManualAmpCompCalibrationValPreset,
initManualAmpCompCalibrationValPreset,initManualAmpCompCalibrationValPreset,initManualAmpCompCalibrationValPreset,initManualAmpCompCalibrationValPreset,

initManualAmpCompCalibrationValPreset,initManualAmpCompCalibrationValPreset};
// Range-PWM value used when probing for the lowest frequency during calibration.
volatile uint16_t ampCompLowestFreqVal = 10;

// Edge counting and sample accumulation for duty/frequency measurement.
int pulseCounter = 0;
int samplesCounter = 0;

// Accumulated time spent in high/low states during measurement windows.
double risingEdgeTimeSum, fallingEdgeTimeSum;

// Main error metric used by most calibration routines:
//  - For duty calibration: (lowTime - highTime) / scaling factor.
//  - For frequency measurement: (targetFreq - measuredFreq).
float DCO_calibration_difference;

// Number of edge samples to accumulate in the current measurement.
uint16_t samplesNumber;

// Note from which DCO calibration starts (MIDI note index).
static constexpr uint8_t DCO_calibration_start_note = 29; // 29 == C0
// Interval in semitones between successive calibration notes.
static constexpr uint8_t calibration_note_interval = 5;
// Starting note used for manual/PW-centered calibration passes.
static constexpr uint8_t manual_DCO_calibration_start_note = DCO_calibration_start_note - 5;

// Current note/voice/oscillator indexes used during calibration.
uint8_t DCO_calibration_current_note;
uint8_t DCO_calibration_current_voice;
uint8_t DCO_calibration_current_OSC;

// Highest reachable note per oscillator (found by highest-frequency search).
uint8_t highestNoteOSC[NUM_OSCILLATORS];

// History used to detect sign flips and convergence behaviour.
double lastDCODifference;
uint8_t lastGapFlipCount;
double lastPIDgap;
uint16_t lastampCompCalibrationVal;

// Current PW value used during PW center/limit calibration.
uint16_t PWCalibrationVal;

// Global debug verbosity level for autotune routines.
byte autotuneDebug = 4;

// Direction selector for unified PW limit search.
enum PWLimitDir {
    PW_LIMIT_LOW,
    PW_LIMIT_HIGH
  };
  

/*********************** VCO calibration  ********************/
/**********************                    *******************/

// float pwm_to_vco_array[] = {
//   1,
//   50,
//   100,
//   200,
//   400,
//   600,
//   800,
//   1000,
//   1200,
//   1400,
//   1600,
//   1800,
//   2000,
//   2200,
//   2400,
//   2600,
//   2800,
//   3000,
//   3200,
//   3400,
//   3600,
//   3800,
//   4000,
//   4200,
//   4400,
//   4600,
//   4800,
//   5000,
//   5200,
//   5400,
//   5600,
//   5800,
//   6000,
//   6200,
//   6400,
//   6600,
//   6800,
//   7000,
//   7200,
//   7400,
//   7600,
//   7800,
//   8000,
//   8200,
//   8400,
//   8500,
//   8600,
//   8700,
//   8800,
//   8900,
//   9000,
//   9100,
//   9200,
//   9300,
//   9400,
//   9500,
//   9600,
//   9700,
//   9800,
//   9999,
//   9999
// };

// const uint16_t pwm_to_vco_array_size = (sizeof(pwm_to_vco_array) / sizeof(float)) - 1;

// float pwm_to_vco_euler_array[pwm_to_vco_array_size + 1];

// float freq_to_vco_array[pwm_to_vco_array_size + 1];

// uint8_t freq_to_vco_array_memcopy[(pwm_to_vco_array_size + 1) * 4];

#endif
