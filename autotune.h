#ifndef __AUTOTUNE_H__
#define __AUTOTUNE_H__

bool autotuneOnFlag;
bool manualTuneOnFlag;
bool firstTuneFlag;

/************************************************/
/****************** DCO calibration ******************/



uint32_t calibrationData[chanLevelVoiceDataSize];

uint8_t currentDCO;

unsigned long DCO_calibration_lastTime;
unsigned long microsNow;
unsigned long currentNoteCalibrationStart;
unsigned long DCOCalibrationStart;

bool DCO_calibration_lastVal = 0;

volatile uint16_t ampCompCalibrationVal;
volatile uint16_t initManualAmpCompCalibrationVal = 60;


int pulseCounter = 0;
float DCO_calibration_avg1, DCO_calibration_avg2;
double DCO_calibration_difference;

uint8_t DCO_calibration_avg1_counter, DCO_calibration_avg2_counter;

uint16_t samplesNumber;

const uint8_t DCO_calibration_start_note = 29;
const uint8_t manual_DCO_calibration_start_note = DCO_calibration_start_note - 5;

uint8_t DCO_calibration_current_note;
uint8_t DCO_calibration_current_voice;
uint8_t DCO_calibration_current_OSC;

uint8_t highestNoteOSC[NUM_OSCILLATORS];

int16_t lastDCODifference;
uint8_t lastGapFlipCount;
uint16_t lastPIDgap;
uint16_t lastampCompCalibrationVal;

uint16_t PWCalibrationVal;

byte autotuneDebug = 4;



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
