#ifndef __LFO_H__
#define __LFO_H__

#include "src/lfo-main/lfo.h"  // required for function generation

//static constexpr uint16_t PWM_CC = 4096;
static constexpr uint16_t LFO1_CC = 4000;
static constexpr uint16_t LFO1_CC_HALF = LFO1_CC / 2;
static constexpr uint16_t LF01_CC_THIRD = LFO1_CC / 3;
static constexpr uint16_t LFO2_CC = 1024;
static constexpr uint16_t LFO2_CC_HALF = LFO2_CC / 2;

static constexpr uint16_t LFO_DRIFT_CC = 1000;
static constexpr uint16_t LFO_DRIFT_CC_HALF = LFO_DRIFT_CC / 2;

//////////////// LFO ian ////////////////////////////////////////

lfo LFO1_class(LFO1_CC + 1);
lfo LFO2_class(LFO2_CC);

lfo LFO_DRIFT_CLASS[8] = {
  lfo(LFO_DRIFT_CC),
  lfo(LFO_DRIFT_CC),
  lfo(LFO_DRIFT_CC),
  lfo(LFO_DRIFT_CC),
  lfo(LFO_DRIFT_CC),
  lfo(LFO_DRIFT_CC),
  lfo(LFO_DRIFT_CC),
  lfo(LFO_DRIFT_CC)
};


/////////////////////////////////////////////////////////////////
byte LFO_DRIFT_WAVEFORM = 2;
float LFO_DRIFT_SPEED_OFFSET[8];
float LFO_DRIFT_SPEED = 0.6;
volatile int16_t LFO_DRIFT_LEVEL[8];


int16_t LFO1Level;
byte LFO1Waveform = 3;
float LFO1Speed = 50;
float LFO1toDCO = 0;
// Q24 fixed-point version of LFO1->DCO modulation depth, kept in sync with LFO1toDCO.
int32_t LFO1toDCO_q24 = 0;
int16_t LFO1toDETUNE1;
uint16_t LFO1toDETUNE2;

volatile int16_t LFO2Level;
byte LFO2Waveform;
float LFO2Speed;
float LFO2toDCO;
uint16_t LFO2toDETUNE1;
// LFO2->detune depth as float plus a Q24 fixed-point version for OSC2 modulation.
float LFO2toDETUNE2 = 0.0f;
int32_t LFO2toDETUNE2_q24 = 0;
volatile uint16_t LFO2toOSC2DETUNE;
volatile uint16_t LFO2toPW;

uint16_t LFO1SpeedVal;
uint16_t LFO2SpeedVal;
uint16_t LFO1toDCOVal;
uint16_t LFO2toVCFVal;

volatile float LFO2toPWM_formula;
volatile int32_t LFO2toPWM_formula_q24;

void LFO1();
void LFO2();


#endif
