#ifndef __LFO_H__
#define __LFO_H__

#include <lfo.h>  // required for function generation

//static const uint16_t PWM_CC = 4096;
static const uint16_t LFO1_CC = 1000;
static const uint16_t LFO1_CC_HALF = LFO1_CC / 2;
static const uint16_t LF01_CC_THIRD = LFO1_CC / 3;
static const uint16_t LFO2_CC = 2000;
static const uint16_t LFO2_CC_HALF = LFO2_CC / 2;

//////////////// LFO ian ////////////////////////////////////////

lfo LFO1_class(LFO1_CC + 1);
//lfo             LFO2_class(LFO2_CC + 1);

/////////////////////////////////////////////////////////////////

int16_t LFO1Level;
byte LFO1Waveform;
float LFO1Speed;
float LFO1toDCO = 0;
int16_t LFO1toDETUNE1;
uint16_t LFO1toDETUNE2;

int16_t LFO2Level;
byte LFO2Waveform;
float LFO2Speed;
float LFO2toDCO;
uint16_t LFO2toDETUNE1;
uint16_t LFO2toDETUNE2;
uint16_t LFO2toOSC2DETUNE;

uint16_t LFO1SpeedVal;
uint16_t LFO2SpeedVal;
uint16_t LFO1toDCOVal;
uint16_t LFO2toVCFVal;



void LFO1();
void LFO2();


#endif
