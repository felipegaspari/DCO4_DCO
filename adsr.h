#ifndef __ADSR_H__
#define __ADSR_H__

#include <adsr.h>

#define ADSR_1_DACSIZE 4096

byte noteStart[NUM_VOICES_TOTAL];
byte noteEnd[NUM_VOICES_TOTAL];

uint16_t ADSR1Level[NUM_VOICES_TOTAL];

static const uint16_t ADSR_1_CC = 4096;

float ADSRMaxLevel = ADSR_1_CC;

uint16_t ADSRMinLevel = 0;

int8_t ADSR3ToOscSelect = 2;

uint16_t ADSR1_attack = 0;
uint16_t ADSR1_decay;
uint16_t ADSR1_sustain;
uint16_t ADSR1_release;

byte ADSR1_curve2Val = 0;

float ADSR1_curve1 = 0.999;
float ADSR1_curve2 = 0.997;

unsigned long tADSR;
unsigned long tADSR_params;

bool ADSRRestart = true;

int16_t ADSR1toDETUNE1;

float ADSR1toDETUNE1_formula;

int16_t ADSR1toPWM;
float ADSR1toPWM_formula;


adsr adsr1_voice_0(ADSR_1_CC);
adsr adsr1_voice_1(ADSR_1_CC);
adsr adsr1_voice_2(ADSR_1_CC);
adsr adsr1_voice_3(ADSR_1_CC);

//bool OSCPhaseLock = false;

struct ADSRStruct {
adsr adsr1_voice;
};

ADSRStruct ADSRVoices[] = {
{adsr1_voice_0},
{adsr1_voice_1},
{adsr1_voice_2},
{adsr1_voice_3},
// {adsr1_voice_4},
// {adsr1_voice_5},
};

#endif