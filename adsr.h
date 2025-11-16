#ifndef __ADSR_H__
#define __ADSR_H__

#define ADSR_1_DACSIZE 4000

#define ARRAY_SIZE 512

#define LIN_TO_EXP_TABLE_SIZE ADSR_1_DACSIZE + 1
uint16_t linToExpLookup[LIN_TO_EXP_TABLE_SIZE];
uint16_t linToLogLookup[LIN_TO_EXP_TABLE_SIZE];
uint16_t maxADSRControlValue = ADSR_1_DACSIZE;

// ADSR Bezier library (provides curve tables and ADSR class)
#include <ADSR_Bezier.h>

volatile byte noteStart[NUM_VOICES_TOTAL];
volatile byte noteEnd[NUM_VOICES_TOTAL];

uint16_t ADSR1Level[NUM_VOICES_TOTAL];

static constexpr uint16_t ADSR_1_CC = 4000;

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
int32_t ADSR1toDETUNE1_scale_q24;

int16_t ADSR1toPWM;
float ADSR1toPWM_formula;
int32_t ADSR1toPWM_formula_q24;

adsr adsr1_voice_0(ADSR_1_CC, ADSR1_curve1, ADSR1_curve2, false,7,7,7);
adsr adsr1_voice_1(ADSR_1_CC, ADSR1_curve1, ADSR1_curve2, false,7,7,7);
adsr adsr1_voice_2(ADSR_1_CC, ADSR1_curve1, ADSR1_curve2, false,7,7,7);
adsr adsr1_voice_3(ADSR_1_CC, ADSR1_curve1, ADSR1_curve2, false,7,7,7);

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