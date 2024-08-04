#include <cstdint>
#ifndef __VOICES_H__
#define __VOICES_H__

void init_voices();

bool note_on_flag_flag[NUM_VOICES_TOTAL];

uint32_t portamentoTimer[NUM_VOICES_TOTAL];
uint32_t portamentoStartMillis[NUM_VOICES_TOTAL];
uint32_t portamentoStartMicros[NUM_VOICES_TOTAL];

bool portamento = true;
uint32_t portamento_time = 0;
float  freqPortaInterval[NUM_VOICES_TOTAL * 2];
float portamento_stop[NUM_VOICES_TOTAL * 2];
float portamento_start[NUM_VOICES_TOTAL * 2];
float portamento_cur_freq[NUM_VOICES_TOTAL * 2];

uint8_t highestNote = 124;

bool sqr1Status;

static const int multiplierTableSize = 200;
const int32_t multiplierTableScale = 10000;

int32_t xMultiplierTable[multiplierTableSize];
int32_t yMultiplierTable[multiplierTableSize];

static const uint16_t maxFrequency = 4000;

// uint8_t note1[NUM_VOICES];
// uint8_t note2[NUM_VOICES];

// volatile register float freq ;
// volatile register float freq2;

// float freq[NUM_VOICES];
// float freq2[NUM_VOICES];

// float ADSRModifier[NUM_VOICES];
// float ADSRModifierOSC1[NUM_VOICES];
// float ADSRModifierOSC2[NUM_VOICES];
// float unisonMODIFIER[NUM_VOICES];

// uint8_t pioNumber;
// uint8_t sm1N;
// uint8_t sm2N;

// uint32_t clk_div1[NUM_VOICES];
// uint32_t clk_div2[NUM_VOICES];

// uint_fast32_t clockdivFraction[NUM_VOICES];
// uint_fast32_t clockdiv2Fraction[NUM_VOICES];

// uint16_t chanLevel[NUM_VOICES];
// uint16_t chanLevel2[NUM_VOICES];

#endif