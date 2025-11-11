#include <cstdint>
#ifndef __VOICES_H__
#define __VOICES_H__

#ifdef RUNNING_AVERAGE
#include "RunningAverage.h"
#endif

void init_voices();
void print_voice_task_timings();

volatile bool note_on_flag_flag[NUM_VOICES_TOTAL];

uint32_t portamentoTimer[NUM_VOICES_TOTAL];
uint32_t portamentoStartMillis[NUM_VOICES_TOTAL];
uint32_t portamentoStartMicros[NUM_VOICES_TOTAL];

bool portamento = true;
uint32_t portamento_time = 0;
float  freqPortaInterval[NUM_VOICES_TOTAL * 2];
float portamento_stop[NUM_VOICES_TOTAL * 2];
float portamento_start[NUM_VOICES_TOTAL * 2];
float portamento_cur_freq[NUM_VOICES_TOTAL * 2];

// Fixed-point (Q24) equivalents for portamento to reduce float ops in audio loop
// Per-microsecond step as Q24 (fits in 32-bit)
int32_t freqPortaInterval_q24[NUM_VOICES_TOTAL * 2];
int64_t portamento_stop_q24[NUM_VOICES_TOTAL * 2];
int64_t portamento_start_q24[NUM_VOICES_TOTAL * 2];
int64_t portamento_cur_freq_q24[NUM_VOICES_TOTAL * 2];
// Fast 32-bit portamento state in Q16 (Hz * 2^16) to avoid 64-bit muls in audio loop
int32_t portamento_start_q16[NUM_VOICES_TOTAL * 2];
int32_t portamento_stop_q16[NUM_VOICES_TOTAL * 2];
int32_t portamento_cur_freq_q16[NUM_VOICES_TOTAL * 2];
// per-microsecond step in Q16
int32_t freqPortaStep_q16[NUM_VOICES_TOTAL * 2];
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

#ifdef RUNNING_AVERAGE
// RunningAverage objects for timing measurements (2000 samples each)
extern RunningAverage ra_pitchbend;
extern RunningAverage ra_osc2_detune;
extern RunningAverage ra_portamento;
extern RunningAverage ra_adsr_modifier;
extern RunningAverage ra_unison_modifier;
extern RunningAverage ra_drift_modifier;
extern RunningAverage ra_modifiers_combination;
extern RunningAverage ra_freq_scaling;
extern RunningAverage ra_interpolate_pitch;
extern RunningAverage ra_get_chan_level;
extern RunningAverage ra_pwm_calculations;
extern RunningAverage ra_voice_task_total;
extern RunningAverage ra_clk_div_calc;
#endif

#endif