#include <cstdint>
#ifndef __VOICES_H__
#define __VOICES_H__

void init_voices();
void print_voice_task_timings();

volatile bool note_on_flag_flag[NUM_VOICES_TOTAL];

uint32_t portamentoTimer[NUM_VOICES_TOTAL];
uint32_t portamentoStartMillis[NUM_VOICES_TOTAL];
uint32_t portamentoStartMicros[NUM_VOICES_TOTAL];

bool portamento = true;
uint32_t portamento_time = 0;

// Portamento mode: 0 = fixed-time glide (current behaviour),
//                  1 = analog-style slew-rate (time scales with interval).
enum PortamentoMode : uint8_t {
  PORTA_MODE_TIME = 0,
  PORTA_MODE_SLEW = 1
};
uint8_t portamento_mode = PORTA_MODE_SLEW;

// Portamento state in Q24 (Hz * 2^24)
int64_t portamento_start_q24[NUM_VOICES_TOTAL * 2];
int64_t portamento_stop_q24[NUM_VOICES_TOTAL * 2];
int64_t portamento_cur_freq_q24[NUM_VOICES_TOTAL * 2];
// per-microsecond step in Q24
int64_t freqPortaStep_q24[NUM_VOICES_TOTAL * 2];

// Portamento state in note-space (Q16 semitones) for slew-rate mode
int32_t porta_note_start_q16[NUM_VOICES_TOTAL * 2];
int32_t porta_note_stop_q16[NUM_VOICES_TOTAL * 2];
int32_t porta_note_cur_q16[NUM_VOICES_TOTAL * 2];
int32_t porta_note_step_q16[NUM_VOICES_TOTAL * 2];

// Float portamento state for the float engine (Hz and semitone domains)
#ifdef USE_FLOAT_VOICE_TASK
// Time-based mode: linear in frequency (Hz)
float porta_freq_start_f[NUM_VOICES_TOTAL * 2];
float porta_freq_stop_f [NUM_VOICES_TOTAL * 2];
float porta_freq_step_f [NUM_VOICES_TOTAL * 2];  // Hz per microsecond
float porta_freq_cur_f  [NUM_VOICES_TOTAL * 2];

// Slew-rate mode: linear in note-space (semitones)
float porta_note_start_f[NUM_VOICES_TOTAL * 2];
float porta_note_stop_f [NUM_VOICES_TOTAL * 2];
float porta_note_cur_f  [NUM_VOICES_TOTAL * 2];
float porta_note_step_f [NUM_VOICES_TOTAL * 2];  // semitones per microsecond
#endif

uint8_t highestNote = 124;

bool sqr1Status;

static const int multiplierTableSize = 200;
const int32_t multiplierTableScale = 10000;

int32_t xMultiplierTable[multiplierTableSize];
int32_t yMultiplierTable[multiplierTableSize];

// Float mirrors of multiplier tables for the RP2350 float path
float   xMultiplierTableF[multiplierTableSize];
float   yMultiplierTableF[multiplierTableSize];

// Precomputed left edge of each segment in Q16 to avoid shifts at runtime
int32_t x0Q16_tbl[multiplierTableSize];
// Precomputed per-segment slopes in Q20: slopeQ20[i] ≈ ((y[i+1]-y[i]) << 20) / (x[i+1]-x[i])
int32_t slopeQ20[multiplierTableSize - 1];
// Float slopes (dy/dx) for the float interpolation path
float   slopeF[multiplierTableSize - 1];
#ifdef PITCH_INTERP_USE_Q8
// Optional lower-precision slope in Q8 for 32-bit fast path
int32_t slopeQ8[multiplierTableSize - 1];
#endif
#ifdef PITCH_INTERP_USE_Q12
// Medium-precision slope in Q12 for balanced speed/accuracy
int32_t slopeQ12[multiplierTableSize - 1];
#endif
// Per-DCO segment cache for interpolation (stores last 'low' index)
int16_t interpSegCache[NUM_VOICES_TOTAL * 2];

static const uint16_t maxFrequency = 4000;

// Q24 frequency constants
static constexpr int32_t Q24_ONE = (1 << 24);
static constexpr int32_t Q24_EPS_DELTA_1P00001 = 168; // round(0.00001 * 2^24)
static constexpr int32_t Q24_ONE_EPS = Q24_ONE + Q24_EPS_DELTA_1P00001;


#ifdef RUNNING_AVERAGE
// RunningAverage objects for timing measurements (2000 samples each)
extern RunningAverage ra_pitchbend;
extern RunningAverage ra_osc2_detune;
extern RunningAverage ra_portamento;
extern RunningAverage ra_porta_core;
extern RunningAverage ra_adsr_modifier;
extern RunningAverage ra_unison_modifier;
extern RunningAverage ra_drift_modifier;
extern RunningAverage ra_modifiers_combination;
extern RunningAverage ra_freq_scaling_x;
extern RunningAverage ra_freq_scaling_ratio;
extern RunningAverage ra_freq_scaling_post;
extern RunningAverage ra_interpolate_pitch;
extern RunningAverage ra_get_chan_level;
extern RunningAverage ra_pwm_calculations;
extern RunningAverage ra_voice_task_total;
extern RunningAverage ra_clk_div_calc;
#endif

#endif