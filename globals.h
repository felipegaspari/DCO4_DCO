#include <cstddef>
#ifndef __GLOBALS_H__
#define __GLOBALS_H__

//#include "include_all.h"

#define NUM_VOICES_TOTAL 4
#define NUM_OSCILLATORS NUM_VOICES_TOTAL * 2

#define MIDI_CHANNEL 1
//#define USE_ADC_STACK_VOICES // gpio 28 (adc 2)
//#define USE_ADC_DETUNE       // gpio 27 (adc 1)

#define SCK 2
#define MOSI 3
#define MISO 4
#define CS 5

#define ENABLE_FS_CALIBRATION

static constexpr uint32_t sysClock = 225000;
static constexpr uint32_t sysClock_Hz = sysClock * 1000;

static constexpr uint16_t DIV_COUNTER = 14000;
static constexpr uint16_t DIV_COUNTER_PW = 1024;

static constexpr uint32_t pioPulseLength = 4000;
static constexpr uint32_t pioPulseLengthTimesEight = pioPulseLength * 8;
static constexpr uint32_t eightPioPulseLength = pioPulseLength / 8;
static constexpr uint32_t correctionPioPulseLength = 7;

static constexpr uint32_t halfSysClock_Hz = sysClock_Hz / 2;
static constexpr uint32_t eightSysClock_Hz_u = sysClock_Hz / 8;
static constexpr uint32_t eightSysClockMinusPulseLength_Hz_u = (sysClock_Hz - pioPulseLength - 8) / 8;
// Q24-scaled clock constants to avoid per-loop shifts
// (removed) Q24-scaled clock constants; direct shift used at call-site


uint32_t loop0_micros;
uint32_t loop1_micros;
uint32_t loop0_microsLast;
uint32_t loop1_microsLast;

volatile uint8_t NUM_VOICES = NUM_VOICES_TOTAL;
volatile uint8_t STACK_VOICES = 1;

volatile uint8_t voiceMode = 1;
uint8_t syncMode = 0;
uint8_t oscSync = 0;
volatile uint8_t polyMode = 1;

uint16_t phaseAlignOSC2 = 0;
// (removed) phaseAlignScale_Q16; use direct computation at call-site

uint8_t unisonDetune = 10;
uint8_t analogDrift = 0;
uint8_t analogDriftSpeed = 0;
uint8_t analogDriftSpread = 0;

float DETUNE = 0.0f, LAST_DETUNE = 0.0f;
float DETUNE2 = 1.00f;
float DETUNE_INTERNAL = 1;
volatile float DETUNE_INTERNAL2 = 1;
uint32_t DETUNE_INTERNAL_FIFO = 1;
float DETUNE_INTERNAL_FIFO_float = 1;
uint32_t* detune_fifo_variable = &DETUNE_INTERNAL_FIFO;
int32_t DETUNE_INTERNAL_FIFO_q24 = (1 << 24);
float BASE_NOTE = 440.0f;


// WEACT RP2040:
static constexpr uint8_t RESET_PINS[NUM_VOICES_TOTAL * 2] = { 29, 27, 19, 18, 15, 13, 12, 8 };
static constexpr uint8_t RANGE_PINS[NUM_VOICES_TOTAL * 2] = { 28, 22, 17, 16, 14, 11, 9, 7 };

// Raspberry Pi Pico:
// static constexpr uint8_t RESET_PINS[NUM_VOICES_TOTAL * 2] = { 28, 26, 19, 18, 15, 13, 12, 8 };
// static constexpr uint8_t RANGE_PINS[NUM_VOICES_TOTAL * 2] = { 27, 22, 17, 16, 14, 11,  9,  7 };

static constexpr uint8_t VOICE_TO_PIO[NUM_VOICES_TOTAL * 2] = { 0, 0, 0, 0, 1, 1, 1, 1 };
static constexpr uint8_t VOICE_TO_SM[NUM_VOICES_TOTAL * 2] = { 0, 1, 2, 3, 0, 1, 2, 3 };

static constexpr uint8_t PW_PINS[NUM_VOICES_TOTAL] = { 3, 2, 4, 5 };

static constexpr int DCO_calibration_pin = 10;

// constexpr uint8_t RESET_PINS[NUM_VOICES_TOTAL * 2] = { 29, 27, 19, 17, /*15, 13, 10, 8*/ };
// constexpr uint8_t RANGE_PINS[NUM_VOICES_TOTAL * 2] = { 28, 22, 18, 16/*, 14, 11, 9, 7*/ };
// constexpr uint8_t VOICE_TO_PIO[NUM_VOICES_TOTAL * 2] = { 1, 1, 1, 1/*, 0, 0, 0, 0*/ };
// constexpr uint8_t VOICE_TO_SM[NUM_VOICES_TOTAL * 2] = { 0, 1, 2, 3/*, 0, 1, 2, 3*/ };

// constexpr uint8_t RESET_PINS[NUM_VOICES_TOTAL * 2] = { /*29, 27, 19, 17, */15, 13, 10, 8 };
// constexpr uint8_t RANGE_PINS[NUM_VOICES_TOTAL * 2] = { /*28, 22, 18, 16,*/ 14, 11, 9, 7 };
// constexpr uint8_t VOICE_TO_PIO[NUM_VOICES_TOTAL * 2] = { /*1, 1, 1, 1,*/ 0, 0, 0, 0 };
// constexpr uint8_t VOICE_TO_SM[NUM_VOICES_TOTAL * 2] = { /*0, 1, 2, 3,*/ 0, 1, 2, 3 };

// constexpr uint8_t RESET_PINS[NUM_VOICES_TOTAL * 2] = { 29, 27, 19, 17, /* 15, 13,*/ 10, 8 };
// constexpr uint8_t RANGE_PINS[NUM_VOICES_TOTAL * 2] = { 28, 22, 18, 16, /* 14, 11,*/ 9, 7 };
// constexpr uint8_t VOICE_TO_PIO[NUM_VOICES_TOTAL * 2] = { 1, 1, 1, 1, 0, 0 /*, 0, 0 */ };
// constexpr uint8_t VOICE_TO_SM[NUM_VOICES_TOTAL * 2] = { 0, 1, 2, 3, 0, 1 /*, 2, 3 */ };

uint8_t RANGE_PWM_SLICES[NUM_VOICES_TOTAL * 2];
uint8_t VCO_PWM_SLICES[NUM_VOICES_TOTAL * 2];
uint8_t PW_PWM_SLICES[NUM_VOICES_TOTAL];

uint16_t PW_CENTER[NUM_VOICES_TOTAL] = { 570, 552, 540, 553 };
uint16_t PW_LOW_LIMIT[NUM_VOICES_TOTAL] = {0,0,0,0};
uint16_t PW_HIGH_LIMIT[NUM_VOICES_TOTAL] = { DIV_COUNTER_PW, DIV_COUNTER_PW, DIV_COUNTER_PW, DIV_COUNTER_PW };
uint16_t PW_LOOKUP[3] = { 0, (DIV_COUNTER_PW / 2) - 1, DIV_COUNTER_PW - 1 };
uint16_t PW_PWM[NUM_VOICES_TOTAL];

volatile uint32_t VOICES[NUM_VOICES_TOTAL];
volatile uint8_t VOICES_LAST[NUM_VOICES_TOTAL];
volatile uint8_t VOICES_LAST_SEQUENCE[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
volatile uint8_t VOICE_NOTES[NUM_VOICES_TOTAL];
volatile uint8_t NEXT_VOICE = 0;

uint32_t LED_BLINK_START = 0;

PIO pio[2] = { pio0, pio1 };

uint8_t midi_serial_status = 0;
int midi_pitch_bend = 8192, last_midi_pitch_bend = 8192;
uint8_t pitchBendRange = 2;
// Precompute 1/12 in Q24 for fast multiplier calculation
static constexpr int32_t RECIP_TWELVE_Q24 = (int32_t)((1.0f / 12.0f) * (float)(1 << 24));
// Precompute 1/180 in Q24 for fast phaseDelay calculation
static constexpr uint32_t RECIP_180_Q24 = (uint32_t)(((1ULL << 24) + 90) / 180);
float pitchBendMultiplier = 1.00f / 12.00f * (float)pitchBendRange;
int32_t pitchBendMultiplier_q24 = 1 << 24;

uint16_t raw;

void init_sm(PIO pio, uint sm, uint offset, uint pin);
void set_frequency(PIO pio, uint sm, float freq);
float get_freq_from_midi_note(uint8_t note);
void led_blinking_task();
uint8_t get_free_voice();
void usb_midi_task();
void serial_midi_task();
void note_on(uint8_t note, uint8_t velocity);
void note_off(uint8_t note);
void voice_task();
void adc_task();


uint32_t offset[2];
uint8_t dataArray[4];

float LFOMultiplier = 1;
float voiceFreq[8];
uint16_t dato_serial;
float dato_serial_float;
uint8_t OSC1_interval = 24;
uint8_t OSC2_serial_detune = 127;
uint8_t OSC2_interval = 36;
float OSC2_detune = 127;
uint16_t OSC2DetuneVal = 256;

bool PWMPotsControlManual;

uint16_t PW[4];

void serial_STM32_task();
void serial_send_voice_freq();
void serial_send_note_on(uint8_t voice_n, uint8_t note_velo);
void serial_send_note_off(uint8_t voice_n);
float get_chan_level(float freq_to_amp_comp);

volatile uint8_t note_on_flag[NUM_VOICES_TOTAL * 2];

bool ledstat = false;

#endif
