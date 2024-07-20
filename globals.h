#include <cstddef>
#ifndef __GLOBALS_H__
#define __GLOBALS_H__

#define NUM_VOICES_TOTAL 4
#define NUM_OSCILLATORS NUM_VOICES_TOTAL * 2
#define MIDI_CHANNEL 1
//#define USE_ADC_STACK_VOICES // gpio 28 (adc 2)
//#define USE_ADC_DETUNE       // gpio 27 (adc 1)

#define SCK 2
#define MOSI 3
#define MISO 4
#define CS 5

static const uint32_t sysClock = 225000;
static const uint32_t sysClock_Hz = sysClock * 1000;

static const uint16_t DIV_COUNTER = 10000;
static const uint16_t DIV_COUNTER_PW = 1024;

static const uint32_t pioPulseLength = 2800;
static const uint32_t pioPulseLengthTimesEight = pioPulseLength * 8;
static const uint32_t eightPioPulseLength = pioPulseLength / 8;
static const uint32_t correctionPioPulseLength = (pioPulseLength / 8) - (pioPulseLength / 500);

static const float halfSysClock_Hz = sysClock_Hz / 2;
static const float eightSysClock_Hz = sysClock_Hz / 8;
static const float eightSysClockMinusPulseLength_Hz = (double)(sysClock_Hz - pioPulseLength - 8) / (double)8;


uint32_t loop0_micros;
uint32_t loop1_micros;
uint32_t loop0_microsLast;
uint32_t loop1_microsLast;

uint8_t NUM_VOICES = NUM_VOICES_TOTAL;
uint8_t STACK_VOICES = 1;

uint8_t voiceMode = 2;
uint8_t oscSync = 0;
uint8_t polyMode = 1;

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
uint32_t* a = &DETUNE_INTERNAL_FIFO;
const float BASE_NOTE = 440.0f;

const uint8_t RESET_PINS[NUM_VOICES_TOTAL * 2] = { 29, 27, 19, 18, 15, 13, 12, 8 };
const uint8_t RANGE_PINS[NUM_VOICES_TOTAL * 2] = { 28, 22, 17, 16, 14, 11, 9, 7 };
const uint8_t VOICE_TO_PIO[NUM_VOICES_TOTAL * 2] = { 1, 1, 1, 1, 0, 0, 0, 0 };
const uint8_t VOICE_TO_SM[NUM_VOICES_TOTAL * 2] = { 0, 1, 2, 3, 0, 1, 2, 3 };

const uint8_t PW_PINS[NUM_VOICES_TOTAL] = { 3, 2, 4, 5 };

const int DCO_calibration_pin = 10;

// const uint8_t RESET_PINS[NUM_VOICES_TOTAL * 2] = { 29, 27, 19, 17, /*15, 13, 10, 8*/ };
// const uint8_t RANGE_PINS[NUM_VOICES_TOTAL * 2] = { 28, 22, 18, 16/*, 14, 11, 9, 7*/ };
// const uint8_t VOICE_TO_PIO[NUM_VOICES_TOTAL * 2] = { 1, 1, 1, 1/*, 0, 0, 0, 0*/ };
// const uint8_t VOICE_TO_SM[NUM_VOICES_TOTAL * 2] = { 0, 1, 2, 3/*, 0, 1, 2, 3*/ };

// const uint8_t RESET_PINS[NUM_VOICES_TOTAL * 2] = { /*29, 27, 19, 17, */15, 13, 10, 8 };
// const uint8_t RANGE_PINS[NUM_VOICES_TOTAL * 2] = { /*28, 22, 18, 16,*/ 14, 11, 9, 7 };
// const uint8_t VOICE_TO_PIO[NUM_VOICES_TOTAL * 2] = { /*1, 1, 1, 1,*/ 0, 0, 0, 0 };
// const uint8_t VOICE_TO_SM[NUM_VOICES_TOTAL * 2] = { /*0, 1, 2, 3,*/ 0, 1, 2, 3 };

// const uint8_t RESET_PINS[NUM_VOICES_TOTAL * 2] = { 29, 27, 19, 17, /* 15, 13,*/ 10, 8 };
// const uint8_t RANGE_PINS[NUM_VOICES_TOTAL * 2] = { 28, 22, 18, 16, /* 14, 11,*/ 9, 7 };
// const uint8_t VOICE_TO_PIO[NUM_VOICES_TOTAL * 2] = { 1, 1, 1, 1, 0, 0 /*, 0, 0 */ };
// const uint8_t VOICE_TO_SM[NUM_VOICES_TOTAL * 2] = { 0, 1, 2, 3, 0, 1 /*, 2, 3 */ };

uint8_t RANGE_PWM_SLICES[NUM_VOICES_TOTAL * 2];
uint8_t VCO_PWM_SLICES[NUM_VOICES_TOTAL * 2];
uint8_t PW_PWM_SLICES[NUM_VOICES_TOTAL];

uint16_t PW_CENTER[NUM_VOICES_TOTAL] = { 570, 552, 540, 553 };
uint16_t PW_LOW_LIMIT[NUM_VOICES_TOTAL];
uint16_t PW_HIGH_LIMIT[NUM_VOICES_TOTAL] = { DIV_COUNTER_PW, DIV_COUNTER_PW, DIV_COUNTER_PW, DIV_COUNTER_PW };
uint16_t PW_LOOKUP[3] = { 0, (DIV_COUNTER_PW / 2) -1, DIV_COUNTER_PW - 1 };
uint16_t PW_PWM[NUM_VOICES_TOTAL];

uint32_t VOICES[NUM_VOICES_TOTAL];
uint8_t VOICES_LAST[NUM_VOICES_TOTAL];
uint8_t VOICES_LAST_SEQUENCE[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
uint8_t VOICE_NOTES[NUM_VOICES_TOTAL];
uint8_t NEXT_VOICE = 0;

uint32_t LED_BLINK_START = 0;

PIO pio[2] = { pio0, pio1 };

uint8_t midi_serial_status = 0;
uint16_t midi_pitch_bend = 0x2000, last_midi_pitch_bend = 0x2000;

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
uint8_t OSC2_interval = 24;
float OSC2_detune = 127;
uint16_t OSC2DetuneVal;

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
