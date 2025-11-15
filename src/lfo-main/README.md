# MO-LFO – micros() based LFO for Arduino / RP2040 / Teensy / ESP32

Light‑weight, sample‑rate–agnostic LFO class based on `micros()`.  
Works well on AVR, ARM (e.g. RP2040 / Pico), Teensy, ESP32, etc.

The LFO can be **free‑running** or **synced to a master clock (BPM‑based)** and supports:

- Saw, triangle, sine (lookup‑table based), square and DC (off)
- Arbitrary DAC / control resolution (you choose the range with the constructor)
- Time‑based phase progression using a 32‑bit phase accumulator

Original project and video by **mo‑thunderz**.  
Video walkthrough: `https://youtu.be/ch03-75Fkuw`

---

## 1. Installation

1. Create a folder called `lfo` in your Arduino libraries folder.
2. Copy `lfo.cpp` and `lfo.h` into the `lfo` folder.
3. Restart the Arduino IDE. You can now `#include <lfo.h>` in your sketches.

The library is header‑only in terms of configuration: you can override compile‑time options (like the sine table resolution) before including `lfo.h`.

---

## 2. Configuration: DAC size and sine table

### 2.1 DAC / control range

The constructor takes the **vertical resolution** of your output:

```cpp
#include <lfo.h>

// Example for a 7‑bit modulation range (0..127)
constexpr int MOD_RANGE = 128;
lfo myLFO(MOD_RANGE);
```

- The LFO output will always be in the range `[0, dacSize-1]` before you apply your own offset/centering.
- For DACs you typically use their native resolution (e.g. `4096` for a 12‑bit DAC, `256` for an 8‑bit PWM output, `128` for MIDI‑style CC values, etc.).

### 2.2 Sine table resolution

`lfo.cpp` uses a lookup table for the sine waveform. Its size is configurable at compile time:

```cpp
// Must be defined before including <lfo.h>.
// LFO_SINE_TABLE_BITS = log2(table_size)
// 8  -> 256 samples
// 9  -> 512 samples (default)
// 10 -> 1024 samples
#define LFO_SINE_TABLE_BITS 9
#include <lfo.h>
```

Larger tables give slightly smoother sine waves at the cost of a few more bytes of flash/RAM.

---

## 3. Public API

### 3.1 Constructor

```cpp
lfo::lfo(int dacSize);
```

- **`dacSize`**: maximum output value + 1.  
  Output range is `[0, dacSize-1]`.

### 3.2 Basic parameters

```cpp
void setWaveForm(int waveForm);     // 0=off, 1=saw, 2=triangle, 3=sin, 4=square
void setAmpl(int ampl);             // amplitude: 0..dacSize-1
void setAmplOffset(int offset);     // DC offset: 0..dacSize-1
```

- The final signal is **centered around `offset`** and has an approximate peak‑to‑peak magnitude of `ampl`.
- For example, with `dacSize=128`, `ampl=127`, `offset=0`, the signal swings between `0` and `127`.

### 3.3 Mode selection: free‑running vs BPM‑sync

```cpp
void setMode(bool mode);            // false = free‑running, true = BPM‑synced
```

- **Mode 0 (free‑running)** uses a frequency in Hz.
- **Mode 1 (sync)** derives frequency from BPM and a musical rate.

#### Mode 0 – free‑running

```cpp
void setMode0Freq(float freqHz);
void setMode0Freq(float freqHz, unsigned long t); // compatibility overload
```

- `freqHz` is the **LFO frequency in Hz**.
- In this optimized implementation the second overload **ignores** the timestamp and behaves like the first one (no phase‑continuous morphing, but faster).

#### Mode 1 – BPM‑sync

```cpp
void setMode1Bpm(float bpm);
void setMode1Rate(float rate);      // see table below
void setMode1Phase(float phase);    // reserved, currently no effect
void sync(unsigned long t);         // reset phase at timestamp t (use micros())
```

Effective LFO frequency in sync mode:

```text
freq_hz = rate * bpm / 60
```

From the table in `lfo.h`, the `rate` values map to musical note lengths:

```text
 rate | LFO cycle duration
------+--------------------
 0.125| 2 bars
 0.25 | 1 bar
 0.5  | half note
 1    | quarter note
 2    | 1/8 note
 3    | 1/12 note
 4    | 1/16 note
 ...
 16   | 1/64 note
```

### 3.4 Reading the LFO

```cpp
int   getWaveForm();
int   getAmpl();
int   getAmplOffset();
bool  getMode();
float getMode0Freq();
float getMode1Rate();
float getPhase();                   // normalized phase [0,1)
int   getWave(unsigned long t);     // main output (call with micros())
```

- `getWave(t)` is **the core function**: you pass a timestamp in microseconds (usually from `micros()`).
- The LFO internally tracks elapsed time and advances a 32‑bit phase accumulator accordingly, so you **do not need a fixed control rate**.

---

## 4. Minimal examples

### 4.1 Simple DAC LFO (Arduino Due style)

This is similar to the original example, but kept short:

```cpp
#include <lfo.h>

constexpr int DACSIZE = 4096;
lfo lfo_class(DACSIZE);

void setup() {
  analogWriteResolution(12);        // Due only

  lfo_class.setWaveForm(1);         // saw
  lfo_class.setAmpl(DACSIZE - 1);   // full scale
  lfo_class.setAmplOffset(0);       // unipolar 0..4095
  lfo_class.setMode(0);             // free‑running
  lfo_class.setMode0Freq(30.0f);    // 30 Hz
}

void loop() {
  unsigned long t = micros();
  analogWrite(DAC0, lfo_class.getWave(t));
}
```

For other boards without a true DAC:

- Reduce `DACSIZE` to match your PWM resolution (e.g. 256).
- Remove `analogWriteResolution(12)`.
- Use the appropriate `analogWrite()` / `dacWrite()` call for your target.

---

## 5. Example: RP2040 / DCO synth style usage (bipolar modulation)

This example mirrors how the library is used in the DCO synth project:  
two LFOs generate **bipolar modulation signals** (centered around 0) that are then used to modulate detune and other parameters in fixed‑point math.

### 5.1 Concept

- We choose a **MIDI‑style range** (0..127) for modulation depths.
- Each LFO uses `dacSize = CC_MAX + 1`, giving an integer output in `[0, CC_MAX]`.
- We subtract half the range to get a **bipolar value** around zero: `[-CC_MAX/2, +CC_MAX/2]`.
- That bipolar value can be multiplied by a scaling factor (float or fixed‑point) to produce the final modulation.

### 5.2 Code

```cpp
#include <lfo.h>

// 7‑bit LFO modulation range (e.g. MIDI CC)
constexpr int LFO_CC       = 127;
constexpr int LFO_CC_RANGE = LFO_CC + 1;
constexpr int LFO_CC_HALF  = LFO_CC_RANGE / 2;   // 64

// Two LFOs: one for pitch/DETUNE, one for e.g. PWM or a second modulation
lfo LFO1(LFO_CC_RANGE);
lfo LFO2(LFO_CC_RANGE);

// Example scaling factors (could also be fixed‑point in a synth engine)
float LFO1_to_detune = 0.005f;   // semitones per LFO step
float LFO2_to_pwm    = 0.01f;    // duty delta per LFO step

void init_LFOs() {
  // LFO1: slow triangle for pitch detune
  LFO1.setWaveForm(2);                 // triangle
  LFO1.setAmpl(LFO_CC);                // 0..127
  LFO1.setAmplOffset(0);               // unipolar, we center later
  LFO1.setMode(0);                     // free‑running
  LFO1.setMode0Freq(0.5f);             // 0.5 Hz

  // LFO2: faster sine for PWM or other modulation
  LFO2.setWaveForm(3);                 // sine
  LFO2.setAmpl(LFO_CC);
  LFO2.setAmplOffset(0);
  LFO2.setMode(0);
  LFO2.setMode0Freq(5.0f);             // 5 Hz
}

void setup() {
  init_LFOs();
}

void loop() {
  unsigned long now = micros();

  // Center both LFOs around 0: [-LFO_CC_HALF, +LFO_CC_HALF]
  int lfo1_raw  = LFO1.getWave(now);
  int lfo2_raw  = LFO2.getWave(now);
  int lfo1_bi   = lfo1_raw - LFO_CC_HALF;
  int lfo2_bi   = lfo2_raw - LFO_CC_HALF;

  // Map to application‑specific domains (here using floats for clarity)
  float detune_semitones = lfo1_bi * LFO1_to_detune;
  float pwm_delta        = lfo2_bi * LFO2_to_pwm;

  // In a real DCO engine you would:
  // - convert detune_semitones to a frequency ratio or Q‑format
  // - apply pwm_delta to your pulse‑width parameter
  // - and then update your oscillators
}
```

### 5.3 How this maps to the synth implementation

In the synth project:

- `LFO1` is used to modulate **detune** using fixed‑point math (Q24) instead of float:
  - The bipolar LFO value (`LFO1Level`) is multiplied by a pre‑scaled fixed‑point factor (`LFO1toDCO_q24`).
  - This keeps the **LFO computations fast and deterministic** inside a tight audio/control loop.
- `LFO2` is used for **other modulation paths** (e.g. PWM or OSC2 detune) in a similar fashion.
- A set of per‑oscillator “drift” LFOs use the same class with slightly randomized speeds and/or phases to simulate analog drift across voices.

The important pattern is:

1. Use the `lfo` class to generate a stable, time‑based modulation signal driven by `micros()`.
2. Convert its integer output into the numeric domain your synth uses (float or fixed‑point).
3. Apply it as a **small delta** on top of your base parameter (pitch, PWM, filter cutoff, etc.).

---

## 6. Notes and updates

- 29.12.2020: `_mode1_rate` changed from `int` to `float` to support rates < 1. Added more getters.
- 16.07.2022: Saw and triangle updated to be compatible with Arduino UNO (thanks to othmar52).
- 2024+: Internal implementation optimized for:
  - 32‑bit fixed‑point phase accumulator (`uint32_t`)
  - Efficient per‑microsecond phase increments for both free and sync modes
  - Optional configurable sine table size via `LFO_SINE_TABLE_BITS`

Have fun :-)

Many thankls to mo‑thunderz, what a cool guy!
