# DCO4_DCO Project: Floating-Point Operations Analysis for Fixed-Point Conversion

This document lists all floating-point operations within the `voice_task` function and its callees. It serves as a reference for migrating to fixed-point arithmetic using a library like `fixmath`.

---

## Analysis of `voice_task()` in `voices.ino`

The `voice_task` function is the primary real-time processing loop for each synthesizer voice.

### 1. Floating-Point Variable Declarations

- `float calcPitchbend;`
- `register float freq;`
- `register float freq2;`
- `float ADSRModifier`, `ADSRModifierOSC1`, `ADSRModifierOSC2`
- `float unisonMODIFIER`
- `float DETUNE_DRIFT_OSC1`, `DETUNE_DRIFT_OSC2`
- `float modifiersAll`, `freqModifiers`, `freq2Modifiers`
- `float ADSR1toPW_calculated`, `LFO2toPW_calculated`

### 2. Pitch Bend Calculation

- **Location:** `voices.ino`, inside `voice_task()`
- **Code Snippet:**
  ```cpp
  if (midi_pitch_bend < 8192) {
    calcPitchbend = (((float)midi_pitch_bend / 8190.99f) - 1.0f) * pitchBendMultiplier;
  } else {
    calcPitchbend = (((float)midi_pitch_bend / 8192.99f) - 1.0f) * pitchBendMultiplier;
  }
  ```
- **Operations:** Type casting `int` to `float`, `float / float`, `float - float`, `float * float`.
- **Variables:** `pitchBendMultiplier` is a `float`.

### 3. Oscillator 2 Detune

- **Location:** `voices.ino`, inside `voice_task()`
- **Code Snippet:**
  ```cpp
  OSC2_detune = 1.00f + (0.0002f * ((int)256 - OSC2DetuneVal));
  ```
- **Operations:** `float * int`, `float + float`.
- **Variables:** `OSC2_detune` is a `float`.

### 4. Portamento (Glide)

- **Location:** `voices.ino`, inside `voice_task()`
- **Code Snippet:**
  ```cpp
  freqPortaInterval[DCO_A] = (float)(portamento_stop[DCO_A] - (float)portamento_start[DCO_A]) / portamento_time;
  // ...
  portamento_cur_freq[DCO_A] = (float)portamento_start[DCO_A] + (float)(freqPortaInterval[DCO_A] * (float)portamentoTimer[i]);
  ```
- **Operations:** `float - float`, `float / int`, `float * float`, `float + float`.
- **Variables:** `portamento_start`, `portamento_stop`, `portamento_cur_freq`, and `freqPortaInterval` are all `float` arrays. Assignments from the `sNotePitches` array (which is `float[]`) are also floating-point operations.

### 5. Modulation Calculations

- **Location:** `voices.ino`, inside `voice_task()`
- **Code Snippet:**
  ```cpp
  float ADSRModifier = (ADSR1toDETUNE1 != 0) ? ((float)linToLogLookup[ADSR1Level[i]] * ADSR1toDETUNE1_formula) : 0;
  float unisonMODIFIER = (unisonDetune != 0) ? (0.00006f * unisonDetune * ((i & 0x01) == 0 ? -(i - 1) : -i)) : 0;
  float DETUNE_DRIFT_OSC1 = (analogDrift != 0) ? (LFO_DRIFT_LEVEL[DCO_A] * 0.0000005f * analogDrift) : 0;
  // ...
  float modifiersAll = DETUNE_INTERNAL_FIFO_float + unisonMODIFIER + calcPitchbend + 1.00001f;
  float freqModifiers = ADSRModifierOSC1 + DETUNE_DRIFT_OSC1 + modifiersAll;
  ```
- **Operations:** `float * float`, `float * int`, `float + float`.
- **Variables:** `ADSR1toDETUNE1_formula` and `DETUNE_INTERNAL_FIFO_float` are `float`.

### 6. Final Frequency Calculation

- **Location:** `voices.ino`, inside `voice_task()`
- **Code Snippet:**
  ```cpp
  freq = freq * (float)((float)interpolatePitchMultiplier(freqModifiers) / (float)multiplierTableScale);
  freq2 = freq2 * OSC2_detune * (float)((float)interpolatePitchMultiplier(freq2Modifiers) / (float)multiplierTableScale);
  ```
- **Operations:** `float * (int / float)`, `float * float * (int / float)`. Involves a call to `interpolatePitchMultiplier`.

### 7. PIO Clock Divider Calculation

- **Location:** `voices.ino`, inside `voice_task()`
- **Code Snippet:**
  ```cpp
  register uint32_t clk_div1 = (uint32_t)((eightSysClock_Hz / freq) - eightPioPulseLength);
  // ...
  clk_div2 = (uint32_t)(sysClock_Hz / freq2);
  ```
- **Operations:** `float / float`.
- **Variables:** `eightSysClock_Hz` and `sysClock_Hz` are `float` constants.

### 8. Amplitude Compensation and PWM Modulation

- **Location:** `voices.ino`, inside `voice_task()`
- **Code Snippet:**
  ```cpp
  chanLevel = get_chan_level_lookup((int32_t)(freq * 100), DCO_A);
  // ...
  float ADSR1toPW_calculated = (ADSR1toPWM != 0) ? ((float)ADSR1Level[i] / 4.00f * ADSR1toPWM_formula) : 0;
  float LFO2toPW_calculated = (LFO2toPW != 0) ? ((float)LFO2Level * LFO2toPWM_formula) : 0;
  PW_PWM[i] = (uint16_t)constrain((DIV_COUNTER_PW - 1 - LFO2toPW_calculated - PW[0] + ADSR1toPW_calculated), 0, DIV_COUNTER_PW - 1);
  ```
- **Operations:** `float * int`, `max(float, float)`, `int / float`, `float * float`, `float + float`, `float - float`.

---

## Analysis of Functions Called From `voice_task()`

### 1. `interpolatePitchMultiplier(float x_float)`

- **Location:** `voices.ino`
- **Code Snippet:**
  ```cpp
  inline int32_t interpolatePitchMultiplier(float x_float) {
    int32_t x = (int32_t)(x_float * (float)multiplierTableScale);
    // ... rest of the function is integer arithmetic
  }
  ```
- **Operation:** `float * float`. This is the only floating-point operation.

### 2. `get_chan_level_lookup(int32_t x, uint8_t voiceN)`

- **Location:** `voices.ino`
- **Code Snippet:**
  ```cpp
  inline uint16_t get_chan_level_lookup(int32_t x, uint8_t voiceN) {
    // ...
    float interpolatedValue = aCoeff[voiceN][i] * x * x + bCoeff[voiceN][i] * x + cCoeff[voiceN][i];
    return round(interpolatedValue);
    // ...
  }
  ```
- **Operations:** A full quadratic equation evaluation: `float * int * int`, `float * int`, `float + float`.
- **Variables:** `aCoeff`, `bCoeff`, `cCoeff` are `float` arrays.
- **Function Call:** `round(float)`.

---
