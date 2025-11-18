## DCO / VCO “Autotune” Overview

This document explains how the current “autotune” code for the DCO synth oscillators works and outlines a plan to improve its readability and maintainability.

### Objectives

- **Primary goal**: Build, for each DCO, a mapping from **note frequency → range PWM value** (`ampCompCalibrationVal`) so that:
  - The DCO waveform’s duty cycle is close to the desired (≈50% or other target), and
  - The perceived amplitude remains more consistent across the keyboard.
- **Secondary goals**:
  - Find and store **PW center** and limits for each voice.
  - Discover each oscillator’s **highest usable note**.
  - Persist calibration tables so that the runtime engine can perform fast interpolation instead of re‑measuring.

### Hardware Context (high level)

- **MCU**: RP2040 (Raspberry Pi Pico–compatible board).
- **Per voice**: 2 DCOs (oscillators).
- **Control signals**:
  - A **range PWM** per DCO, which controls the analog amplitude / duty behaviour.
  - A **PW PWM** per voice, which controls the pulse‑width of the waveform.
  - DCOs are driven by **PIO state machines**.
- **Measurement input**:
  - A digital pin (`DCO_calibration_pin`) receives the DCO signal.
  - The code times rising and falling edges using `micros()` to infer duty cycle and frequency.

---

## Data Structures and Globals (current design)

The autotune system is heavily global, with state defined in headers:

- **Flags and indices** (from `autotune.h`):
  - `calibrationFlag`, `manualCalibrationFlag`, `firstTuneFlag`
  - `currentDCO`
  - `DCO_calibration_current_note`, `DCO_calibration_current_voice`, `DCO_calibration_current_OSC`
  - `manualCalibrationOffset[NUM_OSCILLATORS]`
- **Calibration table**:
  - `calibrationData[chanLevelVoiceDataSize]`
    - Flat array of `uint32_t` pairs:
      - `[freq0, pwm0, freq1, pwm1, ...]`
      - `freq` values are `sNotePitches[note] * 100`
      - `pwm` values are `ampCompCalibrationVal` (range PWM) that produced best behaviour at that frequency.
- **Measurement / timing state**:
  - `edgeDetectionLastTime`, `microsNow`, `currentNoteCalibrationStart`, `DCOCalibrationStart`
  - `pulseCounter`, `samplesCounter`
  - `risingEdgeTimeSum`, `fallingEdgeTimeSum`
  - `DCO_calibration_difference` (central error metric)
  - `highestNoteOSC[NUM_OSCILLATORS]`
- **PID‑related state** (from `PID.h`):
  - Standard PID variables:
    - `PIDSetpoint`, `PIDInput`, `PIDOutput`
    - Tunings: `aggKp`, `aggKi`, `aggKd`, `midKp`, `midKi`, `midKd`, `consKp`, `consKi`, `consKd`
    - `myPID` instance from `PID_v1`
  - Calibration search helpers:
    - `PIDMinGap`, `PIDMinGapCounter`
    - `bestGap`, `bestCandidate`
    - `PIDOutputLowerLimit`, `PIDOutputHigherLimit`, `PIDLimitsFormula`
    - `sampleTime`
    - `arrayPos` (index into `calibrationData`)

> **Note**: These globals make the code work but tie everything together tightly and make behaviour harder to reason about.

---

## Measurement Core: Duty‑Cycle and Frequency Error

### `find_gap(byte specialMode)`

This is the **core measurement primitive** used in multiple calibration routines.

- **Inputs**:
  - `specialMode == 0`: normal operation.
  - `specialMode == 2`: “find lowest freq” mode, uses more samples.
- **Process**:
  - Sets `samplesNumber`:
    - `10` for normal operation, `14` for special mode.
  - Initializes `edgeDetectionLastTime = micros()`.
  - Loops, reading `digitalRead(DCO_calibration_pin)` and timing edges:
    - If no edges for > 100ms:
      - Resets counters.
      - Sets `DCO_calibration_difference = 1.16999f` as a **sentinel** meaning “timeout / invalid”.
      - Returns `1.16999f`.
    - On each debounced edge change (≥ 30µs since last):
      - After a couple of initial pulses, accumulates durations:
        - Low time into `fallingEdgeTimeSum`
        - High time into `risingEdgeTimeSum`
      - Increments `samplesCounter`.
  - Once `samplesCounter == samplesNumber`:
    - Computes:

      \[
      DCO\_calibration\_difference =
      \frac{\text{fallingEdgeTimeSum}}{\text{samplesNumber}/4} -
      \frac{\text{risingEdgeTimeSum}}{\text{samplesNumber}/4}
      \]

      This is effectively **(average LOW duration − average HIGH duration)** scaled.
      A perfectly symmetric 50% duty cycle yields ≈0.
    - Resets counters and returns `DCO_calibration_difference`.

- **Interpretation**:
  - `DCO_calibration_difference > 0` → low portion longer than high → wrong duty direction.
  - `DCO_calibration_difference < 0` → high portion longer than low.
  - Magnitude indicates how badly the duty deviates from 50%.

### Related helpers

- **`DCO_calibration_debug()`**:
  - Variant of `find_gap()` used for debugging. Repeatedly prints `DCO_calibration_difference` and note.
- **`VCO_measure_frequency()`**:
  - Uses total period (`risingEdgeTimeSum + fallingEdgeTimeSum`) to compute **frequency**.
  - Sets `DCO_calibration_difference = targetFreq − measuredFreq` as a frequency error instead of duty error.
- **`DCO_calibration_find_highest_freq()`, `PID_find_highest_freq()`, `find_highest_freq()`**:
  - Use similar timing logic plus PID or fixed search to find the **highest achievable frequency** for the current DCO and map it to a note.

---

## Calibration Entry Points

### `init_DCO_calibration()`

Legacy initializer for PID‑based DCO calibration:

- Sets `currentDCO = 0`.
- Sets starting note:
  - `VOICE_NOTES[0] = DCO_calibration_start_note` (e.g. 23)
  - `DCO_calibration_current_note = DCO_calibration_start_note`
- Initializes the first entries of `calibrationData`:
  - `[0] = 0` (placeholder)
  - `[1] = ampCompLowestFreqVal`
  - `[2] = sNotePitches[manual_DCO_calibration_start_note - 12] * 100`
  - `[3] = initManualAmpCompCalibrationVal[currentDCO]`
- Sets PID parameters (`PIDMinGap`, `samplesNumber`, `sampleTime`, output limits).
- Turns off all oscillators and centers PW for even voices.
- Calls `voice_task_autotune(0, PIDLimitsFormula)` to start the first measurement.

> This path works with `PID_dco_calibration()`, which is largely superseded by the newer `calibrate_DCO()` routine.

### `DCO_calibration()`

Main DCO amplitude calibration routine (current “production” path):

1. **Global shutdown / reset**:
   - For each oscillator:
     - Stop its PIO output and set range PWM to 0.
   - For each even voice:
     - Set `PW[i] = DIV_COUNTER_PW / 2` and write to PW PWM.
2. **Per‑DCO loop**:
   - For `currentDCO` from 0 to `NUM_OSCILLATORS - 1`:
     - Call `restart_DCO_calibration()` to:
       - Reinitialize timing, thresholds, and `calibrationData` header.
     - Set:
       - `ampCompCalibrationVal = initManualAmpCompCalibrationVal[currentDCO] + manualCalibrationOffset[currentDCO]`
       - Apply it to the DCO’s range PWM.
     - If the DCO index is even:
       - If `firstTuneFlag` is true:
         - Run `find_PW_center(0)` and store PW center.
       - Else:
         - Run `find_PW_center(0)` to refine / confirm PW center.
     - If the DCO index is odd:
       - Reset `DCO_calibration_current_note` and `VOICE_NOTES[0]` to the start note.
     - Run **amplitude calibration**:
       - Call `calibrate_DCO()` (see next section).
       - Print out the `calibrationData` for inspection.
       - Call `update_FS_voice(currentDCO)` to persist the table.
     - Call `restart_DCO_calibration()` to reset for the next DCO.
3. **Finalization**:
   - `calibrationFlag = false`
   - `init_FS()` to reload calibration data from storage.
   - `precompute_amp_comp_for_engine()` to rebuild runtime amp‑comp tables.

### `VCO_calibration()`

Similar structure to `DCO_calibration()`, with some differences in how notes are set and without PW centering.

- Reuses the same **`calibrate_DCO()`** function to fill `calibrationData` for VCO‑style calibration.

### `restart_DCO_calibration()`

Shared reset routine:

- Resets starting note and `DCO_calibration_current_note`.
- Initializes `calibrationData` header:
  - `calibrationData[0] = 0`
  - `calibrationData[1] = ampCompLowestFreqVal`
  - `calibrationData[2] = sNotePitches[startNote - calibration_note_interval - 12] * 100`
  - `calibrationData[3] = initManualAmpCompCalibrationVal[currentDCO] + manualCalibrationOffset[currentDCO]`
- Clears timers, thresholds (`PIDMinGap`), search trackers (`bestGap`, `bestCandidate`), and counters.
- Turns off all oscillators (safety) and delays briefly.

---

## PW Center and Limits

### `find_PW_center(uint8_t mode)`

Goal: for the **current DCO’s voice** (group of two DCOs), find the **PW value** that yields the best duty behaviour at a reference note.

- **Modes**:
  - `mode == 0`:
    - Uses `manual_DCO_calibration_start_note` as the calibration note.
    - Target gap (`bestGap`) threshold ≈ 20.
    - `voiceTaskMode = 2`.
  - `mode == 1`:
    - Uses a high note (e.g. 76).
    - Tighter target gap ≈ 5.
    - `voiceTaskMode = 3`.
- **Process**:
  - Sets `VOICE_NOTES[0]` and `DCO_calibration_current_note` accordingly.
  - Sets `PIDOutputLowerLimit = 0`, `PIDOutputHigherLimit = DIV_COUNTER_PW`.
  - Chooses initial `PWCalibrationVal`:
    - If `firstTuneFlag`:
      - Use middle of the range and write back into `PW_CENTER`.
    - Else:
      - Start from previously stored `PW_CENTER`.
  - Calls `voice_task_autotune(voiceTaskMode, ampCompCalibrationVal)`.
  - Enters a loop:
    - Writes `PWCalibrationVal` to the PW PWM.
    - Waits (`delay(30)`).
    - Calls `find_gap(2)` to get a duty error estimate.
    - Tracks:
      - `bestGap` and `bestCandidate` (the PW that produced the smallest magnitude error).
      - Sign changes between the current gap and `lastDCODifference` to detect “swinging”.
    - Adjusts `PWCalibrationVal` (coarse or fine steps) based on gap sign and magnitude.
    - Breaks when `bestGap <= targetGap` or swing logic indicates diminishing returns.
  - At the end, calls `update_FS_PWCenter(currentDCO/2, bestCandidate)` and updates `PW_CENTER[currentDCO/2]`.

### `find_PW_low_limit()`

Goal: find the **lowest PW value** that still yields acceptable duty/frequency behaviour.

- Similar loop using `find_gap(0)` as the metric.
- Starts from `PW_CENTER[currentDCO/2] / 2`.
- Uses a target error proportional to the period at `DCO_calibration_start_note`.
- After convergence, calls `update_FS_PW_Low_Limit(currentDCO/2, PWCalibrationVal)` and updates `PW_LOW_LIMIT[currentDCO/2]`.

---

## Legacy PID‑Based DCO Calibration

### `PID_dco_calibration()`

Older calibration loop that uses the PID library to drive `ampCompCalibrationVal` per note.

- **At each iteration**:
  - Reads `DCO_calibration_difference` (from `find_gap`) and clamps to ±1500 → `PIDInput`.
  - `PIDgap = abs(PIDSetpoint - PIDInput)` measures how far we are from the target (0).
  - Over time, if calibration takes too long, `PIDMinGap` is gradually increased to relax the tolerance.
  - Tracks `bestGap` and `bestCandidate` based on `PIDgap`.
  - Detects sign flips of `DCO_calibration_difference` vs `lastDCODifference` and adjusts `PIDMinGap` or declares a “swing”.
  - When `PIDgap < PIDMinGap` or swings indicate it’s not improving:
    - Writes the current note’s calibration pair into `calibrationData`:
      - Frequency (`sNotePitches[DCO_calibration_current_note - 12] * 100`)
      - Best PWM (`bestCandidate`)
    - Moves to the next note (`DCO_calibration_current_note += calibration_note_interval`).
    - Recomputes `PIDMinGap`, new `PIDOutputLowerLimit` / `PIDOutputHigherLimit`, etc.
    - Resets trackers and starts measurement again at the new note.
  - Otherwise:
    - Adjusts `ampCompCalibrationVal` ±1 or ±2 based on the magnitude of `DCO_calibration_difference`.

> The new `calibrate_DCO()` routine captures similar behaviour with more explicit search and interpolation and less reliance on PID internals.

---

## Newer Calibration Loop: `calibrate_DCO()`

This function is the **current main algorithm** for building the `[frequency, PWM]` calibration table for each DCO (and reused for VCO).

### High‑level idea

For a set of calibration notes:

1. Use interpolation from previously calibrated points to **guess a good starting PWM** for the next note.
2. Around that starting point, use **simple search plus sign‑change detection** and **local neighbour probing** to find the PWM that makes `find_gap(0)` as close to 0 as possible.
3. Store the best result in `calibrationData` as `[freq, pwm]`.

### Detailed flow

- Constants / setup:
  - `rangeSamples = 2`
    - Number of neighbouring voltages probed around each sign change.
  - `numPresetVoltages = chanLevelVoiceDataSize`
    - Total number of calibration entries (2 per note: freq + pwm).

- Main loop:

```cpp
for (int j = 4; j < numPresetVoltages; j += 2) {
    ...
}
```

For each `j`:

1. **Determine calibration note**:
   - `DCO_calibration_current_note = DCO_calibration_start_note + (calibration_note_interval * (j - 4) / 2);`
   - `VOICE_NOTES[0] = DCO_calibration_current_note;`

2. **Pick initial PWM guess (`currentAmpCompCalibrationVal`)**:
   - For the first calibrated note (`j == 4`):
     - Start from `(initManualAmpCompCalibrationVal[currentDCO] + manualCalibrationOffset[currentDCO]) * 1.35`.
   - For the second (`j == 6`):
     - Use **logarithmic interpolation** between the first two header entries in `calibrationData`.
   - For later notes:
     - Use **quadratic interpolation** based on the previous three calibration points in the table.

3. **Check for hardware limit**:
   - If `currentAmpCompCalibrationVal > DIV_COUNTER * 0.98`:
     - Call `find_highest_freq()` to find the DCO’s maximum achievable frequency.
     - Store this as an endpoint in `calibrationData[j]` with `calibrationData[j+1] = DIV_COUNTER`.
     - Fill remaining entries with sentinel values indicating “unreachable”.
     - Break out of the loop (calibration done at high end).

4. **Set allowed PWM range and tolerance**:
   - `minAmpComp = currentAmpCompCalibrationVal * 0.8;`
   - `maxAmpComp = currentAmpCompCalibrationVal * 1.3;`
   - `tolerance = 1e6 / f^2 / 4`, where:
     - `f = sNotePitches[VOICE_NOTES[0] - 12]`
   - This makes the tolerance tighter at higher frequencies (shorter periods).

5. **Search loop for best PWM**:

   - Initialize:
     - `bestAmpComp = currentAmpCompCalibrationVal`
     - `closestToZero = large number` (e.g. 50000)
     - Buffers for neighbourhood measurements:
       - `lowerMeasurements[]`, `higherMeasurements[]`
       - `lowerVoltages[]`, `higherVoltages[]`
     - `flipCounter = 0`
   - Loop:
     - Apply the current PWM to hardware:
       - `voice_task_autotune(0, currentAmpCompCalibrationVal)`
     - `delay(10)`
     - Measure:
       - `avgValue = find_gap(0)`
       - Ignore sentinel `1.16999f` (measurement timeout).
     - Update best:
       - If `abs(avgValue)` is smaller than `abs(closestToZero)`:
         - Update `closestToZero` and `bestAmpComp`.
     - Detect sign changes:
       - If `avgValue` and `previousAvgValue` have opposite signs:
         - For each `i` in `[0 .. rangeSamples-1]`:
           - Probe `currentAmpCompCalibrationVal - (i+1)` and `+ (i+1)`.
           - Measure `find_gap(0)` for each and store into `lowerMeasurements[i]`, `higherMeasurements[i]`.
         - Compare all stored gaps (and the current one) to refine `bestAmpComp` / `closestToZero`.
         - If `abs(closestToZero) <= tolerance`:
           - Break (we are close enough).
         - Else:
           - Relax the tolerance (`tolerance *= 1.2`, then potentially `*= 1.5` after multiple flips).
           - Increment `flipCounter` and break when flips exceed a threshold with reasonably small error.
     - If not yet done:
       - Adjust the PWM:
         - For errors smaller than `tolerance * 20`: step by ±1.
         - For larger errors: step by ±2.
       - Check for being outside `[minAmpComp, maxAmpComp]` and log a warning.
     - Update `previousAvgValue = avgValue` and continue.

6. **Store the note’s calibration pair**:

   - Once the loop ends:

     - `calibrationData[j]   = sNotePitches[DCO_calibration_current_note - 12] * 100;`
     - `calibrationData[j+1] = bestAmpComp;`

   - Log the chosen PWM and error, then move to the next note.

---

## How Calibration Data is Used at Runtime

After a full `DCO_calibration()` / `VCO_calibration()` pass:

- For each DCO:
  - `update_FS_voice(currentDCO)` writes `calibrationData` into persistent storage (flash / FS).
- Once all oscillators are processed:
  - `init_FS()` reloads calibration tables.
  - `precompute_amp_comp_for_engine()` builds ready‑to‑use **amp‑comp lookup tables**.

At runtime:

- For any requested note / frequency:
  - The engine looks up the DCO’s calibration table and interpolates between nearby entries.
  - The resulting `ampCompCalibrationVal` is written to the range PWM, compensating amplitude across the keyboard.

---

## Refactor Plan

The current autotune implementation is functional but hard to maintain because of:

- Heavy use of **global mutable state** declared in headers.
- **Magic constants** embedded throughout the code.
- **Mixed responsibilities** (hardware I/O, algorithm, and logging in the same functions).
- Duplicated edge‑measurement logic and sentinel values.

Below is a proposed, incremental refactor plan that aims to **preserve behaviour** while improving structure and readability.

### Phase 1 – Documentation and Naming (low risk)

- **Extract documentation**:
  - Use this `AUTOTUNE.md` as the living reference for how calibration works.
  - Add brief block comments or `///` Doxygen‑style comments above key functions:
    - `DCO_calibration()`, `VCO_calibration()`, `calibrate_DCO()`, `find_gap()`, `find_PW_center()`, etc.
- **Replace magic constants with named `constexpr` values**:
  - Examples:
    - `1.16999f` → `kGapTimeoutSentinel`
    - `30` (debounce microseconds) → `kEdgeDebounceUs`
    - `100000` (timeout in `find_gap`) → `kGapTimeoutUs`
    - Ratios like `0.8`, `1.3`, `20`, etc., for tolerance and PWM range scaling.
  - Place these constants in a dedicated header (e.g. `autotune_constants.h`) and include where needed.
- **Clarify naming**:
  - Prefer more descriptive names where it doesn’t break too many references:
    - `DCO_calibration_difference` → `dutyErrorUs` (or similar)
    - `bestCandidate` → `bestAmpCompPwm`, etc.

### Phase 2 – Encapsulate Calibration State

- **Introduce a `struct DCOCalibrationContext`** holding:
  - Current note, DCO index, current PWM, bestGap, best PWM, timers, etc.
  - This replaces many globals (or at least groups them).
- **Refactor functions to take a context reference**:
  - Example:
    - `float find_gap(DCOCalibrationContext& ctx, byte specialMode);`
    - `void calibrate_DCO(DCOCalibrationContext& ctx);`
  - Internally, this can still use some globals at first, but state will be more visibly grouped.
- **Optionally create a `CalibrationSession` struct**:
  - Handles per‑run data like `calibrationData[]`, `highestNoteOSC[]`, etc.

### Phase 3 – Extract Measurement Module

- **Create a small “measurement” module**, e.g. `autotune_measurement.{h,ino}`:
  - Responsible for:
    - Edge timing (`find_gap`, `DCO_calibration_debug`, `VCO_measure_frequency`).
    - Standard return types:
      - `enum class GapResultStatus { Ok, Timeout };`
    - Functions returning structured results:
      - `struct GapResult { GapResultStatus status; float dutyError; };`
  - Advantage:
    - Calibration algorithms (`calibrate_DCO`, `PID_dco_calibration`, PW center finding) simply **consume** these results without worrying about digital I/O details.

### Phase 4 – Separate Algorithm from Hardware I/O

- **Introduce a thin hardware abstraction layer** for autotune:
  - Functions like:
    - `void set_dco_amp(uint8_t dcoIndex, uint16_t ampPwm);`
    - `void set_voice_pw(uint8_t voiceIndex, uint16_t pwPwm);`
    - `void disable_all_oscillators();`
  - These wrap:
    - `pio_sm_put`, `pio_sm_exec`
    - `pwm_set_chan_level`, `pwm_set_enabled`
  - This allows:
    - Calibration algorithms to be written in terms of **“set amp to X, measure gap”**.
    - Potential test harnesses or simulations to mock out hardware functions.

- **Cleanly isolate logging**:
  - Wrap `Serial.println` calls behind macros or functions with log levels.
  - Allows easier toggling of verbosity and makes the core loops less noisy.

### Phase 5 – Consolidate and Simplify Algorithms

- **Unify PID and non‑PID paths where possible**:
  - Decide whether to keep `PID_dco_calibration()` as a legacy option or remove it entirely once `calibrate_DCO()` is stable.
  - If both are kept:
    - Extract shared logic (e.g. note iteration, table writing) into common helpers.
- **Simplify the search logic**:
  - The current `calibrate_DCO()` already does interpolation + local search.
  - After refactoring:
    - Consider extracting a generic “target‑zero search” helper:
      - Inputs: initial guess, min/max bounds, tolerance, function `measureError(pwm)`.
      - Output: best PWM, best error.
    - This helper could then be reused for:
      - DCO amplitude calibration.
      - PW center and limits.
      - Lowest / highest frequency search.

### Phase 6 – Testing and Safety

- **Add regression hooks**:
  - Provide an easy way to **dump calibration tables** to serial or storage.
  - Allow re‑loading previously known‑good tables to compare new vs old behaviour.
- **Small‑step refactoring**:
  - After each change:
    - Run a full calibration on hardware.
    - Compare resulting `calibrationData` and resulting sound to a baseline.
  - Keep high‑risk changes (e.g. algorithm changes) separate from low‑risk cleanups (naming, comments, constants).

---

## Summary

- The autotune system:
  - Measures DCO waveform duty using edge timing on a digital pin.
  - Searches for PWM values that minimize duty error at a set of calibration notes.
  - Stores those `(frequency, PWM)` pairs per DCO for fast runtime interpolation.
- The newer `calibrate_DCO()` routine is the main workhorse for DCO/VCO amplitude calibration, building tables using interpolation + local search.
- The refactor plan focuses on:
  - Documenting behaviour,
  - Naming and extracting constants,
  - Encapsulating calibration state,
  - Separating measurement and hardware I/O from algorithm,
  - And gradually simplifying the calibration logic without changing its observable behaviour.


