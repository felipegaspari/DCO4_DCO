## Autotune Refactor – Current Architecture

This document describes the **refactored** version of the autotune / DCO calibration code, focusing on structure, responsibilities, and how the pieces fit together after the clean‑up work.

It is meant to complement `AUTOTUNE.md` (which explains the original behaviour and algorithms) by documenting **how** that behaviour is now organized in code.

---

## High‑Level Overview

### Goals (unchanged)

- Build, for each DCO, a calibration table:
  - **Input**: note frequency.
  - **Output**: range PWM value (`ampCompCalibrationVal`) that yields the desired duty behaviour (≈50%) / amplitude.
- Use these tables at runtime for fast amplitude compensation via interpolation.

### Key structural changes

- Introduced:
  - `DCOCalibrationContext` – a small context struct grouping the main calibration state for `calibrate_DCO`.
  - `autotune_measurement.h` – wrapper around `find_gap()` returning a structured `GapMeasurement`.
  - Small helper functions to:
    - Turn oscillators off and reset PW center (`disable_all_oscillators_and_range_pwm`, `reset_even_pw_to_center`).
    - Measure gap for a given PWM (`measure_gap_for_amp`).
    - Detect sign changes in the duty error (`did_sign_change`).
    - Evaluate neighbour measurements (`update_best_from_neighbours`).
    - Step the PWM according to error (`step_amp_from_error`).
    - Compute the initial PWM guess for each calibration point (`compute_initial_amp_for_note`).
    - Store a per-note calibration result and log it (`store_note_result`).
- Legacy code is clearly marked and wrapped in `#if 0` blocks where unused.

The **calibration algorithm** itself is intentionally unchanged; the refactor focuses on readability, separation of concerns, and making state dependencies explicit.

---

## Files and Their Roles (Refactored)

### `autotune.h`

Defines **global state** used by autotune, now with clearer comments:

- **Flags**:
  - `calibrationFlag`, `manualCalibrationFlag`, `firstTuneFlag`.
- **Manual calibration state**:
  - `manualCalibrationStage`, `manualCalibrationOffset[NUM_OSCILLATORS]`.
- **DCO calibration state**:
  - `calibrationData[chanLevelVoiceDataSize]` – temporary buffer for `[freq, pwm]` pairs.
  - `currentDCO` – index of the DCO being calibrated.
  - Timing helpers: `edgeDetectionLastTime`, `microsNow`, `currentNoteCalibrationStart`, `DCOCalibrationStart`.
  - Edge detection state: `edgeDetectionLastVal`, `pulseCounter`, `samplesCounter`.
  - Amplitude values:
    - `ampCompCalibrationVal` – current range PWM during calibration.
    - `initManualAmpCompCalibrationValPreset` – baseline preset.
    - `initManualAmpCompCalibrationVal[NUM_OSCILLATORS]` – per‑oscillator presets.
    - `ampCompLowestFreqVal` – PWM used when probing very low frequency.
  - Duty / frequency error:
    - `DCO_calibration_difference` – central error value.
    - `samplesNumber` – number of edge samples per measurement.
  - Note indices:
    - `DCO_calibration_start_note`, `calibration_note_interval`, `manual_DCO_calibration_start_note`.
    - `DCO_calibration_current_note`, `DCO_calibration_current_voice`, `DCO_calibration_current_OSC`.
  - Other tracking:
    - `highestNoteOSC[NUM_OSCILLATORS]`.
    - `lastDCODifference`, `lastGapFlipCount`, `lastPIDgap`, `lastampCompCalibrationVal`.
    - `PWCalibrationVal`.
  - `autotuneDebug` – global debug verbosity.

### `PID.h`

Holds **PID controller state** for legacy calibration routines, now documented:

- Main variables: `PIDSetpoint`, `PIDInput`, `PIDOutput`.
- Tunings: `aggKp`, `midKp`, `consKp` plus `PIDKMultiplier`.
- Search helpers: `PIDMinGap`, `PIDMinGapCounter`, `bestGap`, `bestCandidate`.
- Output bounds: `PIDOutputLowerLimit`, `PIDOutputHigherLimit`, `PIDLimitsFormula`.
- Timing: `sampleTime`, `PIDComputeTimer`.
- Table index: `arrayPos`.
- PID instance: `myPID`.

### `autotune_constants.h`

Defines **common constants** to avoid magic numbers:

- `kGapTimeoutSentinel` – special float value used when gap measurements time out.
- `kGapTimeoutUs` – max time without an edge before timeout.
- `kEdgeDebounceMinUs` – min time between edges to consider them valid.

### `autotune_measurement.h`

Wraps the low‑level measurement function and exposes a structured interface:

- Forward declaration:

  - `float find_gap(byte specialMode);`

- `struct GapMeasurement`:
  - `bool timedOut;`
  - `float value;`
- `measure_gap(byte specialMode)`:
  - Calls `find_gap`.
  - Sets `timedOut = (value == kGapTimeoutSentinel)`.
  - Returns `{timedOut, value}`.

Used in:

- `find_PW_center` (autotune.ino).
- Legacy `find_lowest_freq` (PID.ino, under `#if 0`).
- `calibrate_DCO` via higher‑level helper `measure_gap_for_amp`.

### `autotune_context.h`

Introduces a **context struct** for DCO calibration:

- `struct DCOCalibrationContext`:
  - `uint8_t& dcoIndex;` – reference to `currentDCO`.
  - `uint8_t& currentNote;` – reference to `DCO_calibration_current_note`.
  - `uint32_t* calibrationData;` – pointer to `calibrationData[]`.
  - `int8_t* manualOffsetByOsc;` – pointer to `manualCalibrationOffset[]`.
  - `int8_t* initManualAmpByOsc;` – pointer to `initManualAmpCompCalibrationVal[]`.

This is a **view** over existing globals; it does not change storage, but allows functions like `calibrate_DCO` to take a clear parameter representing the current calibration target.

### `autotune.ino`

Main DCO autotune orchestration. Key pieces after refactor:

- Includes:
  - `autotune_constants.h`
  - `autotune_measurement.h`
  - `autotune_context.h`

- **Hardware helpers**:
  - `static void disable_all_oscillators_and_range_pwm()`:
    - For each DCO:
      - Sets PIO SM clock divider and pulls.
      - Sets range PWM channel to 0.
    - Used wherever a global “all off” state is needed.
  - `static void reset_even_pw_to_center()`:
    - For each even voice:
      - Sets `PW[i] = DIV_COUNTER_PW / 2` and writes it via PWM.
    - Used to ensure a known PW configuration before calibration.

- **`init_DCO_calibration()`** (legacy):
  - Sets up the **PID‑based** calibration for oscillator 0.
  - Initializes `calibrationData`, timers, PID bounds.
  - Now uses:
    - `disable_all_oscillators_and_range_pwm();`
    - `reset_even_pw_to_center();`
  - Kept for compatibility; main path uses `DCO_calibration()` + `calibrate_DCO`.

- **`DCO_calibration()`** – main entry point:
  - Uses helpers:
    - `disable_all_oscillators_and_range_pwm();`
    - `reset_even_pw_to_center();`
  - For each DCO:
    - Sets `currentDCO`.
    - Calls `restart_DCO_calibration()` to reset state and `calibrationData` header.
    - Sets `ampCompCalibrationVal` from `initManualAmpCompCalibrationVal + manualCalibrationOffset`.
    - For even DCOs:
      - Calls `find_PW_center(0)` to locate PW center.
    - For odd DCOs:
      - Resets `DCO_calibration_current_note` and `VOICE_NOTES[0]`.
    - Builds a `DCOCalibrationContext ctx`:
      - Binds `currentDCO`, `DCO_calibration_current_note`, `calibrationData`, `manualCalibrationOffset`, `initManualAmpCompCalibrationVal`.
    - Calls `calibrate_DCO(ctx)` to fill the table.
    - Persists via `update_FS_voice(currentDCO)`.
    - Calls `restart_DCO_calibration()` again before the next DCO.
  - After all DCOs:
    - `calibrationFlag = false;`
    - `init_FS();`
    - `precompute_amp_comp_for_engine();`

- **`restart_DCO_calibration()`**:
  - Resets notes, `calibrationData[0..3]`, timers, and error tracking.
  - Now uses:
    - `disable_all_oscillators_and_range_pwm();` to ensure a clean starting point.

- **`find_PW_center(uint8_t mode)`**:
  - Uses `measure_gap(2)` under the hood to locate PW center.
  - Keeps the same algorithm, but with clearer separation of measurement vs logic.

- Other functions (`find_PW_low_limit`, `find_gap`, `DCO_calibration_debug`, etc.) retain their core implementation but are supported by the new constants and measurement helpers.

- Unused / legacy functions are wrapped in `#if 0` **with comments**:
  - `DCO_calibration_find_highest_freq`
  - `VCO_calibration`
  - `VCO_measure_frequency`

### `PID.ino`

Contains the **search‑based DCO calibration algorithm** (`calibrate_DCO`) and interpolation helpers, plus legacy PID routines.

- Includes:
  - `autotune_constants.h`
  - `autotune_measurement.h`
  - `autotune_context.h`

- **New helpers**:

  - `static bool did_sign_change(float previous, float current);`
    - Determines if the duty error changed sign between measurements.
    - Used to trigger neighbour probing near zero.

  - `static float measure_gap_for_amp(uint16_t ampPwm);`
    - Applies `ampPwm` via `voice_task_autotune`.
    - Waits for 10 µs.
    - Calls `measure_gap(0)` and returns `gm.value`.

  - `static void update_best_from_neighbours(...)`
    - Given `lowerMeasurements`, `higherMeasurements`, and their voltages plus `avgValue`, updates:
      - `closestToZero` (best absolute error).
      - `bestAmpComp` (PWM corresponding to that error).
    - Encapsulates the previous inlined loops that compared neighbours.

  - `static void step_amp_from_error(float avgValue, double tolerance, uint16_t& currentAmpCompCalibrationVal);`
    - Adjusts `currentAmpCompCalibrationVal`:
      - If `abs(avgValue) < tolerance * 20` → step by ±1.
      - Else → step by ±2.
    - Keeps previous behaviour but is now named and isolated.

  - `static uint16_t compute_initial_amp_for_note(const DCOCalibrationContext& ctx, int j);`
    - Encapsulates the initial PWM guess for each table index:
      - `j == 4`: manual preset scaled by 1.35.
      - `j == 6`: logarithmic interpolation based on the first two calibration pairs.
      - Else: quadratic interpolation based on the previous three calibration pairs.

  - `static double compute_gap_tolerance_for_freq(double freqHz, double dutyErrorFraction);`
    - Computes the allowed absolute gap (in microseconds) for a given frequency and desired duty-cycle error fraction \(\varepsilon\).
    - Uses the relation \(|p - 0.5| = |\text{gap}| / (2T)\) with \(T = 10^6/f\), giving \(|\text{gap}|_\text{max} = 2\varepsilon T\).
    - `calibrate_DCO` currently calls this with a compile-time fraction (e.g. 0.5% duty error) but it’s parameterized so different tolerances could be used later.

  - `static void store_note_result(DCOCalibrationContext& ctx, int j, uint16_t bestAmpComp, float closestToZero);`
    - Writes the final `[freq, pwm]` pair for the current note into `ctx.calibrationData`.
    - Prints a short human‑readable summary over `Serial`.

- **`calibrate_DCO(DCOCalibrationContext& ctx)`**:

  Core responsibilities are unchanged; refactor only clarifies structure:

  1. **Per‑note setup**:
     - For each `j` (table index), computes:
       - `ctx.currentNote` = start note + `calibration_note_interval * (j - 4)/2`.
       - `VOICE_NOTES[0]` updated accordingly.
     - Initial PWM guess is now computed through `compute_initial_amp_for_note(ctx, j)`:
       - `j == 4`:
         - Manual preset plus manual offset, scaled by 1.35.
       - `j == 6`:
         - Logarithmic interpolation based on `ctx.calibrationData[2..5]`.
       - Else:
         - Quadratic interpolation based on the previous three points.

  2. **Bounds & tolerance**:
     - `minAmpComp = currentAmpCompCalibrationVal * 0.8;`
     - `maxAmpComp = currentAmpCompCalibrationVal * 1.3;`
     - `tolerance = 1e6 / f² / 4`, where `f = sNotePitches[VOICE_NOTES[0] - 12]`.

  3. **Search loop** (per note):
     - Initializes:
       - `bestAmpComp`, `closestToZero`, `previousAvgValue`, neighbour arrays, `flipCounter`.
     - In each iteration:
       - Gets error with `float avgValue = measure_gap_for_amp(currentAmpCompCalibrationVal);`
       - Updates `closestToZero` / `bestAmpComp` if this is an improvement (ignoring sentinel).
       - Uses `did_sign_change(previousAvgValue, avgValue)` to detect crossing through zero:
         - If a sign change:
           - Probes neighbour PWM values (`±(i+1)` for `i < rangeSamples`) via `measure_gap_for_amp`.
           - Calls `update_best_from_neighbours` to pick the best among neighbours and current.
           - If `abs(closestToZero) <= tolerance` or flip count is high with small error, breaks.
           - Else, relaxes `tolerance` and increments `flipCounter`.
       - Calls `step_amp_from_error(avgValue, tolerance, currentAmpCompCalibrationVal)` to adjust PWM.
       - Logs if PWM leaves `[minAmpComp, maxAmpComp]`.
       - Updates `previousAvgValue`.

  4. **Store per‑note result**:
     - On exit from the loop:
       - `store_note_result(ctx, j, bestAmpComp, closestToZero);`
       - This writes the `[freq, pwm]` pair and logs the note index, best PWM, and final error.

  5. **Behavioural fix (bug clean‑up)**:
     - The original code used a confusing expression when updating `closestToZero` / `bestAmpComp`:
       - `if (abs(avgValue) < abs(closestToZero && avgValue != 1.16999f))`
     - This has been corrected to:
       - `if (avgValue != kGapTimeoutSentinel && abs(avgValue) < abs(closestToZero))`
     - This matches the intended semantics: ignore timeout readings and only update the best candidate when the new measurement is valid and closer to zero in absolute value.

- **Legacy functions** (under `#if 0`) remain documented but disabled:
  - `PID_dco_calibration`, `PID_find_highest_freq`, `find_lowest_freq`.

- **Interpolation helpers** (`quadraticInterpolation`, `logarithmicInterpolation`, etc.) are unchanged; they are still used to compute initial PWM guesses and could be reused elsewhere.

---

## Behaviour vs Original Version

- **Calibration behaviour**:
  - The numerical algorithm is intentionally preserved:
    - Same initial guesses.
    - Same step sizes.
    - Same tolerance computation and relaxation strategy.
    - Same state transitions and stopping conditions.
- **Structural improvements**:
  - Clearer separation of concerns:
    - Measurement vs search logic vs hardware I/O.
  - Explicit `DCOCalibrationContext` makes dependencies clear.
  - Reusable helpers reduce code duplication and risk of inconsistencies.
  - Legacy code is clearly marked and isolated.

---

## Suggested Future Work (Optional)

- Gradually move more state into context‑like structs to reduce dependence on globals (especially for testability).
- Introduce a minimal logging abstraction (instead of raw `Serial.println`) to control verbosity centrally.
- If you decide to keep or remove legacy PID paths, either:
  - Move them to a separate `autotune_legacy.ino`, or
  - Remove them and simplify the headers further.
- Once you’re confident in the refactor, consider writing a short **developer guide** section documenting:
  - How to run calibration on hardware.
  - What typical logs look like.
  - How to verify new changes against expected calibration tables.


