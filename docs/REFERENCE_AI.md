## DCO4_DCO Project: AI Codebase Reference

This document is a **semantic map** of the firmware for the DCO4 board (RP2040 / RP2350-class, 4 voices, 2 DCOs per voice).  
It explains what each file does and how the main subsystems (voices, modulation, calibration, storage, I/O) fit together.

Related docs:
- Flat file + function + call-site inventory: [`FILE_INDEX.md`](FILE_INDEX.md) (same `docs/` folder)
- System topology (other boards): [`SYSTEM_OVERVIEW.md`](SYSTEM_OVERVIEW.md)
- Float vs fixed build flags and precision modes: [`ENGINE_OPTIONS.md`](ENGINE_OPTIONS.md)
- Autotune algorithms / refactor layout: [`AUTOTUNE.md`](AUTOTUNE.md), [`AUTOTUNE_REFACTORED.md`](AUTOTUNE_REFACTORED.md)
- Repo entry point: [`../README.md`](../README.md)

---

## 1. Top-Level Sketch, Cores and Aggregated Includes

- **`DCO4_DCO.ino`**  
  - Main application for the RP2040 / RP2350.  
  - Runs on both cores using the Arduino dual-core API:
    - `setup()` / `loop()` (core 0): USB/serial/MIDI I/O, LFO evaluation, cross‑core detune FIFO.
    - `setup1()` / `loop1()` (core 1): PID & FS init, ADSR init, DCO calibration/autotune, real‑time voice engine.  
  - **Engine build options** (top of file, before includes):
    - `USE_FLOAT_ENGINE` (current default **ON**) → defines `USE_FLOAT_VOICE_TASK` and `USE_FLOAT_AMP_COMP`.
    - Fixed-path knobs (active when float voice task is off): `PITCH_USE_RATIO_Q16`, `PITCH_INTERP_USE_Q12` / `Q8`, `HIGH_PRECISION_CLKDIV`.
    - See [`ENGINE_OPTIONS.md`](ENGINE_OPTIONS.md) for precision vs speed trade-offs.
  - Configures USB product strings in `setup()` (via Adafruit TinyUSB), toggles board pins (23/24) for hardware fixes, and selects DCO calibration mode.
  - Core‑0 pushes `DETUNE_INTERNAL_q24` (LFO1 detune) through `rp2040.fifo`; core‑1 pops it and uses it inside the voice task (float path converts Q24 → float each frame).
  - `loop1()` calls `voice_task_main()` (dispatch to float or fixed), then `bench_service(1)` to hand profiler counters to core 0.
  - **Profiling** (`RUNNING_AVERAGE`, optionally `RUNNING_AVERAGE_FINE`): both loops bracketed by `BENCH_*` probes from [`bench.h`](../bench.h); core 0 prints via `bench_poll_core0()` at ~1 Hz by default. See [`BENCHMARKING.md`](BENCHMARKING.md).

- **`include_all.h`**  
  - Convenience umbrella header used by most `.ino` implementation files.  
  - Pulls in RP2040/Arduino headers and project modules: `params_def`, `param_router`, `globals`, `FS`, `noteList`, `amp_comp`, `Serial`, `midi`, `voices`, `state_machines`, `PWM`, `utils`, `Timer_millis`, `LFO`, `adsr`, `PID`, `autotune`.

- **`globals.h`**  
  - System‑wide constants and state:
    - Voice/DCO counts: `NUM_VOICES_TOTAL = 4`, `NUM_OSCILLATORS = 8`.
    - Clock and PIO timing constants (`sysClock` = 225000 kHz → `sysClock_Hz`, `pioPulseLength`, OSR chunk sizes, timing overheads).
    - Fixed‑point detune and pitch‑bend multipliers (`DETUNE_INTERNAL_q24`, `DETUNE_INTERNAL2_q24`, `pitchBendMultiplier_q24`).
    - Global voice arrays (`VOICE_NOTES`, `VOICES`, `note_on_flag`, `PW`, etc.).
    - Hardware pin mappings for reset, range and PW PWM pins and PIO/SM routing (active map: **WEACT**; Pico variant commented).
    - `DCO_calibration_pin = 10`; `ENABLE_FS_CALIBRATION`.
    - Shared PIO array `pio[2]`, timer variables, MIDI pitch bend state and helper prototypes.

---

## 2. Voice Architecture & Real-Time Engine

- **`voices.h`**  
  - Declares `init_voices()` and core voice‑engine globals:
    - Portamento configuration and mode (`PORTA_MODE_TIME` / `PORTA_MODE_SLEW`).
    - Per‑DCO portamento state in **Q24 Hz** (`portamento_*_q24`) and in **Q16 semitone space** for slew‑rate mode.
    - When `USE_FLOAT_VOICE_TASK`: parallel float portamento state (`porta_*_f`) in Hz and semitone domains.
    - Precomputed pitch multiplier table storage (`xMultiplierTable`, `yMultiplierTable`, float mirrors `xMultiplierTableF` / `yMultiplierTableF`, `slopeQ*` / `slopeF`, `interpSegCache`).

- **`voices.ino`**  
  - Central **voice engine** and DCO front‑end with a **compile-time dual implementation**:
    - `init_voices()` sets initial notes, builds pitch multiplier tables (`initMultiplierTables()` — integer and float mirrors), sets voice mode and runs an initial `voice_task_main()`.
    - `voice_task_main()` → `voice_task_float()` **or** `voice_task()` depending on `USE_FLOAT_VOICE_TASK`.

    - **`voice_task()`** (fixed hot path, when `!USE_FLOAT_VOICE_TASK`):
      - For each active MIDI voice (mapped to 2 DCOs):
        - Computes per‑voice portamento in either **time‑based frequency space** or **slew‑rate note space** (Q24 / Q16).
        - Combines fixed‑point modulators:
          - Pitch‑bend (`calcPitchbend_q24`).
          - LFO1 detune (`DETUNE_INTERNAL_FIFO_q24` from the FIFO).
          - Unison detune pattern.
          - Per‑DCO drift LFO (`LFO_DRIFT_LEVEL`) with analog drift amount.
          - ADSR‑to‑detune in Q24 using `ADSR1toDETUNE1_scale_q24` and `linToLogLookup`.
        - Evaluates a **precomputed pitch multiplier table** using integer interpolation (`interpolateRatioQ16_cached` or `interpolatePitchMultiplierIntQ16_cached` + Q8/Q12/Q20 slope modes) to map summed modifiers to a frequency ratio. Flag behaviour: [`ENGINE_OPTIONS.md`](ENGINE_OPTIONS.md).
        - Produces final DCO frequencies in **Q24 Hz**, then clock‑dividers via `HIGH_PRECISION_CLKDIV`:
          - **1**: 64‑bit divide on full Q24 Hz (~4 µs/voice, preferred low‑note accuracy).
          - **0**: compact **Q4 Hz** then 32‑bit divide (~1 µs/voice).
          - Corrected OSR clock dividers (`clk_div1`, `clk_div2`) including OSC2 phase‑alignment support.
        - Performs **amplitude compensation** via `get_chan_level_lookup_fast()` (Q8 Hz domain) using precomputed quadratic windows (`amp_comp.h`).
        - Writes new dividers and amplitude levels into the PIO state machines and range PWM channels.
        - At 99 µs intervals (`timer99microsFlag`), updates PW PWM per voice, combining ADSR1 and LFO2 modulation in integer math and using `get_PW_level_interpolated()` to map raw PW counters into calibrated duty levels.

    - **`voice_task_float()`** (float hot path, when `USE_FLOAT_VOICE_TASK` — **current default**):
      - Same overall structure (portamento → modifiers → ratio → clkdiv → amp → PIO/PWM/PW), but in **Hz / float**:
        - Float portamento state; pitch bend / LFO / ADSR / drift converted from Q24 globals where needed.
        - `interpolateRatioFloat_cached` (requires `PITCH_USE_RATIO_Q16`; the `#else` stub does not compile).
        - Clkdiv always `sysClock_Hz / freqHz` in float (`HIGH_PRECISION_CLKDIV` ignored).
        - Amp via `get_chan_level_for_engine()` → float or fixed facade depending on `USE_FLOAT_AMP_COMP`.
      - Details: [`ENGINE_OPTIONS.md`](ENGINE_OPTIONS.md) §6.

    - Reference/debug variants (not called from `loop1`):
      - `voice_task_simple()` – legacy float‑based, simplified path.
      - `voice_task_debug()` – float reference/debug implementation.
      - `voice_task_gold_reference()` – **removed**; older docs may still mention it.
    - Voice allocation helpers:
      - `get_free_voice_sequential()` and `get_free_voice()` implement round‑robin and oldest‑voice‑steal strategies for poly/stack/unison modes.
      - `setVoiceMode()` configures `NUM_VOICES` / `STACK_VOICES` for mono, 4‑voice poly and stacked modes.
      - `setSyncMode()` reconfigures PIO sideset pins to implement different oscillator sync topologies and forces re‑trigger of all voices.
    - Amplitude compensation helpers:
      - `get_chan_level_lookup_fast()` – optimized fixed‑point quadratic interpolation per DCO (when `!USE_FLOAT_AMP_COMP`), using cached window indices and Q28 reciprocals.
      - `get_chan_level_float()` / `get_chan_level_for_engine()` – float Hz path and engine-agnostic facade.
      - `get_PW_level_interpolated()` – maps PW counts into calibrated limits and center values (shared by both engines).
    - Calibration front‑end:
      - `voice_task_autotune()` – dedicated per‑oscillator routine used during DCO/DCO+PW calibration to drive the PIO and PWM into specific measurement or calibration modes (float-style clkdiv + `get_chan_level_for_engine`).
    - Timing diagnostics: `BENCH_*` probes in `voice_task()` / `voice_task_float()`; report formatting in [`bench.h`](../bench.h). See [`BENCHMARKING.md`](BENCHMARKING.md).

---

## 3. Oscillator, PIO State Machines and PWM

- **`pico-dco.pio` / `pico-dco.pio.h`**  
  - PIO programs that generate the actual DCO rectangular wave trains with support for:
    - High and low periods controlled via OSR loads.
    - Optional oscillator sync (reset / phase‑aligned modes).
  - `frequency_sync_4_jumps_program` is the primary program used for production.

- **`state_machines.h` / `state_machines.ino`**  
  - `init_pio()` loads the PIO program into both PIO blocks and calls `start_voice_sms()`.
  - `start_voice_sms()`:
    - For each DCO, chooses the sideset pin (reset/sync pin) based on `syncMode`.
    - Calls `init_sm_sync()` to configure each state machine using the PIO program and the appropriate reset/sideset pins.
    - Preloads each SM with `pioPulseLength` and writes it to `pio_y` as the fixed high‑time base.
  - `set_frequency()` – simple helper for test scenarios (not used in the main voice engine).

- **`PWM.h` / `PWM.ino`**  
  - `init_pwm()` configures RP2040 PWM slices for:
    - RANGE PWM (per DCO amplitude): `RANGE_PINS` mapped to `RANGE_PWM_SLICES`, `wrap = DIV_COUNTER`.
    - PW PWM (per voice pulse width): `PW_PINS` mapped to `PW_PWM_SLICES`, `wrap = DIV_COUNTER_PW`.
  - Provides the low‑level PWM targets used by both the voice task and calibration routines.

---

## 4. Envelope Generator (ADSR) and Modulation (LFO & Drift)

- **External library: `ADSR_Bezier`** (`#include <ADSR_Bezier.h>`)  
  - Installed as an Arduino library (not vendored under `src/` in this repo).  
  - RP2040‑friendly ADSR class (`adsr`) using **Bézier‑based curve lookup tables**.
  - Supports configurable attack/decay/release curves (8 shapes), micros‑ or millis‑based timing, and integer outputs with no float in the envelope hot path.

- **`adsr.h` / `adsr.ino`**  
  - DCO4‑specific wiring of the ADSR Bezier library:
    - Defines the main ADSR resolution (`ADSR_1_DACSIZE = 4000`, `ARRAY_SIZE = 512`) and log/exp lookup tables (`linToLogLookup`).
    - Instantiates one `adsr` object per voice (`adsr1_voice_0 .. 3`) and wraps them in `ADSRVoices[]`.
    - Global ADSR parameters: `ADSR1_attack`, `ADSR1_decay`, `ADSR1_sustain`, `ADSR1_release`, curve parameters, restart flag and modulation depths (`ADSR1toDETUNE1`, `ADSR1toPWM`), including precomputed Q24 scale `ADSR1toDETUNE1_scale_q24`.
  - `init_ADSR()`:
    - Generates Bézier tables via `adsrBezierInitTables()`.
    - Fills `linToLogLookup` using `linearToLogarithmic()`.
    - Applies initial A/D/S/R and restart settings to all voices.
  - `ADSR_update()`:
    - Called from `loop1()` at ~10 kHz (conditioned by `Timer_millis` flags).
    - Processes `noteStart[]` / `noteEnd[]` flags, triggers `noteOn()` / `noteOff()` on per‑voice ADSRs.
    - Updates `ADSR1Level[i]` for every active voice (used by the voice engine for amplitude and detune modulation).
    - Calls `ADSR_set_parameters()` to lazily propagate parameter changes.
  - `ADSR_set_parameters()`:
    - Debounces A/D/S/R changes (checks against last values every 5 ms).
    - Efficiently re‑applies updates only to parameters that changed, across all voices.
  - Helper functions:
    - `ADSR1_set_restart()` – toggles legato vs per‑trigger behaviour for all voices.
    - `ADSR1_change_curves()` – re‑applies timing and restart settings after curve changes (hook point for future curve editing).

- **External library: `mo-lfo`** (`#include <mo-lfo.h>`)  
  - LFO class from **mo‑thunderz**, installed as an Arduino library (not under `src/`).
  - Uses a 32‑bit fixed‑point phase accumulator driven by `micros()`.
  - Supports waveforms: off, saw, triangle, sine (lookup table), square.
  - Works in free‑running or BPM‑synced mode; exposes phase and amplitude control.
  - Used as the basis for LFO1, LFO2 and per‑DCO drift LFOs.

- **`LFO.h` / `LFO.ino`**  
  - DCO4‑specific LFO layer around the library:
    - Creates global LFO instances:
      - `LFO1_class` – main detune LFO (high‑resolution CC range).
      - `LFO2_class` – secondary LFO (e.g. PW modulation, OSC2 detune).
      - `LFO_DRIFT_CLASS[8]` – per‑oscillator drift LFOs.
    - Defines modulation ranges and scaling constants (`LFO1_CC`, `LFO2_CC`, `LFO_DRIFT_CC`, etc.).
    - Exposes globals for UI and parameter mapping: `LFO1Level`, `LFO2Level`, waveforms, speeds, and modulation depths (`LFO1toDCO_q24`, `LFO2toDETUNE2_q24`, `LFO2toPW`, etc.).
  - `init_LFOs()` / `init_LFO1()` / `init_LFO2()`:
    - Configure waveforms, amplitudes, offsets and initial frequencies for the two main LFOs.
  - `init_DRIFT_LFOs()` / `init_DRIFT_LFO()`:
    - Initialize drift LFOs with per‑oscillator speed offsets (spread factor) using `expConverterFloat()`.
  - `LFO1()` / `LFO2()`:
    - Called from `loop()` on core 0.
    - Compute bipolar LFO outputs and convert detune modulation amounts directly to Q24 fixed‑point (`DETUNE_INTERNAL_q24`, `DETUNE_INTERNAL2_q24`).
  - `DRIFT_LFOs()`:
    - Updates `LFO_DRIFT_LEVEL[i]` for every DCO using a shared timestamp, producing per‑oscillator slow drift signals.

---

## 5. Tuning, Calibration & Amplitude Compensation

- **`amp_comp.h`**  
  - Defines data structures and precomputation for **per‑DCO amplitude compensation**, with a dual runtime engine:
    - Shared: `freq_to_amp_comp_array`, plateau metadata, float coeffs `aCoeff` / `bCoeff` / `cCoeff`, `AMP_COMP_MAX_HZ = 7000`.
    - **Fixed** (`!USE_FLOAT_AMP_COMP`): `ampCompFrequencyArray` in **Q8 Hz** (`FREQ_FRAC_BITS = 8`), `ampCompArray` as `int32_t`, per‑window model `y(t) = a*t^2 + b*t + c` with `T_FRAC = 12`, `invDxWIN_q28`, `aQWIN_fast` / `bQWIN_fast`.
    - **Float** (`USE_FLOAT_AMP_COMP`): `ampCompFrequencyHz`, `ampCompArray` as `uint16_t`, runtime `y = (a*x+b)*x+c` in Hz.
  - `precomputeCoefficients()` / `precomputeCoefficients_float()` — selected by `precompute_amp_comp_for_engine()` after FS load in `setup1()`.
  - `precomputeCoefficients_OLD()` — legacy precomputation path retained for reference and debugging.
  - Flag / format details: [`ENGINE_OPTIONS.md`](ENGINE_OPTIONS.md) §7.

- **`autotune.h` / `autotune.ino`** (+ helper headers)  
  - DCO and PW **autocalibration subsystem**:
    - Flags and state: `calibrationFlag`, `manualCalibrationFlag`, `firstTuneFlag`, `manualCalibrationStage`, offsets per oscillator, PW calibration values, note indices.
    - Calibration arrays (`calibrationData[]`) store [frequency, amplitude] pairs used to rebuild amp‑comp tables.
  - Included helpers (see [`AUTOTUNE_REFACTORED.md`](AUTOTUNE_REFACTORED.md)):
    - **`autotune_constants.h`** — shared constants / sizes.
    - **`autotune_context.h`** — `DCOCalibrationContext` grouping for `calibrate_DCO`.
    - **`autotune_measurement.h`** — structured `GapMeasurement` wrappers around edge timing.
  - `init_DCO_calibration()`:
    - Sets initial note, PWM centre and target sample counts, clears accumulators and global flags.
    - Ensures all oscillators are temporarily muted and PW is centralized before measuring.
    - Runs `voice_task_autotune()` to feed the PIO with an initial calibration waveform.
  - `DCO_calibration()` / `VCO_calibration()`:
    - High‑level procedures that:
      - Iterate across all oscillators and notes.
      - For each oscillator, optionally find PW centre (`find_PW_center()`), then call `calibrate_DCO()` to populate `calibrationData[]`.
      - Persist data using `update_FS_voice()` and refresh amp‑comp tables with `init_FS()` and `precompute_amp_comp_for_engine()`.
  - `restart_DCO_calibration()`:
    - Reset calibration state, PWM levels and measurement accumulators between oscillators.
  - `find_PW_center()` / `find_PW_low_limit()`:
    - Step PW until the measured duty cycle gap around 50% (or low limit) is within a target tolerance using `find_gap()`.
    - Persist PW calibration values into LittleFS via `update_FS_PWCenter()` / `update_FS_PW_Low_Limit()`.
  - `find_gap()` / `DCO_calibration_find_highest_freq()` / `DCO_calibration_debug()` / `VCO_measure_frequency()`:
    - Edge‑timing routines that measure the DCO duty cycle or frequency at the calibration pin, using pulse timing over multiple cycles.
    - Provide raw error values (`DCO_calibration_difference`) used by PID, search routines or calibration heuristics.
  - `calibrate_DCO()` and interpolation helpers (`quadraticInterpolation`, `exponentialInterpolation`, `logarithmicInterpolation*`, `linearInterpolation`, `expInterpolationSolveY()`):
    - Use a mix of polynomial, exponential and logarithmic interpolation to derive good amplitude starting points between measured calibration anchors.

- **`PID.h` / `PID.ino`**  
  - Wraps the `PID_v1` Arduino library for use in calibration and frequency search:
    - Defines PID terms (three Kp/Ki/Kd presets), gap tracking, output limits and helper variables.
    - `init_PID()` initializes PID state and setpoint.
  - `PID_dco_calibration()`:
    - Main PID‑driven DCO calibration loop:
      - Uses `find_gap()` to measure duty‑cycle errors.
      - Adjusts `ampCompCalibrationVal` until the gap is below `PIDMinGap`, tracking best candidates and detecting oscillation (“flip”) conditions.
      - When calibrated for a note, stores the result, advances to the next note, recomputes min gap and limits, and triggers new `voice_task_autotune()` runs.
  - `PID_find_highest_freq()`, `find_highest_freq()`, `find_lowest_freq()`:
    - PID‑based search helpers used in some calibration modes to identify highest/lowest usable frequencies per DCO.

---

## 6. Storage & State Persistence (LittleFS)

- **`FS.h` / `FS.ino`**  
  - Encapsulates **LittleFS‑based persistent storage** for:
    - DCO amp‑comp tables (`voiceTables` file).
    - PW centre and limit values (`PWCenter`, `PWHighLimit`, `PWLowLimit` files).
  - `init_FS()`:
    - Mounts LittleFS and opens/creates calibration files.
    - Reads amp‑comp bank data from flash (`freq_x100` format) and reconstructs either:
      - Fixed engine: `ampCompFrequencyArray` (Q8 Hz) + `ampCompArray` (`int32_t`), or
      - Float engine: `ampCompFrequencyHz` + `ampCompArray` (`uint16_t`),
      depending on `USE_FLOAT_AMP_COMP`.
    - Loads PW calibration values into `PW_CENTER` and `PW_LOW_LIMIT`.
  - `update_FS_voice()`:
    - Writes a single oscillator’s calibration slice (`calibrationData[]`) back to `voiceTables` in binary form.
  - `update_FS_PWCenter()` / `update_FS_PW_High_Limit()` / `update_FS_PW_Low_Limit()`:
    - Update PW centre and limit values for a given voice in their corresponding files.

---

## 7. MIDI, Serial Protocols and Parameter Updates

- **`midi.h` / `midi.ino`**  
  - Uses Adafruit TinyUSB MIDI and `MIDI.h` to expose both USB and DIN MIDI inputs:
    - `Adafruit_USBD_MIDI usb_midi` + `MIDI_USB` instance.
    - `MIDI_SERIAL` instance bound to `Serial1`.
  - `init_midi()` registers handlers for note on/off, CC, program change and pitch bend for both ports.
  - Handlers:
    - `handleNoteOn()` / `handleNoteOff()` forward events to the internal `note_on()` / `note_off()` functions (voice allocator).
    - `handleControlChange()` uses CC 42 to adjust pitch‑bend range and recompute `pitchBendMultiplier_q24`.
    - `handlePitchBend()` updates `midi_pitch_bend` in globals.
  - `note_on()` / `note_off()`:
    - Implement voice allocation based on `voiceMode` and `polyMode`:
      - Mono, polyphonic, and stacked/unison modes, including voice reuse when already playing a note.
    - For every assigned voice:
      - Update `VOICE_NOTES[]`, `VOICES[]`, trigger `note_on_flag[]`, `noteStart[]` / `noteEnd[]` and notify the external controller via `serial_send_note_on()` / `serial_send_note_off()`.

- **`Serial.h` / `Serial.ino`**  
  - Configures UARTs:
    - `Serial1`: MIDI DIN input — RX **1** / TX **0** @ 31.25 kbps.
    - `Serial2`: high‑speed link to the mainboard — RX **21** / TX **20** @ ~2.5 Mbps.
    - `Serial`: USB CDC debug console.
  - Implements a **robust non‑blocking frame parser** for Serial2 (`serial_parser.h`):
    - Commands:  
      - `'f'` – 16‑bit PW value (LE) → `PW[0]`.  
      - `'s'` – 4×16‑bit ADSR parameters (BE) → `ADSR1_attack/decay/sustain/release`.  
      - `'p'` – paramNumber + 16‑bit value (BE) → `update_parameters()`.  
      - `'w'` – paramNumber + signed 8‑bit value → `update_parameters()` (sign‑extended).  
      - `'x'` – paramNumber + 32‑bit value (LE) → `update_parameters()` (e.g. debug/monitoring params).  
    - Uses timeouts to discard partial frames and recover gracefully.
  - Outgoing helpers:
    - `serial_STM32_task()` – main parser pump (called from `loop()`).
    - `serial_send_note_on()` / `serial_send_note_off()` – send compact note events to the main controller.
    - `serialSendParam32()` – send 32‑bit parameter/debug values.
    - Additional debug/monitoring helpers (some commented).
  - Shared headers: `serial_protocol.h`, `serial_param_protocol.h`, `serial_parser.h`. How-to: [`README_serial_and_params.md`](README_serial_and_params.md).  
    **`serial_input_protocol.h` is not in this repo** (Mainboard / Input only).

- **`params_def.h` / `param_router.h` / `params.ino`**  
  - Canonical `ParamId` enum and table‑driven router.
  - Central **parameter apply** (`update_parameters(paramNumber, paramValue)`) for UI/MIDI‑driven changes:
    - Oscillator configuration (wave on/off, intervals, OSC2 detune, sync modes).
    - LFO settings (waveforms, speeds, routing depths, drift spread/speed).
    - Voice/stack mode, unison detune, analog drift amount.
    - Portamento time and mode (time‑based vs slew‑rate) – updates `portamento_time` and `portamento_mode`.
    - ADSR mods (ADSR1→detune, ADSR1→PWM) with precomputed fixed‑point scales (`ADSR1toDETUNE1_scale_q24`).
    - Calibration control flags (`calibrationFlag`, `manualCalibrationFlag`, stages, offsets).
  - Converts raw UI values into:
    - Exponential or logarithmic curves using `expConverter*()` helpers.
    - Fixed‑point Q24 modulation depths (`LFO1toDCO_q24`, `LFO2toDETUNE2_q24`).
    - Per‑oscillator drift speed offsets and LFO frequency updates.

---

## 8. Timing, Utility Functions and Note Tables

- **`Timer_millis.h` / `Timer_millis.ino`**  
  - Provides a set of millisecond and microsecond timers:
    - 99 µs, 223 µs, ~1 ms, 200 ms, 1000 ms (others commented out).
  - `millisTimer()`:
    - Resets and updates flag variables (`timer99microsFlag`, `timer200msFlag`, `timer1000msFlag`, etc.) used by:
      - `loop1()` (core 1) to rate‑limit ADSR updates.
      - The voice task to schedule PW updates.
      - `bench_poll_core0()` for ~1 Hz profiler dumps (when `RUNNING_AVERAGE` is on).

- **`utils.h` / `utils.ino`**  
  - Small helper utilities:
    - `uintToStr()` – integer to C‑string conversion.
    - Mapping helpers from linear to logarithmic/exponential parameter curves (`linearToLogarithmic`, `linearToExponential`, `expConverter*`).
    - `controls_formula_update()` – converts raw control values into float parameters (e.g. LFO speeds, LFO1→DCO depth).
    - `led_blinking_task()` – simple LED heartbeat and activity feedback using `LED_BLINK_START`.

- **`noteList.h`**  
  - Static tables for MIDI note → frequency mapping:
    - Individual note defines (e.g. `NOTE_A4 = 440.00`) and a contiguous `sNotePitches[]` float array (float engine / helpers).
    - `sNotePitches_q24[]` – 64‑bit **Q24 fixed‑point** version of the same table (fixed engine).

---

## 9. USB, System Config and Metadata

- **`tusb_config.h`**  
  - TinyUSB configuration for the RP2040 USB stack (endpoints, buffer sizes, etc.), shared with Adafruit TinyUSB.

- **`usb_descriptors.c`**  
  - USB MIDI device descriptors (much of the file is commented / legacy).  
  - Product identity is also set from `setup()` (“FELA” / “DCO-4” style strings via TinyUSB APIs).

- **`irq_tuner.h` / `irq_tuner.ino`**  
  - Experimental/alternate DCO tuning method using IRQ‑based frequency measurement.  
  - Currently not active in the main flow but kept as a future reference.

- **`DCO4_DCO.code-workspace`**  
  - Project workspace configuration for VSCode/Cursor (if present in the working tree).

---

## 10. External Libraries (not in-repo)

These are **Arduino libraries** installed in the IDE / sketchbook `libraries` folder, not under a repo `src/` tree:

- **`ADSR_Bezier`** — Bézier ADSR used by `adsr.*` (see section 4).
- **`mo-lfo`** — LFO class used by `LFO.*` (see section 4).
- **`PID_v1`**, **Adafruit TinyUSB**, **MIDI** (FortySevenEffects), **LittleFS** (core).

---

## 11. Conventions

- `*.h` – Declarations, constants, global state and struct/class definitions.  
- `*.ino` – Implementation files with function bodies and logic.  
- Engine math flags – edit only the block at the top of `DCO4_DCO.ino`; see [`ENGINE_OPTIONS.md`](ENGINE_OPTIONS.md).  
- Shared serial/param headers – keep `ParamId` numbers stable across boards; see [`README_serial_and_params.md`](README_serial_and_params.md).

---

### Summary

This firmware implements a **dual‑core DCO synthesizer** (RP2040 / RP2350-class) with:
- A compile-time **float or fixed-point** voice engine (float is the current default) and matching amplitude-compensation paths — see [`ENGINE_OPTIONS.md`](ENGINE_OPTIONS.md).
- A table‑driven pitch path (per‑voice portamento, LFOs, drift, ADSR modulation) feeding PIO clock dividers and range/PW PWM.
- Robust DCO and PWM calibration via edge‑timing (and optional PID), persisted in LittleFS.
- MIDI over USB and DIN, plus a high‑speed UART protocol to a main controller for parameters and UI.
- Clean separation between the hot voice/control loops and slower calibration, storage and UI-facing code.

Use this reference to quickly locate subsystems, understand data flow, and safely extend or optimize specific parts of the DCO4 firmware.
