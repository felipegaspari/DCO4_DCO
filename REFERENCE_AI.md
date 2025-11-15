## DCO4_DCO Project: AI Codebase Reference

This document is a **semantic map** of the firmware for the DCO4 board (RP2040, 4 voices, 2 DCOs per voice).  
It explains what each file does and how the main subsystems (voices, modulation, calibration, storage, I/O) fit together.

---

## 1. Top-Level Sketch, Cores and Aggregated Includes

- **`DCO4_DCO.ino`**  
  - Main application for the RP2040.  
  - Runs on both cores using the Arduino dual-core API:
    - `setup()` / `loop()` (core 0): USB/serial/MIDI I/O, LFO evaluation, cross‑core detune FIFO.
    - `setup1()` / `loop1()` (core 1): PID & FS init, ADSR init, DCO calibration/autotune, real‑time voice engine.  
  - Configures USB descriptors (via Adafruit TinyUSB), toggles board pins (23/24) for hardware fixes, and selects DCO calibration mode.
  - Core‑0 pushes `DETUNE_INTERNAL_q24` (LFO1 detune) through `rp2040.fifo`; core‑1 pops and uses it inside `voice_task()`.
  - Optionally prints detailed timing statistics when `RUNNING_AVERAGE` is enabled.

- **`include_all.h`**  
  - Convenience umbrella header used by most `.ino` implementation files.  
  - Pulls in RP2040/Arduino headers and all project modules: `globals`, `fixed_types`, `FS`, `noteList`, `amp_comp`, `Serial`, `midi`, `voices`, `state_machines`, `PWM`, `utils`, `Timer_millis`, `LFO`, `adsr`, `PID`, `autotune`.

- **`globals.h`**  
  - System‑wide constants and state:
    - Voice/DCO counts: `NUM_VOICES_TOTAL = 4`, `NUM_OSCILLATORS = 8`.
    - Clock and PIO timing constants (`sysClock_Hz`, `pioPulseLength`, OSR chunk sizes, timing overheads).
    - Fixed‑point detune and pitch‑bend multipliers (`DETUNE_INTERNAL_q24`, `DETUNE_INTERNAL2_q24`, `pitchBendMultiplier_q24`).
    - Global voice arrays (`VOICE_NOTES`, `VOICES`, `note_on_flag`, `PW`, etc.).
    - Hardware pin mappings for reset, range and PW PWM pins and PIO/SM routing.
    - Shared PIO array `pio[2]`, timer variables, MIDI pitch bend state and helper prototypes.

---

## 2. Voice Architecture & Real-Time Engine

- **`voices.h`**  
  - Declares `init_voices()`, `print_voice_task_timings()` and core voice‑engine globals:
    - Portamento configuration and mode (`PORTA_MODE_TIME` / `PORTA_MODE_SLEW`).
    - Per‑DCO portamento state in **Q24 Hz** (`portamento_*_q24`) and in **Q16 semitone space** for slew‑rate mode.
    - Precomputed pitch multiplier table storage (`xMultiplierTable`, `yMultiplierTable`, `slopeQ*`, `interpSegCache`).
    - RunningAverage externs (when enabled) for fine‑grained performance profiling.

- **`voices.ino`**  
  - Central **voice engine** and DCO front‑end, fully ported to fixed‑point:
    - `init_voices()` sets initial notes, builds pitch multiplier tables (`initMultiplierTables()`), sets voice mode and runs an initial `voice_task()`.
    - `voice_task()` (hot path, called from `loop1()` on core 1):
      - For each active MIDI voice (mapped to 2 DCOs):
        - Computes per‑voice portamento in either **time‑based frequency space** or **slew‑rate note space**.
        - Combines fixed‑point modulators:
          - Pitch‑bend (`calcPitchbend_q24`).
          - LFO1 detune (`DETUNE_INTERNAL_FIFO_q24` from the FIFO).
          - Unison detune pattern.
          - Per‑DCO drift LFO (`LFO_DRIFT_LEVEL`) with analog drift amount.
          - ADSR‑to‑detune in Q24 using `ADSR1toDETUNE1_scale_q24` and `linToLogLookup`.
        - Evaluates a **precomputed pitch multiplier table** using integer interpolation (`interpolateRatioQ16_cached` or `interpolatePitchMultiplierIntQ16_cached`) to map summed modifiers to a frequency ratio.
        - Produces final DCO frequencies in **Q24 Hz**, converts to compact **Q4 Hz** for fast clock‑divider math, and calculates:
          - High‑precision total PIO cycles (optional 64‑bit division path).
          - Corrected OSR clock dividers (`clk_div1`, `clk_div2`) including OSC2 phase‑alignment support.
        - Performs **amplitude compensation** via `get_chan_level_lookup_fast()` using precomputed quadratic windows over the calibration table (`amp_comp.h`).
        - Writes new dividers and amplitude levels into the PIO state machines and range PWM channels.
        - At 99 µs intervals (`timer99microsFlag`), updates PW PWM per voice, combining ADSR1 and LFO2 modulation in integer math and using `get_PW_level_interpolated()` to map raw PW counters into calibrated duty levels.
      - Provides several reference/debug variants:
        - `voice_task_simple()` – legacy float‑based, simplified path.
        - `voice_task_debug()` – float reference/debug implementation.
        - `voice_task_gold_reference()` – high‑precision long‑double reference implementation.
    - Voice allocation helpers:
      - `get_free_voice_sequential()` and `get_free_voice()` implement round‑robin and oldest‑voice‑steal strategies for poly/stack/unison modes.
      - `setVoiceMode()` configures `NUM_VOICES` / `STACK_VOICES` for mono, 4‑voice poly and stacked modes.
      - `setSyncMode()` reconfigures PIO sideset pins to implement different oscillator sync topologies and forces re‑trigger of all voices.
    - Amplitude compensation helpers:
      - `get_chan_level_lookup_fast()` – optimized fixed‑point quadratic interpolation per DCO, using cached window indices and Q28 reciprocals.
      - `get_chan_level_lookup()` / `_float()` – legacy and fallback variants.
      - `get_PW_level_interpolated()` – maps PW counts into calibrated limits and center values.
    - Calibration front‑end:
      - `voice_task_autotune()` – dedicated per‑oscillator routine used during DCO/DCO+PW calibration to drive the PIO and PWM into specific measurement or calibration modes.
    - Timing diagnostics:
      - `print_voice_task_timings()` – prints detailed microsecond averages for each phase of `voice_task()` when `RUNNING_AVERAGE` is active.

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
  - `set_frequency()` – simple helper for test scenarios (not used in the main fixed‑point engine).

- **`PWM.h` / `PWM.ino`**  
  - `init_pwm()` configures RP2040 PWM slices for:
    - RANGE PWM (per DCO amplitude): `RANGE_PINS` mapped to `RANGE_PWM_SLICES`, `wrap = DIV_COUNTER`.
    - PW PWM (per voice pulse width): `PW_PINS` mapped to `PW_PWM_SLICES`, `wrap = DIV_COUNTER_PW`.
  - Provides the low‑level PWM targets used by both `voice_task()` and calibration routines.

---

## 4. Envelope Generator (ADSR) and Modulation (LFO & Drift)

- **`src/ADSR_Bezier/ADSR_Bezier.h` / `.cpp` and `README.md`**  
  - Self‑contained, RP2040‑friendly ADSR class (`adsr`) using **Bézier‑based curve lookup tables**.
  - Supports configurable attack/decay/release curves (8 shapes), micros‑ or millis‑based timing, and integer outputs with no float in the hot path.
  - The DCO4 project integrates this directly; see `src/ADSR_Bezier/README.md` for detailed behaviour.

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

- **`src/lfo-main/lfo.h` / `.cpp` and `README.md`**  
  - LFO class from **mo‑thunderz**, customized for RP2040:
    - Uses a 32‑bit fixed‑point phase accumulator driven by `micros()`.
    - Supports waveforms: off, saw, triangle, sine (lookup table), square.
    - Works in free‑running or BPM‑synced mode; exposes phase and amplitude control.
    - The DCO4 project uses this library as the basis for LFO1, LFO2 and per‑DCO drift LFOs.

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
  - Defines data structures and precomputation for **per‑DCO amplitude compensation**:
    - Calibration data arrays (`freq_to_amp_comp_array`, `ampCompFrequencyArray`, `ampCompArray`).
    - High‑precision float and double polynomial coefficients for offline/reference use (`aCoeff`, `bCoeff`, `cCoeff`, `aCoeffD`, …).
    - Fixed‑point per‑window quadratic model `y(t) = a*t^2 + b*t + c` with Q(T_FRAC) coefficients for run‑time evaluation.
    - Precomputed frequency bases, spans, Q28 reciprocals (`invDxWIN_q28`) and fast integer coefficient variants (`aQWIN_fast`, `bQWIN_fast`).
  - `precomputeCoefficients()`:
    - Called once in `setup1()` after reading calibration data from FS.
    - Performs table sanitization, plateau handling, and populates all fixed‑point window parameters used by `get_chan_level_lookup_fast()`.
  - `precomputeCoefficients_OLD()`:
    - Legacy precomputation path retained for reference and debugging.

- **`autotune.h` / `autotune.ino`**  
  - DCO and PW **autocalibration subsystem**:
    - Flags and state: `calibrationFlag`, `manualCalibrationFlag`, `firstTuneFlag`, `manualCalibrationStage`, offsets per oscillator, PW calibration values, note indices.
    - Calibration arrays (`calibrationData[]`) store [frequency, amplitude] pairs used to rebuild amp‑comp tables.
  - `init_DCO_calibration()`:
    - Sets initial note, PWM centre and target sample counts, clears accumulators and global flags.
    - Ensures all oscillators are temporarily muted and PW is centralized before measuring.
    - Runs `voice_task_autotune()` to feed the PIO with an initial calibration waveform.
  - `DCO_calibration()` / `VCO_calibration()`:
    - High‑level procedures that:
      - Iterate across all oscillators and notes.
      - For each oscillator, optionally find PW centre (`find_PW_center()`), then call `calibrate_DCO()` to populate `calibrationData[]`.
      - Persist data using `update_FS_voice()` and refresh amp‑comp tables with `init_FS()` and `precomputeCoefficients()`.
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
    - Reads amp‑comp bank data and reconstructs `freq_to_amp_comp_array` and per‑oscillator tables in fixed‑point Hz (`ampCompFrequencyArray`) and amplitude (`ampCompArray`).
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
    - `Serial1`: MIDI DIN input (31.25 kbps).
    - `Serial2`: high‑speed point‑to‑point link (~2.5 Mbps) to the main controller board.
    - `Serial`: USB CDC debug console.
  - Implements a **robust non‑blocking frame parser** for Serial2:
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

- **`params.ino`**  
  - Central **parameter router** (`update_parameters(paramNumber, paramValue)`) for UI/MIDI‑driven changes:
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
      - `voice_task()` to schedule PW updates.
      - `print_running_averages()` for periodic diagnostic prints.

- **`utils.h` / `utils.ino`**  
  - Small helper utilities:
    - `uintToStr()` – integer to C‑string conversion.
    - Mapping helpers from linear to logarithmic/exponential parameter curves (`linearToLogarithmic`, `linearToExponential`, `expConverter*`).
    - `controls_formula_update()` – converts raw control values into float parameters (e.g. LFO speeds, LFO1→DCO depth).
    - `led_blinking_task()` – simple LED heartbeat and activity feedback using `LED_BLINK_START`.

- **`noteList.h`**  
  - Static tables for MIDI note → frequency mapping:
    - Individual note defines (e.g. `NOTE_A4 = 440.00`) and a contiguous `sNotePitches[]` float array.
    - `sNotePitches_q24[]` – 64‑bit **Q24 fixed‑point** version of the same table, used throughout the voice engine to avoid floats.

---

## 9. USB, System Config and Metadata

- **`tusb_config.h`**  
  - TinyUSB configuration for the RP2040 USB stack (endpoints, buffer sizes, etc.), shared with Adafruit TinyUSB.

- **`usb_descriptors.c`**  
  - USB MIDI device descriptors:
    - Defines the DCO4 as a USB MIDI device with appropriate vendor/product IDs.
    - Configures interfaces and endpoints used by `Adafruit_USBD_MIDI`.

- **`irq_tuner.h` / `irq_tuner.ino`**  
  - Experimental/alternate DCO tuning method using IRQ‑based frequency measurement.  
  - Currently not active in the main flow but kept as a future reference.

- **`DCO4_DCO.code-workspace`**  
  - Project workspace configuration for VSCode/Cursor.

---

## 10. External Library Subtrees (ADSR & LFO Variants)

- **`src/ADSR_Bezier/`**  
  - Standalone ADSR Bezier library used directly by this project.
  - Contains:
    - `ADSR_Bezier.h` / `.cpp` – library implementation (see section 4).
    - `README.md` – full explanation of the envelope design and API.
    - Example sketch under `examples/ADSR_example/`.

- **`src/ADSR_Bezier_millis/`**  
  - Variant of the ADSR Bezier library using `millis()` as the internal timebase.
  - Provided as reference and for potential reuse; not used in the main DCO4 firmware.

- **`src/lfo-main/`**  
  - Base LFO library (mo‑thunderz) used by `LFO.h`:
    - `lfo.h` / `lfo.cpp` – class implementation.
    - `README.md` – detailed documentation.
    - Example sketches under `examples/`.

---

## 11. Legacy / Experimental Copies

- **`autotune_copy.h` / `autotune_copy.ino`**  
  - Earlier or experimental versions of the autotune subsystem.  
  - Kept for reference; the active implementation is `autotune.h` / `autotune.ino`.

- **`PID_copy.h` / `PID_copy.ino`**  
  - Alternate PID tuning strategies and experiments.  
  - Retained as a reference for future tuning work.

---

## 12. Conventions

- `*.h` – Declarations, constants, global state and struct/class definitions.  
- `*.ino` – Implementation files with function bodies and logic.  
- `_copy` suffix – Legacy or experimental implementations still useful as reference.  
- `src/...` – Standalone libraries (ADSR, LFO) vendored into this repo, used from project‑level headers (`adsr.h`, `LFO.h`).

---

### Summary

This firmware implements a **fixed‑point, dual‑core, RP2040‑based DCO synthesizer** with:
- A high‑precision, table‑driven voice engine (per‑voice portamento, LFOs, drift, ADSR modulation).
- Robust DCO and PWM calibration via PID and edge‑timing, persisted in LittleFS and used by a fast amplitude‑compensation layer.
- MIDI over USB and DIN, plus a high‑speed UART protocol to a main controller for parameters and UI.
- Clean separation between hot audio/control loops (fixed‑point, no divisions) and slower calibration, storage and UI code.

Use this reference to quickly locate subsystems, understand data flow, and safely extend or optimize specific parts of the DCO4 firmware.
