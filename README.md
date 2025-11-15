## DCO4 – RP2040 DCO Voice Board (4‑voice, 2 DCOs per voice)

DCO4 is a **digitally‑controlled oscillator (DCO) voice board** built around the RP2040 (Raspberry Pi Pico–class MCU).  
It implements **4 independent synth voices**, each with **2 DCOs**, envelopes, LFOs, calibration and MIDI control, and is designed to be driven by a separate main controller and UI.

The aim of the project is to create a FULLY DIGITALLY CONTROLLED ANALOG SYNTH, with patch saving for all parameters.

This repository contains the **firmware for the DCO4 voice board**. Other boards in the system are:

- A **main controller board** (brain/master).
- An **input controller** (pots, knobs, encoders).
- A **screen board** (TFT display).

Those boards talk to DCO4 over MIDI and a high‑speed UART link; this repo focuses on generating accurate, calibrated DCO waveforms.

---

## Features

- **Polyphony & Oscillators**
  - 4 voices, each with 2 DCOs (8 oscillators total).
  - Mono, poly and stacked/unison voice modes.
  - Oscillator intervals and OSC2 detune.
  - Multiple oscillator sync modes (hard sync, phase alignment, etc.).

- **Sound Shaping & Modulation**
  - Bezier‑based ADSR envelope per voice (high‑resolution, microsecond timing).
  - LFO1 and LFO2 for pitch/detune and PWM or other modulation.
  - Per‑oscillator “analog drift” LFOs to simulate warm, unstable DCOs.
  - ADSR→pitch and ADSR→PWM modulation paths.
  - Velocity, pitch bend and CC support via MIDI.

- **Calibration & Stability**
  - Per‑oscillator DCO calibration using edge‑timing and a PID controller.
  - Frequency‑dependent amplitude compensation to keep level consistent across the spectrum.
  - Automatic PW center and low‑limit calibration.
  - Calibration data stored in flash using LittleFS and reused at boot.

- **Performance & Implementation**
  - Runs on both RP2040 cores:
    - Core 0: MIDI and serial I/O, LFO evaluation, cross‑core detune FIFO.
    - Core 1: envelopes, timers, calibration routines and the real‑time voice/DCO engine.
  - Heavy real‑time math implemented in **fixed‑point** (no float in the hot path).
  - PIO‑based DCO pulse generation with per‑oscillator clock dividers and phase control.

---

## High‑Level Architecture

At a very high level, the firmware is split into a few major subsystems:

- **Voice engine (`voices.*`)**
  - Converts MIDI note events into per‑DCO frequencies, including portamento, unison, pitch‑bend, LFO and ADSR modulation.
  - Computes PIO clock dividers and range PWM levels, and writes them to the PIO state machines and PWM slices.

- **Modulation (`adsr.*`, `src/ADSR_Bezier`, `LFO.*`, `src/lfo-main`)**
  - ADSR envelopes are implemented via a Bezier‑based lookup‑table engine (per‑voice instances).
  - LFOs are based on mo‑thunderz’ Arduino LFO class, adapted for RP2040 and used for:
    - LFO1 (pitch/detune),
    - LFO2 (PWM/other),
    - per‑oscillator drift.

- **Calibration (`autotune.*`, `amp_comp.*`, `PID.*`, `FS.*`)**
  - Autotune module measures DCO duty cycle and frequency at a calibration pin while sweeping amplitudes/PW.
  - Builds amplitude‑compensation tables per oscillator, precomputes quadratic windows and stores them in flash.
  - PID routines drive the calibration search, and `FS.*` handles LittleFS read/write of all calibration banks.

- **I/O (`midi.*`, `Serial.*`, `params.ino`)**
  - USB and DIN MIDI via Adafruit TinyUSB + `MIDI.h`.
  - High‑speed Serial2 link to the main controller with a simple binary protocol:
    - Sends note on/off events,
    - Receives parameter updates and ADSR/LFO settings.
  - `params.ino` converts those incoming values into the internal fixed‑point parameters used by the engine.

For a **deep, file‑by‑file technical map** (intended for developers and AI agents), see `REFERENCE_AI.md`.

---

## Hardware Overview

The firmware targets an RP2040 board wired roughly as follows:

- 8 DCO outputs driven by PIO state machines (frequency and phase controlled by clock dividers).
- Per‑DCO **range PWM** outputs to set amplitude based on calibration.
- Per‑voice **PW PWM** outputs to modulate pulse width.
- A dedicated **calibration input pin** to measure DCO pulses during autotune.
- UARTs:
  - `Serial1`: 5‑pin DIN MIDI input.
  - `Serial2`: high‑speed link to the main controller/UI.
- USB: MIDI over USB for easy testing and integration with a DAW or MIDI host.

Exact pin mappings and hardware constants are defined in `globals.h`.

---

## Building and Flashing

The project is written as a standard Arduino sketch plus additional `.ino`/`.h` files.

### 1. Prerequisites

- **Toolchain**
  - Arduino IDE or another environment that supports the RP2040 Arduino core (e.g. Earle Philhower’s core).

- **Board support**
  - Install the RP2040 / Raspberry Pi Pico board package for Arduino.
  - Select the correct board (e.g. *Raspberry Pi Pico* or your WEACT RP2040 variant).

- **Libraries** (all available via Arduino Library Manager or as standard dependencies)
  - Adafruit TinyUSB for RP2040 (`Adafruit_TinyUSB`).
  - MIDI library (`MIDI.h`, by FortySevenEffects).
  - PID controller (`PID_v1`).
  - LittleFS support (provided by the RP2040 core).
  - The ADSR and LFO libraries are included under `src/` and used via `adsr.h` / `LFO.h`.

### 2. Build steps

1. Open `DCO4_DCO.ino` in the Arduino IDE (or configure your alternative build system to use it as the main sketch).
2. Select the appropriate RP2040 board and serial port.
3. Ensure the required libraries are installed.
4. Compile and upload the sketch to the DCO4 board.

Once flashed, the board will enumerate as a USB MIDI device and will also listen on DIN MIDI / Serial2 as wired.

---

## Calibration Workflow (Overview)

Calibration is normally triggered from the main controller / UI via the high‑speed Serial2 protocol.  
At a high level, the flow is:

1. The controller sets the appropriate flags/parameters (e.g. “start calibration”).
2. DCO4 runs through:
   - Per‑DCO amplitude calibration across multiple notes.
   - PW center and (optionally) low‑limit finding.
3. Calibration data is written to LittleFS (`voiceTables`, `PWCenter`, `PWLowLimit`, etc.).
4. On the next boot, `init_FS()` loads those tables and `precomputeCoefficients()` prepares fast lookup structures for the main engine.

Because the calibration routines directly control the oscillators and PWM outputs and can take a while, they are not normally run at every power‑up – the stored data is reused.

---

## Repository Layout (Quick Guide)

- **`DCO4_DCO.ino`** – main sketch, core‑0/1 setup and loops.
- **`REFERENCE_AI.md`** – detailed, file‑by‑file technical reference (for developers/AI).
- **`voices.*`** – polyphony, voice engine and DCO clock‑divider logic.
- **`adsr.*`, `src/ADSR_Bezier/`** – ADSR envelope integration.
- **`LFO.*`, `src/lfo-main/`** – LFO and drift modulation.
- **`autotune.*`, `amp_comp.*`, `PID.*`, `FS.*`** – calibration and amplitude compensation.
- **`midi.*`, `Serial.*`, `params.ino`** – MIDI, serial protocols, parameter updates.
- **`globals.h`, `state_machines.*`, `PWM.*`, `Timer_millis.*`, `noteList.h`, `utils.*`** – core infrastructure, timing and helpers.

---

## Contributing / Hacking

- Start by reading `REFERENCE_AI.md` to understand how the subsystems interconnect.
- When modifying the real‑time engine (`voices.*`), keep:
  - Fixed‑point formats consistent.
  - The hot path free of floating‑point divisions.
- For new modulation paths or parameters, route them through:
  - `params.ino` (for serial/MIDI control),
  - appropriate globals in `globals.h`,
  - and the relevant engine module (`voices`, `adsr`, `LFO`, etc.).

Feel free to adapt this firmware to other RP2040‑based synth projects; the voice engine, ADSR and LFO layers are quite modular and can be reused with different front‑ends or UI controllers.


