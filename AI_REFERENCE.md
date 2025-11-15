# DCO4_DCO Project Reference

This reference describes the structure and purpose of each file in the DCO4_DCO project, as analyzed for AI assistant or developer reference.

---

## Main Application & Entry Point

### DCO4_4.ino
- **Main project file:** Initialization and main loops for all hardware, MIDI, LFOs, system state, and synth logic.
- Calls setup/init functions for USB, MIDI, LFO, voice, PIO, tuning, and invokes real-time synthesizer state machines.

---

## Core Synth Components

### voices.h / voices.ino
- **Voice management:** Handles multiple synth voices, each with two oscillators.
- Allocates/activates voices, calculates oscillator frequencies and PWM outputs, supports portamento (glide), and voice allocation policy.
- Includes autotuning logic for each voice.

### adsr.h / adsr.ino
- **Envelope Control (ADSR):** Declares and manages Attack, Decay, Sustain, Release envelopes for each voice.
- Envelopes shape note amplitude and/or pitch over time for expressivity.

### LFO.h / LFO.ino
- **LFO Modulation:** Low-frequency oscillator code for modulation (vibrato, tremolo, pitch, filter).
- Initializes LFO waveform/params and makes LFO results available to be routed through the synth engine.

### amp_comp.h
- **Amplitude Compensation:** Contains calibration tables/data for adjusting oscillator amplitude as a function of frequency to linearize volume across pitch range.

---

## Calibration and Tuning

### autotune.h / autotune.ino
- **Digital Oscillator Auto-Tuning:** Variables and routines for automated tuning/calibration of oscillators (“autotune” process).
- Manages calibration data buffers and tuning processes for each oscillator/voice.

### PID.h / PID.ino
- **PID Control Tuning:** Configuration and process for PID-based search for optimal tuning parameters during calibration.
- Automated oscillator calibration logic with PID controller.

### FS.h / FS.ino
- **Filesystem / Persistent Storage:** Uses LittleFS to persist calibration and amplitude compensation tables on the device.
- Reads/writes calibration tables used in runtime oscillator level calculation.

---

## Hardware & IO

### PWM.h / PWM.ino
- **PWM Setup:** Hardware PWM timer/channels initialization for controlling DCO frequency.

### state_machines.h / state_machines.ino
- **PIO Initialization:** Logic to configure and enable RP2040's PIO state machines for waveform/frequency timing.

### pico-dco.pio / pico-dco.pio.h
- **PIO Assembly Source:** Custom PIO ASM programs for hardware-level signal generation, compiled headers for C/C++.

### irq_tuner.h / irq_tuner.ino
- **IRQ-Driven Tuner (disabled/commented):** Logic for (currently unused) interrupt-based tuner/frequency measurement.

### Serial.h / Serial.ino
- **Serial Port I/O:** Initialization and external command parsing for various synth parameters via UART (communication and debug).
- Also handles parameter updates and triggers for calibration/tuning via external messages.

### midi.h / midi.ino
- **MIDI Handling:** MIDI input/output (USB and serial) for synthesizer note/event control. Translates input MIDI messages to synth voice activity.

### Timer_millis.h / Timer_millis.ino
- **Timing Utilities:** Millisecond/microsecond timing utilities and update flags for periodic tasks.

---

## Utilities & Data

### globals.h
- **Project-wide Globals:** Constants, macro definitions, pin/out/voice mappings, and all shared state for voice/oscillator/PWM control.

### noteList.h
- **Note Frequency Table:** Accurate tables for converting MIDI note number to frequency (Hz), including all named notes.

### utils.h / utils.ino
- **Helper Functions:** Data conversion, value mapping, exponential response curves, and other math functions for parameter normalization/scaling.

---

## USB Interface & Device

### tusb_config.h / usb_descriptors.c
- **USB MIDI Device Layer:** Configuration files for USB stack device descriptors, specifically for enabling MIDI-over-USB.

---

## Project Workspace

### DCO4_DCO.code-workspace
- **IDE/Editor Session:** Workspace configuration (VSCode or compatible), not used in firmware logic but may be relevant for build tools or automation.

---

# How to Use This File
- Use as a master map of the firmware/module architecture.
- Each section maps to logical responsibilities in the DCO4_DCO synth project.
- For further details, reference specific .h/.ino files as indicated.
