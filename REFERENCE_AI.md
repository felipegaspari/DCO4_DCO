# DCO4_DCO Project: AI Codebase Reference

This document provides a high-level reference for every file in this project. It is meant as a quick onboarding guide and a semantic map for AI agents or developers.

---

## Top-Level Application & Includes

**DCO4_DCO.ino**  
Main entry point; initializes, sets up, and loops main MCU tasks, orchestration for all subsystems (MIDI, synthetic voices, storage, calibration, etc.).

**include_all.h**  
Aggregates all project headers—convenience include for the whole codebase.

---

## Voice Architecture & Processing

**voices.h / voices.ino**  
Manages voice allocation, state, portamento, pitchbend, modulations, note assignment, and per-voice logic. Central hub for polyphonic and monophonic synthesis behavior.

---

## Oscillator and PWM

**PWM.h / PWM.ino**  
Hardware abstraction and setup for PWM outputs—configures the RP2040 PWM peripherals to generate accurate oscillator and pulse width modulation signals.

**pico-dco.pio / pico-dco.pio.h**  
PIO (Programmable IO) state machines firmware for pulse/frequency generation; low-level microcode driving DCO pulse trains for the synth.

---

## Envelope and Modulation

**adsr.h / adsr.ino**  
Implements the ADSR envelope generator with Bezier/interpolation curves and parameter setting per voice.

**LFO.h / LFO.ino**  
Implements multiple low-frequency oscillators (LFOs) used for modulation, detuning, drift, and cyclic modulation of voice parameters.

---

## Tuning, Calibration & Compensation

**amp_comp.h**  
Defines and precomputes frequency-to-amplitude compensation for DCOs; holds lookups and quadratic coefficients for fast interpolation.

**autotune.h / autotune.ino**  
Implements calibration, autotuning, and management of voice/DCO settings and persistent storage of parameters.

**autotune_copy.h / autotune_copy.ino**  
Alternate or legacy implementations of autotune functionality; retained for reference or experiment.

---

## Storage & State Persistence

**FS.h / FS.ino**  
Manages persistence of calibration, tuning, and other parameter data using LittleFS; includes routines for updating and restoring voice data, pulse widths, and banks.

---

## Core MIDI & Serial Interaction

**midi.h / midi.ino**  
Drivers for MIDI (USB and Serial), including note/control handling and MIDI event callbacks for use by the synth engine.

**Serial.h / Serial.ino**  
Serial port initialization & protocol handler, manages data exchange between MCUs and for host interaction.

---

## Control, Parameters, and Utilities

**params.ino**  
Handles real-time parameter updates from external controllers, MIDI CCs, or other interfaces, affecting global and per-voice controls.

**globals.h**  
Defines globals, constants, synthetic architecture specs (voice/circuit counts, pin mappings), and universal state variables used throughout the project.

**noteList.h**  
Defines tables mapping MIDI notes to frequencies (Hz) for quick pitch calculations.

**utils.h / utils.ino**  
General math and conversion helpers (mapping, interpolation, exponential/log converters, display formatting, etc.), plus utility routines like LED status.

---

## PID Control

**PID.h / PID.ino**  
Implements precise PID algorithms for tuning DCO frequency and controlling drift, especially during calibration/autotune.

**PID_copy.h / PID_copy.ino**  
Alternate versions of PID control logic kept for reference or fallback.

---

## PIO State Machines & Initialization

**state_machines.h / state_machines.ino**  
Initialization and management for RP2040 PIO state machines used by the synth's oscillators (including sync).

---

## Timing, Scheduling, and Miscellaneous

**Timer_millis.h / Timer_millis.ino**  
Real-time scheduling using microsecond and millisecond timers; provides flag logic for time-based events and debouncing.

**irq_tuner.h / irq_tuner.ino**  
Experimental/alternate DCO tuning methodology using IRQ/timers for frequency measurement.

---

## USB and System

**tusb_config.h**  
TinyUSB configuration header for USB stack (MIDI over USB, endpoints, descriptors).

**usb_descriptors.c**  
Defines USB device descriptors; required for enumeration as a MIDI device on host.

---

## Project Metadata

**DCO4_DCO.code-workspace**  
VSCode/Cursor workspace settings and layout.

---

## Conventions
- `*.h` = Header (declarations, data structures, constants)
- `*.ino` = Implementation (code/logic, routines)
- `_copy` and similar = Legacy/backups or experimental code

---

### Summary
This codebase is a modular RP2040-based DCO synthesizer system with robust calibration, tuning, and modulation support for each voice/oscillator. Key subsystems include:
- Multi-voice management/logic
- Real-time DCO modulation
- Automated calibration and persistent storage
- MIDI and serial connectivity (host and peer)
- Fast mathematical and interpolation routines for precise synthesis

Use this guide to quickly locate responsibilities and find functional entry points for further reading or code manipulation.
