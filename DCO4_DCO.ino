// #include <stdint.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <math.h>

/*  *** TO DO ***
- Fix PULSE PWN not received or updated when loading patches.
- Ask the AI to optimize and clean the autotune code. 
*/

// ---------------------------------------------------------------------------
// Profiling (see docs/BENCHMARKING.md)
// ---------------------------------------------------------------------------
// RUNNING_AVERAGE enables the hot-path profiler in bench.h: per-probe count / mean / min /
// max / total plus each probe's share of its core's wall clock. Off means zero cost.
// RUNNING_AVERAGE_FINE additionally instruments the smallest stages (a few multiplies
// each). Every probe is an optimisation barrier, so enabling FINE changes codegen — it is
// there to measure that distortion, not to be left on.
#define RUNNING_AVERAGE
// #define RUNNING_AVERAGE_FINE

// ---------------------------------------------------------------------------
// Voice engine build options
// ---------------------------------------------------------------------------
// High-level engine selection:
// - For RP2040 (no FPU): comment this out to use the fixed-point engine.
// - For RP2350 (with FPU): leave defined to use the float-based engine.
#define USE_FLOAT_ENGINE

// Derived switches for the different subsystems:
#ifdef USE_FLOAT_ENGINE
  // Use float-based voice task (pitch path, modifiers, clock-divider, etc.)
  #define USE_FLOAT_VOICE_TASK
  // Use float-based amplitude compensation (pure Hz domain).
  #define USE_FLOAT_AMP_COMP
#endif


// ---------------------------------------------------------------------------
// RP2040 or fixed point engine specific settings
  // Pitch interpolation mode:
  #define PITCH_USE_RATIO_Q16 1 // Uncomment this to use Q16 for pitch interpolation. dEFAULT mode.

  // IF PITCH_USE_RATIO_Q16 IS NOT DEFINED, THEN:
    // Use Q12 for pitch interpolation. Q12 is a good compromise between accuracy and speed.
    // This mostly affects the multiplier table interpolation (pitch bend, detune, unison, ADSR, drift etc.) applied to frequency.
    // Higher precision means smaller stepping when modulating frequency.
    //
    // #ifdef PITCH_INTERP_USE_Q8_ 32-bit friendly path: slope in Q8, delta in Q8; total 16 frac bits
    // #ifdef PITCH_INTERP_USE_Q12: enables medium-precision path: slope in Q12, delta in Q12; total 24 frac bits
    // else: enables high-precision path: slope in Q20, delta in Q16
  #define PITCH_INTERP_USE_Q12  // Uncomment this to use Q12 for pitch interpolation WHEN PITCH_USE_RATIO_Q16 IS NOT DEFINED.
  //#define PITCH_INTERP_USE_Q8 // Uncomment this to use Q8 for pitch interpolation WHEN PITCH_USE_RATIO_Q16 IS NOT DEFINED.
  
  
  
  // Select clock-divider precision mode for the fixed-point path:
  // 0 = fast 32-bit fixed-point, 1 = high-precision 64bit integer division
  // High precision is preferred for better accuracy at low frequencies, but it is much slower than fixed point. 
  // High precision is the default method, at 4uS per voice. Fixed-point takes 1uS per voice.
  // The fixed-point method is there in case I want to try some crazy fast modulation, or to move the project to a much slower processor.
#define HIGH_PRECISION_CLKDIV 1

// Uncomment to benchmark float vs double clock-divider calculations in voice_task_float:
// #define CLKDIV_BENCHMARK

#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
//#include "tusb_config.h"

#include "pico/stdlib.h"
// #include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico-dco.pio.h"
#include "hardware/pwm.h"
// #include "hardware/spi.h"

#include "LittleFS.h"
// #include <SingleFileDrive.h>
// #include <EEPROM.h>

#include <stdint.h>
#include "params_def.h"
#include "param_router.h"

#include "globals.h"
#include "bench.h"

#include "FS.h"

#include "noteList.h"
#include "amp_comp.h"

#include "Serial.h"
#include "midi.h"
#include "voices.h"
#include "state_machines.h"
#include "PWM.h"
#include "utils.h"
#include "Timer_millis.h"

#include "LFO.h"
#include "adsr.h"

#include "PID.h"
#include "autotune.h"

// #include "irq_tuner.h"

// ****************************************************************************************** //

// Core 0 boot: serial, MIDI, LFOs, board fix pins, USB descriptors, calibration input pin.
void setup() {
  //set_sys_clock_khz(sysClock, true);
  // EEPROM.begin(512);
  bench_init_core();  // SysTick is per core; core 1 arms its own in setup1()
  init_serial();
  init_midi();

  init_LFOs();
  init_DRIFT_LFOs();

  
  // init_tuner();
  // init_tuning_tables();

  pinMode(23, OUTPUT);
  digitalWrite(23, HIGH);

  pinMode(24, OUTPUT);  // Fix pin on DCO BOARD
  digitalWrite(24, HIGH);

  USBDevice.setManufacturerDescriptor("FELA         ");
  USBDevice.setProductDescriptor("DCO-4        ");

  pinMode(DCO_calibration_pin, INPUT_PULLUP);

  // gpio_init(11);
  // gpio_set_dir(11, GPIO_IN);
  // gpio_pull_down(11);
}

// Core 1 boot: PID, LittleFS cal load, ADSR, amp-comp precompute, PWM/PIO, voices.
// Clears calibrationFlag so the init_DCO_calibration block below is currently unreachable.
void setup1() {

  //set_sys_clock_khz(sysClock, true);

  bench_init_core();

  init_PID();

  init_FS();

  init_ADSR();

  // Select amplitude-compensation precompute based on engine type.
  precompute_amp_comp_for_engine();

  calibrationFlag = false;
  manualCalibrationFlag = false;
  firstTuneFlag = true;

  init_pwm();
  init_pio();
  init_voices();

  if (calibrationFlag == true) {
    init_DCO_calibration();
    voice_task_autotune(0, ampCompCalibrationVal);
  }
}

// Core 0 forever loop: MIDI read, Serial2 parser, LFO1; ~100 µs LFO2 + drift + FIFO push of detune.
void loop() {
  BENCH_PERIOD(loop0_period);
  loop0_micros = micros();

  {
    BENCH_BEGIN(loop0_io);
    MIDI_USB.read();
    MIDI_SERIAL.read();
    serial_STM32_task();
    LFO1();
    BENCH_END(loop0_io);
  }

  if ((loop0_micros - loop0_microsLast) > 100) {
    {
      BENCH_BEGIN(loop0_lfo2);
      LFO2();
      BENCH_END(loop0_lfo2);
    }

    {
      BENCH_BEGIN(loop0_drift);
      DRIFT_LFOs();
      BENCH_END(loop0_drift);
    }

    {
      // Transfer LFO1 detune modulation as a raw Q24 fixed-point integer via FIFO.
      BENCH_BEGIN(loop0_fifo_push);
      rp2040.fifo.push_nb((uint32_t)DETUNE_INTERNAL_q24);
      BENCH_END(loop0_fifo_push);
    }

    loop0_microsLast = loop0_micros;
  }

  // Snapshot core 0's probes and print once core 1 has handed its own over. All profiler
  // serial traffic happens here, never on the audio core.
  bench_poll_core0();
}

// Core 1 forever loop: soft timers; auto/manual calibration OR ADSR + FIFO pop + voice_task_main.
void loop1() {
  BENCH_PERIOD(loop1_period);

  {
    BENCH_BEGIN(loop1_millis);
    millisTimer();
    BENCH_END(loop1_millis);
  }

  if (calibrationFlag == true) {
    if (manualCalibrationFlag == true) {
      VOICE_NOTES[0]               = manual_DCO_calibration_start_note;
      DCO_calibration_current_note = manual_DCO_calibration_start_note;
      ampCompCalibrationVal = initManualAmpCompCalibrationValPreset + manualCalibrationOffset[manualCalibrationStage / 2];
      voice_task_autotune(0, ampCompCalibrationVal);
      // In manual calibration mode, continuously measure and report the duty
      // difference so the screen can display live feedback for the user.
      DCO_calibration_debug();
      //Serial.println((String) "PW value: " + (PW[0] / 4));

    } else {
      DCO_calibration();
    }
  } else {

    loop1_micros = micros();

    if ((loop1_micros - loop1_microsLast) > 100) {
      {
        BENCH_BEGIN(loop1_adsr);
        ADSR_update();
        BENCH_END(loop1_adsr);
      }
      loop1_microsLast = loop1_micros;
    }

    {
      // Receive Q24 detune value from core 0; reinterpret raw bits back to signed.
      BENCH_BEGIN(loop1_fifo_pop);
      rp2040.fifo.pop_nb(detune_fifo_variable);
      DETUNE_INTERNAL_FIFO_q24 = (int32_t)DETUNE_INTERNAL_FIFO;
      BENCH_END(loop1_fifo_pop);
    }

    {
      BENCH_BEGIN(voice_task);
      voice_task_main();
      BENCH_END(voice_task);
    }
  }

  // Hand this core's counters to core 0, which does all the printing.
  bench_service(1);
}
