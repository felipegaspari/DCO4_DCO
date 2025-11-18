// #include <stdint.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <math.h>

/*  *** TO DO ***
- Fix PULSE PWN not received or updated when loading patches.
- Ask the AI to optimize and clean the autotune code. 
*/

// #define RUNNING_AVERAGE

#ifdef RUNNING_AVERAGE
#include "RunningAverage.h"
// Note: RunningAverage objects for voice_task timing are defined in voices.ino
#endif

// ---------------------------------------------------------------------------
// Voice engine build options
// ---------------------------------------------------------------------------
// High-level engine selection:
// - For RP2040 (no FPU): comment this out to use the fixed-point engine.
// - For RP2350 (with FPU): leave defined to use the float-based engine.
// #define USE_FLOAT_ENGINE

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

#ifdef RUNNING_AVERAGE
RunningAverage ra_loop1_ADSR_and_detune(2000);
RunningAverage ra_loop0_LFOs(2000);
RunningAverage ra_loop0_DRIFT_LFOs(2000);
RunningAverage ra_loop0_MIDI_and_serial(2000);
RunningAverage ra_loop0_memcpy(2000);

#endif

// ****************************************************************************************** //

void setup() {
  //set_sys_clock_khz(sysClock, true);
  // EEPROM.begin(512);
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

void setup1() {

  //set_sys_clock_khz(sysClock, true);

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

void loop() {
  // unsigned long loop0_start_time = micros();
  // unsigned long loop0_total_time;
  loop0_micros = micros();

  MIDI_USB.read();
  MIDI_SERIAL.read();
  serial_STM32_task();

  LFO1();

#ifdef RUNNING_AVERAGE
ra_loop0_MIDI_and_serial.addValue((float)(micros() - loop0_micros));
#endif

  if ((loop0_micros - loop0_microsLast) > 100) {
#ifdef RUNNING_AVERAGE
    unsigned long t_loop0_LFOs = micros();
#endif

    LFO2();

    #ifdef RUNNING_AVERAGE
    ra_loop0_LFOs.addValue((float)(micros() - t_loop0_LFOs));
    unsigned long t_loop0_DRIFT_LFOs = micros();
#endif

    DRIFT_LFOs();
    #ifdef RUNNING_AVERAGE
    ra_loop0_DRIFT_LFOs.addValue((float)(micros() - t_loop0_DRIFT_LFOs));
    unsigned long t_loop0_memcpy = micros();
#endif

    // Transfer LFO1 detune modulation as a raw Q24 fixed-point integer via FIFO.
    rp2040.fifo.push_nb((uint32_t)DETUNE_INTERNAL_q24);

#ifdef RUNNING_AVERAGE
    ra_loop0_memcpy.addValue((float)(micros() - t_loop0_memcpy));
#endif

    loop0_microsLast = loop0_micros;
    // Serial.println((String)"a" + (micros() - a));
  }
  // loop0_total_time = micros() - loop0_start_time;
  // if (loop0_total_time > 10) {
  //// Serial.println(loop0_total_time);
  // }
}

void loop1() {
  // unsigned long loop1_start_time = micros();
  // unsigned long loop1_total_time;

  millisTimer();

  if (calibrationFlag == true) {
    if (manualCalibrationFlag == true) {
      VOICE_NOTES[0] = manual_DCO_calibration_start_note;
      ampCompCalibrationVal = initManualAmpCompCalibrationValPreset + manualCalibrationOffset[manualCalibrationStage / 2];
      voice_task_autotune(0, ampCompCalibrationVal);
      //DCO_calibration_debug(); // disabled because of manual calibration bug on osc 0 offset. needs fix
      Serial.println((String) "PW value: " + (PW[0] / 4));

    } else {
      DCO_calibration();
    }
  } else {

    loop1_micros = micros();

    if ((loop1_micros - loop1_microsLast) > 100) {
#ifdef RUNNING_AVERAGE
      unsigned long t_loop1_ADSR_and_detune = micros();
#endif
      ADSR_update();

#ifdef RUNNING_AVERAGE
      ra_loop1_ADSR_and_detune.addValue((float)(micros() - t_loop1_ADSR_and_detune));
#endif
      loop1_microsLast = loop1_micros;
    }
    
          // Receive Q24 detune value from core 0; reinterpret raw bits back to signed.
          rp2040.fifo.pop_nb(detune_fifo_variable);
          DETUNE_INTERNAL_FIFO_q24 = (int32_t)DETUNE_INTERNAL_FIFO;

    // loop speed
    //  loop1_start_time = micros();
    // Serial.println("pre voice task");
    voice_task_main();
    //voice_task_gold_reference();
    //voice_task_simple();
    //voice_task_debug();
    // Serial.println("post voice task");
    // loop speed
    // loop1_total_time = micros() - loop1_start_time;
    //  if (loop1_total_time > 50) {
    // Serial.println(loop1_total_time);
    //  }
    // Serial.println("loop1");
  }

  #ifdef RUNNING_AVERAGE
  if (timer1000msFlag) {
    print_running_averages();
  }
  #endif
}

#ifdef RUNNING_AVERAGE
void print_running_averages() {
  Serial.println("--------------------------------");
  Serial.println("RUNNING AVERAGES");
  Serial.println("--------------------------------");
  Serial.println("Loop0");
  Serial.println((String) "Loop0 MIDI and Serial: " + ra_loop0_MIDI_and_serial.getFastAverage());
  Serial.println((String) "Loop0 LFOs: " + ra_loop0_LFOs.getFastAverage());
  Serial.println((String) "Loop0 DRIFT LFOs: " + ra_loop0_DRIFT_LFOs.getFastAverage());
  Serial.println((String) "Loop0 memcpy: " + ra_loop0_memcpy.getFastAverage());
  Serial.println("Loop1");
  Serial.println((String) "Loop1 ADSR and Detune: " + ra_loop1_ADSR_and_detune.getFastAverage());



  print_voice_task_timings();
}
#endif