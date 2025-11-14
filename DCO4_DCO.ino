// #include <stdint.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <math.h>

/*  *** TO DO ***
- Fix PULSE PWN not received or updated when loading patches.
- Fix portament not working properly when updating the portamento value.
- Ask the AI to optimize and clean the autotune code. 
*/

#define RUNNING_AVERAGE

#ifdef RUNNING_AVERAGE
#include "RunningAverage.h"
// Note: RunningAverage objects for voice_task timing are defined in voices.ino
#endif

#define PITCH_INTERP_USE_Q12
#define PITCH_USE_RATIO_Q16 1

// Select clock-divider precision mode: 0 = fast 32-bit fixed-point, 1 = high-precision 64bit integer division
// High precision is preferred for better accuracy at low frequencies, but it is much slower.
#define HIGH_PRECISION_CLKDIV 1

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

#include "fixed_types.h"

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

  precomputeCoefficients();

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
    voice_task();
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