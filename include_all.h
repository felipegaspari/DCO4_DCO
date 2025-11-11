#ifndef __INCLUDE_ALL__H_
#define __INCLUDE_ALL__H_

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
//#include <SingleFileDrive.h>
// #include <EEPROM.h>

#include "globals.h"
#include "fixed_types.h"

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

#endif