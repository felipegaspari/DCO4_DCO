#ifndef __AUTOTUNE_MEASUREMENT_H__
#define __AUTOTUNE_MEASUREMENT_H__

#include "autotune_constants.h"

// Forward declaration so this header can be included before the definition
// in autotune.ino.
float find_gap(byte specialMode);

// Simple wrapper around find_gap() that interprets the timeout sentinel and
// returns a structured result instead of a raw float.
struct GapMeasurement {
  bool timedOut;
  float value;
};

inline GapMeasurement measure_gap(byte specialMode) {
  float v = find_gap(specialMode);
  GapMeasurement result;
  result.timedOut = (v == kGapTimeoutSentinel);
  result.value = v;
  return result;
}

#endif  // __AUTOTUNE_MEASUREMENT_H__


