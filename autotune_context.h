#ifndef __AUTOTUNE_CONTEXT_H__
#define __AUTOTUNE_CONTEXT_H__

#include <stdint.h>

// Lightweight context for DCO calibration routines.
// For now this simply groups references/pointers to existing global state
// so that functions like calibrate_DCO() can be written against a single
// parameter without changing behaviour.
struct DCOCalibrationContext {
  // Reference to the global currentDCO index.
  uint8_t& dcoIndex;
  // Reference to the global DCO_calibration_current_note.
  uint8_t& currentNote;
  // Pointer to the per-DCO calibration buffer (calibrationData).
  uint32_t* calibrationData;
  // Pointer to the per-osc manual calibration offsets.
  int8_t* manualOffsetByOsc;
  // Pointer to the per-osc initial manual amp-comp values.
  int8_t* initManualAmpByOsc;

  DCOCalibrationContext(
    uint8_t& dcoIndexRef,
    uint8_t& currentNoteRef,
    uint32_t* calibrationDataPtr,
    int8_t* manualOffsetPtr,
    int8_t* initManualAmpPtr
  )
    : dcoIndex(dcoIndexRef),
      currentNote(currentNoteRef),
      calibrationData(calibrationDataPtr),
      manualOffsetByOsc(manualOffsetPtr),
      initManualAmpByOsc(initManualAmpPtr) {}
};

#endif  // __AUTOTUNE_CONTEXT_H__


