#ifndef __AUTOTUNE_CONSTANTS_H__
#define __AUTOTUNE_CONSTANTS_H__

// Common constants used by the DCO/VCO autotune and measurement code.
// Kept here to avoid magic numbers scattered across the implementation.

// Sentinel value returned by gap-measurement routines to indicate
// a timeout or invalid measurement.
constexpr float kGapTimeoutSentinel = 1.16999f;

// Maximum time (in microseconds) without seeing an edge before a
// measurement is considered timed out.
constexpr unsigned long kGapTimeoutUs = 100000UL;

// Minimum time (in microseconds) between detected edges to treat
// them as valid (simple debounce).
constexpr unsigned long kEdgeDebounceMinUs = 20UL;

// Target duty fractions for PW calibration:
//  - Center:  50% duty
//  - Low:      2% duty (user-adjustable if desired)
//  - High:    98% duty (user-adjustable if desired)
constexpr double kPWCenterDutyFraction = 0.5;
constexpr double kPWLowDutyFraction    = 0.02;
constexpr double kPWHighDutyFraction   = 0.98;

// Polarity of the digital calibration signal on DCO_calibration_pin.
// If your hardware inverts the waveform (so the pin is high when the
// actual DCO output is low, and vice versa), set this to true. All duty
// measurements (find_gap / measure_gap) will automatically compensate.
constexpr bool kGapPolarityInverted    = false;   // set to false for non-inverted hardware

// Duty tolerance used when validating PW low/high limits and PW center lock-in.
// A sample whose duty is within ±kPWLimitDutyTolerance of the target
// low/center/high duty is considered "in tolerance".
constexpr double kPWLimitDutyTolerance = 0.01;  // ±5%

#endif  // __AUTOTUNE_CONSTANTS_H__


