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
constexpr unsigned long kEdgeDebounceMinUs = 30UL;

// Target duty fractions for PW calibration:
//  - Center:  50% duty
//  - Low:     10% duty (user-adjustable if desired)
//  - High:    90% duty (user-adjustable if desired)
constexpr double kPWCenterDutyFraction = 0.5;
constexpr double kPWLowDutyFraction    = 0.02;
constexpr double kPWHighDutyFraction   = 0.98;

#endif  // __AUTOTUNE_CONSTANTS_H__


