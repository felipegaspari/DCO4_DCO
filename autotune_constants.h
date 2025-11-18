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

#endif  // __AUTOTUNE_CONSTANTS_H__


