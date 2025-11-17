#ifndef __AMP_COMP_H__
#define __AMP_COMP_H__

#include "include_all.h"
#include <math.h>
#include <limits.h>

// Common table dimensions and shared data
static constexpr int     ampCompTableSize = 22;
static constexpr int32_t AMP_COMP_MAX_HZ  = 7000;

int32_t freq_to_amp_comp_array[352];

bool    plateauWindow[NUM_OSCILLATORS][ampCompTableSize - 1];

// ----- Fixed-point (Q8) amp-comp data, used only when !USE_FLOAT_AMP_COMP -----
#ifndef USE_FLOAT_AMP_COMP

int32_t ampCompArray[NUM_OSCILLATORS][ampCompTableSize + 1];

// Frequency values for amplitude compensation are stored as fixed-point Hz (Q(FREQ_FRAC_BITS)).
static constexpr int     FREQ_FRAC_BITS    = 8;
static constexpr int32_t AMP_COMP_SENTINEL_FREQ_Q = 50000000; // sentinel marker from FS data (Q8)
static constexpr int32_t AMP_COMP_MAX_HZ_Q = (int32_t)(AMP_COMP_MAX_HZ << FREQ_FRAC_BITS);

int32_t  ampCompFrequencyArray[NUM_OSCILLATORS][ampCompTableSize + 1]; // Q8 Hz

// Per-window normalized quadratic in t = (x - x0) / (x2 - x0), where x,x0,x2 are integer Hz.
// Runtime uses 32-bit fixed-point t (Q(T_FRAC)) and precomputed integer coefficients.
static constexpr int     T_FRAC           = 12;
int32_t  xBaseWIN   [NUM_OSCILLATORS][ampCompTableSize - 1];
int32_t  dxWIN      [NUM_OSCILLATORS][ampCompTableSize - 1];
// Use Q28 reciprocal to avoid underflow on very large dx while keeping shifts small
uint32_t invDxWIN_q28[NUM_OSCILLATORS][ampCompTableSize - 1];
int64_t  aQWIN      [NUM_OSCILLATORS][ampCompTableSize - 1]; // Q(T_FRAC)
int64_t  bQWIN      [NUM_OSCILLATORS][ampCompTableSize - 1]; // Q(T_FRAC)
uint16_t cQWIN      [NUM_OSCILLATORS][ampCompTableSize - 1];
int32_t  aQWIN_fast [NUM_OSCILLATORS][ampCompTableSize - 1];
int32_t  bQWIN_fast [NUM_OSCILLATORS][ampCompTableSize - 1];

#endif // !USE_FLOAT_AMP_COMP

// High-precision float coefficients (Hz-domain): y = a*x^2 + b*x + c
// Used by both fixed-point (for reference) and float amp-comp paths.
float    aCoeff[NUM_OSCILLATORS][ampCompTableSize - 1];
float    bCoeff[NUM_OSCILLATORS][ampCompTableSize - 1];
float    cCoeff[NUM_OSCILLATORS][ampCompTableSize - 1];

// Float-domain frequency breakpoints (Hz) used only by the float amp-comp path.
#ifdef USE_FLOAT_AMP_COMP
uint16_t ampCompArray[NUM_OSCILLATORS][ampCompTableSize + 1];    
float    ampCompFrequencyHz[NUM_OSCILLATORS][ampCompTableSize + 1];
#endif


/**
 * @brief Pre-calculates all necessary data for the final, non-hybrid amplitude compensation function.
 *
 * This function's sole purpose is to prepare the data for `get_chan_level_final`.
 * It is called once at startup. For every 3-point window in the compensation table, it:
 * 1.  Loads the raw data into local variables for sanitization.
 * 2.  Optionally cleans the local data by handling sentinels and smoothing plateaus
 *     (new path).
 * 3.  Computes the complete, consistent data package (base, width, reciprocal, and
 *     normalized coefficients) needed for the fast fixed-point quadratic calculation.
 */
// Fixed-point precompute: builds Q-format tables for the fixed engine.
// Only compiled when the float amp-comp engine is not in use.
#ifndef USE_FLOAT_AMP_COMP

static void precomputeCoefficients() {
  static_assert(T_FRAC > 0 && T_FRAC < 28, "T_FRAC must be in a valid range for the math to work.");

  // --- Data Sanitization ---
  // Append a final point to each table to guarantee it reaches the defined maximum.
  // This makes the system robust to incomplete calibration data from the filesystem.
  for (int j = 0; j < NUM_OSCILLATORS; ++j) {
      ampCompFrequencyArray[j][ampCompTableSize] = AMP_COMP_MAX_HZ_Q;
      ampCompArray[j][ampCompTableSize] = DIV_COUNTER;
  }

  const double freqScale    = (double)(1u << FREQ_FRAC_BITS);
  const double invFreqScale = 1.0 / freqScale;
  const double maxFreqHz    = (double)AMP_COMP_MAX_HZ;

  for (int j = 0; j < NUM_OSCILLATORS; j++) {
    for (int i = 0; i < ampCompTableSize - 1; ++i) {
      double x0_f = (double)ampCompFrequencyArray[j][i]     * invFreqScale;
      double x1_f = (double)ampCompFrequencyArray[j][i + 1] * invFreqScale;
      double x2_f = (double)ampCompFrequencyArray[j][i + 2] * invFreqScale;
      double y0_f = (double)ampCompArray[j][i];
      double y1_f = (double)ampCompArray[j][i + 1];
      double y2_f = (double)ampCompArray[j][i + 2];

      if (ampCompFrequencyArray[j][i + 1] >= AMP_COMP_SENTINEL_FREQ_Q) x1_f = maxFreqHz;
      if (ampCompFrequencyArray[j][i + 2] >= AMP_COMP_SENTINEL_FREQ_Q) x2_f = maxFreqHz;

      if (y1_f >= DIV_COUNTER && y2_f >= DIV_COUNTER) {
        double plateau_end_y = (double)DIV_COUNTER;
        x1_f = (x0_f + maxFreqHz) * 0.5;
        y1_f = (y0_f + plateau_end_y) * 0.5;
        x2_f = maxFreqHz;
        y2_f = plateau_end_y;

        ampCompFrequencyArray[j][i + 1] = (int32_t)llround(x1_f * freqScale);
        ampCompArray[j][i + 1] = (uint16_t)llround(y1_f);
        ampCompFrequencyArray[j][i + 2] = AMP_COMP_MAX_HZ_Q;
        ampCompArray[j][i + 2] = (uint16_t)DIV_COUNTER;
      }

      plateauWindow[j][i] = (ampCompArray[j][i + 1] >= (int32_t)DIV_COUNTER &&
                              ampCompArray[j][i + 2] >= (int32_t)DIV_COUNTER);

      // --- 2. Calculate Float Coefficients (for the fallback) ---
      long double denom_ld = (long double)(x0_f - x1_f) * (long double)(x0_f - x2_f) * (long double)(x1_f - x2_f);
      if (denom_ld == 0.0L) denom_ld = 1.0L;
      long double inv_denom_ld = 1.0L / denom_ld;

      long double aVal_ld = ((long double)x2_f * (long double)(y1_f - y0_f) +
                             (long double)x1_f * (long double)(y0_f - y2_f) +
                             (long double)x0_f * (long double)(y2_f - y1_f)) * inv_denom_ld;
      long double bVal_ld = ((long double)x2_f * (long double)x2_f * (long double)(y0_f - y1_f) +
                             (long double)x1_f * (long double)x1_f * (long double)(y2_f - y0_f) +
                             (long double)x0_f * (long double)x0_f * (long double)(y1_f - y2_f)) * inv_denom_ld;
      long double cVal_ld = ((long double)x1_f * (long double)x2_f * (long double)(x1_f - x2_f) * (long double)y0_f +
                             (long double)x2_f * (long double)x0_f * (long double)(x2_f - x0_f) * (long double)y1_f +
                             (long double)x0_f * (long double)x1_f * (long double)(x0_f - x1_f) * (long double)y2_f) * inv_denom_ld;

      aCoeff[j][i] = (float)aVal_ld;
      bCoeff[j][i] = (float)bVal_ld;
      cCoeff[j][i] = (float)cVal_ld;

      long double dx02_ld = (long double)x2_f - (long double)x0_f;
      if (dx02_ld <= 0.0L) dx02_ld = 1.0L;
      long double inv_dx02_ld = 1.0L / dx02_ld;
      long double t1_ld = ((long double)x1_f - (long double)x0_f) * inv_dx02_ld;
      long double d20_ld = (long double)y2_f - (long double)y0_f;
      long double d10_ld = (long double)y1_f - (long double)y0_f;

      long double denom_norm_ld = (t1_ld * t1_ld - t1_ld);
      if (denom_norm_ld == 0.0L) denom_norm_ld = 1.0L;
      long double inv_denom_norm_ld = 1.0L / denom_norm_ld;

      long double aN_ld = (d10_ld - d20_ld * t1_ld) * inv_denom_norm_ld;
      long double bN_ld = d20_ld - aN_ld;

      xBaseWIN[j][i] = ampCompFrequencyArray[j][i];
      dxWIN[j][i]    = ampCompFrequencyArray[j][i + 2] - ampCompFrequencyArray[j][i];
      if (dxWIN[j][i] <= 0) dxWIN[j][i] = 1;

      // Exact integer rounding for Q28 reciprocal
      {
        uint32_t dxu = (uint32_t)dxWIN[j][i];
        uint64_t num = (uint64_t)1ULL << 28;
        invDxWIN_q28[j][i] = (uint32_t)((num + (dxu >> 1)) / dxu);
      }

      aQWIN[j][i] = (int64_t)llroundl(aN_ld * (long double)(1LL << T_FRAC));
      bQWIN[j][i] = (int64_t)llroundl(bN_ld * (long double)(1LL << T_FRAC));

      int32_t c_temp = (int32_t)lrint(y0_f);
      if (c_temp < 0) c_temp = 0;
      if (c_temp > (int32_t)DIV_COUNTER) c_temp = (int32_t)DIV_COUNTER;
      cQWIN[j][i] = (uint16_t)c_temp;

      int64_t aFastLL = llroundl(aN_ld * (long double)(1 << T_FRAC));
      if (aFastLL > (int64_t)INT32_MAX) aFastLL = (int64_t)INT32_MAX;
      if (aFastLL < (int64_t)INT32_MIN) aFastLL = (int64_t)INT32_MIN;
      aQWIN_fast[j][i] = (int32_t)aFastLL;

      // Derive b so that a + b = (y2 - y0) in fast scaling => exact match at t=1
      int64_t d20_int = ((int64_t)ampCompArray[j][i + 2] - (int64_t)ampCompArray[j][i]) << T_FRAC;
      int64_t bFastLL = d20_int - aFastLL;
      if (bFastLL > (int64_t)INT32_MAX) bFastLL = (int64_t)INT32_MAX;
      if (bFastLL < (int64_t)INT32_MIN) bFastLL = (int64_t)INT32_MIN;
      bQWIN_fast[j][i] = (int32_t)bFastLL;
    }
  }
}
#endif  // !USE_FLOAT_AMP_COMP

/**
 * @brief Float-only variant of amplitude compensation precompute.
 *
 * This version prepares only the data needed by the pure-float amp-comp path:
 *  - Sanitises ampCompFrequencyHz / ampCompArray (sentinels, plateaus).
 *  - Computes float quadratic coefficients aCoeff/bCoeff/cCoeff in Hz domain.
 *  - Fills ampCompFrequencyHz (sanitised breakpoints in Hz).
 *
 * It intentionally skips all fixed-point specific structures (xBaseWIN, dxWIN,
 * invDxWIN_q28, aQWIN, bQWIN, cQWIN, aQWIN_fast, bQWIN_fast) so the float engine
 * can be built without any fixed-point amp-comp math if desired.
 */
// Float-only precompute: prepares Hz-domain tables for the float engine.
// Float-only precompute: prepares Hz-domain tables for the float engine.
#ifdef USE_FLOAT_AMP_COMP
static void precomputeCoefficients_float() {
  // Ensure each table has a final point at the defined maximum in Hz.
  for (int j = 0; j < NUM_OSCILLATORS; ++j) {
    ampCompFrequencyHz[j][ampCompTableSize] = (float)AMP_COMP_MAX_HZ;
    ampCompArray[j][ampCompTableSize]       = DIV_COUNTER;
  }

  const double maxFreqHz = (double)AMP_COMP_MAX_HZ;

  for (int j = 0; j < NUM_OSCILLATORS; ++j) {
    for (int i = 0; i < ampCompTableSize - 1; ++i) {
      // Work directly in Hz domain using the float frequency table.
      double x0_f = (double)ampCompFrequencyHz[j][i];
      double x1_f = (double)ampCompFrequencyHz[j][i + 1];
      double x2_f = (double)ampCompFrequencyHz[j][i + 2];
      double y0_f = (double)ampCompArray[j][i];
      double y1_f = (double)ampCompArray[j][i + 1];
      double y2_f = (double)ampCompArray[j][i + 2];

      // Clamp any out-of-band/sentinel frequencies to the defined maximum.
      if (x1_f >= maxFreqHz) x1_f = maxFreqHz;
      if (x2_f >= maxFreqHz) x2_f = maxFreqHz;

      // Plateau smoothing: if both subsequent points are at/beyond full scale,
      // synthesise a smooth transition into the plateau and update the tables.
      if (y1_f >= DIV_COUNTER && y2_f >= DIV_COUNTER) {
        double plateau_end_y = (double)DIV_COUNTER;
        x1_f = (x0_f + maxFreqHz) * 0.5;
        y1_f = (y0_f + plateau_end_y) * 0.5;
        x2_f = maxFreqHz;
        y2_f = plateau_end_y;

        ampCompFrequencyHz[j][i + 1] = (float)x1_f;
        ampCompArray[j][i + 1]       = (uint16_t)llround(y1_f);
        ampCompFrequencyHz[j][i + 2] = (float)maxFreqHz;
        ampCompArray[j][i + 2]       = (uint16_t)DIV_COUNTER;
      }

      // Mark plateau windows for fast runtime clamping in both paths.
      plateauWindow[j][i] = (ampCompArray[j][i + 1] >= (int32_t)DIV_COUNTER &&
                             ampCompArray[j][i + 2] >= (int32_t)DIV_COUNTER);

      // Compute float quadratic coefficients y = a*x^2 + b*x + c in Hz domain.
      long double denom_ld =
        (long double)(x0_f - x1_f) * (long double)(x0_f - x2_f) * (long double)(x1_f - x2_f);
      if (denom_ld == 0.0L) denom_ld = 1.0L;
      long double inv_denom_ld = 1.0L / denom_ld;

      long double aVal_ld = ((long double)x2_f * (long double)(y1_f - y0_f) +
                             (long double)x1_f * (long double)(y0_f - y2_f) +
                             (long double)x0_f * (long double)(y2_f - y1_f)) * inv_denom_ld;
      long double bVal_ld = ((long double)x2_f * (long double)x2_f * (long double)(y0_f - y1_f) +
                             (long double)x1_f * (long double)x1_f * (long double)(y2_f - y0_f) +
                             (long double)x0_f * (long double)x0_f * (long double)(y1_f - y2_f)) * inv_denom_ld;
      long double cVal_ld = ((long double)x1_f * (long double)x2_f * (long double)(x1_f - x2_f) * (long double)y0_f +
                             (long double)x2_f * (long double)x0_f * (long double)(x2_f - x0_f) * (long double)y1_f +
                             (long double)x0_f * (long double)x1_f * (long double)(x0_f - x1_f) * (long double)y2_f) * inv_denom_ld;

      aCoeff[j][i] = (float)aVal_ld;
      bCoeff[j][i] = (float)bVal_ld;
      cCoeff[j][i] = (float)cVal_ld;

      // Store the left breakpoint in Hz for this window (kept in sync with any smoothing).
      ampCompFrequencyHz[j][i] = (float)x0_f;
    }

    // The final breakpoint (ampCompTableSize) in Hz has already been initialised to AMP_COMP_MAX_HZ.
  }
}
#endif  // USE_FLOAT_AMP_COMP


// ---------------------------------------------------------------------------
// Engine-agnostic amp-comp API
// ---------------------------------------------------------------------------
// These helpers let the rest of the code call a single precompute/lookup API,
// while the concrete implementation is selected at compile time.

// Dispatch to the correct precompute routine based on the active amp-comp mode.
static inline void precompute_amp_comp_for_engine() {
#ifdef USE_FLOAT_AMP_COMP
  precomputeCoefficients_float();
#else
  precomputeCoefficients();
#endif
}

// Unified lookup facade: always take frequency in Hz, choose implementation at compile time.
// The concrete implementations live in voices.ino; we only provide the wrapper and prototypes here.

#ifdef USE_FLOAT_AMP_COMP
// Float engine: pure-Hz fast lookup implemented in voices.ino
uint16_t get_chan_level_float(float freqHz, uint8_t voiceN);
#else
// Fixed engine: fast fixed-point lookup implemented in voices.ino
uint16_t get_chan_level_lookup_fast(int32_t x, uint8_t voiceN);
#endif

static inline uint16_t get_chan_level_for_engine(float freqHz, uint8_t voiceN) {
#ifdef USE_FLOAT_AMP_COMP
  // Float amp-comp: delegate directly to the Hz-domain lookup.
  return get_chan_level_float(freqHz, voiceN);
#else
  // Fixed-point amp-comp: convert Hz to Q(FREQ_FRAC_BITS) and call fast lookup.
  if (freqHz <= 0.0f) return 0;
  if (freqHz >= (float)AMP_COMP_MAX_HZ) {
    return get_chan_level_lookup_fast(AMP_COMP_MAX_HZ_Q, voiceN);
  }

  int32_t x_q = (int32_t)lrintf(freqHz * (float)(1 << FREQ_FRAC_BITS));
  return get_chan_level_lookup_fast(x_q, voiceN);
#endif
}



// #else

// uint_fast32_t freq_to_amp_comp_array[] = {
// 0,0,
// 1635,40,
// 2183,50,
// 2914,62,
// 3889,79,
// 5191,101,
// 6930,130,
// 9250,170,
// 12347,222,
// 16481,292,
// 22000,386,
// 29366,511,
// 39200,675,
// 52325,924,
// 69846,1252,
// 93233,1688,
// 124451,2231,
// 166122,3034,
// 221746,4132,
// 295996,5632,
// 395107,7676,
// 505830,10000,
// 20000000,10000,
// };
// uint8_t ampCompArraySize = sizeof(freq_to_amp_comp_array);
// #endif

// float freq_to_amp_comp_array_float[] = {
// 18.35, 0,
// 20.60, 2,
// 27.50, 6,
// 32.70, 9,
// 43.65, 16,
// 55.00, 23,
// 65.41, 29,
// 87.31, 42,
// 110.00, 56,
// 130.81, 68,
// 174.61, 96,
// 220.00, 122,
// 261.63, 148,
// 349.23, 201,
// 440.00, 255,
// 659.25, 389,
// 880.00, 516,
// 1318.51, 768,
// 1786.00, 1024,
// 2094.00, 1198,
// 2798.00, 1670,
// 3520.00, 1944,
// 3734.00, 2052,
// 4352.00, 2342,
// 4717.00, 2499,
// 30000.00, 2500.00,
// };

// float freq_to_vco_array[] = {
// 20.60, 22,
// 21.83, 41,
// 32.70, 200,
// 43.65, 296,
// 65.41, 462,
// 87.31, 572,
// 130.81, 732,
// 174.61, 847,
// 261.63, 1019,
// 349.23, 1147,
// 523.25, 1322,
// 698.46, 1449,
// 1046.50, 1621,
// 1396.91, 1739,
// 2093.00, 1894,
// 4186.00, 2164,
// 8372.01, 2425,
// 30000.00, 2500.00,
// };

// float freq_to_vco_array[] = {    // 10000 divisions
// 0, 1,
// 16.35, 69,	// C0
// 20.60, 352,  	// E0
// 21.83, 441,  	// F0
// 27.50, 777,		// A0
// 32.70, 1037, 	// C1
// 43.65, 1464, 	// F1
// 65.41, 2080, 	// C2
// 87.31, 2543, 	// F2
// 130.81, 3180, 	// C3
// 174.61, 3651,	// F3
// 261.63, 4333,	// C4
// 349.23, 4827,	// F4
// 523.25, 5521,	// C5
// 698.46, 6035,	// F5
// 1046.50, 6715,	// C6
// 1396.91, 7189,	// F6
// 2093.00, 7826,	// C7
// 4186.00, 8910,	// C8
// 8372.01, 9999,	// C9
// 30000.00, 9999,
// };

#endif