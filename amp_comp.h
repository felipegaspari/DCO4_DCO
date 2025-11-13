#ifndef __AMP_COMP_H__
#define __AMP_COMP_H__

#include "include_all.h"
#include <math.h>
#include <limits.h>

static constexpr int ampCompTableSize = 22;
// Windows with steep slopes (upper band) evaluated in double precision
static constexpr int AMP_COMP_DOUBLE_WINDOW_START = ampCompTableSize - 10;
// Frequency values for amplitude compensation are stored as fixed-point Hz (Q(FREQ_FRAC_BITS))
static constexpr int FREQ_FRAC_BITS = 8;
static constexpr int32_t AMP_COMP_SENTINEL_FREQ_Q = 50000000; // sentinel marker from FS data (Q8)
// Maximum frequency (Hz) for which we apply amplitude compensation.
// At or above this frequency, get_chan_level() returns full scale (DIV_COUNTER).
static constexpr int32_t AMP_COMP_MAX_HZ = 7000;
static constexpr int32_t AMP_COMP_MAX_HZ_Q = (int32_t)(AMP_COMP_MAX_HZ << FREQ_FRAC_BITS);

static constexpr int AMP_COMP_FAST_T_FRAC = 12;
static constexpr int AMP_COMP_FAST_RECIP_EXTRA = 12;
static constexpr int AMP_COMP_FAST_COEFF_FRAC = 6;
static constexpr int AMP_COMP_FAST_SHIFT = AMP_COMP_FAST_COEFF_FRAC + AMP_COMP_FAST_T_FRAC;
static constexpr int AMP_COMP_FAST_SLOPE_FRAC = 12;

uint8_t tMulShiftFast[NUM_OSCILLATORS][ampCompTableSize - 2];
bool useQuadWindow[NUM_OSCILLATORS][ampCompTableSize - 2];
int32_t slopeDx_fast[NUM_OSCILLATORS][ampCompTableSize - 2];

int32_t freq_to_amp_comp_array[352];
uint8_t ampCompArraySize = FSVoiceDataSize / 4;

int32_t ampCompFrequencyArray[NUM_OSCILLATORS][ampCompTableSize];
int32_t ampCompArray[NUM_OSCILLATORS][ampCompTableSize];

// High-precision float coefficients (original model): y = a*x^2 + b*x + c
float aCoeff[NUM_OSCILLATORS][ampCompTableSize - 2];
float bCoeff[NUM_OSCILLATORS][ampCompTableSize - 2];
float cCoeff[NUM_OSCILLATORS][ampCompTableSize - 2];
double aCoeffD[NUM_OSCILLATORS][ampCompTableSize - 2];
double bCoeffD[NUM_OSCILLATORS][ampCompTableSize - 2];
double cCoeffD[NUM_OSCILLATORS][ampCompTableSize - 2];
bool useDoubleWindow[NUM_OSCILLATORS][ampCompTableSize - 2];
bool plateauWindow[NUM_OSCILLATORS][ampCompTableSize - 2];

// Per-window normalized quadratic in t = (x - x0) / (x2 - x0), where x,x0,x2 are integer Hz.
// Runtime uses 32-bit fixed-point t (Q(T_FRAC)) and precomputed integer coefficients.
static constexpr int T_FRAC = 14;
int32_t xBaseWIN[NUM_OSCILLATORS][ampCompTableSize - 2];
int32_t dxWIN[NUM_OSCILLATORS][ampCompTableSize - 2];
// Use Q28 reciprocal to avoid underflow on very large dx while keeping shifts small
uint32_t invDxWIN_q28[NUM_OSCILLATORS][ampCompTableSize - 2];
int64_t aQWIN[NUM_OSCILLATORS][ampCompTableSize - 2]; // Q(T_FRAC) wide
int64_t bQWIN[NUM_OSCILLATORS][ampCompTableSize - 2]; // Q(T_FRAC) wide
uint16_t cQWIN[NUM_OSCILLATORS][ampCompTableSize - 2];
uint16_t invDxWIN_qFast[NUM_OSCILLATORS][ampCompTableSize - 2];
int32_t aQWIN_fast[NUM_OSCILLATORS][ampCompTableSize - 2];
int32_t bQWIN_fast[NUM_OSCILLATORS][ampCompTableSize - 2];
// (removed legacy 32-bit-only fast-path parameters and integer coeffs)


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
static void precomputeCoefficients() {
  static_assert(T_FRAC > 0 && T_FRAC < 28, "T_FRAC must be in a valid range for the math to work.");

  const double freqScale    = (double)(1u << FREQ_FRAC_BITS);
  const double invFreqScale = 1.0 / freqScale;
  const double maxFreqHz    = (double)AMP_COMP_MAX_HZ;
  const int32_t maxFreqQ    = AMP_COMP_MAX_HZ_Q;
  const uint32_t fastRecipNumerator = (uint32_t)1 << (AMP_COMP_FAST_T_FRAC + AMP_COMP_FAST_RECIP_EXTRA);
  const int fastCoeffShift = AMP_COMP_FAST_COEFF_FRAC + AMP_COMP_FAST_T_FRAC;

  for (int j = 0; j < NUM_OSCILLATORS; j++) {
    for (int i = 0; i < ampCompTableSize - 2; ++i) {
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

      long double aVal_ld = ((long double)x2_f * (long double)(y1_f - y0_f) +
                             (long double)x1_f * (long double)(y0_f - y2_f) +
                             (long double)x0_f * (long double)(y2_f - y1_f)) / denom_ld;
      long double bVal_ld = ((long double)x2_f * (long double)x2_f * (long double)(y0_f - y1_f) +
                             (long double)x1_f * (long double)x1_f * (long double)(y2_f - y0_f) +
                             (long double)x0_f * (long double)x0_f * (long double)(y1_f - y2_f)) / denom_ld;
      long double cVal_ld = ((long double)x1_f * (long double)x2_f * (long double)(x1_f - x2_f) * (long double)y0_f +
                             (long double)x2_f * (long double)x0_f * (long double)(x2_f - x0_f) * (long double)y1_f +
                             (long double)x0_f * (long double)x1_f * (long double)(x0_f - x1_f) * (long double)y2_f) / denom_ld;

      aCoeff[j][i] = (float)aVal_ld;
      bCoeff[j][i] = (float)bVal_ld;
      cCoeff[j][i] = (float)cVal_ld;
      aCoeffD[j][i] = (double)aVal_ld;
      bCoeffD[j][i] = (double)bVal_ld;
      cCoeffD[j][i] = (double)cVal_ld;

      long double dx02_ld = (long double)x2_f - (long double)x0_f;
      if (dx02_ld <= 0.0L) dx02_ld = 1.0L;
      long double t1_ld = ((long double)x1_f - (long double)x0_f) / dx02_ld;
      long double d20_ld = (long double)y2_f - (long double)y0_f;
      long double d10_ld = (long double)y1_f - (long double)y0_f;

      long double denom_norm_ld = (t1_ld * t1_ld - t1_ld);
      if (denom_norm_ld == 0.0L) denom_norm_ld = 1.0L;

      long double aN_ld = (d10_ld - d20_ld * t1_ld) / denom_norm_ld;
      long double bN_ld = d20_ld - aN_ld;

      // Heuristic: quadratic only if mid deviation from linear > 0.75 units
      double y_mid_lin = y0_f + (double)(0.5L * d20_ld);
      double y_mid_quad = (double)(aN_ld * 0.25L + bN_ld * 0.5L) + y0_f;
      double dev_mid = fabs(y_mid_quad - y_mid_lin);
      useQuadWindow[j][i] = (dev_mid > 0.75);

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

      // Fast reciprocal and per-window t scaling to keep 32-bit safe
      uint32_t invFast = (uint32_t)((fastRecipNumerator + (uint32_t)(dxWIN[j][i] >> 1)) / (uint32_t)dxWIN[j][i]);
      if (invFast == 0) invFast = 1;
      if (invFast > 0xFFFFu) invFast = 0xFFFFu;
      invDxWIN_qFast[j][i] = (uint16_t)invFast;

      uint8_t s = 0;
      uint64_t prod64 = (uint64_t)(uint32_t)dxWIN[j][i] * (uint64_t)invFast;
      while (prod64 > 0x7FFFFFFFull && s < 31) { ++s; prod64 >>= 1; }
      tMulShiftFast[j][i] = s;

      // Linear slope in y per Q8-Hz for fast linear path
      int64_t slopeLL = llroundl(d20_ld * (long double)(1 << AMP_COMP_FAST_SLOPE_FRAC) / (long double)dxWIN[j][i]);
      if (slopeLL > (int64_t)INT32_MAX) slopeLL = (int64_t)INT32_MAX;
      if (slopeLL < (int64_t)INT32_MIN) slopeLL = (int64_t)INT32_MIN;
      slopeDx_fast[j][i] = (int32_t)slopeLL;

      int64_t aFastLL = llroundl(aN_ld * (long double)(1 << AMP_COMP_FAST_COEFF_FRAC));
      if (aFastLL > (int64_t)INT32_MAX) aFastLL = (int64_t)INT32_MAX;
      if (aFastLL < (int64_t)INT32_MIN) aFastLL = (int64_t)INT32_MIN;
      aQWIN_fast[j][i] = (int32_t)aFastLL;

      // Derive b so that a + b = (y2 - y0) in fast scaling => exact match at t=1
      int64_t d20_int = ((int64_t)ampCompArray[j][i + 2] - (int64_t)ampCompArray[j][i]) << AMP_COMP_FAST_COEFF_FRAC;
      int64_t bFastLL = d20_int - aFastLL;
      if (bFastLL > (int64_t)INT32_MAX) bFastLL = (int64_t)INT32_MAX;
      if (bFastLL < (int64_t)INT32_MIN) bFastLL = (int64_t)INT32_MIN;
      bQWIN_fast[j][i] = (int32_t)bFastLL;

      useDoubleWindow[j][i] = false;
    }
  }
}

// Function to precompute the coefficients
void precomputeCoefficients_OLD() {
  for (int j = 0; j < NUM_OSCILLATORS; j++) {
    // Precompute precise float quadratic coefficients (original model)
    for (int i = 0; i < ampCompTableSize - 2; i++) {
      // Detect sentinel points and synthesize a reasonable third point near the cap
      if (ampCompFrequencyArray[j][i + 1] >= AMP_COMP_SENTINEL_FREQ_Q) {
        ampCompFrequencyArray[j][i + 1] = AMP_COMP_MAX_HZ_Q;
        ampCompArray[j][i + 1] = ampCompArray[j][i];
      }
      if (ampCompFrequencyArray[j][i + 2] >= AMP_COMP_SENTINEL_FREQ_Q) {
        int32_t f0 = ampCompFrequencyArray[j][i];
        int32_t f1 = ampCompFrequencyArray[j][i + 1];
        int32_t step = f1 - f0;
        if (step <= 0) step = (AMP_COMP_MAX_HZ_Q - f1);
        if (step <= 0) step = 1;
        int32_t synthetic = f1 + step;
        if (synthetic > AMP_COMP_MAX_HZ_Q) synthetic = AMP_COMP_MAX_HZ_Q;
        ampCompFrequencyArray[j][i + 2] = synthetic;
        ampCompArray[j][i + 2] = ampCompArray[j][i + 1];
      }
      // Detect onset of plateau (multiple consecutive DIV_COUNTER values)
      bool plateauWin = (ampCompArray[j][i + 1] >= DIV_COUNTER && ampCompArray[j][i + 2] >= DIV_COUNTER);

      double x0f = (double)ampCompFrequencyArray[j][i]     / (double)(1u << FREQ_FRAC_BITS);
      double x1f = (double)ampCompFrequencyArray[j][i + 1] / (double)(1u << FREQ_FRAC_BITS);
      double x2f = (double)ampCompFrequencyArray[j][i + 2] / (double)(1u << FREQ_FRAC_BITS);
      double y0f = (double)ampCompArray[j][i];
      double y1f = (double)ampCompArray[j][i + 1];
      double y2f = (double)ampCompArray[j][i + 2];

      if (plateauWin) {
        double xCap = (double)AMP_COMP_MAX_HZ;
        double xMid = (x0f + xCap) * 0.5;
        double yMid = (y0f + (double)DIV_COUNTER) * 0.5;
        ampCompFrequencyArray[j][i + 1] = (int32_t)llround(xMid * (double)(1u << FREQ_FRAC_BITS));
        ampCompArray[j][i + 1] = (uint16_t)llround(yMid);
        ampCompFrequencyArray[j][i + 2] = AMP_COMP_MAX_HZ_Q;
        ampCompArray[j][i + 2] = (uint16_t)DIV_COUNTER;
        x1f = xMid;
        x2f = xCap;
        y1f = yMid;
        y2f = (double)DIV_COUNTER;
      }
      // Solve for a, b, c in y = a*x^2 + b*x + c using three points
      double denom = (x0f - x1f) * (x0f - x2f) * (x1f - x2f);
      if (denom == 0.0) denom = 1.0;
      double aVal = (x2f * (y1f - y0f) + x1f * (y0f - y2f) + x0f * (y2f - y1f)) / denom;
      double bVal = (x2f * x2f * (y0f - y1f) + x1f * x1f * (y2f - y0f) + x0f * x0f * (y1f - y2f)) / denom;
      double cVal = (x1f * x2f * (x1f - x2f) * y0f + x2f * x0f * (x2f - x0f) * y1f + x0f * x1f * (x0f - x1f) * y2f) / denom;
      aCoeff[j][i] = (float)aVal;
      bCoeff[j][i] = (float)bVal;
      cCoeff[j][i] = (float)cVal;
      aCoeffD[j][i] = aVal;
      bCoeffD[j][i] = bVal;
      cCoeffD[j][i] = cVal;
      double slopeSpan = fabs((y2f - y0f) / (x2f - x0f));
      useDoubleWindow[j][i] =
        (fabs(aCoeffD[j][i]) > 1e6) ||
        (fabs(bCoeffD[j][i]) > 1e3) ||
        (slopeSpan > 0.4);
    }
    // Precompute per-window normalized quadratic coefficients (32-bit runtime)
    for (int i = 0; i < ampCompTableSize - 2; i++) {
      double x0f = (double)ampCompFrequencyArray[j][i]     / (double)(1u << FREQ_FRAC_BITS);
      double x1f = (double)ampCompFrequencyArray[j][i + 1] / (double)(1u << FREQ_FRAC_BITS);
      double x2f = (double)ampCompFrequencyArray[j][i + 2] / (double)(1u << FREQ_FRAC_BITS);
      double y0f = (double)ampCompArray[j][i];
      double y1f = (double)ampCompArray[j][i + 1];
      double y2f = (double)ampCompArray[j][i + 2];
      double dx02f = x2f - x0f;
      if (dx02f == 0.0) dx02f = 1.0;
      double t1 = (x1f - x0f) / dx02f; // in (0,1)
      double d20 = y2f - y0f;
      double d10 = y1f - y0f;
      double denom = (t1 * t1 - t1);
      if (denom == 0.0) denom = 1.0;
      double aN = (d10 - d20 * t1) / denom;
      double bN = d20 - aN;
      // Store for runtime 32-bit evaluation
      xBaseWIN[j][i] = (int32_t)ampCompFrequencyArray[j][i];
      dxWIN[j][i] = (int32_t)(ampCompFrequencyArray[j][i + 2] - ampCompFrequencyArray[j][i]);
      if (dxWIN[j][i] == 0) dxWIN[j][i] = 1;
      // Reciprocal of dx in Q28 for fast t computation without underflow
      invDxWIN_q28[j][i] = (uint32_t)llround((double)268435456.0 / (double)dxWIN[j][i]); // 2^28 / dx
      // Coefficients in Q(T_FRAC) for 32-bit eval (stored as int32_t, no saturation)
      aQWIN[j][i] = (int64_t)llround((double)aN * (double)(1LL << T_FRAC));
      bQWIN[j][i] = (int64_t)llround((double)bN * (double)(1LL << T_FRAC));
      // (removed legacy shift/scale and integer coeff precompute)
      // c in y units, clamp to valid range
      int32_t ctmp = (int32_t)lrintf(y0f);
      if (ctmp < 0) ctmp = 0;
      if (ctmp > (int32_t)DIV_COUNTER) ctmp = DIV_COUNTER;
      cQWIN[j][i] = (uint16_t)ctmp;
    }
  }
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
// 20.60, 88,
// 21.83, 164,
// 32.70, 800,
// 43.65, 1184,
// 65.41, 1848,
// 87.31, 2288,
// 130.81, 2928,
// 174.61, 3388,
// 261.63, 4076,
// 349.23, 4588,
// 523.25, 5288,
// 698.46, 5796,
// 1046.50, 6484,
// 1396.91, 6956,
// 2093.00, 7576,
// 4186.00, 8656,
// 8372.01, 9700,
// 30000.00, 9999,
// 50000.00, 9999,
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