#ifndef __AMP_COMP_H__
#define __AMP_COMP_H__

#include "include_all.h"

static constexpr int ampCompTableSize = 22;

int32_t freq_to_amp_comp_array[352];
uint8_t ampCompArraySize = FSVoiceDataSize / 4;

// Arrays to store the precomputed coefficients
float aCoeff[NUM_OSCILLATORS][ampCompTableSize - 2];
float bCoeff[NUM_OSCILLATORS][ampCompTableSize - 2];
float cCoeff[NUM_OSCILLATORS][ampCompTableSize - 2];

int32_t ampCompFrequencyArray[NUM_OSCILLATORS][ampCompTableSize];
int32_t ampCompArray[NUM_OSCILLATORS][ampCompTableSize];

// Function to precompute the coefficients
void precomputeCoefficients() {
  for (int j = 0; j < NUM_OSCILLATORS; j++) {
    for (int i = 0; i < ampCompTableSize - 2; i++) {
      long x0 = ampCompFrequencyArray[j][i];
      long x1 = ampCompFrequencyArray[j][i + 1];
      long x2 = ampCompFrequencyArray[j][i + 2];
      long y0 = ampCompArray[j][i];
      long y1 = ampCompArray[j][i + 1];
      long y2 = ampCompArray[j][i + 2];

      // Calculate the coefficients of the quadratic polynomial
      aCoeff[j][i] = (float)(y2 - (x2 * (y1 - y0) + x1 * y0 - x0 * y1) / (x1 - x0)) / (x2 * (x2 - x1 - x0) + x1 * x0);
      bCoeff[j][i] = (float)(y1 - y0 - aCoeff[j][i] * (x1 * x1 - x0 * x0)) / (x1 - x0);
      cCoeff[j][i] = y0 - aCoeff[j][i] * x0 * x0 - bCoeff[j][i] * x0;
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