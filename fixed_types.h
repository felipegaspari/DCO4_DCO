#ifndef __FIXED_TYPES_H__
#define __FIXED_TYPES_H__

#include <FixMath.h>

// Common fixed-point aliases for this project
using fx_q16_16_u = UFix<16,16>;
using fx_q16_16_s = SFix<16,16>;
using fx_q0_24_u  = UFix<0,24>;
using fx_q0_24_s  = SFix<0,24>;
using fx_q2_30_s  = SFix<2,30>;
using fx_q0_31_u  = UFix<0,31>;

// Helpful compile-time constants
static constexpr auto FX_ONE_Q24 = UFixAuto<1>().sL<24>();
static constexpr auto FX_ONE_Q16 = UFixAuto<1>().sL<16>();

// Division is not implemented in FixMath: prefer multiplicative inverses
// Precomputed reciprocals (compile-time when possible)
static constexpr auto FX_INV_8192_Q0_24 = UFixAuto<1>().sR<13>(); // 1/8192 in Q0.24

#endif


