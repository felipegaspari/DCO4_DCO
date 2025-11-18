#ifndef __PID_H__
#define __PID_H__

#include <PID_v1.h>

// PID controller variables used by legacy calibration routines:
//  - PIDSetpoint: desired error (usually 0).
//  - PIDInput: measured error (from duty/frequency measurements).
//  - PIDOutput: control output (e.g. target frequency or range-PWM).
double PIDSetpoint = 0, PIDInput, PIDOutput;

// Legacy tuning presets (kept for reference).
// double aggKp=0.01, aggKi=0.06, aggKd=0.0012;
// double midKp=0.008, midKi=0.05, midKd=0.0009;
// double consKp=0.006, consKi=0.04, consKd=0.0007;

// Base scaling factor for the active PID gains.
double PIDKMultiplier = 2;
// Aggressive, medium, and conservative PID gains (currently not dynamically switched).
double aggKp = 0.0008 * PIDKMultiplier, aggKi = 0.005 * PIDKMultiplier, aggKd = 0.000006 * PIDKMultiplier;
double midKp = 0.0007 * PIDKMultiplier, midKi = 0.004 * PIDKMultiplier, midKd = 0.000005 * PIDKMultiplier;
double consKp = 0.0006 * PIDKMultiplier, consKi = 0.003 * PIDKMultiplier, consKd = 0.000004 * PIDKMultiplier;

// Threshold for how close to the target we need to be (in error units)
// before declaring a calibration step “good enough”.
double PIDMinGap;
// Number of consecutive iterations where the PID gap has been below PIDMinGap.
uint8_t PIDMinGapCounter = 0;

// Best (smallest) gap seen so far and the corresponding candidate PWM.
double bestGap;
uint16_t bestCandidate;

// Extra multipliers used in some PID-based search routines.
double PIDTuningMultiplier = 1;
double PIDTuningMultiplierKi = 1;

// Dynamic bounds and heuristic “expected” amplitude for the PID output.
double PIDOutputLowerLimit, PIDOutputHigherLimit, PIDLimitsFormula;
// Target interval between PID compute steps (microseconds-based logic in code).
float sampleTime;

// Timestamp of the last PID computation, used to enforce sampleTime.
unsigned long PIDComputeTimer;

// Index into calibrationData[] updated by PID-based calibration.
byte arrayPos;

// Specify the links and initial tuning parameters for the PID controller.
PID myPID(&PIDInput, &PIDOutput, &PIDSetpoint, consKp, consKi, consKd, P_ON_E, REVERSE);

#endif