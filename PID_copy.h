// #ifndef __PID_H__
// #define __PID_H__

// #include <PID_v1.h>

// double PIDSetpoint = 0, PIDInput, PIDOutput;

// // double aggKp=0.01, aggKi=0.06, aggKd=0.0012;
// // double midKp=0.008, midKi=0.05, midKd=0.0009;
// // double consKp=0.006, consKi=0.04, consKd=0.0007;

// double PIDKMultiplier = 2;
// double aggKp = 0.0008 * PIDKMultiplier, aggKi = 0.005 * PIDKMultiplier, aggKd = 0.000006 * PIDKMultiplier;
// double midKp = 0.0007 * PIDKMultiplier, midKi = 0.004 * PIDKMultiplier, midKd = 0.000005 * PIDKMultiplier;
// double consKp = 0.0006 * PIDKMultiplier, consKi = 0.003 * PIDKMultiplier, consKd = 0.000004 * PIDKMultiplier;

// double PIDMinGap;
// uint8_t PIDMinGapCounter = 0;

// double bestGap;
// uint16_t bestCandidate;

// double PIDTuningMultiplier = 1;
// double PIDTuningMultiplierKi = 1;

// double PIDOutputLowerLimit, PIDOutputHigherLimit, PIDLimitsFormula;
// float sampleTime;

// unsigned long PIDComputeTimer;

// byte arrayPos;

// //Specify the links and initial tuning parameters
// PID myPID(&PIDInput, &PIDOutput, &PIDSetpoint, consKp, consKi, consKd, P_ON_E, REVERSE);

// #endif