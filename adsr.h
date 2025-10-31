#ifndef __ADSR_H__
#define __ADSR_H__

#define ADSR_1_DACSIZE 4000

#define ARRAY_SIZE 512

#define LIN_TO_EXP_TABLE_SIZE ADSR_1_DACSIZE + 1
uint16_t linToExpLookup[LIN_TO_EXP_TABLE_SIZE];
uint16_t linToLogLookup[LIN_TO_EXP_TABLE_SIZE];
uint16_t maxADSRControlValue = ADSR_1_DACSIZE;

struct Point {
  float x, y;
};


int _curve0_table[ARRAY_SIZE];
int _curve1_table[ARRAY_SIZE];
int _curve2_table[ARRAY_SIZE];
int _curve3_table[ARRAY_SIZE];
int _curve4_table[ARRAY_SIZE];
int _curve5_table[ARRAY_SIZE];
int _curve6_table[ARRAY_SIZE];
int _curve7_table[ARRAY_SIZE];
int *_curve_tables[8] = { _curve0_table, _curve1_table, _curve2_table, _curve3_table, _curve4_table, _curve5_table, _curve6_table, _curve7_table };

#include <ADSR_Bezier_millis.h>

Point bezierCubic(const Point& A, const Point& P1, const Point& P2, const Point& B, float t) {
  float one_minus_t = 1.0f - t;
  float one_minus_t_squared = one_minus_t * one_minus_t;
  float t_squared = t * t;
  float x = one_minus_t_squared * one_minus_t * A.x + 3 * one_minus_t_squared * t * P1.x + 3 * one_minus_t * t_squared * P2.x + t_squared * t * B.x;
  float y = one_minus_t_squared * one_minus_t * A.y + 3 * one_minus_t_squared * t * P1.y + 3 * one_minus_t * t_squared * P2.y + t_squared * t * B.y;
  return { x, y };
}

float findYForX(const Point& A, const Point& P1, const Point& P2, const Point& B, float xTarget, float tol = 1e-5) {
  float tLow = 0.0f;
  float tHigh = 1.0f;
  float tMid;

  while ((tHigh - tLow) > tol) {
    tMid = (tLow + tHigh) / 2.0f;
    Point midPoint = bezierCubic(A, P1, P2, B, tMid);
    if (midPoint.x < xTarget) {
      tLow = tMid;
    } else {
      tHigh = tMid;
    }
  }

  Point resultPoint = bezierCubic(A, P1, P2, B, tMid);
  return resultPoint.y;
}

// void generateBezierArray(Point A, Point B, Point P1, Point P2, uint16_t arraySize, uint16_t (&array)[4096]) {

//   for (int x = 0; x < arraySize; ++x) {
//     float yResult = findYForX(A, P1, P2, B, static_cast<float>(x));

//     array[x] = yResult;
//   }
// }

void adsrCreateTables(float maxVal, int numPoints) {

  Point A = { 0, maxVal };  // Punto inicial
  Point B = { maxVal, 0 };
  Point P1[8] = { { 250, 1500 }, { 840, 1780 }, { 400, 430 }, { 2170, 3610 }, { 400, 1380 }, { 1140, 3750 }, { 200, 2700 }, { 0, 4095 } };
  Point P2[8] = { { 1500, 250 }, { 1160, 210 }, { 920, 420 }, { 3730, 2610 }, { 3830, 2890 }, { 1850, 1080 }, { 720, 3050 }, { 4095, 0 } };

  for (int j = 0; j < 8; j++) {

    float multiplier = (float)(maxVal + 1) / (float)(numPoints - 1);

    // Imprimir los puntos de la curva
    for (float i = 0; i < numPoints; i++) {
      float xTarget = multiplier * i;
      float yResult = findYForX(A, P1[j], P2[j], B, xTarget);

      _curve_tables[j][(int)i] = (int)round(yResult);
    }
  }
}

volatile byte noteStart[NUM_VOICES_TOTAL];
volatile byte noteEnd[NUM_VOICES_TOTAL];

uint16_t ADSR1Level[NUM_VOICES_TOTAL];

static constexpr uint16_t ADSR_1_CC = 4000;

float ADSRMaxLevel = ADSR_1_CC;

uint16_t ADSRMinLevel = 0;

int8_t ADSR3ToOscSelect = 2;

uint16_t ADSR1_attack = 0;
uint16_t ADSR1_decay;
uint16_t ADSR1_sustain;
uint16_t ADSR1_release;

byte ADSR1_curve2Val = 0;

float ADSR1_curve1 = 0.999;
float ADSR1_curve2 = 0.997;

unsigned long tADSR;
unsigned long tADSR_params;

bool ADSRRestart = true;

int16_t ADSR1toDETUNE1;

float ADSR1toDETUNE1_formula;

int16_t ADSR1toPWM;
float ADSR1toPWM_formula;

adsr adsr1_voice_0(ADSR_1_CC, ADSR1_curve1, ADSR1_curve2, false,7,7,7);
adsr adsr1_voice_1(ADSR_1_CC, ADSR1_curve1, ADSR1_curve2, false,7,7,7);
adsr adsr1_voice_2(ADSR_1_CC, ADSR1_curve1, ADSR1_curve2, false,7,7,7);
adsr adsr1_voice_3(ADSR_1_CC, ADSR1_curve1, ADSR1_curve2, false,7,7,7);

//bool OSCPhaseLock = false;

struct ADSRStruct {
adsr adsr1_voice;
};

ADSRStruct ADSRVoices[] = {
{adsr1_voice_0},
{adsr1_voice_1},
{adsr1_voice_2},
{adsr1_voice_3},
// {adsr1_voice_4},
// {adsr1_voice_5},
};

#endif