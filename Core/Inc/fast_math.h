#ifndef FAST_MATH_H
#define FAST_MATH_H

#include <stdint.h>
#include "stm32g431xx.h"

typedef struct { float alpha, beta; } FM_AlphaBeta;
typedef struct { float d, q; } FM_DQ;

void  FM_CordicInit(void);
uint8_t FM_CordicSelfTest(float *sin45_out, float *cos45_out);
float FM_WrapPi(float x);
void  FM_CordicSinCos(float theta_rad, float *s, float *c);
float FM_CordicAtan2(float y, float x);
float FM_FastSqrt(float x);

static inline float FM_Clamp(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

#endif
