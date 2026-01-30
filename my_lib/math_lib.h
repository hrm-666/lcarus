#ifndef _MATH_LIB_H
#define _MATH_LIB_H

#include "stm32f10x.h"

float FastSqrt(float x);

float fast_atan2(float y, float x) ;

uint16_t  ThrottleLimit(uint16_t thr_in,uint16_t thr_min,uint16_t thr_max);
float direction_to_zero(float in_dat,float min_dat,float max_dat);
float MyAbs(float f);
int16_t DirecteToValue(int16_t inData, int16_t minData, int16_t maxData, int16_t value);
int ConstrainInt(int input, int min, int max);
float ConstrainFloat(float input, float min, float max);
#endif
