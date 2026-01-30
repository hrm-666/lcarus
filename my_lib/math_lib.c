#include "math_lib.h"

//数据限幅
int ConstrainInt(int input, int min, int max)
{
	if(input < min){
		input = min;
	}
	else if(input > max){
		input = max;
	}
	
	return input;
}

float ConstrainFloat(float input, float min, float max)
{
	if(input < min){
		input = min;
	}
	else if(input > max){
		input = max;
	}
	
	return input;
}

/*快速开平方求倒*/
float FastSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//取绝对值
float MyAbs(float f)
{
	if (f >= 0.0f)
	{
		return f;
	}

	return -f;
}


uint16_t  ThrottleLimit(uint16_t thr_in, uint16_t thr_min, uint16_t thr_max)
{
	if(thr_in<thr_min)	thr_in = thr_min;
	if(thr_in>thr_max)	thr_in = thr_max;
	
	return thr_in;
}

float direction_to_zero(float in_dat, float min_dat, float max_dat)
{
    if(in_dat>min_dat&&in_dat<max_dat)  
        in_dat = 0;
    
    return in_dat;
}

int16_t DirecteToValue(int16_t inData, int16_t minData, int16_t maxData, int16_t value) 
{
    if(inData > minData && inData < maxData){
        inData = value;
    }

    return inData;
}


