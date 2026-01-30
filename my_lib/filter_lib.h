#ifndef _FILTER_H_
#define _FILTER_H_

#include "stm32f10x.h"

typedef struct{
	float fs;		//采样频率
	float fc;		//截止频率
	float lastOutput;		//上一次的输出，初始化为0即可
	float alpha;		//平滑系数，由采样频率和截至频率求得，初始化为0即可
}S_ButterworthLpf;

typedef struct{
	uint16_t num;		//允许最大的抖动次数
	uint16_t lastValue;		//上一次的值，初始化为0即可
	uint16_t recordValue;		//记录旧的值，初始化为0即可
	uint16_t count;		//抖动计数，初始化为0即可
}S_DebounceFilter;

float ButterworthLpf(S_ButterworthLpf *filter, float input);
void SmallToLargeSort(uint32_t *arr, uint8_t len);
void FilterTest(void);
float MoveMiddleFilter(uint8_t *num, float *arr, uint8_t n, float input);
float MoveAverageFilter(uint8_t *num, float *arr, uint8_t n, float input);
#endif

