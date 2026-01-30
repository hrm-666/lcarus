#ifndef _ACC_H_
#define _ACC_H_

#include "stm32f10x.h"

typedef struct{
	short x;
	short y;
	short z;
}S_AccShort;

typedef struct{
	float x;
	float y;
	float z;
}S_AccFloat;

typedef struct 
{
	S_AccShort rawData;		//加速度原始数据
	S_AccFloat gravity;			//加速转换成重力G的数据
}S_Acc;

void GetAccInfo(S_Acc *exAcc);
uint8_t AccInit(void);
void AccDataHandle(void);
uint8_t GetAccSensorInitStatus(void);
#endif
