#ifndef _CMPASS_H_
#define _CMPASS_H_

#include "stm32f10x.h"

typedef struct{
	//磁力计数据原始值
	int16_t rawX;
	int16_t rawY;
	int16_t rawZ;
	
	//磁力计数据转换成单位高斯
	float gaussX;
	float gaussY;
	float gaussZ;
}S_Compass;

void CompassHandle(void);
uint8_t CompassInit(void);
uint8_t GetCompassInitStatus(void);
void GetCompassInfo(S_Compass *exCompass);
#endif
