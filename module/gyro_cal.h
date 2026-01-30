#ifndef _gyro_cal_h_
#define _gyro_cal_h_

#include "stm32f10x.h"

typedef struct{
	float x;
	float y;
	float z;
}S_GyroB;

uint8_t GetGyroCalibrateData(float *gyroX,float *gyroY,float *gyroZ);
uint8_t GyroCalibrate_Main(void);
void GyroCaliBrateHandle(void);
#endif


