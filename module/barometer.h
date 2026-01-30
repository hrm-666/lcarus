#ifndef _BAROMETER_H_
#define _BAROMETER_H_

#include "stm32f10x.h"

typedef struct{
  float pressure;		//气压
	float temperature;		//温度
	float height;		//海拔高度
	float relativeAltitude;		//与起飞点的相对高度
	float verticalVel;		//垂直速度
}S_Barometer;

void Barometer_Main(void);
void GetBarometerInfo(S_Barometer *exBaro);
uint8_t GetBarometerSensorInitStatus(void);
#endif
