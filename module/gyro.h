#ifndef _GYRO_H_
#define _GYRO_H_

#include "stm32f10x.h"

#define PI						3.1415926535898f
#define DEG_TO_RAD              (PI / 180.0f)
#define RAD_TO_DEG				(180.0f / PI) 
#define GYRO_RAW_TO_RAD_S		(GYRO_RAW_TO_DEG_S * DEG_TO_RAD)

typedef enum{
	SET_GYRO_DEG_S = 0,
	SET_GYRO_RAD_S,
}E_SetGyroInfoOption;

typedef struct
{
	short x;
	short y;
	short z;
}T_GyroShort;

typedef struct
{
	float x;
	float y;
	float z;
}T_GyroFloat;

typedef struct{
    T_GyroShort rawData;		//陀螺仪原始数据

    T_GyroFloat degS;		//陀螺仪转换成度的数据
    T_GyroFloat radS;		//陀螺仪转换成弧度的数据
}S_Gyro;

void GetGyroInfo(S_Gyro *exGyro);
uint8_t SetGyroInfo(S_Gyro *exGyro, E_SetGyroInfoOption option);
uint8_t GyroInit(void);
void GyroDataHandle(void);
uint8_t GetGyroSensorInitStatus(void);
#endif
