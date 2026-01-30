#ifndef _acc_cal_h_
#define _acc_cal_h_

#include "stm32f10x.h"

//加速计校准步骤
typedef enum{
	ACC_CAL_SETP_BEGIN = 0,
	ACC_CAL_SETP_UP,
	ACC_CAL_SETP_DOWN,
	ACC_CAL_SETP_FORWARD,
	ACC_CAL_SETP_BACK,
	ACC_CAL_SETP_LEFT,
	ACC_CAL_SETP_RIGHT,
	ACC_CAL_SETP_SAVE,
	ACC_CAL_SETP_DONE,
}E_AccCalStep;

//尺度因子
typedef struct{
	float x;
	float y;
	float z;
}S_K;

//零偏误差
typedef struct{
	float x;
	float y;
	float z;
}S_B;

void getAccData(E_AccCalStep calStep);
int AccCalibrate(E_AccCalStep *calStep);
uint8_t GetAccCalibrateData(float *accX,float *accY,float *accZ);
uint8_t AccCalibrate_Main(void);
void AccCalibrateHandle(void);
#endif

