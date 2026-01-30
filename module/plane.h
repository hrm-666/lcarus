#ifndef	_PLANE_H_
#define _PLANE_H_

#include "stm32f10x.h"
#include "acc_cal.h"
#include "compass_cal.h"

typedef enum{
	LOCK = 0,
	UNLOCKING,
	UNLOCK,
	LOCKING,
}Lock;//锁状态

typedef enum{
	SIGNAL_LOST = 0,
	SIGNAL_NORMAL,
}Signal;//信号状态

typedef enum{
	POWER_NORMAL = 0,
	POWER_LOWER,
}Power;//电量状态

typedef enum{
	ACC_CAL_NOT = 0,
	ACC_CAL_START,
	ACC_CAL_DONE,
	ACC_CAL_FAIL,
	ACC_CAL_CANCEL,
	ACC_CALIBREAD,
}AccCal;

typedef enum{
	ACC_CAL_DATA_READ_NOT = 0,
	ACC_CAL_DATA_READING,
}AccCalDataRead;

typedef enum{
	GYRO_CAL_NOT = 0,
	GYRO_CAL_START,
	GYRO_CAL_DATA_READING,
	GYRO_CAL_DONE,
	GYRO_CAL_FAIL,
	GYRO_CAL_CANCEL,
	GYRO_CALIBREAD,
}GyroCal;

typedef enum{
	PAIR_NOT = 0,
	PAIR_START,
	PAIR_DONE,
	PAIR_FAIL,
}PairStatus;

typedef enum{
    CONTROL_MODE_ATTITUDE = 0,      //姿态模式
    CONTROL_MODE_ALTITUDE,      //定高模式
    CONTROL_MODE_POSITION,      //位置模式
}E_ControlMode;

typedef enum{
	COMPASS_CAL_NOT = 0,
	COMPASS_CAL_START,
	COMPASS_CAL_DONE,
	COMPASS_CAL_FAIL,
	COMPASS_CAL_CANCEL,
	COMPASS_CALIBREAD,
}E_CompassCal;

typedef enum{
	COMPASS_CAL_DATA_READ_NOT = 0,
	COMPASS_CAL_DATA_READING,
}E_CompassCalDataRead;

typedef enum{
	SET_PLANE_LOCK = 0,
	SET_PLANE_SIGNAL,
	SET_PLANE_POWER,
	SET_PLANE_PAIR,
	SET_PLANE_ACC_CAL_STATUS,
	SET_PLANE_ACC_CAL_STEP,
	SET_PLANE_ACC_DATA_READ_STATUS,
	SET_PLANE_GYRO_CAL_STATUS,
	SET_PLANE_VOLTAGE,
	SET_PLANE_CONTROL_MODE,
	SET_PLANE_RELATIVE_ALTITUDE,
	SET_PLANE_COMPASS_CAL_STATUS,
	SET_PLANE_COMPASS_CAL_STEP,
	SET_PLANE_COMPASS_CAL_READ_STATUS,
}E_SetPlaneInfoList;

//#pragma pack(1)
typedef struct
{
	Lock lock;		//锁状态
	Signal signal;		//信号状态
	Power power;	//电量状态
	PairStatus pair;		//配对状态
	
	AccCal accCalStatus;		//加速度计校准状态
	E_AccCalStep accCalStep;		//加速度校准步骤
	AccCalDataRead	accCalDataRead;		//加速度计校准数据读取状态

	GyroCal gyroCalStatus;		//陀螺仪校准状态
	
	E_ControlMode flyMode;		//飞行模式
	
	E_CompassCal compassCalStatus;	//磁罗盘校准状态
	E_CompassCalStep compassCalStep;		//磁罗盘校准步骤
	E_CompassCalDataRead	compassCalDataRead;		//磁罗盘校准数据读取状态
	
	float voltage;		//电压值
	
	float relativeAltitude;		//相对高度
}S_Plane;

void transmitPacket(void);
void GetPlaneInfo(S_Plane *exPlane);
uint8_t SetPlaneInfo(S_Plane *exPlane, E_SetPlaneInfoList planeInfo);
int SendData(uint8_t *sendData, uint16_t dataLen);
void TransmitInit(void);
uint8_t Crc(uint8_t *Pdata, uint8_t len);
void TempSetPlaneInfo(S_Plane *exPlane);
void SendPlaneData(void);
void PlaneInit(void);
#endif
