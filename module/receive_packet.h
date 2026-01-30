#ifndef _parse_packet_h_
#define _parse_packet_h_

#include "si24r1.h"
#include "stm32f10x.h"

typedef enum{
	CMD_ROCKER_DATA = 0xA3,		//遥感值
	CMD_ACC_CAL = 0xA4,		//加速计校准
	CMD_GYRO_CAL = 0xA5,		//陀螺仪校准
	CMD_COMPASS_CAL = 0xA6,		//磁力计校准
	CMD_IMU = 0xA7,		//请求陀螺仪数据
}E_Cmd;

typedef enum{
	FLY_MODE_ATTITUDE = 0,      //姿态模式
	FLY_MODE_ALTITUDE,      //定高模式
	FLY_MODE_POSITION,      //位置模式
}E_FlyMode;

typedef enum{
	EMERGENCY_LOCK_FALSE = 0,
	EMERGENCY_LOCK_TRUE,			//紧急锁定使能
}E_EmergencyLock;

typedef struct{
	uint16_t throttle;		//油门
	int8_t pit;		//俯仰舵向
	int8_t roll;		//横滚舵向
	int8_t yaw;		//偏航舵向
	E_FlyMode flyMode;		//飞行模式
	E_EmergencyLock emergencyLock;		//紧急锁桨
	E_Cmd cmd;		//命令，不同命令请求不同数据
}S_Remote;

void GetRemoteInfo(S_Remote *exRemote);
void ReceiveInit(void);
int AnalysisData(uint8_t data, uint8_t *receiveData);
void ReceiveHandle(void);
void AnalysisHandle(void);
void TransmitHandle(void);
#endif

