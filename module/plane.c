#include "plane.h"
#include "transmit_packet.h"
#include "receive_packet.h"
#include "imu.h"
#include "rtthread.h"
#include "my_lib.h"
#include "log_lib.h"
#include <rtthread.h>
#include "task_manage.h"

static rt_mutex_t plane_mutex = RT_NULL;

static S_Plane plane = {
	.lock = (Lock)LOCK,
	.signal = (Signal)SIGNAL_LOST,		//信号状态
	.power = (Power)POWER_NORMAL,	//电量状态
	.pair = (PairStatus)PAIR_NOT,		//配对状态
	.accCalStatus = (AccCal)ACC_CAL_NOT,		//加速度计校准状态
	.accCalStep = (E_AccCalStep)ACC_CAL_SETP_BEGIN,		//加速度校准步骤
	.gyroCalStatus = (GyroCal)GYRO_CAL_NOT,		//陀螺仪校准状态
	.voltage = (float)0,		//电压值
	.flyMode = (E_ControlMode)CONTROL_MODE_ALTITUDE,
};

void PlaneInit(void)
{
	//创建互斥锁
	plane_mutex = rt_mutex_create("plane_mutex", RT_IPC_FLAG_PRIO);
	if (plane_mutex == RT_NULL){
		LogError("create dynamic plane_mutex failed.\n");
		return;
	}
}

void SendPlaneData(void)
{
	uint8_t sendBuf[27] = {0};		//还要封包处理，最大可用字节为27
	uint8_t buffLen = 0;	
	
	sendBuf[buffLen++] = CMD_ROCKER_DATA;
	sendBuf[buffLen++] = plane.lock;
	sendBuf[buffLen++] = plane.signal;
	sendBuf[buffLen++] = plane.power;
	sendBuf[buffLen++] = plane.pair;
	sendBuf[buffLen++] = plane.flyMode;
	sendBuf[buffLen++] = (uint8_t)((uint16_t)(plane.voltage * 100) >> 8);
	sendBuf[buffLen++] = (uint8_t)(plane.voltage * 100);
	sendBuf[buffLen++] = (uint8_t)((uint16_t)(plane.relativeAltitude * 10) >> 8);
	sendBuf[buffLen++] = (uint8_t)(plane.relativeAltitude * 10);
	
	StoreToTransmitQueue(sendBuf, buffLen);
}

//设置plane数据
uint8_t SetPlaneInfo(S_Plane *exPlane, E_SetPlaneInfoList planeInfo)
{
	rt_mutex_take(plane_mutex, RT_WAITING_FOREVER);

	switch (planeInfo){
		case SET_PLANE_LOCK:
			plane.lock = exPlane->lock;
			break;
		case SET_PLANE_SIGNAL:
			plane.signal = exPlane->signal;
			break;
		case SET_PLANE_POWER:
			plane.power = exPlane->power;
			break;
		case SET_PLANE_PAIR:
			plane.pair = exPlane->pair;
			break;
		case SET_PLANE_ACC_CAL_STATUS:
			plane.accCalStatus = exPlane->accCalStatus;
			break;
		case SET_PLANE_ACC_CAL_STEP:
			plane.accCalStep = exPlane->accCalStep;
			break;
		case SET_PLANE_ACC_DATA_READ_STATUS:
			plane.accCalDataRead = exPlane->accCalDataRead;
			break;
		case SET_PLANE_GYRO_CAL_STATUS:
			plane.gyroCalStatus = exPlane->gyroCalStatus;
			break;
		case SET_PLANE_VOLTAGE:
			plane.voltage = exPlane->voltage;
			break;
		case SET_PLANE_CONTROL_MODE:
			plane.flyMode = exPlane->flyMode;
			break;
		case SET_PLANE_RELATIVE_ALTITUDE:
			plane.relativeAltitude = exPlane->relativeAltitude;
			break;
		case SET_PLANE_COMPASS_CAL_STATUS:
			plane.compassCalStatus = exPlane->compassCalStatus;
			break;
		case SET_PLANE_COMPASS_CAL_STEP:
			plane.compassCalStep = exPlane->compassCalStep;
			break;
		case SET_PLANE_COMPASS_CAL_READ_STATUS:
			plane.compassCalDataRead = exPlane->compassCalDataRead;
			break;
		default:
			LogError("Option does not exist");
			break;
	}

	rt_mutex_release(plane_mutex);

	return 0;
}

//获取plane数据
void GetPlaneInfo(S_Plane *exPlane)
{
	StructCopy((uint8_t *)&plane, (uint8_t *)exPlane, sizeof(plane));
}
