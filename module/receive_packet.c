#include "receive_packet.h"
#include "si24r1.h"
#include "math_lib.h"
#include "controller.h"
#include "pair_freq.h"
#include "board_config.h"
#include "transmit_packet.h"
#include "my_lib.h"
#include "log_lib.h"
#include <rtthread.h>
#include <string.h>
#include "task_manage.h"
#include "plane.h"
#include "queue_lib.h"
#include "imu.h"
#include "crc_lib.h"

#define RECEIVE_BUFF_LENGHT		256		//无线通信模块接收缓冲区长度
static S_Remote remote = {0};		//遥控数据

uint8_t receiveBuff[RECEIVE_BUFF_LENGHT] = {0};		//无线通信模块接收缓冲区
static S_Queue receiveQueue = {0};		//接收队列

static rt_mutex_t remote_info_mutex = RT_NULL;

void ReceiveData(void);
void AnalysisHandle(void);

void ReceiveHandle(void)
{
	ReceiveData();
	AnalysisHandle();
}

void ReceiveInit(void)
{
	QueueInit(&receiveQueue, receiveBuff, RECEIVE_BUFF_LENGHT);
	
	//创建互斥锁
	remote_info_mutex = rt_mutex_create("remote_info_mutex", RT_IPC_FLAG_PRIO);
	if (remote_info_mutex == RT_NULL){
		LogError("create dynamic remote_info_mutex failed.\n");
		return;
	}

}

void ReceiveData(void)
{
	S_Plane plane;
	uint8_t receiveLen = 0;
	uint8_t receiveBuf[32] = {0};
	uint8_t ret = 0;
	
	GetPlaneInfo(&plane);
	
	if(plane.pair == PAIR_DONE){		//判断是否已对频
		receiveLen = Si24r1_RxPacket(receiveBuf);
//		LogDma("receiveLen: %d", receiveLen);
		if(receiveLen > 0){		//判断是否接收到数据
			for(int i = 0; i < receiveLen; i++){
				ret = EnQueue(&receiveQueue, receiveBuf[i]);
				if(ret == 1){
					LogError("receiveQueue is full");
					break;
				}
			}
		}
	}
}

//数据包解析
void AnalysisHandle(void)
{
	S_Plane plane;
	static uint8_t signalLostCount = 0;		//信号丢失计数
	int ret = -1;
	uint8_t receiveData[32] = {0};
	uint8_t data = 0;
	uint8_t sendData[32] = {0};
	uint8_t sendLen = 0;
	
	GetPlaneInfo(&plane);
	
	if(plane.pair != PAIR_DONE){		//判断是否已对频
		return;
	}
	
	for(int i = 0; i < 32; i++){		//一包数据最多32字节
		if(QueueLength(&receiveQueue) > 0){
//			LogInfo("data: %x", data);
			if(DeQueue(&receiveQueue, &data) == 1){
				LogError("dequeue fail");
				return;
			}
			
			ret = AnalysisData(data, receiveData);
			if(ret == 0){		//解析正确
				break;
			}
		} else {
			break;
		}
	}
	
	if(ret == 0){
		signalLostCount = 0;
		plane.signal = SIGNAL_NORMAL;
		SetPlaneInfo(&plane, SET_PLANE_SIGNAL);
		
		if(receiveData[0] == CMD_ROCKER_DATA){
			rt_mutex_take(remote_info_mutex, RT_WAITING_FOREVER);
			
			remote.throttle = receiveData[2]<<8 | receiveData[1];		//油门
			remote.throttle = ThrottleLimit(remote.throttle,0,950);		//油门限制范围
			remote.throttle = direction_to_zero(remote.throttle,0,5);
			remote.throttle = DirecteToValue(remote.throttle, 480, 520, 500);
			
			remote.pit = (int16_t)(((int8_t)receiveData[3]) - 50);		//俯仰舵向
			remote.pit = direction_to_zero(remote.pit,-5,5);		//消除抖动
			
			remote.roll = -(int16_t)(((int8_t)receiveData[4]) - 50);		//横滚舵向
			remote.roll = direction_to_zero(remote.roll,-5,5);
			
			remote.yaw =  (int16_t)(((int8_t)receiveData[5]) - 50);		//偏航舵向
			remote.yaw = direction_to_zero(remote.yaw,-5,5);
			
			remote.flyMode = (E_FlyMode)receiveData[6];		//左功能按键
			plane.flyMode = (E_ControlMode)remote.flyMode;			
			SetPlaneInfo(&plane, SET_PLANE_CONTROL_MODE);
			
			remote.emergencyLock = (E_EmergencyLock)receiveData[7];
			
			rt_mutex_release(remote_info_mutex);
			
			SendPlaneData();
		}				
		else if(receiveData[0] == CMD_ACC_CAL){

			if(receiveData[1] == 1){
				LogInfo("acc calibrate start");
				plane.accCalStatus = ACC_CAL_START;
				SetPlaneInfo(&plane, SET_PLANE_ACC_CAL_STATUS);			
			} 
			else if(receiveData[1] == 2){
				LogInfo("acc calibrate cancel");
				plane.accCalStatus = ACC_CAL_CANCEL;
				SetPlaneInfo(&plane, SET_PLANE_ACC_CAL_STATUS);
			}
			
			sendData[sendLen++] = CMD_ACC_CAL;
			sendData[sendLen++] = plane.accCalStatus;
			sendData[sendLen++] = plane.accCalStep;
			sendData[sendLen++] = plane.accCalDataRead;
			StoreToTransmitQueue(sendData, sendLen);
		}
		else if(receiveData[0] == CMD_GYRO_CAL){
			if(receiveData[1] == 1 && plane.gyroCalStatus != GYRO_CAL_DATA_READING){
				LogInfo("gyro calibrate start");
				plane.gyroCalStatus = GYRO_CAL_START;
				SetPlaneInfo(&plane, SET_PLANE_GYRO_CAL_STATUS);
			}
			else if(receiveData[1] == 2){
				LogInfo("gyro calibrate cancel");
				plane.gyroCalStatus = GYRO_CAL_CANCEL;
				SetPlaneInfo(&plane, SET_PLANE_GYRO_CAL_STATUS);
			}
			
			sendData[sendLen++] = CMD_GYRO_CAL;
			sendData[sendLen++] = plane.gyroCalStatus;
			StoreToTransmitQueue(sendData, sendLen);
		}
		else if(receiveData[0] == CMD_COMPASS_CAL){
			if(receiveData[1] == 1){
				LogInfo("compass calibrate start");
				plane.compassCalStatus = COMPASS_CAL_START;
				SetPlaneInfo(&plane, SET_PLANE_COMPASS_CAL_STATUS);
			}
			else if(receiveData[1] == 2){
				LogInfo("compass calibrate cancel");
				plane.compassCalStatus = COMPASS_CAL_CANCEL;
				SetPlaneInfo(&plane, SET_PLANE_COMPASS_CAL_STATUS);
			}
			
			sendData[sendLen++] = CMD_COMPASS_CAL;
			sendData[sendLen++] = plane.compassCalStatus;
			sendData[sendLen++] = plane.compassCalStep;
			sendData[sendLen++] = plane.compassCalDataRead;
			StoreToTransmitQueue(sendData, sendLen);
		}
		else if(receiveData[0] == CMD_IMU){
			S_Imu imu = {0};
			
			GetImuInfo(&imu);
			
			sendData[sendLen++] = CMD_IMU;
			
			int16_t temp = 0;

			temp = (int16_t)(imu.roll * 100);
			sendData[sendLen++] = (uint8_t)(temp >> 8);
			sendData[sendLen++] = (uint8_t)(temp);
			
			temp = (int16_t)(imu.pitch * 100);
			sendData[sendLen++] = (uint8_t)(temp >> 8);
			sendData[sendLen++] = (uint8_t)(temp);
			
			temp = (int16_t)(imu.yaw * 100);
			sendData[sendLen++] = (uint8_t)(temp >> 8);
			sendData[sendLen++] = (uint8_t)(temp);
			
			StoreToTransmitQueue(sendData, sendLen);		//存入发送队列
		}
	} else {
		if(signalLostCount < 200){		//约2秒
				signalLostCount++;		//信号丢失计数
				plane.signal = SIGNAL_NORMAL;
				SetPlaneInfo(&plane, SET_PLANE_SIGNAL);
		}
		else if(signalLostCount == 200){	
			plane.signal = SIGNAL_LOST;		//超时信号丢失
//			LogInfo("lost");
			SetPlaneInfo(&plane, SET_PLANE_SIGNAL);
			if(remote.throttle > 10){	
				remote.throttle -= 1;		//信号丢失减速处理                
			}
		}
			
	}
	
}

int AnalysisData(uint8_t data, uint8_t *receiveData)
{
	static uint8_t analysisState = 0;
	static uint16_t dataLen = 0;
	static uint8_t receiveBuf[128] = {0};
	static uint8_t bufLen = 0;
	static uint8_t crc = 0;
	
	switch(analysisState){
		case 0:
			if(data == 0xAA){
				analysisState = 1;
			} else {
				analysisState = 0;
			}
			break;
		case 1:
			dataLen = (uint16_t)data << 8;		//有效数据长度高字节
			analysisState = 2;
			break;
		case 2:
			dataLen = dataLen | data;		//有效数据长度低字节
			analysisState = 3;
			break;
		case 3:
			if( bufLen < dataLen){		//有效数据长度
				receiveBuf[bufLen++] = data;
			}
			
			if(bufLen == dataLen){
				analysisState = 4;
			}
			break;
		case 4:
			crc = data;		//crc校验位
			analysisState = 5;
			break;
		case 5:
			analysisState = 0;
			if(data == 0xAD){		//包尾
				if(crc == crc8_maxim(receiveBuf, dataLen)){
					memcpy(receiveData, receiveBuf, bufLen);
					bufLen = 0;
					return 0;
				}
			}
			bufLen = 0;
			break;
		default:
			break;
	}
	
	return -1;
}

void GetRemoteInfo(S_Remote *exRemote)
{
	if(rt_mutex_take(remote_info_mutex, RT_WAITING_NO) == RT_EOK){
		StructCopy((uint8_t *)&remote, (uint8_t *)exRemote, sizeof(remote));
		rt_mutex_release(remote_info_mutex);
	}
}

