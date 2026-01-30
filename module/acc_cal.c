#include "acc_cal.h"
#include "mpu6050.h"
#include "flash.h"
#include "led.h"
#include "systick.h"
#include "transmit_packet.h"
#include "acc.h"
#include "log_lib.h"
#include "math.h"
#include "plane.h"

#define ACC_CAL_FLASH_ADDR		0x08000000 + 1024*62		//加速度计校准数据存储地址
#define ACC_CAL_DATA_HEAD	0xB2		//加速度计校准数据存储帧头

static S_K k = {0};		//尺度因子
static S_B b = {0};		//零点偏移补偿

void AccCalibrateHandle(void);
uint8_t AccCalReadToFlash(uint32_t accCalAddr,S_K *kData,S_B *bData);

//加速度校准处理入口
void AccCalibrateHandle(void)
{
	S_Plane plane;
	int ret = 0;
	static uint16_t accCalFailCount = 0;
	static E_AccCalStep lastCalStep = ACC_CAL_SETP_BEGIN;
	static uint8_t flashReadFlag = 0;

	GetPlaneInfo(&plane);

	if(flashReadFlag == 0) {
		flashReadFlag = 1;
		ret = AccCalReadToFlash(ACC_CAL_FLASH_ADDR, &k, &b);
		if(ret == 0){		//判断从flash读出的值是否正确
			plane.accCalStatus = ACC_CALIBREAD;
			SetPlaneInfo(&plane, SET_PLANE_ACC_CAL_STATUS);
		} else {
			plane.accCalStatus = ACC_CAL_NOT;
			SetPlaneInfo(&plane, SET_PLANE_ACC_CAL_STATUS);
		}
	}
	else if(plane.accCalStatus == ACC_CAL_START){
		LogInfo("acc calibrate start.");
		
		ret = AccCalibrate(&plane.accCalStep);		//开始校准
		SetPlaneInfo(&plane, SET_PLANE_ACC_CAL_STEP);
		if(ret >= ACC_CAL_SETP_UP && ret <= ACC_CAL_SETP_SAVE){
			plane.accCalDataRead = ACC_CAL_DATA_READING;
			SetPlaneInfo(&plane, SET_PLANE_ACC_DATA_READ_STATUS);
		} else {
			plane.accCalDataRead = ACC_CAL_DATA_READ_NOT;
			SetPlaneInfo(&plane, SET_PLANE_ACC_DATA_READ_STATUS);
		}

		if(plane.accCalStep == ACC_CAL_SETP_DONE){
			LogInfo("acc calibrate done.");
			plane.accCalStatus = ACC_CAL_DONE;
			SetPlaneInfo(&plane, SET_PLANE_ACC_CAL_STATUS);
		}

		accCalFailCount++;		//失败计数
		if(plane.accCalStep != lastCalStep){
			lastCalStep = plane.accCalStep;
			accCalFailCount = 0;
		}
		else if(accCalFailCount > 30){
			LogInfo("acc calibrate fail.");
			accCalFailCount = 0;
			plane.accCalStatus = ACC_CAL_FAIL;
			SetPlaneInfo(&plane, SET_PLANE_ACC_CAL_STATUS);
		}
	}
	else if(plane.accCalStatus == ACC_CAL_DONE || plane.accCalStatus == ACC_CAL_FAIL){		//校准完成和失败处理
		rt_thread_mdelay(3000);		//延时，用于指示灯显示一段时间
		plane.accCalStep = ACC_CAL_SETP_BEGIN;		//更新校准步骤
		SetPlaneInfo(&plane, SET_PLANE_ACC_CAL_STEP);
		plane.accCalDataRead = ACC_CAL_DATA_READ_NOT;
		SetPlaneInfo(&plane, SET_PLANE_ACC_DATA_READ_STATUS);
		flashReadFlag = 0;		//复位flash读取标志位，重新读取flash里的值
	}
	else if(plane.accCalStatus == ACC_CAL_CANCEL){		//判断是否取消
		LogInfo("acc calibrate cancel.");
		plane.accCalStep = ACC_CAL_SETP_BEGIN;
		SetPlaneInfo(&plane, SET_PLANE_ACC_CAL_STEP);
		plane.accCalDataRead = ACC_CAL_DATA_READ_NOT;
		SetPlaneInfo(&plane, SET_PLANE_ACC_DATA_READ_STATUS);
		rt_thread_mdelay(3000);		//延时，用于指示灯显示一段时间
		
		flashReadFlag = 0;		//复位flash读取标志位，重新读取flash里的值
	}
	
}

//将K和B的值写如FLASH
void AccCalDataWriteToFlash(uint32_t accCalAddr,S_K *kData,S_B *bData)
{
	uint8_t arrLen = 0;
	uint32_t arrTemp[10] = {0};

	arrTemp[arrLen++] = ACC_CAL_DATA_HEAD;		//添加帧头
	
	arrTemp[arrLen++] = *(uint32_t *)&kData->x;
	arrTemp[arrLen++] = *(uint32_t *)&bData->x;
	
	arrTemp[arrLen++] = *(uint32_t *)&kData->y;
	arrTemp[arrLen++] = *(uint32_t *)&bData->y;
	
	arrTemp[arrLen++] = *(uint32_t *)&kData->z;
	arrTemp[arrLen++] = *(uint32_t *)&bData->z;

  FlashWriteSpecifyData(accCalAddr,arrTemp,arrLen);
	
}

//从FLASH中读出K和B的值
uint8_t AccCalReadToFlash(uint32_t accCalAddr,S_K *kData,S_B *bData)
{
	uint32_t arrTemp[10] = {0};
	uint8_t arrLen = 0;
	uint32_t res = 0;
	
	STM32_FLASH_Read(accCalAddr,arrTemp,10);
	
	res = *(uint32_t *)(accCalAddr);	//读取帧头
	
	arrLen++;		//除去帧头
	if(res == ACC_CAL_DATA_HEAD){	//判断帧头
		kData->x = *(float *)&arrTemp[arrLen++];
		bData->x = *(float *)&arrTemp[arrLen++];

		kData->y = *(float *)&arrTemp[arrLen++];
		bData->y = *(float *)&arrTemp[arrLen++];
		
		kData->z = *(float *)&arrTemp[arrLen++];
		bData->z = *(float *)&arrTemp[arrLen++];
		
		return 0;
	}

	return 1;
}

/*
 * @brief: 加速度计校准
 * @calStep: 校准步骤
 * @return: 0，成功；-1，数据相差过大；2~7，对应校准步骤（E_AccCalStep）
*/
int AccCalibrate(E_AccCalStep *calStep)
{
	static uint8_t calCount = 0;
	S_Acc acc = {0};
	static uint8_t count = 1;
	static float posAccX = 0, nagAccX = 0;
	static float posAccY = 0, nagAccY = 0;
	static float posAccZ = 0, nagAccZ = 0;
	float tempValue = 0;
	float subValue = 0;
	
	GetAccInfo(&acc);
	
	switch(*calStep){
		case ACC_CAL_SETP_DONE: 
			*calStep = ACC_CAL_SETP_BEGIN;
			return *calStep;
		case ACC_CAL_SETP_BEGIN: 
			*calStep = ACC_CAL_SETP_UP;
			calCount = 0;
			count = 0;
			return *calStep;
		case ACC_CAL_SETP_UP:		//正放
			tempValue = acc.gravity.z;
			rt_thread_mdelay(500);		//延时等待稳定
			GetAccInfo(&acc);
			subValue = fabs(tempValue - acc.gravity.z);		//计算两次的差值
			if(acc.gravity.z > 0.8f && subValue < 0.05f){		//差值小于0.05，判定为稳定状态
				count++;		//采集次数计数
				posAccZ += acc.gravity.z;
				if(count == 10){		//采集十次
					posAccZ = posAccZ/10.0f;		//取十次平均值
					*calStep = ACC_CAL_SETP_DOWN;
					calCount++;
					count = 0;
				}
				return *calStep;
			}
			break;
		case ACC_CAL_SETP_DOWN:		//倒放
			tempValue = acc.gravity.z;
			rt_thread_mdelay(500);		//延时等待稳定
			GetAccInfo(&acc);
			subValue = fabs(tempValue - acc.gravity.z);
			if(acc.gravity.z < -0.8f && subValue < 0.05f){
				count++;
				nagAccZ += acc.gravity.z;
				if(count == 10){
					nagAccZ = nagAccZ/10.0f;
					*calStep = ACC_CAL_SETP_BACK;
					calCount++;
					count = 0;
				}
				return *calStep;
			}
			break;
		case ACC_CAL_SETP_BACK:		//后放
			tempValue = acc.gravity.y;
			rt_thread_mdelay(500);		//延时等待稳定
			GetAccInfo(&acc);
			subValue = fabs(tempValue - acc.gravity.y);
			if(acc.gravity.y > 0.90f && subValue < 0.05f){
				count++;
				posAccY += acc.gravity.y;
				if(count == 10){
					posAccY = posAccY/10.0f;
					*calStep = ACC_CAL_SETP_FORWARD;
					calCount++;
					count = 0;
				}
				return *calStep;
			}
			break;
		case ACC_CAL_SETP_FORWARD:		//前放
			tempValue = acc.gravity.y;
			rt_thread_mdelay(500);		//延时等待稳定
			GetAccInfo(&acc);
			subValue = fabs(tempValue - acc.gravity.y);
			if(acc.gravity.y < -0.90f && subValue < 0.05f){
				count++;
				nagAccY += acc.gravity.y;
				if(count == 10){
					nagAccY = nagAccY/10.0f;
					*calStep = ACC_CAL_SETP_LEFT;
					calCount++;
					count = 0;
				}
				return *calStep;
			}
			break;
		case ACC_CAL_SETP_LEFT:		//左放
			tempValue = acc.gravity.x;
			rt_thread_mdelay(500);		//延时等待稳定
			GetAccInfo(&acc);
			subValue = fabs(tempValue - acc.gravity.x);
			if(acc.gravity.x > 0.90f && subValue < 0.05f){
				count++;
				posAccX += acc.gravity.x;
				if(count == 10){
					posAccX = posAccX/10.0f;
					*calStep = ACC_CAL_SETP_RIGHT;
					calCount++;
					count = 0;
				}
				return *calStep;
			}
			break;
		case ACC_CAL_SETP_RIGHT:		//右放
			tempValue = acc.gravity.x;
			rt_thread_mdelay(500);		//延时等待稳定
			GetAccInfo(&acc);
			subValue = fabs(tempValue - acc.gravity.x);
			if(acc.gravity.x < -0.90f && subValue < 0.05f){
				count++;
				nagAccX += acc.gravity.x;
				if(count == 10){
					nagAccX = nagAccX/10.0f;
					*calStep = ACC_CAL_SETP_SAVE;
					calCount++;
					count = 0;
				}
				return *calStep;
			}
			break;
		case ACC_CAL_SETP_SAVE:		//保存数据
			if(calCount == 6){
				
				//求出K和B的值
				k.x = -2/(nagAccX - posAccX);
				b.x = (posAccX + nagAccX)/(nagAccX - posAccX);
				
				k.y = -2/(nagAccY - posAccY);
				b.y = (posAccY + nagAccY)/(nagAccY - posAccY);
				
				k.z = -2/(nagAccZ - posAccZ);
				b.z = (posAccZ + nagAccZ)/(nagAccZ - posAccZ);
				
				AccCalDataWriteToFlash(ACC_CAL_FLASH_ADDR,&k,&b);		//写入flash
				*calStep = ACC_CAL_SETP_DONE;
				calCount = 0;

				return 0;
			}
			break;
		default :
			break;
	}
	
	return -1;
}

//对加速值校准
uint8_t GetAccCalibrateData(float *accX,float *accY,float *accZ)
{
	S_Plane plane;

	GetPlaneInfo(&plane);

	if(plane.accCalStatus == ACC_CALIBREAD){
		*accX = (*accX)*(k.x) + b.x;
		*accY = (*accY)*(k.y) + b.y;
		*accZ = (*accZ)*(k.z) + b.z;

		return 0;
	}

	return 1;
}
