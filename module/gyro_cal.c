#include "gyro_cal.h"
#include "mpu6050.h"
#include "receive_packet.h"
#include "flash.h"
#include "led.h"
#include "systick.h"
#include "transmit_packet.h"
#include "gyro.h"
#include "log_lib.h"
#include <rtthread.h>
#include "math_lib.h"
#include "plane.h"

#define GYRO_CAL_FLASH_ADDR		0x08000000 + 1024*63		//陀螺仪校准数据存储地址
#define GYRO_CAL_DATA_HEAD	0xB1		//陀螺仪校准数据存储帧头

static S_GyroB gyroB = {0};		//零点偏移补偿

uint8_t GyroCalDataReadFromFlash(uint32_t gyroCalAddr,S_GyroB *exB);
void GyroCalDataWriteToFlash(uint32_t gyroCalAddr,S_GyroB *exB);
uint8_t GryoCalibrate(S_GyroB *exGyroB);
void GyroCaliBrateHandle(void);

//陀螺仪校准主入口函数
void GyroCaliBrateHandle(void)
{
	uint8_t ret = 0;
	S_Plane plane;
	S_Gyro gyro = {0};
	static uint16_t gyroCalFailCount = 0;
	static uint8_t flashReadFlag = 0;
	
	GetPlaneInfo(&plane);
	GetGyroInfo(&gyro);
	
	if(plane.gyroCalStatus == GYRO_CAL_START){
		LogInfo("gyro calibrate data reading.");
		plane.gyroCalStatus = GYRO_CAL_DATA_READING;
		SetPlaneInfo(&plane, SET_PLANE_GYRO_CAL_STATUS);
	}
	else if(plane.gyroCalStatus == GYRO_CAL_DATA_READING){

		ret = GryoCalibrate(&gyroB);
		if(ret == 0){
			LogInfo("gyro calibrate done.");
			GyroCalDataWriteToFlash(GYRO_CAL_FLASH_ADDR,&gyroB);		//校准值写入flash
			plane.gyroCalStatus = GYRO_CAL_DONE;		//设置标志位
			SetPlaneInfo(&plane, SET_PLANE_GYRO_CAL_STATUS);		//改变标志位后写入
			rt_thread_mdelay(3000);		//延时，用于指示灯显示一段时间
			
			flashReadFlag = 0;		//复位flash读取标志位，重新读取flash里的值
		}

		gyroCalFailCount++;
		if(gyroCalFailCount == 10){
			LogInfo("gyro calibrate fail.");
			gyroCalFailCount = 0;
			plane.gyroCalStatus = GYRO_CAL_FAIL;
			SetPlaneInfo(&plane, SET_PLANE_GYRO_CAL_STATUS);
			rt_thread_mdelay(3000);		//延时，用于指示灯显示一段时间
			
			flashReadFlag = 0;		//复位flash读取标志位，重新读取flash里的值
		}
	}
	else if(flashReadFlag == 0){		//若已校准，读取校准数据
		flashReadFlag = 1;
		
		ret = GyroCalDataReadFromFlash(GYRO_CAL_FLASH_ADDR,&gyroB);
		if(ret == 0){
			plane.gyroCalStatus = GYRO_CALIBREAD;
			SetPlaneInfo(&plane, SET_PLANE_GYRO_CAL_STATUS);
			
		} else {
			plane.gyroCalStatus = GYRO_CAL_NOT;
			SetPlaneInfo(&plane, SET_PLANE_GYRO_CAL_STATUS);
		}
	}
	
}

//陀螺仪校准
uint8_t GryoCalibrate(S_GyroB *exGyroB)
{
	float tempGryoX = 0;
	float tempGryoY = 0;
	float tempGryoZ = 0;
	float sub;
	uint8_t calCount = 0;
	S_Gyro gyro;
	
	GetGyroInfo(&gyro);
	
	tempGryoX = gyro.degS.x;
	tempGryoY = gyro.degS.y;
	tempGryoZ = gyro.degS.z;

	rt_thread_mdelay(500);		//延时后检测是否数据是否稳定

	GetGyroInfo(&gyro);
	sub = gyro.degS.x - tempGryoX;		//计算差值
	if(MyAbs(sub) < 0.5f){		//差值小于0.5判定为稳定状态
		exGyroB->x = gyro.degS.x;		//采集静态偏移值
		calCount++;
	}
	
	sub = gyro.degS.y - tempGryoY;
	if(MyAbs(sub) < 0.5f){
		exGyroB->y = gyro.degS.y;
		calCount++;
	}
	
	sub = gyro.degS.z - tempGryoZ;
	if(MyAbs(sub) < 0.5f){
		exGyroB->z = gyro.degS.z;
		calCount++;
	}
	
	if(calCount == 3){
		return 0;
	}
	
	return 1;
	  
}

//从Flash读取陀螺仪数据
uint8_t GyroCalDataReadFromFlash(uint32_t gyroCalAddr, S_GyroB *exB)
{
	uint32_t arrTemp[5] = {0};
	uint8_t arrLen = 0;
	uint32_t ret = 0;
	
	STM32_FLASH_Read(gyroCalAddr,arrTemp,5);
	
	ret = *(uint32_t *)(gyroCalAddr);	//读取帧头
	
	arrLen++;		//除去帧头
	if(ret == GYRO_CAL_DATA_HEAD){	//判断帧头
		exB->x = *(float *)&arrTemp[arrLen++];
		exB->y = *(float *)&arrTemp[arrLen++];
		exB->z = *(float *)&arrTemp[arrLen++];
		
		return 0;
		
	} else{
		return 1;
	}
	
}

//陀螺仪校准数据存入到Flash
void GyroCalDataWriteToFlash(uint32_t gyroCalAddr, S_GyroB *exB)
{
	uint8_t arrLen = 0;
	uint32_t arrTemp[5] = {0};

	arrTemp[arrLen++] = GYRO_CAL_DATA_HEAD;		//添加帧头

	arrTemp[arrLen++] = *(uint32_t *)&exB->x;
	arrTemp[arrLen++] = *(uint32_t *)&exB->y;
	arrTemp[arrLen++] = *(uint32_t *)&exB->z;

  FlashWriteSpecifyData(gyroCalAddr,arrTemp,arrLen);
	
}

//获取陀螺仪校准数据
uint8_t GetGyroCalibrateData(float *gyroX,float *gyroY,float *gyroZ)
{
	S_Plane plane;
	
	GetPlaneInfo(&plane);
	
	if(plane.gyroCalStatus == GYRO_CALIBREAD){
		*gyroX = (*gyroX) - gyroB.x;
		*gyroY = (*gyroY) - gyroB.y;
		*gyroZ = (*gyroZ) - gyroB.z;
		
		return 0;
	}
	
	return 1;

}
