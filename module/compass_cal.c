#include "compass_cal.h"
#include "transmit_packet.h"
#include "log_lib.h"
#include <rtthread.h>
#include "compass.h"
#include "imu.h"
#include <math.h>
#include "string.h"
#include "flash.h"
#include "log_lib.h"
#include "plane.h"

#define COMPASS_CAL_FLASH_ADDR		0x08000000 + 1024*60		//磁力计校准数据存储地址
#define COMPASS_CAL_DATA_HEAD	0xB3		//磁力计校准数据存储帧头

static S_CompassOffset compassOffset = {0};		//磁力计数据校准数据

int CompassCalibrate(E_CompassCalStep *calStep);
uint8_t CompassCalReadToFlash(uint32_t readAddr, S_CompassOffset *compassOffset);

void CompassCalibrateHandle(void)
{
	S_Plane plane;
	int ret = 0;
	static uint16_t compassCalFailCount = 0;
	static E_CompassCalStep lastCalStep = COMPASS_CAL_SETP_BEGIN;
	static uint8_t flashReadFlag = 0;

	GetPlaneInfo(&plane);

	if(flashReadFlag == 0) {
		flashReadFlag = 1;
		ret = CompassCalReadToFlash(COMPASS_CAL_FLASH_ADDR, &compassOffset);
		if(ret == 0){		//判断从flash读出的值是否正确
			plane.compassCalStatus = COMPASS_CALIBREAD;		//磁力计已校准
			SetPlaneInfo(&plane, SET_PLANE_COMPASS_CAL_STATUS);
		} else {
			plane.compassCalStatus = COMPASS_CAL_NOT;		//磁力计未校准
			SetPlaneInfo(&plane, SET_PLANE_COMPASS_CAL_STATUS);
		}
	}
	else if(plane.compassCalStatus == COMPASS_CAL_START){		//校准启动
		LogInfo("compass calibrate start.");
		
		ret = CompassCalibrate(&plane.compassCalStep);		//开始校准
		SetPlaneInfo(&plane, SET_PLANE_COMPASS_CAL_STEP);		//更新校准步骤
		if(ret >= COMPASS_CAL_SETP_HORIZONTAL_ROTATION && ret <= COMPASS_CAL_SETP_SAVE){
			plane.compassCalDataRead = COMPASS_CAL_DATA_READING;
			SetPlaneInfo(&plane, SET_PLANE_COMPASS_CAL_READ_STATUS);
		} else {
			plane.compassCalDataRead = COMPASS_CAL_DATA_READ_NOT;
			SetPlaneInfo(&plane, SET_PLANE_COMPASS_CAL_READ_STATUS);
		}

		if(plane.compassCalStep == COMPASS_CAL_SETP_DONE){
			LogInfo("compass calibrate done.");
			plane.compassCalStatus = COMPASS_CAL_DONE;
			SetPlaneInfo(&plane, SET_PLANE_COMPASS_CAL_STATUS);
		}
		
		//超时判断失败
		compassCalFailCount++;		//失败计数
		if(plane.compassCalStep != lastCalStep){
			lastCalStep = plane.compassCalStep;
			compassCalFailCount = 0;
		}
		else if(compassCalFailCount > 1500){
			compassCalFailCount = 0;
			LogInfo("compass calibrate fail.");
			plane.compassCalStatus = COMPASS_CAL_FAIL;
			SetPlaneInfo(&plane, SET_PLANE_COMPASS_CAL_STATUS);		
		}
	}
	else if(plane.compassCalStatus == COMPASS_CAL_DONE || plane.compassCalStatus == COMPASS_CAL_FAIL){		//校准完成和失败处理
		
		rt_thread_mdelay(3000);		//延时，用于指示灯显示一段时间
		plane.compassCalStep = COMPASS_CAL_SETP_BEGIN;		//重置校准步骤
		SetPlaneInfo(&plane, SET_PLANE_COMPASS_CAL_STEP);
		plane.compassCalDataRead = COMPASS_CAL_DATA_READ_NOT;		//更新数据读取状态
		SetPlaneInfo(&plane, SET_PLANE_COMPASS_CAL_READ_STATUS);
		flashReadFlag = 0;		//复位flash读取标志位，重新读取flash里的值
	}
	else if(plane.compassCalStatus == COMPASS_CAL_CANCEL){		//校准取消
		LogInfo("compass calibrate cancel.");
		
		plane.compassCalStep = COMPASS_CAL_SETP_BEGIN;		//更新校准步骤
		SetPlaneInfo(&plane, SET_PLANE_COMPASS_CAL_STEP);
		
		plane.compassCalDataRead = COMPASS_CAL_DATA_READ_NOT;		//更新数据读取状态
		SetPlaneInfo(&plane, SET_PLANE_COMPASS_CAL_READ_STATUS);	
		
		rt_thread_mdelay(3000);		//延时，用于指示灯显示一段时间
		
		flashReadFlag = 0;		//复位flash读取标志位，重新读取flash里的值
	}

}

//将偏差值写入flash
void CompassCalDataWriteToFlash(uint32_t writeAddr, S_CompassOffset *compassOffset)
{
	uint8_t arrLen = 0;
	uint32_t arrTemp[10] = {0};

	arrTemp[arrLen++] = COMPASS_CAL_DATA_HEAD;		//添加帧头
	
	arrTemp[arrLen++] = *(uint32_t *)&compassOffset->x;	
	arrTemp[arrLen++] = *(uint32_t *)&compassOffset->y;	
	arrTemp[arrLen++] = *(uint32_t *)&compassOffset->z;

  FlashWriteSpecifyData(writeAddr,arrTemp,arrLen);
}

//从FLASH中读出偏差值
uint8_t CompassCalReadToFlash(uint32_t readAddr, S_CompassOffset *compassOffset)
{
	uint32_t arrTemp[10] = {0};
	uint8_t arrLen = 0;
	uint32_t ret = 0;
	
	STM32_FLASH_Read(readAddr, arrTemp, 10);
	
	ret = *(uint32_t *)(readAddr);	//读取帧头
	
	arrLen++;		//除去帧头
	if(ret == COMPASS_CAL_DATA_HEAD){	//判断帧头
		compassOffset->x = *(int16_t *)&arrTemp[arrLen++];
		compassOffset->y = *(int16_t *)&arrTemp[arrLen++];
		compassOffset->z = *(int16_t *)&arrTemp[arrLen++];
		
		return 0;
	}

	return 1;
}

int CompassCalibrate(E_CompassCalStep *calStep)
{
	static uint8_t stepCount = 0;
	S_Compass compass = {0};
	static uint16_t positionCount = 1;
	static int16_t compassMinX = 0, compassMaxX = 0;
	static int16_t compassMinY = 0, compassMaxY = 0;
	static int16_t compassMinZ = 0, compassMaxZ = 0;
	static float rotationArr[13] = {-180.0, -150.0, -120.0, -90.0, -60.0, -30.0, 0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0};
	S_Imu imu = {0};
		
	GetCompassInfo(&compass);
	GetImuInfo(&imu);
	
	switch(*calStep){
		case COMPASS_CAL_SETP_BEGIN:
			*calStep = COMPASS_CAL_SETP_HORIZONTAL_ROTATION;
			stepCount = 0;
			positionCount = 0;
			return *calStep;
		case COMPASS_CAL_SETP_HORIZONTAL_ROTATION:		//水平旋转

			//检测旋转位置，确保每个位置都转到位
			for(int i = 0; i < 13; i++){

				if(rotationArr[i] == 1000){
					continue;
				}
				else if(fabs(rotationArr[i] - imu.yaw) < 5){		//当差值小于5，判定为已旋转到位
					rotationArr[i] = 1000;		//将当前位置设一个较大值，后续不再加入计算
					positionCount++;
				}
			}

			if(compassMinX > compass.rawX){		//找x轴最小值
				compassMinX = compass.rawX;
			}
			if(compassMinY > compass.rawY){		//找y轴最小值
				compassMinY = compass.rawY;
			}
			
			if(compassMaxX < compass.rawX){		//找x轴最大值
				compassMaxX = compass.rawX;
			}
			if(compassMaxY < compass.rawY){		//找y轴最大值
				compassMaxY = compass.rawY;
			}
			
			if(positionCount == 13){		//判断所有位置是否都旋转到位
				float temp[13] = {-180.0, -150.0, -120.0, -90.0, -60.0, -30.0, 0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0};
				
				memcpy((void *)rotationArr, (void *)temp, sizeof(temp));		//复位所有位置，用于Z轴校准
				
				positionCount = 0;		//清零位置计数
				stepCount++;
				*calStep = COMPASS_CAL_SETP_VERTICAL_ROTATION;
					
				LogInfo("x y data collect done");
			}
			return *calStep;
		case COMPASS_CAL_SETP_VERTICAL_ROTATION:		//侧向垂直旋转
//			//检测旋转位置，确保每个位置都转到位
//			for(int i = 0; i < 13; i++){
//				if(rotationArr[i] == 1000){
//					continue;
//				}
//				else if(fabs(rotationArr[i] - imu.roll) < 5){		//当差值小于5，判定为已旋转到位
//					rotationArr[i] = 1000;		//将当前位置设一个较大值，后续不再加入计算
//					positionCount++;
//				}
//			}
			positionCount++;
			
			if(compassMinZ > compass.rawZ){
				compassMinZ = compass.rawZ;
			}
			
			if(compassMaxZ < compass.rawZ){
				compassMaxZ = compass.rawZ;
			}
			
			if(positionCount >= 1000){
				positionCount = 0;		//清零位置计数
				stepCount++;
				*calStep = COMPASS_CAL_SETP_SAVE;
				LogInfo("z data collect done");
			}
			return *calStep;
		case COMPASS_CAL_SETP_SAVE:		//保存数据
			if(stepCount == 2){
				
				compassOffset.x = (compassMaxX + compassMinX) / 2;
				compassOffset.y = (compassMaxY + compassMinY) / 2;
				compassOffset.z = (compassMaxZ + compassMinZ) / 2;
				
				CompassCalDataWriteToFlash(COMPASS_CAL_FLASH_ADDR, &compassOffset);		//写入flash
				*calStep = COMPASS_CAL_SETP_DONE;
				stepCount = 0;
				
				LogInfo("compass calibrate done");
				return 0;
			}
			break;
		default:
			break;
	}
	
	return -1;
}

uint8_t GetCompassCalData(int16_t *compassX, int16_t *compassY, int16_t *compassZ)
{
	S_Plane plane = {LOCK};

	GetPlaneInfo(&plane);
	
	if(plane.compassCalStatus == COMPASS_CALIBREAD){
		*compassX = (*compassX) - compassOffset.x;
		*compassY = (*compassY) - compassOffset.y;
		*compassZ = (*compassZ) - compassOffset.z;
		
		return 1;
	}
	
	return 0;
}
