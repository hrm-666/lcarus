#include "gyro.h"
#include <rtthread.h>
#include "mpu6050.h"
#include "my_lib.h"
#include "transmit_packet.h"
#include "log_lib.h"
#include "gyro_cal.h"
#include <string.h>
#include "filter_lib.h"

#define USE_MPU6050_GYRO_SENSOR

static uint8_t gyroSensorInitStatus = 0;		//陀螺仪初始化状态
static S_Gyro gyro = {0};		//陀螺仪结构体变量
static rt_mutex_t gyro_info_mutex = RT_NULL;		//陀螺仪互斥锁

void GyroDataHandle(void);

//陀螺仪初始化
uint8_t GyroInit(void)
{
	uint8_t ret = 0;
	
	ret = Mpu6050Init();
	if(ret == 1){
		gyroSensorInitStatus = 1;
		return 1;
	}
	
	/* 创建互斥锁 */
	gyro_info_mutex = rt_mutex_create("gyro_info_mutex", RT_IPC_FLAG_PRIO);
	if (gyro_info_mutex == RT_NULL)
	{
			rt_kprintf("create dynamic gyro_info_mutex failed.\n");
			return 1;
	}
	
	return 0;
}

//陀螺仪数据转换成度/秒
void GyroDataTransformDeg(float *gyroX,float *gyroY,float *gyroZ)
{
	*gyroX = *gyroX * GYRO_RAW_TO_DEG_S;
	*gyroY = *gyroY * GYRO_RAW_TO_DEG_S;
	*gyroZ = *gyroZ * GYRO_RAW_TO_DEG_S;
}

//陀螺仪数据转换成弧度/秒
void GyroDataTransformRad(float *gyroX,float *gyroY,float *gyroZ)
{
	*gyroX = *gyroX * GYRO_RAW_TO_RAD_S;
	*gyroY = *gyroY * GYRO_RAW_TO_RAD_S;
	*gyroZ = *gyroZ * GYRO_RAW_TO_RAD_S;
}

//陀螺仪数据处理
void GyroDataHandle(void)
{
  float gyroX = 0, gyroY = 0, gyroZ = 0;

	rt_mutex_take(gyro_info_mutex, RT_WAITING_FOREVER);

#ifdef USE_MPU6050_GYRO_SENSOR
    //获取陀螺仪原始数据
	MPU6050_Get_Gyroscope((short *)&gyro.rawData.x, (short *)&gyro.rawData.y, (short *)&gyro.rawData.z);

#endif
    
	gyroX = (float)gyro.rawData.x;
	gyroY = (float)gyro.rawData.y;
	gyroZ = (float)gyro.rawData.z;
	GyroDataTransformDeg(&gyroX, &gyroY, &gyroZ);
	gyro.degS.x = gyroX;
	gyro.degS.y = gyroY;
	gyro.degS.z = gyroZ; 
	
  gyroX = (float)gyro.rawData.x;
	gyroY = (float)gyro.rawData.y;
	gyroZ = (float)gyro.rawData.z;
  GyroDataTransformRad(&gyroX, &gyroY, &gyroZ);
	gyro.radS.x = (float)gyroX;
	gyro.radS.y = (float)gyroY;
	gyro.radS.z = (float)gyroZ;

	GetGyroCalibrateData(&gyro.degS.x, &gyro.degS.y, &gyro.degS.z);

	rt_mutex_release(gyro_info_mutex);
}

//获取陀螺仪数据
void GetGyroInfo(S_Gyro *exGyro)
{
	if(rt_mutex_take(gyro_info_mutex, RT_WAITING_NO) == RT_EOK){
		StructCopy((uint8_t *)&gyro, (uint8_t *)exGyro, sizeof(gyro));
		rt_mutex_release(gyro_info_mutex);
	}	
}

//获取陀螺仪初始化状态
uint8_t GetGyroSensorInitStatus(void)
{
	return gyroSensorInitStatus;
}
