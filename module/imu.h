#ifndef _imu_h_
#define _imu_h_

#include "stm32f10x.h"

#define USE_COMPASS		//融合磁力计数据

typedef struct 
{
	//解算需要的陀螺仪数据，单位（弧度/秒）
	float gyroX;
	float gyroY;
	float gyroZ;
	
	//解算需要的加速度计数据，单位（重力g）
	float accX;
	float accY;
	float accZ;
	
	//解算需要的磁罗盘数据，单位（高斯）
	float compassX;
	float compassY;
	float compassZ;
	
	//姿态角
	float roll;
	float pitch;
	float yaw;
}S_Imu;

typedef struct{
	float droneToEarth[3][3];		//机体 -> 地理坐标系的旋转矩阵
	float earthToDrone[3][3];		//地理 -> 机体坐标系的旋转矩阵
}S_RotationMatrix;

void RotationMatrixDroneToEarth(void);
void RotationMatrixEarthToDrone(void);

void GetImuInfo(S_Imu *exImu);
void ImuHandle(void);
void GetDcmMatrix(S_RotationMatrix *exRMatrix);
#endif

