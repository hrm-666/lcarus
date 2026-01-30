#include "imu.h"
#include "math_lib.h"
#include "math.h"
#include "led.h"
#include "systick.h"
#include "usart.h"
#include "my_lib.h"
#include "acc.h"
#include "gyro.h"
#include <string.h>
#include "compass.h"

#define DEBUG_LEVEL	DEBUG_INFO
#include "log_lib.h"

//加速度计误差补偿参数
#define accKi  0.005f
#define accKp  0.6f

#ifdef USE_COMPASS
//磁罗盘误差补偿参数
#define compassKi  0.005f
#define compassKp  1.0f
#endif

static S_Imu imu = {0};		//姿态数据
static S_RotationMatrix rMatrix = {0};		//旋转矩阵

static float q0 = 1.0f, q1 = 0, q2 = 0, q3 = 0;		//四元数
static float errIntX = 0, errIntY = 0, errIntZ = 0;		//加速度计误差积分

int ImuUpdate(S_Imu *imuData, uint8_t delayMs);

void ImuHandle(void);

//航向角估计
void HeadingEstimate(void)
{
	double sin_pit = 0, sin_rol = 0, sin_yaw = 0;
	double cos_pit = 0, cos_rol = 0, cos_yaw = 0;  
	double hX = 0, hY = 0, heading = 0;
	S_Compass compass = {0};
	
	GetCompassInfo(&compass);
	
	//通过欧拉角转换成方向余弦的每个元素
    sin_pit = sin(imu.pitch * DEG_TO_RAD);
    cos_pit = cos(imu.pitch * DEG_TO_RAD);
    sin_rol = sin(imu.roll * DEG_TO_RAD);
    cos_rol = cos(imu.roll * DEG_TO_RAD);
    sin_yaw = sin(imu.yaw * DEG_TO_RAD);
    cos_yaw = cos(imu.yaw * DEG_TO_RAD);  
	
	//倾角补偿
    hX = compass.rawX * cos_rol + compass.rawZ * sin_rol;
    hY = compass.rawX * sin_pit * sin_rol + compass.rawY * cos_pit - compass.rawZ * cos_rol * sin_pit;

	//计算航向角，0度表示磁北
    heading = atan2(hX, hY)*57.296f; 
	if(heading < 0){
		heading += 360;
	}
	
	// 磁偏角因地而异，需要查询当地值(本示例为海南海口)
	float magnetic_declination = -2.0f; // 示例值(单位:度)

	// 计算真航向(相对于地理北极)
	float true_heading = heading + magnetic_declination;
	if(true_heading > 360) true_heading -= 360;
	if(true_heading < 0) true_heading += 360;
	
	LogRaw("%f, %f\r\n", heading, true_heading);
}

void ImuHandle(void)
{
	S_Acc acc = {0};
	S_Gyro gyro = {0};
	S_Compass compass = {0};
	static uint32_t lastTime = 0;
	
	GetAccInfo(&acc);
	GetGyroInfo(&gyro);
	GetCompassInfo(&compass);
	
	imu.gyroX = gyro.radS.x;
	imu.gyroY = gyro.radS.y;
	imu.gyroZ = gyro.radS.z;
	
	imu.accX = acc.gravity.x;
	imu.accY = acc.gravity.y;
	imu.accZ = acc.gravity.z;
	
	imu.compassX = compass.gaussX;
	imu.compassY = compass.gaussY;
	imu.compassZ = compass.gaussZ;

//	LogInfo("%d", rt_tick_get() - lastTime);
	ImuUpdate(&imu, rt_tick_get() - lastTime);
	lastTime = rt_tick_get();
	LogDebug("%f, %f, %f", imu.roll, imu.pitch, imu.yaw);
}

int ImuUpdate(S_Imu *imuData, uint8_t delayMs)
{
	float norm;
	float gbX, gbY, gbZ;
	float errX, errY, errZ;
	
	#ifdef USE_COMPASS
	float hX = 0, hY = 0, hZ = 0;
	float bX = 0, bZ = 0;
	float wX = 0, wY = 0;
	float yawErr = 0, yawErrInt = 0;
	#endif
	
	if(imuData->accX * imuData->accY * imuData->accZ==0){
		return -1;
	}
    
	//加速度计数据归一化成单位矢量
	norm = sqrt(imuData->accX*imuData->accX + imuData->accY*imuData->accY + imuData->accZ*imuData->accZ);
	imuData->accX = imuData->accX / norm;
	imuData->accY = imuData->accY / norm;
	imuData->accZ = imuData->accZ / norm;
	
	#ifdef USE_COMPASS
	//磁力计数据归一化
	norm = FastSqrt(imuData->compassX*imuData->compassX + imuData->compassY*imuData->compassY + imuData->compassZ*imuData->compassZ);
	imuData->compassX = imuData->compassX * norm;
	imuData->compassY = imuData->compassY * norm;
	imuData->compassZ = imuData->compassZ * norm;

	//计算映射到地理坐标系下各轴磁场分量
	hX = imu.compassX * rMatrix.droneToEarth[0][0] + imu.compassY * rMatrix.droneToEarth[0][1] + imu.compassZ * rMatrix.droneToEarth[0][2];
	hY = imu.compassX * rMatrix.droneToEarth[1][0] + imu.compassY * rMatrix.droneToEarth[1][1] + imu.compassZ * rMatrix.droneToEarth[1][2];
	hZ = imu.compassX * rMatrix.droneToEarth[2][0] + imu.compassY * rMatrix.droneToEarth[2][1] + imu.compassZ * rMatrix.droneToEarth[2][2];
	
	//计算地理坐标系磁场，by = 0
	bX = sqrt(hX * hX + hY * hY);
	bZ = hZ;
	
	//转到机体坐标系下
	wX = bX * rMatrix.earthToDrone[0][0] + bZ * rMatrix.earthToDrone[0][2];
	wY = bX * rMatrix.earthToDrone[1][0] + bZ * rMatrix.earthToDrone[1][2];
//	wZ = bX * rMatrix.earthToDrone[2][0] + bZ * rMatrix.earthToDrone[2][2];

	yawErr = imuData->compassX * wY - imuData->compassY * wX;		//叉乘得到偏航误差
	yawErrInt += yawErr * compassKi;		//误差积分
	
	#endif
	
	//计算机体重力向量
	gbX = rMatrix.earthToDrone[0][2];
	gbY = rMatrix.earthToDrone[1][2];
	gbZ = rMatrix.earthToDrone[2][2];

	//叉乘得到误差
//	errX = imuData->accY*gbZ - imuData->accZ*gbY + imuData->compassY * wZ - imuData->compassZ * wY;
//	errY = imuData->accZ*gbX - imuData->accX*gbZ + imuData->compassZ * wX - imuData->compassX * wZ;
//	errZ = imuData->accX*gbY - imuData->accY*gbX + imuData->compassX * wY - imuData->compassY * wX;
	errX = imuData->accY*gbZ - imuData->accZ*gbY;
	errY = imuData->accZ*gbX - imuData->accX*gbZ;
	errZ = imuData->accX*gbY - imuData->accY*gbX;

	//对误差向量进行积分（axb=|a||b|sinθ，当ab都是归一化向量，θ是弧度制并且值比较小时，axb=1x1xsinθ≈θ；
	//则两个向量的弧度角误差分到xyz三个轴上时，就分别为x轴误差(aybz-azby)弧度，y轴误差(azbx-axbz)弧度，z轴误差(axby-aybx)弧度））
	errIntX = errIntX + errX*accKi;
	errIntY = errIntY + errY*accKi;
	errIntZ = errIntZ + errZ*accKi;

	//通过accKp、accKi两个参数来调节加速度计传感器数据与陀螺仪传感器数据之前的权重。
	imuData->gyroX = imuData->gyroX + accKp*errX + errIntX;
	imuData->gyroY = imuData->gyroY + accKp*errY + errIntY;
	
	#ifdef USE_COMPASS
	imuData->gyroZ = imuData->gyroZ + accKp*errZ + errIntZ + compassKp*yawErr + yawErrInt;		//补偿偏航误差
	#else
	imuData->gyroZ = imuData->gyroZ + accKp*errZ + errIntZ;
	#endif

	//一阶龙格库塔法更新四元数 
	q0 = q0 + (-q1*imuData->gyroX - q2*imuData->gyroY - q3*imuData->gyroZ)* delayMs * 0.0005f;
	q1 = q1 + ( q0*imuData->gyroX + q2*imuData->gyroZ - q3*imuData->gyroY)* delayMs * 0.0005f;
	q2 = q2 + ( q0*imuData->gyroY - q1*imuData->gyroZ + q3*imuData->gyroX)* delayMs * 0.0005f;
	q3 = q3 + ( q0*imuData->gyroZ + q1*imuData->gyroY - q2*imuData->gyroX)* delayMs * 0.0005f; 

	//把上述运算后的四元数进行归一化处理。得到了物体经过旋转后的新的四元数。
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;

	//四元数转换成欧拉角
	imuData->pitch = atan2(2.0f*(q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3) * RAD_TO_DEG;
	imuData->roll = asin(2.0f*(q0*q2 - q1*q3)) * RAD_TO_DEG;        

	#ifdef USE_COMPASS
	imuData->yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3 *q3)) * RAD_TO_DEG;
	#else
	imuData->yaw += imuData->gyroZ * RAD_TO_DEG * delayMs * 0.001f;		//z轴角速度积分得偏航角
	#endif
	
	//偏航转换为0~360度
//	if(imuData->yaw < 0){
//		imuData->yaw += 180;
//	}
	imuData->yaw = ConstrainFloat(imuData->yaw, -179.0f, 179.0f);

	RotationMatrixDroneToEarth();	//旋转矩阵更新
	RotationMatrixEarthToDrone();	//旋转矩阵的逆矩阵更新

	return 0;
}

//旋转矩阵：机体坐标系 -> 地理坐标系
void RotationMatrixDroneToEarth(void)
{
	rMatrix.droneToEarth[0][0] = 1.0f - 2.0f * q2*q2 - 2.0f * q3*q3;
	rMatrix.droneToEarth[0][1] = 2.0f * (q1*q2 -q0*q3);
	rMatrix.droneToEarth[0][2] = 2.0f * (q1*q3 +q0*q2);

	rMatrix.droneToEarth[1][0] = 2.0f * (q1*q2 +q0*q3);
	rMatrix.droneToEarth[1][1] = 1.0f - 2.0f * q1*q1 - 2.0f * q3*q3;
	rMatrix.droneToEarth[1][2] = 2.0f * (q2*q3 -q0*q1);
	 
	rMatrix.droneToEarth[2][0] = 2.0f * (q1*q3 -q0*q2);
	rMatrix.droneToEarth[2][1] = 2.0f * (q2*q3 +q0*q1);
	rMatrix.droneToEarth[2][2] = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;   
}


//旋转矩阵的转置矩阵：地理坐标系 -> 机体坐标系
void RotationMatrixEarthToDrone(void)
{
	rMatrix.earthToDrone[0][0] = 1.0f - 2.0f * q2*q2 - 2.0f * q3*q3;
	rMatrix.earthToDrone[0][1] = 2.0f * (q1*q2 +q0*q3);    
	rMatrix.earthToDrone[0][2] = 2.0f * (q1*q3 -q0*q2); 

	rMatrix.earthToDrone[1][0] = 2.0f * (q1*q2 -q0*q3);
	rMatrix.earthToDrone[1][1] = 1.0f - 2.0f * q1*q1 - 2.0f * q3*q3;  
	rMatrix.earthToDrone[1][2] = 2.0f * (q2*q3 +q0*q1);    

	rMatrix.earthToDrone[2][0] = 2.0f * (q1*q3 +q0*q2);
	rMatrix.earthToDrone[2][1] = 2.0f * (q2*q3 -q0*q1);
	rMatrix.earthToDrone[2][2] = 1.0f - 2.0f * q1*q1 - 2.0f * q2*q2;   
}

void GetImuInfo(S_Imu *exImu)
{
	StructCopy((uint8_t *)&imu, (uint8_t *)exImu,sizeof(imu));
}

void GetDcmMatrix(S_RotationMatrix *exRMatrix)
{
	memcpy((void *)exRMatrix, (void *)&rMatrix, sizeof(rMatrix));
}
