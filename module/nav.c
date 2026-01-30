#include "nav.h"
#include "imu.h"
#include "math.h"
#include "acc.h"
#include "gyro.h"
#include <rtthread.h>
#include "log_lib.h"
#include "my_lib.h"
#include "barometer.h"

static S_Nav nav = {0};		//导航系结构体变量

static rt_mutex_t nav_info_mutex = RT_NULL;

void NavAcc(void);

uint8_t NavAccInit(void)
{
	//创建互斥锁
	nav_info_mutex = rt_mutex_create("nav_info_mutex", RT_IPC_FLAG_PRIO);
    if (nav_info_mutex == RT_NULL){
        rt_kprintf("create dynamic nav_info_mutex failed.\n");
        return 1;
    }
		
	return 0;
}

void NavAcc(void)
{
	S_RotationMatrix rMatrix = {0};
	S_Acc acc = {0};
	static uint16_t delayCount = 0;
	S_Barometer baro = {0};
	static float staticHeight = 0;
	const float dt = 0.005f;

    GetDcmMatrix(&rMatrix);
	GetAccInfo(&acc);
	GetBarometerInfo(&baro);

	rt_mutex_take(nav_info_mutex, RT_WAITING_FOREVER);

	//载体加速度转到导航系下的三轴加速度
	nav.globalAccX = rMatrix.droneToEarth[0][0] * acc.gravity.x + rMatrix.droneToEarth[0][1] * acc.gravity.y + rMatrix.droneToEarth[0][2] * acc.gravity.z;
	nav.globalAccY = rMatrix.droneToEarth[1][0] * acc.gravity.x + rMatrix.droneToEarth[1][1] * acc.gravity.y + rMatrix.droneToEarth[1][2] * acc.gravity.z;
	nav.globalAccZ = rMatrix.droneToEarth[2][0] * acc.gravity.x + rMatrix.droneToEarth[2][1] * acc.gravity.y + rMatrix.droneToEarth[2][2] * acc.gravity.z;      

	nav.globalAccX *= gravity_mss;
	nav.globalAccY *= gravity_mss;
	nav.globalAccZ = nav.globalAccZ * gravity_mss - gravity_mss;		//g转换成m/s^2，并减去重力加速度

	rt_mutex_release(nav_info_mutex);
	
	if(delayCount < 2000){		//等待数据稳定后再记录
		delayCount++;
		staticHeight = baro.height;
	} else {
//		nav.globalAccZ -= staticNavGlobalAccZ;
		baro.height -= staticHeight;
		
		nav.positionZ += nav.velZ * dt + 0.5f * nav.globalAccZ * dt * dt;		//计算位移，s = v0 * t + 1/2 * a * t * t
		nav.positionZ = nav.positionZ * 0.98f + baro.height * 0.02f;		//与气压计互补滤波求位移
		
		nav.velZ += nav.globalAccZ * dt;		//加速度积分得速度，v = a * t
		nav.velZ = nav.velZ * 0.98f + baro.verticalVel * 0.02f;		//与气压计互补滤波求速度
		
	}
		
//	LogRaw("%f, %f, %f, %f\r\n", nav.velZ, nav.positionZ, baro.height, nav.globalAccZ);	

}

void GetNavAcc(S_Nav *exNav)
{
	if(rt_mutex_take(nav_info_mutex, RT_WAITING_NO) == RT_EOK){
		StructCopy((uint8_t *)&nav, (uint8_t *)exNav, sizeof(nav));
		rt_mutex_release(nav_info_mutex);
	}
}
