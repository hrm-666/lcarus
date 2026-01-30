#include "acc.h"
#include "mpu6050.h"
#include <rtthread.h>
#include "acc_cal.h"
#include "log_lib.h"
#include "my_lib.h"
#include "transmit_packet.h"
#include <string.h>
#include "filter_lib.h"

#define USE_MPU6050_SENSOR

typedef struct{
	S_ButterworthLpf filterX;
	S_ButterworthLpf filterY;
	S_ButterworthLpf filterZ;
}S_AccFilter;

static uint8_t accSensorInitStatus = 0;		//加速度计初始化状态
static S_Acc acc = {0};		///加速度计数据
static S_AccFilter accFilter = {{200, 30, 0}, {200, 30, 0}, {200, 30, 0}};		//加速度计巴特沃斯低通滤波参数

static rt_mutex_t acc_info_mutex = RT_NULL;		//加速度计互斥锁

void AccDataHandle(void);

uint8_t AccInit(void)
{
	uint8_t ret = 0;
	
	ret = Mpu6050Init();
	if(ret == 1){
		accSensorInitStatus = 1;
	}
	
	//创建互斥锁
	acc_info_mutex = rt_mutex_create("acc_info_mutex", RT_IPC_FLAG_PRIO);
	if (acc_info_mutex == RT_NULL){
		LogError("create dynamic acc_info_mutex failed.\n");
		return 1;
	}

	return 0;
}

void AccDataHandle(void)
{
	float accX = 0, accY = 0, accZ = 0;

	rt_mutex_take(acc_info_mutex, RT_WAITING_FOREVER);

#ifdef USE_MPU6050_SENSOR
	//获取加速度原始数据
	MPU6050_Get_Accelerometer((short *)&acc.rawData.x, (short *)&acc.rawData.y, (short *)&acc.rawData.z);

	accX = (float)acc.rawData.x;
	accY = (float)acc.rawData.y;
	accZ = (float)acc.rawData.z;

	//加速度数据单位转换
	AccDataTransToG(&accX, &accY, &accZ);
	acc.gravity.x = (float)accX;
	acc.gravity.y = (float)accY;
	acc.gravity.z = (float)accZ;
#endif

	//数据低通滤波
	acc.gravity.x = ButterworthLpf(&accFilter.filterX, acc.gravity.x);
	acc.gravity.y = ButterworthLpf(&accFilter.filterY, acc.gravity.y);
	acc.gravity.z = ButterworthLpf(&accFilter.filterZ, acc.gravity.z);
//	LogInfo("%f", acc.gravity.z);

	GetAccCalibrateData(&acc.gravity.x, &acc.gravity.y, &acc.gravity.z);		//校准数据

	rt_mutex_release(acc_info_mutex);

}

void GetAccInfo(S_Acc *exAcc)
{
	if(rt_mutex_take(acc_info_mutex, RT_WAITING_NO) == RT_EOK){
		StructCopy((uint8_t *)&acc, (uint8_t *)exAcc, sizeof(acc));
		rt_mutex_release(acc_info_mutex);
	}	
}

uint8_t GetAccSensorInitStatus(void)
{
	return accSensorInitStatus;
}
