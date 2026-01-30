#include "compass.h"
#include "qmc5883l.h"
#include "log_lib.h"
#include "my_lib.h"
#include <rtthread.h>
#include "compass_cal.h"

static uint8_t compassInitStatus = 0;		//磁力计初始化状态

static S_Compass compass = {0};		//磁力计数据

static rt_mutex_t compass_info_mutex = RT_NULL;

uint8_t CompassInit(void)
{
	uint8_t ret = 0;
	
	ret = Qmc5883lInit();
	if(ret == 1){
		compassInitStatus = 1;
		LogError("compass init fail.");
		return 1;
	}
	
	//创建互斥锁
	compass_info_mutex = rt_mutex_create("acc_info_mutex", RT_IPC_FLAG_PRIO);
	if (compass_info_mutex == RT_NULL){
		LogError("create dynamic compass_info_mutex failed.\n");
		return 1;
	}
	
	return 0;	
}

void CompassHandle(void)
{
	int16_t tempCompassX = 0, tempCompassY = 0, tempCompassZ = 0;
	
	rt_mutex_take(compass_info_mutex, RT_WAITING_FOREVER);
	
	//获取磁力计原始数据
	GetQmc5883lData(&compass.rawX, &compass.rawY, &compass.rawZ);
	
	tempCompassX = compass.rawX;
	tempCompassY = compass.rawY;
	tempCompassZ = compass.rawZ;

//	GetCompassCalData(&compass.rawX, &compass.rawY, &compass.rawZ);
	GetCompassCalData(&tempCompassX, &tempCompassY, &tempCompassZ);
	
	compass.gaussX = (float)tempCompassX;
	compass.gaussY = (float)tempCompassY;
	compass.gaussZ = (float)tempCompassZ;
	
	CompassDataTransToGauss(&compass.gaussX, &compass.gaussY, &compass.gaussZ);		//原始数据转换成高斯
	
//	LogRaw("%d, %d, %d\r\n", compass.rawX, compass.rawY, compass.rawZ);
//	LogRaw("%f, %f, %f\r\n", compass.gaussX, compass.gaussY, compass.gaussZ);	
	rt_mutex_release(compass_info_mutex);
}

uint8_t GetCompassInitStatus(void)
{
	return compassInitStatus;
}

void GetCompassInfo(S_Compass *exCompass)
{
	if(rt_mutex_take(compass_info_mutex, RT_WAITING_NO) == RT_EOK){
		StructCopy((uint8_t *)&compass, (uint8_t *)exCompass, sizeof(compass));
		rt_mutex_release(compass_info_mutex);
	}
}
