#include "barometer.h"
#include <rtthread.h>
//#define DEBUG_LEVEL	DEBUG_LOG
#include "log_lib.h"
#include "bmp280.h"
#include "math.h"
#include <string.h>
#include "filter_lib.h"
#include "task_manage.h"
#include "transmit_packet.h"
#include "plane.h"

static rt_thread_t barometer_thread = RT_NULL;
	
static S_Barometer baro = {0};		//气压计数据变量
static uint8_t barometerSensorInitStatus = 0;		//气压计初始化状态
static S_ButterworthLpf baroFilter = {50, 20, 0};		//气压计数据低通滤波参数

static rt_mutex_t barometer_info_mutex = RT_NULL;

static float baroMoveMiddleFilterArr[7] = {0};		//气压计中值滤波数组
static uint8_t baroMoveMiddleFilterNum = 0;		//气压计中值滤波数据序号

void BarometerDataHandle(void);
double PressureToHeight(double baroPressure);
static double VerticalVelocity(const double verticalHeight, uint8_t delayMs);

void barometer_thread_entry(void *param)
{
	while(1){
		BarometerDataHandle();
		rt_thread_mdelay(20);
	}
}

void Barometer_Main(void)
{
	uint8_t ret = 0;
	
	ret = Bmp280Init();
	if(ret == 1){
		barometerSensorInitStatus = 1;
		return;
	}
	/* 创建互斥锁 */
    barometer_info_mutex = rt_mutex_create("barometer_info_mutex", RT_IPC_FLAG_PRIO);
    if (barometer_info_mutex == RT_NULL)
    {
        rt_kprintf("create dynamic barometer_info_mutex failed.\n");
        return;
    }
	
	//创建线程
	barometer_thread = rt_thread_create("barometer_thread", barometer_thread_entry, RT_NULL, 512, BAROMETER_PRIORITY, 5);
	
	//启动线程
	if (barometer_thread != RT_NULL){
		rt_thread_startup(barometer_thread);
    return;
	} else {
		LogError("barometer_thread start fail");
		return;
	}

}

void BarometerDataHandle(void)
{
	static double takeOffAltitude = 0;
	S_Plane plane;
//	while(BMP280_GetStatus(BMP280_MEASURING) != RESET){
//		// rt_thread_mdelay(1);
//		// return;
//	}
//	while(BMP280_GetStatus(BMP280_IM_UPDATE) != RESET){
//		// rt_thread_mdelay(1);
//		// return;
//	}


	rt_mutex_take(barometer_info_mutex, RT_WAITING_FOREVER);
	
	baro.temperature = BMP280_Get_Temperature();
	
	baro.pressure = Bmp280GetPressure();
	baro.height = PressureToHeight(baro.pressure);		//转换为海拔高度
	baro.height = MoveMiddleFilter(&baroMoveMiddleFilterNum, baroMoveMiddleFilterArr, 7, baro.height);		//滑动中值滤波
	baro.height = ButterworthLpf(&baroFilter, baro.height);		//巴特沃斯低通滤波

	baro.verticalVel = VerticalVelocity(baro.height, 20);
	
	rt_mutex_release(barometer_info_mutex);

	GetPlaneInfo(&plane);
	if(plane.lock == UNLOCKING){
		takeOffAltitude = baro.height;
	}
	else if(takeOffAltitude != 0){
		baro.relativeAltitude = baro.height - takeOffAltitude;
		
		plane.relativeAltitude = baro.relativeAltitude;
		SetPlaneInfo(&plane, SET_PLANE_RELATIVE_ALTITUDE);
	}
	
//	LogRaw("%f, %f, %f, %f", baro.height, baro.temperature, baro.pressure, baro.verticalVel);

}

double PressureToHeight(double baroPressure)
{
	return 44330.0 * (1- powf((baroPressure/P0),(1/5.256)));
}

static double VerticalVelocity(const double verticalHeight, uint8_t delayMs)
{
	static double lastHeight = 0;
	double heightDifference = 0;
	double verticalVelocity = 0;
	
	if(lastHeight == 0){
		lastHeight = verticalHeight;
	}
	
	heightDifference = verticalHeight - lastHeight;		//计算高度差
	verticalVelocity = heightDifference/(delayMs * 0.001f);		//高度的微分求垂直速度
	
	lastHeight = verticalHeight;	//更新上一次的高度;
	
	return verticalVelocity;		//单位为(m/s）

}

void GetBarometerInfo(S_Barometer *exBaro)
{
	if(rt_mutex_take(barometer_info_mutex, RT_WAITING_NO) == RT_EOK){
		memcpy((void *)exBaro, (void *)&baro, sizeof(baro));
		rt_mutex_release(barometer_info_mutex);
	}
}

uint8_t GetBarometerSensorInitStatus(void)
{
	return barometerSensorInitStatus;
}
