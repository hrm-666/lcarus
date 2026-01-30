#include "led_status.h"
#include "transmit_packet.h"
#include "led.h"
#include <rtthread.h>
#include "log_lib.h"
#include "acc.h"
#include "gyro.h"
#include "barometer.h"
#include "pair_freq.h"
#include "plane.h"
#include "compass.h"

static rt_thread_t led_status_thread = RT_NULL;

void LedStatus(void);
void SensorIintStatusCheck(void);

void led_status_entry(void *param)
{
	SensorIintStatusCheck();
	
	while(1){
		LedStatus();
		
		rt_thread_mdelay(20);
	}
}

void LedStatus_Main(void)
{	
	Ws2812LedInit();
	LogInfo("led init");
	
	//创建线程
	led_status_thread = rt_thread_create("led_status_thread", led_status_entry, RT_NULL, 512, 10, 5);
	
	//启动线程
	if (led_status_thread != RT_NULL){
		rt_thread_startup(led_status_thread);
	}
	
}	

void SensorIintStatusCheck(void)
{
	uint8_t ret = 0;
		
	while(1){		//阻塞在当前函数，防止运行LedStatus()导致指示灯混乱
		ret |= GetTransmiteModuleInitStatus();
		ret |= GetAccSensorInitStatus(); 
		ret |= GetGyroSensorInitStatus(); 
		ret |= GetBarometerSensorInitStatus();
		ret |= GetCompassInitStatus();
		if(ret == 1){
			TopLedBlink(RED, 2, 20);
		} else {
			return;
		}
		
		rt_thread_mdelay(20);
	}

}

//状态灯
void LedStatus(void)
{
	S_Plane plane;
	
	GetPlaneInfo(&plane);
	
	if(plane.pair == PAIR_NOT){		//未对频
		TopLedColorSet(RED);
	}
	else if(plane.pair == PAIR_START){		//对频中
		TopLedBlink(CYAN, 5, 20);
	}
	else if(plane.pair == PAIR_FAIL){		//对频失败
		TopLedColorSet(YELLOW);
	}
	else if(plane.pair == PAIR_DONE){
		if(plane.signal == SIGNAL_LOST){
			TopLedBlink(YELLOW, 2, 20);
		}
		else if(plane.gyroCalStatus == GYRO_CAL_DATA_READING || plane.accCalDataRead == ACC_CAL_DATA_READING || plane.compassCalDataRead == COMPASS_CAL_DATA_READING){
			TopLedBlink(PURPLE, 5, 20);
		}
		else if(plane.gyroCalStatus == GYRO_CAL_DONE || plane.accCalStatus == ACC_CAL_DONE || plane.compassCalStatus == COMPASS_CAL_DONE){
			TopLedColorSet(PURPLE);
		}
		else if(plane.gyroCalStatus == GYRO_CAL_CANCEL || plane.accCalStatus == ACC_CAL_CANCEL || plane.compassCalStatus == COMPASS_CAL_CANCEL){
			TopLedBlink(RED, 5, 20);
		}
		else if(plane.gyroCalStatus == GYRO_CAL_FAIL || plane.accCalStatus == ACC_CAL_FAIL || plane.compassCalStatus == COMPASS_CAL_FAIL){
			TopLedColorSet(RED);
		}
		else if(plane.lock == UNLOCKING){
			TopLedBlink(GREEN, 5, 20);
		}
		else if(plane.lock == UNLOCK){
			TopLedColorSet(GREEN);
		}
		else if(plane.lock == LOCKING){
			TopLedBlink(BLUE, 5, 20);
		}
		else if(plane.lock == LOCK){
			TopLedColorSet(BLUE);
		}
	}
	
	if(plane.power == POWER_LOWER){
		BottomLedBlink(RED);
	} else {
		BottomLedBlink(WHITE);
	}

}
