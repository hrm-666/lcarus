#include "stm32f10x.h"
#include "systick.h"

#include "board_config.h"

#include "usart.h"
#include "pwm.h"
#include "i2c.h"
#include "adc.h"
#include "spi.h"

#include "led.h"
#include "compass.h"
#include "acc.h"
#include "gyro.h"
#include "bmp280.h"

#include "pid.h"

#include "led_status.h"
#include "pair_freq.h"
#include "barometer.h"

#include "nav.h"

#include "transmit_packet.h"
#include "receive_packet.h"
#include "ano_communicate.h"

#include "task_manage.h"
#include "nvic.h"

#include "log_lib.h"
#include <rtthread.h>
#include "fc_status.h"

#include "battery_detect.h"

#include "cpu_usage.h"
#include "plane.h"

#include "controller.h"

int main(void)
{
	//板子配置初始化
	BoardConfig();
	
	//初始化外设
	UartInit();
	AdcInit();
	I2cInit();
	SpiInit();
	PwmInit();
	rt_thread_mdelay(200);

	Ws2812LedInit();		//状态灯初始化
	GyroInit();		//陀螺仪初始化
	AccInit();		//加速度计初始化
	CompassInit();		//磁力计初始化

	AllPidInit();		//pid参数初始化
	PlaneInit();		//飞机状态初始化
	rt_thread_mdelay(200);

	LedStatus_Main();		//指示灯线程初始化
	Pair_Main();		//对频线程初始化
	Barometer_Main();		//气压计线程初始化

	NavAccInit();		//导航系初始化

	TransmitInit();		//发送功能初始化
	ReceiveInit();		//接收功能初始化
	AnoCommunicate_Main();		//匿名上位机收发线程初始化

	ControlInit();		//控制功能初始化
	MasterTask();		//飞控主任务初始化
	SecondTask();		//次要任务初始化
	ThirdTask();		//三级任务初始化
	NvicConfig();		//配置串口接收中断
	
	CpuUsage_Main();		//cpu利用率计算线程初始化
	
	rt_thread_mdelay(200);
	LogInfo("system init done");

	while(1){
		
		VoltageDetect();		//低电压检测
		PlaneLockStatus();
		rt_thread_mdelay(50);
	}
}
