#include "task_manage.h"
#include <rtthread.h>
#include "log_lib.h"
#include "gyro.h"
#include "acc.h"
#include "imu.h"
#include "pid.h"
#include "controller.h"
#include "task_manage.h"
#include "nav.h"
#include "compass.h"
#include "acc_cal.h"
#include "gyro_cal.h"
#include "compass_cal.h"
#include "receive_packet.h"

static rt_thread_t master_task_thread = RT_NULL;
static rt_thread_t second_task_thread = RT_NULL;
static rt_thread_t third_task_thread = RT_NULL;

void master_task_thread_entry(void *param)
{
	while(1){
		GyroDataHandle();
		AccDataHandle();
		CompassHandle();
		ImuHandle();
		NavAcc();
		ControlHandle();
		
		rt_thread_mdelay(5);
	}
}

//飞控主任务
void MasterTask(void)
{	
	//创建线程
	master_task_thread = rt_thread_create("master_task_thread", master_task_thread_entry, RT_NULL, 2048, MASTER_TASK_PRIORITY, 10);
	
	//启动线程
	if (master_task_thread != RT_NULL){
		rt_thread_startup(master_task_thread);
	} else {
		LogError("master_task_thread start fail");
	}
}

void second_task_thread_entry(void *param)
{
	while(1){
		AccCalibrateHandle();
		GyroCaliBrateHandle();
		CompassCalibrateHandle();
		
		rt_thread_mdelay(10);
	}
}

//次要任务
void SecondTask(void)
{	
	//创建线程
	second_task_thread = rt_thread_create("second_task_thread", second_task_thread_entry, RT_NULL, 2048, SECOND_TASK_PRIORITY, 5);
	
	//启动线程
	if (second_task_thread != RT_NULL){
		rt_thread_startup(second_task_thread);
	} else {
		LogError("second_task_thread start fail");
	}
	
}

void third_task_thread_entry(void *param)
{
	while(1){
		ReceiveHandle();
		AnalysisHandle();
		
		TransmitHandle();
		
		rt_thread_mdelay(5);
	}
}

//三级任务
void ThirdTask(void)
{
	//创建线程
	third_task_thread = rt_thread_create("third_task_thread", third_task_thread_entry, RT_NULL, 1024, THIRD_TASK_PRIORITY, 5);
	
	//启动线程
	if (third_task_thread != RT_NULL){
		rt_thread_startup(third_task_thread);
	} else {
		LogError("third_task_thread start fail");
	}
}

