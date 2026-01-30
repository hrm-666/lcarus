#include "cpu_usage.h"
#include "rtthread.h"
#include "task_manage.h"
#include "log_lib.h"

#define CPU_USAGE_CALC_TICK    100		//计算CPU_USAGE_CALC_TICK(ms)内cpu利用率，该参数可根据线程周期进行调整
#define CPU_USAGE_LOOP        100

static rt_thread_t cpu_usage_task_thread = RT_NULL;
	
static rt_uint8_t  cpu_usage_major = 0, cpu_usage_minor= 0;
static rt_uint32_t total_count = 0;

static void cpu_usage_idle_hook(void);
void cpu_usage_get(rt_uint8_t *major, rt_uint8_t *minor);

void CpuUsageInit(void)
{
    /* set idle thread hook */
    rt_thread_idle_sethook(cpu_usage_idle_hook);
}

void cpu_usage_task_thread_entry(void *param)
{
	rt_uint8_t major = 0, minor = 0;
	
	while(1){
		cpu_usage_get(&major, &minor);
		LogInfo("cpu usage: %d:%d%%", major, minor);
		
		rt_thread_mdelay(500);
	}
}

void CpuUsage_Main(void)
{	
	CpuUsageInit();
	
	//创建线程
	cpu_usage_task_thread = rt_thread_create("cpu_usage_task_thread", cpu_usage_task_thread_entry, RT_NULL, 512, CPU_USAGE_PRIORITY, 5);
	
	//启动线程
	if (cpu_usage_task_thread != RT_NULL){
		rt_thread_startup(cpu_usage_task_thread);
	} else {
		LogError("cpu_usage_task_thread_entry start fail");
	}
	
}

static void cpu_usage_idle_hook(void)
{
    rt_tick_t tick;
    rt_uint32_t count;
    volatile rt_uint32_t loop;

	//计算cpu在CPU_USAGE_CALC_TICK时间内的运行速率
    if (total_count == 0)
    {
        /* get total count */
        rt_enter_critical();		//禁止任务调度
        tick = rt_tick_get();		//获取系统时间
        while(rt_tick_get() - tick < CPU_USAGE_CALC_TICK)		//在CPU_USAGE_CALC_TICK内，total_count累加
        {
            total_count ++;
            loop = 0;
            while (loop < CPU_USAGE_LOOP) loop ++;
        }
        rt_exit_critical();
    }

	//计算空闲时间
    count = 0;
    /* get CPU usage */
    tick = rt_tick_get();
    while (rt_tick_get() - tick < CPU_USAGE_CALC_TICK)
    {
        count ++;
        loop  = 0;
        while (loop < CPU_USAGE_LOOP) loop ++;
    }

	//计算利用率
    /* calculate major and minor */
    if (count < total_count)
    {
        count = total_count - count;	//利用时间 = 总时间 - 空闲时间
        cpu_usage_major = (count * 100) / total_count;		//利用时间转化为百分比
        cpu_usage_minor = ((count * 100) % total_count) * 100 / total_count;

    }
    else
    {
        total_count = count;

        /* no CPU usage */
        cpu_usage_major = 0;
        cpu_usage_minor = 0;
    }
}

void cpu_usage_get(rt_uint8_t *major, rt_uint8_t *minor)
{
    RT_ASSERT(major != RT_NULL);
    RT_ASSERT(minor != RT_NULL);

    *major = cpu_usage_major;
    *minor = cpu_usage_minor;
}

