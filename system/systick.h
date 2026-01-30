#ifndef _systick_h_
#define _systick_h_

#include "stm32f10x.h"
#include <rtthread.h>
//void SystickInit(void);

//void delay_us(uint32_t time);

//void delay_ms(uint32_t time);
#define delay_ms(ms)		rt_thread_mdelay(ms)
#endif

