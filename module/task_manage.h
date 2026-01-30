#ifndef _MANAGE_H_
#define _MANAGE_H_

#include "stm32f10x.h"

//各线程优先级
#define MASTER_TASK_PRIORITY	5
#define SECOND_TASK_PRIORITY	5
#define THIRD_TASK_PRIORITY		5

#define LED_STATUS_PRIORITY		5
#define PAIR_PRIORITY					5
#define BAROMETER_PRIORITY		5
#define ANO_COMMUNICATE_PRIORITY	5

#define CPU_USAGE_PRIORITY	30

void MasterTask(void);
void SecondTask(void);
void ThirdTask(void);
#endif
