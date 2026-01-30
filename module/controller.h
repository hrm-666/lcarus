#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "stm32f10x.h"

typedef struct {
    uint16_t out1;
    uint16_t out2;
    uint16_t out3;
    uint16_t out4;
}S_Motor;

typedef struct
{
    uint16_t baseValue;		//基础油门值
    uint16_t finalOut;		//最终油门输出值
}S_Throttle;

void ControlHandle(void);
void ControlInit(void);
#endif

