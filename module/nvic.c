#include "nvic.h"


void NvicConfig(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;    

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);             //优先级组别2
    
    //串口数据接收中断
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;		//抢占优先级，数值越小，优先级越高
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//响应优先级，数值越小，优先级越高
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
