#ifndef _QUEUE_LIB_H_
#define _QUEUE_LIB_H_

#include "stm32f10x.h"

//==============队列数据结构==============↓
typedef struct{
	uint8_t *pArr;
	uint16_t size;
	uint16_t front;
	uint16_t rear;
}S_Queue;

void QueueInit(S_Queue *queue, uint8_t *arr, uint16_t size);
uint16_t QueueLength(S_Queue *queue);
uint8_t EnQueue(S_Queue *queue, uint8_t data);
uint8_t DeQueue(S_Queue *queue, uint8_t *data);
//==============队列数据结构==============↑

#endif
