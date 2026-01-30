#include "transmit_packet.h"
#include "receive_packet.h"
#include "my_lib.h"
#include <stdio.h>
#include <string.h>
#include "log_lib.h"
#include <rtthread.h>
#include "task_manage.h"
#include "queue_lib.h"
#include "plane.h"
#include "crc_lib.h"

#define TRANSMIT_BUFF_LENGHT	256		//无线发送缓冲区长度

uint8_t transmitBuff[TRANSMIT_BUFF_LENGHT] = {0};		//无线发送缓冲区
static S_Queue transmitQueue = {0};		//无线发送队列

static rt_mutex_t transmit_mutex = RT_NULL;

void TransmitInit(void)
{
	QueueInit(&transmitQueue, transmitBuff, TRANSMIT_BUFF_LENGHT);
	
	/* 创建互斥锁 */
	transmit_mutex = rt_mutex_create("transmit_mutex", RT_IPC_FLAG_PRIO);
	if (transmit_mutex == RT_NULL)
	{
		rt_kprintf("create dynamic transmit_mutex failed.\n");
		return;
	}
}

//向遥控器发送飞机数据
void TransmitHandle(void)
{
	uint8_t sendBuff[32] = {0};
	uint8_t data = 0;
	int ret = 0;
	S_Plane plane = {LOCK};
	uint8_t sendDataLen = 0;
	
	GetPlaneInfo(&plane);
	if(plane.pair != PAIR_DONE){
		return;
	}
	
	rt_mutex_take(transmit_mutex, RT_WAITING_FOREVER);
	for(int i = 0; i < 32; i++){
		ret = QueueLength(&transmitQueue);
		if(ret > 0){
//			LogInfo("QueueLength: %d", ret);
			DeQueue(&transmitQueue, &data);		//从发送队列里取出数据
//			LogInfo("data: %x", data);
			sendBuff[sendDataLen++] = data;		//转移到发送数组中
		} else {
			break;
		}
	}
	rt_mutex_release(transmit_mutex);	
	
	if(sendDataLen != 0){
//		LogInfo("sendDataLen: %d", sendDataLen);
		ret = SPI_Write_Buf(W_ACK_PLOAD, sendBuff, sendDataLen);		//发送数据
	}
	else {
		uint8_t heartPacket[5] = {0xA1, 0xB1, 0xC2, 0xD1, 0xE1};
		ret = SPI_Write_Buf(W_ACK_PLOAD, heartPacket, 5);		//发送心跳包
	}	
	
}

uint8_t StoreToTransmitQueue(uint8_t *sendData, uint16_t dataLen)
{
	uint8_t ret = 0;
	uint8_t sendArr[32] = {0};
	uint8_t arrLen = 0;
	
	if(dataLen > 27){		//判断包是否过长
		return 0;
	}
	
	//封包
	sendArr[arrLen++] = 0xAA;
	sendArr[arrLen++] = dataLen >> 8;
	sendArr[arrLen++] = dataLen;
	
	for(int i = 0; i < dataLen; i++){
		sendArr[arrLen++] = sendData[i];
	}
	
	sendArr[arrLen++] = crc8_maxim(sendData, dataLen);
	sendArr[arrLen++] = 0xAD;
	
	//存入发送队列
	rt_mutex_take(transmit_mutex, RT_WAITING_FOREVER);
	
	for(int i = 0; i < arrLen; i++){
		ret = EnQueue(&transmitQueue, sendArr[i]);
		if(ret == 1){
			LogError("transmit queue is full");
			return 0;
		}
	}
	
	rt_mutex_release(transmit_mutex);
	
	return 1;
}

