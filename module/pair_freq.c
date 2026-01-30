#include "pair_freq.h"
#include "si24r1.h"
#include "spi.h"
#include "receive_packet.h"
#include "transmit_packet.h"
#include "my_lib.h"
#include "log_lib.h"
#include "systick.h"
#include <stdio.h>
#include <string.h>
#include "task_manage.h"
#include "plane.h"
#include "crc_lib.h"

static rt_thread_t pair_thread = RT_NULL;

static uint8_t transmiteModuleInitStatus = 0;		//通信模块初始化状态

void PairHandle(void);
int SendData(uint8_t *sendData, uint16_t dataLen);

void pair_thread_entry(void *param)
{
	
	while(1){
		S_Plane plane;
		
		GetPlaneInfo(&plane);
		
		if(plane.pair == PAIR_DONE){
			return;		//对频完成终止该线程
		} else {
			PairHandle();
		}

		rt_thread_mdelay(25);
	}
}

void Pair_Main(void)
{
	uint8_t ret = 0;
	
	ret = Si24r1Init();
	if(ret == 1){
		transmiteModuleInitStatus = 1;
		return;
	}
	
	//创建线程
	pair_thread = rt_thread_create("pair_thread", pair_thread_entry, RT_NULL, 512, PAIR_PRIORITY, 5);
	
	//启动线程
	if (pair_thread != RT_NULL){
		rt_thread_startup(pair_thread);
	} else {
		LogError("pair_thread start fail");
		return;
	}
}

//对频
void PairHandle(void)
{
	uint8_t receiveBuf[20] = {0};
	int ret = -1;
	uint8_t receiveData[20] = {0};
	uint8_t sendData[20] = {0};
	uint8_t sendDetectData = 0xB2;
	static PairPacket sendPairPacket = {STEP1,{0x1F,0x2E,0x3D,0x4C,0x5B},5};
	static PairStep lastStep = STEP1;
	static PairPacket receivePairPacket = {STEP1,{0x1F,0x2E,0x3D,0x4C,0x5B},5};
	S_Plane plane;
	static uint8_t sendCount = 0;
	static uint8_t pairCount = 0;
	uint8_t receiveBufLength = 0;
	
	GetPlaneInfo(&plane);
//	LogInfo("plane.pair: %d", plane.pair);
	if(plane.pair != PAIR_DONE || plane.pair == PAIR_FAIL){
		receiveBufLength = Si24r1_RxPacket((u8 *)&receiveBuf);
//		LogInfo("receiveBufLength: %d", receiveBufLength);
		if(receiveBufLength > 0){
			for(int i = 0; i < 20; i++){
				ret = AnalysisData(receiveBuf[i], receiveData);
//				LogInfo("receiveData[0]: %d", receiveData[0]);
				if(ret == 0){
					break;
				}
			}
		}
	}

	
	if(ret == 0){
		if(receiveData[0] == 0xB1){
			sendData[0] = sendDetectData;
			SendData(sendData, 1);
		}
		else if(receiveData[0] == STEP1){
			LogInfo("pair step2");
			plane.pair = PAIR_START;
			SetPlaneInfo(&plane, SET_PLANE_PAIR);
			sendPairPacket.step = STEP2;
			sendData[0] = sendPairPacket.step;
			
			SendData(sendData, 1);
			lastStep = STEP2;
		}
		else if(receiveData[0] == STEP3 && lastStep == STEP2){
			LogInfo("pair step3");
			
			receivePairPacket.addr[0] = receiveData[1];
			receivePairPacket.addr[1] = receiveData[2];
			receivePairPacket.addr[2] = receiveData[3];
			receivePairPacket.addr[3] = receiveData[4];
			receivePairPacket.addr[4] = receiveData[5];
			receivePairPacket.freq_channel = receiveData[6];
			
			NRF_CE_L;
			SPI_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)receivePairPacket.addr,RX_ADR_WIDTH);		//写入新的通信地址
			SPI_Write_Reg(NRF_WRITE_REG+RF_CH,receivePairPacket.freq_channel);		//写入新的通信频点
			NRF_CE_H; 
			
			lastStep = STEP3;
		}
		else if(receiveData[0] == STEP4 && lastStep == STEP3){
			if(sendCount < 2){		//负载ack下次才发送，所以需装载两次
				LogInfo("pair step5");
				
				sendPairPacket.step = STEP5;
				sendData[0] = sendPairPacket.step;
				SendData(sendData, 1);
				
				if(sendCount == 1){
					LogInfo("pair done");
					plane.pair = PAIR_DONE;
					SetPlaneInfo(&plane, SET_PLANE_PAIR);
				}
				
				sendCount++;
			}
		}
	}
	
	if(plane.pair == PAIR_START){
		pairCount++;
		if(pairCount > 250){
			pairCount = 0;
			
			LogInfo("pair fail");
			
			plane.pair = PAIR_FAIL;
			SetPlaneInfo(&plane, SET_PLANE_PAIR);
			
			//对频失败，地址和通道复位
			receivePairPacket.addr[0] = 0x1F;
			receivePairPacket.addr[1] = 0x2E;
			receivePairPacket.addr[2] = 0x3D;
			receivePairPacket.addr[3] = 0x4C;
			receivePairPacket.addr[4] = 0x5B;
			receivePairPacket.freq_channel = 5;
			
			NRF_CE_L;
			SPI_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)receivePairPacket.addr,RX_ADR_WIDTH);		//写入新的通信地址
			SPI_Write_Reg(NRF_WRITE_REG+RF_CH,receivePairPacket.freq_channel);		//写入新的通信频点
			NRF_CE_H; 
		}
	}
			
}

int SendData(uint8_t *sendData, uint16_t dataLen)
{
	uint8_t sendArr[32] = {0};
	uint8_t arrLen = 0;
	
	if(dataLen > 28){		//检查发送包长度
		LogError("lenth so much");
		return -1;
	}		
	
	sendArr[arrLen++] = 0xAA;
	sendArr[arrLen++] = dataLen >> 8;
	sendArr[arrLen++] = dataLen;
	
	for(int i = 0; i < dataLen; i++){
		sendArr[arrLen++] = sendData[i];
	}
	
	sendArr[arrLen++] = crc8_maxim(sendData, dataLen);
	sendArr[arrLen++] = 0xAD;
		
	SPI_Write_Buf(W_ACK_PLOAD, sendArr, arrLen);
	
	return 0;
}

uint8_t GetTransmiteModuleInitStatus(void)
{
	return transmiteModuleInitStatus;
}
