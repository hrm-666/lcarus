#ifndef _pair_freq_h_
#define _pair_freq_h_

#include "stm32f10x.h"

typedef enum{
	STEP1,
	STEP2,
	STEP3,
	STEP4,
	STEP5,
}PairStep;

typedef enum{
	DETECT_NOT = 0,
	DETECT_NORMAL,
}DetectStatus;

typedef struct
{
	PairStep step;		//对频步骤
	uint8_t addr[5];		//对频地址
	uint8_t freq_channel;		//对频通道
}PairPacket;

void PairHandle(void);
void GetPairPacketInfo(PairPacket *exPairPacket);
void Pair_Main(void);
uint8_t GetTransmiteModuleInitStatus(void);
#endif

