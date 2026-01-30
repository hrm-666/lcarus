#ifndef _transmit_packet_h
#define _transmit_packet_h

#include "stm32f10x.h"

void TransmitInit(void);
uint8_t StoreToTransmitQueue(uint8_t *sendData, uint16_t dataLen);

#endif
