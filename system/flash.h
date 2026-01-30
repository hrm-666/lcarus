#ifndef _flash_h_
#define _flash_h_

#include "stm32f10x.h"

//需根据芯片型号调整以下参数
#define STM32_FLASH_BASE 0x08000000
#define STM32_FLASH_SIZE 64
#define STM_SECTOR_SIZE 1024		//每一页的大小

 #define MAX_LENGTH	1024		//设定flash读写接口一次能操作的最大字节数

float STM32_FLASH_ReadHalfWord(u32 faddr);														
uint8_t FlashWriteSpecifyData(uint32_t writeAddr,uint32_t *writeBuf,uint8_t arrLen);
void STM32_FLASH_Read(u32 ReadAddr, uint32_t *pBuffer,u16 Num)  ;																	
uint8_t FlashWrite(uint32_t writeAddr, uint16_t *writeBuf, uint16_t writeLen);
void FlashWriteByte(uint32_t addr, uint8_t *writeData, uint16_t writeLen);
void FlashReadByte(uint32_t addr, uint8_t *readData, uint16_t readLen);
uint8_t FlashWriteSpecifyData(uint32_t writeAddr,uint32_t *writeBuf, uint8_t arrLen);
#endif


