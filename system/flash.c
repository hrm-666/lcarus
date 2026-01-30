#include "flash.h"
#include "receive_packet.h"
      					
volatile FLASH_Status flashStatus = FLASH_COMPLETE;		//FLASH操作状态变量

#define FLASH_WAITETIME  	50000          	//FLASH等待超时时间

void FlashReadSpecifyWord(uint32_t flashAddr, uint16_t *readBuf, uint16_t readLen)
{
	for(int i = 0; i < readLen; i++){
		readBuf[i] = *(uint32_t*)flashAddr;		
		flashAddr += 2;//偏移1个字节.	
	}

}

uint8_t FlashWriteSpecifyWord(uint32_t writeAddr, uint16_t *writeBuf, uint16_t arrLen)
{
	FLASH_Status flashStatus = FLASH_COMPLETE;		//FLASH操作状态变量
	
	for(int i = 0; i < arrLen; i++){
		flashStatus = FLASH_ProgramHalfWord(writeAddr, writeBuf[i]);      //写入一个字节
		if(flashStatus != FLASH_COMPLETE){
			return 1;
		}
		writeAddr += 2;
	}
	
	return 0;
}


uint8_t FlashWrite(uint32_t writeAddr, uint16_t *writeBuf, uint16_t writeLen)	
{
	uint32_t secpos = 0;	   //扇区地址
	uint16_t secoff = 0;	   //扇区内偏移地址(16位字计算)
	uint16_t secremain = 0; //扇区内剩余地址(16位字计算)	   
 	uint16_t i = 0;
	uint32_t offaddr = 0;   //去掉0X08000000后的地址
	static uint16_t tempBuf[STM_SECTOR_SIZE / 2] = {0};
	uint8_t needEraseFlag = 0;
	
	if(writeAddr < STM32_FLASH_BASE || (writeAddr >= (STM32_FLASH_BASE + 1024 * STM32_FLASH_SIZE))){		//检查地址是否合法
		return 1;
	}
	
	FLASH_Unlock();		//解锁
	offaddr = writeAddr - STM32_FLASH_BASE;		//实际偏移地址，例：0x8005020 - 0x8000000 = 0x5020
	secpos = offaddr / STM_SECTOR_SIZE;			//扇区地址，例：0x5020 / 0x400（1024） = 0x14 = 20
	secoff = (offaddr % STM_SECTOR_SIZE) / 2;		//在扇区内的偏移，例：0x5020 % 0x400（1024） = 0x20 = 32
	secremain = STM_SECTOR_SIZE / 2 - secoff;		//扇区剩余空间大小，例：1024 - 32 = 992
	
	if(writeLen <= secremain){
		secremain = writeLen;		//不大于该扇区范围
	}
	
	while(1){	
		FlashReadSpecifyWord(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, tempBuf, STM_SECTOR_SIZE / 2);//读出整个扇区的内容
		for(i = 0; i < secremain; i++){		//判断写入的地址是否存在数据，存在则擦除
			if(tempBuf[secoff + i] != 0XFFFF){
				needEraseFlag = 1;
				break;		//需要擦除
			}
		}
		
		if(needEraseFlag == 1){		//需要擦除
			FLASH_ErasePage(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE);	//擦除这个扇区
			FLASH_WaitForLastOperation(FLASH_WAITETIME);		//等待上次操作完成
//			CLEAR_BIT(FLASH->CR, FLASH_CR_PER);		//清除CR寄存器的PER位，此操作应该在FLASH_PageErase()中完成！
//																						//但是HAL库里面并没有做，应该是HAL库bug！
			for(i = 0; i < secremain; i++){		//将要写入的数据转移到tempBuf中
				tempBuf[i + secoff] = writeBuf[i];
			}
			FlashWriteSpecifyWord(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, tempBuf, STM_SECTOR_SIZE / 2);		//整体写入 
			
			needEraseFlag = 0;				
		} else {
			FLASH_WaitForLastOperation(FLASH_WAITETIME);		//等待上次操作完成
			FlashWriteSpecifyWord(writeAddr, writeBuf, secremain);		//不需要擦除则直接写入
		}
		
		if(writeLen == secremain){
			break;		//写入结束了
		} else{		//写入未结束
			secpos++;		//扇区地址增1
			secoff = 0;		//偏移位置为0 	 
			writeBuf += secremain;  	//指针偏移
			writeAddr += secremain * 2;		//写地址偏移   
			writeLen -= secremain;		//长度更新
			
			if(writeLen > (STM_SECTOR_SIZE / 2)){
				secremain = STM_SECTOR_SIZE / 2;//下一个扇区还是写不完
			} else{
				secremain = writeLen;		//下一个扇区可以写完了
			}
		}	 
	}
	
	FLASH_Lock();		//上锁
	
	return 0;
}

/**
* @brief 从flash写入指定字节数据，不影响同页内的其他数据
* @param addr: 要写入的flash地址
* @param writeData: 写入数据的首地址
* @param n: 写入长度，n <= 65535，同时需要调整‘MAX_LENGTH’参数
*/
void FlashWriteByte(uint32_t addr, uint8_t *writeData, uint16_t writeLen)
{
	uint16_t buffLen = 0;
	static uint16_t writeBuf[MAX_LENGTH / 2] = {0};
	int i = 0;
	uint16_t tempLen = 0;
	
	while(writeLen > 0){
		if(writeLen <= MAX_LENGTH){
			buffLen = writeLen / 2 + writeLen % 2;
		} else{
			buffLen = MAX_LENGTH / 2;
		}

		for(i = 0; i < buffLen; i++){
			writeBuf[i] = (uint16_t)writeData[i*2];
			tempLen++;
			if((i * 2 + 1) < writeLen){		//判断数组是否存在溢出
				writeBuf[i] |= writeData[i*2+1] << 8;
				tempLen++;
			}
		}
		FlashWrite(addr, writeBuf, buffLen);

		writeLen -= tempLen;		//更新读取长度，一个buffLen是16字节，所以要*2
		addr += tempLen;

	}
	
}

/**
* @brief 从flash读取指定字节数据，不影响同页内的其他数据
* @param addr: 要读取的flash地址
* @param readData: 存放读出数据的首地址
* @param n: 读取长度，n <= 65535，同时需要调整‘MAX_LENGTH’参数
*/
void FlashReadByte(uint32_t addr, uint8_t *readData, uint16_t readLen)
{
	uint16_t buffLen = 0;
	static uint16_t readBuf[MAX_LENGTH / 2] = {0};
	int i = 0;
	uint16_t tempLen = 0;
	
	while(readLen > 0){
		if(readLen <= MAX_LENGTH){
			buffLen = readLen / 2 + readLen % 2;
		} else{
			buffLen = MAX_LENGTH / 2;
		}

		FlashReadSpecifyWord(addr, readBuf, buffLen);
		
		for(i = 0; i < buffLen; i++){
			readData[i*2] = (uint8_t)(readBuf[i]);
			tempLen++;		//记录读取长度
			if((i * 2 + 1) < readLen){		//判断数组是否存在溢出
				readData[i*2+1] = (uint8_t)(readBuf[i] >> 8);
				tempLen++;
			}
		}

		readLen -= tempLen;		//更新读取长度，一个buffLen是16字节，所以要*2
		addr += tempLen;
	}
}

/**
* @brief 往flash中写入指定长度的数据
* @param writeAddr: flash页地址
* @param writeBuf: 数据数组的首地址
* @param arrLen:数据长度
* @retval 1，成功；0，失败
*/
uint8_t FlashWriteSpecifyData(uint32_t writeAddr,uint32_t *writeBuf, uint8_t arrLen)
{
	uint8_t i;
	
	FLASH_UnlockBank1();                                                                //写之前解除闪存锁
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);           //清除错误标志
  flashStatus = FLASH_ErasePage(writeAddr);                                      //写入前先进行页擦除
	for(i = 0; i < arrLen; i++){
		if(flashStatus == FLASH_COMPLETE){                                                  //判断状态位
			flashStatus = FLASH_ProgramWord(writeAddr, writeBuf[i]);      //写入一个字 
			writeAddr += 4;
		}else {
			return 0;
		}
		
	}
	FLASH_LockBank1();   
	
	return 1;
	
}


//读取指定地址的半字(16位数据)
//faddr:读地址(此地址必须为2的倍数!!)
//返回值:对应数据.
float STM32_FLASH_ReadHalfWord(u32 faddr)
{
	return *(float*)faddr; 
} 


//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//Num:字节数
void STM32_FLASH_Read(u32 ReadAddr, uint32_t *pBuffer,u16 Num)   	
{
	u16 i;
	for( i = 0; i < Num; i++){
		pBuffer[i] = *(uint32_t*)ReadAddr;		
		ReadAddr += 4;//偏移4个字节.	
	}
	
}
