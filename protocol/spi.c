#include "spi.h"

void SpiInit(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(	SPI_BUS_RCC_PORT, ENABLE); 
	RCC_APB2PeriphClockCmd(	SPI_BUS_NUM_RCC | SPI_AFIO_RCC, ENABLE);	
 
	//SCK 、MISO、MOSI
	GPIO_InitStructure.GPIO_Pin = SCK_PIN | MISO_PIN | MOSI_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  	 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_BUS_PORT, &GPIO_InitStructure);	
	

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  	
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;	
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;		
	SPI_InitStructure.SPI_CRCPolynomial = 7;		
	SPI_Init(SPI1, &SPI_InitStructure);  	
	
	SPI_Cmd(SPI1, ENABLE); 		 
}   


//SPI硬件读写单字节数据
u8 Spi_RW_Byte(u8 TxData)
{			 	
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);			  
    SPI_I2S_SendData(SPI1, TxData); 

    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    
    return SPI_I2S_ReceiveData(SPI1); 				    
}


