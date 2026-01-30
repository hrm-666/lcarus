#ifndef _spi_h_
#define _spi_h_

#include "stm32f10x.h"

//SCK, MOSI, MISO
#define SPI_BUS_RCC_PORT	RCC_APB2Periph_GPIOA	//spi时钟端口
#define SPI_BUS_NUM_RCC		RCC_APB2Periph_SPI1		//spi号
#define SPI_AFIO_RCC			RCC_APB2Periph_AFIO		//spi复用时钟线
#define SPI_BUS_PORT			GPIOA

#define SCK_PIN		GPIO_Pin_5
#define MISO_PIN	GPIO_Pin_6
#define MOSI_PIN	GPIO_Pin_7


void SpiInit(void);			 

u8 Spi_RW_Byte(u8 TxData);

#endif

