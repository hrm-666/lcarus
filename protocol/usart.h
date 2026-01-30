#ifndef _usart2_h_
#define _usart2_h_

#include "stm32f10x.h"
#include "stdio.h"

#define USART1_RCC_PORT		RCC_APB2Periph_GPIOA
#define USART1_RCC_NUM		RCC_APB2Periph_USART1		
#define USART1_NUM				USART1

#define USART1_TX_PIN		GPIO_Pin_9
#define USART1_RX_PIN		GPIO_Pin_10


#define USART2_RCC_PORT		RCC_APB2Periph_GPIOA
#define USART2_RCC_NUM		RCC_APB1Periph_USART2		
#define USART2_NUM				USART2

#define USART2_TX_PIN		GPIO_Pin_2
#define USART2_RX_PIN		GPIO_Pin_3

void UartInit(void);
void Usart2DMASendData(uint8_t *sendBuff, u16 len);
void Usart1DMASendData(uint8_t *sendBuff, u16 len);
void Uart1Init(void);
#endif
